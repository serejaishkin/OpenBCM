// Dma64TxRing.cpp — host-driven D11 dma64 TX ring with real IOKit DMA.

#include "Dma64TxRing.hpp"
#include <IOKit/IOKitKeys.h>
#include <libkern/OSDebug.h>

#define LOG_PREFIX "bcm-dma64-tx"

// D11 TX channel 0 register window = d11core + 0x200
static constexpr uint32_t kD11TxCh0Base = 0x200;

static constexpr uint32_t kRegCtl   = 0x00;
static constexpr uint32_t kRegProd  = 0x04;
static constexpr uint32_t kRegBaseLo = 0x08;
static constexpr uint32_t kRegBaseHi = 0x0c;
static constexpr uint32_t kRegDone  = 0x10;

static constexpr uint32_t kCtlDmaEn = 0x00000001u;

static constexpr uint32_t kTxCtlFirst = 0x80000000u;
static constexpr uint32_t kTxCtlLast  = 0x60000000u;
static constexpr uint32_t kTxCtlSingle = kTxCtlFirst | kTxCtlLast;
static constexpr uint32_t kTxCtlWrap  = 0x10000000u;

Dma64TxRing::
Dma64TxRing(ChipConfigurator *cfg, uint16_t channel, uint16_t count)
    : mCfg(cfg), mChannel(channel), mCount(count) {
}

Dma64TxRing::
~Dma64TxRing() {
    stop();
}

bool Dma64TxRing::
init() {
    if (!mCfg || !mCfg->bar0Address()) {
        IOLog("%s: no BAR0 mapping\n", LOG_PREFIX);
        return false;
    }
    if (!mCfg->getD11CoreBase()) {
        IOLog("%s: D11 core base unknown\n", LOG_PREFIX);
        return false;
    }

    if (!allocDma()) {
        IOLog("%s: DMA allocation failed\n", LOG_PREFIX);
        return false;
    }

    auto regRead = [this](uint32_t off) -> uint32_t {
        return mCfg->readD11Reg(kD11TxCh0Base + off);
    };
    auto regWrite = [this](uint32_t off, uint32_t val) {
        mCfg->writeD11Reg(kD11TxCh0Base + off, val);
    };
    auto descWrite = [this](uint32_t idx, uint32_t ctl, uint32_t len, uint64_t addr) {
        volatile UInt32 *d = (volatile UInt32 *)(mDmaVa + mTableOffset + idx * 16);
        d[0] = ctl;
        d[1] = len;
        d[2] = (uint32_t)(addr & 0xffffffffu);
        d[3] = (uint32_t)(addr >> 32);
    };
    auto descReadCtl = [this](uint32_t idx) -> uint32_t {
        return OSReadLittleInt32((void *)(mDmaVa + mTableOffset + idx * 16), 0);
    };
    auto descReadLen = [this](uint32_t idx) -> uint32_t {
        return OSReadLittleInt32((void *)(mDmaVa + mTableOffset + idx * 16), 4);
    };
    auto descReadAddr = [this](uint32_t idx) -> uint64_t {
        uint32_t lo = OSReadLittleInt32((void *)(mDmaVa + mTableOffset + idx * 16), 8);
        uint32_t hi = OSReadLittleInt32((void *)(mDmaVa + mTableOffset + idx * 16), 12);
        return ((uint64_t)hi << 32) | lo;
    };

    mRing = std::make_unique<D11::TxRing>(mCount, regRead, regWrite,
                                          descWrite, descReadCtl, descReadLen, descReadAddr);
    mRing->setTableBusAddr(mTableBusAddr);

    if (!mRing->reset()) {
        IOLog("%s: TX reset timeout\n", LOG_PREFIX);
        return false;
    }
    if (!mRing->init(mTableBusAddr, 0)) {
        IOLog("%s: TX init failed\n", LOG_PREFIX);
        return false;
    }

    mRunning = true;
    IOLog("%s: TX ring initialized: %u slots, table bus 0x%llx\n",
          LOG_PREFIX, mCount, (unsigned long long)mTableBusAddr);
    return true;
}

void Dma64TxRing::
stop() {
    if (!mRunning) return;
    mRunning = false;
    if (mRing) {
        mCfg->writeD11Reg(kD11TxCh0Base + kRegCtl, 0);
    }
    if (mDmaBuf) {
        mDmaBuf->release();
        mDmaBuf = nullptr;
        mDmaVa = nullptr;
    }
}

bool Dma64TxRing::
allocDma() {
    // Layout: [descriptor table: count*16] [frame buffers: count*mFrameSize]
    uint32_t tableSize = mCount * 16;
    uint32_t bufPoolStart = (tableSize + 4095) & ~4095;
    uint32_t bufPoolSize = mCount * mFrameSize;
    uint32_t totalSize = bufPoolStart + bufPoolSize;

    mDmaBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIOMemoryPhysicallyContiguous, totalSize, 0xffffffffULL);
    if (!mDmaBuf) {
        IOLog("%s: IOBufferMemoryDescriptor alloc failed (size %u)\n",
              LOG_PREFIX, totalSize);
        return false;
    }

    mDmaPhys = mDmaBuf->getPhysicalAddress();
    mDmaVa = (volatile UInt8 *)mDmaBuf->getBytesNoCopy();
    if (!mDmaVa) {
        IOLog("%s: DMA buffer no VA\n", LOG_PREFIX);
        mDmaBuf->release();
        mDmaBuf = nullptr;
        return false;
    }

    mTableOffset = 0;
    mFramePoolOffset = bufPoolStart;
    mTableBusAddr = (uint64_t)mDmaPhys + mTableOffset;

    IOLog("%s: DMA buf phys 0x%llx va 0x%llx size %u table@0x%x frames@0x%x\n",
          LOG_PREFIX, (unsigned long long)mDmaPhys,
          (unsigned long long)(uintptr_t)mDmaVa, totalSize,
          mTableOffset, mFramePoolOffset);
    return true;
}

bool Dma64TxRing::
enqueue(const void *data, uint32_t len) {
    if (!mRunning || !mRing) return false;
    if (len == 0 || len > mFrameSize) {
        IOLog("%s: enqueue: invalid frame len %u\n", LOG_PREFIX, len);
        return false;
    }
    uint64_t frameBus = mTableBusAddr + mFramePoolOffset + (uint64_t)mProd * mFrameSize;

    // Copy frame data into the TX buffer slot.
    void *frameVa = (void *)(mDmaVa + mFramePoolOffset + mProd * mFrameSize);
    memcpy(frameVa, data, len);

    bool ok = mRing->enqueue(frameBus, len);
    return ok;
}

uint32_t Dma64TxRing::
pollTx() {
    if (!mRunning || !mRing) return 0;
    uint32_t reclaimed = 0;
    mRing->reclaimTx([&reclaimed](const D11::TxDone &done) {
        reclaimed++;
    });
    return reclaimed;
}
