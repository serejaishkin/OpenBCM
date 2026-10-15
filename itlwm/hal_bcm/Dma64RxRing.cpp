// Dma64RxRing.cpp — host-driven D11 dma64 RX ring with real IOKit DMA.

#include "Dma64RxRing.hpp"
#include <IOKit/IOKitKeys.h>
#include <libkern/OSDebug.h>

#define LOG_PREFIX "bcm-dma64-rx"

// D11 RX channel 0 register window = d11core + 0x220
static constexpr uint32_t kD11RxCh0Base = 0x220;

// Per-channel register offsets (within the channel's 0x40 window).
static constexpr uint32_t kRegCtl   = 0x00;
static constexpr uint32_t kRegProd  = 0x04;
static constexpr uint32_t kRegBaseLo = 0x08;
static constexpr uint32_t kRegBaseHi = 0x0c;
static constexpr uint32_t kRegDone  = 0x10;

static constexpr uint32_t kCtlDmaEn = 0x00000001u;
static constexpr uint32_t kCtlReset = 0x00000002u;

static constexpr uint32_t kRxCtlOwn = 0x80000000u;
static constexpr uint32_t kRxCtlWrap = 0x10000000u;

Dma64RxRing::
Dma64RxRing(ChipConfigurator *cfg, uint16_t channel, uint16_t count, uint32_t bufSize)
    : mCfg(cfg), mChannel(channel), mCount(count), mBufSize(bufSize) {
}

Dma64RxRing::
~Dma64RxRing() {
    stop();
}

bool Dma64RxRing::
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

    // Build D11::RxRing callbacks backed by real DMA + BAR0 registers.
    uint32_t rxChRegBase = kD11RxCh0Base; // RX chan 0 window offset from D11 core
    auto regRead = [this](uint32_t off) -> uint32_t {
        return mCfg->readD11Reg(rxChRegBase + off);
    };
    auto regWrite = [this](uint32_t off, uint32_t val) {
        mCfg->writeD11Reg(rxChRegBase + off, val);
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

    mRing = std::make_unique<D11::RxRing>(mCount, regRead, regWrite,
                                          descWrite, descReadCtl, descReadLen, descReadAddr);
    mRing->setTableBusAddr(mTableBusAddr);

    // Reset then init the RX channel.
    if (!mRing->reset()) {
        IOLog("%s: RX reset timeout\n", LOG_PREFIX);
        return false;
    }
    if (!mRing->init(mTableBusAddr, 0)) {
        IOLog("%s: RX init failed\n", LOG_PREFIX);
        return false;
    }

    // Post all RX buffers.
    postAllBuffers();

    mRunning = true;
    IOLog("%s: RX ring initialized: %u slots, %u byte buffers, table bus 0x%llx\n",
          LOG_PREFIX, mCount, mBufSize, (unsigned long long)mTableBusAddr);
    return true;
}

void Dma64RxRing::
stop() {
    if (!mRunning) return;
    mRunning = false;
    if (mRing) {
        // Disable DMA
        uint32_t ctl = mRing ? 0 : 0;
        // Best-effort: clear DMA_EN
        mCfg->writeD11Reg(kD11RxCh0Base + kRegCtl, 0);
    }
    if (mDmaBuf) {
        mDmaBuf->release();
        mDmaBuf = nullptr;
        mDmaVa = nullptr;
    }
}

bool Dma64RxRing::
allocDma() {
    // Layout: [descriptor table: count*16] [RX buffers: count*bufSize]
    // Round table size up to 16-byte alignment, buffer pool to page alignment.
    uint32_t tableSize = mCount * 16;
    uint32_t bufPoolSize = mCount * mBufSize;
    // Align buffer pool start to 4096 for safe DMA.
    uint32_t bufPoolStart = (tableSize + 4095) & ~4095;
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
    mBufOffset = bufPoolStart;
    mTableBusAddr = (uint64_t)mDmaPhys + mTableOffset;

    IOLog("%s: DMA buf phys 0x%llx va 0x%llx size %u table@0x%x bufs@0x%x\n",
          LOG_PREFIX, (unsigned long long)mDmaPhys,
          (unsigned long long)(uintptr_t)mDmaVa, totalSize,
          mTableOffset, mBufOffset);
    return true;
}

void Dma64RxRing::
postAllBuffers() {
    if (!mRing || !mDmaVa) return;
    for (uint16_t i = 0; i < mCount; i++) {
        uint64_t bufBus = mTableBusAddr + mBufOffset + (uint64_t)i * mBufSize;
        if (!mRing->postBuffer(bufBus, mBufSize)) {
            IOLog("%s: postBuffer failed at slot %u\n", LOG_PREFIX, i);
            break;
        }
    }
}

uint32_t Dma64RxRing::
poll(std::function<void(mbuf_t, int8_t, uint8_t)> cb) {
    if (!mRunning || !mRing || !cb) return 0;
    return mRing->pollRx([this, &cb](const D11::RxFrame &frame) {
        // Build an mbuf wrapping the received DMA buffer data.
        // Find which slot this address corresponds to.
        uint64_t slotBase = mTableBusAddr + mBufOffset;
        uint32_t slotIdx = (uint32_t)((frame.addr - slotBase) / mBufSize);
        if (slotIdx >= mCount) {
            IOLog("%s: RX frame addr 0x%llx outside buffer pool\n",
                  LOG_PREFIX, (unsigned long long)frame.addr);
            return;
        }
        volatile UInt8 *frameVa = mDmaVa + mBufOffset + slotIdx * mBufSize;

        mbuf_t m = mbuf_alloc(MBUF_WAITOK, MBUF_TYPE_DATA);
        if (!m) {
            IOLog("%s: mbuf_alloc failed\n", LOG_PREFIX);
            return;
        }
        mbuf_setdata(m, (void *)frameVa, frame.len);
        mbuf_setlen(m, frame.len);
        mbuf_pkthdr_setlen(m, frame.len);

        int8_t rssi = readRssi();
        uint8_t chan = 0; // TODO: read current channel from PHY
        cb(m, rssi, chan);

        // Re-post this buffer for next reception.
        mRing->postBuffer(frame.addr, mBufSize);
    });
}

int8_t Dma64RxRing::
readRssi() {
    // TODO(RE): decode RSSI from D11 PHY/rxh registers.
    return -50;
}
