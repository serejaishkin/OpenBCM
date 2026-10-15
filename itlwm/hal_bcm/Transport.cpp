// Transport.cpp — transport implementations.
//
// 43A0 (BCM4360), CORRECTED after RE of AirPortBrcmNIC:
//   * The chip's control transport IS BCDC/olmsg (Apple's `_wlc_ol_*` +
//     `_bcm_olmsg_*`): one 64 KB coherent DMA buffer split into two 0x7800
//     rings, doorbell at BAR1+0x48, shared-info magics 0xa5a5a5a5/0x5a5a5a5a.
//   * The DATA path is host-driven D11 SoftMAC `dma64` TX/RX rings (see
//     `_wlc_bmac_recv`->`_wlc_recv`, `_wlc_bmac_txstatus`), NOT olmsg.
//
// 43BA/43A3 keep the msgbuf path (vanilla Apple Skywalk stack, BCMC-style).

#include "Transport.hpp"
#include <IOKit/IODeviceTreeSupport.h>
#include <libkern/OSDebug.h>

#define LOG_PREFIX "bcmp-transport"

// olmsg register offsets within BAR1 (see DLIMAGE_FINDINGS.md).
static constexpr UInt32 kOlRegDoorbellStatus = 0x48; // H2D |=1; D2H bit8 W1C
static constexpr UInt32 kOlRegIntMask        = 0x4c; // bit8 = D2H olmsg IRQ
static constexpr UInt32 kOlIntBit            = 0x100;

// ===========================================================================
// BcdcOlTransport — BCM4360 (43A0) control plane
// ===========================================================================

bool BcdcOlTransport::init(IOPCIDevice *device, UInt32 chipNumber) {
    mDevice = device;
    mChipNumber = chipNumber;
    return mDevice != nullptr;
}

bool BcdcOlTransport::allocBuffer() {
    // One 64 KB physically-contiguous buffer, matching `_wlc_ol_attach`:
    // `_osl_dma_alloc_consistent_force32(ol+0x10, 2, 0x10000, &bus0, &bus1, &h)`.
    // 32-bit-visible DMA is required by the firmware; use a physically-masked
    // descriptor so the ring base is addressable by the ARM.
    mDmaBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIOMemoryPhysicallyContiguous, ol_msg::kBufferSize, 0xffffffffULL);
    if (!mDmaBuf || mDmaBuf->prepare() != kIOReturnSuccess) {
        IOLog("%s: olmsg DMA buffer alloc failed\n", LOG_PREFIX);
        return false;
    }
    mBufVA = (UInt8 *)mDmaBuf->getBytesNoCopy();
    mDmaPhys = mDmaBuf->getPhysicalAddress();

    // The firmware reads absolute 32-bit bus addresses of the two rings; wire
    // them through the ol+0x34/+0x40/+0x48 pattern via the shared-info block
    // once the bring-up path (TODO(RE)) publishes the buffer.
    if (ol_msg::create(mBufVA, ol_msg::kBufferSize) != ol_msg::kOk ||
        ol_msg::init(&mOlMsg, mBufVA, ol_msg::kBufferSize, 0, 1) != ol_msg::kOk) {
        IOLog("%s: olmsg create/init failed\n", LOG_PREFIX);
        return false;
    }
    IOLog("%s: olmsg buffer VA %p phys 0x%llx\n", LOG_PREFIX, mBufVA,
          (unsigned long long)mDmaPhys);
    return true;
}

bool BcdcOlTransport::start() {
    if (!allocBuffer()) {
        return false;
    }
    // BAR1 holds the doorbell / interrupt registers.
    mBar1Map = mDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);
    if (!mBar1Map) {
        IOLog("%s: cannot map BAR1\n", LOG_PREFIX);
        return false;
    }
    mBar1 = (volatile UInt8 *)mBar1Map->getVirtualAddress();
    setInterruptMask(true);
    mRunning = true;
    return true;
}

void BcdcOlTransport::stop() {
    mRunning = false;
    if (mDmaBuf) {
        mDmaBuf->complete();
        OSSafeReleaseNULL(mDmaBuf);
    }
    OSSafeReleaseNULL(mBar1Map);
    mBar1 = nullptr;
    mBufVA = nullptr;
}

void BcdcOlTransport::notifyFirmware() {
    if (!mBar1) {
        return;
    }
    volatile UInt32 *reg = (volatile UInt32 *)((uintptr_t)mBar1 + kOlRegDoorbellStatus);
    UInt32 v = *reg;
    v |= 1;
    *reg = v;
    (void)*reg; // read-back flush
}

bool BcdcOlTransport::sendControl(const void *msg, UInt32 len) {
    if (!mRunning || !msg || len == 0 || len > 0xffff) {
        return false;
    }
    if (ol_msg::writeMsg(&mOlMsg, (UInt8 *)msg, (UInt16)len) != ol_msg::kOk) {
        IOLog("%s: olmsg TX ring full\n", LOG_PREFIX);
        return false;
    }
    notifyFirmware();
    return true;
}

void BcdcOlTransport::drainRx(void (*sink)(const OlControlMessage *, void *), void *ctx) {
    if (!mRunning) {
        return;
    }
    UInt8 buf[0x800];
    for (;;) {
        UInt16 n = ol_msg::readMsg(&mOlMsg, buf, sizeof(buf));
        if (n == 0) {
            break;
        }
        if (sink) {
            sink((const OlControlMessage *)buf, ctx);
        }
    }
}

bool BcdcOlTransport::checkAndClearInterrupt() {
    if (!mRunning || !mBar1) {
        return false;
    }
    volatile UInt32 *reg = (volatile UInt32 *)((uintptr_t)mBar1 + kOlRegDoorbellStatus);
    UInt32 v = *reg;
    if (!(v & kOlIntBit)) {
        return false;
    }
    *reg = kOlIntBit;          // write-1-to-clear bit 8
    (void)*reg;
    setInterruptMask(true);
    return true;
}

void BcdcOlTransport::setInterruptMask(bool enable) {
    if (!mBar1) {
        return;
    }
    volatile UInt32 *reg = (volatile UInt32 *)((uintptr_t)mBar1 + kOlRegIntMask);
    UInt32 v = *reg;
    if (enable) {
        v |= kOlIntBit;
    } else {
        v &= ~kOlIntBit;
    }
    *reg = v;
    (void)*reg;
}

// ===========================================================================
// MsgbufTransport — BCM43602/BCM4350 (stock BCMC ring surface, unchanged)
// ===========================================================================

bool MsgbufTransport::init(IOPCIDevice *device) {
    mDevice = device;
    return mDevice != nullptr;
}

bool MsgbufTransport::start() {
    PCIeRingInfo info {};
    info.h2dRingSize = 256;
    info.d2hRingSize = 256;
    info.maxTxAggregations = 32;
    info.maxRxAggregations = 32;
    if (!createSubmissionRing(info) || !createCompletionRing(info)) {
        return false;
    }
    return true;
}

void MsgbufTransport::stop() {
}

bool MsgbufTransport::createSubmissionRing(const PCIeRingInfo &info) {
    // TODO(RE): AppleBCMWLANPCIeSubmissionRing layout from reversed binary.
    return true;
}

bool MsgbufTransport::createCompletionRing(const PCIeRingInfo &info) {
    // TODO(RE): AppleBCMWLANPCIeCompletionRing layout from reversed binary.
    return true;
}
