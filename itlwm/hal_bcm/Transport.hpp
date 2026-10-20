// Transport.hpp — host-side transports for legacy Broadcom chips.
//
// 43A0 (BCM4360), CORRECTED after RE of AirPortBrcmNIC (see FORK_PLAN.md):
//   * control plane  = BCDC / olmsg (two 0x7800 rings in one 64 KB coherent DMA
//     buffer + doorbell at BAR1+0x48), mirroring `_wlc_ol_*` / `_bcm_olmsg_*`.
//     There is NO "C-4360__sap AppleCore" ring protocol.
//   * data plane     = host-driven D11 SoftMAC `dma64` TX/RX rings (16-byte
//     descriptors), NOT the olmsg channel; the ARM dlimage is a PM/scan/ARP-ND/
//     WOWL coprocessor.
// 43BA/43A3 (BCM43602/4350) use the SAME BcdcOlTransport: RE of AirPortBrcmNIC
// 1400.1.1 shows one `_wlc_ol_up` loader for 43a0/43a3/43ba, differing only in
// the embedded firmware blob and the TCM base (0x180000 for 0xaa52/0x4350).
// The MsgbufTransport below is retained but unused.

#ifndef Transport_hpp
#define Transport_hpp

#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include "BcmChip.hpp"
#include "OlMsg.hpp"

// One control message on the BCDC/olmsg channel.
//   {u32 id, u32 seq (stamped by the ring), u32 payloadLen, payload[...]}
struct OlControlMessage {
    UInt32  id;
    UInt32  seq;
    UInt32  payloadLen;
    UInt8   payload[];
};

// BCM4360 (43A0) control transport: a single 64 KB coherent DMA buffer split
// into two olmsg rings, with the H2D doorbell / D2H status at BAR1+0x48 and the
// interrupt mask at BAR1+0x4c (decoded from AirPortBrcmNIC `_bcm_olmsg_*`,
// `_wlc_ol_msg_send`, `_wlc_ol_intstatus`, `_wlc_ol_enable_intrs`).
class BcdcOlTransport {
public:
    bool init(IOPCIDevice *device, UInt32 chipNumber);
    bool start();
    void stop();
    bool isRunning() const { return mRunning; }

    // Host -> device: copy the message into the TX ring and ring the doorbell.
    bool sendControl(const void *msg, UInt32 len);

    // Device -> host: drain queued messages; `sink` may be null (just drain).
    void drainRx(void (*sink)(const OlControlMessage *, void *), void *ctx);

    // Interrupt handling (BAR1+0x48 / +0x4c).
    bool checkAndClearInterrupt();
    void setInterruptMask(bool enable);

    // 32-bit bus address of the 64 KB ring buffer (published to the firmware
    // via the shared-info block; `ol+0x34` in `_wlc_ol_up`).
    IOPhysicalAddress dmaPhysicalAddress() const { return mDmaPhys; }

private:
    bool allocBuffer();
    void notifyFirmware();       // RMW |=1 at BAR1+0x48

    IOPCIDevice *mDevice {nullptr};
    UInt32 mChipNumber {0};
    IOMemoryMap *mBar1Map {nullptr};
    volatile UInt8 *mBar1 {nullptr};
    IOBufferMemoryDescriptor *mDmaBuf {nullptr};   // 64 KB, physically contiguous
    IOPhysicalAddress mDmaPhys {0};
    UInt8 *mBufVA {nullptr};
    ol_msg::Info mOlMsg {};
    bool mRunning {false};
};

// msgbuf variant (BCM43602/BCM4350) — unchanged stock BCMC ring surface.
class MsgbufTransport {
public:
    bool init(IOPCIDevice *device);
    bool start();
    void stop();
private:
    bool createSubmissionRing(const PCIeRingInfo &info);
    bool createCompletionRing(const PCIeRingInfo &info);
    IOPCIDevice *mDevice {nullptr};
    void *mSubmissionRing {nullptr};
    void *mCompletionRing {nullptr};
};

#endif /* Transport_hpp */
