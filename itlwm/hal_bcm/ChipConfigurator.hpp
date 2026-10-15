// ChipConfigurator.hpp — chip bring-up for BCM4360 (43A0, Apple fullmac) and
// BCM43602/BCM4350 (43BA/43A3, msgbuf).
// Structure and register flow modelled on AppleBCMWLANChipConfigurator from
// the reversed BCMC 1.1.0 binary (analysis/key_functions/*).

#ifndef ChipConfigurator_hpp
#define ChipConfigurator_hpp

#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <libkern/OSByteOrder.h>
#include "BcmChip.hpp"
#include "Dlimage.hpp"
#include "SharedInfo.hpp"

// A core described by the EROM table. 16 bytes per entry, matching the
// layout written by EROM::Table::probe in the reversed binary.
struct CoreDescriptor {
    UInt32 coreId;    // e.g. 0x83e (Cortex-R4), 0x847 (A7), 0x82a (M3),
                      //      0x840 (D11), 0x827, 0x80e (internal memory)
    UInt32 flags;     // wrapper/address flags decodes from the EROM word
    UInt32 base;      // core base address (from findCoreAddresses)
    UInt32 size;      // core region size
};

class ChipConfigurator {
public:
    bool initWithDevice(IOPCIDevice *device);
    ~ChipConfigurator();
    bool probe();
    bool enterDownloadState();
    bool exitDownloadState(UInt32 vectors);
    bool downloadFirmware(const UInt8 *fw, IOByteCount length);
    UInt32 awaitFirmware(UInt32 timeoutMs);
    bool requestFirmware(const UInt8 **fw, IOByteCount *len, const ChipDescriptor *chip);

    // Broacom-ol bring-up helpers (decoded from AirPortBrcmNIC `_wlc_ol_up`,
    // see DLIMAGE_FINDINGS.md "Host-side bring-up").
    // Per-chip TCM/RAM load base for the ARM firmware image: 0x180000 for
    // 0x4350/0x4354/0xaa52/0xa9c6 and 0 for 0x4352/0x4360.
    static UInt32 tcmBaseForChip(UInt32 chipId);
    // Per-chip ARM SRAM size used to locate the shared-info block:
    // S = BAR1 + tcm_base + tcm_size - 0x5f80. The 4360 TCM is 512 KB
    // (0x80000 = `_si_tcm_size()` Bank0x83e walk); accept a runtimes value so
    // the poll at [S+0x5028] is the final arbiter.
    static UInt32 tcmSizeForChip(UInt32 chipId);

    // BAR1 VA of the chip register/memory window (`[ol+0x80]`), or nullptr.
    volatile UInt8 *bar1Address() const { return mBar1; }

    // Publish the shared-info block (magics + olmsg DMA bus addr + ring size)
    // at BAR1 + tcm_base + tcm_size - 0x5f80. Must be called after the olmsg
    // 64 KB buffer is allocated and before the firmware image is copied.
    // `dmaPhys` is the 32-bit bus address of that buffer (`ol+0x34`).
    bool publishSharedInfo(UInt32 dmaPhys, UInt32 dmaPhysHi = 0);

    // Poll [S+0x5028] until nonzero (firmware booted), ~2 s timeout, mirroring
    // `_wlc_ol_up`'s 0x7d0 * 1 ms loop. Returns nonzero ready flag / 0 on timeout.
    UInt32 awaitFirmwareReady(UInt32 timeoutMs);

    // Parsed container info for the firmware resource (valid after a successful
    // requestFirmware()).
    const dl_image::Info &getDlimageInfo() const { return mDlimageInfo; }

    // Probed values (mirrors of BCMC's Configurator fields).
    UInt32 getChipId() const { return mChipId; }
    UInt32 getChipRevision() const { return mChipRev; }
    UInt32 getInterconnectType() const { return mInterconnect; }
    UInt32 getCoreCount() const { return mCoreCount; }
    const CoreDescriptor *getCoreList() const { return mCoreList; }

    // One-shot single-line bring-up diagnostics (called by the controller
    // after awaitFirmwareReady): EROM core table, R4 base, tcm base/size,
    // computed shared-info S and ready offset. Grepped by
    // scripts/collect_boot_logs.sh's summary under "BOOT REPORT".
    void logBootReport();

private:
    bool probeEROM();
    bool readSROM();
    bool probeMemoryInfo();
    bool findCoreAddresses(UInt32 word, UInt32 *baseOut, UInt32 *sizeOut);
    const CoreDescriptor *findCoreById(UInt32 coreId) const;
    // TCM/RAM load base: parsed DBPP loadAddress, else the per-chip fallback
    // (`[ol+0x60]` == tcm_base in `_wlc_ol_up`).
    UInt32 tcmLoadBase() const;

    // Numeric shared-info offsets live in SharedInfo.hpp (pure module, host-
    // testable via scripts/test_bringup.cpp); ChipConfigurator uses
    // shared_info::{blockOffset, readyOffset, fits, gapBytes} directly.
    IOPCIDevice *mDevice {nullptr};
    IOMemoryMap *mMap {nullptr};       // chip memories map (BAR0)
    IOMemoryMap *mBar1Map {nullptr};   // BAR1: ARM TCM + doorbell + shared-info
    volatile UInt8 *mBar1 {nullptr};
    UInt32 mChipId {0};                // 0xb0 field
    UInt32 mChipRev {0};               // 0xb4 field
    UInt32 mInterconnect {0};          // 0xb8 field
    UInt32 mBackplaneCapabilities {0}; // 0xbc field
    UInt32 mBackplaneCapabilityData {0}; // 0xc0 field

    static constexpr UInt32 kMaxCores = 16;
    CoreDescriptor mCoreList[kMaxCores] {};
    UInt32 mCoreCount {0};
    UInt32 mTcmBase {0};
    UInt32 mTcmSize {0};
    const ChipDescriptor *mDescriptor {nullptr};
    dl_image::Info mDlimageInfo {};
    // Effective number of bytes memcpy'd into the TCM (`_wlc_ol_up` step 9 uses
    // the resource length, NOT the DBPP declared size which can exceed the TCM).
    UInt32 mFirmwareLength {0};
};

#endif /* ChipConfigurator_hpp */