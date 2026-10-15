// BcmChip.hpp — core Broadcom chip types for the OpenBCM (BCM) HAL.
// Extracted from the standalone AppleBCMWLANCompanion project so the OpenBCM
// HAL can reuse the reversed firmware bring-up (AppleCore `_wlc_ol_up`).

#ifndef BcmChip_hpp
#define BcmChip_hpp

#include <libkern/OSTypes.h>
#include <libkern/c++/OSObject.h>
#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>

// PCI device IDs handled by this driver.
enum class ChipDeviceID : UInt16 {
    kBCM4360   = 0x43a0,   // BCM4360  (BCM94360CS/CS2/CD/HMB, MBA/MBP 2013-2015)
    kBCM43602  = 0x43ba,   // BCM43602 (BCM943602CS/CDP/BAED, DW1830)
    kBCM4350   = 0x43a3,   // BCM4350  (BCM94350ZAE, DW1820A)
};

// Internal chip ids reported by the backplane (raw chipcommon CHIPID register).
enum class ChipNumber : UInt32 {
    kBCM4360   = 0x00004360,   // BCM4360  (PCI 14e4:43a0)
    kBCM43602  = 0x0000aa52,   // BCM43602 (PCI 14e4:43ba)
    kBCM4350   = 0x00004350,   // BCM4350  (PCI 14e4:43a3)
};

// ARM core ids that the configurator recognises (from Configurator::probe).
enum class CoreID : UInt32 {
    kCortexR4 = 0x0000083e,   // only core accepted by BCMC 1.1.0
    kCortexA7 = 0x00000847,   // rejected: "not supported at this moment"
    kCortexM3 = 0x0000082a,   // rejected
    kInternalMemory = 0x0000080e, // internal memory core (exitDownloadState gate)
};

#define GET_BITFIELD(v, hi, lo) (((v) >> (lo)) & ((1u << ((hi) - (lo) + 1)) - 1))

struct PCIeRingInfo {
    UInt32 h2dRingSize;   // host-to-device submission ring
    UInt32 d2hRingSize;   // device-to-host completion ring
    UInt32 maxTxAggregations;
    UInt32 maxRxAggregations;
};

enum class TransportType : UInt8 {
    kAppleCore = 0,  // 4360/43602/4350: the AirPortBrcmNIC `_wlc_ol_up`
                     // BCDC/olmsg control plane + host-driven D11 SoftMAC data path.
    kMmsgbuf   = 1,  // reserved
};

struct ChipDescriptor {
    UInt16 deviceId;   // PCI device id
    UInt32 chipNumber; // backplane chip id
    TransportType transport;
    const char *firmwareResourceName; // embedded resource in the kext bundle
    UInt32 sromSlide;   // SROM slide offset for this chip
    bool  needsCortexR4; // configurator gate
};

// All three run the same AppleCore `_wlc_ol_up` loader; the firmware resource
// name is informational (the image is selected by chipNumber/rev via the
// embedded arrays).
static const ChipDescriptor kBcmChipTable[] = {
    { 0x43a0, 0x00004360, TransportType::kAppleCore, "apple_4352pci_dlarray.bin",    0x00000000, true },
    { 0x43ba, 0x0000aa52, TransportType::kAppleCore, "apple_43602pci_dlarray.bin",   0x00000000, true },
    { 0x43a3, 0x00004350, TransportType::kAppleCore, "apple_4350pci_dlarray.bin",    0x40000000, true },
};

// Firmware IPC channel addresses exposed after download (shared memory hdr).
struct FirmwareIPCAddresses {
    UInt32 d2hRingAddr;
    UInt32 d2hRingWriteIndices;
    UInt32 d2hRingWriteIndexAddr;
    UInt32 d2hRingReadIndexAddr;
    UInt32 h2dRingWriteIndexAddr;
    UInt32 h2dRingReadIndexAddr;
};

#endif /* BcmChip_hpp */
