// BcmChip.hpp — core Broadcom chip types for the OpenBCM (BCM) HAL.
// Extracted from the standalone AppleBCMWLANCompanion project so the OpenBCM
// HAL can reuse the reversed firmware bring-up (AppleCore `_wlc_ol_up`).

#ifndef BcmChip_hpp
#define BcmChip_hpp

#include <libkern/OSTypes.h>
#include <libkern/c++/OSObject.h>
#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOKitKeys.h>
#include <libkern/OSString.h>

// PCI device IDs handled by this driver.
// These cover Broadcom Wi-Fi chips supported by the old Apple AirPortBrcmNIC /
// AppleBCMWLAN stack (Skywalk and pre-Skywalk). Only chips with embedded or
// portable firmware are fully functional; others match but will need a
// firmware resource added before they can bring up.
enum class ChipDeviceID : UInt16 {
    // BCM4360 family (PCI 14e4:43a0) - fullmac-ish, AppleCore olmsg
    kBCM4360   = 0x43a0,   // BCM4360  (BCM94360CS/CS2/CD/HMB, MBA/MBP 2013-2015)
    // BCM43602 family (PCI 14e4:43ba) - fullmac-ish, AppleCore olmsg
    kBCM43602  = 0x43ba,   // BCM43602 (BCM943602CS/CDP/BAED, DW1830)
    // BCM4350 family (PCI 14e4:43a3) - msgbuf-ish
    kBCM4350   = 0x43a3,   // BCM4350  (BCM94350ZAE, DW1820A)
    // Other Broadcom Wi-Fi chips supported by AppleBCMWLAN but not yet wired
    // up with their own firmware image. They match here and will use a
    // fallback firmware resource or fail gracefully.
    kBCM4352   = 0x4352,   // BCM4352  (BCM94352ZAE)
    kBCM4357   = 0x4357,   // BCM4357
    kBCM4358   = 0x4358,   // BCM4358
    kBCM4359   = 0x4359,   // BCM4359
    kBCM4365   = 0x4365,   // BCM4365
    kBCM4356   = 0x4356,   // BCM4356
    kBCM43224  = 0x43224,  // BCM43224
    kBCM43225  = 0x43225,  // BCM43225
    kBCM43227  = 0x43227,  // BCM43227
    kBCM43228  = 0x43228,  // BCM43228
    kBCM43236  = 0x43236,  // BCM43236
    kBCM43237  = 0x43237,  // BCM43237
    kBCM43238  = 0x43238,  // BCM43238
    kBCM43239  = 0x43239,  // BCM43239
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

// Firmware source priority: embedded array → kext bundle resource → filesystem.
enum class FirmwareSource : UInt8 {
    kEmbedded = 0,   // compiled-in DlimageArrays (default)
    kBundle   = 1,   // resource inside the kext bundle (Firmware/ dir)
    kFilesystem = 2, // /Library/Extensions/OpenBCM/Firmware/<name>
};

// Softmac vs fullmac: softmac = host-driven D11 TX/RX (this project),
// fullmac = firmware handles MAC/PHY, host only sends ioctls.
enum class ChipType : UInt8 {
    kSoftmac = 0,  // host-driven D11 SoftMAC (OpenBCM default)
    kFullmac = 1,  // firmware owns data plane, host sends commands only
};

// Per-device override, sourced from Info.plist personality dictionaries.
// When present, overrides the corresponding kBcmChipTable entry for this PCI
// device id.
struct ChipDeviceConfig {
    UInt16   deviceId {0};          // PCI device id this config applies to
    UInt32   chipNumber {0};        // override backplane chip id (0 = keep table)
    UInt32   chipRev {0xffffffff};  // override chip revision (0xffffffff = keep table)
    FirmwareSource fwSource {FirmwareSource::kEmbedded};
    char     firmwareResourceName[64] {0}; // embedded name or bundle filename
    TransportType transport {TransportType::kAppleCore};
    ChipType chipType {ChipType::kSoftmac};
    UInt32   d11CoreBase {0};       // override D11 core base (0 = keep table)
    bool     needsCortexR4 {true};
};

struct ChipDescriptor {
    UInt16 deviceId;   // PCI device id
    UInt32 chipNumber; // backplane chip id
    TransportType transport;
    const char *firmwareResourceName; // embedded resource in the kext bundle
    UInt32 sromSlide;   // SROM slide offset for this chip
    bool  needsCortexR4; // configurator gate
    // D11 SoftMAC core base address (backplane address of core id 0x840).
    // Filled by ChipConfigurator::probe() from the EROM table; hardcoded
    // fallback values below pending per-chip EROM validation.
    UInt32 d11CoreBase;
    // Softmac vs fullmac: softmac = host-driven D11 SoftMAC (OpenBCM default),
    // fullmac = firmware handles MAC/PHY, host only sends commands only.
    ChipType chipType {ChipType::kSoftmac};

    // Apply a per-device personality override on top of this table entry.
    void applyOverride(const ChipDeviceConfig &cfg);
};

// All three run the same AppleCore `_wlc_ol_up` loader; the firmware resource
// name is informational (the image is selected by chipNumber/rev via the
// embedded arrays).
static const ChipDescriptor kBcmChipTable[] = {
    { 0x43a0, 0x00004360, TransportType::kAppleCore, "apple_4352pci_dlarray.bin",    0x00000000, true, 0x18004000, ChipType::kSoftmac },
    { 0x43ba, 0x0000aa52, TransportType::kAppleCore, "apple_43602pci_dlarray.bin",   0x00000000, true, 0x18004000, ChipType::kSoftmac },
    { 0x43a3, 0x00004350, TransportType::kAppleCore, "apple_4350pci_dlarray.bin",    0x40000000, true, 0x18004000, ChipType::kSoftmac },
    // Other Broadcom chips: no embedded firmware yet, will try bundle or
    // filesystem path. Chip type is softmac unless the chip is known fullmac.
    { 0x4352, 0x00004352, TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x4357, 0x00004357, TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x4358, 0x00004358, TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x4359, 0x00004359, TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x4365, 0x00004365, TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x4356, 0x00004356, TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43224,0x000043224,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43225,0x000043225,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43227,0x000043227,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43228,0x000043228,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43236,0x000043236,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43237,0x000043237,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43238,0x000043238,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
    { 0x43239,0x000043239,TransportType::kAppleCore, nullptr,    0x00000000, false, 0x0, ChipType::kSoftmac },
};

inline void ChipDescriptor::applyOverride(const ChipDeviceConfig &cfg) {
    if (cfg.chipNumber != 0) chipNumber = cfg.chipNumber;
    // chipRev is read from hardware in ChipConfigurator::probe(); overrides
    // are applied there via mChipRevOverride.
    if (cfg.firmwareResourceName[0] != 0) firmwareResourceName = cfg.firmwareResourceName;
    if (cfg.transport != TransportType::kAppleCore) transport = cfg.transport;
    if (cfg.chipType != ChipType::kSoftmac) chipType = cfg.chipType;
    if (cfg.d11CoreBase != 0) d11CoreBase = cfg.d11CoreBase;
    if (cfg.needsCortexR4 != false) needsCortexR4 = cfg.needsCortexR4;
}

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
