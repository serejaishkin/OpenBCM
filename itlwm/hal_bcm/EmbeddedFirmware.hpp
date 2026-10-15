// EmbeddedFirmware.hpp — resolver for firmware images compiled into the kext.
//
// The 43A0 dlimage (`_dlarray_4352pci`) is embedded as a const byte array by
// the generated source Chip/DlimageArrays.cpp (produced by
// scripts/gen_dlimage_arrays.py, driven from build.sh). This mirrors Apple's own
// approach in AirPortBrcmNIC, which ships `_dlarray_4352pci` as a __const symbol.

#ifndef EmbeddedFirmware_hpp
#define EmbeddedFirmware_hpp

#include <libkern/OSTypes.h>
#include <IOKit/IOTypes.h>

// Returns a pointer to the embedded firmware image for the given backplane
// chip number (e.g. 0x4360, 0xaa52) and core revision, or nullptr if none is
// compiled in. `chipRev` selects the BCM43602 a2 image (rev>=2) from the base
// image, mirroring AirPortBrcmNIC's `wlc_ol_fw_get`. On success *len receives
// the image length in bytes. The returned buffer is const and lives in __const
// for the kext lifetime.
const UInt8 *bcmc_get_dlimage(UInt32 chipNumber, UInt32 chipRev, IOByteCount *len);

#endif /* EmbeddedFirmware_hpp */
