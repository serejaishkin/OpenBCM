// SharedInfo.hpp — host-side shared-info block geometry, decoded from
// AirPortBrcmNIC `_wlc_ol_up` step 7 (see DLIMAGE_FINDINGS.md "Host-side
// bring-up"). Pure C++ (no IOKit) so the exact constants/offsets used by the
// kext can be unit-tested on the host (scripts/test_bringup.cpp).
//
// Layout (all u32 little-endian), S = TCM_base + TCM_size - 0x5f80:
//   S+0x0000 = 0xa5a5a5a5  magic1
//   S+0x0004 = olmsg DMA bus addr lo   ([ol+0x34])
//   S+0x0008 = olmsg DMA bus addr hi   ([ol+0x38]; 0 for force32 alloc)
//   S+0x000c = 0x00010000  olmsg buffer size ([ol+0x30])
//   S+0x0014 = 0
//   S+0x0018 = 0
//   S+0x0020 = misc flag ([[wlc]+0x110]; 0 is safe)
//   S+0x5f5c = 0x5a5a5a5a  magic2
//   S+0x5ed8 = 0
//   S+0x5028 = boot-ready flag: firmware writes nonzero, host polls (step 13)

#ifndef SharedInfo_hpp
#define SharedInfo_hpp

#include <stdint.h>
#include <stddef.h>

namespace shared_info {

static constexpr uint32_t kTcmTail    = 0x5f80; // S distance from the TCM end
static constexpr uint32_t kMagic1     = 0xa5a5a5a5;
static constexpr uint32_t kMagic2     = 0x5a5a5a5a;
static constexpr uint32_t kOffMagic1  = 0x00;   // == S+0x0000
static constexpr uint32_t kOffD2hAddr = 0x04;   // olmsg bus addr lo (== [ol+0x34])
static constexpr uint32_t kOffH2dAddr = 0x08;   // bus addr hi (== [ol+0x38])
static constexpr uint32_t kOffRingSz  = 0x0c;   // olmsg buffer size (== [ol+0x30])
static constexpr uint32_t kOffFlags   = 0x20;   // misc flag (== [[wlc]+0x110])
static constexpr uint32_t kMagic2Off  = 0x5f5c;
static constexpr uint32_t kReadyOff   = 0x5028; // firmware boots when != 0
static constexpr uint32_t kOlmsgSize  = 0x10000; // h2d/d2h ring allocation

// Offset of the shared-info block from the TCM base:
//   S_addr = TCM_base + TCM_size - 0x5f80   (`_wlc_ol_up` step 7)
inline uint32_t blockOffset(uint32_t tcmBase, uint32_t tcmSize) {
    return tcmBase + tcmSize - kTcmTail;
}

// Offset of the boot-ready flag: S + 0x5028.
inline uint32_t readyOffset(uint32_t tcmBase, uint32_t tcmSize) {
    return blockOffset(tcmBase, tcmSize) + kReadyOff;
}

// True if the firmware image (imageSize) plus the shared-info tail fits in a
// TCM of `tcmSize`. 64-bit intermediate to avoid overflow on 0x180000 bases.
inline bool fits(uint32_t tcmSize, uint32_t imageSize) {
    return (uint64_t)kTcmTail + imageSize <= tcmSize;
}

// Bytes the loader must zero between the image end and the shared-info block
// (`_wlc_ol_up` step 9 gap memset). 0 if the image already touches S.
inline uint32_t gapBytes(uint32_t tcmBase, uint32_t tcmSize, uint32_t imageBytes) {
    uint32_t s = blockOffset(tcmBase, tcmSize);
    uint64_t end = (uint64_t)tcmBase + imageBytes;
    if (s <= (uint64_t)end) {
        return 0;
    }
    return s - (uint32_t)end;
}

} // namespace shared_info

#endif /* SharedInfo_hpp */