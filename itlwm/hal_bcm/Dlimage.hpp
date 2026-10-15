// Dlimage.hpp — pure parser for Apple's AirPortBrcmNIC `_dlarray_*` firmware
// containers. Deliberately free of IOKit/OSObject dependencies so it can be
// compiled and self-tested on the host (see scripts/test_dlimage.*).
//
// Two container formats exist (verified 2026-10-11, DLIMAGE_FINDINGS.md):
//   * kRaw  — bare 16-word ARM vector table + Thumb-2 code, no header. Used by
//             `_dlarray_4352pci`, which is the blob selected for chipid 0x4360
//             (BCM4360 / PCI 14e4:43a0). Load base is the per-chip TCM base
//             (0 for 0x4352/0x4360).
//   * kDbpp — 16-word vector table, 0xfa fill 0x040..0x073, then "DBPP" at
//             0x074 + u32 size + u32 loadAddress (0x180000). Used by the
//             4350/43602 blobs (msgbuf chips, handled elsewhere).

#ifndef Dlimage_hpp
#define Dlimage_hpp

#include <stdint.h>
#include <stddef.h>

namespace dl_image {

enum class Container : uint8_t {
    kRaw  = 0,
    kDbpp = 1,
};

struct Info {
    Container container {Container::kRaw};
    uint32_t  payloadOffset {0};    // start of executable image
    uint32_t  imageSize {0};        // declared size (kDbpp) or resource size (kRaw)
    uint32_t  loadAddress {0};      // kDbpp load field, else fallback TCM base
    bool      valid {false};
};

static constexpr uint32_t kDbppMagicOffset = 0x074;
static constexpr uint32_t kDbppMagic       = 0x50504244u; // "DBPP" little-endian
static constexpr uint32_t kDbppCodeOffset  = 0x080;
static constexpr uint32_t kMinImageLen     = 0x080;

static inline uint32_t readLE32(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

// Parse a dlimage resource. `fallbackLoadAddress` is the per-chip TCM base used
// for raw containers (which carry no load address). Returns true and fills
// `out` on success; on failure `out->valid` is false.
static inline bool parse(const uint8_t *data, size_t len, uint32_t fallbackLoadAddress,
                         Info *out) {
    if (out) {
        out->valid = false;
    }
    if (!data || !out || len < kMinImageLen) {
        return false;
    }

    if (readLE32(data + kDbppMagicOffset) == kDbppMagic) {
        out->container   = Container::kDbpp;
        out->payloadOffset = kDbppCodeOffset;
        out->imageSize   = readLE32(data + kDbppMagicOffset + 4);
        out->loadAddress = readLE32(data + kDbppMagicOffset + 8);
        out->valid       = true;
        return true;
    }

    out->container   = Container::kRaw;
    out->payloadOffset = 0;
    out->imageSize   = (uint32_t)len;
    out->loadAddress = fallbackLoadAddress;
    out->valid       = true;
    return true;
}

} // namespace dl_image

#endif /* Dlimage_hpp */
