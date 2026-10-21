// Chanspec.hpp — Broadcom chanspec encode/decode, pure C++ (no IOKit).
// Layouts taken from Linux brcmutil/d11.c + include/brcmu_d11.h:
//   d11n  : 802.11n cores  (4331, 43224, ...)
//   d11ac : 802.11ac cores (4350, 4356, 43602, 4360, 4365, ...)
// BCM4360/43602/4350 are AC parts: use Io::kD11ac for them.

#ifndef Chanspec_hpp
#define Chanspec_hpp

#include <stdint.h>

namespace bcm {
namespace chanspec {

enum class Io : uint8_t { kD11n = 1, kD11ac = 2 };
enum class Bw : uint8_t { k20 = 20, k40 = 40, k80 = 80, k160 = 160 };
enum class Band : uint8_t { k2G, k5G };
// Sideband. For 20 MHz it is ignored on encode (decoded as kNone).
enum class Sb : uint8_t { kNone, kL, kU };

static constexpr uint16_t kChMask = 0x00ff;
static constexpr uint8_t  kMax2GChannel = 14;

// d11n
static constexpr uint16_t kN_SbMask = 0x0300, kN_SbL = 0x0100, kN_SbU = 0x0200, kN_SbN = 0x0300;
static constexpr uint16_t kN_BwMask = 0x0c00, kN_Bw20 = 0x0800, kN_Bw40 = 0x0c00;
static constexpr uint16_t kN_BndMask = 0x3000, kN_Bnd5G = 0x1000, kN_Bnd2G = 0x2000;

// d11ac
static constexpr uint16_t kAc_SbMask = 0x0700, kAc_SbShift = 8;   // LLL=0 LLU=1 ... UUU=7
static constexpr uint16_t kAc_BwMask = 0x3800;
static constexpr uint16_t kAc_Bw20 = 0x1000, kAc_Bw40 = 0x1800, kAc_Bw80 = 0x2000, kAc_Bw160 = 0x2800;
static constexpr uint16_t kAc_BndMask = 0xc000, kAc_Bnd2G = 0x0000, kAc_Bnd5G = 0xc000;

// Encode a chanspec. For d11ac, 20 MHz uses sideband "L" (as brcmf does);
// 40 MHz honours `sb`; 80/160 MHz use sideband index 0 (LL/LLL) — callers
// that need a specific 80 MHz control position should use encodeAcRaw().
// Returns 0 for unsupported combinations (d11n: only 20/40 MHz).
inline uint16_t encode(Io io, uint8_t chnum, Bw bw = Bw::k20, Sb sb = Sb::kNone) {
    uint16_t cs = chnum;
    if (io == Io::kD11n) {
        if (bw == Bw::k20) {
            cs |= kN_SbN | kN_Bw20;
        } else if (bw == Bw::k40 && sb != Sb::kNone) {
            cs |= (sb == Sb::kL ? kN_SbL : kN_SbU) | kN_Bw40;
        } else {
            return 0;
        }
        cs |= (chnum <= kMax2GChannel) ? kN_Bnd2G : kN_Bnd5G;
        return cs;
    }
    uint16_t sbv = 0;                       // SB_L == 0x0000 (LLL)
    if (bw == Bw::k40 && sb == Sb::kU)
        sbv = 1u << kAc_SbShift;            // SB_U == LLU == 0x0100
    uint16_t bwv = 0;
    switch (bw) {
    case Bw::k20:  bwv = kAc_Bw20;  break;
    case Bw::k40:  bwv = kAc_Bw40;  break;
    case Bw::k80:  bwv = kAc_Bw80;  break;
    case Bw::k160: bwv = kAc_Bw160; break;
    }
    cs |= sbv | bwv;
    cs |= (chnum <= kMax2GChannel) ? kAc_Bnd2G : kAc_Bnd5G;
    return cs;
}

inline uint16_t encode20(Io io, uint8_t chnum) { return encode(io, chnum, Bw::k20); }

struct Decoded {
    bool     valid {false};
    uint8_t  chnum {0};          // center channel (as in the chanspec)
    uint8_t  controlCh {0};      // control (primary) channel
    Bw       bw {Bw::k20};
    Band     band {Band::k2G};
};

inline Decoded decode(Io io, uint16_t cs) {
    Decoded d;
    d.chnum = (uint8_t)(cs & kChMask);
    int ctl = d.chnum;
    if (io == Io::kD11n) {
        switch (cs & kN_BwMask) {
        case kN_Bw20: d.bw = Bw::k20; break;
        case kN_Bw40:
            d.bw = Bw::k40;
            ctl += ((cs & kN_SbMask) == kN_SbL) ? -2 : 2;   // CH_10MHZ_APART = 2 (in channel numbers)
            break;
        default: return d;
        }
        switch (cs & kN_BndMask) {
        case kN_Bnd5G: d.band = Band::k5G; break;
        case kN_Bnd2G: d.band = Band::k2G; break;
        default: return d;
        }
    } else {
        unsigned sb = (cs & kAc_SbMask) >> kAc_SbShift;
        switch (cs & kAc_BwMask) {
        case kAc_Bw20: d.bw = Bw::k20; break;
        case kAc_Bw40:
            d.bw = Bw::k40;
            if (sb == 0) ctl -= 2; else if (sb == 1) ctl += 2; else return d;
            break;
        case kAc_Bw80:
            d.bw = Bw::k80;
            switch (sb) {   // LL LU UL UU  (values 0..3 of LLL LLU LUL LUU)
            case 0: ctl -= 6; break;
            case 1: ctl -= 2; break;
            case 2: ctl += 2; break;
            case 3: ctl += 6; break;
            default: return d;
            }
            break;
        case kAc_Bw160:
            d.bw = Bw::k160;
            switch (sb) {
            case 0: ctl -= 14; break;
            case 1: ctl -= 10; break;
            case 2: ctl -= 6;  break;
            case 3: ctl -= 2;  break;
            case 4: ctl += 2;  break;
            case 5: ctl += 6;  break;
            case 6: ctl += 10; break;
            case 7: ctl += 14; break;
            }
            break;
        default: return d;
        }
        switch (cs & kAc_BndMask) {
        case kAc_Bnd5G: d.band = Band::k5G; break;
        case kAc_Bnd2G: d.band = Band::k2G; break;
        default: return d;
        }
    }
    d.controlCh = (uint8_t)ctl;
    d.valid = true;
    return d;
}

// Convenience: control channel of a chanspec, 0 when invalid.
inline uint8_t controlChannel(Io io, uint16_t cs) {
    Decoded d = decode(io, cs);
    return d.valid ? d.controlCh : 0;
}

} // namespace chanspec
} // namespace bcm

#endif /* Chanspec_hpp */
