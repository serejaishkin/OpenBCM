// FullmacFwSelect.hpp — pure C++: which brcmfmac firmware / TCM RAM base a chip needs.
// Port of brcmfmac pcie.c brcmf_pcie_fwnames[] and chip.c brcmf_chip_tcm_rambase().
// Chip numbers use Linux's BRCM_CC_*_CHIP_ID values (e.g. 43602 decimal == 0xaa52, 0x4350).
// BCM4360 (0x4360) is deliberately absent: Linux has no FullMAC firmware for it.
#ifndef FullmacFwSelect_hpp
#define FullmacFwSelect_hpp

#include <stdint.h>

namespace fwsel {

struct Entry { uint32_t chip, revMask; const char *stem; };

// First match wins: (1u << rev) & revMask, like BRCMF_FW_ENTRY (mask over revision bits).
static const Entry kTable[] = {
    {43602,  0xFFFFFFFF, "brcmfmac43602-pcie"},
    {43465,  0xFFFFFFF0, "brcmfmac4366c-pcie"},
    {0x4350, 0x000000FF, "brcmfmac4350c2-pcie"},
    {0x4350, 0xFFFFFF00, "brcmfmac4350-pcie"},
    {43525,  0xFFFFFFF0, "brcmfmac4365c-pcie"},
    {0x4355, 0x000007FF, "brcmfmac4355-pcie"},
    {0x4355, 0xFFFFF800, "brcmfmac4355c1-pcie"},
    {0x4356, 0xFFFFFFFF, "brcmfmac4356-pcie"},
    {43567,  0xFFFFFFFF, "brcmfmac43570-pcie"},
    {43569,  0xFFFFFFFF, "brcmfmac43570-pcie"},
    {43570,  0xFFFFFFFF, "brcmfmac43570-pcie"},
    {0x4358, 0xFFFFFFFF, "brcmfmac4358-pcie"},
    {0x4359, 0x000001FF, "brcmfmac4359-pcie"},
    {0x4359, 0xFFFFFE00, "brcmfmac4359c-pcie"},
    {0x4365, 0x0000000F, "brcmfmac4365b-pcie"},
    {0x4365, 0xFFFFFFF0, "brcmfmac4365c-pcie"},
    {0x4366, 0x0000000F, "brcmfmac4366b-pcie"},
    {0x4366, 0xFFFFFFF0, "brcmfmac4366c-pcie"},
    {43664,  0xFFFFFFF0, "brcmfmac4366c-pcie"},
    {43666,  0xFFFFFFF0, "brcmfmac4366c-pcie"},
    {0x4371, 0xFFFFFFFF, "brcmfmac4371-pcie"},
};

// Firmware stem for a chip/revision, nullptr when unsupported (e.g. BCM4360).
inline const char *stemFor(uint32_t chip, uint32_t rev) {
    for (const Entry &e : kTable)
        if (e.chip == chip && rev < 32 && ((1u << rev) & e.revMask)) return e.stem;
    return nullptr;
}

// TCM RAM base; 0xFFFFFFFF = unknown chip.
inline uint32_t ramBase(uint32_t chip, uint32_t rev) {
    switch (chip) {
    case 0x4350: case 0x4354: case 0x4356: case 43567: case 43569: case 43570:
    case 0x4358: case 43602: case 0x4371:
        return 0x180000;
    case 43465: case 43525: case 0x4365: case 0x4366: case 43664: case 43666:
        return 0x200000;
    case 0x4355: case 0x4359:
        return rev < 9 ? 0x180000 : 0x160000;
    default:
        return 0xFFFFFFFF;
    }
}

// Normalise a user-supplied name ("brcmfmac4350-pcie.bin" / "4350" / path) to a stem.
// Writes into out (cap bytes). Strips directory, ".bin"/".txt".
inline void normalizeStem(const char *in, char *out, unsigned cap) {
    unsigned n = 0, start = 0;
    for (unsigned i = 0; in[i]; i++) if (in[i] == '/') start = i + 1;
    for (unsigned i = start; in[i] && n + 1 < cap; i++) out[n++] = in[i];
    out[n] = 0;
    if (n > 4 && (out[n-4] == '.') && ((out[n-3]=='b'&&out[n-2]=='i'&&out[n-1]=='n') ||
                                      (out[n-3]=='t'&&out[n-2]=='x'&&out[n-1]=='t')))
        out[n-4] = 0;
}

} // namespace fwsel

#endif
