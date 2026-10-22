// Erom.hpp — AI/DMP (PL-368) EROM scan, pure C++. Port of brcmfmac chip.c
// brcmf_chip_dmp_erom_scan(): enumerates cores and yields register base and
// AI wrapper base for each, which PcieBringup needs.
//
// Reader contract: `uint32_t read32(uint32_t addr)` (BAR0 window must already
// cover the EROM; Linux reads it through the window set to enum base).

#ifndef Erom_hpp
#define Erom_hpp

#include <stdint.h>
#include "PcieBringup.hpp"

namespace erom {

static constexpr uint32_t kEnumBase = 0x18000000;       // SI_ENUM_BASE_DEFAULT
static constexpr uint32_t kCcEromPtr = 0xfc;            // chipcregs.eromptr
static constexpr uint16_t kCorePmu = 0x827, kCoreGci = 0x840;

static constexpr uint32_t kDescTypeMsk = 0xF, kDescEmpty = 0, kDescValid = 1,
                          kDescComponent = 1, kDescMasterPort = 3, kDescAddress = 5,
                          kDescAddrSizeGt32 = 8, kDescEot = 0xF;
static constexpr uint32_t kCompPartnum = 0x000FFF00, kCompPartnumS = 8,
                          kCompRevision = 0xFF000000, kCompRevisionS = 24,
                          kCompNumSwrap = 0x00F80000, kCompNumSwrapS = 19,
                          kCompNumMwrap = 0x0007C000, kCompNumMwrapS = 14;
static constexpr uint32_t kSlaveAddrBase = 0xFFFFF000, kSlaveType = 0xC0, kSlaveTypeS = 6,
                          kSlaveSizeType = 0x30, kSlaveSizeTypeS = 4;
static constexpr uint32_t kTypeSlave = 0, kTypeSwrap = 2, kTypeMwrap = 3;
static constexpr uint32_t kSize4K = 0, kSize8K = 1, kSizeDesc = 3;

static constexpr unsigned kMaxCores = 32;
static constexpr unsigned kMaxDescriptors = 4096;        // runaway guard

template <class Reader>
class Scanner {
public:
    explicit Scanner(Reader &r) : mR(r) {}

    // Returns number of cores found (0 on failure). Reads eromptr from chipcommon.
    unsigned scan(pcie::Core *out, unsigned cap, uint32_t enumBase = kEnumBase) {
        uint32_t ea = mR.read32(enumBase + kCcEromPtr);
        return scanAt(ea, out, cap);
    }

    unsigned scanAt(uint32_t ea, pcie::Core *out, unsigned cap) {
        unsigned n = 0, guard = 0;
        uint8_t type = 0;
        while (type != kDescEot && guard++ < kMaxDescriptors) {
            uint32_t val = getDesc(&ea, &type);
            if (!(val & kDescValid)) continue;
            if (type == kDescEmpty || type != kDescComponent) continue;
            uint16_t id = (val & kCompPartnum) >> kCompPartnumS;
            val = getDesc(&ea, &type);
            if ((val & kDescTypeMsk) != kDescComponent) return n;
            unsigned nmw = (val & kCompNumMwrap) >> kCompNumMwrapS;
            unsigned nsw = (val & kCompNumSwrap) >> kCompNumSwrapS;
            uint8_t rev = (val & kCompRevision) >> kCompRevisionS;
            if (nmw + nsw == 0 && id != kCorePmu && id != kCoreGci) continue;
            uint32_t base, wrap;
            if (getRegaddr(&ea, &base, &wrap) != 0) continue;
            if (n < cap && n < kMaxCores) {
                out[n].id = id; out[n].rev = rev; out[n].base = base; out[n].wrap = wrap;
                n++;
            }
        }
        return n;
    }

    static const pcie::Core *find(const pcie::Core *cores, unsigned n, uint16_t id) {
        for (unsigned i = 0; i < n; i++) if (cores[i].id == id) return &cores[i];
        return nullptr;
    }

private:
    uint32_t getDesc(uint32_t *ea, uint8_t *type) {
        uint32_t v = mR.read32(*ea);
        *ea += 4;
        if (type) {
            *type = (uint8_t)(v & kDescTypeMsk);
            if ((*type & ~kDescAddrSizeGt32) == kDescAddress) *type = (uint8_t)kDescAddress;
        }
        return v;
    }

    int getRegaddr(uint32_t *ea, uint32_t *regbase, uint32_t *wrapbase) {
        uint8_t desc;
        uint32_t wraptype;
        *regbase = *wrapbase = 0;
        getDesc(ea, &desc);
        if (desc == kDescMasterPort) {
            wraptype = kTypeMwrap;
        } else if (desc == kDescAddress) {
            *ea -= 4;
            wraptype = kTypeSwrap;
        } else {
            *ea -= 4;
            return -1;
        }
        unsigned guard = 0;
        do {
            uint32_t val;
            do {
                val = getDesc(ea, &desc);
                if (desc == kDescEot) { *ea -= 4; return -1; }
                if (guard++ > kMaxDescriptors) return -1;
            } while (desc != kDescAddress && desc != kDescComponent);
            if (desc == kDescComponent) { *ea -= 4; return 0; }
            if (val & kDescAddrSizeGt32) getDesc(ea, nullptr);
            uint32_t sztype = (val & kSlaveSizeType) >> kSlaveSizeTypeS;
            if (sztype == kSizeDesc) {
                uint32_t sz = getDesc(ea, nullptr);
                if (sz & kDescAddrSizeGt32) getDesc(ea, nullptr);
            }
            if (sztype != kSize4K && sztype != kSize8K) continue;
            uint32_t stype = (val & kSlaveType) >> kSlaveTypeS;
            if (*regbase == 0 && stype == kTypeSlave) *regbase = val & kSlaveAddrBase;
            if (*wrapbase == 0 && stype == wraptype) *wrapbase = val & kSlaveAddrBase;
        } while (*regbase == 0 || *wrapbase == 0);
        return 0;
    }

    Reader &mR;
};

} // namespace erom

#endif /* Erom_hpp */
