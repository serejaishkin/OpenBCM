// Host test: EROM scan against a synthetic DMP table.
#include <stdio.h>
#include <map>
#include <vector>
#include "Erom.hpp"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Rd {
    std::map<uint32_t, uint32_t> m;
    uint32_t read32(uint32_t a) { auto it = m.find(a); return it == m.end() ? 0xF : it->second; }  // default: EOT
};

static uint32_t compA(uint16_t id) { return (0x4bfu << 20) | ((uint32_t)id << 8) | 1; }
static uint32_t compB(uint8_t rev, unsigned nmw, unsigned nsw) {
    return ((uint32_t)rev << 24) | (nsw << 19) | (nmw << 14) | 1;
}
static uint32_t mport() { return 3; }
static uint32_t addr(uint32_t base, unsigned type, unsigned sz) {
    return base | (type << 6) | (sz << 4) | 5;
}

int main() {
    Rd r;
    uint32_t ea = 0x18109000;
    r.m[erom::kEnumBase + erom::kCcEromPtr] = ea;
    std::vector<uint32_t> t;
    // chipcommon: no master wrap, 1 slave wrap
    t.push_back(compA(0x800)); t.push_back(compB(43, 0, 1));
    t.push_back(addr(0x18000000, 0, 0)); t.push_back(addr(0x18100000, 2, 0));
    // cr4: master port + master wrap, one slave
    t.push_back(compA(0x83E)); t.push_back(compB(2, 1, 0));
    t.push_back(mport());
    t.push_back(addr(0x18002000, 0, 0));
    t.push_back(addr(0x18102000, 3, 0));
    // pcie2 with a 64-bit address + size desc region that must be skipped
    t.push_back(compA(0x83C)); t.push_back(compB(13, 0, 1));
    t.push_back(addr(0x18003000, 0, 0));
    t.push_back(addr(0x20000000, 0, 3) | 8); t.push_back(0x0); t.push_back(0x00100000); // gt32 + size desc
    t.push_back(addr(0x18103000, 2, 0));
    // component without ports and not PMU: skipped
    t.push_back(compA(0x135)); t.push_back(compB(1, 0, 0));
    // PMU without ports is kept
    t.push_back(compA(0x827)); t.push_back(compB(20, 0, 0));
    t.push_back(addr(0x18012000, 0, 0)); t.push_back(addr(0x18112000, 2, 0));
    t.push_back(0xF);
    for (size_t i = 0; i < t.size(); i++) r.m[ea + 4 * i] = t[i];

    erom::Scanner<Rd> s(r);
    pcie::Core c[8];
    unsigned n = s.scan(c, 8);
    CHECK(n == 4);
    const pcie::Core *cc = erom::Scanner<Rd>::find(c, n, 0x800);
    const pcie::Core *cr4 = erom::Scanner<Rd>::find(c, n, 0x83E);
    const pcie::Core *pc = erom::Scanner<Rd>::find(c, n, 0x83C);
    const pcie::Core *pmu = erom::Scanner<Rd>::find(c, n, 0x827);
    CHECK(cc && cc->base == 0x18000000 && cc->wrap == 0x18100000 && cc->rev == 43);
    CHECK(cr4 && cr4->base == 0x18002000 && cr4->wrap == 0x18102000 && cr4->rev == 2);
    CHECK(pc && pc->base == 0x18003000 && pc->wrap == 0x18103000 && pc->rev == 13);
    CHECK(pmu && pmu->base == 0x18012000 && pmu->wrap == 0x18112000);
    CHECK(!erom::Scanner<Rd>::find(c, n, 0x135));

    // truncated table (no EOT, reads return garbage zeros) must terminate
    Rd z;
    z.m[erom::kEnumBase + erom::kCcEromPtr] = 0x1000;
    erom::Scanner<Rd> s2(z);
    pcie::Core c2[4];
    (void)s2.scan(c2, 4);

    if (g_fail) { printf("%d failures\n", g_fail); return 1; }
    printf("erom: OK\n");
    return 0;
}
