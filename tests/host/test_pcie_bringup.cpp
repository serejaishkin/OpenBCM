// Host test: PcieBringup.hpp against a simulated BCM43602 (AI wrappers, CR4 RAM
// banks, a "firmware" that announces the shared RAM address once released).
#include <stdio.h>
#include <string.h>
#include <map>
#include <vector>
#include "PcieBringup.hpp"
#include "Nvram.hpp"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

using namespace pcie;

struct SimChip {
    // cores
    Core cc{kCoreChipCommon, 0, 0x18000000, 0x18100000};
    Core pcie2{kCorePcie2, 13, 0x18003000, 0x18103000};
    Core cr4{kCoreArmCr4, 0, 0x18002000, 0x18102000};
    Core imem{kCoreInternalMem, 0, 0x18004000, 0x18104000};
    Core d11{kCore80211, 0, 0x18001000, 0x18101000};

    uint32_t window {0};
    std::map<uint32_t, uint32_t> regs;       // absolute reg address -> value
    std::map<uint32_t, uint32_t> cfg;        // PCI config space
    std::vector<uint8_t> tcm;
    uint32_t ramBase {0x180000}, ramSize {0xE0000};
    uint32_t fwSharedAddr {0x1c8000};
    int pollsBeforeUp {3};
    bool cpuReleased {false};
    uint32_t bankIdx {0};
    std::vector<uint32_t> bankInfo{0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f}; // 8 banks * 256 KB? (0x1f+1)*8192
    int msSlept {0};
    std::vector<std::pair<uint32_t,uint32_t>> log;   // (addr,value) of every BAR0 write
    uint32_t haltedWrites {0};

    SimChip() : tcm(0x400000, 0) {
        regs[cr4.base + kCr4Cap] = 0x44;                              // nab=4, nbb=4 -> 8 banks
        for (auto *c : {&cc, &pcie2, &cr4, &imem, &d11}) {
            regs[c->wrap + kResetCtl] = 0;
            regs[c->wrap + kIoctl] = kIoctlClk;
        }
        regs[cr4.wrap + kIoctl] = kIoctlClk | kArmCr4IoctlCpuHalt;
        cfg[kCfgLinkStatusCtrl] = 0x43;
    }

    // Bus interface
    void setBar0Window(uint32_t a) { window = a; }
    uint32_t regRead32(uint32_t off) {
        uint32_t addr = window + off;
        if (addr == cr4.base + kCr4BankInfo) return bankIdx < bankInfo.size() ? bankInfo[bankIdx] : 0;
        return regs.count(addr) ? regs[addr] : 0;
    }
    void regWrite32(uint32_t off, uint32_t v) {
        uint32_t addr = window + off;
        log.push_back({addr, v});
        if (addr == cr4.base + kCr4BankIdx) bankIdx = v;
        if (addr == cr4.wrap + kIoctl) cpuReleased = !(v & kArmCr4IoctlCpuHalt) && started();
        // AI wrapper: writing reset-ctl records the state; reset clears itself on 0.
        regs[addr] = v;
        if (addr == cr4.wrap + kIoctl) cpuReleased = !(v & kArmCr4IoctlCpuHalt) && started();
    }
    bool started() { uint32_t v; memcpy(&v, &tcm[0], 4); return v != 0; }   // reset vector present
    uint32_t cfgRead32(uint32_t off) { return cfg[off]; }
    void cfgWrite32(uint32_t off, uint32_t v) { cfg[off] = v; }
    uint32_t tcmRead32(uint32_t a) {
        if (a == ramBase + ramSize - 4 && cpuReleased && pollsBeforeUp-- <= 0) {
            memcpy(&tcm[a], &fwSharedAddr, 4);
        }
        uint32_t v; memcpy(&v, &tcm[a], 4); return v;
    }
    void tcmWrite32(uint32_t a, uint32_t v) { memcpy(&tcm[a], &v, 4); }
    void tcmWrite(uint32_t a, const void *p, size_t n) { memcpy(&tcm[a], p, n); }
    void sleepMs(uint32_t ms) { msSlept += ms; }
    void sleepUs(uint32_t) {}
};

static void test_core_control() {
    SimChip chip;
    Bringup<SimChip> b(chip);
    // reset: with reset ctl clear the wrapper gets prereset|FGC|CLK, then RESET, then reset|FGC|CLK, ... post|CLK
    chip.regs[chip.d11.wrap + kResetCtl] = 0;
    b.coreReset(chip.d11, 0x8, 0x4, 0x10);
    CHECK(chip.regs[chip.d11.wrap + kIoctl] == (0x10 | kIoctlClk));
    CHECK(chip.regs[chip.d11.wrap + kResetCtl] == 0);          // released
    CHECK(b.coreIsUp(chip.d11));
    b.coreDisable(chip.d11, 0x8, 0x4);
    CHECK(chip.regs[chip.d11.wrap + kResetCtl] == kResetCtlReset);
    CHECK(chip.regs[chip.d11.wrap + kIoctl] == (0x4 | kIoctlFgc | kIoctlClk));
    CHECK(!b.coreIsUp(chip.d11));
    // disableCr4 keeps HALT
    b.disableCr4(chip.cr4);
    CHECK((chip.regs[chip.cr4.wrap + kIoctl] & kArmCr4IoctlCpuHalt) != 0);
    // setPassive parks d11 with PHYCLOCKEN
    Core d11s[1] = {chip.d11};
    b.setPassive(chip.cr4, d11s, 1);
    CHECK(chip.regs[chip.d11.wrap + kResetCtl] == kResetCtlReset);
    CHECK((chip.regs[chip.d11.wrap + kIoctl] & kD11IoctlPhyClockEn) != 0);
}

static void test_ram_geometry() {
    SimChip chip;
    Bringup<SimChip> b(chip);
    // 8 banks, info 0x1f -> (0x1f+1) * 8192 = 256 KB each -> 2 MB; use 1K-block flag for bank 0..
    chip.bankInfo = {0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b};   // 28 * 8192 = 0x38000 each
    CHECK(b.cr4RamSize(chip.cr4) == 8u * 0x38000u);
    chip.bankInfo = {0x7 | 0x200, 0x200, 0x200, 0x200, 0x200, 0x200, 0x200, 0x200}; // 1K blocks
    CHECK(b.cr4RamSize(chip.cr4) == 8 * 1024 + 7 * 1024);
    CHECK(Bringup<SimChip>::ramBase(kChipId43602) == 0x180000);
}

static void test_reset_device() {
    SimChip chip;
    Bringup<SimChip> b(chip);
    b.resetDevice(chip.cc, chip.pcie2);
    CHECK(chip.msSlept >= 100);
    bool wd = false;
    for (auto &w : chip.log) if (w.first == chip.cc.base + kCcWatchdog && w.second == 4) wd = true;
    CHECK(wd);
    CHECK(chip.cfg[kCfgLinkStatusCtrl] == 0x43);                         // ASPM restored
    // pcie2 rev 13 -> config register round trip happened (11 CONFIGADDR writes)
    int n = 0;
    for (auto &w : chip.log) if (w.first == chip.pcie2.base + kPcie2ConfigAddr) n++;
    CHECK(n == 11);
    chip.log.clear();
    chip.pcie2.rev = 14;
    b.resetDevice(chip.cc, chip.pcie2);
    n = 0;
    for (auto &w : chip.log) if (w.first == chip.pcie2.base + kPcie2ConfigAddr) n++;
    CHECK(n == 0);
}

static void test_download() {
    SimChip chip;
    Bringup<SimChip> b(chip);
    std::vector<uint8_t> fw(4096, 0xAB);
    uint32_t resetVec = 0x00180004;
    memcpy(fw.data(), &resetVec, 4);
    const char *txt = "boardrev=0x1203\nvendid=0x14e4\n";
    uint8_t nv[256]; nvram::Info ni;
    CHECK(nvram::strip((const uint8_t *)txt, strlen(txt), nv, sizeof(nv), &ni) == nvram::Result::kOk);

    Bringup<SimChip>::Geometry g{chip.ramBase, chip.ramSize};
    uint32_t shared = 0;
    auto st = b.downloadAndStart(chip.cr4, chip.imem, kChipId43602, g, fw.data(), fw.size(), nv, ni.length, &shared);
    CHECK(st == Status::kOk && shared == chip.fwSharedAddr);
    CHECK(memcmp(&chip.tcm[chip.ramBase], fw.data(), fw.size()) == 0);                 // image at rambase
    // nvram sits at the end of RAM; firmware then overwrites the last word with the
    // shared-RAM address, so only the part before it is compared.
    CHECK(memcmp(&chip.tcm[chip.ramBase + chip.ramSize - ni.length], nv, ni.length - 4) == 0);
    uint32_t tail; memcpy(&tail, &chip.tcm[chip.ramBase + chip.ramSize - 4], 4);
    CHECK(tail == chip.fwSharedAddr);
    uint32_t vec; memcpy(&vec, &chip.tcm[0], 4);
    CHECK(vec == resetVec);                                                              // reset vector at 0
    CHECK(chip.bankIdx == 7);                                                             // 43602 bank power-down ran
    CHECK((chip.regs[chip.cr4.wrap + kIoctl] & kArmCr4IoctlCpuHalt) == 0);              // CPU running
    CHECK(chip.msSlept >= 150);

    // Firmware never answers -> timeout after ~5 s.
    SimChip dead; dead.pollsBeforeUp = 1 << 30;
    Bringup<SimChip> bd(dead);
    Bringup<SimChip>::Geometry gd{dead.ramBase, dead.ramSize};
    st = bd.downloadAndStart(dead.cr4, dead.imem, kChipId43602, gd, fw.data(), fw.size(), nv, ni.length, &shared);
    CHECK(st == Status::kFwTimeout && dead.msSlept >= 5000);

    // Firmware announces an address outside RAM.
    SimChip bad; bad.fwSharedAddr = 0x10; bad.pollsBeforeUp = 0;
    Bringup<SimChip> bb(bad);
    Bringup<SimChip>::Geometry gb{bad.ramBase, bad.ramSize};
    st = bb.downloadAndStart(bad.cr4, bad.imem, kChipId43602, gb, fw.data(), fw.size(), nv, ni.length, &shared);
    CHECK(st == Status::kBadSharedAddr);

    // Argument checks.
    CHECK(b.downloadAndStart(chip.cr4, chip.imem, kChipId43602, g, fw.data(), 2, nv, ni.length, &shared) == Status::kInvalidArg);
    CHECK(b.downloadAndStart(chip.cr4, chip.imem, kChipId43602, g, fw.data(), fw.size(), nv, chip.ramSize, &shared) == Status::kBadRam);
}

int main() {
    test_core_control();
    test_ram_geometry();
    test_reset_device();
    test_download();
    if (g_fail) { printf("%d check(s) FAILED\n", g_fail); return 1; }
    printf("all tests passed\n");
    return 0;
}
