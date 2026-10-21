// PcieBringup.hpp — brcmfmac-style PCIe bring-up for FullMAC firmware
// (BCM43602 / ARM CR4), pure C++ over a Bus interface.
//
// Ported from Linux brcmfmac: pcie.c (reset_device, attach, enter/exit
// download state, download_fw_nvram), chip.c (AI resetcore/coredisable,
// cr4 set_passive/set_active, CR4 RAM geometry). The Bus abstraction lets the
// same sequence run in the kext (IOPCIDevice BAR0/BAR2) and in host tests
// against a simulated chip (tests/host/test_pcie_bringup.cpp).
//
// Bus contract (all methods are required):
//   void     setBar0Window(uint32_t addr);        // PCI cfg 0x80 (BAR0 window)
//   uint32_t regRead32(uint32_t off);              // BAR0, offset inside the 4 KB window
//   void     regWrite32(uint32_t off, uint32_t v);
//   uint32_t cfgRead32(uint32_t off);              // PCI config space
//   void     cfgWrite32(uint32_t off, uint32_t v);
//   uint32_t tcmRead32(uint32_t addr);             // BAR2 (dongle RAM), addr = RAM offset
//   void     tcmWrite32(uint32_t addr, uint32_t v);
//   void     tcmWrite(uint32_t addr, const void *p, size_t n);
//   void     sleepMs(uint32_t ms);
//   void     sleepUs(uint32_t us);

#ifndef PcieBringup_hpp
#define PcieBringup_hpp

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace pcie {

// ---- BCMA core ids -----------------------------------------------------------
static constexpr uint16_t kCoreChipCommon = 0x800, kCoreInternalMem = 0x80E,
                          kCore80211 = 0x812, kCorePcie2 = 0x83C, kCoreArmCr4 = 0x83E;
// ---- AI wrapper registers -----------------------------------------------------
static constexpr uint32_t kIoctl = 0x408, kResetCtl = 0x800;
static constexpr uint32_t kIoctlClk = 0x1, kIoctlFgc = 0x2, kResetCtlReset = 0x1;
static constexpr uint32_t kArmCr4IoctlCpuHalt = 0x20;
static constexpr uint32_t kD11IoctlPhyClockEn = 0x4, kD11IoctlPhyReset = 0x8;
// ---- ARM CR4 core registers (relative to core base) ---------------------------
static constexpr uint32_t kCr4Cap = 0x04, kCr4BankIdx = 0x40, kCr4BankInfo = 0x44, kCr4BankPda = 0x4C;
// ---- chipcommon / PCIe2 registers ---------------------------------------------
static constexpr uint32_t kCcWatchdog = 0x80;
static constexpr uint32_t kPcie2ConfigAddr = 0x120, kPcie2ConfigData = 0x124;
static constexpr uint32_t kPcie2IntMask = 0x24, kPcie2MailboxInt = 0x48, kPcie2MailboxMask = 0x4C,
                          kPcie2H2dMailbox0 = 0x140, kPcie2H2dMailbox1 = 0x144;
// ---- PCI config offsets ----------------------------------------------------------
static constexpr uint32_t kCfgBar0Window = 0x80, kCfgLinkStatusCtrl = 0xBC, kAspmEnable = 3;
static constexpr uint32_t kCfgRegs[] = {0x4, 0x4C, 0x58, 0x5C, 0x60, 0x64, 0xDC, 0x228, 0x248, 0x4E0, 0x4F4};
// ---- misc -----------------------------------------------------------------------------
static constexpr uint32_t kFwUpTimeoutMs = 5000;
static constexpr uint32_t kBar0RegSize = 0x1000;
static constexpr uint32_t kChipId43602 = 43602;             // 0xaa52, see BcmChip.hpp
static constexpr uint32_t kRamBase43602 = 0x180000;

struct Core {
    uint16_t id {0};
    uint32_t rev {0};
    uint32_t base {0};       // register window base (enumeration space)
    uint32_t wrap {0};       // AI wrapper base (from EROM)
};

enum class Status : uint8_t {
    kOk, kNoCore, kFwTimeout, kBadSharedAddr, kBadRam, kInvalidArg,
};

template <class Bus>
class Bringup {
public:
    explicit Bringup(Bus &bus) : mBus(bus) {}

    // ---- buscore accessors (brcmf_pcie_buscore_read32/write32) ------------------
    uint32_t read32(uint32_t addr) {
        mBus.setBar0Window(addr & ~(kBar0RegSize - 1));
        return mBus.regRead32(addr & (kBar0RegSize - 1));
    }
    void write32(uint32_t addr, uint32_t v) {
        mBus.setBar0Window(addr & ~(kBar0RegSize - 1));
        mBus.regWrite32(addr & (kBar0RegSize - 1), v);
    }
    // brcmf_pcie_select_core + register access inside the selected core window.
    void selectCore(const Core &c) { mBus.setBar0Window(c.base); }

    // ---- AI core control (chip.c) -------------------------------------------------
    bool coreIsUp(const Core &c) {
        uint32_t ioctl = read32(c.wrap + kIoctl);
        bool up = (ioctl & (kIoctlFgc | kIoctlClk)) == kIoctlClk;
        return up && (read32(c.wrap + kResetCtl) & kResetCtlReset) == 0;
    }

    void coreDisable(const Core &c, uint32_t prereset, uint32_t reset) {
        if ((read32(c.wrap + kResetCtl) & kResetCtlReset) == 0) {
            write32(c.wrap + kIoctl, prereset | kIoctlFgc | kIoctlClk);
            (void)read32(c.wrap + kIoctl);
            write32(c.wrap + kResetCtl, kResetCtlReset);
            mBus.sleepUs(20);
            for (int i = 0; i < 300 && read32(c.wrap + kResetCtl) != kResetCtlReset; i++)
                mBus.sleepUs(1);
        }
        write32(c.wrap + kIoctl, reset | kIoctlFgc | kIoctlClk);
        (void)read32(c.wrap + kIoctl);
    }

    void coreReset(const Core &c, uint32_t prereset, uint32_t reset, uint32_t postreset) {
        coreDisable(c, prereset, reset);
        int count = 0;
        while (read32(c.wrap + kResetCtl) & kResetCtlReset) {
            write32(c.wrap + kResetCtl, 0);
            if (++count > 50) break;
            mBus.sleepUs(50);
        }
        write32(c.wrap + kIoctl, postreset | kIoctlClk);
        (void)read32(c.wrap + kIoctl);
    }

    // ARM CR4: keep only the HALT bit, then reset with HALT set (disable_arm).
    void disableCr4(const Core &cr4) {
        uint32_t val = read32(cr4.wrap + kIoctl) & kArmCr4IoctlCpuHalt;
        coreReset(cr4, val, kArmCr4IoctlCpuHalt, kArmCr4IoctlCpuHalt);
    }

    // brcmf_chip_cr4_set_passive: halt the CPU, park the D11 cores.
    void setPassive(const Core &cr4, const Core *d11, size_t nD11) {
        disableCr4(cr4);
        for (size_t i = 0; i < nD11; i++)
            coreDisable(d11[i], kD11IoctlPhyReset | kD11IoctlPhyClockEn, kD11IoctlPhyClockEn);
    }

    // ---- CR4 RAM geometry (brcmf_chip_tcm_ramsize) ---------------------------------
    uint32_t cr4RamSize(const Core &cr4) {
        uint32_t cap = read32(cr4.base + kCr4Cap);
        uint32_t nab = cap & 0xf, nbb = (cap & 0xf0) >> 4;
        uint32_t total = 0;
        for (uint32_t i = 0; i < nab + nbb; i++) {
            write32(cr4.base + kCr4BankIdx, i);
            uint32_t info = read32(cr4.base + kCr4BankInfo);
            uint32_t blk = 8192;
            if (info & 0x200) blk >>= 3;
            total += ((info & 0x7f) + 1) * blk;
        }
        return total;
    }
    static uint32_t ramBase(uint32_t chipId) { return chipId == kChipId43602 ? kRamBase43602 : 0; }

    // ---- brcmf_pcie_reset_device ------------------------------------------------------
    void resetDevice(const Core &cc, const Core &pcie2) {
        selectCore(pcie2);
        uint32_t lsc = mBus.cfgRead32(kCfgLinkStatusCtrl);          // disable ASPM
        mBus.cfgWrite32(kCfgLinkStatusCtrl, lsc & ~kAspmEnable);
        selectCore(cc);
        mBus.regWrite32(kCcWatchdog, 4);                              // watchdog reset
        mBus.sleepMs(100);
        selectCore(pcie2);
        mBus.cfgWrite32(kCfgLinkStatusCtrl, lsc);                     // restore ASPM
        if (pcie2.rev <= 13) {
            for (uint32_t off : kCfgRegs) {
                mBus.regWrite32(kPcie2ConfigAddr, off);
                uint32_t v = mBus.regRead32(kPcie2ConfigData);
                mBus.regWrite32(kPcie2ConfigData, v);
            }
        }
    }

    // brcmf_pcie_attach: BAR1 window may not be sized properly.
    void attachFixups(const Core &pcie2) {
        selectCore(pcie2);
        mBus.regWrite32(kPcie2ConfigAddr, 0x4e0);
        uint32_t cfg = mBus.regRead32(kPcie2ConfigData);
        mBus.regWrite32(kPcie2ConfigData, cfg);
    }

    // ---- brcmf_pcie_download_fw_nvram + exit_download_state -----------------------------
    struct Geometry { uint32_t ramBase {0}, ramSize {0}; };

    // `nvram` is the blob produced by nvram::strip (already padded + token).
    // On success *sharedAddr holds the shared-RAM address announced by firmware.
    Status downloadAndStart(const Core &cr4, const Core &internalMem, uint32_t chipId,
                            const Geometry &g, const uint8_t *fw, size_t fwLen,
                            const uint8_t *nvram, size_t nvLen, uint32_t *sharedAddr) {
        if (!fw || fwLen < 4 || !g.ramSize || fwLen > g.ramSize) return Status::kInvalidArg;
        if (nvLen > g.ramSize - fwLen) return Status::kBadRam;

        // enter download state (43602: power down banks 5 and 7)
        if (chipId == kChipId43602) {
            write32(cr4.base + kCr4BankIdx, 5);
            write32(cr4.base + kCr4BankPda, 0);
            write32(cr4.base + kCr4BankIdx, 7);
            write32(cr4.base + kCr4BankPda, 0);
        }
        mBus.tcmWrite(g.ramBase, fw, fwLen);                                 // image at rambase
        uint32_t resetVec = (uint32_t)fw[0] | ((uint32_t)fw[1] << 8) |
                            ((uint32_t)fw[2] << 16) | ((uint32_t)fw[3] << 24);
        // All TCM addresses are absolute dongle addresses (BAR2 is indexed by them).
        const uint32_t sharedSlot = g.ramBase + g.ramSize - 4;
        mBus.tcmWrite32(sharedSlot, 0);                                       // "FW alive" marker
        if (nvram && nvLen)
            mBus.tcmWrite(g.ramBase + g.ramSize - (uint32_t)nvLen, nvram, nvLen);
        uint32_t written = mBus.tcmRead32(sharedSlot);

        // exit download state
        if (chipId == kChipId43602) coreReset(internalMem, 0, 0, 0);
        mBus.tcmWrite32(0, resetVec);                                         // activate: reset vector
        coreReset(cr4, kArmCr4IoctlCpuHalt, 0, 0);                            // release CPU

        uint32_t addr = written;
        for (uint32_t loops = kFwUpTimeoutMs / 50; addr == written && loops; loops--) {
            mBus.sleepMs(50);
            addr = mBus.tcmRead32(sharedSlot);
        }
        if (addr == written) return Status::kFwTimeout;
        if (addr < g.ramBase || addr >= g.ramBase + g.ramSize) return Status::kBadSharedAddr;
        if (sharedAddr) *sharedAddr = addr;
        return Status::kOk;
    }

    // ---- doorbell / interrupts (brcmf_pcie_*, reginfo_default) ----------------------------
    // Like brcmfmac, bring-up leaves the BAR0 window on the PCIe2 core
    // (selectCore(pcie2)); these then touch PCIe2 registers directly.
    void ringBell0() { mBus.regWrite32(kPcie2H2dMailbox0, 1); }
    void hostReady() { mBus.regWrite32(kPcie2H2dMailbox1, 1); }
    void enableInterrupts(uint32_t mask) { mBus.regWrite32(kPcie2MailboxMask, mask); }
    void disableInterrupts() { mBus.regWrite32(kPcie2MailboxMask, 0); }
    uint32_t mailboxInt() { return mBus.regRead32(kPcie2MailboxInt); }
    void ackMailboxInt(uint32_t v) { mBus.regWrite32(kPcie2MailboxInt, v); }

private:
    Bus &mBus;
};

} // namespace pcie

#endif /* PcieBringup_hpp */
