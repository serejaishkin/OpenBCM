// FullmacPcie.hpp — IOKit glue for the FullMAC (brcmfmac-style) PCIe path.
// Ties the host-tested pure layers together on real hardware:
//   IoBus (BAR0 regs + BAR2 TCM + PCI cfg window) -> pcie::Bringup -> erom::Scanner
//   -> firmware/nvram download -> shared-RAM discovery -> common rings -> msgbuf::Engine
// Header-only (hal_bcm is a folder reference in the Xcode project).
// NOT built or run on hardware yet — written against brcmfmac pcie.c.
//
// BAR window rule: after start() the BAR0 window stays on the PCIe2 core; the ring
// bell / mailbox helpers rely on it. Do not call Bringup::read32/write32 afterwards.

#ifndef FullmacPcie_hpp
#define FullmacPcie_hpp

#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include "PcieBringup.hpp"
#include "Erom.hpp"
#include "FullmacFwSelect.hpp"
#include "Nvram.hpp"
#include "Msgbuf.hpp"
#include "MsgbufEngine.hpp"
#include "WlcCodec.hpp"

namespace fullmac {

static constexpr uint32_t kMbIntFn0 = 0x0300, kMbIntD2hDb = 0xFF0000;

struct IoBus {
    IOPCIDevice *dev {nullptr};
    IOMemoryMap *regMap {nullptr}, *tcmMap {nullptr};
    volatile uint8_t *regs {nullptr}, *tcm {nullptr};
    size_t tcmSize {0};
    uint32_t window {0xFFFFFFFF};

    void setBar0Window(uint32_t a) {
        if (a == window) return;
        dev->configWrite32(pcie::kCfgBar0Window, a);
        window = a;
    }
    uint32_t regRead32(uint32_t off) { return *(volatile uint32_t *)(regs + off); }
    void regWrite32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(regs + off) = v; }
    uint32_t cfgRead32(uint32_t off) { return dev->configRead32(off); }
    void cfgWrite32(uint32_t off, uint32_t v) { dev->configWrite32(off, v); }
    void sleepUs(uint32_t us) { IODelay(us); }
    void sleepMs(uint32_t ms) { IOSleep(ms); }

    bool inTcm(uint32_t a, size_t n) const { return (size_t)a + n <= tcmSize; }
    uint32_t tcmRead32(uint32_t a) { return inTcm(a, 4) ? *(volatile uint32_t *)(tcm + a) : 0xFFFFFFFF; }
    uint16_t tcmRead16(uint32_t a) { return inTcm(a, 2) ? *(volatile uint16_t *)(tcm + a) : 0xFFFF; }
    void tcmWrite32(uint32_t a, uint32_t v) { if (inTcm(a, 4)) *(volatile uint32_t *)(tcm + a) = v; }
    void tcmWrite16(uint32_t a, uint16_t v) { if (inTcm(a, 2)) *(volatile uint16_t *)(tcm + a) = v; }
    void tcmWrite(uint32_t a, const void *p, size_t n) {
        if (!inTcm(a, n)) return;
        const uint8_t *s = (const uint8_t *)p;
        size_t i = 0;
        for (; i + 4 <= n; i += 4) { uint32_t w; memcpy(&w, s + i, 4); *(volatile uint32_t *)(tcm + a + i) = w; }
        for (; i < n; i++) *(volatile uint8_t *)(tcm + a + i) = s[i];
    }
    // For msgbuf::parseSharedInfo / readRingInfo.
    uint32_t read32(uint32_t a) { return tcmRead32(a); }
    uint16_t read16(uint32_t a) { return tcmRead16(a); }
};

struct HostDma {
    bool allocDma(size_t len, msgbuf::Dma *out) {
        IOBufferMemoryDescriptor *d = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
            kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous, len, 0xFFFFFFFFFFFFF000ULL);
        if (!d) return false;
        if (d->prepare() != kIOReturnSuccess) { d->release(); return false; }
        out->va = d->getBytesNoCopy();
        out->len = len;
        out->bus = d->getPhysicalSegment(0, nullptr);
        out->priv = d;
        memset(out->va, 0, len);
        return out->bus != 0;
    }
    void freeDma(msgbuf::Dma *d) {
        IOBufferMemoryDescriptor *m = (IOBufferMemoryDescriptor *)d->priv;
        if (m) { m->complete(); m->release(); }
        *d = msgbuf::Dma();
    }
};

class Pcie {
public:
    struct Config {
        const uint8_t *fw {nullptr};   size_t fwLen {0};
        const uint8_t *nvram {nullptr}; size_t nvLen {0};   // raw board .txt
        uint32_t chipId {pcie::kChipId43602};
        uint32_t chipRev {0};
    };
    typedef void (*EventCb)(void *ctx, const wlc::codec::EventMsg &ev);

    Pcie() : mBringup(mBus), mEngine(mHost) {}
    ~Pcie() { stop(); }

    // Returns 0 on success, else a step number (see log) for diagnosing on hardware.
    int start(IOPCIDevice *dev, const Config &cfg, EventCb cb, void *cbCtx) {
        mBus.dev = dev;
        dev->setMemoryEnable(true);
        dev->setBusMasterEnable(true);
        mBus.regMap = dev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
        mBus.tcmMap = dev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress2);
        if (!mBus.regMap || !mBus.tcmMap) { IOLog("OpenBCM fullmac: BAR map failed\n"); return 1; }
        mBus.regs = (volatile uint8_t *)mBus.regMap->getVirtualAddress();
        mBus.tcm = (volatile uint8_t *)mBus.tcmMap->getVirtualAddress();
        mBus.tcmSize = mBus.tcmMap->getLength();
        IOLog("OpenBCM fullmac: BAR0 %llu bytes, BAR2 %llu bytes\n",
              (unsigned long long)mBus.regMap->getLength(), (unsigned long long)mBus.tcmSize);

        // enumerate cores
        struct R { pcie::Bringup<IoBus> *b; uint32_t read32(uint32_t a) { return b->read32(a); } } rd{&mBringup};
        erom::Scanner<R> sc(rd);
        pcie::Core cores[erom::kMaxCores];
        unsigned n = sc.scan(cores, erom::kMaxCores);
        const pcie::Core *cc = erom::Scanner<R>::find(cores, n, pcie::kCoreChipCommon);
        const pcie::Core *cr4 = erom::Scanner<R>::find(cores, n, pcie::kCoreArmCr4);
        const pcie::Core *p2 = erom::Scanner<R>::find(cores, n, pcie::kCorePcie2);
        const pcie::Core *im = erom::Scanner<R>::find(cores, n, pcie::kCoreInternalMem);
        IOLog("OpenBCM fullmac: %u cores (cc=%d cr4=%d pcie2=%d imem=%d)\n", n, !!cc, !!cr4, !!p2, !!im);
        if (!cc || !cr4 || !p2 || !im) return 2;
        pcie::Core d11[4]; size_t nd = 0;
        for (unsigned i = 0; i < n && nd < 4; i++) if (cores[i].id == pcie::kCore80211) d11[nd++] = cores[i];

        mBringup.setPassive(*cr4, d11, nd);
        mBringup.resetDevice(*cc, *p2);
        mBringup.attachFixups(*p2);
        mBringup.setPassive(*cr4, d11, nd);

        pcie::Bringup<IoBus>::Geometry g;
        g.ramBase = fwsel::ramBase(cfg.chipId, cfg.chipRev);
        if (g.ramBase == 0xFFFFFFFF) { IOLog("OpenBCM fullmac: unknown RAM base for chip 0x%x\n", cfg.chipId); return 9; }
        g.ramSize = mBringup.cr4RamSize(*cr4);
        IOLog("OpenBCM fullmac: RAM base 0x%x size 0x%x\n", g.ramBase, g.ramSize);

        size_t cap = cfg.nvLen + 64;
        uint8_t *nv = (uint8_t *)IOMalloc(cap);
        if (!nv) return 3;
        nvram::Info ni;
        nvram::Result nr = nvram::strip(cfg.nvram, cfg.nvLen, nv, cap, &ni);
        if (nr != nvram::Result::kOk) { IOFree(nv, cap); IOLog("OpenBCM fullmac: nvram strip failed %d\n", (int)nr); return 4; }
        uint32_t shared = 0;
        pcie::Status st = mBringup.downloadAndStart(*cr4, *im, cfg.chipId, g, cfg.fw, cfg.fwLen,
                                                    nv, ni.length, &shared);
        IOFree(nv, cap);
        IOLog("OpenBCM fullmac: firmware start status %d shared=0x%x\n", (int)st, shared);
        if (st != pcie::Status::kOk) return 5;

        msgbuf::SharedInfo sh = msgbuf::parseSharedInfo(mBus, shared);
        if (!sh.valid) { IOLog("OpenBCM fullmac: bad shared info\n"); return 6; }
        msgbuf::DhiRingInfo ri = msgbuf::readRingInfo(mBus, sh.ringInfoAddr);
        msgbuf::RingPlan plan = msgbuf::planRings(sh, ri, /*allowHostIdx=*/false);   // TCM indices
        if (!plan.ok) return 7;

        mP2 = *p2;
        mEventCb = cb; mEventCtx = cbCtx;
        Msgbuf_RingInit(plan);
        msgbuf::Ring *rp[msgbuf::kNumCommon];
        for (int i = 0; i < msgbuf::kNumCommon; i++) rp[i] = &mRing[i];
        if (!mEngine.init(rp, sh.rxDataOffset, &Pcie::eventTrampoline, this)) return 8;

        mBringup.selectCore(*p2);                       // window stays here from now on
        mBringup.enableInterrupts(kMbIntFn0 | kMbIntD2hDb);
        mEngine.postControlBuffers();
        if (sh.hostRdyDb1) mBringup.hostReady();
        mUp = true;
        return 0;
    }

    void stop() {
        if (mUp) mBringup.disableInterrupts();
        mUp = false;
        mEngine.release();
        for (int i = 0; i < msgbuf::kNumCommon; i++) if (mRingDma[i].va) mHost.freeDma(&mRingDma[i]);
        if (mBus.regMap) { mBus.regMap->release(); mBus.regMap = nullptr; }
        if (mBus.tcmMap) { mBus.tcmMap->release(); mBus.tcmMap = nullptr; }
    }

    // Drain mailbox + control-complete ring. Call from a timer (or the ISR).
    void poll() {
        if (!mUp) return;
        uint32_t s = mBringup.mailboxInt();
        if (s == 0xFFFFFFFF) return;                      // device gone
        if (s) mBringup.ackMailboxInt(s);
        mEngine.processControlComplete();
    }

    // Synchronous ioctl (polling). Returns false on timeout/ring full; *status = firmware status.
    bool ioctl(uint32_t cmd, const void *in, uint16_t inLen, void *out, uint16_t outCap,
               int16_t *status = nullptr, uint32_t timeoutMs = 2000) {
        if (!mUp) return false;
        if (!mEngine.submitIoctl(0, cmd, in, inLen, outCap ? outCap : inLen)) return false;
        for (uint32_t t = 0; t < timeoutMs * 10; t++) {
            poll();
            if (mEngine.ioctlState() == msgbuf::IoctlState::kDone) {
                if (status) *status = mEngine.ioctlStatus();
                mEngine.takeIoctlResponse(out, outCap);
                return true;
            }
            IODelay(100);
        }
        return false;
    }

    bool isUp() const { return mUp; }
    msgbuf::Engine<HostDma> &engine() { return mEngine; }

private:
    struct RingCtx { Pcie *self; uint32_t wIdx, rIdx; };

    static void bell(void *c) { ((RingCtx *)c)->self->mBringup.ringBell0(); }
    static void updR(void *c) { RingCtx *r = (RingCtx *)c; msgbufRing(r)->rPtr = r->self->mBus.tcmRead16(r->rIdx); }
    static void updW(void *c) { RingCtx *r = (RingCtx *)c; msgbufRing(r)->wPtr = r->self->mBus.tcmRead16(r->wIdx); }
    static void wrR(void *c) { RingCtx *r = (RingCtx *)c; r->self->mBus.tcmWrite16(r->rIdx, msgbufRing(r)->rPtr); }
    static void wrW(void *c) { RingCtx *r = (RingCtx *)c; r->self->mBus.tcmWrite16(r->wIdx, msgbufRing(r)->wPtr); }
    static msgbuf::Ring *msgbufRing(RingCtx *r) { return &r->self->mRing[r - r->self->mCtx]; }

    static void eventTrampoline(void *ctx, const wlc::codec::EventMsg &ev) {
        Pcie *p = (Pcie *)ctx;
        if (p->mEventCb) p->mEventCb(p->mEventCtx, ev);
    }

    bool Msgbuf_RingInit(const msgbuf::RingPlan &plan) {
        for (int i = 0; i < msgbuf::kNumCommon; i++) {
            const msgbuf::RingDesc &d = plan.rings[i];
            size_t bytes = (size_t)d.maxItem * d.itemSize;
            if (!mHost.allocDma(bytes, &mRingDma[i])) return false;
            mBus.tcmWrite16(d.memAddr + msgbuf::kRingMaxItemOff, (uint16_t)d.maxItem);
            mBus.tcmWrite16(d.memAddr + msgbuf::kRingLenItemsOff, (uint16_t)d.itemSize);
            mBus.tcmWrite32(d.memAddr + msgbuf::kRingMemBaseAddrOff, (uint32_t)(mRingDma[i].bus & 0xffffffff));
            mBus.tcmWrite32(d.memAddr + msgbuf::kRingMemBaseAddrOff + 4, (uint32_t)(mRingDma[i].bus >> 32));
            mCtx[i] = RingCtx{this, d.wIdxAddr, d.rIdxAddr};
            mRing[i].registerCallbacks(&bell, &updR, &updW, &wrR, &wrW, &mCtx[i]);
            mRing[i].config((uint16_t)d.maxItem, (uint16_t)d.itemSize, mRingDma[i].va);
        }
        return true;
    }

    IoBus mBus;
    HostDma mHost;
    pcie::Bringup<IoBus> mBringup;
    msgbuf::Engine<HostDma> mEngine;
    pcie::Core mP2;
    msgbuf::Ring mRing[msgbuf::kNumCommon];
    msgbuf::Dma mRingDma[msgbuf::kNumCommon];
    RingCtx mCtx[msgbuf::kNumCommon];
    EventCb mEventCb {nullptr};
    void *mEventCtx {nullptr};
    bool mUp {false};
};

} // namespace fullmac

#endif /* FullmacPcie_hpp */
