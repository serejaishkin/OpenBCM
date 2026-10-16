// D11.hpp — pure-C++ host-side model of the Broadcom D11 SoftMAC `dma64`
// TX/RX descriptor rings, reverse-engineered from AirPortBrcmNIC
// (see D:/GitHub/BCM_94360cs/firmware/extracted_dlimage/D11_FINDINGS.md).
//
// This module contains NO IOKit and touches no hardware.  It is purely
// logic + index math, so it can be compiled and exercised on the host
// (g++/clang) before being wired into ItlBcm's real BAR1/MMIO callbacks.
//
// Design notes:
//   * The D11 core exposes one register window per DMA channel:
//       TX chan N = d11core + 0x200 + N*0x40   (N = 0..5)
//       RX chan 0 = d11core + 0x220            (paired with TX ch0)
//   * Per-channel register map (offset within the +0x40 window):
//       +0x00  DMA ctl:   bit0 DMA_EN, bit1 RESET (write 2 to reset)
//       +0x04  producer/PTR = ring-table bus addr + idx*16
//       +0x08/+0x0c  descriptor table bus addr low / high (+AE bits)
//       +0x10  consumer/DONE + reset status
//   * 16-byte descriptor: {ctl, addrlo, addrhi, resv}
//   * TX ctl bits: 0x80000000 SOP(first), 0x60000000 EOT/last (single desc
//     of a frame → 0xe0000000), 0x10000000 at slot count-1 (wrap marker),
//     0x40000 frag marker.
//   * RX ctl bits: 0x80000000 always, 0x10000000 at slot count-1.
//   * Ring wrap uses producer-consumer distance; consumer (HW) advances by
//     reading regbase+0x10 and computing ((hw - tableBus) >> 4) & (count-1).
//
// Callback split (all raw function pointers with `void *ctx` — no std::function,
// safe for kernel builds with -nostdinc):
//   * RegRead/RegWrite  — MMIO to the D11 channel window. `ctx` is caller-provided.
//   * DescRead/DescWrite — descriptor table memory (16 bytes/slot). `ctx` is caller-provided.
//   * pollRx / reclaimTx callbacks receive `cbCtx` — caller-provided.
// --------------------------------------------------------------------------

#ifndef D11_hpp
#define D11_hpp

#include <stdint.h>
#include <stddef.h>

namespace bcm {
namespace d11 {

// Per-channel register offsets (within the channel's 0x40 window).
static constexpr uint32_t kRegCtl    = 0x00; // DMA_EN bit0, RESET bit1
static constexpr uint32_t kRegProd   = 0x04; // producer / PTR
static constexpr uint32_t kRegBaseLo = 0x08; // desc table bus addr low
static constexpr uint32_t kRegBaseHi = 0x0c; // desc table bus addr high
static constexpr uint32_t kRegDone   = 0x10; // consumer / DONE + reset status

// DMA ctl bits.
static constexpr uint32_t kCtlDmaEn  = 0x00000001u;
static constexpr uint32_t kCtlReset  = 0x00000002u;

// TX descriptor ctl bits.
static constexpr uint32_t kTxCtlFirst   = 0x80000000u; // SOP
static constexpr uint32_t kTxCtlLast    = 0x60000000u; // EOT/last of chain
static constexpr uint32_t kTxCtlSingle  = kTxCtlFirst | kTxCtlLast; // 0xe0000000
static constexpr uint32_t kTxCtlWrap    = 0x10000000u; // slot == count-1
static constexpr uint32_t kTxCtlFrag    = 0x00040000u;

// RX descriptor ctl bits.
static constexpr uint32_t kRxCtlOwn   = 0x80000000u; // always set by host
static constexpr uint32_t kRxCtlWrap  = 0x10000000u; // slot == count-1

// Callback types with void *ctx (caller-provided context, typically `this`).
using RegRead  = uint32_t (*)(void *ctx, uint32_t offset);          // offset within channel window
using RegWrite = void (*)(void *ctx, uint32_t offset, uint32_t value);
using DescWrite = void (*)(void *ctx, uint32_t idx, uint32_t ctl, uint32_t len, uint64_t addr);
using DescReadCtl = uint32_t (*)(void *ctx, uint32_t idx);
using DescReadLen = uint32_t (*)(void *ctx, uint32_t idx);
using DescReadAddr = uint64_t (*)(void *ctx, uint32_t idx);

// Result of an RX poll: a completed frame ready to be handed to net80211
// via ieee80211_inputm.
struct RxFrame {
    uint64_t addr;   // bus address of the buffer holding the frame
    uint32_t len;    // fill length written by firmware at desc +0x04
};

// Result of a TX reclaim poll: bus address of a completed descriptor.
struct TxDone {
    uint64_t addr;
    uint32_t len;
};

// ---------------- RX ring ----------------

class RxRing {
public:
    RxRing(uint16_t count,
           void *ctx,
           RegRead rd, RegWrite wr,
           DescWrite descWrite,
           DescReadCtl descReadCtl,
           DescReadLen descReadLen,
           DescReadAddr descReadAddr)
        : count_(count), ctx_(ctx), rd_(rd), wr_(wr),
          descWrite_(descWrite),
          descReadCtl_(descReadCtl),
          descReadLen_(descReadLen),
          descReadAddr_(descReadAddr) {
        prod_ = 0;
        cons_ = 0;
        posted_ = 0;
    }

    // Reset the RX channel: write RESET, poll DONE until reset clears.
    bool reset() {
        wr_(ctx_, kRegCtl, kCtlReset);
        for (int i = 0; i < 10000; i++) {
            uint32_t v = rd_(ctx_, kRegDone);
            if ((v & 0xf0000000u) == 0x30000000u) {
                cons_ = 0; prod_ = 0; posted_ = 0;
                return true;
            }
        }
        return false;
    }

    // Program the descriptor table base address and enable the channel.
    bool init(uint64_t tableBusAddr, uint32_t addrExtCtlBits) {
        wr_(ctx_, kRegBaseLo, (uint32_t)(tableBusAddr & 0xffffffffu));
        wr_(ctx_, kRegBaseHi, (uint32_t)(tableBusAddr >> 32));
        uint32_t ctl = rd_(ctx_, kRegCtl);
        ctl &= 0xfffcffffu;
        ctl |= (addrExtCtlBits & 0x3u) << 16;
        ctl |= kCtlDmaEn;
        wr_(ctx_, kRegCtl, ctl);
        tableBusAddr_ = tableBusAddr;
        cons_ = 0; prod_ = 0; posted_ = 0;
        return true;
    }

    // Set the table bus address used to compute producer reg writes.
    void setTableBusAddr(uint64_t base) { tableBusAddr_ = base; }

    // Post one RX buffer descriptor for the next producer slot.
    // Returns false if the ring is full.
    bool postBuffer(uint64_t addr, uint32_t bufSize) {
        if (posted_ - cons_ >= count_) return false;
        uint32_t idx = (uint32_t)prod_ & (uint32_t)(count_ - 1);
        uint32_t ctl = kRxCtlOwn;
        if (idx == (uint32_t)(count_ - 1)) ctl |= kRxCtlWrap;
        descWrite_(ctx_, idx, ctl, bufSize, addr);
        // producer reg = tableBusAddr + slot*16
        wr_(ctx_, kRegProd, (uint32_t)(tableBusAddr_ + (uint64_t)(prod_) * 16));
        prod_++;
        posted_++;
        return true;
    }

    // Poll for completed RX frames.  Reads HW consumer at kRegDone, walks
    // descriptors our consumer hasn't reclaimed yet, and invokes `cb` per
    // completed frame.  Each completed buffer is re-posted automatically.
    void pollRx(void *cbCtx, void (*cb)(void *cbCtx, const RxFrame &)) {
        uint32_t hwDone = rd_(ctx_, kRegDone);
        uint64_t hwIdx = ((hwDone - (uint32_t)tableBusAddr_) >> 4) & (uint32_t)(count_ - 1);
        while (cons_ != hwIdx) {
            uint32_t idx = (uint32_t)cons_ & (uint32_t)(count_ - 1);
            uint32_t ctl = descReadCtl_(ctx_, idx);
            if (ctl & kRxCtlOwn) break; // HW still owns it
            RxFrame f;
            f.addr = descReadAddr_(ctx_, idx);
            f.len  = descReadLen_(ctx_, idx);
            cb(cbCtx, f);
            // re-arm this buffer (cons_ must be incremented first so
            // postBuffer's fullness check sees the freed slot).
            cons_++;
            postBuffer(f.addr, f.len);
        }
    }

private:
    uint16_t count_;
    void    *ctx_;
    RegRead rd_;
    RegWrite wr_;
    DescWrite descWrite_;
    DescReadCtl descReadCtl_;
    DescReadLen descReadLen_;
    DescReadAddr descReadAddr_;
    uint64_t tableBusAddr_ {0};
    uint32_t prod_;
    uint32_t cons_;
    uint32_t posted_;
};

// ---------------- TX ring ----------------

class TxRing {
public:
    TxRing(uint16_t count,
           void *ctx,
           RegRead rd, RegWrite wr,
           DescWrite descWrite,
           DescReadCtl descReadCtl,
           DescReadLen descReadLen,
           DescReadAddr descReadAddr)
        : count_(count), ctx_(ctx), rd_(rd), wr_(wr),
          descWrite_(descWrite),
          descReadCtl_(descReadCtl),
          descReadLen_(descReadLen),
          descReadAddr_(descReadAddr) {
        prod_ = 0;
        cons_ = 0;
    }

    bool reset() {
        wr_(ctx_, kRegCtl, kCtlReset);
        for (int i = 0; i < 10000; i++) {
            uint32_t v = rd_(ctx_, kRegDone);
            if ((v & 0xf0000000u) == 0x30000000u) {
                cons_ = 0; prod_ = 0;
                return true;
            }
        }
        return false;
    }

    bool init(uint64_t tableBusAddr, uint32_t addrExtCtlBits) {
        wr_(ctx_, kRegBaseLo, (uint32_t)(tableBusAddr & 0xffffffffu));
        wr_(ctx_, kRegBaseHi, (uint32_t)(tableBusAddr >> 32));
        uint32_t ctl = rd_(ctx_, kRegCtl);
        ctl &= 0xfffcffffu;
        ctl |= (addrExtCtlBits & 0x3u) << 16;
        // leave DMA_EN off until the first frame is enqueued
        wr_(ctx_, kRegCtl, ctl);
        tableBusAddr_ = tableBusAddr;
        prod_ = 0; cons_ = 0;
        return true;
    }

    void setTableBusAddr(uint64_t base) { tableBusAddr_ = base; }

    // Enqueue one TX frame (single-descriptor).  Returns false if full.
    bool enqueue(uint64_t addr, uint32_t len) {
        if (prod_ - cons_ >= count_) return false;
        uint32_t idx = (uint32_t)prod_ & (uint32_t)(count_ - 1);
        uint32_t ctl = kTxCtlSingle;
        if (idx == (uint32_t)(count_ - 1)) ctl |= kTxCtlWrap;
        descWrite_(ctx_, idx, ctl, len, addr);
        prod_++;
        wr_(ctx_, kRegProd, (uint32_t)(tableBusAddr_ + (uint64_t)(prod_ - 1) * 16));
        uint32_t c = rd_(ctx_, kRegCtl);
        c |= kCtlDmaEn;
        wr_(ctx_, kRegCtl, c);
        return true;
    }

    // Reclaim completed TX descriptors.
    void reclaimTx(void *cbCtx, void (*cb)(void *cbCtx, const TxDone &)) {
        uint32_t hwDone = rd_(ctx_, kRegDone);
        uint64_t hwIdx = ((hwDone - (uint32_t)tableBusAddr_) >> 4) & (uint32_t)(count_ - 1);
        while (cons_ != hwIdx) {
            uint32_t idx = (uint32_t)cons_ & (uint32_t)(count_ - 1);
            uint32_t ctl = descReadCtl_(ctx_, idx);
            if (ctl & kTxCtlFirst) break;
            TxDone d{descReadAddr_(ctx_, idx), descReadLen_(ctx_, idx)};
            cb(cbCtx, d);
            cons_++;
        }
    }

private:
    uint16_t count_;
    void    *ctx_;
    RegRead rd_;
    RegWrite wr_;
    DescWrite descWrite_;
    DescReadCtl descReadCtl_;
    DescReadLen descReadLen_;
    DescReadAddr descReadAddr_;
    uint64_t tableBusAddr_ {0};
    uint32_t prod_;
    uint32_t cons_;
};

} // namespace d11
} // namespace bcm

#endif /* D11_hpp */
