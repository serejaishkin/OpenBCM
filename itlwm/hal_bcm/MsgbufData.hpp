// MsgbufData.hpp — data plane of the PCIe msgbuf protocol: flowrings (one per
// ifidx+access category in STA mode), TX post/status, RX buffer posting and
// RX completion delivery. Pure C++ over a Host interface (DMA only).
// Port of the data half of brcmfmac msgbuf.c + flowring.c (STA/indirect mode).
//
// Frames are 802.3 (Ethernet) both ways. TX copies the frame into a DMA slot
// (no mbuf mapping), so callers can free their buffer immediately.
//
// Host contract: allocDma(len, Dma*) / freeDma(Dma*) as in MsgbufEngine.hpp.
// Single-threaded (driver workloop).

#ifndef MsgbufData_hpp
#define MsgbufData_hpp

#include <stdint.h>
#include <string.h>
#include "Msgbuf.hpp"
#include "MsgbufEngine.hpp"      // msgbuf::Dma

namespace msgbuf {

// 802.1d priority -> firmware FIFO (brcmf_flowring_prio2fifo): 0=BK 1=BE 2=VI 3=VO
static constexpr uint8_t kPrio2Fifo[8] = {1, 0, 0, 1, 2, 2, 3, 3};

template <class Host>
class DataPath {
public:
    static constexpr unsigned kTxSlots = 128;
    static constexpr unsigned kRxSlots = 128;
    static constexpr unsigned kSlotSize = kMaxPktSize;          // 2048
    static constexpr unsigned kMaxFlows = 16;                   // concurrent (ifidx,fifo) rings
    static constexpr unsigned kMaxPending = 32;                 // frames queued while a ring is created
    static constexpr unsigned kRxRepostThreshold = 32;

    typedef void (*RxSink)(void *ctx, uint8_t ifidx, const uint8_t *frame, uint16_t len, uint16_t flags);
    typedef void (*TxDoneSink)(void *ctx, bool ok);

    DataPath(Host &h) : mHost(h) {}
    ~DataPath() { release(); }

    // flowRings: Ring objects for flowring ids kFlowringIdStart.. (already wired to index slots).
    bool init(Ring *ctlSubmit, Ring *rxpost, Ring *txCompl, Ring *rxCompl,
              Ring **flowRings, unsigned nFlowRings, uint32_t rxDataOffset,
              uint16_t maxRxPost, RxSink rx, TxDoneSink txd, void *ctx) {
        mCtl = ctlSubmit; mRxPost = rxpost; mTxC = txCompl; mRxC = rxCompl;
        mFlowRings = flowRings; mNFlowRings = nFlowRings;
        mRxDataOffset = rxDataOffset;
        mMaxRxPost = maxRxPost < kRxSlots ? maxRxPost : kRxSlots;
        mRx = rx; mTxDone = txd; mCtx = ctx;
        if (!mHost.allocDma((size_t)kTxSlots * kSlotSize, &mTxPool)) return false;
        if (!mHost.allocDma((size_t)kRxSlots * kSlotSize, &mRxPool)) { release(); return false; }
        for (unsigned i = 0; i < kMaxFlows; i++) mFlow[i] = Flow();
        return true;
    }

    void release() {
        for (unsigned i = 0; i < kMaxFlows; i++)
            if (mFlow[i].ring.va) mHost.freeDma(&mFlow[i].ring);
        if (mTxPool.va) mHost.freeDma(&mTxPool);
        if (mRxPool.va) mHost.freeDma(&mRxPool);
        memset(mTxUsed, 0, sizeof(mTxUsed)); memset(mRxUsed, 0, sizeof(mRxUsed));
        mRxPosted = 0;
    }

    // ---- RX buffers ---------------------------------------------------------------
    void fillRx() {
        unsigned added = 0;
        while (mRxPosted < mMaxRxPost) {
            int slot = -1;
            for (unsigned i = 0; i < kRxSlots; i++) if (!mRxUsed[i]) { slot = (int)i; break; }
            if (slot < 0) break;
            void *it = mRxPost->reserveForWrite();
            if (!it) break;
            mRxUsed[slot] = true;
            buildRxBufPost((RxBufPost *)it, (uint32_t)slot, kSlotSize, mRxPool.bus + (uint64_t)slot * kSlotSize);
            mRxPosted++; added++;
        }
        if (added) mRxPost->writeComplete();
    }

    // ---- TX -----------------------------------------------------------------------
    // `frame` = full 802.3 frame (14-byte header + payload). Returns false when out of
    // slots / pending space (caller should drop or retry).
    bool transmit(uint8_t ifidx, const uint8_t *frame, uint16_t len, uint8_t prio) {
        if (len < 14 || len > kSlotSize) return false;
        uint8_t fifo = kPrio2Fifo[prio & 7];
        int fi = findFlow(ifidx, fifo);
        if (fi < 0) {
            fi = createFlow(ifidx, fifo, frame + 0 /*da*/, frame + 6 /*sa*/);
            if (fi < 0) return false;
        }
        int slot = allocTx();
        if (slot < 0) return false;
        memcpy((uint8_t *)mTxPool.va + (size_t)slot * kSlotSize, frame, len);
        mTxLen[slot] = len; mTxPrio[slot] = prio & 7; mTxIf[slot] = ifidx; mTxFlow[slot] = (uint8_t)fi;
        Flow &f = mFlow[fi];
        if (f.state == Flow::kOpen) {
            if (!postTx(f, slot)) { freeTx(slot); return false; }
            f.ringRef->writeComplete();
        } else {
            if (f.nPending >= kMaxPending) { freeTx(slot); return false; }
            f.pending[f.nPending++] = (uint8_t)slot;
        }
        return true;
    }

    // Flowring create completion (from the control-complete ring).
    void onFlowCreate(uint16_t wireFlowId, uint16_t status) {
        for (unsigned i = 0; i < kMaxFlows; i++) {
            Flow &f = mFlow[i];
            if (f.state != Flow::kCreating || f.wireId != wireFlowId) continue;
            if (status != 0) { dropFlow(f); return; }
            f.state = Flow::kOpen;
            for (unsigned k = 0; k < f.nPending; k++)
                if (!postTx(f, f.pending[k])) { freeTx(f.pending[k]); if (mTxDone) mTxDone(mCtx, false); }
            if (f.nPending) f.ringRef->writeComplete();
            f.nPending = 0;
            return;
        }
    }

    // ---- completion rings -----------------------------------------------------------
    unsigned processTxComplete() {
        unsigned total = 0;
        for (;;) {
            uint16_t n = 0;
            const uint8_t *p = (const uint8_t *)mTxC->getReadPtr(&n);
            if (!p) break;
            for (uint16_t i = 0; i < n; i++, p += mTxC->itemLen(), total++) {
                const TxStatus *m = (const TxStatus *)p;
                if (m->msg.msgtype != kTxStatus) continue;
                uint32_t idx = m->msg.requestId - 1;
                if (idx >= kTxSlots || !mTxUsed[idx]) continue;
                freeTx((int)idx);
                if (mTxDone) mTxDone(mCtx, true);
            }
            mTxC->readComplete(n);
        }
        return total;
    }

    unsigned processRxComplete() {
        unsigned total = 0;
        for (;;) {
            uint16_t n = 0;
            const uint8_t *p = (const uint8_t *)mRxC->getReadPtr(&n);
            if (!p) break;
            for (uint16_t i = 0; i < n; i++, p += mRxC->itemLen(), total++) {
                const RxComplete *m = (const RxComplete *)p;
                if (m->msg.msgtype != kRxCmplt) continue;
                uint32_t idx = m->msg.requestId;
                if (idx >= kRxSlots || !mRxUsed[idx]) { badRx++; continue; }
                mRxUsed[idx] = false;
                if (mRxPosted) mRxPosted--;
                uint16_t off = m->dataOffset ? m->dataOffset : (uint16_t)mRxDataOffset;
                uint16_t len = m->dataLen;
                if ((uint32_t)off + len > kSlotSize) { badRx++; continue; }
                if (mRx) mRx(mCtx, m->msg.ifidx, (const uint8_t *)mRxPool.va + (size_t)idx * kSlotSize + off,
                             len, m->flags);
                rxDelivered++;
            }
            mRxC->readComplete(n);
        }
        if (total && mRxPosted + kRxRepostThreshold <= mMaxRxPost) fillRx();
        return total;
    }

    unsigned rxDelivered {0}, badRx {0}, txDropped {0};

private:
    struct Flow {
        enum State : uint8_t { kFree, kCreating, kOpen } state {kFree};
        uint8_t ifidx {0}, fifo {0}, nPending {0};
        uint16_t wireId {0};
        unsigned ringIdx {0};
        Ring *ringRef {nullptr};
        Dma ring;
        uint8_t pending[kMaxPending] {};
    };

    int findFlow(uint8_t ifidx, uint8_t fifo) {
        for (unsigned i = 0; i < kMaxFlows; i++)
            if (mFlow[i].state != Flow::kFree && mFlow[i].ifidx == ifidx && mFlow[i].fifo == fifo) return (int)i;
        return -1;
    }

    int createFlow(uint8_t ifidx, uint8_t fifo, const uint8_t *da, const uint8_t *sa) {
        int fi = -1;
        for (unsigned i = 0; i < kMaxFlows; i++) if (mFlow[i].state == Flow::kFree) { fi = (int)i; break; }
        if (fi < 0) return -1;
        // pick a free hardware flowring index
        unsigned ri = 0;
        for (; ri < mNFlowRings; ri++) {
            bool taken = false;
            for (unsigned i = 0; i < kMaxFlows; i++)
                if (mFlow[i].state != Flow::kFree && mFlow[i].ringIdx == ri) { taken = true; break; }
            if (!taken) break;
        }
        if (ri >= mNFlowRings) return -1;
        Flow &f = mFlow[fi];
        if (!mHost.allocDma((size_t)kTxFlowringMaxItem * kTxFlowringItemSize, &f.ring)) return -1;
        void *it = mCtl->reserveForWrite();
        if (!it) { mHost.freeDma(&f.ring); return -1; }
        f.ifidx = ifidx; f.fifo = fifo; f.ringIdx = ri; f.nPending = 0;
        f.wireId = (uint16_t)(kFlowringIdStart + ri);
        f.ringRef = mFlowRings[ri];
        f.ringRef->config((uint16_t)kTxFlowringMaxItem, (uint16_t)kTxFlowringItemSize, f.ring.va);
        buildFlowringCreate((FlowringCreateReq *)it, ifidx, da, sa, fifo, f.wireId,
                            (uint16_t)kTxFlowringMaxItem, (uint16_t)kTxFlowringItemSize, f.ring.bus);
        f.state = Flow::kCreating;
        mCtl->writeComplete();
        return fi;
    }

    void dropFlow(Flow &f) {
        for (unsigned k = 0; k < f.nPending; k++) { freeTx(f.pending[k]); txDropped++; if (mTxDone) mTxDone(mCtx, false); }
        if (f.ring.va) mHost.freeDma(&f.ring);
        f = Flow();
    }

    bool postTx(Flow &f, int slot) {
        void *it = f.ringRef->reserveForWrite();
        if (!it) return false;
        const uint8_t *base = (const uint8_t *)mTxPool.va + (size_t)slot * kSlotSize;
        buildTxPost((TxMsgHdr *)it, f.ifidx, (uint32_t)slot + 1, base, mTxPrio[slot],
                    (uint16_t)(mTxLen[slot] - 14), mTxPool.bus + (uint64_t)slot * kSlotSize + 14);
        return true;
    }

    int allocTx() {
        for (unsigned i = 0; i < kTxSlots; i++) if (!mTxUsed[i]) { mTxUsed[i] = true; return (int)i; }
        return -1;
    }
    void freeTx(int i) { mTxUsed[i] = false; }

    Host &mHost;
    Ring *mCtl {nullptr}, *mRxPost {nullptr}, *mTxC {nullptr}, *mRxC {nullptr};
    Ring **mFlowRings {nullptr};
    unsigned mNFlowRings {0};
    uint32_t mRxDataOffset {0};
    uint16_t mMaxRxPost {0};
    unsigned mRxPosted {0};
    RxSink mRx {nullptr};
    TxDoneSink mTxDone {nullptr};
    void *mCtx {nullptr};
    Dma mTxPool, mRxPool;
    bool mTxUsed[kTxSlots] {}, mRxUsed[kRxSlots] {};
    uint16_t mTxLen[kTxSlots] {};
    uint8_t mTxPrio[kTxSlots] {}, mTxIf[kTxSlots] {}, mTxFlow[kTxSlots] {};
    Flow mFlow[kMaxFlows];
};

} // namespace msgbuf

#endif /* MsgbufData_hpp */
