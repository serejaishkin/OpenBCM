// MsgbufEngine.hpp — control plane of the PCIe msgbuf protocol: ioctl/iovar
// request + completion, posted response/event buffers, D2H control-complete
// dispatch. Pure C++ over a Host interface (DMA allocation), mirroring the
// control half of brcmfmac msgbuf.c. Data path (flowrings, rxpost, tx) is a
// separate step.
//
// Host contract:
//   bool allocDma(size_t len, msgbuf::Dma *out);   // coherent, zeroed, 64-bit bus addr
//   void freeDma(msgbuf::Dma *d);
//
// Threading: single-threaded (the kext runs it on the driver workloop).

#ifndef MsgbufEngine_hpp
#define MsgbufEngine_hpp

#include <stdint.h>
#include <string.h>
#include "Msgbuf.hpp"
#include "WlcCodec.hpp"

namespace msgbuf {

struct Dma {
    void    *va {nullptr};
    uint64_t bus {0};
    size_t   len {0};
};

enum class IoctlState : uint8_t { kIdle, kPending, kDone };

template <class Host>
class Engine {
public:
    typedef void (*EventSink)(void *ctx, const wlc::codec::EventMsg &ev);

    static constexpr unsigned kMaxCtlBufs = 32;              // outstanding posted ctl buffers
    static constexpr unsigned kMaxIoctlRespBufs = 8;
    static constexpr unsigned kMaxEventBufs = 8;

    Engine(Host &host) : mHost(host) {}
    ~Engine() { release(); }

    // rings[] indexed by CommonRing ids; `rxDataOffset` from SharedInfo.
    bool init(Ring *rings[kNumCommon], uint32_t rxDataOffset, EventSink sink, void *sinkCtx) {
        for (int i = 0; i < kNumCommon; i++) mRing[i] = rings[i];
        mRxDataOffset = rxDataOffset;
        mSink = sink;
        mSinkCtx = sinkCtx;
        if (!mHost.allocDma(kMaxCtlPktSize, &mReqBuf)) return false;      // ioctl request payload
        return true;
    }

    void release() {
        for (unsigned i = 0; i < kMaxCtlBufs; i++)
            if (mBuf[i].used) { mHost.freeDma(&mBuf[i].dma); mBuf[i].used = false; }
        if (mReqBuf.va) { mHost.freeDma(&mReqBuf); mReqBuf = Dma(); }
        mCurIoctlResp = mCurEvent = 0;
    }

    // Post response and event buffers to the firmware. Call after the rings are
    // configured and again whenever buffers are consumed (done internally).
    void postControlBuffers() {
        postBuffers(false, kMaxIoctlRespBufs - mCurIoctlResp);
        postBuffers(true, kMaxEventBufs - mCurEvent);
    }

    // ---- ioctl ------------------------------------------------------------------------
    // Submit one ioctl/iovar. `outLen` = size of the buffer the caller wants back.
    // Returns false when another ioctl is in flight or the ring is full.
    bool submitIoctl(uint8_t ifidx, uint32_t cmd, const void *in, uint16_t inLen, uint16_t outLen) {
        if (mIoctl == IoctlState::kPending) return false;
        if (inLen > kMaxCtlPktSize || outLen > kMaxCtlPktSize) return false;
        Ring *r = mRing[kH2dControlSubmit];
        void *slot = r->reserveForWrite();
        if (!slot) return false;
        mTransId++;
        if (in && inLen) memcpy(mReqBuf.va, in, inLen); else if (inLen) memset(mReqBuf.va, 0, inLen);
        uint16_t reqIn = inLen < outLen ? inLen : outLen;                 // brcmf: min(len, max) of one len
        (void)reqIn;
        buildIoctlReq((IoctlReqHdr *)slot, ifidx, cmd, mTransId, inLen, outLen, mReqBuf.bus);
        mIoctl = IoctlState::kPending;
        mIoctlRespLen = 0;
        mIoctlStatus = 0;
        r->writeComplete();
        return true;
    }

    IoctlState ioctlState() const { return mIoctl; }
    int16_t ioctlStatus() const { return mIoctlStatus; }
    uint16_t ioctlRespLen() const { return mIoctlRespLen; }
    // Copy the response out and return to idle. Returns bytes copied.
    uint16_t takeIoctlResponse(void *out, uint16_t cap) {
        uint16_t n = mIoctlRespLen < cap ? mIoctlRespLen : cap;
        if (out && n) memcpy(out, mIoctlResp, n);
        mIoctl = IoctlState::kIdle;
        return n;
    }

    // ---- D2H control complete ring ----------------------------------------------------------
    // Returns the number of messages processed.
    unsigned processControlComplete() {
        Ring *r = mRing[kD2hControlComplete];
        unsigned total = 0;
        for (;;) {
            uint16_t n = 0;
            const uint8_t *p = (const uint8_t *)r->getReadPtr(&n);
            if (!p) break;
            for (uint16_t i = 0; i < n; i++, p += r->itemLen()) {
                dispatch(p);
                total++;
            }
            r->readComplete(n);
        }
        return total;
    }

    // Counters useful for logging / tests.
    unsigned eventsDelivered {0}, ioctlCompletions {0}, unknownMsgs {0}, badBuffers {0};
    uint16_t lastFlowCreateStatus {0};

private:
    struct Slot { Dma dma; bool used {false}; bool isEvent {false}; };

    int allocSlot() {
        for (unsigned i = 0; i < kMaxCtlBufs; i++) if (!mBuf[i].used) return (int)i;
        return -1;
    }

    unsigned postBuffers(bool event, unsigned count) {
        Ring *r = mRing[kH2dControlSubmit];
        unsigned done = 0;
        for (unsigned i = 0; i < count; i++) {
            int idx = allocSlot();
            if (idx < 0) break;
            Slot &s = mBuf[idx];
            if (!mHost.allocDma(kMaxCtlPktSize, &s.dma)) break;
            void *slot = r->reserveForWrite();
            if (!slot) { mHost.freeDma(&s.dma); break; }
            s.used = true;
            s.isEvent = event;
            buildRespOrEventPost((RxIoctlRespOrEvent *)slot, event ? kEventBufPost : kIoctlRespBufPost,
                                 (uint32_t)idx, (uint16_t)s.dma.len, s.dma.bus);
            done++;
        }
        if (done) {
            r->writeComplete();
            if (event) mCurEvent += done; else mCurIoctlResp += done;
        }
        return done;
    }

    // Take ownership of the posted buffer behind `requestId`; nullptr if bogus.
    Slot *takeSlot(uint32_t requestId, bool event) {
        if (requestId >= kMaxCtlBufs || !mBuf[requestId].used || mBuf[requestId].isEvent != event) {
            badBuffers++;
            return nullptr;
        }
        return &mBuf[requestId];
    }

    void freeSlot(Slot *s) { mHost.freeDma(&s->dma); s->used = false; }

    void dispatch(const uint8_t *item) {
        const CommonHdr *h = (const CommonHdr *)item;
        switch (h->msgtype) {
        case kIoctlCmplt: {
            const IoctlRespHdr *m = (const IoctlRespHdr *)item;
            ioctlCompletions++;
            Slot *s = takeSlot(m->msg.requestId, false);
            mIoctlStatus = (int16_t)m->compl_.status;
            uint16_t len = m->respLen;
            if (s) {
                if (len > s->dma.len) len = (uint16_t)s->dma.len;
                if (len > sizeof(mIoctlResp)) len = sizeof(mIoctlResp);
                memcpy(mIoctlResp, s->dma.va, len);
                freeSlot(s);
            } else {
                len = 0;
            }
            mIoctlRespLen = len;
            mIoctl = IoctlState::kDone;
            if (mCurIoctlResp) mCurIoctlResp--;
            postBuffers(false, kMaxIoctlRespBufs - mCurIoctlResp);
            break;
        }
        case kWlEvent: {
            const RxEvent *m = (const RxEvent *)item;
            if (mCurEvent) mCurEvent--;
            Slot *s = takeSlot(m->msg.requestId, true);
            if (s) {
                uint16_t len = m->eventDataLen;
                size_t off = mRxDataOffset;
                if (off < s->dma.len && len <= s->dma.len - off) {
                    wlc::codec::EventMsg ev;
                    if (wlc::codec::parseEventFrame((const uint8_t *)s->dma.va + off, len, &ev)) {
                        eventsDelivered++;
                        if (mSink) mSink(mSinkCtx, ev);
                    } else {
                        badBuffers++;
                    }
                } else {
                    badBuffers++;
                }
                freeSlot(s);
            }
            postBuffers(true, kMaxEventBufs - mCurEvent);
            break;
        }
        case kIoctlPtrReqAck:
            break;                                           // request accepted, response follows
        case kFlowRingCreateCmplt: {
            const FlowringResp *m = (const FlowringResp *)item;
            lastFlowCreateStatus = m->compl_.status;
            break;
        }
        case kGenStatus: case kRingStatus: case kFlowRingDeleteCmplt:
        case kFlowRingFlushCmplt: case kTxStatus: case kRxCmplt:
            break;                                           // data path handles these (later step)
        default:
            unknownMsgs++;
            break;
        }
    }

    Host &mHost;
    Ring *mRing[kNumCommon] {};
    Slot mBuf[kMaxCtlBufs];
    Dma mReqBuf;
    uint32_t mRxDataOffset {0};
    EventSink mSink {nullptr};
    void *mSinkCtx {nullptr};
    unsigned mCurIoctlResp {0}, mCurEvent {0};
    uint16_t mTransId {0};
    IoctlState mIoctl {IoctlState::kIdle};
    int16_t mIoctlStatus {0};
    uint16_t mIoctlRespLen {0};
    uint8_t mIoctlResp[kMaxCtlPktSize];
};

} // namespace msgbuf

#endif /* MsgbufEngine_hpp */
