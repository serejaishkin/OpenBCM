// Msgbuf.hpp — PCIe "msgbuf" protocol of Broadcom FullMAC firmware, pure C++.
//
// Source of truth: Linux brcmfmac (pcie.c, msgbuf.c, msgbuf.h, commonring.c,
// bus.h). Nothing here touches IOKit or hardware: shared-RAM access goes
// through a Reader/Writer template parameter, ring memory is a plain pointer.
// That makes it testable on the host against a simulated device
// (tests/host/test_msgbuf.cpp). The IOKit glue (BAR2 TCM window, DMA buffers,
// doorbells, interrupts) belongs to a future MsgbufTransport.
//
// Applies to chips running a FullMAC firmware (brcmfmac43602-pcie.bin & co),
// NOT to the Apple dlimage/olmsg path.

#ifndef Msgbuf_hpp
#define Msgbuf_hpp

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace msgbuf {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
enum MsgType : uint8_t {
    kGenStatus = 0x1, kRingStatus = 0x2, kFlowRingCreate = 0x3, kFlowRingCreateCmplt = 0x4,
    kFlowRingDelete = 0x5, kFlowRingDeleteCmplt = 0x6, kFlowRingFlush = 0x7,
    kFlowRingFlushCmplt = 0x8, kIoctlPtrReq = 0x9, kIoctlPtrReqAck = 0xA,
    kIoctlRespBufPost = 0xB, kIoctlCmplt = 0xC, kEventBufPost = 0xD, kWlEvent = 0xE,
    kTxPost = 0xF, kTxStatus = 0x10, kRxBufPost = 0x11, kRxCmplt = 0x12,
};

static constexpr uint32_t kIoctlReqPktId = 0xFFFE;
static constexpr uint16_t kMaxPktSize = 2048;
static constexpr uint16_t kMaxCtlPktSize = 8192;
static constexpr uint8_t  kPktFlagsFrame8023 = 0x01, kPktFlagsFrame80211 = 0x02,
                          kPktFlagsFrameMask = 0x07, kPktFlagsPrioShift = 5;

// Common rings (index == ring id for the common ones).
enum CommonRing : uint8_t {
    kH2dControlSubmit = 0, kH2dRxpostSubmit = 1,
    kD2hControlComplete = 2, kD2hTxComplete = 3, kD2hRxComplete = 4,
    kNumH2dCommon = 2, kNumD2hCommon = 3, kNumCommon = 5,
};
static constexpr uint16_t kFlowringIdStart = 2;     // first H2D flowring id

static constexpr uint32_t kRingMaxItem[kNumCommon] = {64, 1024, 64, 1024, 1024};
static constexpr uint32_t kRingItemSizePreV7[kNumCommon] = {40, 32, 24, 16, 32};
static constexpr uint32_t kRingItemSizeV7[kNumCommon]    = {40, 32, 24, 24, 40};
static constexpr uint32_t kTxFlowringMaxItem = 512, kTxFlowringItemSize = 48;

// Shared RAM (offsets inside the firmware's shared struct).
static constexpr uint32_t kSharedVersionMask = 0x00FF;
static constexpr uint32_t kSharedDmaIndex = 0x10000, kSharedDma2BIdx = 0x100000,
                          kSharedHostRdyDb1 = 0x10000000;
static constexpr uint8_t  kMinSharedVersion = 5, kMaxSharedVersion = 7;
static constexpr uint32_t kShConsoleAddr = 20, kShMaxRxbufpost = 34, kShRxDataOffset = 36,
                          kShHtodMbData = 40, kShDtohMbData = 44, kShRingInfoAddr = 48;
static constexpr uint16_t kDefMaxRxbufpost = 255;
// Ring memory descriptor inside TCM (16 bytes per ring).
static constexpr uint32_t kRingMemSz = 16, kRingMaxItemOff = 4, kRingLenItemsOff = 6,
                          kRingMemBaseAddrOff = 8;

// ---------------------------------------------------------------------------
// Wire structures (little-endian, natural alignment == packed here).
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct BufAddr { uint32_t low, high; };
struct CommonHdr { uint8_t msgtype, ifidx, flags, rsvd0; uint32_t requestId; };
struct CompletionHdr { uint16_t status, flowRingId; };

struct IoctlReqHdr {
    CommonHdr msg; uint32_t cmd; uint16_t transId, inputBufLen, outputBufLen, rsvd0[3];
    BufAddr reqBufAddr; uint32_t rsvd1[2];
};
struct TxMsgHdr {
    CommonHdr msg; uint8_t txhdr[14]; uint8_t flags, segCnt;
    BufAddr metadataBufAddr, dataBufAddr; uint16_t metadataBufLen, dataLen; uint32_t rsvd0;
};
struct RxBufPost {
    CommonHdr msg; uint16_t metadataBufLen, dataBufLen; uint32_t rsvd0;
    BufAddr metadataBufAddr, dataBufAddr;
};
struct RxIoctlRespOrEvent {
    CommonHdr msg; uint16_t hostBufLen, rsvd0[3]; BufAddr hostBufAddr; uint32_t rsvd1[4];
};
struct GenStatus { CommonHdr msg; CompletionHdr compl_; uint16_t writeIdx; uint32_t rsvd0[3]; };
struct RingStatus { CommonHdr msg; CompletionHdr compl_; uint16_t writeIdx, rsvd0[5]; };
struct RxEvent { CommonHdr msg; CompletionHdr compl_; uint16_t eventDataLen, seqnum, rsvd0[4]; };
struct IoctlRespHdr {
    CommonHdr msg; CompletionHdr compl_; uint16_t respLen, transId; uint32_t cmd, rsvd0;
};
struct TxStatus { CommonHdr msg; CompletionHdr compl_; uint16_t metadataLen, txStatus; };
struct RxComplete {
    CommonHdr msg; CompletionHdr compl_; uint16_t metadataLen, dataLen, dataOffset, flags;
    uint32_t rxStatus0, rxStatus1, rsvd0;
};
struct FlowringCreateReq {
    CommonHdr msg; uint8_t da[6], sa[6]; uint8_t tid, ifFlags; uint16_t flowRingId;
    uint8_t tc, priority; uint16_t intVector, maxItems, lenItem; BufAddr flowRingAddr;
};
struct FlowringDeleteReq { CommonHdr msg; uint16_t flowRingId, reason; uint32_t rsvd0[7]; };
struct FlowringResp { CommonHdr msg; CompletionHdr compl_; uint32_t rsvd0[3]; };

// brcmf_pcie_dhi_ringinfo, read from TCM at shared.ring_info_addr.
struct DhiRingInfo {
    uint32_t ringmem, h2dWIdxPtr, h2dRIdxPtr, d2hWIdxPtr, d2hRIdxPtr;
    BufAddr h2dWIdxHost, h2dRIdxHost, d2hWIdxHost, d2hRIdxHost;
    uint16_t maxFlowrings, maxSubmissionrings, maxCompletionrings;
};
#pragma pack(pop)

static_assert(sizeof(CommonHdr) == 8, "CommonHdr");
static_assert(sizeof(IoctlReqHdr) == 40, "IoctlReqHdr == control submit item size");
static_assert(sizeof(TxMsgHdr) == 48, "TxMsgHdr == flowring item size");
static_assert(sizeof(RxBufPost) == 32, "RxBufPost == rxpost item size");
static_assert(sizeof(RxIoctlRespOrEvent) == 40, "RxIoctlRespOrEvent");
static_assert(sizeof(IoctlRespHdr) == 24, "IoctlRespHdr == control complete item size");
static_assert(sizeof(TxStatus) == 16, "TxStatus");
static_assert(sizeof(RxComplete) == 32, "RxComplete");
static_assert(sizeof(FlowringCreateReq) == 40, "FlowringCreateReq");
static_assert(sizeof(FlowringDeleteReq) == 40, "FlowringDeleteReq");
static_assert(sizeof(FlowringResp) == 24, "FlowringResp");
static_assert(sizeof(DhiRingInfo) == 58, "DhiRingInfo");

// ---------------------------------------------------------------------------
// Request builders (caller supplies the ring slot).
// ---------------------------------------------------------------------------
inline void buildIoctlReq(IoctlReqHdr *r, uint8_t ifidx, uint32_t cmd, uint16_t transId,
                          uint16_t inLen, uint16_t outLen, uint64_t reqBufBus) {
    memset(r, 0, sizeof(*r));
    r->msg.msgtype = kIoctlPtrReq;
    r->msg.ifidx = ifidx;
    r->msg.requestId = kIoctlReqPktId;
    r->cmd = cmd;
    r->transId = transId;
    r->inputBufLen = inLen;
    r->outputBufLen = outLen;
    r->reqBufAddr.low = (uint32_t)reqBufBus;
    r->reqBufAddr.high = (uint32_t)(reqBufBus >> 32);
}

// Post a host buffer for ioctl responses (type kIoctlRespBufPost) or firmware
// events (kEventBufPost). `reqId` is echoed back in the completion.
inline void buildRespOrEventPost(RxIoctlRespOrEvent *r, uint8_t msgtype, uint32_t reqId,
                                 uint16_t bufLen, uint64_t bus) {
    memset(r, 0, sizeof(*r));
    r->msg.msgtype = msgtype;
    r->msg.requestId = reqId;
    r->hostBufLen = bufLen;
    r->hostBufAddr.low = (uint32_t)bus;
    r->hostBufAddr.high = (uint32_t)(bus >> 32);
}

inline void buildRxBufPost(RxBufPost *r, uint32_t reqId, uint16_t dataLen, uint64_t bus) {
    memset(r, 0, sizeof(*r));
    r->msg.msgtype = kRxBufPost;
    r->msg.requestId = reqId;
    r->dataBufLen = dataLen;
    r->dataBufAddr.low = (uint32_t)bus;
    r->dataBufAddr.high = (uint32_t)(bus >> 32);
}

// TX post of an 802.3 frame: the first 14 bytes (Ethernet header) go inline in
// `txhdr`, the remainder is DMA'd from `dataBus` (frame payload without eth hdr).
inline void buildTxPost(TxMsgHdr *r, uint8_t ifidx, uint32_t pktId, const uint8_t *ethHdr14,
                        uint8_t prio, uint16_t payloadLen, uint64_t payloadBus) {
    memset(r, 0, sizeof(*r));
    r->msg.msgtype = kTxPost;
    r->msg.ifidx = ifidx;
    r->msg.requestId = pktId;
    memcpy(r->txhdr, ethHdr14, 14);
    r->flags = (uint8_t)(kPktFlagsFrame8023 | ((prio & 7) << kPktFlagsPrioShift));
    r->segCnt = 1;
    r->dataLen = payloadLen;
    r->dataBufAddr.low = (uint32_t)payloadBus;
    r->dataBufAddr.high = (uint32_t)(payloadBus >> 32);
}

inline void buildFlowringCreate(FlowringCreateReq *r, uint8_t ifidx, const uint8_t *da,
                                const uint8_t *sa, uint8_t tid, uint16_t flowId,
                                uint16_t maxItems, uint16_t lenItem, uint64_t ringBus) {
    memset(r, 0, sizeof(*r));
    r->msg.msgtype = kFlowRingCreate;
    r->msg.ifidx = ifidx;
    memcpy(r->da, da, 6);
    memcpy(r->sa, sa, 6);
    r->tid = tid;
    r->flowRingId = flowId;
    r->maxItems = maxItems;
    r->lenItem = lenItem;
    r->flowRingAddr.low = (uint32_t)ringBus;
    r->flowRingAddr.high = (uint32_t)(ringBus >> 32);
}

// ---------------------------------------------------------------------------
// CommonRing — direct port of brcmf_commonring (single producer / consumer
// per ring; locking is the caller's job). Index sync with the device goes
// through callbacks, so the same class serves TCM indices and host DMA indices.
// ---------------------------------------------------------------------------
class Ring {
public:
    typedef void (*Hook)(void *ctx);
    void registerCallbacks(Hook ringBell, Hook updateRptr, Hook updateWptr,
                           Hook writeRptr, Hook writeWptr, void *ctx) {
        mBell = ringBell; mUpdR = updateRptr; mUpdW = updateWptr;
        mWrR = writeRptr; mWrW = writeWptr; mCtx = ctx;
    }
    void config(uint16_t depth, uint16_t itemLen, void *buf) {
        mDepth = depth; mItemLen = itemLen; mBuf = (uint8_t *)buf;
        rPtr = 0; if (mWrR) mWrR(mCtx);
        wPtr = 0; if (mWrW) mWrW(mCtx);
        mFPtr = 0; mWasFull = false;
    }

    uint16_t depth() const { return mDepth; }
    uint16_t itemLen() const { return mItemLen; }

    // Device-visible indices. Transport callbacks read/write these.
    uint16_t rPtr {0}, wPtr {0};

    bool writeAvailable() {
        bool retry = true;
        for (;;) {
            uint16_t avail = (rPtr <= wPtr) ? (uint16_t)(mDepth - wPtr + rPtr) : (uint16_t)(rPtr - wPtr);
            if (avail > 1) {
                if (!mWasFull) return true;
                if (avail > mDepth / 8) { mWasFull = false; return true; }
                if (!retry) return false;
            } else if (!retry) {
                mWasFull = true;
                return false;
            }
            if (mUpdR) mUpdR(mCtx);
            retry = false;
        }
    }

    // Reserve one slot; nullptr when the ring is full (one slot always stays free).
    void *reserveForWrite() {
        bool retry = true;
        for (;;) {
            uint16_t avail = (rPtr <= wPtr) ? (uint16_t)(mDepth - wPtr + rPtr) : (uint16_t)(rPtr - wPtr);
            if (avail > 1) {
                void *p = mBuf + (size_t)wPtr * mItemLen;
                if (++wPtr == mDepth) wPtr = 0;
                return p;
            }
            if (!retry) { mWasFull = true; return nullptr; }
            if (mUpdR) mUpdR(mCtx);
            retry = false;
        }
    }

    // Reserve up to n contiguous slots (never wraps within one reservation).
    void *reserveForWriteMultiple(uint16_t n, uint16_t *alloced) {
        bool retry = true;
        for (;;) {
            uint16_t avail = (rPtr <= wPtr) ? (uint16_t)(mDepth - wPtr + rPtr) : (uint16_t)(rPtr - wPtr);
            if (avail > 1) {
                void *p = mBuf + (size_t)wPtr * mItemLen;
                uint16_t a = n < (uint16_t)(avail - 1) ? n : (uint16_t)(avail - 1);
                if ((uint32_t)a + wPtr > mDepth) a = (uint16_t)(mDepth - wPtr);
                wPtr = (uint16_t)(wPtr + a);
                if (wPtr == mDepth) wPtr = 0;
                *alloced = a;
                return p;
            }
            if (!retry) { mWasFull = true; return nullptr; }
            if (mUpdR) mUpdR(mCtx);
            retry = false;
        }
    }

    // Publish the write pointer and ring the doorbell.
    void writeComplete() {
        mFPtr = wPtr;
        if (mWrW) mWrW(mCtx);
        if (mBell) mBell(mCtx);
    }

    void writeCancel(uint16_t n) { wPtr = (wPtr == 0) ? (uint16_t)(mDepth - n) : (uint16_t)(wPtr - n); }

    // Contiguous run of readable items (does not wrap). nullptr when empty.
    const void *getReadPtr(uint16_t *nItems) {
        if (mUpdW) mUpdW(mCtx);
        *nItems = (wPtr >= rPtr) ? (uint16_t)(wPtr - rPtr) : (uint16_t)(mDepth - rPtr);
        if (*nItems == 0) return nullptr;
        return mBuf + (size_t)rPtr * mItemLen;
    }

    void readComplete(uint16_t n) {
        rPtr = (uint16_t)(rPtr + n);
        if (rPtr == mDepth) rPtr = 0;
        if (mWrR) mWrR(mCtx);
    }

private:
    uint16_t mDepth {0}, mItemLen {0}, mFPtr {0};
    uint8_t *mBuf {nullptr};
    bool mWasFull {false};
    Hook mBell {nullptr}, mUpdR {nullptr}, mUpdW {nullptr}, mWrR {nullptr}, mWrW {nullptr};
    void *mCtx {nullptr};
};

// ---------------------------------------------------------------------------
// Shared RAM discovery + ring planning
// ---------------------------------------------------------------------------
struct SharedInfo {
    bool     valid {false};
    uint32_t addr {0};
    uint32_t flags {0};
    uint8_t  version {0};
    uint8_t  dmaIdxSize {0};          // 0 = indices live in TCM, 2 or 4 = host DMA indices
    bool     hostRdyDb1 {false};
    uint16_t maxRxbufpost {0};
    uint32_t rxDataOffset {0};
    uint32_t htodMbDataAddr {0}, dtohMbDataAddr {0};
    uint32_t ringInfoAddr {0};
    uint32_t consoleAddr {0};
};

// Dev must provide: uint32_t read32(uint32_t tcmAddr); uint16_t read16(uint32_t tcmAddr);
template <class Dev>
inline SharedInfo parseSharedInfo(Dev &dev, uint32_t sharedAddr) {
    SharedInfo s;
    s.addr = sharedAddr;
    s.flags = dev.read32(sharedAddr);
    s.version = (uint8_t)(s.flags & kSharedVersionMask);
    if (s.version < kMinSharedVersion || s.version > kMaxSharedVersion) return s;   // invalid
    if (s.flags & kSharedDmaIndex)
        s.dmaIdxSize = (s.flags & kSharedDma2BIdx) ? 2 : 4;
    s.hostRdyDb1 = (s.flags & kSharedHostRdyDb1) != 0;
    s.maxRxbufpost = dev.read16(sharedAddr + kShMaxRxbufpost);
    if (s.maxRxbufpost == 0) s.maxRxbufpost = kDefMaxRxbufpost;
    s.rxDataOffset = dev.read32(sharedAddr + kShRxDataOffset);
    s.htodMbDataAddr = dev.read32(sharedAddr + kShHtodMbData);
    s.dtohMbDataAddr = dev.read32(sharedAddr + kShDtohMbData);
    s.ringInfoAddr = dev.read32(sharedAddr + kShRingInfoAddr);
    s.consoleAddr = dev.read32(sharedAddr + kShConsoleAddr);
    s.valid = true;
    return s;
}

template <class Dev>
inline DhiRingInfo readRingInfo(Dev &dev, uint32_t ringInfoAddr) {
    DhiRingInfo ri;
    memset(&ri, 0, sizeof(ri));
    ri.ringmem = dev.read32(ringInfoAddr + 0);
    ri.h2dWIdxPtr = dev.read32(ringInfoAddr + 4);
    ri.h2dRIdxPtr = dev.read32(ringInfoAddr + 8);
    ri.d2hWIdxPtr = dev.read32(ringInfoAddr + 12);
    ri.d2hRIdxPtr = dev.read32(ringInfoAddr + 16);
    ri.maxFlowrings = dev.read16(ringInfoAddr + 52);
    ri.maxSubmissionrings = dev.read16(ringInfoAddr + 54);
    ri.maxCompletionrings = dev.read16(ringInfoAddr + 56);
    return ri;
}

// Result of laying out the five common rings.
struct RingDesc {
    uint8_t  id;
    uint32_t maxItem, itemSize;
    uint32_t memAddr;       // TCM address of this ring's 16-byte descriptor
    uint32_t wIdxAddr, rIdxAddr;   // TCM address, or offset into the host index buffer
};

struct RingPlan {
    bool     ok {false};
    uint16_t maxFlowrings {0}, maxSubmissionrings {0}, maxCompletionrings {0};
    uint8_t  idxStride {0};          // bytes per index slot
    bool     hostIndices {false};
    // Host index buffer layout (valid when hostIndices): byte offsets + total size.
    uint32_t hostH2dW {0}, hostH2dR {0}, hostD2hW {0}, hostD2hR {0}, hostBufSize {0};
    RingDesc rings[kNumCommon];
};

// Port of brcmf_pcie_init_ringbuffers' index/ring-memory arithmetic.
// `allowHostIdx` lets the caller refuse DMA indices (e.g. no buffer available).
inline RingPlan planRings(const SharedInfo &sh, const DhiRingInfo &ri, bool allowHostIdx) {
    RingPlan p;
    if (!sh.valid) return p;
    if (sh.version >= 6) {
        p.maxSubmissionrings = ri.maxSubmissionrings;
        p.maxFlowrings = ri.maxFlowrings;
        p.maxCompletionrings = ri.maxCompletionrings;
    } else {
        p.maxSubmissionrings = ri.maxFlowrings;                      // legacy meaning
        if (p.maxSubmissionrings < kNumH2dCommon) return p;
        p.maxFlowrings = (uint16_t)(p.maxSubmissionrings - kNumH2dCommon);
        p.maxCompletionrings = kNumD2hCommon;
    }
    if (p.maxFlowrings > 512) return p;                              // "invalid max_flowrings"

    uint32_t h2dW, h2dR, d2hW, d2hR;
    p.hostIndices = sh.dmaIdxSize != 0 && allowHostIdx;
    if (!p.hostIndices) {
        h2dW = ri.h2dWIdxPtr; h2dR = ri.h2dRIdxPtr; d2hW = ri.d2hWIdxPtr; d2hR = ri.d2hRIdxPtr;
        p.idxStride = 4;                                             // TCM indices: u16 at u32 stride
    } else {
        p.idxStride = sh.dmaIdxSize;
        p.hostH2dW = 0;
        p.hostH2dR = p.hostH2dW + p.maxSubmissionrings * p.idxStride;
        p.hostD2hW = p.hostH2dR + p.maxSubmissionrings * p.idxStride;
        p.hostD2hR = p.hostD2hW + p.maxCompletionrings * p.idxStride;
        p.hostBufSize = (uint32_t)(p.maxSubmissionrings + p.maxCompletionrings) * p.idxStride * 2;
        h2dW = p.hostH2dW; h2dR = p.hostH2dR; d2hW = p.hostD2hW; d2hR = p.hostD2hR;
    }
    const uint32_t *sizes = sh.version < 7 ? kRingItemSizePreV7 : kRingItemSizeV7;
    uint32_t mem = ri.ringmem;
    for (uint8_t i = 0; i < kNumCommon; i++) {
        RingDesc &d = p.rings[i];
        d.id = i;
        d.maxItem = kRingMaxItem[i];
        d.itemSize = sizes[i];
        d.memAddr = mem;
        if (i < kNumH2dCommon) { d.wIdxAddr = h2dW; d.rIdxAddr = h2dR; h2dW += p.idxStride; h2dR += p.idxStride; }
        else                   { d.wIdxAddr = d2hW; d.rIdxAddr = d2hR; d2hW += p.idxStride; d2hR += p.idxStride; }
        mem += kRingMemSz;
    }
    p.ok = true;
    return p;
}

// Doorbell semantics are chip-core specific (PCIe core registers) and live in
// the IOKit transport; see docs/FULLMAC_PLAN.md.

} // namespace msgbuf

#endif /* Msgbuf_hpp */
