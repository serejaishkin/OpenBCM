// Host test: msgbuf::DataPath (flowrings, TX post/status, RX post/complete) against simulated rings.
#include <stdio.h>
#include <string.h>
#include <map>
#include <memory>
#include <vector>
#include "MsgbufData.hpp"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)
using namespace msgbuf;

struct SimHost {
    std::map<uint64_t, std::unique_ptr<std::vector<uint8_t>>> bufs;
    uint64_t next {0x100000000ULL};
    bool allocDma(size_t len, Dma *d) {
        auto v = std::make_unique<std::vector<uint8_t>>(len, 0);
        d->va = v->data(); d->bus = next; d->len = len;
        next += (len + 0xFFFF) & ~(size_t)0xFFFF;
        bufs[d->bus] = std::move(v);
        return true;
    }
    void freeDma(Dma *d) { bufs.erase(d->bus); *d = Dma(); }
    uint8_t *at(uint64_t bus) {
        auto it = bufs.upper_bound(bus);
        if (it == bufs.begin()) return nullptr;
        --it;
        return bus - it->first < it->second->size() ? it->second->data() + (bus - it->first) : nullptr;
    }
};

struct SimRing {
    Ring ring;
    std::vector<uint8_t> mem;
    uint8_t *base {nullptr};      // ring storage the device reads (defaults to mem)
    uint16_t devR {0}, devW {0};
    bool h2d;
    uint16_t depth, item;
    int bells {0};
    void setup(bool isH2d, uint16_t d, uint16_t it) {
        h2d = isH2d; depth = d; item = it; mem.assign((size_t)d * it, 0);
        ring.registerCallbacks(&bell, &updR, &updW, &wrR, &wrW, this);
        base = mem.data();
        ring.config(d, it, mem.data());
    }
    static void bell(void *c) { ((SimRing *)c)->bells++; }
    static void updR(void *c) { auto *s = (SimRing *)c; s->ring.rPtr = s->devR; }
    static void updW(void *c) { auto *s = (SimRing *)c; s->ring.wPtr = s->devW; }
    static void wrR(void *c) { auto *s = (SimRing *)c; s->devR = s->ring.rPtr; }
    static void wrW(void *c) { auto *s = (SimRing *)c; s->devW = s->ring.wPtr; }
    // device consumes next H2D item; nullptr if empty
    const uint8_t *devPop() {
        if (devR == devW) return nullptr;
        const uint8_t *p = base + (size_t)devR * item;
        devR = (uint16_t)((devR + 1) % depth);
        return p;
    }
    void devPush(const void *it, size_t len) {
        uint8_t *slot = base + (size_t)devW * item;
        memset(slot, 0, item);
        memcpy(slot, it, len);
        devW = (uint16_t)((devW + 1) % depth);
    }
};

static int g_txDone = 0, g_txFail = 0, g_rx = 0;
static std::vector<uint8_t> g_lastRx; static uint8_t g_lastRxIf; static uint16_t g_lastRxFlags;
static void onTxDone(void *, bool ok) { ok ? g_txDone++ : g_txFail++; }
static void onRx(void *, uint8_t ifidx, const uint8_t *f, uint16_t len, uint16_t flags) {
    g_rx++; g_lastRx.assign(f, f + len); g_lastRxIf = ifidx; g_lastRxFlags = flags;
}

static std::vector<uint8_t> mkFrame(uint8_t da5, size_t payload) {
    std::vector<uint8_t> f(14 + payload);
    uint8_t da[6] = {2, 0, 0, 0, 0, da5}, sa[6] = {2, 0, 0, 0, 0, 0x99};
    memcpy(&f[0], da, 6); memcpy(&f[6], sa, 6); f[12] = 0x08; f[13] = 0x00;
    for (size_t i = 0; i < payload; i++) f[14 + i] = (uint8_t)(i * 7 + da5);
    return f;
}

int main() {
    SimHost host;
    SimRing ctl, rxp, txc, rxc, fl[4];
    ctl.setup(true, 64, 40); rxp.setup(true, 1024, 32); txc.setup(false, 1024, 24); rxc.setup(false, 1024, 40);
    Ring *flr[4];
    for (int i = 0; i < 4; i++) { fl[i].setup(true, 512, 48); flr[i] = &fl[i].ring; }

    DataPath<SimHost> dp(host);
    CHECK(dp.init(&ctl.ring, &rxp.ring, &txc.ring, &rxc.ring, flr, 4, /*rxDataOffset*/ 8, 255, &onRx, &onTxDone, nullptr));

    // --- RX buffer posting
    dp.fillRx();
    int posted = 0; uint32_t firstId = 0xFFFF; uint64_t firstBus = 0;
    while (const uint8_t *p = rxp.devPop()) {
        const RxBufPost *m = (const RxBufPost *)p;
        CHECK(m->msg.msgtype == kRxBufPost && m->dataBufLen == 2048);
        if (!posted) { firstId = m->msg.requestId; firstBus = ((uint64_t)m->dataBufAddr.high << 32) | m->dataBufAddr.low; }
        posted++;
    }
    CHECK(posted == 128);                                          // capped at kRxSlots

    // --- TX: first frame creates a flowring and queues
    auto f1 = mkFrame(1, 100), f2 = mkFrame(1, 60);
    CHECK(dp.transmit(0, f1.data(), (uint16_t)f1.size(), 0));       // prio 0 -> fifo 1
    CHECK(dp.transmit(0, f2.data(), (uint16_t)f2.size(), 3));       // prio 3 -> fifo 1 (same ring)
    const FlowringCreateReq *cr = (const FlowringCreateReq *)ctl.devPop();
    CHECK(cr && cr->msg.msgtype == kFlowRingCreate && cr->flowRingId == 2 && cr->tid == 1);
    CHECK(cr && cr->maxItems == 512 && cr->lenItem == 48);
    CHECK(cr && memcmp(cr->da, &f1[0], 6) == 0 && memcmp(cr->sa, &f1[6], 6) == 0);
    CHECK(ctl.devPop() == nullptr);                                  // only one create
    CHECK(fl[0].devPop() == nullptr);                                // nothing posted before create completes
    uint64_t ringBus = cr ? (((uint64_t)cr->flowRingAddr.high << 32) | cr->flowRingAddr.low) : 0;
    fl[0].base = host.at(ringBus);                                  // device DMA-reads the ring the host allocated
    CHECK(fl[0].base != nullptr);

    dp.onFlowCreate(2, 0);
    const TxMsgHdr *t1 = (const TxMsgHdr *)fl[0].devPop(), *t2 = (const TxMsgHdr *)fl[0].devPop();
    CHECK(t1 && t1->msg.msgtype == kTxPost && t1->msg.requestId == 1 && t1->dataLen == 100);
    CHECK(t1 && memcmp(t1->txhdr, f1.data(), 14) == 0 && (t1->flags & kPktFlagsFrameMask) == kPktFlagsFrame8023);
    CHECK(t2 && t2->msg.requestId == 2 && t2->dataLen == 60 && ((t2->flags >> kPktFlagsPrioShift) & 7) == 3);
    if (t1) {
        uint64_t b = ((uint64_t)t1->dataBufAddr.high << 32) | t1->dataBufAddr.low;
        CHECK(host.at(b) && memcmp(host.at(b), f1.data() + 14, 100) == 0);
    }
    // later frames go straight out
    auto f3 = mkFrame(1, 40);
    CHECK(dp.transmit(0, f3.data(), (uint16_t)f3.size(), 0));
    const TxMsgHdr *t3 = (const TxMsgHdr *)fl[0].devPop();
    CHECK(t3 && t3->msg.requestId == 3);

    // different access category -> a second flowring (wire id 3)
    CHECK(dp.transmit(0, f3.data(), (uint16_t)f3.size(), 6));        // prio 6 -> fifo 3
    const FlowringCreateReq *cr2 = (const FlowringCreateReq *)ctl.devPop();
    CHECK(cr2 && cr2->flowRingId == 3 && cr2->tid == 3);
    if (cr2) fl[1].base = host.at(((uint64_t)cr2->flowRingAddr.high << 32) | cr2->flowRingAddr.low);

    // --- TX status frees slots and notifies
    for (uint32_t id = 1; id <= 3; id++) {
        TxStatus ts; memset(&ts, 0, sizeof ts); ts.msg.msgtype = kTxStatus; ts.msg.requestId = id; ts.compl_.flowRingId = 2;
        txc.devPush(&ts, sizeof ts);
    }
    TxStatus bogus; memset(&bogus, 0, sizeof bogus); bogus.msg.msgtype = kTxStatus; bogus.msg.requestId = 9999;
    txc.devPush(&bogus, sizeof bogus);
    CHECK(dp.processTxComplete() == 4);
    CHECK(g_txDone == 3);

    // flow create failure drops pending
    dp.onFlowCreate(3, 5);
    CHECK(g_txFail == 1);
    CHECK(dp.transmit(0, f3.data(), (uint16_t)f3.size(), 6));        // flow can be recreated
    CHECK(ctl.devPop() != nullptr);

    // --- RX delivery
    {
        uint8_t *buf = host.at(firstBus);
        auto rf = mkFrame(7, 50);
        memcpy(buf + 8, rf.data(), rf.size());                         // rxDataOffset = 8
        RxComplete rc; memset(&rc, 0, sizeof rc);
        rc.msg.msgtype = kRxCmplt; rc.msg.ifidx = 0; rc.msg.requestId = firstId;
        rc.dataLen = (uint16_t)rf.size(); rc.flags = kPktFlagsFrame8023;
        rxc.devPush(&rc, sizeof rc);
        RxComplete bad = rc; bad.msg.requestId = 5000; rxc.devPush(&bad, sizeof bad);
        RxComplete oob = rc; oob.msg.requestId = firstId + 1; oob.dataLen = 4000; rxc.devPush(&oob, sizeof oob);
        CHECK(dp.processRxComplete() == 3);
        CHECK(g_rx == 1 && g_lastRx == rf && g_lastRxIf == 0 && g_lastRxFlags == kPktFlagsFrame8023);
        CHECK(dp.badRx == 2);
    }
    if (g_fail) { printf("%d failures\n", g_fail); return 1; }
    printf("msgbuf data: OK\n");
    return 0;
}
