// Host test: msgbuf::Engine against a simulated firmware that serves the
// control rings (ioctl responses + events).
#include <stdio.h>
#include <string.h>
#include <deque>
#include <map>
#include <memory>
#include <vector>
#include "MsgbufEngine.hpp"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

using namespace msgbuf;

struct SimHost {
    std::map<uint64_t, std::unique_ptr<std::vector<uint8_t>>> bufs;
    uint64_t next {0x100000000ULL};
    int allocs {0}, frees {0};
    bool failAlloc {false};
    bool allocDma(size_t len, Dma *d) {
        if (failAlloc) return false;
        auto v = std::make_unique<std::vector<uint8_t>>(len, 0);
        d->va = v->data(); d->bus = next; d->len = len;
        next += 0x10000;
        bufs[d->bus] = std::move(v);
        allocs++;
        return true;
    }
    void freeDma(Dma *d) { bufs.erase(d->bus); *d = Dma(); frees++; }
    uint8_t *at(uint64_t bus) { auto it = bufs.find(bus); return it == bufs.end() ? nullptr : it->second->data(); }
};

struct SimFw {
    SimHost *host;
    Ring *h2d {nullptr}, *d2h {nullptr};
    std::vector<uint8_t> h2dMem, d2hMem;
    uint16_t h2dDevR {0}, h2dDevW {0}, d2hDevR {0}, d2hDevW {0};
    std::deque<std::pair<uint32_t, uint64_t>> ioctlBufs, eventBufs;   // (reqId, bus)
    uint32_t rxDataOffset {0};
    int ioctlsSeen {0};
    bool sendAck {false};

    // ring callbacks
    static void h2dUpdR(void *c) { auto *f = (SimFw *)c; f->h2d->rPtr = f->h2dDevR; }
    static void h2dWrW(void *c) { auto *f = (SimFw *)c; f->h2dDevW = f->h2d->wPtr; }
    static void d2hUpdW(void *c) { auto *f = (SimFw *)c; f->d2h->wPtr = f->d2hDevW; }
    static void d2hWrR(void *c) { auto *f = (SimFw *)c; f->d2hDevR = f->d2h->rPtr; }

    void pushD2h(const void *item, size_t len) {
        uint8_t *slot = &d2hMem[(size_t)d2hDevW * 24];
        memset(slot, 0, 24);
        memcpy(slot, item, len);
        d2hDevW = (uint16_t)((d2hDevW + 1) % 64);
    }

    void poll() {
        while (h2dDevR != h2dDevW) {
            const uint8_t *it = &h2dMem[(size_t)h2dDevR * 40];
            const CommonHdr *h = (const CommonHdr *)it;
            if (h->msgtype == kIoctlRespBufPost || h->msgtype == kEventBufPost) {
                const RxIoctlRespOrEvent *m = (const RxIoctlRespOrEvent *)it;
                uint64_t bus = ((uint64_t)m->hostBufAddr.high << 32) | m->hostBufAddr.low;
                (h->msgtype == kEventBufPost ? eventBufs : ioctlBufs).push_back({m->msg.requestId, bus});
            } else if (h->msgtype == kIoctlPtrReq) {
                const IoctlReqHdr *m = (const IoctlReqHdr *)it;
                ioctlsSeen++;
                uint64_t reqBus = ((uint64_t)m->reqBufAddr.high << 32) | m->reqBufAddr.low;
                const uint8_t *req = host->at(reqBus);
                if (sendAck) {
                    IoctlRespHdr ack = {};
                    ack.msg.msgtype = kIoctlPtrReqAck;
                    pushD2h(&ack, sizeof(ack));
                }
                if (!ioctlBufs.empty()) {
                    auto rb = ioctlBufs.front(); ioctlBufs.pop_front();
                    uint8_t *out = host->at(rb.second);
                    // response = "R:" + request payload (first inputBufLen bytes)
                    out[0] = 'R'; out[1] = ':';
                    memcpy(out + 2, req, m->inputBufLen);
                    IoctlRespHdr r = {};
                    r.msg.msgtype = kIoctlCmplt;
                    r.msg.requestId = rb.first;
                    r.compl_.status = (uint16_t)(int16_t)(m->cmd == 999 ? -23 : 0);
                    r.respLen = (uint16_t)(2 + m->inputBufLen);
                    r.transId = m->transId;
                    r.cmd = m->cmd;
                    pushD2h(&r, sizeof(r));
                }
            }
            h2dDevR = (uint16_t)((h2dDevR + 1) % 64);
        }
    }

    bool sendEvent(const uint8_t *frame, uint16_t len, bool wrongId = false) {
        if (eventBufs.empty()) return false;
        auto eb = eventBufs.front(); eventBufs.pop_front();
        uint8_t *out = host->at(eb.second);
        memcpy(out + rxDataOffset, frame, len);
        RxEvent e = {};
        e.msg.msgtype = kWlEvent;
        e.msg.requestId = wrongId ? 31 : eb.first;
        e.eventDataLen = len;
        pushD2h(&e, sizeof(e));
        return true;
    }
};

static size_t makeEventFrame(uint8_t *f, uint32_t type, uint32_t status, uint16_t flags) {
    memset(f, 0, 14 + 10 + 48);
    f[12] = 0x88; f[13] = 0x6c;
    f[14] = 0x80; f[15] = 0x01;
    f[19] = 0x00; f[20] = 0x10; f[21] = 0x18;
    f[23] = 0x01;
    uint8_t *m = f + 24;
    m[1] = 2;
    m[2] = (uint8_t)(flags >> 8); m[3] = (uint8_t)flags;
    for (int i = 0; i < 4; i++) { m[4 + i] = (uint8_t)(type >> (24 - 8 * i)); m[8 + i] = (uint8_t)(status >> (24 - 8 * i)); }
    return 14 + 10 + 48;
}

struct Got { std::vector<wlc::codec::EventMsg> ev; };
static void sink(void *c, const wlc::codec::EventMsg &e) { ((Got *)c)->ev.push_back(e); }

static void test_engine() {
    SimHost host;
    SimFw fw; fw.host = &host; fw.rxDataOffset = 8;
    Ring rings[kNumCommon];
    fw.h2dMem.assign(64 * 40, 0); fw.d2hMem.assign(64 * 24, 0);
    fw.h2d = &rings[kH2dControlSubmit]; fw.d2h = &rings[kD2hControlComplete];
    rings[kH2dControlSubmit].registerCallbacks(nullptr, SimFw::h2dUpdR, nullptr, nullptr, SimFw::h2dWrW, &fw);
    rings[kH2dControlSubmit].config(64, 40, fw.h2dMem.data());
    rings[kD2hControlComplete].registerCallbacks(nullptr, nullptr, SimFw::d2hUpdW, SimFw::d2hWrR, nullptr, &fw);
    rings[kD2hControlComplete].config(64, 24, fw.d2hMem.data());
    Ring *rp[kNumCommon] = {&rings[0], &rings[1], &rings[2], &rings[3], &rings[4]};

    Got got;
    Engine<SimHost> eng(host);
    CHECK(eng.init(rp, fw.rxDataOffset, sink, &got));
    eng.postControlBuffers();
    fw.poll();
    CHECK(fw.ioctlBufs.size() == 8 && fw.eventBufs.size() == 8);

    // --- query ioctl: request, completion, response, repost ---
    const char req[] = "ver";
    CHECK(eng.submitIoctl(0, 262, req, sizeof(req), 256));
    CHECK(!eng.submitIoctl(0, 262, req, sizeof(req), 256));           // one in flight
    CHECK(eng.ioctlState() == IoctlState::kPending);
    fw.sendAck = true;
    fw.poll();
    CHECK(fw.ioctlsSeen == 1);
    CHECK(eng.processControlComplete() == 2);                          // ack + completion
    CHECK(eng.ioctlState() == IoctlState::kDone && eng.ioctlStatus() == 0);
    CHECK(eng.ioctlRespLen() == 2 + sizeof(req));
    char resp[64] = {0};
    CHECK(eng.takeIoctlResponse(resp, sizeof(resp)) == 2 + sizeof(req));
    CHECK(strcmp(resp, "R:ver") == 0);
    CHECK(eng.ioctlState() == IoctlState::kIdle);
    fw.poll();
    CHECK(fw.ioctlBufs.size() == 8);                                   // consumed buffer was reposted

    // --- firmware error status ---
    CHECK(eng.submitIoctl(0, 999, nullptr, 0, 0));
    fw.sendAck = false;
    fw.poll();
    eng.processControlComplete();
    CHECK(eng.ioctlState() == IoctlState::kDone && eng.ioctlStatus() == -23);
    eng.takeIoctlResponse(nullptr, 0);
    fw.poll();

    // --- events delivered through the sink; rx_dataoffset honoured ---
    uint8_t frame[128];
    size_t fl = makeEventFrame(frame, wlc::E_PSK_SUP, 6, 0);
    CHECK(fw.sendEvent(frame, (uint16_t)fl));
    fl = makeEventFrame(frame, wlc::E_LINK, 0, 1);
    CHECK(fw.sendEvent(frame, (uint16_t)fl));
    CHECK(eng.processControlComplete() == 2);
    CHECK(got.ev.size() == 2);
    CHECK(got.ev[0].eventType == wlc::E_PSK_SUP && got.ev[0].status == 6);
    CHECK(got.ev[1].eventType == wlc::E_LINK && (got.ev[1].flags & 1));
    fw.poll();
    CHECK(fw.eventBufs.size() == 8);                                   // event buffers reposted

    // --- malformed event frame / bogus request id are counted, not delivered ---
    uint8_t junk[40] = {0};
    CHECK(fw.sendEvent(junk, sizeof(junk)));
    eng.processControlComplete();
    CHECK(got.ev.size() == 2 && eng.badBuffers == 1);
    fl = makeEventFrame(frame, wlc::E_ROAM, 0, 0);
    CHECK(fw.sendEvent(frame, (uint16_t)fl, true));                    // request id 31 never posted
    eng.processControlComplete();
    CHECK(got.ev.size() == 2 && eng.badBuffers == 2);
    fw.poll();

    // --- unknown message type ---
    IoctlRespHdr u = {}; u.msg.msgtype = 0x7f;
    fw.pushD2h(&u, sizeof(u));
    eng.processControlComplete();
    CHECK(eng.unknownMsgs == 1);

    // --- release frees every buffer ---
    eng.release();
    CHECK(host.bufs.empty());
}

static void test_alloc_failure() {
    SimHost host; host.failAlloc = true;
    Ring rings[kNumCommon]; std::vector<uint8_t> mem(64 * 40);
    rings[0].config(64, 40, mem.data());
    Ring *rp[kNumCommon] = {&rings[0], &rings[1], &rings[2], &rings[3], &rings[4]};
    Engine<SimHost> eng(host);
    CHECK(!eng.init(rp, 0, nullptr, nullptr));
}

int main() {
    test_engine();
    test_alloc_failure();
    if (g_fail) { printf("%d check(s) FAILED\n", g_fail); return 1; }
    printf("all tests passed\n");
    return 0;
}
