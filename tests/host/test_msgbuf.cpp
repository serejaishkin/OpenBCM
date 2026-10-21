// Host tests for Msgbuf.hpp and Nvram.hpp against a simulated device.
#include <stdio.h>
#include <string.h>
#include <vector>
#include "Msgbuf.hpp"
#include "Nvram.hpp"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct FakeTcm {
    std::vector<uint8_t> mem;
    FakeTcm() : mem(0x10000, 0) {}
    uint32_t read32(uint32_t a) { uint32_t v; memcpy(&v, &mem[a], 4); return v; }
    uint16_t read16(uint32_t a) { uint16_t v; memcpy(&v, &mem[a], 2); return v; }
    void w32(uint32_t a, uint32_t v) { memcpy(&mem[a], &v, 4); }
    void w16(uint32_t a, uint16_t v) { memcpy(&mem[a], &v, 2); }
};

static void test_shared_and_plan() {
    FakeTcm t;
    const uint32_t sh = 0x1000, ri = 0x2000;
    t.w32(sh, 7 | msgbuf::kSharedDmaIndex | msgbuf::kSharedHostRdyDb1);   // v7, dma idx (4B)
    t.w16(sh + 34, 0);                                                  // -> default 255
    t.w32(sh + 36, 0);
    t.w32(sh + 40, 0x1100); t.w32(sh + 44, 0x1104);
    t.w32(sh + 48, ri); t.w32(sh + 20, 0x1200);
    t.w32(ri + 0, 0x3000);                                              // ringmem
    t.w32(ri + 4, 0x4000); t.w32(ri + 8, 0x4100); t.w32(ri + 12, 0x4200); t.w32(ri + 16, 0x4300);
    t.w16(ri + 52, 40); t.w16(ri + 54, 42); t.w16(ri + 56, 3);

    auto s = msgbuf::parseSharedInfo(t, sh);
    CHECK(s.valid && s.version == 7 && s.dmaIdxSize == 4 && s.hostRdyDb1);
    CHECK(s.maxRxbufpost == 255 && s.ringInfoAddr == ri && s.consoleAddr == 0x1200);
    CHECK(s.htodMbDataAddr == 0x1100 && s.dtohMbDataAddr == 0x1104);

    auto r = msgbuf::readRingInfo(t, s.ringInfoAddr);
    CHECK(r.ringmem == 0x3000 && r.maxFlowrings == 40 && r.maxSubmissionrings == 42 && r.maxCompletionrings == 3);

    auto p = msgbuf::planRings(s, r, true);
    CHECK(p.ok && p.hostIndices && p.idxStride == 4);
    CHECK(p.hostH2dW == 0 && p.hostH2dR == 42 * 4 && p.hostD2hW == 84 * 4 && p.hostD2hR == 84 * 4 + 12);
    CHECK(p.hostBufSize == (42 + 3) * 4 * 2);
    CHECK(p.rings[0].memAddr == 0x3000 && p.rings[4].memAddr == 0x3000 + 4 * 16);
    CHECK(p.rings[0].itemSize == 40 && p.rings[1].itemSize == 32 && p.rings[3].itemSize == 24 && p.rings[4].itemSize == 40);
    CHECK(p.rings[1].wIdxAddr == 4 && p.rings[1].rIdxAddr == 42 * 4 + 4);        // h2d rings advance by stride
    CHECK(p.rings[2].wIdxAddr == 84 * 4 && p.rings[3].wIdxAddr == 84 * 4 + 4);   // d2h rings
    CHECK(p.rings[0].maxItem == 64 && p.rings[1].maxItem == 1024);

    // Same firmware but host indices refused -> TCM pointers, stride 4.
    auto q = msgbuf::planRings(s, r, false);
    CHECK(q.ok && !q.hostIndices && q.rings[0].wIdxAddr == 0x4000 && q.rings[1].wIdxAddr == 0x4004);
    CHECK(q.rings[2].rIdxAddr == 0x4300 && q.rings[4].rIdxAddr == 0x4308);

    // Pre-v7 item sizes and the legacy max_flowrings meaning (v5).
    t.w32(sh, 5);
    t.w16(ri + 52, 42);                                                  // legacy: submission rings
    auto s5 = msgbuf::parseSharedInfo(t, sh);
    auto r5 = msgbuf::readRingInfo(t, ri);
    auto p5 = msgbuf::planRings(s5, r5, true);
    CHECK(p5.ok && !p5.hostIndices && p5.maxFlowrings == 40 && p5.maxCompletionrings == 3);
    CHECK(p5.rings[3].itemSize == 16 && p5.rings[4].itemSize == 32);

    // Bad versions / bad flowring counts.
    t.w32(sh, 4); CHECK(!msgbuf::parseSharedInfo(t, sh).valid);
    t.w32(sh, 8); CHECK(!msgbuf::parseSharedInfo(t, sh).valid);
    t.w32(sh, 7); s = msgbuf::parseSharedInfo(t, sh);
    t.w16(ri + 52, 600); r = msgbuf::readRingInfo(t, ri);
    CHECK(!msgbuf::planRings(s, r, true).ok);
}

// --- ring semantics against a fake device that consumes/produces items ----------
struct RingEnv {
    uint16_t devR {0}, devW {0};        // device-side indices
    int bells {0};
    msgbuf::Ring *ring {nullptr};
    static void bell(void *c) { ((RingEnv *)c)->bells++; }
    static void updR(void *c) { auto *e = (RingEnv *)c; e->ring->rPtr = e->devR; }
    static void updW(void *c) { auto *e = (RingEnv *)c; e->ring->wPtr = e->devW; }
    static void wrR(void *c) { auto *e = (RingEnv *)c; e->devR = e->ring->rPtr; }
    static void wrW(void *c) { auto *e = (RingEnv *)c; e->devW = e->ring->wPtr; }
};

static void test_ring_submit() {
    msgbuf::Ring ring; RingEnv env; env.ring = &ring;
    std::vector<uint8_t> buf(8 * 40);
    ring.registerCallbacks(RingEnv::bell, RingEnv::updR, nullptr, nullptr, RingEnv::wrW, &env);
    ring.config(8, 40, buf.data());
    CHECK(ring.writeAvailable());
    // One slot always stays free: 7 reservations succeed, the 8th fails.
    int got = 0;
    for (int i = 0; i < 8; i++) {
        void *p = ring.reserveForWrite();
        if (!p) break;
        CHECK(p == buf.data() + (size_t)i * 40);
        got++;
    }
    CHECK(got == 7);
    CHECK(!ring.writeAvailable());
    ring.writeComplete();
    CHECK(env.devW == 7 && env.bells == 1);
    // Device consumes 3 items; the host learns it through updR on the retry path.
    env.devR = 3;
    void *p = ring.reserveForWrite();
    CHECK(p == buf.data() + 7 * 40);               // slot 7, then wraps to 0
    CHECK(ring.wPtr == 0);
    p = ring.reserveForWrite();
    CHECK(p == buf.data());
    ring.writeCancel(1);                             // undo the last reservation
    CHECK(ring.wPtr == 0);
    ring.writeCancel(1);
    CHECK(ring.wPtr == 7);

    // reserveForWriteMultiple never wraps inside one reservation.
    msgbuf::Ring r2; RingEnv e2; e2.ring = &r2;
    std::vector<uint8_t> b2(8 * 32);
    r2.registerCallbacks(nullptr, RingEnv::updR, nullptr, nullptr, RingEnv::wrW, &e2);
    r2.config(8, 32, b2.data());
    for (int i = 0; i < 5; i++) r2.reserveForWrite();
    e2.devR = 5; r2.rPtr = 5;                       // device drained everything (r_ptr already synced)
    uint16_t al = 0;
    void *m = r2.reserveForWriteMultiple(6, &al);
    CHECK(m == b2.data() + 5 * 32 && al == 3 && r2.wPtr == 0);   // 3 up to the end, no wrap
    m = r2.reserveForWriteMultiple(6, &al);
    CHECK(m == b2.data() && al == 4);               // free = r - w = 5, one slot stays reserved -> 4
}

static void test_ring_complete() {
    msgbuf::Ring ring; RingEnv env; env.ring = &ring;
    std::vector<uint8_t> buf(4 * 24);
    ring.registerCallbacks(nullptr, nullptr, RingEnv::updW, RingEnv::wrR, nullptr, &env);
    ring.config(4, 24, buf.data());
    uint16_t n = 99;
    CHECK(ring.getReadPtr(&n) == nullptr && n == 0);
    env.devW = 3;                                    // device produced 3 items
    const void *p = ring.getReadPtr(&n);
    CHECK(p == buf.data() && n == 3);
    ring.readComplete(2);
    CHECK(ring.rPtr == 2 && env.devR == 2);
    env.devW = 1;                                    // device wrapped: w < r -> run to the end
    p = ring.getReadPtr(&n);
    CHECK(p == buf.data() + 2 * 24 && n == 2);
    ring.readComplete(2);
    CHECK(ring.rPtr == 0 && env.devR == 0);
    p = ring.getReadPtr(&n);
    CHECK(p == buf.data() && n == 1);
}

static void test_builders() {
    using namespace msgbuf;
    IoctlReqHdr io;
    buildIoctlReq(&io, 0, 262, 5, 64, 1024, 0x123456789aULL);
    CHECK(io.msg.msgtype == kIoctlPtrReq && io.msg.requestId == kIoctlReqPktId && io.cmd == 262);
    CHECK(io.transId == 5 && io.inputBufLen == 64 && io.outputBufLen == 1024);
    CHECK(io.reqBufAddr.low == 0x3456789a && io.reqBufAddr.high == 0x12);
    uint8_t raw[40]; memcpy(raw, &io, 40);
    CHECK(raw[0] == 0x09 && raw[4] == 0xFE && raw[5] == 0xFF);        // request id 0xFFFE LE

    RxBufPost rx; buildRxBufPost(&rx, 77, 2048, 0x1000);
    CHECK(rx.msg.msgtype == kRxBufPost && rx.msg.requestId == 77 && rx.dataBufLen == 2048 && rx.dataBufAddr.low == 0x1000);
    RxIoctlRespOrEvent ev; buildRespOrEventPost(&ev, kEventBufPost, 9, 1024, 0x200000000ULL);
    CHECK(ev.msg.msgtype == kEventBufPost && ev.hostBufAddr.high == 2 && ev.hostBufLen == 1024);

    uint8_t eth[14] = {1,2,3,4,5,6, 7,8,9,10,11,12, 0x08, 0x00};
    TxMsgHdr tx; buildTxPost(&tx, 0, 3, eth, 5, 100, 0x9000);
    CHECK(tx.msg.msgtype == kTxPost && memcmp(tx.txhdr, eth, 14) == 0 && tx.dataLen == 100);
    CHECK(tx.flags == (kPktFlagsFrame8023 | (5 << kPktFlagsPrioShift)) && tx.segCnt == 1);

    uint8_t da[6] = {1,1,1,1,1,1}, sa[6] = {2,2,2,2,2,2};
    FlowringCreateReq fc; buildFlowringCreate(&fc, 0, da, sa, 4, 2, 512, 48, 0xabc000);
    CHECK(fc.msg.msgtype == kFlowRingCreate && fc.flowRingId == 2 && fc.maxItems == 512 && fc.lenItem == 48);
    CHECK(memcmp(fc.da, da, 6) == 0 && memcmp(fc.sa, sa, 6) == 0 && fc.flowRingAddr.low == 0xabc000);
}

static void test_nvram() {
    const char *src =
        "# comment line\n"
        "\n"
        "manfid=0x2d0\n"
        "prodid=0x052e\r\n"
        "  \n"
        "RAW1*junk=1\n"
        "macaddr=00:90:4c:c5:12:38\n"
        "bad line without equals\n"
        "sromrev=11";                                // no trailing newline
    uint8_t out[512];
    nvram::Info info;
    auto r = nvram::strip((const uint8_t *)src, strlen(src), out, sizeof(out), &info);
    CHECK(r == nvram::Result::kOk && info.boardrevAdded);
    const char *expect = "manfid=0x2d0\0prodid=0x052e\0macaddr=00:90:4c:c5:12:38\0sromrev=11\0boardrev=0xff\0";
    size_t el = 13 + 14 + 25 + 11 + 14;
    CHECK(memcmp(out, expect, el) == 0);
    CHECK(info.length % 4 == 0);
    uint32_t token; memcpy(&token, out + info.length - 4, 4);
    uint32_t words = (uint32_t)((info.length - 4) / 4);
    CHECK(token == ((~words << 16) | (words & 0xffff)));
    CHECK((token & 0xffff) == words && (token >> 16) == (~words & 0xffff));
    CHECK(out[el] == 0 && (info.length - 4) >= el + 1);          // at least one terminating NUL

    // boardrev present: nothing appended.
    const char *s2 = "boardrev=0x1203\nvendid=0x14e4\n";
    r = nvram::strip((const uint8_t *)s2, strlen(s2), out, sizeof(out), &info);
    CHECK(r == nvram::Result::kOk && !info.boardrevAdded);
    CHECK(memcmp(out, "boardrev=0x1203\0vendid=0x14e4\0", 30) == 0);

    const char *s3 = "devpath0=pcie/1/1/\nboardrev=0x1\n";
    CHECK(nvram::strip((const uint8_t *)s3, strlen(s3), out, sizeof(out), &info) == nvram::Result::kMultiDevice);
    const char *s4 = "# only comments\n\n";
    CHECK(nvram::strip((const uint8_t *)s4, strlen(s4), out, sizeof(out), &info) == nvram::Result::kEmpty);
    CHECK(nvram::strip((const uint8_t *)src, strlen(src), out, 16, &info) == nvram::Result::kTooSmall);
}

int main() {
    test_shared_and_plan();
    test_ring_submit();
    test_ring_complete();
    test_builders();
    test_nvram();
    if (g_fail) { printf("%d check(s) FAILED\n", g_fail); return 1; }
    printf("all tests passed\n");
    return 0;
}
