#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "FwInit.hpp"
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

struct Call { uint32_t cmd; std::vector<uint8_t> in; };
struct FakeIo {
    std::vector<Call> calls;
    std::string failVar;                 // iovar name that returns firmware error
    uint8_t eventMask[16] {};
    bool ioctl(uint32_t cmd, const void *in, uint16_t len, void *out, uint16_t cap, int16_t *st) {
        const uint8_t *p = (const uint8_t *)in;
        calls.push_back({cmd, std::vector<uint8_t>(p, p + len)});
        *st = 0;
        if (cmd == fwinit::C_GET_VAR && !strcmp((const char *)p, "event_msgs") && out) {
            memcpy(out, eventMask, cap < 16 ? cap : 16);
        }
        if ((cmd == fwinit::C_SET_VAR) && !failVar.empty() && failVar == (const char *)p) *st = -23;
        return true;
    }
    std::vector<const Call *> find(uint32_t cmd, const char *var = nullptr) {
        std::vector<const Call *> r;
        for (auto &c : calls)
            if (c.cmd == cmd && (!var || !strcmp((const char *)c.in.data(), var))) r.push_back(&c);
        return r;
    }
};

int main() {
    FakeIo io;
    io.eventMask[0] = 0x80;               // firmware default bit that must survive
    fwinit::Init<FakeIo> init(io);
    std::vector<uint8_t> clm(3000, 0xAB);
    fwinit::Report r = init.run(clm.data(), clm.size());
    CHECK(r.failed == 0 && r.clmLoaded && r.eventsSet);

    auto cl = io.find(fwinit::C_SET_VAR, "clmload");
    CHECK(cl.size() == 3);                                   // 1400 + 1400 + 200
    if (cl.size() == 3) {
        const uint8_t *a = cl[0]->in.data() + 8;             // after "clmload\0"
        uint16_t f0 = a[0] | (a[1] << 8), f2 = cl[2]->in[8] | (cl[2]->in[9] << 8);
        CHECK(f0 == (0x2 | (1 << 12)));                      // BEGIN | ver 1
        CHECK(f2 == (0x4 | (1 << 12)));                      // END | ver 1, BEGIN cleared
        CHECK(a[2] == 2);                                    // DL_TYPE_CLM
        CHECK(cl[0]->in.size() == 8 + 12 + 1400 && cl[2]->in.size() == 8 + 12 + 200);
    }
    auto em = io.find(fwinit::C_SET_VAR, "event_msgs");
    CHECK(em.size() == 1);
    if (em.size() == 1) {
        const uint8_t *m = em[0]->in.data() + 11;            // "event_msgs\0"
        CHECK(em[0]->in.size() == 11 + 16);
        CHECK(m[0] & 0x80);                                  // preserved
        CHECK(m[69 >> 3] & (1 << (69 & 7)));                 // ESCAN_RESULT
        CHECK(m[16 >> 3] & (1 << (16 & 7)));                 // LINK
        CHECK(m[46 >> 3] & (1 << (46 & 7)));                 // PSK_SUP
    }
    CHECK(io.find(fwinit::C_SET_SCAN_CHANNEL_TIME).size() == 1);
    CHECK(io.find(fwinit::C_UP).size() == 1 && io.find(fwinit::C_SET_INFRA).size() == 1);
    auto rt = io.find(fwinit::C_SET_ROAM_TRIGGER);
    CHECK(rt.size() == 1 && rt[0]->in.size() == 8 && (int32_t)(rt[0]->in[0] | rt[0]->in[1] << 8 | rt[0]->in[2] << 16 | (uint32_t)rt[0]->in[3] << 24) == -75);

    // no CLM -> no clmload; a failing event_msgs is reported
    FakeIo io2; io2.failVar = "event_msgs";
    fwinit::Init<FakeIo> i2(io2);
    fwinit::Report r2 = i2.run(nullptr, 0);
    CHECK(io2.find(fwinit::C_SET_VAR, "clmload").empty());
    CHECK(!r2.eventsSet && r2.failed == 1);
    if (g_fail) return 1;
    printf("fwinit: OK\n");
    return 0;
}
