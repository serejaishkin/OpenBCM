// Host unit tests for Chanspec.hpp and WlcCodec.hpp (no IOKit).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <initializer_list>
#include "Chanspec.hpp"
#include "WlcCodec.hpp"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

// Verbatim C copy of Linux struct brcmf_bss_info_le (natural alignment).
#pragma pack(push, 1)
struct le16 { uint16_t v; };
#pragma pack(pop)
struct ref_bss_info {
    uint32_t version, length;
    uint8_t BSSID[6];
    uint16_t beacon_period, capability;
    uint8_t SSID_len, SSID[32];
    struct { uint32_t count; uint8_t rates[16]; } rateset;
    uint16_t chanspec, atim_window;
    uint8_t dtim_period;
    uint16_t RSSI;
    int8_t phy_noise;
    uint8_t n_cap;
    uint32_t nbss_cap;
    uint8_t ctl_ch;
    uint32_t reserved32[1];
    uint8_t flags, reserved[3];
    uint8_t basic_mcs[16];
    uint16_t ie_offset;
    uint32_t ie_length;
    uint16_t SNR;
};

static void test_layouts() {
    using namespace wlc::codec;
    CHECK(offsetof(ref_bss_info, BSSID) == bss::kBssid);
    CHECK(offsetof(ref_bss_info, capability) == bss::kCapability);
    CHECK(offsetof(ref_bss_info, SSID_len) == bss::kSsidLen);
    CHECK(offsetof(ref_bss_info, SSID) == bss::kSsid);
    CHECK(offsetof(ref_bss_info, rateset) == bss::kRateCount);
    CHECK(offsetof(ref_bss_info, chanspec) == bss::kChanspec);
    CHECK(offsetof(ref_bss_info, RSSI) == bss::kRssi);
    CHECK(offsetof(ref_bss_info, phy_noise) == bss::kPhyNoise);
    CHECK(offsetof(ref_bss_info, ctl_ch) == bss::kCtlCh);
    CHECK(offsetof(ref_bss_info, flags) == bss::kFlags);
    CHECK(offsetof(ref_bss_info, basic_mcs) == bss::kBasicMcs);
    CHECK(offsetof(ref_bss_info, ie_offset) == bss::kIeOffset);
    CHECK(offsetof(ref_bss_info, ie_length) == bss::kIeLength);
    CHECK(offsetof(ref_bss_info, SNR) == bss::kSnr);
    CHECK(sizeof(ref_bss_info) == bss::kFixedSize);

    struct ref_join_scan { uint8_t t; uint32_t a, b, c, d; };
    struct ref_assoc { uint8_t bssid[6]; uint32_t n; uint16_t list[1]; };
    struct ref_ext_join { uint8_t ssid[36]; ref_join_scan s; ref_assoc a; };
    CHECK(sizeof(ref_join_scan) == 20);
    CHECK(offsetof(ref_ext_join, a) == 56);
    CHECK(offsetof(ref_ext_join, a.n) - 0 == 64);
    CHECK(sizeof(ref_ext_join) == kExtJoinFixed);
}

static void test_chanspec() {
    using namespace bcm::chanspec;
    // Known Linux values.
    CHECK(encode20(Io::kD11n, 1) == 0x2b01);
    CHECK(encode20(Io::kD11n, 6) == 0x2b06);
    CHECK(encode20(Io::kD11n, 36) == 0x1b24);
    CHECK(encode20(Io::kD11ac, 1) == 0x1001);
    CHECK(encode20(Io::kD11ac, 36) == 0xd024);
    CHECK(encode(Io::kD11ac, 42, Bw::k80) == 0xe02a);
    CHECK(encode(Io::kD11ac, 38, Bw::k40, Sb::kL) == 0xd826);
    CHECK(encode(Io::kD11ac, 38, Bw::k40, Sb::kU) == 0xd926);
    CHECK(encode(Io::kD11n, 3, Bw::k40, Sb::kL) == 0x2d03);
    CHECK(encode(Io::kD11n, 3, Bw::k80) == 0);
    // Round trips.
    for (int ch : {1, 6, 11, 13, 14, 36, 40, 64, 100, 149, 165}) {
        for (Io io : {Io::kD11n, Io::kD11ac}) {
            Decoded d = decode(io, encode20(io, (uint8_t)ch));
            CHECK(d.valid && d.chnum == ch && d.controlCh == ch && d.bw == Bw::k20);
            CHECK(d.band == (ch <= 14 ? Band::k2G : Band::k5G));
        }
    }
    Decoded d = decode(Io::kD11ac, encode(Io::kD11ac, 42, Bw::k80));
    CHECK(d.valid && d.bw == Bw::k80 && d.chnum == 42 && d.controlCh == 36);
    d = decode(Io::kD11ac, encode(Io::kD11ac, 38, Bw::k40, Sb::kU));
    CHECK(d.valid && d.bw == Bw::k40 && d.controlCh == 40);
    CHECK(controlChannel(Io::kD11ac, 0x0000) == 0);
}

static void test_escan() {
    using namespace wlc::codec;
    uint16_t cs[3] = {0x1001, 0x1006, 0xd024};
    uint8_t buf[512];
    size_t n = buildEscanPayload(buf, sizeof(buf), wlc::ESCAN_ACTION_START, 0x1234, cs, 3,
                                 nullptr, 0, wlc::SCANTYPE_ACTIVE);
    CHECK(n == 8 + 64 + 8);                       // 3 chanspecs padded to 8
    CHECK(get32(buf) == 1 && get16(buf + 4) == 1 && get16(buf + 6) == 0x1234);
    const uint8_t *sp = buf + 8;
    CHECK(get32(sp) == 0);
    for (int i = 0; i < 6; i++) CHECK(sp[36 + i] == 0xff);
    CHECK(sp[42] == 2 && sp[43] == 0);
    CHECK(get32(sp + 44) == 0xffffffffu && get32(sp + 56) == 0xffffffffu);
    CHECK(get32(sp + 60) == 3);
    CHECK(get16(sp + 64) == 0x1001 && get16(sp + 68) == 0xd024);

    const uint8_t ssid[] = "Home";
    n = buildEscanPayload(buf, sizeof(buf), 1, 7, cs, 3, ssid, 4, 0);
    CHECK(n == 8 + 64 + 8 + 36);
    CHECK(get32(buf + 8 + 60) == ((1u << 16) | 3));
    CHECK(get32(buf + 8 + 0) == 4 && memcmp(buf + 8 + 4, "Home", 4) == 0);
    CHECK(get32(buf + 8 + 64 + 8) == 4 && memcmp(buf + 8 + 64 + 12, "Home", 4) == 0);

    CHECK(buildEscanPayload(buf, 40, 1, 1, cs, 3, nullptr, 0, 0) == 0);   // too small
    n = buildEscanAbort(buf, sizeof(buf), 9);
    CHECK(n == 72 && get16(buf + 4) == 3 && get32(buf + 8 + 60) == 0);
}

static void test_join_pmk_iovar() {
    using namespace wlc::codec;
    uint8_t buf[256];
    const uint8_t ssid[] = "Net";
    const uint8_t bssid[6] = {1, 2, 3, 4, 5, 6};
    size_t n = buildExtJoin(buf, sizeof(buf), ssid, 3, bssid, nullptr, 0);
    CHECK(n == 72);
    CHECK(get32(buf) == 3 && memcmp(buf + 4, "Net", 3) == 0);
    CHECK(buf[36] == 0xff && get32(buf + 40) == 0xffffffffu && get32(buf + 52) == 0xffffffffu);
    CHECK(memcmp(buf + 56, bssid, 6) == 0 && get32(buf + 64) == 0);
    uint16_t cs[3] = {1, 2, 3};
    n = buildExtJoin(buf, sizeof(buf), ssid, 3, nullptr, cs, 3);
    CHECK(n == 72 + 4 && get32(buf + 64) == 3 && get16(buf + 72) == 3);
    for (int i = 0; i < 6; i++) CHECK(buf[56 + i] == 0xff);

    uint8_t psk[32];
    for (int i = 0; i < 32; i++) psk[i] = (uint8_t)i;
    n = buildWsecPmk(buf, sizeof(buf), psk, 32, false);
    CHECK(n == 132 && get16(buf) == 32 && get16(buf + 2) == 0 && memcmp(buf + 4, psk, 32) == 0);
    CHECK(buf[36] == 0 && buf[131] == 0);
    n = buildWsecPmk(buf, sizeof(buf), (const uint8_t *)"passphrase", 10, true);
    CHECK(n == 132 && get16(buf + 2) == 1);

    n = buildBsscfgInt(buf, sizeof(buf), "wsec", 0, wlc::WSEC_AES);
    CHECK(n == strlen("bsscfg:wsec") + 1 + 4 + 4);
    CHECK(memcmp(buf, "bsscfg:wsec\0", 12) == 0);
    CHECK(get32(buf + 12) == 0 && get32(buf + 16) == wlc::WSEC_AES);
    n = buildIovar(buf, sizeof(buf), "sup_wpa", "\x01\0\0\0", 4);
    CHECK(n == 8 + 4 && memcmp(buf, "sup_wpa\0", 8) == 0 && buf[8] == 1);
    n = buildSsidLe(buf, sizeof(buf), ssid, 3);
    CHECK(n == 36 && get32(buf) == 3);
}

static size_t make_bss(uint8_t *out, const char *ssid, int16_t rssi, uint16_t chanspec,
                       const uint8_t *ies, uint32_t ieLen) {
    memset(out, 0, 128);
    wlc::codec::put32(out, 109);
    wlc::codec::put32(out + 4, 128 + ieLen);
    const uint8_t b[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01};
    memcpy(out + 8, b, 6);
    wlc::codec::put16(out + 14, 100);
    wlc::codec::put16(out + 16, 0x0411);
    out[18] = (uint8_t)strlen(ssid);
    memcpy(out + 19, ssid, strlen(ssid));
    wlc::codec::put16(out + 72, chanspec);
    wlc::codec::put16(out + 78, (uint16_t)rssi);
    out[80] = (uint8_t)(int8_t)-92;
    out[88] = 36;
    wlc::codec::put16(out + 116, 128);
    wlc::codec::put32(out + 120, ieLen);
    wlc::codec::put16(out + 124, 30);
    if (ieLen) memcpy(out + 128, ies, ieLen);
    return 128 + ieLen;
}

static void test_bss_and_escan() {
    using namespace wlc::codec;
    uint8_t ies[] = {0, 4, 'T', 'e', 's', 't', 48, 2, 1, 0};
    uint8_t rec1[256], rec2[256], buf[1024];
    size_t l1 = make_bss(rec1, "Alpha", -55, 0xd024, ies, sizeof(ies));
    size_t l2 = make_bss(rec2, "Beta", -71, 0x1006, nullptr, 0);

    BssInfo bi;
    CHECK(parseBssInfo(rec1, l1, &bi));
    CHECK(bi.ssidLen == 5 && memcmp(bi.ssid, "Alpha", 5) == 0 && bi.ssid[5] == 0);
    CHECK(bi.rssi == -55 && bi.noise == -92 && bi.chanspec == 0xd024 && bi.ctlCh == 36);
    CHECK(bi.capability == 0x0411 && bi.beaconPeriod == 100 && bi.snr == 30);
    CHECK(bi.ieLen == sizeof(ies) && bi.ies && memcmp(bi.ies, ies, sizeof(ies)) == 0);
    CHECK(bi.recordLen == l1);
    CHECK(!parseBssInfo(rec1, 100, &bi));                  // truncated
    CHECK(!parseBssInfo(rec1, l1 - 1, &bi));               // length > avail
    uint8_t bad[256]; memcpy(bad, rec1, l1); put32(bad, 7);
    CHECK(!parseBssInfo(bad, l1, &bi));                    // wrong version

    // escan_result with two records
    put32(buf, (uint32_t)(12 + l1 + l2));
    put32(buf + 4, 109);
    put16(buf + 8, 0x1234);
    put16(buf + 10, 2);
    memcpy(buf + 12, rec1, l1);
    memcpy(buf + 12 + l1, rec2, l2);
    EscanResult er;
    CHECK(parseEscanResult(buf, 12 + l1 + l2, &er));
    CHECK(er.bssCount == 2 && er.syncId == 0x1234 && er.bssAvail == l1 + l2);
    const uint8_t *p = er.bssData; size_t left = er.bssAvail; int seen = 0;
    for (int i = 0; i < er.bssCount; i++) {
        CHECK(parseBssInfo(p, left, &bi));
        if (i == 0) CHECK(bi.rssi == -55); else CHECK(bi.rssi == -71 && bi.ssidLen == 4);
        p += bi.recordLen; left -= bi.recordLen; seen++;
    }
    CHECK(seen == 2 && left == 0);
    // buflen larger than actual data must not over-read
    put32(buf, 9999);
    CHECK(parseEscanResult(buf, 12 + l1 + l2, &er) && er.bssAvail == l1 + l2);
    CHECK(!parseEscanResult(buf, 11, &er));
}

static void test_event() {
    using namespace wlc::codec;
    uint8_t m[48 + 4] = {0};
    m[0] = 0; m[1] = 2;                       // version 2
    m[2] = 0; m[3] = 1;                       // flags LINK
    m[4] = 0; m[5] = 0; m[6] = 0; m[7] = 46;  // E_PSK_SUP
    m[11] = 6;                                // status 6
    m[23] = 4;                                // datalen 4
    memcpy(m + 24, "\x11\x22\x33\x44\x55\x66", 6);
    memcpy(m + 30, "wlan0", 5);
    m[46] = 0; m[47] = 0;
    memcpy(m + 48, "\xde\xad\xbe\xef", 4);
    EventMsg ev;
    CHECK(parseEventMsg(m, sizeof(m), &ev));
    CHECK(ev.eventType == wlc::E_PSK_SUP && ev.status == kPskSupKeyed && (ev.flags & kEventFlagLink));
    CHECK(ev.dataLen == 4 && ev.dataAvail == 4 && ev.data[0] == 0xde);
    CHECK(strcmp(ev.ifname, "wlan0") == 0 && ev.addr[5] == 0x66);
    CHECK(!parseEventMsg(m, 47, &ev));

    uint8_t f[14 + 10 + 48] = {0};
    f[12] = 0x88; f[13] = 0x6c;
    f[14] = 0x80; f[15] = 0x01;               // subtype 32769
    f[18] = 0x00;                             // version
    f[19] = 0x00; f[20] = 0x10; f[21] = 0x18; // OUI at +5..7
    f[22] = 0x00; f[23] = 0x01;               // usr_subtype
    memcpy(f + 24, m, 48);
    CHECK(parseEventFrame(f, sizeof(f), &ev) && ev.eventType == wlc::E_PSK_SUP);
    f[20] = 0x11;
    CHECK(!parseEventFrame(f, sizeof(f), &ev));
}

static void test_beacon() {
    using namespace wlc::codec;
    uint8_t out[256]; const uint8_t b[6] = {1,2,3,4,5,6};
    uint8_t ies[] = {0, 2, 'A', 'B', 3, 1, 6};
    size_t n = buildBeacon(out, sizeof(out), b, (const uint8_t *)"AB", 2, 0x0411, 100, ies, sizeof(ies));
    CHECK(n == 36 + sizeof(ies));
    CHECK(out[0] == 0x80 && out[1] == 0x00);
    CHECK(out[4] == 0xff && memcmp(out + 10, b, 6) == 0 && memcmp(out + 16, b, 6) == 0);
    CHECK(get16(out + 32) == 100 && get16(out + 34) == 0x0411 && memcmp(out + 36, ies, sizeof(ies)) == 0);
    n = buildBeacon(out, sizeof(out), b, (const uint8_t *)"AB", 2, 0x0401, 0, nullptr, 0);
    CHECK(n == 36 + 4 && out[36] == 0 && out[37] == 2 && get16(out + 32) == 100);
    CHECK(buildBeacon(out, 30, b, (const uint8_t *)"AB", 2, 0, 0, nullptr, 0) == 0);
}

int main() {
    test_beacon();
    test_layouts();
    test_chanspec();
    test_escan();
    test_join_pmk_iovar();
    test_bss_and_escan();
    test_event();
    if (g_fail) { printf("%d check(s) FAILED\n", g_fail); return 1; }
    printf("all tests passed\n");
    return 0;
}
