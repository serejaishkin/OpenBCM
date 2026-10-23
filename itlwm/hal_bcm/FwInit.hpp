// FwInit.hpp — post-boot firmware configuration for FullMAC, pure C++.
// Port of brcmfmac common.c brcmf_c_preinit_dcmds() (CLM upload, mpc, join_pref,
// event mask, scan times, txbf) + cfg80211.c brcmf_config_dongle()/brcmf_dongle_roam().
//
// CRITICAL: the firmware only emits events whose bit is set in "event_msgs";
// without it escan results / LINK / PSK_SUP never arrive.
//
// Io contract: bool ioctl(uint32_t cmd, const void *in, uint16_t inLen, void *out,
//                         uint16_t outCap, int16_t *fwStatus)   (true = round trip done)

#ifndef FwInit_hpp
#define FwInit_hpp

#include <stdint.h>
#include <string.h>
#include "WlcCodec.hpp"

namespace fwinit {

enum Cmd : uint32_t {
    C_UP = 2, C_SET_INFRA = 20, C_SET_ROAM_TRIGGER = 55, C_SET_ROAM_DELTA = 57, C_SET_PM = 86,
    C_SET_SCAN_CHANNEL_TIME = 185, C_SET_SCAN_UNASSOC_TIME = 187, C_SET_FAKEFRAG = 219,
    C_GET_VAR = 262, C_SET_VAR = 263,
};
// Events brcmfmac registers handlers for (fweh.h numbering).
static const uint8_t kEvents[] = {
    0 /*SET_SSID*/, 5 /*DEAUTH*/, 6 /*DEAUTH_IND*/, 8 /*ASSOC_IND*/, 10 /*REASSOC_IND*/,
    12 /*DISASSOC_IND*/, 16 /*LINK*/, 17 /*MIC_ERROR*/, 19 /*ROAM*/, 46 /*PSK_SUP*/,
    54 /*IF*/, 69 /*ESCAN_RESULT*/,
};
static constexpr unsigned kEventMaskLen = 16;
static constexpr uint32_t kScanChannelTime = 40, kScanUnassocTime = 40;
static constexpr int32_t kRoamTrigger = -75, kRoamDelta = 20, kBandAll = 3;
static constexpr uint32_t kBcnTimeoutRoamOn = 8;
static constexpr uint16_t kMaxChunk = 1400, kDlBegin = 0x0002, kDlEnd = 0x0004,
                          kDlHandlerVer = 1, kDlVerShift = 12, kDlTypeClm = 2;

struct Report { unsigned failed {0}; bool clmLoaded {false}; bool eventsSet {false}; };

template <class Io>
class Init {
public:
    explicit Init(Io &io) : mIo(io) {}

    bool setVar(const char *name, const void *data, uint16_t len) {
        uint8_t buf[1500];
        size_t n = wlc::codec::buildIovar(buf, sizeof(buf), name, data, len);
        if (!n) return false;
        int16_t st = 0;
        return mIo.ioctl(C_SET_VAR, buf, (uint16_t)n, nullptr, 0, &st) && st == 0;
    }
    bool setVarInt(const char *name, uint32_t v) {
        uint8_t b[4]; wlc::codec::put32(b, v);
        return setVar(name, b, 4);
    }
    bool getVar(const char *name, void *out, uint16_t cap) {
        uint8_t buf[64];
        size_t n = wlc::codec::buildIovar(buf, sizeof(buf), name, nullptr, 0);
        if (!n) return false;
        int16_t st = 0;
        return mIo.ioctl(C_GET_VAR, buf, (uint16_t)n, out, cap, &st) && st == 0;
    }
    bool setCmdInt(uint32_t cmd, uint32_t v) {
        uint8_t b[4]; wlc::codec::put32(b, v);
        int16_t st = 0;
        return mIo.ioctl(cmd, b, 4, nullptr, 0, &st) && st == 0;
    }
    bool setCmdPair(uint32_t cmd, uint32_t a, uint32_t b2) {
        uint8_t b[8]; wlc::codec::put32(b, a); wlc::codec::put32(b + 4, b2);
        int16_t st = 0;
        return mIo.ioctl(cmd, b, 8, nullptr, 0, &st) && st == 0;
    }

    // brcmf_c_download_blob("clmload"): 1400-byte chunks with a 12-byte header
    // {u16 flag, u16 dload_type, u32 len, u32 crc=0}.
    bool downloadClm(const uint8_t *blob, size_t size) {
        uint8_t chunk[12 + kMaxChunk];
        uint16_t flag = kDlBegin;
        size_t done = 0;
        while (done < size) {
            size_t n = size - done;
            if (n > kMaxChunk) n = kMaxChunk; else flag |= kDlEnd;
            uint16_t f = (uint16_t)(flag | (kDlHandlerVer << kDlVerShift));
            chunk[0] = (uint8_t)f; chunk[1] = (uint8_t)(f >> 8);
            chunk[2] = (uint8_t)kDlTypeClm; chunk[3] = 0;
            wlc::codec::put32(chunk + 4, (uint32_t)n);
            wlc::codec::put32(chunk + 8, 0);
            memcpy(chunk + 12, blob + done, n);
            if (!setVar("clmload", chunk, (uint16_t)(12 + n))) return false;
            flag &= (uint16_t)~kDlBegin;
            done += n;
        }
        return true;
    }

    // Everything brcmfmac does before the interface comes up. Non-fatal failures are
    // counted in Report::failed (txbf, roam tuning etc. are allowed to fail).
    Report run(const uint8_t *clm, size_t clmLen) {
        Report r;
        if (clm && clmLen) r.clmLoaded = downloadClm(clm, clmLen);
        if (clm && clmLen && !r.clmLoaded) r.failed++;
        if (!setVarInt("mpc", 1)) r.failed++;
        {   // join_pref: RSSI_DELTA boost 8 on 5 GHz, then RSSI
            const uint8_t jp[8] = {4 /*RSSI_DELTA*/, 2, 8, 1 /*5G*/, 1 /*RSSI*/, 2, 0, 0};
            if (!setVar("join_pref", jp, sizeof(jp))) r.failed++;
        }
        {   // event_msgs: read-modify-write so firmware defaults stay enabled
            uint8_t mask[kEventMaskLen] = {0};
            if (!getVar("event_msgs", mask, sizeof(mask))) { r.failed++; memset(mask, 0, sizeof(mask)); }
            for (uint8_t e : kEvents) mask[e >> 3] |= (uint8_t)(1u << (e & 7));
            r.eventsSet = setVar("event_msgs", mask, sizeof(mask));
            if (!r.eventsSet) r.failed++;
        }
        if (!setCmdInt(C_SET_SCAN_CHANNEL_TIME, kScanChannelTime)) r.failed++;
        if (!setCmdInt(C_SET_SCAN_UNASSOC_TIME, kScanUnassocTime)) r.failed++;
        (void)setVarInt("txbf", 1);                       // not supported everywhere
        // brcmf_config_dongle
        if (!setCmdInt(C_UP, 0)) r.failed++;
        if (!setCmdInt(C_SET_PM, 0)) r.failed++;          // power save off (PM_OFF)
        if (!setVarInt("bcn_timeout", kBcnTimeoutRoamOn)) r.failed++;
        if (!setVarInt("roam_off", 0)) r.failed++;
        (void)setCmdPair(C_SET_ROAM_TRIGGER, (uint32_t)kRoamTrigger, (uint32_t)kBandAll);
        (void)setCmdPair(C_SET_ROAM_DELTA, (uint32_t)kRoamDelta, (uint32_t)kBandAll);
        if (!setCmdInt(C_SET_INFRA, 1)) r.failed++;
        if (!setCmdInt(C_SET_FAKEFRAG, 1)) r.failed++;    // frameburst
        return r;
    }

private:
    Io &mIo;
};

} // namespace fwinit

#endif /* FwInit_hpp */
