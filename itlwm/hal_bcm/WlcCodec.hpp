// WlcCodec.hpp — pure C++ builders/parsers for the WLC/BCDC dictionary.
// No IOKit, no allocation: everything works on caller-provided buffers, so it
// is unit-tested on the host (tests/host/test_wlc_codec.cpp).
//
// Wire layouts follow Linux brcmfmac (fwil_types.h / fweh.h), natural C
// alignment (structs are NOT packed unless stated). Little-endian host data,
// big-endian only inside the event header.

#ifndef WlcCodec_hpp
#define WlcCodec_hpp

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "WlcDefs.hpp"

namespace wlc {
namespace codec {

// ---- little/big endian helpers (host assumed little-endian: x86_64/arm64) ----
inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
inline uint16_t getBe16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
inline uint32_t getBe32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

static constexpr uint32_t kNegOne32 = 0xFFFFFFFFu;

// ---- iovar framing ----------------------------------------------------------
// "name\0" + data. Returns total length or 0 when it does not fit.
inline size_t buildIovar(uint8_t *out, size_t cap, const char *name,
                         const void *data, size_t dataLen) {
    size_t nl = strlen(name) + 1;
    if (nl + dataLen > cap) return 0;
    memcpy(out, name, nl);
    if (dataLen) memcpy(out + nl, data, dataLen);
    return nl + dataLen;
}

// "bsscfg:<name>\0" + le32 bsscfg index + data  (brcmf_fil_bsscfg_data_set).
inline size_t buildBsscfgIovar(uint8_t *out, size_t cap, const char *name,
                               uint32_t bsscfgIdx, const void *data, size_t dataLen) {
    static const char kPrefix[] = "bsscfg:";
    size_t pl = sizeof(kPrefix) - 1, nl = strlen(name) + 1;
    if (pl + nl + 4 + dataLen > cap) return 0;
    memcpy(out, kPrefix, pl);
    memcpy(out + pl, name, nl);
    put32(out + pl + nl, bsscfgIdx);
    if (dataLen) memcpy(out + pl + nl + 4, data, dataLen);
    return pl + nl + 4 + dataLen;
}

inline size_t buildBsscfgInt(uint8_t *out, size_t cap, const char *name,
                             uint32_t bsscfgIdx, uint32_t value) {
    uint8_t v[4];
    put32(v, value);
    return buildBsscfgIovar(out, cap, name, bsscfgIdx, v, 4);
}

// ---- escan ------------------------------------------------------------------
// iovar "escan" payload (brcmf_escan_params_le + scan_params v1):
//   le32 version(1) | le16 action | le16 sync_id | scan_params_le (64) |
//   le16 chanspec[n] | pad to 4 | brcmf_ssid_le[nssid] (36 each)
static constexpr size_t kScanParamsFixed = 64;
static constexpr size_t kEscanHeader = 8;

inline size_t buildEscanPayload(uint8_t *out, size_t cap, uint16_t action, uint16_t syncId,
                                const uint16_t *chanspecs, uint32_t nChan,
                                const uint8_t *ssid, uint32_t ssidLen, uint8_t scanType) {
    uint32_t nSsid = (ssid && ssidLen) ? 1 : 0;
    if (ssidLen > 32) return 0;
    size_t chBytes = (size_t)nChan * 2;
    size_t chPadded = (chBytes + 3) & ~(size_t)3;
    size_t total = kEscanHeader + kScanParamsFixed + chPadded + (size_t)nSsid * 36;
    if (total > cap) return 0;
    memset(out, 0, total);
    uint8_t *p = out;
    put32(p, ESCAN_REQ_VERSION);                         // version 1
    put16(p + 4, action);
    put16(p + 6, syncId);
    p += kEscanHeader;
    // scan_params_le
    put32(p + 0, nSsid ? ssidLen : 0);                       // ssid_le.len (primary SSID)
    if (nSsid) memcpy(p + 4, ssid, ssidLen);                 // ssid_le.ssid[32]
    memset(p + 36, 0xFF, 6);                                 // bssid = broadcast
    p[42] = 2;                                               // bss_type = ANY
    p[43] = scanType;
    put32(p + 44, kNegOne32);                                // nprobes
    put32(p + 48, kNegOne32);                                // active_time
    put32(p + 52, kNegOne32);                                // passive_time
    put32(p + 56, kNegOne32);                                // home_time
    put32(p + 60, (nSsid << SCAN_PARAMS_NSSID_SHIFT) | (nChan & SCAN_PARAMS_COUNT_MASK));
    p += kScanParamsFixed;
    for (uint32_t i = 0; i < nChan; i++) put16(p + 2 * i, chanspecs[i]);
    p += chPadded;
    if (nSsid) {                                             // ssid array after (aligned) chanlist
        put32(p, ssidLen);
        memcpy(p + 4, ssid, ssidLen);
    }
    return total;
}

// Abort an escan (brcmf_abort_scanning): action ABORT(3), no channels.
inline size_t buildEscanAbort(uint8_t *out, size_t cap, uint16_t syncId) {
    return buildEscanPayload(out, cap, 3, syncId, nullptr, 0, nullptr, 0, 0);
}

// ---- join -------------------------------------------------------------------
// brcmf_ext_join_params_le, natural alignment:
//   0  ssid_le {le32 len; u8[32]}                  36
//   36 scan_le {u8 type; pad3; le32 x4}            20
//   56 assoc_le {u8 bssid[6]; pad2; le32 n; le16 list[]} 12 + 2*n  (sizeof incl. list[1] = 72 for n<=1)
static constexpr size_t kExtJoinFixed = 72;

inline size_t buildExtJoin(uint8_t *out, size_t cap, const uint8_t *ssid, uint32_t ssidLen,
                           const uint8_t *bssid /*nullable*/,
                           const uint16_t *chanspecs, uint32_t nChan) {
    if (ssidLen > 32) return 0;
    size_t total = kExtJoinFixed + (nChan > 1 ? (size_t)(nChan - 1) * 2 : 0);
    if (total > cap) return 0;
    memset(out, 0, total);
    put32(out, ssidLen);
    memcpy(out + 4, ssid, ssidLen);
    out[36] = 0xFF;                                    // scan_type = -1
    for (int i = 0; i < 4; i++) put32(out + 40 + 4 * i, kNegOne32);
    if (bssid) memcpy(out + 56, bssid, 6); else memset(out + 56, 0xFF, 6);
    put32(out + 64, nChan);
    for (uint32_t i = 0; i < nChan; i++) put16(out + 68 + 2 * i, chanspecs[i]);
    return total;
}

// WLC_SET_SSID legacy payload: brcmf_ssid_le (36 bytes).
inline size_t buildSsidLe(uint8_t *out, size_t cap, const uint8_t *ssid, uint32_t ssidLen) {
    if (ssidLen > 32 || cap < 36) return 0;
    memset(out, 0, 36);
    put32(out, ssidLen);
    memcpy(out + 4, ssid, ssidLen);
    return 36;
}

// ---- security ---------------------------------------------------------------
// brcmf_wsec_pmk_le { le16 key_len; le16 flags; u8 key[128]; } = 132 bytes.
// flags: 0 = raw PMK (32 bytes), BRCMF_WSEC_PASSPHRASE(1) = ASCII passphrase.
static constexpr size_t kWsecPmkSize = 132;
static constexpr uint16_t kWsecPassphrase = 1;

inline size_t buildWsecPmk(uint8_t *out, size_t cap, const uint8_t *key, uint16_t keyLen, bool passphrase) {
    if (cap < kWsecPmkSize || keyLen > 128) return 0;
    memset(out, 0, kWsecPmkSize);
    put16(out, keyLen);
    put16(out + 2, passphrase ? kWsecPassphrase : 0);
    memcpy(out + 4, key, keyLen);
    return kWsecPmkSize;
}

// ---- bss_info_le (version 109) ----------------------------------------------
// Natural-alignment offsets (verified by the host test against a C struct copy
// of brcmf_bss_info_le): sizeof == 128.
namespace bss {
static constexpr size_t kVersion = 0, kLength = 4, kBssid = 8, kBeaconPeriod = 14,
    kCapability = 16, kSsidLen = 18, kSsid = 19, kRateCount = 52, kRates = 56,
    kChanspec = 72, kAtim = 74, kDtim = 76, kRssi = 78, kPhyNoise = 80,
    kNCap = 81, kNbssCap = 84, kCtlCh = 88, kFlags = 96, kBasicMcs = 100,
    kIeOffset = 116, kIeLength = 120, kSnr = 124, kFixedSize = 128;
}

struct BssInfo {
    uint8_t  bssid[6];
    uint8_t  ssid[33];       // NUL-terminated for logging
    uint8_t  ssidLen;
    uint16_t capability;
    uint16_t beaconPeriod;
    uint16_t chanspec;
    uint8_t  ctlCh;
    int16_t  rssi;
    int8_t   noise;
    int16_t  snr;
    const uint8_t *ies;      // points into the input buffer
    uint32_t ieLen;
    uint32_t recordLen;      // bytes consumed (the record's `length` field)
};

// Parse one record at `p`. Returns false when truncated/invalid.
inline bool parseBssInfo(const uint8_t *p, size_t avail, BssInfo *out) {
    if (avail < bss::kFixedSize) return false;
    uint32_t ver = get32(p + bss::kVersion);
    uint32_t len = get32(p + bss::kLength);
    if (ver < 108 || ver > 0xff) return false;       // sanity: 109 expected
    if (len < bss::kFixedSize || len > avail) return false;
    memset(out, 0, sizeof(*out));
    memcpy(out->bssid, p + bss::kBssid, 6);
    out->beaconPeriod = get16(p + bss::kBeaconPeriod);
    out->capability = get16(p + bss::kCapability);
    uint8_t sl = p[bss::kSsidLen];
    if (sl > 32) sl = 32;
    out->ssidLen = sl;
    memcpy(out->ssid, p + bss::kSsid, sl);
    out->chanspec = get16(p + bss::kChanspec);
    out->rssi = (int16_t)get16(p + bss::kRssi);
    out->noise = (int8_t)p[bss::kPhyNoise];
    out->ctlCh = p[bss::kCtlCh];
    out->snr = (int16_t)get16(p + bss::kSnr);
    uint16_t ieOff = get16(p + bss::kIeOffset);
    uint32_t ieLen = get32(p + bss::kIeLength);
    if (ieOff <= len && (uint64_t)ieOff + ieLen <= len) {
        out->ies = p + ieOff;
        out->ieLen = ieLen;
    }
    out->recordLen = len;
    return true;
}

// ---- escan_result_le ----------------------------------------------------------
// {le32 buflen; le32 version; le16 sync_id; le16 bss_count; bss_info_le[]}
static constexpr size_t kEscanResultFixed = 12;

struct EscanResult {
    uint32_t bufLen;
    uint32_t version;
    uint16_t syncId;
    uint16_t bssCount;
    const uint8_t *bssData;   // first bss_info record
    size_t bssAvail;
};

inline bool parseEscanResult(const uint8_t *p, size_t len, EscanResult *out) {
    if (len < kEscanResultFixed) return false;
    out->bufLen = get32(p);
    out->version = get32(p + 4);
    out->syncId = get16(p + 8);
    out->bssCount = get16(p + 10);
    out->bssData = p + kEscanResultFixed;
    size_t usable = out->bufLen < len ? out->bufLen : len;      // never trust bufLen over len
    out->bssAvail = usable > kEscanResultFixed ? usable - kEscanResultFixed : 0;
    return true;
}

// ---- synthetic beacon -----------------------------------------------------------
// Rebuilds an 802.11 beacon from a firmware BSS record so net80211 can ingest
// it as a scan result: mgmt header (24) + ts(8) bi(2) cap(2) + IEs.
// If the firmware gave no IEs, a minimal SSID IE is synthesised.
static constexpr size_t kBeaconHdr = 24 + 12;

inline size_t buildBeacon(uint8_t *out, size_t cap, const uint8_t *bssid,
                          const uint8_t *ssid, uint8_t ssidLen,
                          uint16_t capability, uint16_t beaconPeriod,
                          const uint8_t *ies, uint32_t ieLen) {
    uint32_t synth = (ies && ieLen) ? 0 : (2u + ssidLen);
    size_t total = kBeaconHdr + (synth ? synth : ieLen);
    if (total > cap || ssidLen > 32) return 0;
    memset(out, 0, kBeaconHdr);
    put16(out, 0x0080);                          // FC: mgmt/beacon (LE: 0x80 0x00)
    memset(out + 4, 0xFF, 6);                    // DA broadcast
    memcpy(out + 10, bssid, 6);                  // SA
    memcpy(out + 16, bssid, 6);                  // BSSID
    put16(out + 32, beaconPeriod ? beaconPeriod : 100);
    put16(out + 34, capability);
    if (synth) {
        out[kBeaconHdr] = 0;                     // SSID IE
        out[kBeaconHdr + 1] = ssidLen;
        memcpy(out + kBeaconHdr + 2, ssid, ssidLen);
    } else {
        memcpy(out + kBeaconHdr, ies, ieLen);
    }
    return total;
}

// ---- event header -------------------------------------------------------------
// brcmf_event = ethhdr(14) + brcm_ethhdr(10) + event_msg_be(48), all __packed,
// event fields big-endian. event_msg_be alone is 48 bytes.
static constexpr size_t kEthHdr = 14, kBrcmEthHdr = 10, kEventMsg = 48;

struct EventMsg {
    uint16_t version;
    uint16_t flags;
    uint32_t eventType;
    uint32_t status;
    uint32_t reason;
    uint32_t authType;
    uint32_t dataLen;
    uint8_t  addr[6];
    char     ifname[17];
    uint8_t  ifidx;
    uint8_t  bsscfgidx;
    const uint8_t *data;      // payload following the message header
    size_t   dataAvail;
};

inline bool parseEventMsg(const uint8_t *p, size_t len, EventMsg *out) {
    if (len < kEventMsg) return false;
    memset(out, 0, sizeof(*out));
    out->version = getBe16(p);
    out->flags = getBe16(p + 2);
    out->eventType = getBe32(p + 4);
    out->status = getBe32(p + 8);
    out->reason = getBe32(p + 12);
    out->authType = getBe32(p + 16);
    out->dataLen = getBe32(p + 20);
    memcpy(out->addr, p + 24, 6);
    memcpy(out->ifname, p + 30, 16);
    out->ifname[16] = 0;
    out->ifidx = p[46];
    out->bsscfgidx = p[47];
    out->data = p + kEventMsg;
    out->dataAvail = len - kEventMsg;
    if (out->dataLen < out->dataAvail) out->dataAvail = out->dataLen;
    return true;
}

// Full on-the-wire event (ethernet-framed). Checks the Broadcom OUI/subtype.
inline bool parseEventFrame(const uint8_t *p, size_t len, EventMsg *out) {
    if (len < kEthHdr + kBrcmEthHdr + kEventMsg) return false;
    const uint8_t *h = p + kEthHdr;
    if (getBe16(h) != BCMILCP_SUBTYPE_VENDOR_LONG) return false;
    if (memcmp(h + 5, BRCM_OUI, 3) != 0) return false;
    if (getBe16(h + 8) != BCMILCP_BCM_SUBTYPE_EVENT) return false;
    return parseEventMsg(p + kEthHdr + kBrcmEthHdr, len - kEthHdr - kBrcmEthHdr, out);
}

// BRCMF_EVENT_MSG_LINK flag (bit 0) = link up for E_LINK.
static constexpr uint16_t kEventFlagLink = 0x01;
// E_PSK_SUP status: BRCMF_E_STATUS_... PSK_SUP_KEYED = 6 (4-way done).
static constexpr uint32_t kPskSupKeyed = 6;

} // namespace codec
} // namespace wlc

#endif /* WlcCodec_hpp */
