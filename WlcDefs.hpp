// WlcDefs.hpp — константы WLC/FIL из Linux brcmfmac (проверено по torvalds/linux master).
// Источники: brcmfmac/fwil.h, fwil_types.h, fweh.h, cfg80211.c; include/brcmu_wifi.h.
// ВАЖНО: это словарь Linux-прошивки (dcmd/BCDC). Для Apple dlimage/olmsg ID сообщений
// могут отличаться — использовать как гипотезу и проверять на железе.
#pragma once
#include <stdint.h>

namespace wlc {

// ---- WLC ioctl (BRCMF_C_*) ----
enum : uint32_t {
    C_UP                 = 2,
    C_DOWN               = 3,
    C_SET_PROMISC        = 10,
    C_SET_INFRA          = 20,
    C_SET_AUTH           = 22,
    C_GET_BSSID          = 23,
    C_GET_SSID           = 25,
    C_SET_SSID           = 26,   // join: brcmf_join_params
    C_GET_CHANNEL        = 29,
    C_SET_CHANNEL        = 30,   // u32 номер канала (не регистр PHY!)
    C_SET_RADIO          = 38,
    C_SET_KEY            = 45,
    C_SET_PASSIVE_SCAN   = 49,
    C_SCAN               = 50,   // brcmf_scan_params_le; также abort (chan_list[0] = -1)
    C_SCAN_RESULTS       = 51,
    C_DISASSOC           = 52,
    C_REASSOC            = 53,
    C_SET_COUNTRY        = 84,
    C_GET_PM             = 85,
    C_SET_PM             = 86,
    C_GET_REVINFO        = 98,
    C_GET_RSSI           = 127,
    C_GET_WSEC           = 133,
    C_SET_WSEC           = 134,
    C_GET_BSS_INFO       = 136,
    C_GET_BANDLIST       = 140,
    C_SET_SCAN_CHANNEL_TIME = 185,
    C_SET_SCAN_UNASSOC_TIME = 187,
    C_GET_VALID_CHANNELS = 217,
    C_SET_SCAN_PASSIVE_TIME = 258,
    C_GET_VAR            = 262,  // iovar get: "name\0" + data
    C_SET_VAR            = 263,  // iovar set: "name\0" + data
    C_SET_WSEC_PMK       = 268,  // brcmf_wsec_pmk_le (PSK 32 байта, flags=0)
};

// ---- Firmware events (BRCMF_E_*) ----
enum : uint32_t {
    E_SET_SSID       = 0,
    E_JOIN           = 1,
    E_AUTH           = 3,
    E_DEAUTH         = 5,
    E_DEAUTH_IND     = 6,
    E_ASSOC          = 7,
    E_REASSOC        = 9,
    E_DISASSOC       = 11,
    E_DISASSOC_IND   = 12,
    E_LINK           = 16,
    E_ROAM           = 19,
    E_SCAN_COMPLETE  = 26,   // 0x1A
    E_PSK_SUP        = 46,
    E_IF             = 54,
    E_ESCAN_RESULT   = 69,   // 0x45 — основной канал результатов скана
    E_PROBERESP_MSG  = 71,
};

// status в event_msg
enum : uint32_t {
    ES_SUCCESS = 0, ES_FAIL = 1, ES_TIMEOUT = 2, ES_NO_NETWORKS = 3,
    ES_ABORT = 4, ES_PARTIAL = 8,   // PARTIAL = один BSS в escan_result
    ES_NEWSCAN = 9, ES_NEWASSOC = 10, ES_SUPPRESS = 12, ES_NOCHANS = 13,
};

// Заголовок события (wire, big-endian), следует за ethhdr + brcm_ethhdr
//   brcm_ethhdr: subtype(be16) length(be16) version(u8) oui[3]={00,10,18} usr_subtype(be16)=1
//   event_msg_be: version(be16) flags(be16) event_type(be32) status(be32) reason(be32)
//                 auth_type(be32) datalen(be32) addr[6] ifname[16] ifidx(u8) bsscfgidx(u8)
static constexpr uint8_t  BRCM_OUI[3]        = {0x00, 0x10, 0x18};
static constexpr uint16_t BCMILCP_SUBTYPE_VENDOR_LONG = 32769;
static constexpr uint16_t BCMILCP_BCM_SUBTYPE_EVENT   = 1;

// ---- chanspec (u16) ----
enum : uint16_t {
    CHSPEC_CHAN_MASK  = 0x00ff,
    CHSPEC_SB_MASK    = 0x0300, CHSPEC_SB_LOWER = 0x0100, CHSPEC_SB_UPPER = 0x0200, CHSPEC_SB_NONE = 0x0300,
    CHSPEC_BW_MASK    = 0x0C00, CHSPEC_BW_10 = 0x0400, CHSPEC_BW_20 = 0x0800, CHSPEC_BW_40 = 0x0C00,
    CHSPEC_BW_80      = 0x2000,   // 802.11ac (в brcmu_wifi.h он пересекается с BAND_2G по маске — см. brcmu_d11 encchspec для ac)
    CHSPEC_BAND_MASK  = 0xf000, CHSPEC_BAND_5G = 0x1000, CHSPEC_BAND_2G = 0x2000,
};
// ВНИМАНИЕ: для 43602/4360 (d11 ac) в brcmfmac используется другая раскладка (brcmu_d11ac_encchspec,
// brcmu_d11.c): BW_80 = 0x2000 там отдельное поле, 2G = 0x0000, 5G = 0xc000. Проверить brcmu_d11.c
// перед использованием. Ниже — только раскладка "d11n" (brcmu_wifi.h).
static inline uint16_t ch20_chspec_d11n(int ch) {
    return (uint16_t)(ch | CHSPEC_BW_20 | CHSPEC_SB_NONE | (ch <= 14 ? CHSPEC_BAND_2G : CHSPEC_BAND_5G));
}
// ch 1  -> 0x2B01 ; ch 6 -> 0x2B06 ; ch 36 -> 0x1B24

// ---- escan ----
static constexpr uint32_t ESCAN_REQ_VERSION    = 1;     // scan_params v1 (fixed 64)
static constexpr uint32_t ESCAN_REQ_VERSION_V2 = 2;     // scan_params v2 (fixed 72)
static constexpr uint16_t ESCAN_ACTION_START   = 1;     // cfg80211.h (START=1, CONTINUE=2, ABORT=3)
static constexpr uint16_t ESCAN_SYNC_ID        = 0x1234;
static constexpr uint32_t SCAN_PARAMS_FIXED_SIZE    = 64;
static constexpr uint32_t SCAN_PARAMS_V2_FIXED_SIZE = 72;
static constexpr uint8_t  SCANTYPE_ACTIVE = 0, SCANTYPE_PASSIVE = 1, SCANTYPE_DEFAULT = 0xFF;
static constexpr uint32_t SCAN_PARAMS_COUNT_MASK  = 0x0000ffff;
static constexpr uint32_t SCAN_PARAMS_NSSID_SHIFT = 16;
static constexpr uint32_t BSS_INFO_VERSION = 109;
// iovar "escan" payload: [u32 version][u16 action][u16 sync_id][brcmf_scan_params_le]
// scan_params_le: ssid_le{u32 len; u8[32]} bssid[6] bss_type(s8; 2=ANY) scan_type(u8)
//   nprobes(-1) active_time(-1) passive_time(-1) home_time(-1) channel_num  chanspec_list[u16...]
//   channel_num = (n_ssids << 16) | n_channels ; ssid-массив идёт после списка каналов, выровнен на 4.
// Результат: событие E_ESCAN_RESULT; status==ES_PARTIAL -> escan_result_le{u32 buflen,u32 version,
//   u16 sync_id,u16 bss_count(=1), bss_info_le}; иначе скан завершён (SUCCESS / ABORT ...).

// ---- security (bsscfg iovars) ----
enum : uint32_t {
    WSEC_WEP = 0x0001, WSEC_TKIP = 0x0002, WSEC_AES = 0x0004,
    WPA_AUTH_DISABLED = 0x0000, WPA_AUTH_PSK = 0x0004, WPA_AUTH_UNSPECIFIED = 0x0002,
    WPA2_AUTH_UNSPECIFIED = 0x0040, WPA2_AUTH_PSK = 0x0080, WPA2_AUTH_PSK_SHA256 = 0x8000,
    WPA3_AUTH_SAE_PSK = 0x40000,
};
// Последовательность connect (brcmf_cfg80211_connect):
//   iovar "wpaie"(IE) ; bsscfg "wpa_auth"=WPA2_AUTH_PSK ; bsscfg "auth"=0(open) ;
//   bsscfg "wsec"=AES_ENABLED ; iovar "sup_wpa"=1 ; C_SET_WSEC_PMK(PSK[32]) ;
//   bsscfg "join" = brcmf_ext_join_params_le {ssid_le, scan_le, assoc_le{bssid,chanspec_num,chanspec[]}}
//   fallback: C_SET_SSID(brcmf_join_params). Прошивка сама делает 4-way (событие E_PSK_SUP, status 6 = COMPLETED).

} // namespace wlc
