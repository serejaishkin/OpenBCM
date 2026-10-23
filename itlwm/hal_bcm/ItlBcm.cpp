/*
 * OpenBCM — ItlBcm.cpp
 *
 * Broadcom HAL service for the AirportItlwm (itlwm) framework. Ported/seeded
 * from the standalone AppleBCMWLANCompanion project's bring-up, then wired into
 * the itlwm net80211 + skywalk/WCL glue exactly like ItlIwm/ItlIwn/ItlIwx.
 *
 * Bring-up (implemented):
 *   IOPCIDevice -> ChipConfigurator::probe -> requestFirmware ->
 *   BcdcOlTransport init/start -> enterDownloadState ->
 *   publishSharedInfo(olmsg DMA) -> downloadFirmware -> awaitFirmwareReady.
 *
 * Network glue (implemented): net80211 ieee80211_ifattach + ic callback wiring.
 * D11 data path (implemented skeleton): Dma64RxRing / Dma64TxRing host-driven
 * dma64 rings; RX frames injected via bcm_rx_input() -> ieee80211_inputm().
 *
 * Control plane (implemented): WLC/BCDC via olmsg. The host sends WLC ioctls
 * and iovars (escan, join, set_ssid, set_wsec_pmk, ...) over olmsg and parses
 * D2H events (ESCAN_RESULT, PSK_SUP, LINK, ...). The ARM dlimage/firmware owns
 * the radio: channel hopping, beacon reception, authentication, association.
 *
 * NOT YET IMPLEMENTED (Group B, see ItlBcm.hpp header note):
 *   - PHY channel switching registers (D11 PHY channel regs TBD)
 *   - WPA3/SAE, PMF, AWDL/AirDrop, MLO
 */

#include "ItlBcm.hpp"
#include "FullmacPcie.hpp"
#include "FullmacFirmware.hpp"
#include "FullmacFwSelect.hpp"
#include "FwInit.hpp"
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSString.h>
#include "WlcDefs.hpp"
#include "WlcCodec.hpp"
#include "Chanspec.hpp"

#include <string.h>
#include <cstdio>

#define super ItlHalService
OSDefineMetaClassAndStructors(ItlBcm, ItlHalService)

#define IC2IFP(_ic_) (&(_ic_)->ic_if)
#define BCM_VENDOR_ID 0x14e4

// ---------------------------------------------------------------------------
// PCI match
// ---------------------------------------------------------------------------

bool ItlBcm::
bcm_match(IOPCIDevice *device)
{
    if (!device)
        return false;
    if (device->configRead16(kIOPCIConfigVendorID) != BCM_VENDOR_ID)
        return false;
    UInt16 dev = device->configRead16(kIOPCIConfigDeviceID);
    for (size_t i = 0; i < sizeof(kBcmChipTable) / sizeof(kBcmChipTable[0]); i++) {
        if (kBcmChipTable[i].deviceId == dev)
            return true;
    }
    return false;
}

void ItlBcm::
setDeviceConfig(const ChipDeviceConfig &devCfg)
{
    mDeviceConfig = devCfg;
    cfg.setDeviceConfig(devCfg);
}

// ---------------------------------------------------------------------------
// net80211 device callbacks
// ---------------------------------------------------------------------------

static struct ieee80211_node *
bcm_node_alloc(struct ieee80211com *ic)
{
    return (struct ieee80211_node *)malloc(sizeof(struct bcm_node), M_DEVBUF, M_NOWAIT | M_ZERO);
}

static int
bcm_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
    struct _ifnet *ifp = IC2IFP(ic);
    struct bcm_softc *sc = (struct bcm_softc *)ifp->if_softc;
    enum ieee80211_state ostate = ic->ic_state;

    // Leaving SCAN: clear flag and chain to stock machine.
    if (nstate != IEEE80211_S_SCAN && ostate == IEEE80211_S_SCAN) {
        sc->sc_scanning = false;
    }

    // fullmac: firmware performs AUTH/ASSOC/4-way itself. net80211 must not
    // emit management frames, so AUTH kicks off a WLC join and the state
    // machine resumes (-> RUN) from the PSK_SUP/LINK D2H events.
    if (nstate == IEEE80211_S_AUTH && sc && sc->sc_driver &&
        sc->sc_driver->isFullmac()) {
        sc->sc_driver->bcm_fullmac_join_start();
        return 0;
    }

    // softmac join path: AUTH/ASSOC/RUN frames flow through bcm_start (TX ring)
    // and bcm_rx_poll -> bcm_rx_input (RX ring); stock net80211 advances the
    // state when responses arrive. Just chain up and log progress.
    if (nstate == IEEE80211_S_AUTH || nstate == IEEE80211_S_ASSOC ||
        nstate == IEEE80211_S_RUN) {
        XYLog("%s: join progress ostate=%u -> nstate=%u\n",
              __FUNCTION__, (unsigned)ostate, (unsigned)nstate);
    }

    // Entering SCAN: driver owns the transition via WLC escan.
    if (nstate == IEEE80211_S_SCAN) {
        ic->ic_state = IEEE80211_S_SCAN;
        if (ostate != IEEE80211_S_SCAN) {
            XYLog("%s: entering SCAN state (ostate=%u)\n",
                  __FUNCTION__, (unsigned)ostate);
            if (sc && sc->sc_driver)
                sc->sc_driver->bcm_bgscan_start();
        }
        return 0;
    }
    return sc->sc_newstate(ic, nstate, arg);
}

static int
bcm_bgscan_start(struct ieee80211com *ic)
{
    struct _ifnet *ifp = IC2IFP(ic);
    struct bcm_softc *sc = (struct bcm_softc *)ifp->if_softc;
    // Firmware-driven scan via WLC escan over olmsg. net80211 calls
    // ic_bgscan_start when it wants a scan; we send escan via olmsg and
    // wait for E_ESCAN_RESULT events from firmware in bcm_wlc_poll_d2h().
    XYLog("%s: background scan requested (WLC escan via olmsg)\n", __FUNCTION__);
    if (sc && sc->sc_driver)
        sc->sc_driver->bcm_bgscan_start();
    return 0;
}

static int
bcm_set_key(struct ieee80211com *ic, struct ieee80211_node *ni, struct ieee80211_key *k)
{
    // Program the key into the device key table (WLC_SET_KEY equivalent).
    // For host-driven SoftMAC, keys are handled in software by net80211;
    // the driver only needs to acknowledge and track key slots for TX/RX
    // decryption offload when the D11 data path is wired.
    if (!ic || !ni || !k) return -1;
    struct bcm_softc *sc = (struct bcm_softc *)ic->ic_if.if_softc;
    if (!sc || !sc->sc_driver) return -1;
    XYLog("%s: key alg=%d len=%d flags=0x%x\n", __FUNCTION__,
          k->ik_flags, k->ik_len, k->ik_flags);
    return 0;
}

static void
bcm_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni, struct ieee80211_key *k)
{
    // Remove the key from the device key table.
    if (!ic || !ni || !k) return;
    XYLog("%s: key alg=%d\n", __FUNCTION__, k->ik_flags);
}

static void
bcm_start(struct _ifnet *ifp)
{
    struct bcm_softc *sc = (struct bcm_softc *)ifp->if_softc;
    if (sc && sc->sc_driver)
        sc->sc_driver->bcm_tx_start(ifp);
}

static void
bcm_watchdog(struct _ifnet *ifp)
{
    (void)ifp;
}

// ---------------------------------------------------------------------------
// net80211 attach (init80211)
// ---------------------------------------------------------------------------

bool ItlBcm::
init80211()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    ic->ic_phytype = IEEE80211_T_OFDM;
    ic->ic_opmode = IEEE80211_M_STA;
    ic->ic_state = IEEE80211_S_INIT;

    ic->ic_caps =
        IEEE80211_C_WEP |
        IEEE80211_C_RSN |
        IEEE80211_C_SCANALL |
        IEEE80211_C_SCANALLBAND |
        IEEE80211_C_SHSLOT |
        IEEE80211_C_SHPREAMBLE |
        IEEE80211_C_QOS;

    ic->ic_sup_rates[IEEE80211_MODE_11A] = ieee80211_std_rateset_11a;
    ic->ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
    ic->ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;

    ifp->if_softc = &com;
    ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST | IFF_DEBUG;
    ifp->if_start = bcm_start;
    ifp->if_watchdog = bcm_watchdog;

    if_attach(ifp);
    ieee80211_ifattach(ifp, getController());
    ieee80211_media_init(ifp);

    ic->ic_node_alloc = bcm_node_alloc;
    ic->ic_bgscan_start = bcm_bgscan_start;
    ic->ic_set_key = bcm_set_key;
    ic->ic_delete_key = bcm_delete_key;
    com.sc_newstate = ic->ic_newstate;
    ic->ic_newstate = bcm_newstate;

    XYLog("%s: net80211 attached, ifp=%p\n", __FUNCTION__, (void *)ifp);
    return true;
}

// ---------------------------------------------------------------------------
// net80211 RX injection (called by the D11 RX interrupt handler).
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_rx_input(mbuf_t m, int rssi, uint8_t chan)
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);
    struct ieee80211_frame *wh;
    struct ieee80211_node *ni;
    struct ieee80211_rxinfo rxi;

    wh = mtod(m, struct ieee80211_frame *);
    ni = ieee80211_find_rxnode(ic, wh);
    memset(&rxi, 0, sizeof(rxi));
    rxi.rxi_flags = 0;     // no hardware decryption
    rxi.rxi_rssi = rssi;
    rxi.rxi_chan = chan;
    // ieee80211_inputm takes ownership of `m` (it frees it on exit).
    ieee80211_inputm(IC2IFP(ic), m, ni, &rxi, nullptr);
    ieee80211_release_node(ic, ni);
}

// ---------------------------------------------------------------------------
// net80211 TX (if_start). Dequeues mbufs from if_snd and programs the D11 TX
// ring. net80211 (incl. AUTH/ASSOC mgmt via ieee80211_send_mgmt) funnels
// everything through here, so wiring this completes the join TX path; RX
// completions arrive via bcm_rx_poll() -> bcm_rx_input().
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_tx_start(struct _ifnet *ifp)
{
    struct mbuf *m;
    while ((m = ifq_dequeue(&ifp->if_snd)) != nullptr) {
        uint32_t len = mbuf_pkthdr_len(m);
        bool sent = false;
        if (isFullmac()) {
            // if_snd carries 802.3 frames; the firmware does the 802.11 encap.
            if (mFm && len >= 14 && len <= 2048) {
                uint8_t frame[2048];
                mbuf_copydata(m, 0, len, frame);
                // IPv4 TOS -> 802.1d priority (top 3 bits); everything else best effort.
                uint8_t prio = (frame[12] == 0x08 && frame[13] == 0x00 && len > 15) ? (frame[15] >> 5) : 0;
                sent = mFm->transmit(0, frame, (uint16_t)len, prio);
            }
            mbuf_freem(m);
            if (sent) ifp->if_opackets++; else ifp->if_oerrors++;
            continue;
        }
        // Bounce through a stack buffer (max 802.11 frame) into the DMA ring.
        if (len > 0 && len <= 2048 && mTxRing) {
            uint8_t frame[2048];
            mbuf_copydata(m, 0, len, frame);
            sent = mTxRing->enqueue(frame, len);
            if (sent)
                mTxRing->pollTx();
        }
        mbuf_freem(m);
        if (sent)
            ifp->if_opackets++;
        else
            ifp->if_oerrors++;
    }
}

// ---------------------------------------------------------------------------
// WLC/BCDC control plane via olmsg
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_wlc_send_ioctl(uint32_t cmd, const void *buf, UInt32 len)
{
    if (mFm) {
        int16_t st = 0;
        if (len > 0xFFFF || !mFm->ioctl(cmd, buf, (uint16_t)len, nullptr, 0, &st))
            XYLog("%s: fullmac ioctl %u failed\n", __FUNCTION__, cmd);
        else if (st)
            XYLog("%s: fullmac ioctl %u firmware status %d\n", __FUNCTION__, cmd, st);
        return;
    }
    // Build olmsg message: {id=cmd, seq, payloadLen, payload[]}
    UInt8 msgBuf[2048];
    if (len > sizeof(msgBuf) - sizeof(UInt32) * 3) {
        XYLog("%s: send_ioctl: payload too large (%u)\n", __FUNCTION__, len);
        return;
    }
    OlControlMessage *msg = (OlControlMessage *)msgBuf;
    msg->id = cmd;
    msg->seq = mWlcScanSeq++;
    msg->payloadLen = len;
    memcpy(msg->payload, buf, len);
    ol.sendControl(msgBuf, sizeof(UInt32) * 3 + len);
    XYLog("%s: send_ioctl cmd=0x%x seq=%u len=%u\n",
          __FUNCTION__, cmd, msg->seq, len);
}

void ItlBcm::
bcm_wlc_send_iovar_set(const char *name, const void *buf, UInt32 len)
{
    if (mFm) {
        UInt8 payload[2048];
        size_t n = wlc::codec::buildIovar(payload, sizeof(payload), name, buf, len);
        if (!n) { XYLog("%s: iovar %s too large\n", __FUNCTION__, name); return; }
        bcm_wlc_send_ioctl(wlc::C_SET_VAR, payload, (UInt32)n);
        return;
    }
    // Build iovar set message: id=C_SET_VAR(263), payload="name\0"+data
    UInt32 nameLen = (UInt32)strlen(name) + 1;
    UInt32 totalLen = nameLen + len;
    if (totalLen > 2048 - sizeof(UInt32) * 3) {
        XYLog("%s: send_iovar_set: payload too large (%u)\n", __FUNCTION__, totalLen);
        return;
    }
    UInt8 msgBuf[2048];
    OlControlMessage *msg = (OlControlMessage *)msgBuf;
    msg->id = wlc::C_SET_VAR;
    msg->seq = mWlcScanSeq++;
    msg->payloadLen = totalLen;
    UInt8 *p = msg->payload;
    memcpy(p, name, nameLen);
    p += nameLen;
    memcpy(p, buf, len);
    ol.sendControl(msgBuf, sizeof(UInt32) * 3 + totalLen);
    XYLog("%s: send_iovar_set name=%s seq=%u len=%u\n",
          __FUNCTION__, name, msg->seq, totalLen);
}

void ItlBcm::
bcm_wlc_scan_channels(const uint8_t *channels, uint32_t count)
{
    // iovar "escan": see WlcCodec.hpp::buildEscanPayload. BCM4360/43602/4350
    // are 802.11ac cores -> d11ac chanspec layout.
    uint16_t chspecs[64];
    if (count > 64) count = 64;
    for (uint32_t i = 0; i < count; i++)
        chspecs[i] = bcm::chanspec::encode20(bcm::chanspec::Io::kD11ac, channels[i]);
    UInt8 payload[512];
    UInt16 syncId = (UInt16)mWlcScanSeq++;
    size_t n = wlc::codec::buildEscanPayload(payload, sizeof(payload),
        wlc::ESCAN_ACTION_START, syncId, chspecs, count, nullptr, 0,
        wlc::SCANTYPE_ACTIVE);
    if (!n) {
        XYLog("%s: escan payload does not fit (%u channels)\n", __FUNCTION__, count);
        return;
    }
    bcm_wlc_send_iovar_set("escan", payload, (UInt32)n);
    XYLog("%s: escan sent (%u channels, sync_id=%u)\n", __FUNCTION__, count, syncId);
}

void ItlBcm::
bcm_wlc_send_bsscfg_set(const char *name, UInt32 value)
{
    // brcmf_fil_bsscfg_int_set: iovar "bsscfg:<name>", payload = le32 bsscfg
    // index (0) followed by le32 value.
    char full[48];
    snprintf(full, sizeof(full), "bsscfg:%s", name);
    UInt32 data[2] = { 0, value };
    bcm_wlc_send_iovar_set(full, data, sizeof(data));
}

void ItlBcm::
bcm_wlc_join(const char *ssid, uint32_t ssid_len, const UInt8 *bssid)
{
    struct ieee80211com *ic = &com.sc_ic;
    struct ieee80211_node *ni = ic->ic_bss;
    UInt32 len = ssid_len > 32 ? 32 : ssid_len;

    // ---- security (brcmf_cfg80211_connect order, see WlcDefs.hpp) ----
    bool secure = (ic->ic_flags & IEEE80211_F_RSNON) != 0;
    if (secure && !(ic->ic_flags & IEEE80211_F_PSK)) {
        // Only WPA/WPA2-PSK is handled by the firmware supplicant here.
        XYLog("%s: RSN network without PSK (802.1X/SAE not supported)\n",
              __FUNCTION__);
        mJoinPending = false;
        return;
    }
    if (secure) {
        bool wpa2 = !ni || (ni->ni_rsnprotos & IEEE80211_PROTO_RSN);
        UInt32 wsec = (ni && !(ni->ni_rsnciphers & IEEE80211_CIPHER_CCMP) &&
                       (ni->ni_rsnciphers & IEEE80211_CIPHER_TKIP))
                      ? wlc::WSEC_TKIP : wlc::WSEC_AES;
        bcm_wlc_send_bsscfg_set("wpa_auth",
            wpa2 ? wlc::WPA2_AUTH_PSK : wlc::WPA_AUTH_PSK);
        bcm_wlc_send_bsscfg_set("auth", 0);            // open system
        bcm_wlc_send_bsscfg_set("wsec", wsec);
        // Association-request RSN IE (brcmf_fil_iovar_data_set "wpaie").
        // TODO(HW): confirm ni_rsnie_tlv carries element id + length.
        if (ni && ni->ni_rsnie_tlv && ni->ni_rsnie_tlv_len)
            bcm_wlc_send_iovar_set("wpaie", ni->ni_rsnie_tlv, ni->ni_rsnie_tlv_len);
        UInt32 one = 1;
        bcm_wlc_send_iovar_set("sup_wpa", &one, sizeof(one));
        // brcmf_wsec_pmk_le { le16 key_len; le16 flags; u8 key[128]; } = 132 B
        UInt8 pmk[wlc::codec::kWsecPmkSize];
        size_t pl = wlc::codec::buildWsecPmk(pmk, sizeof(pmk), ic->ic_psk, 32, false);
        bcm_wlc_send_ioctl(wlc::C_SET_WSEC_PMK, pmk, (UInt32)pl);
        memset(pmk, 0, sizeof(pmk));
    } else {
        bcm_wlc_send_bsscfg_set("wpa_auth", wlc::WPA_AUTH_DISABLED);
        bcm_wlc_send_bsscfg_set("auth", 0);
        bcm_wlc_send_bsscfg_set("wsec", 0);
    }

    // ---- bsscfg "join": brcmf_ext_join_params_le (72 B, see WlcCodec.hpp) ----
    // Linux sends it with brcmf_fil_bsscfg_data_set(): "bsscfg:join" + le32
    // bsscfg index (0) + params.
    UInt8 ej[4 + wlc::codec::kExtJoinFixed + 4];
    wlc::codec::put32(ej, 0);
    size_t ejLen = wlc::codec::buildExtJoin(ej + 4, sizeof(ej) - 4, (const UInt8 *)ssid,
                                            len, bssid, nullptr, 0);
    bcm_wlc_send_iovar_set("bsscfg:join", ej, (UInt32)(4 + ejLen));
    XYLog("%s: join sent (ssid=%.*s, %s)\n", __FUNCTION__, (int)len, ssid,
          secure ? "PSK" : "open");

    // The old WLC_SET_SSID path stays as the documented fallback.
    // TODO(HW): if olmsg rejects "join", call bcm_wlc_join_legacy().
}

void ItlBcm::
bcm_wlc_join_legacy(const char *ssid, uint32_t ssid_len)
{
    // WLC_SET_SSID (26): brcmf_ssid_le { le32 len; u8 ssid[32]; }
    UInt8 payload[36];
    UInt32 len = ssid_len > 32 ? 32 : ssid_len;
    memset(payload, 0, sizeof(payload));
    memcpy(payload, &len, 4);
    memcpy(payload + 4, ssid, len);
    bcm_wlc_send_ioctl(wlc::C_SET_SSID, payload, 36);
}

void ItlBcm::
bcm_wlc_poll_d2h()
{
    if (mFm) { mFm->poll(); return; }
    if (!ol.isRunning())
        return;
    // Drain olmsg RX ring and process WLC events.
    // Each OlControlMessage: {id, seq, payloadLen, payload[]}
    // For D2H events: id = event type, payload = event data
    ol.drainRx(ItlBcm::s_wlc_d2h_sink, this);
}

void ItlBcm::
s_wlc_d2h_sink(void *ctx, const OlControlMessage *msg)
{
    ItlBcm *self = static_cast<ItlBcm *>(ctx);
    if (self)
        self->bcm_wlc_d2h_msg(msg);
}

void ItlBcm::
bcm_wlc_d2h_msg(const OlControlMessage *msg)
{
    bcm_wlc_handle_event(msg->id, msg->payload, msg->payloadLen, nullptr);
}

void ItlBcm::
s_fm_rx(void *ctx, uint8_t ifidx, const uint8_t *frame, uint16_t len, uint16_t flags)
{
    ItlBcm *self = static_cast<ItlBcm *>(ctx);
    (void)ifidx;
    if (!self || (flags & msgbuf::kPktFlagsFrameMask) != msgbuf::kPktFlagsFrame8023 || len < 14)
        return;
    struct _ifnet *ifp = &self->com.sc_ic.ic_ac.ac_if;
    mbuf_t m = nullptr;
    unsigned int maxChunks = 1;
    mbuf_allocpacket(MBUF_DONTWAIT, len, &maxChunks, &m);
    if (!m) { ifp->if_ierrors++; return; }
    mbuf_setlen(m, len);
    mbuf_pkthdr_setlen(m, len);
    memcpy(mbuf_data(m), frame, len);
    struct mbuf_list ml = MBUF_LIST_INITIALIZER();
    ml_enqueue(&ml, m);
    if_input(ifp, &ml);                       // 802.3 straight to the stack; no net80211 involvement
}

void ItlBcm::
s_fm_txdone(void *ctx, bool ok)
{
    (void)ctx; (void)ok;                      // slot already recycled; nothing else to account
}

void ItlBcm::
s_fm_event(void *ctx, const wlc::codec::EventMsg &ev)
{
    ItlBcm *self = static_cast<ItlBcm *>(ctx);
    if (self)
        self->bcm_wlc_handle_event(ev.eventType, ev.data, (UInt32)ev.dataAvail, &ev);
}

void ItlBcm::
bcm_wlc_handle_event(uint32_t id, const UInt8 *payload, UInt32 len,
                     const wlc::codec::EventMsg *evIn)
{
    XYLog("%s: D2H event id=0x%x (%u) len=%u\n", __FUNCTION__, id, id, len);

    switch (id) {
    case wlc::E_ESCAN_RESULT: {
        wlc::codec::EscanResult er;
        if (!wlc::codec::parseEscanResult(payload, len, &er)) break;
        XYLog("%s: ESCAN_RESULT buflen=%u version=%u sync_id=%u bss_count=%u\n",
              __FUNCTION__, er.bufLen, er.version, er.syncId, er.bssCount);
        const UInt8 *rec = er.bssData;
        size_t left = er.bssAvail;
        for (UInt16 i = 0; i < er.bssCount; i++) {
            wlc::codec::BssInfo bi;
            if (!wlc::codec::parseBssInfo(rec, left, &bi)) {
                XYLog("%s: bad bss_info record %u\n", __FUNCTION__, i);
                break;
            }
            uint8_t chan = bi.ctlCh ? bi.ctlCh :
                bcm::chanspec::controlChannel(bcm::chanspec::Io::kD11ac, bi.chanspec);
            bcm_wlc_scan_result(bi.bssid, bi.ssid, bi.ssidLen, (int8_t)bi.rssi,
                                chan, bi.capability, bi.beaconPeriod, bi.ies, bi.ieLen);
            rec += bi.recordLen;
            left -= bi.recordLen;
        }
        if (er.bssCount == 0)
            bcm_scan_done();     // empty result = scan finished
        break;
    }
    case wlc::E_PSK_SUP:
    case wlc::E_LINK: {
        // Prefer the real event layout (brcmf_event_msg_be). If the olmsg
        // payload is instead a bare u32 status (earlier hypothesis), fall back
        // to that. TODO(HW): confirm which one the dlimage emits.
        wlc::codec::EventMsg ev;
        bool haveMsg;
        if (evIn) { ev = *evIn; haveMsg = true; }
        else haveMsg = wlc::codec::parseEventMsg(payload, len, &ev) && ev.eventType == id;
        uint32_t status = 0;
        bool linkUp = false;
        if (haveMsg) {
            status = ev.status;
            linkUp = (ev.flags & wlc::codec::kEventFlagLink) != 0;
        } else if (len >= 4) {
            memcpy(&status, payload, 4);
            linkUp = (status & 1) != 0;
        } else {
            break;
        }
        XYLog("%s: event %u status=%u link=%d (%s)\n", __FUNCTION__, id, status,
              (int)linkUp, haveMsg ? "event_msg" : "u32");
        if (!isFullmac()) break;
        if (id == wlc::E_PSK_SUP && status == wlc::codec::kPskSupKeyed)
            bcm_fullmac_link_event(true);          // 4-way handshake complete
        else if (id == wlc::E_LINK && !linkUp)
            bcm_fullmac_link_event(false);
        break;
    }
    case wlc::E_SCAN_COMPLETE:
        // Legacy scan complete
        XYLog("%s: E_SCAN_COMPLETE\n", __FUNCTION__);
        bcm_scan_done();
        break;
    default:
        XYLog("%s: unhandled D2H event id=0x%x\n", __FUNCTION__, id);
        break;
    }
}

void ItlBcm::
bcm_wlc_scan_result(const UInt8 *bssid, const UInt8 *ssid, uint32_t ssid_len,
                    int8_t rssi, uint8_t chan, uint16_t capability,
                    uint16_t beaconPeriod, const UInt8 *ies, uint32_t ieLen)
{
    // Inject a firmware-reported BSS into net80211's node tree as a beacon.
    UInt8 frame[wlc::codec::kBeaconHdr + 1024];
    if (ieLen > 1024) ies = nullptr, ieLen = 0;
    size_t frameLen = wlc::codec::buildBeacon(frame, sizeof(frame), bssid, ssid,
        (UInt8)ssid_len, capability, beaconPeriod, ies, ieLen);
    if (!frameLen) {
        XYLog("%s: beacon build failed\n", __FUNCTION__);
        return;
    }
    mbuf_t m = nullptr;
    unsigned int maxChunks = 1;
    mbuf_allocpacket(MBUF_DONTWAIT, frameLen, &maxChunks, &m);
    if (!m) {
        XYLog("%s: mbuf alloc failed\n", __FUNCTION__);
        return;
    }
    mbuf_setlen(m, frameLen);
    mbuf_pkthdr_setlen(m, frameLen);
    memcpy(mbuf_data(m), frame, frameLen);
    bcm_rx_input(m, rssi, chan);     // takes ownership of m
}

// ---------------------------------------------------------------------------
// net80211 RX poll timer (50 ms): keeps data/mgmt flowing in RUN state.
// ---------------------------------------------------------------------------

void ItlBcm::
s_rx_poll(OSObject *target, IOTimerEventSource *sender)
{
    (void)sender;
    ItlBcm *self = OSDynamicCast(ItlBcm, target);
    if (!self) return;
    self->bcm_rx_poll();
    // Re-arm: keep polling while the interface is up.
    if (self->mRxPollTimer && self->com.sc_running) {
        self->mRxPollTimer->setTimeoutMS(50);
        self->mRxPollTimer->enable();
    }
}

void ItlBcm::
bcm_rx_poll()
{
    if (!com.sc_running || !mRxRing)
        return;
    if (mHostScanActive) {
        // Host-driven scan: report the current hop channel.
        mRxRing->setCurrentChan(mHostScanList[mHostScanIdx]);
    } else if (com.sc_scanning && com.sc_scan_count > 0) {
        uint8_t idx = com.sc_scan_idx > 0 ? com.sc_scan_idx - 1 : 0;
        if (idx < com.sc_scan_count)
            mRxRing->setCurrentChan(com.sc_scan_list[idx]);
        else
            mRxRing->setCurrentChan(0);
    } else {
        mRxRing->setCurrentChan(0);
    }
    mRxRing->poll(this, ItlBcm::s_rxDeliver);
}

void ItlBcm::
s_rxDeliver(void *ctx, mbuf_t m, int8_t rssi, uint8_t chan)
{
    ItlBcm *self = static_cast<ItlBcm *>(ctx);
    if (!self) {
        mbuf_freem(m);
        return;
    }
    self->bcm_rx_input(m, rssi, chan);
}

// ---------------------------------------------------------------------------
// WLC scan state machine (firmware-driven via olmsg)
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_scan_start()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    if (com.sc_scanning) return;

    // Build the active channel list from net80211's regdomain view
    // (ic_chan_avail, populated by ieee80211_media_init() from the
    // ic_channels[] table above) instead of a hardcoded table, and honour
    // the WCL-requested bitmap when the scan is restricted.
    com.sc_scan_count = 0;
    com.sc_scan_idx = 0;
    for (int ch = 1; ch <= IEEE80211_CHAN_MAX &&
         com.sc_scan_count < (int)sizeof(com.sc_scan_list); ch++) {
        if (!(ic->ic_chan_avail[ch / 8] & (1U << (ch % 8))))
            continue;
        if (ic->ic_wcl_scan_restricted) {
            if (ch >= 256) continue;
            if (!(ic->ic_wcl_scan_channels[ch / 8] & (1U << (ch % 8))))
                continue;
        }
        com.sc_scan_list[com.sc_scan_count++] = (uint8_t)ch;
    }
    if (com.sc_scan_count == 0) {
        // Defensive fallback: 2.4 GHz only
        for (int ch = 1; ch <= 14 &&
             com.sc_scan_count < (int)sizeof(com.sc_scan_list); ch++)
            com.sc_scan_list[com.sc_scan_count++] = (uint8_t)ch;
        XYLog("%s: ic_chan_avail empty, falling back to 2.4 GHz 1-14\n",
              __FUNCTION__);
    }
    if (com.sc_scan_count == 0) {
        XYLog("%s: no usable channels (restricted scan selected none)\n",
              __FUNCTION__);
        return;
    }
    com.sc_scanning = true;
    mWlcScanActive = true;

    // Create scan timer on the main workloop.
    if (!mScanTimer && getMainWorkLoop()) {
        mScanTimer = IOTimerEventSource::timerEventSource(
            this, ItlBcm::s_scan_timer);
        if (mScanTimer) {
            getMainWorkLoop()->addEventSource(mScanTimer);
        }
    }
    if (mScanTimer) {
        mScanTimer->setTimeoutMS(100); // 100 ms per channel group
        mScanTimer->enable();
    }
    XYLog("%s: scan started (%u channels via WLC escan)\n",
          __FUNCTION__, com.sc_scan_count);

    // Send escan immediately
    bcm_wlc_scan_channels(com.sc_scan_list, com.sc_scan_count);
}

void ItlBcm::
bcm_scan_next_channel()
{
    // Firmware-driven escan sends all channels at once; we just wait for
    // E_ESCAN_RESULT events via bcm_wlc_poll_d2h(). The timer is used to
    // retry if no response arrives.
    if (!com.sc_scanning) return;

    // Re-send escan if no results yet (simple retry)
    if (mWlcScanActive && com.sc_scan_count > 0) {
        bcm_wlc_scan_channels(com.sc_scan_list, com.sc_scan_count);
    }

    if (mScanTimer) {
        mScanTimer->setTimeoutMS(500); // retry every 500 ms
        mScanTimer->enable();
    }
}

void ItlBcm::
bcm_scan_done()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    com.sc_scanning = false;
    mWlcScanActive = false;
    if (mScanTimer) {
        mScanTimer->disable();
    }

    // net80211 leaves SCAN state and reports results to WCL/menu.
    XYLog("%s: scan complete, leaving SCAN state\n", __FUNCTION__);
    ieee80211_end_scan(ifp);
}

// ---------------------------------------------------------------------------
// Host-driven passive scan (D11 SoftMAC).
//
// The host hops channels via the D11 PHY, sends a probe request on each
// channel through the TX ring, and receives beacons/probe-responses through
// the RX ring (bcm_rx_poll -> bcm_rx_input -> ieee80211_inputm).
// ---------------------------------------------------------------------------

void ItlBcm::
s_host_scan_timer(OSObject *target, IOTimerEventSource *sender)
{
    (void)sender;
    ItlBcm *self = OSDynamicCast(ItlBcm, target);
    if (!self) return;
    self->bcm_host_scan_next_channel();
}

bool ItlBcm::
bcm_build_probe_req(uint8_t *buf, uint32_t bufSize, uint32_t *frameLen)
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    if (bufSize < 64 || !buf || !frameLen) return false;

    // 802.11 Probe Request frame:
    //   FC = 0x0040 (Probe Request, ToDS=0, FromDS=0)
    //   Dur = 0
    //   DA = broadcast
    //   SA = ic_myaddr
    //   BSSID = broadcast
    //   Seq = 0
    //   SSID IE (len=0, broadcast)
    //   Supported Rates IE (1,2,5.5,11,18,24,36,54 Mbps)
    struct {
        UInt16 fc;
        UInt16 dur;
        UInt8  da[6];
        UInt8  sa[6];
        UInt8  bssid[6];
        UInt16 seq;
        UInt8  ssid_ie[2];
        UInt8  rates_ie[10];
    } __attribute__((packed)) frame;

    memset(&frame, 0, sizeof(frame));
    frame.fc = 0x0040; // probe request
    memcpy(frame.da, "\xff\xff\xff\xff\xff\xff", 6);
    IEEE80211_ADDR_COPY(frame.sa, ic->ic_myaddr);
    memcpy(frame.bssid, "\xff\xff\xff\xff\xff\xff", 6);
    frame.seq = 0;

    // SSID IE: 0x00, len=0 (broadcast SSID)
    frame.ssid_ie[0] = 0x00;
    frame.ssid_ie[1] = 0x00;

    // Supported Rates IE: 0x01, len=8
    // 0x82=1MB, 0x84=2MB, 0x8b=5.5MB, 0x96=11MB, 0x24=18MB, 0x30=24MB, 0x48=36MB, 0x6c=54MB
    frame.rates_ie[0] = 0x01;
    frame.rates_ie[1] = 8;
    frame.rates_ie[2] = 0x82;
    frame.rates_ie[3] = 0x84;
    frame.rates_ie[4] = 0x8b;
    frame.rates_ie[5] = 0x96;
    frame.rates_ie[6] = 0x24;
    frame.rates_ie[7] = 0x30;
    frame.rates_ie[8] = 0x48;
    frame.rates_ie[9] = 0x6c;

    uint32_t len = (uint32_t)sizeof(frame);
    if (len > bufSize) return false;
    memcpy(buf, &frame, len);
    *frameLen = len;
    return true;
}

void ItlBcm::
bcm_host_scan_probe(uint8_t chan)
{
    // Program the D11 PHY to the target channel (also programs radio for TX).
    if (mRxRing) {
        mRxRing->setCurrentChan(chan);
    }

    // Build and send a probe request.
    uint8_t frameBuf[128];
    uint32_t frameLen = 0;
    if (!bcm_build_probe_req(frameBuf, sizeof(frameBuf), &frameLen)) {
        XYLog("%s: probe req build failed\n", __FUNCTION__);
        return;
    }
    if (!mTxRing || !mTxRing->enqueue(frameBuf, frameLen)) {
        XYLog("%s: probe req TX enqueue failed (chan=%u)\n", __FUNCTION__, (unsigned)chan);
        return;
    }
    mTxRing->pollTx();
    XYLog("%s: probe sent on channel %u (len=%u)\n",
          __FUNCTION__, (unsigned)chan, frameLen);
}

void ItlBcm::
bcm_host_scan_next_channel()
{
    if (!mHostScanActive || mHostScanCount == 0) {
        bcm_scan_done();
        return;
    }

    // Move to next channel.
    mHostScanIdx++;
    if (mHostScanIdx >= mHostScanCount) {
        // Scan complete.
        mHostScanActive = false;
        mHostScanIdx = 0;
        mHostScanCount = 0;
        bcm_scan_done();
        return;
    }

    uint8_t chan = mHostScanList[mHostScanIdx];

    // Send probe on this channel.
    bcm_host_scan_probe(chan);

    // Re-arm timer for next channel (200 ms dwell).
    if (mHostScanTimer) {
        mHostScanTimer->setTimeoutMS(200);
        mHostScanTimer->enable();
    }
}

void ItlBcm::
bcm_bgscan_start()
{
#ifdef BCM_HOST_SCAN
    // Opt-in: host-driven passive scan (needs a working D11 PHY channel
    // switch, which is still TODO(RE)). Never valid for fullmac.
    if (!isFullmac()) {
        bcm_host_scan_start();
        return;
    }
#endif
    // Default for both modes: firmware-driven scan via WLC escan over olmsg;
    // results arrive as E_ESCAN_RESULT in bcm_wlc_poll_d2h().
    XYLog("%s: scan requested (WLC escan via olmsg, %s)\n", __FUNCTION__,
          isFullmac() ? "fullmac" : "softmac");
    bcm_scan_start();
}

void ItlBcm::
s_scan_timer(OSObject *target, IOTimerEventSource *sender)
{
    (void)sender;
    ItlBcm *self = OSDynamicCast(ItlBcm, target);
    if (!self) return;
    self->bcm_scan_next_channel();
}

// ---------------------------------------------------------------------------
// fullmac join: net80211 AUTH -> WLC join; firmware reports completion.
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_fullmac_join_start()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct ieee80211_node *ni = ic->ic_bss;
    if (!ni) {
        XYLog("%s: no ic_bss\n", __FUNCTION__);
        return;
    }
    mJoinPending = true;
    XYLog("%s: joining '%.*s'\n", __FUNCTION__, (int)ni->ni_esslen, ni->ni_essid);
    bcm_wlc_join((const char *)ni->ni_essid, ni->ni_esslen, ni->ni_bssid);
}

void ItlBcm::
bcm_fullmac_link_event(bool up)
{
    struct ieee80211com *ic = &com.sc_ic;
    if (up && mJoinPending) {
        mJoinPending = false;
        XYLog("%s: link up -> RUN\n", __FUNCTION__);
        com.sc_newstate(ic, IEEE80211_S_RUN, -1);
    } else if (!up && ic->ic_state == IEEE80211_S_RUN) {
        XYLog("%s: link down -> SCAN\n", __FUNCTION__);
        ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
    }
}

void ItlBcm::
bcm_host_scan_start()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    if (mHostScanActive) return;

    // Build channel list from net80211's regdomain view.
    mHostScanCount = 0;
    mHostScanIdx = 0;
    for (int ch = 1; ch <= IEEE80211_CHAN_MAX &&
         mHostScanCount < (int)sizeof(mHostScanList); ch++) {
        if (!(ic->ic_chan_avail[ch / 8] & (1U << (ch % 8))))
            continue;
        if (ic->ic_wcl_scan_restricted) {
            if (ch >= 256) continue;
            if (!(ic->ic_wcl_scan_channels[ch / 8] & (1U << (ch % 8))))
                continue;
        }
        mHostScanList[mHostScanCount++] = (uint8_t)ch;
    }
    if (mHostScanCount == 0) {
        // Defensive fallback: 2.4 GHz only
        for (int ch = 1; ch <= 14 &&
             mHostScanCount < (int)sizeof(mHostScanList); ch++)
            mHostScanList[mHostScanCount++] = (uint8_t)ch;
        XYLog("%s: ic_chan_avail empty, falling back to 2.4 GHz 1-14\n",
              __FUNCTION__);
    }
    if (mHostScanCount == 0) {
        XYLog("%s: no usable channels\n", __FUNCTION__);
        return;
    }

    com.sc_scanning = true;
    mHostScanActive = true;

    // Create scan timer on the main workloop.
    if (!mHostScanTimer && getMainWorkLoop()) {
        mHostScanTimer = IOTimerEventSource::timerEventSource(
            this, ItlBcm::s_host_scan_timer);
        if (mHostScanTimer) {
            getMainWorkLoop()->addEventSource(mHostScanTimer);
        }
    }

    // Start on first channel.
    bcm_host_scan_next_channel();

    XYLog("%s: host-driven scan started (%u channels)\n",
          __FUNCTION__, mHostScanCount);
}

// ---------------------------------------------------------------------------
// ItlHalService
// ---------------------------------------------------------------------------

// OpenCore DeviceProperties land on the PCI device node; read a string property
// (OSString, or OSData holding the raw bytes).
static bool bcm_read_str_prop(IOPCIDevice *dev, const char *key, char *out, size_t cap)
{
    OSObject *o = dev->getProperty(key);
    if (!o) return false;
    if (OSString *str = OSDynamicCast(OSString, o)) {
        strlcpy(out, str->getCStringNoCopy(), cap);
        return out[0] != 0;
    }
    if (OSData *d = OSDynamicCast(OSData, o)) {
        size_t n = d->getLength() < cap - 1 ? d->getLength() : cap - 1;
        memcpy(out, d->getBytesNoCopy(), n);
        out[n] = 0;
        return out[0] != 0;
    }
    return false;
}

bool ItlBcm::
fullmac_attach(IOPCIDevice *device)
{
    // Firmware + nvram are embedded in the kext (OpenCore-injected kexts cannot rely on
    // bundle Resources). Selection: DeviceProperties `openbcm-fw` / `openbcm-nvram`
    // (stem names, e.g. "brcmfmac4350c2-pcie"), else auto by chip/rev like brcmfmac.
    char prop[96], fwStem[96], nvStem[96];
    if (bcm_read_str_prop(device, "openbcm-fw", prop, sizeof(prop))) {
        fwsel::normalizeStem(prop, fwStem, sizeof(fwStem));
        XYLog("bcm: fullmac firmware from device property: %s\n", fwStem);
    } else {
        const char *auto_ = fwsel::stemFor(com.sc_chipnumber, com.sc_chiprev);
        if (!auto_) {
            XYLog("bcm: no FullMAC firmware known for chip 0x%x rev %u (set openbcm-fw)\n",
                  com.sc_chipnumber, com.sc_chiprev);
            return false;
        }
        strlcpy(fwStem, auto_, sizeof(fwStem));
        XYLog("bcm: fullmac firmware auto-selected: %s\n", fwStem);
    }
    if (bcm_read_str_prop(device, "openbcm-nvram", prop, sizeof(prop)))
        fwsel::normalizeStem(prop, nvStem, sizeof(nvStem));
    else
        strlcpy(nvStem, fwStem, sizeof(nvStem));

    const FullmacBlob *fwb = bcmc_fullmac_find_fw(fwStem);
    const FullmacBlob *nvb = bcmc_fullmac_find_nvram(nvStem);
    if (!fwb || !nvb) {
        XYLog("bcm: fullmac blobs not embedded (fw '%s' %s, nvram '%s' %s); "
              "add files to firmware/fullmac and run scripts/gen_fullmac_arrays.py\n",
              fwStem, fwb ? "ok" : "MISSING", nvStem, nvb ? "ok" : "MISSING");
        return false;
    }
    const UInt8 *fw = fwb->data, *nv = nvb->data;
    UInt32 fwLen = fwb->len, nvLen = nvb->len;
    if (const FullmacBlob *clm = bcmc_fullmac_find_clm(fwStem)) {
        mFmClm = clm->data; mFmClmLen = clm->len;
        XYLog("bcm: fullmac CLM blob embedded (%u B)\n", mFmClmLen);
    }
    mFm = new fullmac::Pcie();
    if (!mFm) return false;
    fullmac::Pcie::Config fc;
    fc.fw = fw; fc.fwLen = fwLen;
    fc.nvram = nv; fc.nvLen = nvLen;
    fc.chipId = com.sc_chipnumber;
    fc.chipRev = com.sc_chiprev;
    mFm->setDataSinks(&ItlBcm::s_fm_rx, &ItlBcm::s_fm_txdone, this);
    int rc = mFm->start(device, fc, &ItlBcm::s_fm_event, this);
    if (rc) {
        XYLog("bcm: fullmac start failed at step %d\n", rc);
        delete mFm;
        mFm = nullptr;
        return false;
    }
    XYLog("bcm: fullmac firmware up\n");
    return true;
}

void ItlBcm::
fullmac_preinit()
{
    struct FmIo {
        fullmac::Pcie *p;
        bool ioctl(uint32_t cmd, const void *in, uint16_t inLen, void *out, uint16_t outCap, int16_t *st) {
            return p->ioctl(cmd, in, inLen, out, outCap, st);
        }
    } io{mFm};
    fwinit::Init<FmIo> init(io);
    fwinit::Report r = init.run(mFmClm, mFmClmLen);
    XYLog("bcm: fullmac preinit: %u step(s) failed, clm=%d events=%d\n",
          r.failed, (int)r.clmLoaded, (int)r.eventsSet);
}

void ItlBcm::
fullmac_set_mac()
{
    UInt8 req[32], mac[32] = {0};
    size_t n = wlc::codec::buildIovar(req, sizeof(req), "cur_etheraddr", nullptr, 0);
    int16_t st = 0;
    bool ok = n && mFm->ioctl(wlc::C_GET_VAR, req, (uint16_t)n, mac, sizeof(mac), &st) && st == 0;
    bool valid = ok && !(mac[0] & 1) && (mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]);
    if (!valid) {
        XYLog("bcm: cur_etheraddr unavailable, using locally administered fallback\n");
        memcpy(mac, cfg.getMacAddress(), 6);
    }
    memcpy(com.sc_ic.ic_myaddr, mac, 6);
    XYLog("bcm: MAC %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool ItlBcm::
attach(IOPCIDevice *device)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);

    memset(&com, 0, sizeof(com));
    com.sc_driver = this;

    if (!cfg.initWithDevice(device)) {
        XYLog("bcm: ChipConfigurator init failed\n");
        return false;
    }
    cfg.setDeviceConfig(mDeviceConfig);
    if (!cfg.probe()) {
        XYLog("bcm: chip probe failed\n");
        return false;
    }
    com.sc_chipnumber = cfg.getChipId();
    com.sc_chiprev = cfg.getChipRevision();

    // Resolve the descriptor into an owned copy (member), so it outlives
    // attach(); then layer the Info.plist personality on top. ChipType comes
    // from the personality when present, else from the table default.
    const ChipDescriptor *tableEntry = nullptr;
    for (size_t i = 0; i < sizeof(kBcmChipTable) / sizeof(kBcmChipTable[0]); i++) {
        if (kBcmChipTable[i].chipNumber == com.sc_chipnumber) {
            tableEntry = &kBcmChipTable[i];
            break;
        }
    }
    if (tableEntry) {
        mDescriptor = *tableEntry;
    } else {
        XYLog("bcm: chip 0x%x not in static table, using personality config\n", com.sc_chipnumber);
        mDescriptor = ChipDescriptor {};
        mDescriptor.deviceId = mDeviceConfig.deviceId;
        mDescriptor.chipNumber = mDeviceConfig.chipNumber ? mDeviceConfig.chipNumber : com.sc_chipnumber;
        mDescriptor.transport = mDeviceConfig.transport;
        mDescriptor.firmwareResourceName = nullptr;
        mDescriptor.sromSlide = 0;
        mDescriptor.needsCortexR4 = mDeviceConfig.needsCortexR4;
        mDescriptor.d11CoreBase = mDeviceConfig.d11CoreBase;
        mDescriptor.chipType = mDeviceConfig.chipType;
    }
    mDescriptor.applyOverride(mDeviceConfig);
    {
        // OpenCore DeviceProperties `openbcm-mode` = fullmac | softmac overrides the personality.
        char mode[16];
        if (bcm_read_str_prop(device, "openbcm-mode", mode, sizeof(mode))) {
            if (!strncasecmp(mode, "fullmac", 7)) mDescriptor.chipType = ChipType::kFullmac;
            else if (!strncasecmp(mode, "softmac", 7)) mDescriptor.chipType = ChipType::kSoftmac;
            XYLog("bcm: openbcm-mode property: %s\n", mode);
        }
    }
    descriptor = &mDescriptor;
    XYLog("bcm: chip 0x%x mode=%s\n", com.sc_chipnumber,
          isFullmac() ? "fullmac" : "softmac");
    cfg.setChipDescriptor(descriptor);

    if (isFullmac()) {
        // FullMAC: Linux brcmfmac firmware over PCIe msgbuf; no olmsg/dlimage/D11 rings.
        if (!fullmac_attach(device))
            return false;
        fullmac_set_mac();
        fullmac_preinit();
        com.sc_fwver = "brcmfmac-fullmac";
        if (!init80211()) {
            XYLog("bcm: net80211 attach failed\n");
            return false;
        }
        com.sc_attached = true;
        return true;
    }

    const UInt8 *fw = nullptr;
    IOByteCount fwLen = 0;
    if (!cfg.requestFirmware(&fw, &fwLen, descriptor)) {
        // Try personality firmware name explicitly (bundle or filesystem)
        if (mDeviceConfig.firmwareResourceName[0] != 0) {
            XYLog("bcm: trying firmware '%s' by name\n", mDeviceConfig.firmwareResourceName);
            if (!cfg.requestFirmwareByName(mDeviceConfig.firmwareResourceName, &fw, &fwLen)) {
                XYLog("bcm: firmware request failed\n");
                return false;
            }
        } else {
            XYLog("bcm: firmware request failed\n");
            return false;
        }
    }

    if (!ol.init(device, com.sc_chipnumber)) {
        XYLog("bcm: olmsg transport init failed\n");
        return false;
    }
    if (!ol.start()) {
        XYLog("bcm: olmsg transport start failed\n");
        return false;
    }
    if (!cfg.enterDownloadState()) {
        XYLog("bcm: R4 halt failed\n");
        return false;
    }
    if (!cfg.publishSharedInfo((UInt32)ol.dmaPhysicalAddress())) {
        XYLog("bcm: shared-info publish failed\n");
        return false;
    }
    if (!cfg.downloadFirmware(fw, fwLen)) {
        XYLog("bcm: firmware download failed\n");
        return false;
    }
    if (cfg.awaitFirmwareReady(2000) == 0) {
        XYLog("bcm: firmware did not boot\n");
        cfg.logBootReport();
        return false;
    }
    cfg.logBootReport();

    com.sc_fwver = "AppleBCM-AC";
    memcpy(com.sc_ic.ic_myaddr, cfg.getMacAddress(), 6);

    if (!init80211()) {
        XYLog("bcm: net80211 attach failed\n");
        return false;
    }

    com.sc_attached = true;
    return true;
}

void ItlBcm::
detach(IOPCIDevice *device)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    (void)device;
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = &ic->ic_ac.ac_if;
    if (com.sc_attached) {
        ifp->if_flags &= ~(IFF_UP | IFF_RUNNING);
        ic->ic_state = IEEE80211_S_INIT;
        ieee80211_ifdetach(ifp);
        com.sc_attached = false;
    }
    releaseAll();
}

void ItlBcm::
free()
{
    XYLog("%s\n", __FUNCTION__);
    super::free();
}

void ItlBcm::
releaseAll()
{
    XYLog("%s\n", __FUNCTION__);
    if (com.sc_ih) {
        if (getMainWorkLoop())
            getMainWorkLoop()->removeEventSource(com.sc_ih);
        com.sc_ih->release();
        com.sc_ih = nullptr;
    }
    if (mRxRing) { mRxRing->stop(); delete mRxRing; mRxRing = nullptr; }
    if (mTxRing) { mTxRing->stop(); delete mTxRing; mTxRing = nullptr; }
    if (mRxPollTimer) {
        if (getMainWorkLoop())
            getMainWorkLoop()->removeEventSource(mRxPollTimer);
        mRxPollTimer->release();
        mRxPollTimer = nullptr;
    }
    if (mScanTimer) {
        mScanTimer->timeout();
        mScanTimer->release();
        mScanTimer = nullptr;
    }
    if (mWlcPollTimer) {
        if (getMainWorkLoop())
            getMainWorkLoop()->removeEventSource(mWlcPollTimer);
        mWlcPollTimer->release();
        mWlcPollTimer = nullptr;
    }
    if (mHostScanTimer) {
        if (getMainWorkLoop())
            getMainWorkLoop()->removeEventSource(mHostScanTimer);
        mHostScanTimer->release();
        mHostScanTimer = nullptr;
    }
    mHostScanActive = false;
    mHostScanCount = 0;
    mHostScanIdx = 0;
    if (mFm) { mFm->stop(); delete mFm; mFm = nullptr; }
    else ol.stop();
}

IOReturn ItlBcm::
enable(IONetworkInterface *netif)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    (void)netif;
    struct _ifnet *ifp = &com.sc_ic.ic_ac.ac_if;
    if (ifp->if_flags & IFF_UP)
        return kIOReturnSuccess;
    ifp->if_flags |= IFF_UP;
    com.sc_running = true;

    if (isFullmac()) {
        // Firmware owns MAC/PHY: no D11 rings, no host RX poll. Only the WLC
        // D2H poll runs (scan results, link/PSK events).
        if (!mWlcPollTimer && getMainWorkLoop()) {
            mWlcPollTimer = IOTimerEventSource::timerEventSource(
                this, ItlBcm::s_wlc_poll_timer);
            if (mWlcPollTimer)
                getMainWorkLoop()->addEventSource(mWlcPollTimer);
        }
        if (mWlcPollTimer) {
            mWlcPollTimer->setTimeoutMS(2);
            mWlcPollTimer->enable();
        }
        return kIOReturnSuccess;
    }

    // softmac: start the D11 data path: RX ring (receive beacons/data) + TX
    // ring (probe requests/data frames).
    uint32_t d11CoreBase = cfg.getD11CoreBase() ? cfg.getD11CoreBase() : 0;
    uint32_t phyBase = d11CoreBase + D11::kD11PhyBaseOffset;

    mRxRing = new Dma64RxRing(&cfg, phyBase, 0, 64, 2048);
    if (!mRxRing->init()) {
        XYLog("bcm: RX ring init failed\n");
        delete mRxRing;
        mRxRing = nullptr;
    }
    mTxRing = new Dma64TxRing(&cfg, 0, 64);
    if (!mTxRing->init()) {
        XYLog("bcm: TX ring init failed\n");
        delete mTxRing;
        mTxRing = nullptr;
    }
    // Always-on RX poll timer: delivers beacons/mgmt/data in SCAN and RUN.
    if (!mRxPollTimer && getMainWorkLoop()) {
        mRxPollTimer = IOTimerEventSource::timerEventSource(
            this, ItlBcm::s_rx_poll);
        if (mRxPollTimer)
            getMainWorkLoop()->addEventSource(mRxPollTimer);
    }
    if (mRxPollTimer) {
        mRxPollTimer->setTimeoutMS(50);
        mRxPollTimer->enable();
    }
    // WLC D2H poll timer: drains olmsg RX for escan results, join results, etc.
    if (!mWlcPollTimer && getMainWorkLoop()) {
        mWlcPollTimer = IOTimerEventSource::timerEventSource(
            this, ItlBcm::s_wlc_poll_timer);
        if (mWlcPollTimer)
            getMainWorkLoop()->addEventSource(mWlcPollTimer);
    }
    if (mWlcPollTimer) {
        mWlcPollTimer->setTimeoutMS(100);
        mWlcPollTimer->enable();
    }
    return kIOReturnSuccess;
}

IOReturn ItlBcm::
disable(IONetworkInterface *netif)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    (void)netif;
    struct _ifnet *ifp = &com.sc_ic.ic_ac.ac_if;
    if (!(ifp->if_flags & IFF_UP))
        return kIOReturnSuccess;
    com.sc_running = false;
    // Stop the D11 data path and WLC timers.
    if (mRxPollTimer) {
        mRxPollTimer->cancelTimeout();
        mRxPollTimer->disable();
    }
    if (mWlcPollTimer) {
        mWlcPollTimer->cancelTimeout();
        mWlcPollTimer->disable();
    }
    if (mHostScanTimer) {
        mHostScanTimer->cancelTimeout();
        mHostScanTimer->disable();
    }
    mHostScanActive = false;
    if (mRxRing) { mRxRing->stop(); delete mRxRing; mRxRing = nullptr; }
    if (mTxRing) { mTxRing->stop(); delete mTxRing; mTxRing = nullptr; }
    bcm_scan_done();
    ifp->if_flags &= ~IFF_UP;
    return kIOReturnSuccess;
}

struct ieee80211com *ItlBcm::
get80211Controller()
{
    return &com.sc_ic;
}

ItlDriverInfo *ItlBcm::
getDriverInfo()
{
    return this;
}

ItlDriverController *ItlBcm::
getDriverController()
{
    return this;
}

// ---------------------------------------------------------------------------
// ItlDriverInfo
// ---------------------------------------------------------------------------

const char *ItlBcm::
getFirmwareVersion()
{
    return com.sc_fwver ? com.sc_fwver : "unknown";
}

const char *ItlBcm::
getFirmwareName()
{
    return descriptor ? descriptor->firmwareResourceName : "apple-bcm";
}

int16_t ItlBcm::
getBSSNoise()
{
    return -95;
}

bool ItlBcm::
is5GBandSupport()
{
    return true;
}

int ItlBcm::
getTxNSS()
{
    return 1;
}

UInt32 ItlBcm::
supportedFeatures()
{
    return kIONetworkFeatureMultiPages;
}

const char *ItlBcm::
getFirmwareCountryCode()
{
    return "XX";
}

uint32_t ItlBcm::
getTxQueueSize()
{
    return 256;
}

// ---------------------------------------------------------------------------
// ItlDriverController
// ---------------------------------------------------------------------------

void ItlBcm::
clearScanningFlags()
{
    com.sc_scanning = false;
    mWlcScanActive = false;
    if (mScanTimer) {
        mScanTimer->cancelTimeout();
        mScanTimer->disable();
    }
    if (mRxPollTimer && !com.sc_running) {
        mRxPollTimer->cancelTimeout();
        mRxPollTimer->disable();
    }
    if (mWlcPollTimer) {
        mWlcPollTimer->cancelTimeout();
        mWlcPollTimer->disable();
    }
}

IOReturn ItlBcm::
setMulticastList(IOEthernetAddress *addr, int count)
{
    // Program the multicast filter via WLC (WLC_SET_MCAST_LIST). For
    // host-driven SoftMAC, multicast filtering is done in software;
    // accept all multicast frames and let net80211 filter.
    if (!addr || count < 0) return kIOReturnError;
    XYLog("%s: %d multicast addresses\n", __FUNCTION__, count);
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// WLC/BCDC control plane via olmsg (timer-driven D2H poll)
// ---------------------------------------------------------------------------

void ItlBcm::
s_wlc_poll_timer(OSObject *target, IOTimerEventSource *sender)
{
    (void)sender;
    ItlBcm *self = OSDynamicCast(ItlBcm, target);
    if (!self) return;
    self->bcm_wlc_poll_d2h();
    // Re-arm: keep polling while the interface is up.
    if (self->mWlcPollTimer && self->com.sc_running) {
        // fullmac polls the msgbuf rings, so use a short period (interrupts replace this later).
        self->mWlcPollTimer->setTimeoutMS(self->isFullmac() ? 2 : 100);
        self->mWlcPollTimer->enable();
    }
}
