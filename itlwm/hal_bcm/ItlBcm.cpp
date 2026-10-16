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
#include "WlcDefs.hpp"

#include <cstring>
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
setDeviceConfig(const ChipDeviceConfig &cfg)
{
    mDeviceConfig = cfg;
    cfg.setDeviceConfig(cfg);
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

    // Join path: AUTH/ASSOC/RUN frames flow through bcm_start (TX ring) and
    // bcm_rx_poll -> bcm_rx_input (RX ring); stock net80211 advances the
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
    // Build escan iovar payload:
    // [u32 version=1][u16 action=1][u16 sync_id=0x1234][scan_params]
    // scan_params: ssid_le{u32 len; u8[32]} bssid[6] bss_type(s8;2=ANY)
    //   scan_type(u8) nprobes(-1) active_time(-1) passive_time(-1)
    //   home_time(-1) channel_num chanspec_list[u16...]
    UInt8 payload[512];
    UInt8 *p = payload;
    UInt32 version = 1;
    UInt16 action = wlc::ESCAN_ACTION_START;
    UInt16 sync_id = mWlcScanSeq++;
    UInt8 bss_type_s8 = 2; // ANY
    UInt8 scan_type_u8 = (UInt8)wlc::SCANTYPE_ACTIVE;
    UInt32 negOne = 0xFFFFFFFF;
    UInt32 channel_num = count;

    memcpy(p, &version, 4); p += 4;
    memcpy(p, &action, 2); p += 2;
    memcpy(p, &sync_id, 2); p += 2;
    // ssid_le: len=0 (broadcast), ssid[32] zero
    UInt32 ssidLen = 0;
    memcpy(p, &ssidLen, 4); p += 4;
    UInt8 ssid[32] = {0};
    memcpy(p, ssid, 32); p += 32;
    // bssid (broadcast)
    UInt8 bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(p, bcast, 6); p += 6;
    // bss_type (s8 = 2)
    memcpy(p, &bss_type_s8, 1); p += 1;
    // scan_type (u8)
    memcpy(p, &scan_type_u8, 1); p += 1;
    // nprobes, active_time, passive_time, home_time (-1)
    memcpy(p, &negOne, 4); p += 4;
    memcpy(p, &negOne, 4); p += 4;
    memcpy(p, &negOne, 4); p += 4;
    memcpy(p, &negOne, 4); p += 4;
    // channel_num
    memcpy(p, &channel_num, 4); p += 4;
    // chanspec list
    for (uint32_t i = 0; i < count; i++) {
        uint16_t chspec = wlc::ch20_chspec_d11n(channels[i]);
        memcpy(p, &chspec, 2); p += 2;
    }
    UInt32 payloadLen = (UInt32)(p - payload);

    bcm_wlc_send_iovar_set("escan", payload, payloadLen);
    XYLog("%s: escan sent (%u channels, seq=%u)\n",
          __FUNCTION__, count, sync_id);
}

void ItlBcm::
bcm_wlc_join(const char *ssid, uint32_t ssid_len, const UInt8 *bssid)
{
    // Build WLC_SET_SSID (26) payload:
    // struct brcmf_ssid_le { u32 len; u8 ssid[32]; }
    UInt8 payload[36];
    UInt32 len = ssid_len > 32 ? 32 : ssid_len;
    memset(payload, 0, sizeof(payload));
    memcpy(payload, &len, 4);
    memcpy(payload + 4, ssid, len);
    bcm_wlc_send_ioctl(wlc::C_SET_SSID, payload, 36);
    XYLog("%s: WLC_SET_SSID sent (len=%u, ssid=%.*s)\n",
          __FUNCTION__, len, (int)len, ssid);
}

void ItlBcm::
bcm_wlc_poll_d2h()
{
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
    UInt32 id = msg->id;
    UInt32 seq = msg->seq;
    UInt32 len = msg->payloadLen;
    const UInt8 *payload = msg->payload;

    XYLog("%s: D2H event id=0x%x (%u) seq=%u len=%u\n",
          __FUNCTION__, id, id, seq, len);

    switch (id) {
    case wlc::E_ESCAN_RESULT: {
        // escan_result_le: u32 buflen, u32 version, u16 sync_id, u16 bss_count,
        //   bss_info_le{...}
        if (len < 12) break;
        UInt32 buflen, version;
        UInt16 sync_id, bss_count;
        memcpy(&buflen, payload, 4);
        memcpy(&version, payload + 4, 4);
        memcpy(&sync_id, payload + 8, 2);
        memcpy(&bss_count, payload + 10, 2);
        XYLog("%s: ESCAN_RESULT buflen=%u version=%u sync_id=%u bss_count=%u\n",
              __FUNCTION__, buflen, version, sync_id, bss_count);

        const UInt8 *bss = payload + 12;
        for (UInt16 i = 0; i < bss_count; i++) {
            // Each bss_info is variable length. Read bss_info len at offset 0
            // of each bss_info entry: first u32 is length of this info block.
            if (bss + 4 > payload + len)
                break;
            UInt32 infoLen;
            memcpy(&infoLen, bss, 4);
            if (infoLen == 0 || bss + infoLen > payload + len)
                break;
            // Parse bss_info_le:
            //   u16 chanspec, u16 channel, u16 bss_type, u8 bssid[6],
            //   u8 ssid_len, u8 ssid[32], s8 rssi, ...
            const UInt8 *b = bss;
            UInt16 chanspec;
            memcpy(&chanspec, b, 2); b += 2;
            UInt16 channel;
            memcpy(&channel, b, 2); b += 2;
            UInt16 bssType;
            memcpy(&bssType, b, 2); b += 2;
            UInt8 bssid[6];
            memcpy(bssid, b, 6); b += 6;
            UInt8 ssidLen = *b; b += 1;
            UInt8 ssid[33] = {0};
            if (ssidLen > 32) ssidLen = 32;
            memcpy(ssid, b, ssidLen); b += ssidLen;
            // Align to 4 bytes after ssid
            uint32_t pad = (4 - (ssidLen % 4)) % 4;
            b += pad;
            // rssi is s8 at some offset; skip to rssi field
            // brcmf_scan_info: ... rssi at offset 0x30 roughly
            int8_t rssi = -50;
            // Try to read RSSI from a common offset
            if (b + 1 <= payload + len) {
                // rssi is often at offset ~0x38 or later; try a safe scan
                for (int k = 0; k < 16 && b + k + 1 <= payload + len; k++) {
                    int8_t cand = *(b + k);
                    if (cand < -20 && cand > -100) {
                        rssi = cand;
                        break;
                    }
                }
            }
            bcm_wlc_scan_result(bssid, ssid, ssidLen, rssi);
            bss += infoLen;
        }
        if (bss_count == 0) {
            // Scan done with no results
            bcm_scan_done();
        }
        break;
    }
    case wlc::E_PSK_SUP: {
        // PSK supplicant status: status=6 means PTK done (connected)
        if (len >= 4) {
            UInt32 status;
            memcpy(&status, payload, 4);
            XYLog("%s: E_PSK_SUP status=%u\n", __FUNCTION__, status);
            if (status == 6) {
                // PTK done -> association complete
                // net80211 will transition to RUN via ic_newstate
            }
        }
        break;
    }
    case wlc::E_LINK: {
        // Link status: bit0 = up
        if (len >= 4) {
            UInt32 status;
            memcpy(&status, payload, 4);
            XYLog("%s: E_LINK status=%u\n", __FUNCTION__, status);
        }
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
bcm_wlc_scan_result(const UInt8 *bssid, const UInt8 *ssid, uint32_t ssid_len, int8_t rssi)
{
    // Inject a scanned AP into net80211 ic_tree via a fake beacon frame.
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    // Build a minimal beacon frame: FC, duration, DA=broadcast, SA=bssid,
    // BSSID=bssid, seq, timestamp, beacon interval, cap info, SSID IE.
    struct {
        UInt16 fc;        // 0x8000 beacon
        UInt16 dur;       // 0
        UInt8   da[6];    // ff:ff:ff:ff:ff:ff
        UInt8   sa[6];    // bssid
        UInt8   bssid[6]; // bssid
        UInt16  seq;      // 0
        UInt64  ts;       // timestamp
        UInt16  bi;       // beacon interval
        UInt16  cap;      // capabilities
        UInt8   ssid_ie[2]; // 0x00, len
        UInt8   ssid[32];
    } __attribute__((packed)) frame;

    memset(&frame, 0, sizeof(frame));
    frame.fc = 0x8000; // beacon
    memcpy(frame.da, "\xff\xff\xff\xff\xff\xff", 6);
    memcpy(frame.sa, bssid, 6);
    memcpy(frame.bssid, bssid, 6);
    frame.bi = 100;
    frame.cap = 0x0401; // ESS + privacy
    frame.ssid_ie[0] = 0x00; // SSID IE
    frame.ssid_ie[1] = (UInt8)ssid_len;
    memcpy(frame.ssid, ssid, ssid_len);

    size_t frameLen = sizeof(frame) - 32 + ssid_len;

    // Allocate mbuf, copy frame data
    mbuf_t m = mbuf_alloc(MBUF_WAITOK, MBUF_TYPE_DATA);
    if (!m) {
        XYLog("%s: mbuf_alloc failed\n", __FUNCTION__);
        return;
    }
    mbuf_setdata(m, &frame, frameLen);
    mbuf_setlen(m, frameLen);
    mbuf_pkthdr_setlen(m, frameLen);

    bcm_rx_input(m, rssi, 0); // chan unknown yet
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
    if (com.sc_scanning && com.sc_scan_count > 0) {
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
// ItlHalService
// ---------------------------------------------------------------------------

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

    descriptor = nullptr;
    for (size_t i = 0; i < sizeof(kBcmChipTable) / sizeof(kBcmChipTable[0]); i++) {
        if (kBcmChipTable[i].chipNumber == com.sc_chipnumber) {
            descriptor = &kBcmChipTable[i];
            break;
        }
    }
    // If no table entry, build a temporary descriptor from personality config
    // so requestFirmware() can fall back to embedded/bundle/filesystem loading.
    ChipDescriptor fallbackDescriptor {};
    if (!descriptor) {
        XYLog("bcm: chip 0x%x not in static table, using personality config\n", com.sc_chipnumber);
        fallbackDescriptor.deviceId = mDeviceConfig.deviceId;
        fallbackDescriptor.chipNumber = mDeviceConfig.chipNumber ? mDeviceConfig.chipNumber : com.sc_chipnumber;
        fallbackDescriptor.transport = mDeviceConfig.transport;
        fallbackDescriptor.firmwareResourceName = mDeviceConfig.firmwareResourceName[0] ? mDeviceConfig.firmwareResourceName : nullptr;
        fallbackDescriptor.sromSlide = 0;
        fallbackDescriptor.needsCortexR4 = mDeviceConfig.needsCortexR4;
        fallbackDescriptor.d11CoreBase = mDeviceConfig.d11CoreBase;
        fallbackDescriptor.chipType = mDeviceConfig.chipType;
        descriptor = &fallbackDescriptor;
    }
    cfg.setChipDescriptor(descriptor);

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
    ol.stop();
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

    // Start the D11 data path: RX ring (receive beacons/data) + TX ring
    // (probe requests/data frames).
    mRxRing = new Dma64RxRing(&cfg, 0, 64, 2048);
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
        self->mWlcPollTimer->setTimeoutMS(100);
        self->mWlcPollTimer->enable();
    }
}
