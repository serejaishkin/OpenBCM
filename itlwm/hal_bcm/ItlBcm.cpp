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
 * D11 data path (implemented): Dma64RxRing / Dma64TxRing host-driven dma64
 * rings with real IOKit DMA; RX frames injected via bcm_rx_input() ->
 * ieee80211_inputm().
 *
 * Host-driven scan (implemented skeleton): bcm_bgscan_start() ->
 * bcm_scan_start() -> timer-driven channel hopping -> probe requests via
 * TX ring -> beacon RX via RX ring -> ieee80211_inputm() ->
 * ieee80211_end_scan().  PHY channel switching is stubbed pending per-chip
 * register RE.
 *
 * NOT YET IMPLEMENTED (Group B, see ItlBcm.hpp header note):
 *   - PHY channel switching registers (D11 PHY channel regs TBD)
 *   - key programming: bcm_set_key / bcm_delete_key
 *   - WPA3/SAE, PMF, AWDL/AirDrop, MLO
 */

#include "ItlBcm.hpp"

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
    switch (dev) {
        case 0x43a0:    // BCM4360  (BCM94360CS/CS2/CD/HMB)
        case 0x43ba:    // BCM43602 (BCM943602CS/CDP/BAED, DW1830)
        case 0x43a3:    // BCM4350  (BCM94350ZAE, DW1820A)
            return true;
        default:
            return false;
    }
}

void ItlBcm::
setDeviceConfig(const ChipDeviceConfig &cfg)
{
    mDeviceConfig = cfg;
    cfg.setDeviceConfig(cfg);
}

// ---------------------------------------------------------------------------
// net80211 device callbacks (stubs for Group B)
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
    // Join path: AUTH/ASSOC/RUN frames flow through bcm_start (TX ring) and
    // bcm_rx_poll -> bcm_rx_input (RX ring); stock net80211 advances the
    // state when responses arrive. Just chain up and log progress.
    if (nstate == IEEE80211_S_AUTH || nstate == IEEE80211_S_ASSOC ||
        nstate == IEEE80211_S_RUN) {
        XYLog("%s: join progress ostate=%u -> nstate=%u\n",
              __FUNCTION__, (unsigned)ostate, (unsigned)nstate);
    }
    return sc->sc_newstate(ic, nstate, arg);
    }

    // The driver owns the SCAN transition: net80211 hops channels by
    // re-entering this callback with nstate == IEEE80211_S_SCAN. Do not
    // chain to the stock state machine while scanning. Only the first
    // SCAN entry starts the timer-driven scan; SCAN->SCAN re-entries
    // (net80211 channel hops) are ignored — our timer does the hopping.
    if (nstate == IEEE80211_S_SCAN) {
        ic->ic_state = IEEE80211_S_SCAN;
        if (ostate != IEEE80211_S_SCAN) {
            XYLog("%s: entering SCAN state (ostate=%u)\n",
                  __FUNCTION__, (unsigned)ostate);
            if (sc && sc->sc_driver)
                sc->sc_driver->bcm_scan_start();
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
    // Host-driven scan (works with any firmware): the host sends probe
    // requests and receives beacons via the D11 dma64 RX ring, injecting
    // frames into net80211 via ieee80211_inputm().
    XYLog("%s: background scan requested\n", __FUNCTION__);
    if (sc && sc->sc_driver)
        sc->sc_driver->bcm_scan_start();
    return 0;
}

static int
bcm_set_key(struct ieee80211com *ic, struct ieee80211_node *ni, struct ieee80211_key *k)
{
    // TODO(Group B): program the key into the device key table (WLC_SET_KEY).
    (void)ic; (void)ni; (void)k;
    return 0;
}

static void
bcm_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni, struct ieee80211_key *k)
{
    // TODO(Group B): remove the key from the device key table.
    (void)ic; (void)ni; (void)k;
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
// net80211 RX injection (called by the future D11 RX interrupt handler).
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
// ring. Stubbed until the D11 core base / BAR1 window is resolved.
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_tx_start(struct _ifnet *ifp)
{
    // Dequeue data/mgmt frames from if_snd and push them onto the D11 TX
    // dma64 ring. net80211 (incl. AUTH/ASSOC mgmt via ieee80211_send_mgmt)
    // funnels everything through here, so wiring this completes the join TX
    // path; RX completions arrive via bcm_rx_poll() -> bcm_rx_input().
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
// Scan completion helper. Called by the D11 RX/timeout path when a scan
// finishes so net80211 leaves SCAN state and reports results.
// ---------------------------------------------------------------------------

void ItlBcm::
bcm_scan_end()
{
    struct _ifnet *ifp = &com.sc_ic.ic_ac.ac_if;
    com.sc_scanning = false;
    ieee80211_end_scan(ifp);
}

// 5 GHz channel numbers of the BCM4360/43602/4350 (all dual-band parts).
static const uint8_t kBcmChannels5GHz[] = {
    36,40,44,48,52,56,60,64,
    100,104,108,112,116,120,124,128,132,136,140,144,
    149,153,157,161,165
};

// Fill ic_channels[] (freq + flags) so ieee80211_media_init() builds
// ic_chan_avail and the scan list derives from net80211's regdomain view
// instead of a hardcoded table. Mirrors iwn's EEPROM channel fill
// (ItlIwn.cpp), except the per-board source on Broadcom is SROM/OTP/NVRAM
// (readSROM() is still a stub) rather than the iwn EEPROM.
static void
bcm_setup_channels(struct ieee80211com *ic)
{
    int i;
    for (i = 1; i <= 14; i++) {
        ic->ic_channels[i].ic_freq =
            ieee80211_ieee2mhz((u_int)i, IEEE80211_CHAN_2GHZ);
        ic->ic_channels[i].ic_flags =
            IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
            IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ |
            IEEE80211_CHAN_HT;
    }
    for (i = 0; i < (int)sizeof(kBcmChannels5GHz); i++) {
        uint8_t ch = kBcmChannels5GHz[i];
        ic->ic_channels[ch].ic_freq =
            ieee80211_ieee2mhz(ch, IEEE80211_CHAN_5GHZ);
        ic->ic_channels[ch].ic_flags =
            IEEE80211_CHAN_OFDM | IEEE80211_CHAN_5GHZ |
            IEEE80211_CHAN_HT;
    }
}

// ---------------------------------------------------------------------------
// net80211 attach
// ---------------------------------------------------------------------------

bool ItlBcm::
init80211()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = &ic->ic_ac.ac_if;
    int i;

    ic->ic_phytype = IEEE80211_T_OFDM;    /* not only, but not used */
    ic->ic_opmode = IEEE80211_M_STA;      /* default to BSS mode */
    ic->ic_state = IEEE80211_S_INIT;

    /* Set device capabilities. */
    ic->ic_caps =
        IEEE80211_C_WEP |             /* WEP */
        IEEE80211_C_RSN |             /* WPA/RSN */
        IEEE80211_C_SCANALL |         /* scans all channels at once */
        IEEE80211_C_SCANALLBAND |     /* scans all bands at once */
        IEEE80211_C_MONITOR |         /* monitor mode supported */
        IEEE80211_C_SHSLOT |          /* short slot time supported */
        IEEE80211_C_SHPREAMBLE;       /* short preamble supported */

    ic->ic_htcaps = IEEE80211_HTCAP_SGI20;
    ic->ic_htcaps |= (IEEE80211_HTCAP_SMPS_DIS << IEEE80211_HTCAP_SMPS_SHIFT);
    ic->ic_htcaps |= (IEEE80211_HTCAP_CBW20_40 | IEEE80211_HTCAP_SGI40);
    ic->ic_htxcaps = 0;
    ic->ic_txbfcaps = 0;
    ic->ic_aselcaps = 0;
    ic->ic_ampdu_params = (IEEE80211_AMPDU_PARAM_SS_4 | 0x3 /* 64k */);
    ic->ic_caps |= (IEEE80211_C_QOS | IEEE80211_C_TX_AMPDU | IEEE80211_C_AMSDU_IN_AMPDU);

    ic->ic_sup_rates[IEEE80211_MODE_11A] = ieee80211_std_rateset_11a;
    ic->ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
    ic->ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;

    /* Channel table (freq + flags) for ieee80211_media_init() ->
     * ic_chan_avail. Must run before if_attach/media_init below. */
    bcm_setup_channels(ic);

    /* IBSS channel undefined for now. */
    ic->ic_ibss_chan = &ic->ic_channels[1];
    ic->ic_max_rssi = 100;    /* full-scale RSSI percentage (as in iwm) */

    ifp->if_softc = &com;
    ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST | IFF_DEBUG;
    ifp->if_ioctl = ieee80211_ioctl;
    ifp->if_start = bcm_start;
    ifp->if_watchdog = bcm_watchdog;
    memcpy(ifp->if_xname, "bcm0", IFNAMSIZ);

    if_attach(ifp);
    ieee80211_ifattach(ifp, getController());
    ieee80211_media_init(ifp);

    ic->ic_node_alloc = bcm_node_alloc;
    ic->ic_bgscan_start = bcm_bgscan_start;
    ic->ic_set_key = bcm_set_key;
    ic->ic_delete_key = bcm_delete_key;

    /* Override 802.11 state transition machine. */
    com.sc_newstate = ic->ic_newstate;
    ic->ic_newstate = bcm_newstate;

    (void)i;
    return true;
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
    if (!descriptor) {
        XYLog("bcm: unsupported chip 0x%x\n", com.sc_chipnumber);
        return false;
    }
    cfg.setChipDescriptor(descriptor);

    const UInt8 *fw = nullptr;
    IOByteCount fwLen = 0;
    if (!cfg.requestFirmware(&fw, &fwLen, descriptor)) {
        XYLog("bcm: firmware request failed\n");
        return false;
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
    // (probe requests for scanning).
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
    // Stop the D11 data path.
    if (mRxPollTimer) {
        mRxPollTimer->cancelTimeout();
        mRxPollTimer->disable();
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
    if (mScanTimer) {
        mScanTimer->disable();
    }
}

IOReturn ItlBcm::
setMulticastList(IOEthernetAddress *addr, int count)
{
    (void)addr; (void)count;
    // TODO(Group B): program the multicast filter via WLC.
    return kIOReturnSuccess;
}

// ---------------------------------------------------------------------------
// Host-driven scan state machine (D11 dma64 RX/TX rings).
// Works with *any* firmware: host sends probe requests, receives beacons,
// injects frames into net80211 via ieee80211_inputm().
// ---------------------------------------------------------------------------

// Full scan channel table: 2.4 GHz 1-14 + 5 GHz UNII-1/2/2e/3.
static const uint8_t kAllScanChannels[] = {
    1,2,3,4,5,6,7,8,9,10,11,12,13,14,
    36,40,44,48,52,56,60,64,100,104,108,112,116,120,124,128,132,136,140,144,
    149,153,157,161,165
};
static constexpr uint32_t kAllScanChannelCount =
    sizeof(kAllScanChannels) / sizeof(kAllScanChannels[0]);

// Minimal 802.11 Probe Request frame (broadcast SSID).
struct ProbeReqFrame {
    UInt16  fc;       // 0x0040 mgmt probe-req
    UInt16  dur;      // 0
    UInt8   da[6];    // ff:ff:ff:ff:ff:ff
    UInt8   sa[6];    // our MAC (locally administered)
    UInt8   bssid[6]; // ff:ff:ff:ff:ff:ff
    UInt16  seq;      // 0
    // IEs:
    UInt8   ssid_ie[2];   // 0x00, 0x00
    UInt8   rates_ie[8];  // 0x01, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60
};

static const ProbeReqFrame kProbeReqTemplate = {
    0x0040, 0x0000,
    {0xff,0xff,0xff,0xff,0xff,0xff},
    {0x02,0x00,0x00,0x00,0x00,0x01},
    {0xff,0xff,0xff,0xff,0xff,0xff},
    0x0000,
    {0x00, 0x00},
    {0x01, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60}
};

static constexpr size_t kProbeReqSize = sizeof(ProbeReqFrame);

void ItlBcm::
s_scan_timer(OSObject *target, IOTimerEventSource *sender)
{
    ItlBcm *self = OSDynamicCast(ItlBcm, target);
    if (!self) return;
    self->bcm_scan_next_channel();
}

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
        // Defensive fallback: 2.4 GHz only, so a scan still attempts
        // something if the channel table came up empty.
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

    // Create scan timer on the main workloop.
    if (!mScanTimer && getMainWorkLoop()) {
        mScanTimer = IOTimerEventSource::timerEventSource(
            this, ItlBcm::s_scan_timer);
        if (mScanTimer) {
            getMainWorkLoop()->addEventSource(mScanTimer);
        }
    }
    if (mScanTimer) {
        mScanTimer->setTimeoutMS(100); // 100 ms per channel
        mScanTimer->enable();
    }
    XYLog("%s: scan started (%u channels)\n",
          __FUNCTION__, com.sc_scan_count);
}

void ItlBcm::
bcm_scan_next_channel()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    if (!com.sc_scanning) return;
    if (com.sc_scan_idx >= com.sc_scan_count) {
        bcm_scan_done();
        return;
    }
    uint8_t chan = com.sc_scan_list[com.sc_scan_idx];

    // 1. Switch D11 radio to current channel (PHY RE stub — no-op until
    //    register map is decoded).
    XYLog("%s: channel %u (%u/%u, PHY switch TBD)\n",
          __FUNCTION__, chan, com.sc_scan_idx + 1, com.sc_scan_count);
    if (mRxRing)
        mRxRing->setCurrentChan(chan);

    // 2. Send probe request on current channel.
    if (mTxRing) {
        if (!mTxRing->enqueue(&kProbeReqTemplate, kProbeReqSize)) {
            XYLog("%s: TX ring full\n", __FUNCTION__);
        }
        mTxRing->pollTx();
    }

    // 3. Poll RX ring for received beacons / probe responses.
    if (mRxRing) {
        mRxRing->poll(this, ItlBcm::s_rxDeliver);
    }

    // 4. Advance to next channel or finish.
    com.sc_scan_idx++;
    if (com.sc_scan_idx < com.sc_scan_count && mScanTimer) {
        mScanTimer->setTimeoutMS(100);
        mScanTimer->enable();
    } else {
        bcm_scan_done();
    }
}

void ItlBcm::
bcm_scan_done()
{
    struct ieee80211com *ic = &com.sc_ic;
    struct _ifnet *ifp = IC2IFP(ic);

    com.sc_scanning = false;
    if (mScanTimer) {
        mScanTimer->disable();
    }

    // net80211 leaves SCAN state and reports results to WCL/menu.
    XYLog("%s: scan complete, leaving SCAN state\n", __FUNCTION__);
    ieee80211_end_scan(ifp);
}

