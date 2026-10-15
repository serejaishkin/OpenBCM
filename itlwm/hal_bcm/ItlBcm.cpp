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
 *
 * NOT YET IMPLEMENTED (Group B, see ItlBcm.hpp header note):
 *   - D11 SoftMAC data path (dma64 TX/RX rings): bcm_start / RX -> ieee80211_input
 *   - WLC scan/join translation: bcm_bgscan_start / bcm_newstate
 *   - key programming: bcm_set_key / bcm_delete_key
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

    if (nstate == IEEE80211_S_SCAN)
        sc->sc_scanning = true;
    else if (ostate == IEEE80211_S_SCAN)
        sc->sc_scanning = false;

    // The driver owns the SCAN transition: net80211 hops channels by
    // re-entering this callback with nstate == IEEE80211_S_SCAN. Do not
    // chain to the stock state machine while scanning.
    if (nstate == IEEE80211_S_SCAN) {
        ic->ic_state = IEEE80211_S_SCAN;
        // TODO(Group B): tune the D11 radio to the current scan channel and
        // enable the RX ring. Without D11 PHY/ucode this is a no-op.
        XYLog("%s: entering SCAN state (ostate=%u)\n",
              __FUNCTION__, (unsigned)ostate);
        return 0;
    }
    return sc->sc_newstate(ic, nstate, arg);
}

static int
bcm_bgscan_start(struct ieee80211com *ic)
{
    struct _ifnet *ifp = IC2IFP(ic);
    struct bcm_softc *sc = (struct bcm_softc *)ifp->if_softc;
    // Background scan (state stays RUN): start D11 RX on the current channel
    // and begin channel hopping; completed frames are fed back via
    // bcm_rx_input(). The driver calls ieee80211_end_scan(ifp) when done.
    // TODO(Group B): program the D11 radio + RX ring; without D11 PHY/ucode
    // no frames arrive and the scan finds nothing.
    sc->sc_scanning = true;
    XYLog("%s: background scan requested (D11 RX not yet wired)\n", __FUNCTION__);
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
    sc->bcm_tx_start(ifp);
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
    struct mbuf *m;
    while ((m = ifq_dequeue(&ifp->if_snd)) != nullptr) {
        mbuf_freem(m);
        ifp->if_oerrors++;  // dropped: no D11 TX path yet
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

    if (!cfg.initWithDevice(device)) {
        XYLog("bcm: ChipConfigurator init failed\n");
        return false;
    }
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
    // TODO(Group B): start the D11 MAC + data path.
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
    // TODO(Group B): stop the D11 MAC + data path.
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
    // TODO(Group B): clear the driver-side scanning flag once bgscan lands.
}

IOReturn ItlBcm::
setMulticastList(IOEthernetAddress *addr, int count)
{
    (void)addr; (void)count;
    // TODO(Group B): program the multicast filter via WLC.
    return kIOReturnSuccess;
}
