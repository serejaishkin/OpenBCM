/*
 * OpenBCM — Broadcom BCM4360/43602/4350 HAL service for the itlwm Airport
 * framework. This is the BCM counterpart of ItlIwm/ItlIwn/ItlIwx: it is created
 * by IOPCIEDeviceWrapper for Broadcom PCIe IDs and handed to AirportItlwm as
 * its ItlHalService.
 *
 * Control plane: AppleCore `_wlc_ol_up` BCDC/olmsg bring-up (ChipConfigurator +
 * BcdcOlTransport, ported from the standalone AppleBCMWLANCompanion project).
 * Network glue: the very same net80211 (itl80211/openbsd) the AirportItlwm
 * skywalk/WCL layer already speaks.
 *
 * Data plane: host-driven D11 SoftMAC `dma64` TX/RX rings (Dma64RxRing /
 * Dma64TxRing).  The host drives the radio directly, so scanning works with
 * *any* firmware (Linux/OpenBCD/whatever) — we send probe requests ourselves
 * and receive beacons via the RX ring, injecting them into net80211 via
 * ieee80211_inputm().
 *
 * STATUS: bring-up + net80211 attach + D11 RX/TX rings implemented; scan
 * state machine + PHY channel switching pending per-chip RE.
 */

#ifndef ItlBcm_hpp
#define ItlBcm_hpp

#include <compat.h>
#include <linux/kernel.h>

#include <IOKit/IOWorkLoop.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/IOEventSource.h>
#include <IOKit/IOTimerEventSource.h>
#include <libkern/c++/OSString.h>
#include <IOKit/IOService.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOLib.h>
#include <libkern/OSKextLib.h>
#include <libkern/c++/OSMetaClass.h>
#include <IOKit/IOFilterInterruptEventSource.h>

#include <HAL/ItlHalService.hpp>
#include <HAL/ItlDriverInfo.hpp>
#include <HAL/ItlDriverController.hpp>

#include "ChipConfigurator.hpp"
#include "Transport.hpp"
#include "D11.hpp"
#include "Dma64RxRing.hpp"
#include "Dma64TxRing.hpp"

struct bcm_node {
    struct ieee80211_node ni;
};

class ItlBcm;

// Per-device soft context, embedded in ItlBcm. Mirrors the role of iwm_softc:
// it owns the ieee80211com and the interface glue.
struct bcm_softc {
    struct ieee80211com sc_ic;
    UInt32 sc_chipnumber;
    UInt32 sc_chiprev;
    IOInterruptEventSource *sc_ih;
    bool sc_attached;
    bool sc_running;
    bool sc_scanning;     // a net80211 scan (fg or bg) is in progress
    uint8_t  sc_scan_list[64]; // active scan channel list (built at start)
    uint8_t  sc_scan_count;    // entries in sc_scan_list
    uint8_t  sc_scan_idx;      // next channel index to visit
    const char *sc_fwver;
    int (*sc_newstate)(struct ieee80211com *, enum ieee80211_state, int);
    ItlBcm *sc_driver;    // back-pointer to owning ItlBcm (set in attach)
};

class ItlBcm : public ItlHalService, ItlDriverInfo, ItlDriverController {
    OSDeclareDefaultStructors(ItlBcm)

public:
    // ItlHalService
    void free() override;
    bool attach(IOPCIDevice *device) override;
    void detach(IOPCIDevice *device) override;
    IOReturn enable(IONetworkInterface *netif) override;
    IOReturn disable(IONetworkInterface *netif) override;
    struct ieee80211com *get80211Controller() override;
#ifdef AIRPORT_WCL
    IOReturn setScanHomeAwayTime(uint32_t milliseconds) override {
        com.sc_ic.ic_wcl_scan_away_time_ms = milliseconds;
        return kIOReturnSuccess;
    }
#endif

    // ItlDriverInfo
    const char *getFirmwareVersion() override;
    int16_t getBSSNoise() override;
    bool is5GBandSupport() override;
    int getTxNSS() override;
    const char *getFirmwareName() override;
    UInt32 supportedFeatures() override;
    const char *getFirmwareCountryCode() override;
    uint32_t getTxQueueSize() override;

    // ItlDriverController
    void clearScanningFlags() override;
    IOReturn setMulticastList(IOEthernetAddress *addr, int count) override;

    // net80211 RX/TX plumbing (Group B). bcm_rx_input is the injection point
    // called by the D11 RX interrupt/poll path; bcm_tx_start dequeues if_snd.
    void bcm_rx_input(mbuf_t m, int rssi, uint8_t chan);
    void bcm_tx_start(struct _ifnet *ifp);
    void bcm_rx_poll();   // drain RX ring -> bcm_rx_input (scan + RUN)

    // D11 dma64 rings (real IOKit DMA, constructed in enable()).
    Dma64RxRing *mRxRing {nullptr};
    Dma64TxRing *mTxRing {nullptr};

    // Scan state machine (host-driven, works with any firmware).
    void bcm_scan_start();                     // called from bcm_bgscan_start
    void bcm_scan_next_channel();              // hop + probe + poll RX
    void bcm_scan_done();                      // ieee80211_end_scan + cleanup

    // PCI match predicate used by IOPCIEDeviceWrapper::probe.
    static bool bcm_match(IOPCIDevice *device);

    void releaseAll();

private:
    bcm_softc com;
    ChipConfigurator cfg;
    BcdcOlTransport ol;
    const ChipDescriptor *descriptor {nullptr};

    // Scan timer: fires periodically during a scan to hop channels.
    IOTimerEventSource *mScanTimer {nullptr};
    static void s_scan_timer(OSObject *target, IOTimerEventSource *sender);
    static void s_rxDeliver(void *ctx, mbuf_t m, int8_t rssi, uint8_t chan);
    // Always-on RX poll timer (50 ms): keeps data/mgmt flowing in RUN state.
    IOTimerEventSource *mRxPollTimer {nullptr};
    static void s_rx_poll(OSObject *target, IOTimerEventSource *sender);

    bool init80211();
};

#endif /* ItlBcm_hpp */
