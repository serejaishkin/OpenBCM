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
 * STATUS: bring-up + net80211 attach implemented; the D11 data path and the
 * WLC scan/join translation are stubs (see the TODO markers in ItlBcm.cpp).
 */

#ifndef ItlBcm_hpp
#define ItlBcm_hpp

#include <memory>

#include <compat.h>
#include <linux/kernel.h>

#include <IOKit/IOWorkLoop.h>
#include <IOKit/network/IOGatedOutputQueue.h>
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

struct bcm_node {
    struct ieee80211_node ni;
};

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
    const char *sc_fwver;
    int (*sc_newstate)(struct ieee80211com *, enum ieee80211_state, int);
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
    // called by the future D11 RX interrupt handler; bcm_tx_start dequeues
    // if_snd and programs the D11 TX ring.
    void bcm_rx_input(mbuf_t m, int rssi, uint8_t chan);
    int bcm_tx_start(struct _ifnet *ifp);

    // D11 dma64 rings (constructed once the D11 core base / BAR1 window is
    // resolved; nullptr until then — see findCoreAddresses TODO).
    std::unique_ptr<D11::RxRing> mRxRing;
    std::unique_ptr<D11::TxRing> mTxRing;

    // PCI match predicate used by IOPCIEDeviceWrapper::probe.
    static bool bcm_match(IOPCIDevice *device);

    void releaseAll();

private:
    bcm_softc com;
    ChipConfigurator cfg;
    BcdcOlTransport ol;
    const ChipDescriptor *descriptor {nullptr};

    bool init80211();
};

#endif /* ItlBcm_hpp */
