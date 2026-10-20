/*
 * OpenBCM — Broadcom BCM4360/43602/4350 HAL service for the itlwm Airport
 * framework. This is the BCM counterpart of ItlIwm/ItlIwn/ItlIwx: it is created
 * by IOPCIEDeviceWrapper for Broadcom PCIe IDs and handed to AirportItlwm as
 * its ItlHalService.
 *
 * Control plane: WLC/BCDC via olmsg (ChipConfigurator + BcdcOlTransport).
 * The host sends WLC ioctl/iovar commands (escan, join, set_ssid,
 * set_wsec_pmk, ...) over olmsg and parses D2H events (ESCAN_RESULT,
 * PSK_SUP, LINK, ...). The ARM dlimage/firmware owns the radio: channel
 * hopping, beacon reception, authentication, association.
 *
 * Data plane: host-driven D11 SoftMAC `dma64` TX/RX rings (Dma64RxRing /
 * Dma64TxRing). Used for data frames and for host-driven passive scan:
 * the host hops channels via the D11 PHY, sends probe requests through the
 * TX ring, and receives beacons/probe-responses through the RX ring,
 * injecting them into net80211 via ieee80211_inputm().
 *
 * STATUS: bring-up + net80211 attach + D11 RX/TX rings implemented; WLC
 * control plane (escan/join/set_ssid/set_wsec_pmk) via olmsg implemented;
 * D11 PHY channel switching implemented with placeholder register values
 * (TODO(RE): replace with real PHY register map from AirPortBrcmNIC).
 * Host-driven passive scan is implemented and is the default scan path.
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

#include <HAL/ItlHalService.hpp>
#include <HAL/ItlDriverInfo.hpp>
#include <HAL/ItlDriverController.hpp>

#include "ChipConfigurator.hpp"
#include "Transport.hpp"
#include "D11.hpp"
#include "Dma64RxRing.hpp"
#include "Dma64TxRing.hpp"
#include "WlcDefs.hpp"

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
    const char *sc_fwver;
    // Channel list for the firmware-driven (WLC escan) scan.
    uint8_t sc_scan_list[64];
    uint8_t sc_scan_count;
    uint8_t sc_scan_idx;
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
    ItlDriverInfo *getDriverInfo() override;
    ItlDriverController *getDriverController() override;
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

    // WLC/BCDC control plane via olmsg (Group B).
    void bcm_wlc_send_ioctl(uint32_t cmd, const void *buf, UInt32 len);
    void bcm_wlc_send_iovar_set(const char *name, const void *buf, UInt32 len);
    void bcm_wlc_scan_channels(const uint8_t *channels, uint32_t count);
    void bcm_wlc_join(const char *ssid, uint32_t ssid_len, const UInt8 *bssid);
    void bcm_wlc_poll_d2h();   // drain olmsg RX and process WLC events
    void bcm_wlc_d2h_msg(const OlControlMessage *msg);  // parse one D2H message
    void bcm_wlc_scan_result(const UInt8 *bssid, const UInt8 *ssid,
                             uint32_t ssid_len, int8_t rssi); // inject scan result

    // D11 dma64 rings (real IOKit DMA, constructed in enable()).
    Dma64RxRing *mRxRing {nullptr};
    Dma64TxRing *mTxRing {nullptr};

    // Firmware-driven scan (WLC escan over olmsg); used by softmac (as the
    // scan path until D11 PHY is reversed) and by fullmac.
    void bcm_scan_start();
    void bcm_scan_next_channel();
    void bcm_scan_done();
    static void s_scan_timer(OSObject *target, IOTimerEventSource *sender);

    // MAC ownership, selected per device by the Info.plist "ChipType".
    //   softmac: host drives D11 dma64 rings (data plane lives in the kext).
    //   fullmac: firmware owns MAC/PHY/data; host only exchanges WLC commands
    //            and events (data plane transport still TODO, see
    //            docs/FULLMAC_PLAN.md).
    bool isFullmac() const { return descriptor && descriptor->isFullmac(); }
    // fullmac join handshake (called from bcm_newstate / D2H events).
    void bcm_fullmac_join_start();
    void bcm_fullmac_link_event(bool up);
    bool mJoinPending {false};

    // WLC scan state (firmware-driven).
    bool mWlcScanActive {false};
    UInt32 mWlcScanSeq {0};
    UInt8 mWlcScanSsid[32] {0};
    UInt8 mWlcScanSsidLen {0};
    UInt8 mWlcScanBssid[6] {0};
    bool mWlcScanBssidSet {false};

    // Host-driven scan state (D11 SoftMAC passive scan).
    bool mHostScanActive {false};
    UInt8 mHostScanList[64] {0};
    UInt8 mHostScanCount {0};
    UInt8 mHostScanIdx {0};
    UInt8 mHostScanSsid[32] {0};
    UInt8 mHostScanSsidLen {0};
    IOTimerEventSource *mHostScanTimer {nullptr};
    static void s_host_scan_timer(OSObject *target, IOTimerEventSource *sender);

    // Host-driven scan: build a probe request frame and send it on `chan`.
    void bcm_host_scan_start();
    void bcm_host_scan_next_channel();
    void bcm_host_scan_probe(uint8_t chan);
    bool bcm_build_probe_req(uint8_t *buf, uint32_t bufSize, uint32_t *frameLen);

    // net80211 ic_bgscan_start callback: start host-driven passive scan.
    void bcm_bgscan_start();

    // PCI match predicate used by IOPCIEDeviceWrapper::probe.
    static bool bcm_match(IOPCIDevice *device);
    void setDeviceConfig(const ChipDeviceConfig &cfg);

    void releaseAll();

private:
    bcm_softc com;
    ChipConfigurator cfg;
    BcdcOlTransport ol;
    const ChipDescriptor *descriptor {nullptr};
    ChipDescriptor mDescriptor {};       // owned copy: table entry + personality override
    ChipDeviceConfig mDeviceConfig {};   // personality overrides from Info.plist

    // WLC/olmsg D2H poll timer: fires periodically during scan to read
    // escan results and other firmware events.
    IOTimerEventSource *mWlcPollTimer {nullptr};
    static void s_wlc_poll_timer(OSObject *target, IOTimerEventSource *sender);
    static void s_wlc_d2h_sink(void *ctx, const OlControlMessage *msg);

    // D11 RX poll timer (50 ms): keeps data/mgmt flowing in RUN state.
    IOTimerEventSource *mRxPollTimer {nullptr};
    IOTimerEventSource *mScanTimer {nullptr};
    static void s_rx_poll(OSObject *target, IOTimerEventSource *sender);
    static void s_rxDeliver(void *ctx, mbuf_t m, int8_t rssi, uint8_t chan);

    bool init80211();
};

#endif /* ItlBcm_hpp */
