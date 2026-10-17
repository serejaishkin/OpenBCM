// Dma64RxRing.hpp — host-driven D11 dma64 RX ring with real IOKit DMA.
//
// Wraps the pure-C++ D11::RxRing logic with IOKit DMA buffer allocation and
// ChipConfigurator-backed register access.  Used by ItlBcm to receive 802.11
// frames (beacons, probe responses, data) directly from the D11 MAC, making
// scanning work with *any* firmware (Linux/OpenBCD/whatever) since the host
// drives the radio itself.
//
// Frame flow:
//   1. Allocate DMA buffer (descriptor table + N RX buffers)
//   2. Program D11 RX channel regs: base addr, enable
//   3. Post RX buffer descriptors (HW OWN bit set)
//   4. Poll regbase+0x10 (DONE) for completed descriptors
//   5. For each completed frame: clear OWN, re-post buffer, deliver mbuf to
//      bcm_rx_input() → ieee80211_inputm()
//
// RX descriptor (16 bytes, per D11_FINDINGS.md):
//   +0x00 ctl: 0x80000000 (OWN, always set by host)
//              0x10000000 at slot count-1 (wrap marker)
//   +0x04 fill length (written by HW on completion)
//   +0x08 addr low
//   +0x0c addr high
// --------------------------------------------------------------------------

#ifndef Dma64RxRing_hpp
#define Dma64RxRing_hpp

#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <sys/kpi_mbuf.h>
#include "D11.hpp"
#include "D11Phy.hpp"
#include "ChipConfigurator.hpp"

// Use the bcm::d11 namespace (project uses -std=gnu++14, not C++17 nested ns)
using namespace bcm::d11;

// Callback type for RX frame delivery with caller context.
// (mbuf_t, rssi, chan). The callee takes ownership of the mbuf.
using Dma64RxCallback = void (*)(void *cbCtx, mbuf_t, int8_t, uint8_t);

class Dma64RxRing {
public:
    // `cfg` provides BAR0 MMIO + D11 core base.  `channel` selects which D11
    // DMA channel (0 = RX paired with TX0; on 4360 only ch0 RX exists).
    // `phyBase` is the D11 PHY register block offset within the core window
    // (see D11Phy::kD11PhyBaseOffset).
    Dma64RxRing(ChipConfigurator *cfg, uint32_t phyBase,
                uint16_t channel = 0, uint16_t count = 64,
                uint32_t bufSize = 2048);

    ~Dma64RxRing();

    bool init();                       // allocate DMA, program rings, enable RX
    void stop();                       // disable RX, release DMA

    // Poll for completed RX frames.  Returns the number delivered via `cb`.
    // `cb` receives (cbCtx, mbuf_t, rssi, chan) and takes ownership of the mbuf.
    uint32_t poll(void *cbCtx, Dma64RxCallback cb);

    bool isRunning() const { return mRunning; }

private:
    bool allocDma();
    void programHw();
    void postAllBuffers();
    uint32_t reclaimCompleted(Dma64RxCallback cb);

    ChipConfigurator *mCfg {nullptr};
    uint16_t mChannel {0};
    uint16_t mCount {0};
    uint32_t mBufSize {0};
    uint32_t mPhyBase {0};       // D11 PHY block offset within core window

    // DMA backing store: descriptor table (count*16) + count RX buffers.
    IOBufferMemoryDescriptor *mDmaBuf {nullptr};
    IOPhysicalAddress mDmaPhys {0};
    volatile UInt8 *mDmaVa {nullptr};

    uint32_t mTableOffset {0};      // descriptor table offset within DMA buf
    uint32_t mBufOffset {0};        // RX buffer pool offset within DMA buf
    uint64_t mTableBusAddr {0};     // bus address of descriptor table

    bcm::d11::RxRing *mRing {nullptr};
    bcm::d11::Phy    *mPhy  {nullptr};   // D11 PHY (channel switch, RSSI)

    uint16_t mProd {0};
    uint16_t mCons {0};
    bool mRunning {false};

    // Current radio channel (set by ItlBcm before poll()); reported to
    // net80211 per frame until the D11 rxh/PHY channel field is decoded.
    uint8_t mCurrentChan {0};
public:
    void setCurrentChan(uint8_t chan);
    uint8_t currentChan() const { return mCurrentChan; }
private:

    // RSSI read from D11 PHY (placeholder until real PHY regs decoded).
    int8_t readRssi();

    // Scan callback set during poll(); invoked by s_pollRxCb per received frame.
    Dma64RxCallback mPollCb {nullptr};
    void *mPollCbCtx {nullptr};
    uint32_t mDelivered {0};
};

#endif /* Dma64RxRing_hpp */
