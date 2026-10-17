// Dma64TxRing.hpp — host-driven D11 dma64 TX ring with real IOKit DMA.
//
// Used to send 802.11 probe requests during scanning.  Mirrors Dma64RxRing
// but for the TX direction: allocate DMA, program D11 TX channel regs,
// enqueue descriptors, poll for completion (reclaim).
//
// TX descriptor (16 bytes, per D11_FINDINGS.md):
//   +0x00 ctl: 0x80000000 SOP | 0x60000000 EOT | 0x10000000 wrap at last slot
//   +0x04 addr low of frame buffer
//   +0x08 addr high / ext
//   +0x0c resv
// --------------------------------------------------------------------------

#ifndef Dma64TxRing_hpp
#define Dma64TxRing_hpp

#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <sys/kpi_mbuf.h>
#include "D11.hpp"
#include "ChipConfigurator.hpp"

// Use the bcm::d11 namespace (project uses -std=gnu++14, not C++17 nested ns)
using namespace bcm::d11;

class Dma64TxRing {
public:
    Dma64TxRing(ChipConfigurator *cfg, uint16_t channel = 0, uint16_t count = 64);

    ~Dma64TxRing();

    bool init();
    void stop();

    // Enqueue a raw frame buffer for TX.  Returns false if the ring is full.
    bool enqueue(const void *data, uint32_t len);

    // Poll for completed TX descriptors.
    uint32_t pollTx();

    bool isRunning() const { return mRunning; }

private:
    bool allocDma();
    void programHw();

    ChipConfigurator *mCfg {nullptr};
    uint16_t mChannel {0};
    uint16_t mCount {0};

    IOBufferMemoryDescriptor *mDmaBuf {nullptr};
    IOPhysicalAddress mDmaPhys {0};
    volatile UInt8 *mDmaVa {nullptr};

    uint32_t mTableOffset {0};
    uint64_t mTableBusAddr {0};

    // Frame buffer pool (each slot holds one max-size frame).
    uint32_t mFramePoolOffset {0};
    uint32_t mFrameSize {2048};

    D11::TxRing *mRing {nullptr};

    uint16_t mProd {0};
    uint16_t mCons {0};
    bool mRunning {false};
};

#endif /* Dma64TxRing_hpp */
