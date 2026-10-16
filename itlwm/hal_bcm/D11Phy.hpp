// D11Phy.hpp — host-driven D11 PHY channel-switch + init (placeholder values).
//
// Wraps D11 PHY register access with RegRead/RegWrite callbacks (same pattern
// as D11.hpp). Provides setChannel(chan) and initPhy() using *placeholder*
// register addresses/values marked TODO(RE). These must be replaced after
// reverse-engineering AirPortBrcmNIC's _wlc_phy_* functions (see
// RE_PHY_CHANNEL_SWITCH.md).
//
// PHY register block base = d11core + kD11PhyBaseOffset (placeholder 0x400).
// All reads/writes are 32-bit MMIO at (base + reg_addr).
//
// STATUS: placeholder only. Radio will NOT actually tune until the real PHY
//         register map is recovered from AirPortBrcmNIC.

#ifndef D11Phy_hpp
#define D11Phy_hpp

#include <stdint.h>
#include <stddef.h>

namespace bcm {
namespace d11 {

// D11 PHY register block offset within the D11 core window.
// TODO(RE): recover from AirPortBrcmNIC _wlc_bmac_attach / _wlc_phy_init.
static constexpr uint32_t kD11PhyBaseOffset = 0x00000400u;

// PHY register addresses (placeholders, TODO(RE)).
enum PhyReg : uint32_t {
    kPhyRegChannel         = 0x00, // channel program register
    kPhyRegRadioChan2G     = 0x04, // 2.4 GHz radio channel value
    kPhyRegRadioChan5G     = 0x08, // 5 GHz radio channel value
    kPhyRegRssi            = 0x0c, // RSSI read (placeholder)
    kPhyRegInitTableAddr   = 0x10, // PHY init table pointer (placeholder)
    kPhyRegInitTableVal    = 0x14, // PHY init table value (placeholder)
    kPhyRegTxPower         = 0x18, // TX power control (placeholder)
    kPhyRegAntGain         = 0x1c, // Antenna gain (placeholder)
};

// 2.4 GHz channels: 1..14
// 5 GHz channels: 36,40,44,48,52,56,60,64,100,104,108,112,116,120,124,128,132,136,140,144,149,153,157,161,165

// Callback types with void *ctx (same as D11.hpp).
using RegRead  = uint32_t (*)(void *ctx, uint32_t offset); // offset within PHY block
using RegWrite = void (*)(void *ctx, uint32_t offset, uint32_t value);

class Phy {
public:
    Phy(uint32_t phyBase,
        void *ctx,
        RegRead rd, RegWrite wr)
        : phyBase_(phyBase), ctx_(ctx), rd_(rd), wr_(wr),
          currentChan_(0) {}

    // Initialize the PHY: run the placeholder init register table.
    // Returns true if writes were issued (always true for placeholder).
    bool initPhy();

    // Set the radio channel.  Programs placeholder PHY registers.
    // TODO(RE): replace with real per-channel register sequence.
    bool setChannel(uint8_t chan);

    uint8_t currentChan() const { return currentChan_; }

    // Read RSSI from PHY register (placeholder: fixed value).
    int8_t readRssi();

private:
    // Write a single PHY register.
    void phyWrite(uint32_t reg, uint32_t val);

    // Read a single PHY register.
    uint32_t phyRead(uint32_t reg);

    uint32_t phyBase_;
    void    *ctx_;
    RegRead rd_;
    RegWrite wr_;
    uint8_t currentChan_;
};

} // namespace d11
} // namespace bcm

#endif /* D11Phy_hpp */
