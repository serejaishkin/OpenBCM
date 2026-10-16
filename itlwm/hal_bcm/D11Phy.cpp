// D11Phy.cpp — D11 PHY channel-switch + init (placeholder values).
//
// TODO(RE): replace all placeholder register addresses/values with the real
//           PHY register map recovered from AirPortBrcmNIC.

#include "D11Phy.hpp"

namespace bcm {
namespace d11 {

// PHY init register table (placeholder).  Each entry: {reg_addr, value}.
// These are NOT real Broadcom values; they are structural placeholders.
// TODO(RE): populate from AirPortBrcmNIC _wlc_phy_init register write trace.
static const struct {
    uint32_t reg;
    uint32_t val;
} kPhyInitTable[] = {
    { kPhyRegInitTableAddr, 0x00000001u }, // placeholder: PHY init enable
    { kPhyRegInitTableVal,  0x00000000u }, // placeholder: PHY init value 0
    { kPhyRegTxPower,       0x00000020u }, // placeholder: TX power default
    { kPhyRegAntGain,       0x0000000Au }, // placeholder: antenna gain
    { 0xffffffffu, 0x00000000u },           // sentinel
};

// Per-channel radio values (placeholder).  Maps channel -> radio register value.
// TODO(RE): real values depend on 2.4 GHz vs 5 GHz radio PLL/VCO settings.
static uint32_t phyChanRadioValue(uint8_t chan) {
    if (chan >= 1 && chan <= 14) {
        // 2.4 GHz: placeholder mapping channel -> base + offset
        return 0x00001000u + (uint32_t)chan;
    }
    if (chan == 36 || chan == 40 || chan == 44 || chan == 48 ||
        chan == 52 || chan == 56 || chan == 60 || chan == 64 ||
        chan == 100 || chan == 104 || chan == 108 || chan == 112 ||
        chan == 116 || chan == 120 || chan == 124 || chan == 128 ||
        chan == 132 || chan == 136 || chan == 140 || chan == 144 ||
        chan == 149 || chan == 153 || chan == 157 || chan == 161 ||
        chan == 165) {
        // 5 GHz: placeholder mapping
        return 0x00002000u + (uint32_t)chan;
    }
    return 0x00000000u; // invalid channel
}

void Phy::
phyWrite(uint32_t reg, uint32_t val) {
    wr_(ctx_, reg, val);
}

uint32_t Phy::
phyRead(uint32_t reg) {
    return rd_(ctx_, reg);
}

bool Phy::
initPhy() {
    for (size_t i = 0; i < sizeof(kPhyInitTable) / sizeof(kPhyInitTable[0]); i++) {
        if (kPhyInitTable[i].reg == 0xffffffffu) break;
        phyWrite(kPhyInitTable[i].reg, kPhyInitTable[i].val);
    }
    return true;
}

bool Phy::
setChannel(uint8_t chan) {
    if (chan == 0) {
        // Channel 0 = no channel (idle).  Do not program radio.
        currentChan_ = 0;
        return true;
    }
    uint32_t radioVal = phyChanRadioValue(chan);
    if (radioVal == 0) {
        return false; // unsupported channel
    }

    // Placeholder channel-switch sequence:
    // 1. Write channel number to the channel program register.
    // 2. Write the radio PLL/VCO value.
    // 3. Small delay (not implemented here; caller handles timing).
    phyWrite(kPhyRegChannel, (uint32_t)chan);
    if (chan <= 14) {
        phyWrite(kPhyRegRadioChan2G, radioVal);
    } else {
        phyWrite(kPhyRegRadioChan5G, radioVal);
    }

    currentChan_ = chan;
    return true;
}

int8_t Phy::
readRssi() {
    // TODO(RE): read RSSI from the PHY RSSI register and convert to dBm.
    // Placeholder: fixed RSSI.
    return -50;
}

} // namespace d11
} // namespace bcm
