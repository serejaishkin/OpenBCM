# RE_PHY_CHANNEL_SWITCH.md — Plan for reverse-engineering BCM4360 D11 PHY channel-switch registers

**Goal:** Recover the D11 PHY register map and channel-switch write sequence so the host
(itlwm/OpenBCM) can program the radio channel directly instead of relying on firmware.

**Status:** Placeholder PHY values currently used; this document is the handoff for a
follow-up RE pass (another AI session or a human reverse engineer).

---

## 1. What needs to be downloaded

| Artifact | Source | Purpose |
|----------|--------|---------|
| `AirPortBrcmNIC.kext` (x86_64, Ventura / Tahoe) | OCLP payloads: `IO80211FamilyLegacy-v1.0.0.zip` / `AirPortBrcmNIC-Tahoe-v1.0.0.zip` from dortania/OpenCore-Legacy-Patcher `payloads/Kexts/Wifi/` | Contains `_wlc_bmac_attach`, `_wlc_phy_*`, `_wlc_bmac_recv`, `_dma64_*`, D11 PHY init + channel-switch code. Thin Mach-O x86_64. |
| `brcmsmac` / `brcmfmac` source (Linux) | `git://git.kernel.org/pub/scm/linux/kernel/git/kvalo/brcm80211.git` | Open-source reference for D11 PHY regs (phyreg table, channel switch, radio init). |
| `broadcom-wl` source | `https://github.com/redhat-acn/bcm-wl` (or vendor tarball) | Proprietary Broadcom wl driver — has `wlc_phy_*` and D11 PHY programming. |
| `b43` driver source (Linux) | `git://git.kernel.org/pub/scm/linux/kernel/git/kvalo/b43.git` | Legacy Broadcom driver; contains PHY reg definitions for N/HT PHY (phy_nrev >= 3). |

Local copies already present:
- `D:/GitHub/BCM_94360cs/firmware/extracted_dlimage/apple_4352pci_dlarray.bin` — ARM Thumb-2 offload image (PM/scan coprocessor, NOT D11 PHY).
- `D:/GitHub/AppleBCMWLANCompanion_reversed/disassembly/full_x86_64.asm` — **WRONG binary**: this is the Skywalk/WCL Lilu plugin, not AirPortBrcmNIC. It has WCL bridge `getCHANNEL`/`setCHANNEL` wrappers only; no D11 PHY regs.

---

## 2. What to reverse engineer in AirPortBrcmNIC

Target functions (x86_64 addresses are from the Ventura/Tahoe kext; verify per build):

| Function | Role | What to extract |
|----------|------|-----------------|
| `_wlc_bmac_attach` (0xfab1f..0xfad79) | D11 attach; programs DMA channel windows + PHY | D11 core base, PHY reg base offsets, initial PHY write sequence |
| `_wlc_phy_init` / `_wlc_phy_setup` | N/HT PHY bring-up | PHY register table (addr → value pairs), radio init |
| `_wlc_phy_channel_switch` / `_wlc_switch_channel` | Set radio channel | Exact register writes per channel (2.4 GHz + 5 GHz), PHY reg offsets |
| `_wlc_phy_rxstart` / `_wlc_phy_txstart` | RX/TX PHY enable | PHY ctl bits used before/after frames |
| `_dma64_txfast` / `__dma_rxfill` | dma64 desc programming | Confirmed already (see `D11_FINDINGS.md`) |
| `_wlc_bmac_dma_rxfill` / `_wlc_bmac_recv` | RX path | How RX frames are delivered (rxh layout) |
| `_wlc_bmac_txstatus` | TX done / reclaim | TX completion semantics |

Key data structures to locate:
- `phyreg` table (array of {reg_addr, value} pairs) referenced from `_wlc_phy_init`.
- Channel table: per-channel PHY/RF register values (often `wlc_phy_reg_2g` / `wlc_phy_reg_5g` arrays).
- `d11regs` / `phyregs` base offsets within the D11 core window.

---

## 3. Known D11 core register context (already confirmed)

From `D11_FINDINGS.md` (authoritative for this project):

```
TX chan N regs = D11 core regs + 0x200 + N*0x40   (N = 0..5)
RX chan 0 regs = D11 core regs + 0x220             (paired with TX ch0)
core regs = wlc->d11core->regs (SI/EROM D11 core base)
```

Per-channel DMA regs (+0x40 window):
- +0x00 DMA ctl (bit0 DMA_EN, bit1 RESET, bits16-17 addr-ext, bits18-31 XTRA)
- +0x04 producer/PTR = table_bus_addr + idx*16
- +0x08/+0x0c desc table bus addr lo/hi
- +0x10 consumer/DONE + reset status

Descriptor (16 bytes): {ctl, addrlo, addrhi, resv}
- TX ctl: 0x80000000 SOP, 0x60000000 EOT, 0xe0000000 single, 0x10000000 wrap, 0x40000 frag
- RX ctl: 0x80000000 OWN, 0x10000000 wrap

**Missing:** D11 PHY register block base offset within the core window, and the PHY register addresses/values for channel switching.

---

## 4. Where PHY registers likely live

Broadcom D11 cores (BCM4360 = 11ac, N/HT PHY, rev >= 3) typically have a PHY register
window inside the D11 core. Common patterns from open-source Broadcom drivers:

- PHY regs are accessed via an index/data or indirect window at a fixed offset from the core base (e.g. core + 0x400..0x7FF region), OR
- Direct MMIO at core + PHY_REG_OFFSET (varies by chip rev).

The `brcmsmac` driver (`drivers/net/wireless/broadcom/brcm80211/brcmsmac/`) defines:
- `D11PHY` regs with offsets like `phyreg 0x00..0xFF`
- `phy_write(phy, reg, val)` / `phy_read(phy, reg)` helpers
- Channel switch tables `brcms_c_phyreg_2g[]` / `brcms_c_phyreg_5g[]`

The `b43` driver (`drivers/net/wireless/broadcom/b43/`) has `phy_n.c` / `phy_nregs` for N-PHY with `NPHY_*` register constants.

**Task:** Find the equivalent in AirPortBrcmNIC — the PHY reg write addresses and the per-channel value table.

---

## 5. Suggested RE workflow

### Step A — Obtain the binary
1. Download OCLP `IO80211FamilyLegacy-v1.0.0.zip` (or Tahoe) from dortania/OpenCore-Legacy-Patcher releases.
2. Extract `AirPortBrcmNIC.kext/Contents/MacOS/AirPortBrcmNIC` (thin x86_64 Mach-O).
3. Copy to `D:/GitHub/BCM_94360cs/firmware/extracted_dlimage/` (or a new `re/` dir).

### Step B — Disassemble
Use any x86_64 disassembler:
- `objdump -d -M intel` (quick)
- Ghidra / IDA / Binary Ninja (preferred for cross-references)
- radare2 / cutter

Produce `full_x86_64_airportbrcmnic.asm` alongside the existing `full_x86_64.asm`.

### Step C — Locate PHY code
Search strings and xrefs:
- `"phy"` / `"PHY"` / `"channel"` / `"switch_channel"`
- `"txstart"` / `"rxstart"`
- Look for `_wlc_phy_*` symbols (if stripped, search for the function prologues referenced from `_wlc_bmac_attach`).

### Step D — Extract PHY register table
1. Find `_wlc_phy_init` — it typically walks a table of `{u16 reg, u16 val}` pairs and writes each to the PHY.
2. Dump the table addresses/values.
3. Find `_wlc_switch_channel` — identifies which regs change per channel (2.4 GHz: 1-14, 5 GHz: 36-165).
4. Record the PHY register base offset and per-channel write sequence.

### Step E — Verify against open source
Cross-check with `brcmsmac`/`b43` PHY reg constants:
- `NPHY_TxRFCORE`, `NPHY_RxRFCORE`, channel-specific PLL/VCO regs, etc.
- Many Broadcom PHY regs are chip-family stable; the 4360 (N-PHY rev 7+) should match.

### Step F — Encode into code
Create `hal_bcm/D11Phy.{hpp,cpp}` with:
- `D11Phy` class wrapping `RegRead/RegWrite` callbacks (same style as `D11.hpp`)
- `setChannel(uint8_t chan)` implementing the recovered write sequence
- `initPhy()` calling the recovered init table
- RSSI read from PHY reg (replaces `readRssi()` stub)

Mark any uncertain bits `TODO(RE)`.

---

## 6. Integration points in current code

- `Dma64RxRing::readRssi()` — stub; replace with PHY reg read.
- `Dma64RxRing::setCurrentChan()` / `mCurrentChan` — used to tag frames with channel.
- `ItlBcm::bcm_bgscan_start` → scan hopping: call `phy.setChannel(chan)` before each dwell.
- `ChipConfigurator::findCoreAddresses()` — must return D11 core base; PHY regs are at `d11core + PHY_OFFSET`.

---

## 7. Deliverables expected from the RE pass

1. `D11Phy.hpp` — PHY register offsets, `D11Phy` class declaration.
2. `D11Phy.cpp` — `setChannel()` + `initPhy()` implementations (concrete register writes).
3. Updated `Dma64RxRing::readRssi()` — real PHY RSSI read.
4. Updated `ItlBcm` scan hopper — calls `phy.setChannel()` per channel.
5. `RE_PHY_CHANNEL_SWITCH.md` updated with findings (reg addresses, values, source offsets).

---

## References

- `D:/GitHub/BCM_94360cs/firmware/extracted_dlimage/D11_FINDINGS.md` — confirmed DMA/ring layout.
- `D:/GitHub/AppleBCMWLANCompanion_reversed/analysis/FORK_PLAN.md` — architecture decision (host-driven D11 SoftMAC).
- `D:/GitHub/BCM_94360cs/firmware/extracted_dlimage/DLIMAGE_FINDINGS.md` — dlimage firmware layout.
- Linux `brcmsmac` (`drivers/net/wireless/broadcom/brcm80211/brcmsmac/phy_n.c`) — N-PHY reg reference.
- Linux `b43` (`drivers/net/wireless/broadcom/b43/phy_n.c`) — NPHY_* register constants.
- Broadcom `wl` / `brcmfmac` source — channel-switch tables.
