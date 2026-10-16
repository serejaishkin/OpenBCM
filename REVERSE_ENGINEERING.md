# REVERSE_ENGINEERING.md

Guide for an AI agent (or human) reverse-engineering Broadcom BCM4360/43602/4350 firmware to extract:

1. **D11 PHY channel-switching register map** — needed so `ChipConfigurator::setChannel()` can retune the radio per scan channel.
2. **WLC/BCDC scan/join command set** — needed if host-driven scan is insufficient.

---

## 0. What we already have (OpenBCM repo)

| File | Purpose |
|------|---------|
| `itlwm/hal_bcm/ChipConfigurator.{hpp,cpp}` | Chip bring-up: `_wlc_ol_up` steps (halt ARM, publish shared-info, download dlimage, poll ready) |
| `itlwm/hal_bcm/D11.hpp` | Pure-C++ model of D11 `dma64` TX/RX descriptor rings (register map: ctl/prod/base/done at `+0x00/+0x04/+0x08/+0x10`) |
| `itlwm/hal_bcm/Transport.hpp` | BCDC/olmsg control rings (64 KB DMA, doorbell at BAR1+0x48, int-mask at BAR1+0x4c) |
| `itlwm/hal_bcm/ItlBcm.cpp` | net80211 glue: scan state machine, RX poll (50 ms), TX via `if_start` |
| `itlwm/hal_bcm/BcmChip.hpp` | Chip descriptors (PCI IDs → backplane chip numbers, transport type, firmware name) |
| `firmware/extracted_dlimage/DLIMAGE_FINDINGS.md` | Decoded `_wlc_ol_up` loader, olmsg layout, D11 dma64 proof |
| `firmware/extracted_dlimage/D11_FINDINGS.md` | Confirmed D11 dma64 register window: `TX = d11core+0x200+N*0x40`, `RX0 = d11core+0x220`, descriptor format {ctl,addrlo,addrhi,resv}, ctl bits |

**Missing:** the actual D11 PHY channel programming sequence (which registers to write to change 2.4/5 GHz channel) and the WLC/BCDC message IDs for scan/join if we decide to offload scan to firmware.

---

## 1. Sources to reverse

### 1.1 Linux brcmfmac driver (PRIMARY)
Repository: `git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/`  
Path: `drivers/net/wireless/broadcom/brcm80211/brcmfmac/`

Key files to grep:
- `brcmfmac.c` — module init, device probe
- `brcmfmac_main.c` — scan/assoc logic, `brcmf_do_escan`, `brcmf_netdev_start_xmit`
- `brcmf_wifi.c` — wl->d11 core, channel setting
- `chip.c` / `chip_common.c` — backplane, chipcommon regs
- `d11.c` / `dma.c` — D11 core, dma64 rings, TX/RX descriptor programming
- `phy.c` / `phy_shim.c` — PHY init, channel switch, radio regs
- `fwil_types.h`, `brcmf_wlan.h` — ioctl/message definitions (`WLC_SET_CHANNEL`, `WLC_SCAN`, `WLC_SET_SSID`, etc.)
- `brcmf_cfg80211.c` — cfg80211 → WLC ioctl translation

Key symbols to find (search in source or symbols):
```
_wlc_bmac_set_channel
_wlc_bmac_channel
_wlc_bmac_recv
_wlc_bmac_dma_rxfill
_wlc_bmac_dma_txfast
_wlc_bmac_txstatus
_wlc_bmac_dma_attach
_dma64_txinit
_dma64_txfast
_dma64_txreclaim
__dma_rxinit
__dma_rxfill
_wlc_scan
_wlc_escan
wlc_ol_dpc
_wlc_ol_msg_send
_wlc_ol_up
_wlc_ol_dpc
```

### 1.2 Linux firmware binaries (SECONDARY)
Repository: `git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/`  
Path: `brcm/`

Files:
- `brcmfmac43602-pcie.bin`
- `brcmfmac4350-pcie.bin`

These contain the ARM dlimage. They are **not** the Apple dlimage; they use Linux msgbuf/BCDC protocol, not Apple `_wlc_ol_up`. Use them to confirm D11 register offsets and channel constants, not for olmsg structure.

### 1.3 AppleBCMWLANCompanion (Apple-specific)
Repository: `D:/GitHub/AppleBCMWLANCompanion_reversed/` (local)  
- `binary/AppleBCMWLANCompanion.kext` — x86_64 kext binary  
- `disassembly/full_x86_64.asm` — full disassembly (1 MB)  
- `symbols/symbol_table_demangled.txt` — demangled symbol table  
- `analysis/key_functions/*.asm` — reversed key functions  

This binary is the Skywalk/WCL interceptor (`AppleBCMWLANInfraProtocolBridge`). It forwards IO80211 ioctls to the real `AirPortBrcmNIC` kext. **It does NOT contain WLC scan logic** — only passthrough stubs. Use it to confirm the Skywalk→BCDC boundary, not scan opcodes.

### 1.4 AirPortBrcmNIC_re.bin (WISH LIST)
Apple proprietary kext (`/System/Library/Extensions/AirPortBrcmNIC.kext/Contents/MacOS/AirPortBrcmNIC` on macOS 10.13–11.7). Contains the real `_wlc_*`, `wlc_scan`, `wlc_assoc`, D11 PHY code. **Not available locally.** If you can obtain it, disassemble with `otool -tV` or Hopper/Ghidra and look for:
- `_wlc_bmac_set_channel` / `_wlc_phy_channel`
- `wlc_scan` / `wlc_escan`
- `_wlc_ol_dpc`
- `_wlc_bmac_recv` / `_wlc_bmac_dma_rxfill` / `_wlc_bmac_dma_txfast`

---

## 2. What to extract (target outputs)

### 2.1 D11 PHY channel switching (REQUIRED for multi-channel scan)
Find in brcmfmac disassembly/source:

| Item | Where to look | Example |
|------|---------------|---------|
| D11 core base | `d11core->regs`, `si_setcore(si, D11_CORE_ID, 0)` | Usually `0x18004000` on 4360/43602 |
| PHY register base offset | `d11core->phyregs` or `si_setcore(si, PHY_CORE_ID, 0)` | `0x18008000` etc. |
| Channel register write | `_wlc_bmac_set_channel`, `_wlc_phy_channel` | Write channel to PHY reg `0x44c` / `0x44e` etc. |
| Radio register writes | `_wlc_phy_radio_init`, `phy_reg_write` | `0x4e`, `0x50` etc. |
| Band select (2G/5G) | `CHAN_2GHZ` vs `CHAN_5GHZ` flags | Reg `0x4e` bit for band |
| XTAL / crystal settings | `phy_cal_txiqlo`, `phy_init` | Crystal offset, PA gain |

Deliverable for AI: a C header snippet like:

```c
// D11 PHY register map (from brcmfmac reverse)
#define D11_PHY_CHANNEL_REG   0x44c   // offset from D11 core base
#define D11_PHY_RADIO_REG     0x4e
#define D11_PHY_BAND_2GHZ     0x0001
#define D11_PHY_BAND_5GHZ     0x0002
#define D11_PHY_CHANNEL_MASK  0xff
```

### 2.2 WLC/BCDC scan command set (OPTIONAL — only if host-driven scan fails)
If host-driven RX scan (already implemented) cannot hop channels without PHY RE, we need WLC scan commands:

| Item | Where to look |
|------|---------------|
| WLC ioctl definitions | `brcmf_wlan.h`: `WLC_SCAN`, `WLC_SCAN_RESULTS`, `WLC_SET_SSID`, `WLC_SET_AUTH`, `WLC_SET_WPA_AUTH`, `WLC_SET_KEY` |
| BCDC message IDs | `AppleBCMWLANCompanion_re.bin` disassembly: olmsg message `id` field values used for scan requests |
| Scan result event | D2H event IDs in `Transport.cpp` / `OlMsg.hpp` (currently `0x4,0x10,0x11,0x12,0x15,0x41`) — check if any is scan-done |

Deliverable: `WLC_SCAN` ioctl number + BCDC message `id` + payload format.

---

## 3. Reverse engineering workflow (step-by-step)

### Step A: Build symbol map from brcmfmac source
```bash
git clone --depth=1 https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/
cd linux/drivers/net/wireless/broadcom/brcm80211/brcmfmac/
# Search for channel programming:
grep -rn "set_channel\|set_chan\|CHANNEL" *.c *.h | head -50
grep -rn "wlc_scan\|wlc_escan\|SCAN_CMD\|SCAN_REQ" *.c *.h | head -50
```

### Step B: Map D11 core registers
Search for `d11core->regs` accesses and `SI_SETCORE` calls:
```bash
grep -rn "d11core" *.c | head -30
grep -rn "setcore.*D11\|D11_CORE_ID" *.c | head -20
```

### Step C: Find PHY channel program sequence
Look for functions that write channel to hardware:
```bash
grep -rn "phy_channel\|set_channel\|channel.*reg\|reg.*channel" *.c | head -40
grep -rn "0x4e\|0x44c\|0x44e\|0x16e\|0x170" *.c 2>/dev/null | head -20
```

### Step D: Confirm TX/RX descriptor format
Cross-check `dma64` descriptor layout in `dma.c` against `D11.hpp`:
```bash
grep -rn "dma64_dd\|dma_desc\|dd_t" *.c *.h | head -20
```

### Step E: Extract firmware symbols (if AppleBCMWLANCompanion binary available)
```bash
# On the local reversed binary:
strings -t x AppleBCMWLANCompanion.kext/Contents/MacOS/AppleBCMWLANCompanion | grep -i "wlc\|scan\|channel\|dma" | head -50
nm -C AppleBCMWLANCompanion.kext/Contents/MacOS/AppleBCMWLANCompanion | grep -i "wlc\|scan\|dma" | head -50
```

---

## 4. Deliverable format for AI → OpenBCM integration

When you find the register map, produce a diff/patch like this:

```diff
--- a/itlwm/hal_bcm/ChipConfigurator.cpp
+++ b/itlwm/hal_bcm/ChipConfigurator.cpp
@@ -X +X @@
+static const UInt32 kD11PhyChanReg = 0x44c;   // from brcmfmac
+static const UInt32 kD11PhyBand2G = 0x0001;
+static const UInt32 kD11PhyBand5G = 0x0002;
+
+void ChipConfigurator::setChannel(UInt8 channel) {
+    UInt32 band = (channel <= 14) ? kD11PhyBand2G : kD11PhyBand5G;
+    writeD11Reg(kD11PhyChanReg, channel | (band << 8));
+}
+
```

And modify `ItlBcm::bcm_scan_next_channel()` to call `cfg.setChannel(com.sc_scan_list[...])` before enqueuing the probe request.

---

## 5. Verification checklist

Before declaring reverse engineering complete, confirm:

- [ ] D11 core base address is confirmed (from `bcma` core scan or EROM `CoreDescriptor`)
- [ ] Channel register write location confirmed (from `brcmfmac` `_wlc_bmac_set_channel`)
- [ ] Band select bit (2G vs 5G) confirmed
- [ ] RX descriptor `rxh` format confirmed (signal/RSSI fields) — needed for `ieee80211_rxinfo.rxi_rssi`
- [ ] olmsg BCDC message IDs for scan (if going firmware-scan route)
- [ ] All constants compile in `ChipConfigurator.hpp` / `ItlBcm.hpp`

---

## 6. References

- `D:/GitHub/OpenBCM/itl80211/openbsd/net80211/` — net80211 stack (scan, input, proto)
- `D:/GitHub/OpenBCM/itlwm/hal_bcm/D11_FINDINGS.md` — existing D11 dma64 reverse notes
- `D:/GitHub/OpenBCM/itlwm/hal_bcm/DLIMAGE_FINDINGS.md` — firmware bring-up notes
- Linux `brcmfmac` source: `https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/` → `drivers/net/wireless/broadcom/brcm80211/brcmfmac/`
- Linux firmware: `https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/` → `brcm/`
- AppleBCMWLANCompanion reversed: `D:/GitHub/AppleBCMWLANCompanion_reversed/disassembly/full_x86_64.asm`
