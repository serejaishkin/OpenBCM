# FullMAC first-boot test guide (BCM43602 and friends)

## One-shot helper (on the Mac)
    scripts/openbcm_probe.sh info    # card IDs + ACPI path for DeviceProperties
    scripts/openbcm_probe.sh nvram   # try to dig board nvram out of Apple's Broadcom kexts -> nvram_out/
    scripts/openbcm_probe.sh log     # kext loaded? + checklist/diagnosis of the OpenBCM log -> openbcm.log
The nvram extraction is best-effort (it assumes Apple keeps `key=value` parameter blocks inside the
AirPortBrcm kexts); check the printed boardtype against your card's subsystem id before using it.

## Prepare
1. Put `brcmfmac43602-pcie.txt` (your board nvram) in `firmware/fullmac/` (the `.bin` is already there).
2. Build the kext in Xcode (target AirportItlwm-Sequoia / Sonoma14.4 / Tahoe). The "Embed FullMAC Firmware"
   phase runs `scripts/gen_fullmac_arrays.py` automatically. Check the build log for
   `wrote …FullmacFwArrays.cpp: N firmware, M nvram, K clm`.
3. OpenCore: add the kext to `Kernel → Add` as usual. Optional per-card overrides under
   `DeviceProperties → Add → PciRoot(0x0)/Pci(…)/Pci(…)` (type String):
   `openbcm-mode` = `fullmac`, `openbcm-fw` = `brcmfmac43602-pcie`, `openbcm-nvram` = `brcmfmac43602-pcie`.
4. Boot args: `-v keepsyms=1 debug=0x100` for the first run.

## Collect logs
    log show --last boot --predicate 'eventMessage CONTAINS "bcm" OR eventMessage CONTAINS "OpenBCM"' > openbcm.log
(or `sudo dmesg | grep -i -E "bcm|OpenBCM"` / the verbose boot screen if it never gets that far.)

## What the log lines mean
| line | meaning |
|---|---|
| `bcm: chip 0x… mode=fullmac` | personality/property selected fullmac |
| `fullmac firmware auto-selected: …` / `from device property` | which image |
| `fullmac blobs not embedded` | step 2 above didn't embed firmware+nvram |
| `OpenBCM fullmac: BAR0 … BAR2 …` | both BARs mapped; sizes must be non-zero (BAR2 ≥ 2 MB expected) |
| `N cores (cc=1 cr4=1 pcie2=1 imem=1)` | EROM scan worked; any 0 → step 2 failure |
| `RAM base 0x180000 size 0x…` | CR4 RAM detected (43602: ~0xE0000) |
| `firmware start status 0 shared=0x…` | firmware booted and published shared RAM |
| `fullmac start failed at step N` | 1 BAR map, 2 cores missing, 3 nvram alloc, 4 nvram strip, 5 firmware didn't start (status printed), 6 bad shared info, 7 ring plan, 8 ring/engine DMA, 9 unknown chip RAM base, 10 flowring init, 11 data path init |
| `MAC xx:…` | `cur_etheraddr` ioctl round trip works (msgbuf ioctl path is alive) |
| `fullmac preinit: 0 step(s) failed` | CLM/event mask/roam setup accepted by firmware |

## Send me
The whole `openbcm.log` (from the first `bcm:` line), the exact card (`lspci`-style 14e4:xxxx and the OC
DeviceProperties you used) and at which line it stops. Typical early failures and where they point:
- stops after BAR line, no cores: BAR0 window / EROM address (Erom.hpp, FullmacPcie.hpp `IoBus::setBar0Window`)
- `firmware start status 2` (timeout): nvram content/placement, or firmware not released (PcieBringup)
- `bad shared info`: shared RAM version outside 5..7 (Msgbuf.hpp `parseSharedInfo`)
- ioctl timeouts after firmware up: ring index handling (FullmacPcie.hpp ring callbacks)
