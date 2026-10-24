# OpenBCM

**OpenBCM — Broadcom BCM4360/43602/4350 Wi-Fi Kernel Extension for macOS, based on the itlwm framework.**

OpenBCM ports the laobamac/itlwm AirportItlwm (net80211 + Skywalk/WCL) stack to Broadcom chips. It is a host-driven SoftMAC implementation: the macOS kernel drives the D11 MAC/PHY directly via `dma64` TX/RX rings, while the ARM dlimage in the chip handles firmware bring-up, PM, scan offload hints, and WOWL.

## Supported hardware

| PCI ID | Chip | Card examples | Firmware |
|--------|------|---------------|----------|
| `14e4:43a0` | BCM4360 | BCM94360CS/CD/HMB | `apple_4352pci_dlarray.bin` (embedded) |
| `14e4:43ba` | BCM43602 | BCM943602CS/CDP/BAED, DW1830 | `apple_43602pci_dlarray.bin` (embedded) |
| `14e4:43a3` | BCM4350 | BCM94350ZAE, DW1820A | `apple_4350pci_dlarray.bin` (embedded) |

All three use the AppleCore `_wlc_ol_up` loader (BCDC/olmsg control plane + host-driven D11 data path).

## Build requirements

- macOS host (Sequoia 15.x or Tahoe 26.x)
- Xcode 26 with `MacOSX26.2.sdk`
- `MacOSX_DEPLOYMENT_TARGET=15.0`, `ARCHS=x86_64`, `CODE_SIGNING_ALLOWED=NO`
- `MacKernelSDK` in `MacKernelSDK/`:
  ```sh
  git clone --depth=1 https://github.com/acidanthera/MacKernelSDK.git MacKernelSDK
  ```

Quick build (embeds firmware, builds, zips): `./build.sh -p Sequoia -n ~/brcmfmac43602-pcie.txt -o brcmfmac43602-pcie`
(`./build.sh -h` for all options, `-e /Volumes/EFI/EFI/OC/Kexts` copies the kext into your EFI).

Manual build:
```sh
git clone https://github.com/serejaishkin/OpenBCM.git
cd OpenBCM
git clone --depth=1 https://github.com/acidanthera/MacKernelSDK.git MacKernelSDK
xcodebuild -project itlwm.xcodeproj -scheme AirportItlwm-Sequoia \
  -configuration Debug ARCHS=x86_64 CODE_SIGNING_ALLOWED=NO build
```

The resulting kext: `build/Products/Debug/Sequoia/OpenBCM.kext`.

## Loading

Disable SIP partially (Recovery Mode → `csrutil enable --without kext`), then:

```sh
sudo kextload /path/to/OpenBCM.kext
kextstat | grep OpenBCM
ifconfig bcm0
```

The menu bar Wi-Fi icon should appear. Scanning finds networks on channels visible to the D11 RX path; connecting performs host-driven AUTH/ASSOC through net80211.

## Device personality (Info.plist)

Per-chip configuration lives in `AirportItlwm/WCL/Info.plist` (Sequoia) and `AirportItlwm/WCL/Sequoia-Info.plist`. Each BCM chip has a personality dictionary with keys:

| Key | Meaning | Example |
|-----|---------|---------|
| `IOPCIMatch` | PCI vendor:device | `0x43BA14E4` |
| `ChipType` | `softmac` or `fullmac` | `softmac` |
| `FirmwareSource` | `embedded`, `bundle`, or `filesystem` | `embedded` |
| `FirmwareFile` | firmware blob filename | `apple_43602pci_dlarray.bin` |
| `ChipNumber` | backplane chip id (decimal) | `4393346` (0xaa52) |
| `ChipRev` | backplane chip revision | `0` |
| `D11CoreBase` | D11 core base address | `26214400` (0x1800400) |
| `NeedsCortexR4` | 1 = halt R4 before download | `1` |

Add a new personality dict under `IOKitPersonalities` to support a new Broadcom PCI ID.

## Softmac / fullmac

`ChipType` в personality выбирает режим на устройстве (один kext, оба режима): `softmac` — хост
ведёт D11 dma64, `fullmac` — MAC/PHY у прошивки, хост шлёт WLC-команды. Детали и план: `docs/FULLMAC_PLAN.md`.

## Firmware sources

Firmware is resolved in this order:

1. **Embedded** — `itlwm/hal_bcm/DlimageArrays.cpp` contains extracted Apple dlimage blobs (`_dlarray_4352pci`, etc.).
2. **Bundle resource** — `Firmware/<name>` inside the kext bundle.
3. **Filesystem** — `/Library/Extensions/OpenBCM/Firmware/<name>`.

## Project status

| Component | Status |
|-----------|--------|
| Chip bring-up (`_wlc_ol_up`) | Implemented |
| net80211 attach | Implemented |
| D11 TX/RX rings (`dma64`) | Implemented (skeleton) |
| Host-driven scan | Implemented (single channel until PHY RE) |
| Authentication / Association | TX path wired; awaiting D11 PHY channel programming |
| WPA2-PSK | Software keys via net80211 |
| WPA3 / SAE / PMF / AWDL / MLO | Not implemented |

## Missing work (help wanted)

- **D11 PHY channel switching** — need register map from `brcmfmac` (`drivers/net/wireless/broadcom/brcm80211/brcmfmac/`) or `AirPortBrcmNIC_re.bin`. See `REVERSE_ENGINEERING.md`.
- **WLC/BCDC scan/join commands** — reverse `wlc_scan`, `wlc_assoc` from AppleBCMWLANCompanion binary.
- **D11 RX data path** — descriptor parsing, beacon frame extraction.
- **WPA3 / SAE / PMF / MLO** — requires firmware key table programming.

## License

Original itlwm code: GPLv2 (laobamac). OpenBCM modifications: same as upstream. Broadcom firmware blobs: copyright their respective owners.

## Credits

- [laobamac/itlwm](https://github.com/laobamac/itlwm) — AirportItlwm framework, net80211 port
- [acidanthera/MacKernelSDK](https://github.com/acidanthera/MacKernelSDK) — kernel headers
- [OpenBSD](https://openbsd.org) — net80211
- [Linux kernel brcmfmac](https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/) — Broadcom firmware reference
