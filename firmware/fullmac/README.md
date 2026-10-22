# FullMAC firmware (brcmfmac / linux-firmware)

`*.bin` = firmware, `*.txt` = board nvram (stem = file name without extension). Everything in this
directory is embedded into the kext by `python3 scripts/gen_fullmac_arrays.py` (use `--only stem,stem`
to embed just what you need). Firmware blobs are Broadcom's, redistributed under
`LICENCE.broadcom_bcm43xx`.

Included: 43602, 4350, 4350c2, 4358, 4366b, 4366c, 4371.
Not found in the mirrors I could reach: 4356, 43570, 4359(c), 4365b/c — drop the files here yourself.
BCM4360 has **no** Linux FullMAC firmware (softmac only).

**nvram is yours to add** (board specific; not in linux-firmware for Apple boards), named like the
firmware, e.g. `brcmfmac43602-pcie.txt`.

## OpenCore DeviceProperties (PciRoot(...)/Pci(...) of the card)
| key | value | effect |
|---|---|---|
| `openbcm-mode` | `fullmac` / `softmac` | overrides the personality (43602 defaults to fullmac) |
| `openbcm-fw` | e.g. `brcmfmac4350c2-pcie` | firmware stem to use (default: auto by chip/rev as in brcmfmac) |
| `openbcm-nvram` | e.g. `brcmfmac43602-pcie.apple,j137` | nvram stem (default: same stem as the firmware) |

Values are plain strings (`String` type in config.plist).
