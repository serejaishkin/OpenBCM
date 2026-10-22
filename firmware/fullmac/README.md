# FullMAC firmware for BCM43602 (PCI 14e4:43ba)

- `brcmfmac43602-pcie.bin` — Broadcom firmware 7.35.177.61 (43602a1, msgbuf/PCIe), from linux-firmware.
  Redistributed under `LICENCE.broadcom_bcm43xx` (same directory).
- `brcmfmac43602-pcie.txt` — **board nvram, add it yourself** (taken from your card/macOS or a board
  file matching it). Not in linux-firmware for Apple boards.

Then embed both into the kext and rebuild:

    python3 scripts/gen_fullmac_arrays.py

(The script defaults to the two files above.) Commit nothing else: the generated
`itlwm/hal_bcm/FullmacFwArrays.cpp` can stay local or be committed if you want CI builds to include it.
