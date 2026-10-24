#!/usr/bin/env python3
"""OpenBCM probe — run on the Mac with the card.

  openbcm_probe.py info                 # Broadcom PCI devices, IDs, ACPI path (for OpenCore DeviceProperties)
  openbcm_probe.py log [-o FILE]        # is the kext loaded + OpenBCM log from this boot + diagnosis
  openbcm_probe.py nvram [-o DIR]       # dig board nvram out of Apple's Broadcom kexts (ranked by your card)
  openbcm_probe.py all                  # info + log + nvram

nvram: Apple's AirPort Brcm kexts carry board parameter blocks ("sromrev=..", "boardtype=..", ...).
We extract every such block, rank it against your card's PCI subsystem id, and write
DIR/brcmfmac<chip>-pcie.txt for the best match plus all candidates under DIR/candidates/.
Review the result before using it; a wrong board file can make the firmware misbehave."""
import os, re, subprocess, sys, glob, argparse

BRCM_VENDOR = 0x14e4
KEXT_GLOBS = [
    "/System/Library/Extensions/IO80211Family.kext/Contents/PlugIns/AirPortBrcm*.kext",
    "/System/Library/Extensions/AirPortBrcm*.kext",
    "/Library/Extensions/AirPortBrcm*.kext",
    "/System/Library/Extensions/IO80211FamilyV2.kext/Contents/PlugIns/AirPortBrcm*.kext",
    "/System/Library/Extensions/IO80211Family.kext/Contents/PlugIns/*.kext",
    "/System/Library/KernelCollections/*",
]
CHIP_FILE = {0x43ba: "brcmfmac43602-pcie.txt", 0x43a3: "brcmfmac4350-pcie.txt",
             0x43a0: "brcmfmac4360-pcie.txt", 0x4356: "brcmfmac4356-pcie.txt",
             0x4358: "brcmfmac4358-pcie.txt", 0x4359: "brcmfmac4359-pcie.txt"}

STEPS = {1: "BAR mapping failed", 2: "cores not found in EROM (BAR0 window/EROM)", 3: "nvram alloc",
         4: "nvram strip failed (bad/empty .txt)", 5: "firmware did not start (nvram or CPU release)",
         6: "bad shared RAM info", 7: "ring plan invalid", 8: "ring/engine DMA", 9: "unknown chip RAM base",
         10: "flowring init", 11: "data path init"}

# ---------------------------------------------------------------- helpers
def run(cmd, timeout=60):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception as e:
        return ""

def le_hex(v):
    m = re.search(r"<([0-9a-fA-F]+)>", v or "")
    if not m: return None
    b = bytes.fromhex(m.group(1))
    return int.from_bytes(b[:4].ljust(4, b"\0"), "little")

# ---------------------------------------------------------------- info
def pci_devices(text=None):
    text = text if text is not None else run(["ioreg", "-r", "-c", "IOPCIDevice", "-l", "-w0"])
    blocks = re.split(r"(?m)^[\s|]*\+-o ", text)
    devs = []
    for b in blocks[1:]:
        def prop(k):
            m = re.search(r'"%s" = (.+)' % re.escape(k), b)
            return m.group(1).strip() if m else None
        ven = le_hex(prop("vendor-id")); dev = le_hex(prop("device-id"))
        if ven != BRCM_VENDOR: continue
        devs.append({
            "name": b.split("<")[0].strip(),
            "device": dev, "subvendor": le_hex(prop("subsystem-vendor-id")),
            "subsystem": le_hex(prop("subsystem-id")), "rev": le_hex(prop("revision-id")),
            "acpi": (prop("acpi-path") or "").strip('"'),
        })
    return devs

def acpi_to_pciroot(acpi):
    """IOACPIPlane:/_SB/PCI0@0/RP03@1c0002/ARPT@0 -> PciRoot(0x0)/Pci(0x1C,0x2)/Pci(0x0,0x0)"""
    segs = re.findall(r"/([A-Za-z0-9_]+)@([0-9a-fA-F]+)", acpi or "")
    if len(segs) < 2: return None
    out = ["PciRoot(0x%X)" % int(segs[0][1], 16)]
    for _, v in segs[1:]:
        n = int(v, 16)
        out.append("Pci(0x%X,0x%X)" % (n >> 16, n & 0xffff))
    return "/".join(out)

FW_STEM = {0x43ba: "brcmfmac43602-pcie", 0x43a3: "brcmfmac4350-pcie (or 4350c2)", 0x4356: "brcmfmac4356-pcie",
           0x4358: "brcmfmac4358-pcie", 0x43a0: None}

def cmd_info():
    devs = pci_devices()
    if not devs:
        print("No Broadcom (14e4) PCI device found in IORegistry.")
        return devs
    for d in devs:
        print("Broadcom device: 14e4:%04x  subsystem %s:%s  rev %s  [%s]" % (
            d["device"] or 0,
            "%04x" % d["subvendor"] if d["subvendor"] is not None else "?",
            "%04x" % d["subsystem"] if d["subsystem"] is not None else "?",
            "%02x" % d["rev"] if d["rev"] is not None else "?", d["name"]))
        print("  ACPI path:", d["acpi"] or "(not exposed) — run `gfxutil -f %s` for the PciRoot(...) path" % d["name"].split("@")[0])
        pr = acpi_to_pciroot(d["acpi"])
        if pr: print("  OpenCore path:", pr)
        if d["device"] == 0x43a0:
            print("  note: BCM4360 has no Linux FullMAC firmware; softmac only (openbcm-mode = softmac)")
        elif d["device"] in FW_STEM:
            print("  DeviceProperties for this card (String): openbcm-mode = fullmac, openbcm-fw = %s" % FW_STEM[d["device"]])
    print("\nAll DeviceProperties are optional (auto-selected by chip/rev); all are String type.")
    return devs

# ---------------------------------------------------------------- log
MILESTONES = [
    ("bcm: chip 0x", "chip probed"),
    ("mode=fullmac", "fullmac selected"),
    ("fullmac firmware auto-selected", "firmware chosen"),
    ("OpenBCM fullmac: BAR0", "BARs mapped"),
    ("cores (cc=1 cr4=1 pcie2=1 imem=1)", "EROM scan ok"),
    ("RAM base", "RAM detected"),
    ("firmware start status 0", "firmware booted"),
    ("bcm: MAC ", "ioctl round trip works"),
    ("fullmac preinit: 0 step(s) failed", "firmware configured"),
    ("fullmac firmware up", "attach complete"),
]

def diagnose(text):
    out = []
    for needle, label in MILESTONES:
        out.append(("[x] " if needle in text else "[ ] ") + label)
    m = re.search(r"fullmac start failed at step (\d+)", text)
    if m:
        s = int(m.group(1))
        out.append("\nFAILED at step %d: %s" % (s, STEPS.get(s, "?")))
    m = re.search(r"firmware start status (\d+)", text)
    if m and m.group(1) != "0":
        out.append("firmware start status %s (1=no core, 2=timeout, 3=bad shared addr, 4=bad RAM, 5=bad arg)" % m.group(1))
    if "blobs not embedded" in text:
        out.append("\nFirmware/nvram are not inside the kext: put brcmfmac*.bin/.txt in firmware/fullmac and rebuild.")
    m = re.search(r"preinit: (\d+) step", text)
    if m and m.group(1) != "0":
        out.append("preinit: %s step(s) failed (see the ioctl lines above)" % m.group(1))
    return "\n".join(out)

def cmd_log(outfile):
    loaded = run(["kmutil", "showloaded", "--list-only"]) or run(["kextstat"])
    hits = [l for l in loaded.splitlines() if re.search(r"serejaishkin|openbcm", l, re.I)]
    stock = [l for l in loaded.splitlines() if re.search(r"zxystd|itlwm", l, re.I) and l not in hits]
    print("OpenBCM kext loaded:", "YES" if hits else "NO")
    for l in hits: print("  " + l.strip())
    if stock:
        print("Stock itlwm/AirportItlwm kext(s) loaded (NOT OpenBCM):")
        for l in stock: print("  " + l.strip())
        if not hits:
            print("  -> you are running upstream itlwm, which has no Broadcom support. Build the\n"
                  "     AirportItlwm-<your macOS> target (bundle id com.serejaishkin.OpenBCM), put it in\n"
                  "     EFI/OC/Kexts, list it in config.plist Kernel->Add and remove/disable the stock itlwm.")
    log = run(["log", "show", "--last", "boot", "--style", "compact", "--predicate",
               'eventMessage CONTAINS[c] "bcm" OR eventMessage CONTAINS[c] "OpenBCM"'], 120)
    if not log:
        log = run(["sh", "-c", "dmesg | grep -i -E 'bcm|OpenBCM'"])
    outfile = outfile or "openbcm.log"
    with open(outfile, "w") as f: f.write(log)
    print("Log written to %s (%d lines)\n" % (outfile, log.count("\n")))
    print(diagnose(log))
    print("\nSend me %s." % outfile)

# ---------------------------------------------------------------- nvram
KV = re.compile(rb"(?:[A-Za-z0-9_.\[\]]{1,40}=[\x20-\x7e]{0,200}[\x00\n]){8,}")

def extract_blocks(data):
    blocks = []
    for m in KV.finditer(data):
        raw = m.group(0)
        if b"boardtype=" not in raw and b"sromrev=" not in raw: continue
        lines = [l.decode("ascii") for l in re.split(rb"[\x00\n]", raw) if l]
        blocks.append(lines)
    return blocks

def kv(lines):
    d = {}
    for l in lines:
        k, _, v = l.partition("=")
        d.setdefault(k, v)
    return d

def score(lines, sub, dev):
    d = kv(lines)
    s = 0
    try:
        if sub is not None and int(d.get("boardtype", "x"), 0) == sub: s += 10
    except ValueError: pass
    if "macaddr" in d: s += 1
    if "sromrev" in d: s += 1
    s += min(len(lines), 200) / 200.0
    return s

def cmd_nvram(outdir, devs=None):
    devs = devs if devs is not None else pci_devices()
    dev = devs[0] if devs else {}
    sub = dev.get("subsystem"); did = dev.get("device")
    outdir = outdir or "nvram_out"
    os.makedirs(os.path.join(outdir, "candidates"), exist_ok=True)
    seen, cands = set(), []
    files = []
    for g in KEXT_GLOBS:
        for p in glob.glob(g):
            if os.path.isdir(p):
                files += [f for f in glob.glob(os.path.join(p, "Contents", "MacOS", "*")) if os.path.isfile(f)]
            elif os.path.isfile(p):
                files.append(p)
    for f in files:
        try: data = open(f, "rb").read()
        except OSError: continue
        for lines in extract_blocks(data):
            key = "\n".join(lines)
            if key in seen: continue
            seen.add(key)
            cands.append((score(lines, sub, did), os.path.basename(f), lines))
    cands.sort(key=lambda c: -c[0])
    print("Scanned %d kext binaries, found %d distinct nvram-like blocks." % (len(files), len(cands)))
    for i, (sc, src, lines) in enumerate(cands):
        d = kv(lines)
        name = "%02d_%s_boardtype-%s.txt" % (i, src, d.get("boardtype", "none"))
        open(os.path.join(outdir, "candidates", name), "w").write("\n".join(lines) + "\n")
        print("  %s  score %.2f  boardtype=%s boardrev=%s macaddr=%s" % (
            name, sc, d.get("boardtype"), d.get("boardrev"), d.get("macaddr")))
    if cands:
        best = os.path.join(outdir, CHIP_FILE.get(did, "brcmfmac-board-pcie.txt"))
        open(best, "w").write("\n".join(cands[0][2]) + "\n")
        print("\nBest guess written to %s (boardtype match with subsystem %s: %s)" % (
            best, "%04x" % sub if sub is not None else "?",
            "yes" if cands[0][0] >= 10 else "NO — verify before use"))
        print("Copy it to firmware/fullmac/ under the firmware's name (e.g. brcmfmac43602-pcie.txt) and rebuild.")
    else:
        print("Nothing found. This macOS may keep the data compressed/in the kernel collection; "
              "try on an older macOS, or send me `ioreg -l -w0 | grep -i -E 'boardtype|sromrev|macaddr'`.")

# ---------------------------------------------------------------- main
def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["info", "log", "nvram", "all"])
    ap.add_argument("-o", "--out")
    a = ap.parse_args(argv)
    if a.cmd in ("info", "all"): devs = cmd_info(); print()
    else: devs = None
    if a.cmd in ("log", "all"): cmd_log(a.out if a.cmd == "log" else None); print()
    if a.cmd in ("nvram", "all"): cmd_nvram(a.out if a.cmd == "nvram" else None, devs)

if __name__ == "__main__":
    main(sys.argv[1:])
