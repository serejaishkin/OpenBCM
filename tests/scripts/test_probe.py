import sys, os, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
import openbcm_probe as p

# --- ioreg parsing
IOREG = '''+-o ARPT@0  <class IOPCIDevice, id 0x1, registered>
    {
      "vendor-id" = <e4140000>
      "device-id" = <ba430000>
      "subsystem-vendor-id" = <6b100000>
      "subsystem-id" = <45010000>
      "revision-id" = <01000000>
      "acpi-path" = "IOACPIPlane:/_SB/PCI0@0/RP05@1c0004/ARPT@0"
    }
+-o GPU@0  <class IOPCIDevice, id 0x2>
    {
      "vendor-id" = <de100000>
      "device-id" = <11110000>
    }
'''
devs = p.pci_devices(IOREG)
assert len(devs) == 1 and devs[0]["device"] == 0x43ba and devs[0]["subsystem"] == 0x145, devs
assert devs[0]["acpi"].endswith("ARPT@0")

# --- nvram extraction ranking
def blob(boardtype):
    kvs = ["sromrev=11", "boardtype=0x%x" % boardtype, "boardrev=0x1301", "macaddr=00:90:4c:c5:12:38",
           "boardflags=0x10000", "boardflags2=0x0", "ccode=0", "regrev=0", "aa2g=7"]
    return ("\0".join(kvs) + "\0\0").encode()
data = b"\x7fELF" + os.urandom(64) + blob(0x145) + os.urandom(50) + blob(0x999) + b"tail"
blocks = p.extract_blocks(data)
assert len(blocks) == 2, blocks
sc = sorted(blocks, key=lambda l: -p.score(l, 0x145, 0x43ba))
assert p.kv(sc[0])["boardtype"] == "0x145"

# --- log diagnosis
good = "bcm: chip 0x aa52 mode=fullmac\nOpenBCM fullmac: BAR0 1 bytes\nfullmac firmware auto-selected: x\n"
d = p.diagnose(good + "OpenBCM fullmac: 5 cores (cc=1 cr4=1 pcie2=1 imem=1)\nOpenBCM fullmac: firmware start status 2 shared=0x0\nbcm: fullmac start failed at step 5\n")
assert "[x] EROM scan ok" in d and "[ ] firmware booted" in d and "step 5" in d and "timeout" in d
assert p.acpi_to_pciroot("IOACPIPlane:/_SB/PCI0@0/RP03@1c0002/ARPT@0") == "PciRoot(0x0)/Pci(0x1C,0x2)/Pci(0x0,0x0)"
assert p.acpi_to_pciroot("") is None
print("probe tests OK")
