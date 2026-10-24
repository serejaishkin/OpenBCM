#!/bin/sh
# OpenBCM build script (macOS + Xcode). Embeds FullMAC firmware, builds the kext, zips it.
#
#   ./build.sh [options]
#     -p PLATFORM   Sequoia (default) | Tahoe | Sonoma14.4 | Sonoma14.0 | Ventura | Monterey | "Big Sur" | Catalina | Mojave | "High Sierra"
#     -c CONFIG     Release (default) | Debug
#     -n FILE       board nvram .txt to copy into firmware/fullmac/ (e.g. -n ~/brcmfmac43602-pcie.txt)
#     -o STEMS      embed only these firmware stems, comma separated (e.g. brcmfmac43602-pcie)
#     -e DIR        copy the built kext to DIR (e.g. /Volumes/EFI/EFI/OC/Kexts)
#     -h            help
set -eu
cd "$(dirname "$0")"

PLATFORM=Sequoia
CONFIG=Release
NVRAM=
ONLY=
EFI=
while getopts "p:c:n:o:e:h" opt; do
    case $opt in
        p) PLATFORM=$OPTARG ;;
        c) CONFIG=$OPTARG ;;
        n) NVRAM=$OPTARG ;;
        o) ONLY=$OPTARG ;;
        e) EFI=$OPTARG ;;
        h|*) sed -n '2,12p' "$0"; exit 0 ;;
    esac
done

die() { echo "error: $*" >&2; exit 1; }

[ "$(uname -s)" = Darwin ] || die "needs macOS with Xcode (this is $(uname -s))"
command -v xcodebuild >/dev/null 2>&1 || die "xcodebuild not found (install Xcode / run xcode-select --install)"
command -v python3 >/dev/null 2>&1 || die "python3 not found (xcode-select --install)"

FWDIR=firmware/fullmac
if [ -n "$NVRAM" ]; then
    [ -f "$NVRAM" ] || die "nvram file not found: $NVRAM"
    cp "$NVRAM" "$FWDIR/$(basename "$NVRAM")"
    echo "nvram: copied $(basename "$NVRAM") into $FWDIR/"
fi

# Firmware + nvram must both be present, otherwise fullmac attach fails at runtime.
NBIN=$(ls "$FWDIR"/*.bin 2>/dev/null | wc -l | tr -d ' ')
NTXT=$(ls "$FWDIR"/*.txt 2>/dev/null | wc -l | tr -d ' ')
[ "$NBIN" -gt 0 ] || die "no firmware (*.bin) in $FWDIR"
if [ "$NTXT" -eq 0 ]; then
    echo "WARNING: no nvram (*.txt) in $FWDIR - fullmac will not start. Get it with:"
    echo "         python3 scripts/openbcm_probe.py nvram -o $FWDIR   (or pass -n FILE)"
fi

echo "== embedding firmware"
if [ -n "$ONLY" ]; then
    python3 scripts/gen_fullmac_arrays.py --only "$ONLY"
else
    python3 scripts/gen_fullmac_arrays.py
fi

echo "== building AirportItlwm-$PLATFORM ($CONFIG, x86_64)"
mkdir -p logs
LOG="logs/build-$PLATFORM.log"
if [ -f "itlwm.xcodeproj/xcshareddata/xcschemes/AirportItlwm-$PLATFORM.xcscheme" ]; then
    SEL="-scheme AirportItlwm-$PLATFORM"
else
    SEL="-target AirportItlwm-$PLATFORM"
fi
# shellcheck disable=SC2086
if ! xcodebuild -project itlwm.xcodeproj $SEL -configuration "$CONFIG" -jobs 3 ARCHS=x86_64 \
        SYMROOT="$PWD/build/products" OBJROOT="$PWD/build/obj" \
        CODE_SIGNING_ALLOWED=NO GIT_COMMIT="_$(git rev-parse --short HEAD 2>/dev/null || echo local)" \
        build > "$LOG" 2>&1; then
    tail -60 "$LOG"
    die "build failed, full log: $LOG"
fi

KEXT=$(find build/products -type d -name OpenBCM.kext -path "*$CONFIG*" 2>/dev/null | head -1)
[ -n "$KEXT" ] || KEXT=$(find build/products -type d -name '*.kext' -path "*$CONFIG*" | head -1)
[ -n "$KEXT" ] || die "build finished but no .kext found under build/products"

OUT="build/OpenBCM-$(echo "$PLATFORM" | tr ' ' '_')-$CONFIG"
rm -rf "$OUT" "$OUT.zip"
mkdir -p "$OUT"
cp -R "$KEXT" "$OUT/"
(cd build && zip -qr "$(basename "$OUT").zip" "$(basename "$OUT")")
echo "== kext: $OUT/$(basename "$KEXT")"
echo "== zip : $OUT.zip"

if [ -n "$EFI" ]; then
    [ -d "$EFI" ] || die "EFI kexts dir not found: $EFI"
    rm -rf "$EFI/$(basename "$KEXT")"
    cp -R "$KEXT" "$EFI/"
    echo "== copied to $EFI (remember: Kernel->Add in config.plist, remove stock itlwm)"
fi
echo "done. After reboot: python3 scripts/openbcm_probe.py all"
