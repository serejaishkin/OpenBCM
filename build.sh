#!/bin/sh
# OpenBCM build script (macOS + Xcode). Embeds FullMAC firmware, builds the kext, zips it.
#
#   ./build.sh [options]
#     -p PLATFORM   Sequoia (default) | Tahoe | Sonoma14.4 | Sonoma14.0 | Ventura | Monterey | "Big Sur" | Catalina | Mojave | "High Sierra"
#     -c CONFIG     Release (default) | Debug
#     -n FILE       board nvram .txt to copy into firmware/fullmac/ (e.g. -n ~/brcmfmac43602-pcie.txt)
#     -o STEMS      embed only these firmware stems, comma separated (e.g. brcmfmac43602-pcie)
#     -e DIR        copy the built kext to DIR (e.g. /Volumes/EFI/EFI/OC/Kexts)
#     -s SDK        macOS SDK to build against (default: auto; Xcode 26 SDKs break MacKernelSDK, so an SDK <= 15.x is used,
#                   downloaded to ~/.openbcm/sdk if no older one is installed)
#     -h            help
set -eu
cd "$(dirname "$0")"

PLATFORM=Sequoia
CONFIG=Release
NVRAM=
ONLY=
EFI=
SDK=
while getopts "p:c:n:o:e:s:h" opt; do
    case $opt in
        p) PLATFORM=$OPTARG ;;
        c) CONFIG=$OPTARG ;;
        n) NVRAM=$OPTARG ;;
        o) ONLY=$OPTARG ;;
        e) EFI=$OPTARG ;;
        s) SDK=$OPTARG ;;
        h|*) sed -n '2,15p' "$0"; exit 0 ;;
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

# ---- MacKernelSDK + SDK selection -------------------------------------------------------
MKS_REF=3f750085caa17ec3a7880f11c11bf4f48cd6a164
if [ ! -d MacKernelSDK/Headers ]; then
    echo "== MacKernelSDK missing, cloning pinned $MKS_REF"
    git clone -q https://github.com/acidanthera/MacKernelSDK.git MacKernelSDK
    git -C MacKernelSDK checkout -q "$MKS_REF"
fi

find_old_sdk() {
    for d in "$(xcode-select -p)/Platforms/MacOSX.platform/Developer/SDKs" /Library/Developer/CommandLineTools/SDKs "$HOME/.openbcm/sdk"; do
        for f in $(ls -d "$d"/MacOSX1[1-5]*.sdk 2>/dev/null | sort -rV); do
            case "$f" in *MacOSX.sdk) continue ;; esac
            echo "$f"; return 0
        done
    done
    return 1
}
if [ -z "$SDK" ]; then
    CUR=$(xcrun --show-sdk-version 2>/dev/null | cut -d. -f1)
    if [ -n "$CUR" ] && [ "$CUR" -gt 15 ]; then
        echo "== Xcode SDK is $CUR.x: MacKernelSDK is written for SDK <= 15 (kernel APIs missing in 26 headers)"
        SDK=$(find_old_sdk || true)
        if [ -z "$SDK" ]; then
            URL=https://github.com/joseluisq/macosx-sdks/releases/download/15.0/MacOSX15.0.sdk.tar.xz
            echo "== no SDK <= 15 installed; downloading MacOSX15.0.sdk (~60 MB)"
            mkdir -p "$HOME/.openbcm/sdk"
            curl -fL --progress-bar "$URL" -o "$HOME/.openbcm/sdk/sdk.tar.xz" || die "SDK download failed; pass -s /path/to/MacOSX15.x.sdk"
            tar -xJf "$HOME/.openbcm/sdk/sdk.tar.xz" -C "$HOME/.openbcm/sdk" || die "SDK unpack failed"
            rm -f "$HOME/.openbcm/sdk/sdk.tar.xz"
            SDK=$(find_old_sdk || true)
        fi
        [ -n "$SDK" ] || die "no SDK <= 15 found; pass -s /path/to/MacOSX15.x.sdk"
    fi
fi
CCARGS=
if [ -n "$SDK" ]; then
    [ -d "$SDK" ] || die "SDK not found: $SDK"
    echo "== compiling against SDK headers: $SDK"
    # Pristine headers: the 26-SDK patch must not be applied when an older SDK is used.
    git -C MacKernelSDK checkout -q -- . 2>/dev/null || true
    # Xcode 26 cannot resolve a Command Line Tools SDK as SDKROOT (Info.plist task fails), so only
    # the compiler is redirected: the wrapper rewrites all references to Xcode's SDK onto the old one.
    mkdir -p build
    REALCLANG=$(xcrun -f clang)
    NEWSDK=$(xcrun --show-sdk-path 2>/dev/null)
    NEWSDK_REAL=$(cd "$NEWSDK" 2>/dev/null && pwd -P)
    WRAP="$PWD/build/clang-sdk-wrapper.sh"
    printf '#!/bin/sh\nexec python3 "%s" "%s" "%s" "%s:%s" -- "$@"\n' \
        "$PWD/scripts/clang_sdk_wrapper.py" "$REALCLANG" "$SDK" "$NEWSDK_REAL" "$NEWSDK" > "$WRAP"
    chmod +x "$WRAP"
    CCARGS="CC=$WRAP CPLUSPLUS=$WRAP"
else
    python3 scripts/patch_mackernelsdk.py MacKernelSDK   # best effort for new SDKs (incomplete)
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
run_xcodebuild() {
    # shellcheck disable=SC2086
    xcodebuild -project itlwm.xcodeproj $SEL -configuration "$CONFIG" -jobs 3 ARCHS=x86_64 $CCARGS \
        SYMROOT="$PWD/build/products" OBJROOT="$PWD/build/obj" \
        CODE_SIGNING_ALLOWED=NO GIT_COMMIT="_$(git rev-parse --short HEAD 2>/dev/null || echo local)" build
}
if ! run_xcodebuild > "$LOG" 2>&1; then
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
