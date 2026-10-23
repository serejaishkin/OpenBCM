#!/bin/sh
# Usage: scripts/openbcm_probe.sh [info|log|nvram|all] [-o OUT]   (default: all)
cd "$(dirname "$0")"
command -v python3 >/dev/null 2>&1 || { echo "python3 not found (install Xcode Command Line Tools: xcode-select --install)"; exit 1; }
[ $# -eq 0 ] && set -- all
exec python3 -I openbcm_probe.py "$@"
