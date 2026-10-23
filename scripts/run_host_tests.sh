#!/bin/sh
# Build and run the pure-C++ host tests (no IOKit / Xcode required).
set -eu
cd "$(dirname "$0")/.."
CXX=${CXX:-c++}
mkdir -p build/host
for t in tests/host/test_*.cpp; do
    name=$(basename "$t" .cpp)
    "$CXX" -std=c++17 -Wall -Wextra -Werror -Iitlwm/hal_bcm "$t" -o "build/host/$name"
    "build/host/$name"
done
# Syntax-only check of the IOKit glue against stub headers (no Xcode needed).
"$CXX" -std=c++17 -fsyntax-only -Wall -Wextra -Werror -Itests/stubs -Iitlwm/hal_bcm tests/stubs/syntax_fullmac.cpp

command -v python3 >/dev/null 2>&1 && python3 -I tests/scripts/test_probe.py
echo "all tests passed"
