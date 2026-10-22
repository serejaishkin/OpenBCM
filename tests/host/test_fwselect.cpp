#include <stdio.h>
#include <string.h>
#include "FullmacFwSelect.hpp"
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)
int main() {
    CHECK(!strcmp(fwsel::stemFor(43602, 1), "brcmfmac43602-pcie"));
    CHECK(!strcmp(fwsel::stemFor(0x4350, 2), "brcmfmac4350c2-pcie"));     // rev 2 -> c2
    CHECK(!strcmp(fwsel::stemFor(0x4350, 8), "brcmfmac4350-pcie"));
    CHECK(!strcmp(fwsel::stemFor(0x4366, 3), "brcmfmac4366b-pcie"));
    CHECK(!strcmp(fwsel::stemFor(0x4366, 4), "brcmfmac4366c-pcie"));
    CHECK(!strcmp(fwsel::stemFor(0x4359, 9), "brcmfmac4359c-pcie"));
    CHECK(fwsel::stemFor(0x4360, 3) == nullptr);                           // no Linux FullMAC fw for 4360
    CHECK(fwsel::ramBase(43602, 1) == 0x180000);
    CHECK(fwsel::ramBase(0x4365, 4) == 0x200000);
    CHECK(fwsel::ramBase(0x4359, 9) == 0x160000 && fwsel::ramBase(0x4359, 3) == 0x180000);
    CHECK(fwsel::ramBase(0x4360, 3) == 0xFFFFFFFF);
    char b[64];
    fwsel::normalizeStem("/x/y/brcmfmac4350-pcie.bin", b, sizeof b); CHECK(!strcmp(b, "brcmfmac4350-pcie"));
    fwsel::normalizeStem("brcmfmac43602-pcie.apple,j137.txt", b, sizeof b); CHECK(!strcmp(b, "brcmfmac43602-pcie.apple,j137"));
    fwsel::normalizeStem("plain", b, sizeof b); CHECK(!strcmp(b, "plain"));
    if (g_fail) return 1;
    printf("fwselect: OK\n");
    return 0;
}
