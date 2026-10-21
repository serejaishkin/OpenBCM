// Nvram.hpp — board NVRAM text -> firmware blob, pure C++.
// Port of brcmf_fw_nvram_strip() (brcmfmac/firmware.c) for single-device files:
//   * "key=value" lines, '#' comments, blank/invalid lines skipped,
//   * entries joined with NUL, "RAW1*" ignored,
//   * "boardrev=0xff" appended when missing,
//   * padded with NULs to a 4-byte multiple (at least one terminating NUL),
//   * followed by the length token le32((~words << 16) | (words & 0xffff)),
//     where words = padded length / 4.
// The blob is written at the END of dongle RAM by the PCIe transport.
// Multi-device files (devpath*=, pcie/N/M/...) are rejected, not stripped.

#ifndef Nvram_hpp
#define Nvram_hpp

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace nvram {

static constexpr size_t kMaxSize = 64000;

enum class Result : uint8_t { kOk, kEmpty, kMultiDevice, kTooSmall };

struct Info {
    size_t   length {0};         // total bytes written to `out` incl. token
    bool     boardrevAdded {false};
};

inline bool isNvramChar(uint8_t c) { return c != '#' && c >= 0x20 && c < 0x7f; }
inline bool isWhitespace(uint8_t c) { return c == ' ' || c == '\r' || c == '\n' || c == '\t'; }

// `out` must have room for roughly inLen + 32 bytes; kTooSmall is returned otherwise.
inline Result strip(const uint8_t *in, size_t inLen, uint8_t *out, size_t outCap, Info *info) {
    enum { IDLE, KEY, VALUE, COMMENT, END } st = IDLE;
    size_t limit = inLen > kMaxSize ? kMaxSize : inLen;
    size_t pos = 0, entry = 0, n = 0;
    bool multi = false, boardrev = false;
    // Worst case: input copied (<= limit) + boardrev + pad + token.
    if (outCap < limit + 14 + 1 + 3 + 4) return Result::kTooSmall;

    auto starts = [&](size_t at, const char *lit) {
        size_t l = strlen(lit);
        return at + l <= inLen && memcmp(in + at, lit, l) == 0;
    };

    while (pos < limit && st != END) {
        uint8_t c = in[pos];
        switch (st) {
        case IDLE:
            if (c == '\n' || c == '#') { st = COMMENT; break; }
            if (isWhitespace(c) || c == 0) { pos++; break; }
            if (isNvramChar(c)) { entry = pos; st = KEY; break; }
            pos++;                                   // invalid char: skip
            break;
        case KEY:
            if (c == '=') {
                st = starts(entry, "RAW1") ? COMMENT : VALUE;
                if (starts(entry, "devpath") || starts(entry, "pcie/")) multi = true;
                if (starts(entry, "boardrev")) boardrev = true;
                pos++;
            } else if (!isNvramChar(c) || c == ' ') {
                st = COMMENT;                        // '=' expected: skip entry
            } else {
                pos++;
            }
            break;
        case VALUE:
            if (!isNvramChar(c)) {
                size_t cp = pos - entry;
                if (n + cp + 1 >= kMaxSize) { st = END; break; }
                memcpy(out + n, in + entry, cp);
                n += cp;
                out[n++] = 0;
                st = IDLE;
            } else {
                pos++;
            }
            break;
        case COMMENT: {
            size_t e = pos;
            while (e < inLen && in[e] != '\n' && in[e] != 0) e++;
            pos = e + 1;
            st = IDLE;
            break;
        }
        case END: break;
        }
    }
    // A value that runs to end-of-input (no trailing newline) is still complete.
    if (st == VALUE && pos > entry && n + (pos - entry) + 1 < kMaxSize) {
        memcpy(out + n, in + entry, pos - entry);
        n += pos - entry;
        out[n++] = 0;
    }
    if (multi) return Result::kMultiDevice;
    if (n == 0) return Result::kEmpty;

    bool added = false;
    if (!boardrev) {
        static const char kDefault[] = "boardrev=0xff";
        memcpy(out + n, kDefault, sizeof(kDefault));   // includes NUL
        n += sizeof(kDefault);
        added = true;
    }
    size_t padded = (n + 1 + 3) & ~(size_t)3;           // roundup(len + 1, 4)
    while (n < padded) out[n++] = 0;
    uint32_t words = (uint32_t)(padded / 4);
    uint32_t token = (~words << 16) | (words & 0xFFFF);
    for (int i = 0; i < 4; i++) out[n + i] = (uint8_t)(token >> (8 * i));
    n += 4;
    if (info) { info->length = n; info->boardrevAdded = added; }
    return Result::kOk;
}

} // namespace nvram

#endif /* Nvram_hpp */
