// OlMsg.hpp — BCDC/olmsg host<->firmware control queue, decoded from
// AirPortBrcmNIC's `_bcm_olmsg_*` family (see DLIMAGE_FINDINGS.md
// "Precise olmsg layout"). Pure C++, no IOKit, so it can be unit-tested on the
// host (scripts/test_olmsg.cpp).
//
// The one 64 KB coherent DMA buffer is laid out as:
//   +0x0000  TX ring-desc {offset=0x20,   size=0x7800, rd, wr}
//   +0x0010  RX ring-desc {offset=0x7820, size=0x7800, rd, wr}
//   +0x0020  TX ring data (0x7800)
//   +0x7820  RX ring data (0x7800)
// Messages are 12-byte header {u32 id, u32 seq, u32 payloadLen} + payload.

#ifndef OlMsg_hpp
#define OlMsg_hpp

#include <stdint.h>
#include <stddef.h>

namespace ol_msg {

static constexpr uint32_t kRingSize   = 0x7800;
static constexpr uint32_t kTxOffset   = 0x20;
static constexpr uint32_t kRxOffset   = kTxOffset + kRingSize; // 0x7820
static constexpr uint32_t kTotalSize  = kRxOffset + kRingSize; // 0xF020
static constexpr uint32_t kBufferSize = 0x10000;
static constexpr uint32_t kHeaderSize = 0x0c;

// Circular-buffer control block (0x10 bytes) at buf[0] and buf[0x10].
struct RingDesc {
    uint32_t offset;
    uint32_t size;
    uint32_t rd;
    uint32_t wr;
};

// Main olmsg struct (mirrors the 0x28-byte struct at `ol+0x50`).
struct Info {
    uint8_t   *buf;
    uint32_t   size;
    RingDesc  *tx;   // host -> device
    RingDesc  *rx;   // device -> host
    uint32_t   seq;
};

enum {
    kOk          = 0,
    kErrNull     = -2,          // 0xfffffffe
    kErrTooSmall = -14,         // 0xfffffff2
    kErrNoTx     = -25,         // 0xffffffe7
    kErrFull     = -22,         // 0xffffffea
};

// `_bcm_olmsg_create`: initialise the two ring descriptors at the buffer start.
inline int create(uint8_t *buf, uint32_t bufSize) {
    if (!buf || bufSize < 0x50) {
        return kErrNull;
    }
    RingDesc *d = (RingDesc *)buf;
    d[0].offset = kTxOffset;
    d[0].size   = kRingSize;
    d[0].rd = 0;
    d[0].wr = 0;
    d[1].offset = kRxOffset;
    d[1].size   = kRingSize;
    d[1].rd = 0;
    d[1].wr = 0;
    if (d[0].offset + d[0].size + d[1].size > bufSize) {
        return kErrTooSmall;
    }
    return kOk;
}

// `_bcm_olmsg_init`: bind the main struct to the buffer and its two descriptors.
inline int init(Info *m, uint8_t *buf, uint32_t size, uint8_t txDescIdx, uint8_t rxDescIdx) {
    if (!m || !buf || size == 0) {
        return kErrNull;
    }
    m->buf  = buf;
    m->size = size;
    m->tx   = (RingDesc *)(buf + (uint32_t)txDescIdx * sizeof(RingDesc));
    m->rx   = (RingDesc *)(buf + (uint32_t)rxDescIdx * sizeof(RingDesc));
    m->seq  = 0;
    return kOk;
}

// `_bcm_olmsg_avail`: free space in the TX ring (= size-1-used).
inline uint32_t avail(const Info *m) {
    if (!m || !m->tx) {
        return 0;
    }
    const RingDesc *d = m->tx;
    uint32_t free = d->rd - d->wr - 1;
    if (d->rd <= d->wr) {
        free += d->size;
    }
    return free;
}

// `_bcm_olmsg_bytes_to_read`: used bytes in the RX ring.
inline uint32_t bytesToRead(const Info *m) {
    if (!m || !m->rx) {
        return 0;
    }
    const RingDesc *d = m->rx;
    uint32_t used = d->wr - d->rd;
    if (d->wr < d->rd) {
        used += d->size;
    }
    return used;
}

inline bool isWriteFull(const Info *m)  { return m && m->tx && ((m->tx->wr + 1) % m->tx->size) == m->tx->rd; }
inline bool isWriteEmpty(const Info *m) { return m && m->tx && m->tx->wr == m->tx->rd; }
inline bool isReadEmpty(const Info *m)  { return !m || !m->rx || m->rx->wr == m->rx->rd; }

// `_bcm_olmsg_writemsg`: copy `len` bytes of `msg` into the TX ring (stamping
// msg[+4] with the sequence number) and advance the TX write index.
inline int writeMsg(Info *m, uint8_t *msg, uint16_t len) {
    if (!m || !m->tx) {
        return kErrNoTx;
    }
    RingDesc *d = m->tx;
    if (avail(m) < len) {
        return kErrFull;
    }
    uint8_t *ring = m->buf + d->offset;
    uint32_t wr = d->wr;
    msg[4] = (uint8_t)(m->seq & 0xff);
    msg[5] = (uint8_t)((m->seq >> 8) & 0xff);
    msg[6] = (uint8_t)((m->seq >> 16) & 0xff);
    msg[7] = (uint8_t)((m->seq >> 24) & 0xff);
    m->seq++;

    uint32_t first = d->size - wr;
    uint32_t n1 = (len < first) ? len : first;
    for (uint32_t i = 0; i < n1; i++) {
        ring[wr + i] = msg[i];
    }
    wr = (wr + n1) % d->size;
    for (uint32_t i = n1; i < len; i++) {
        ring[wr++] = msg[i];
        if (wr == d->size) {
            wr = 0;
        }
    }
    d->wr = wr;
    return kOk;
}

// `_bcm_olmsg_peekbytes`: read `len` bytes from the RX ring without advancing.
// Returns the number of bytes copied (0 if the RX ring has fewer than `len`).
inline uint32_t peekBytes(const Info *m, uint8_t *out, uint32_t len) {
    if (!m || !m->rx) {
        return 0;
    }
    const RingDesc *d = m->rx;
    if (d->wr == d->rd) {
        return 0;
    }
    if (bytesToRead(m) < len) {
        return 0;
    }
    const uint8_t *ring = m->buf + d->offset;
    uint32_t rd = d->rd;
    uint32_t n1 = d->size - rd;
    if (n1 >= len) {
        for (uint32_t i = 0; i < len; i++) {
            out[i] = ring[rd + i];
        }
        return len;
    }
    for (uint32_t i = 0; i < n1; i++) {
        out[i] = ring[rd + i];
    }
    for (uint32_t i = n1; i < len; i++) {
        out[i] = ring[i - n1];
    }
    return len;
}

// `_bcm_olmsg_readbytes`: peekBytes then advance the RX read index.
inline uint32_t readBytes(Info *m, uint8_t *out, uint32_t len) {
    uint32_t n = peekBytes(m, out, len);
    if (m && m->rx) {
        m->rx->rd = (m->rx->rd + n) % m->rx->size;
    }
    return n;
}

// `_bcm_olmsg_peekmsg_len`: total length of the next RX message, or 0.
inline uint16_t peekMsgLen(const Info *m) {
    if (!m || !m->rx || bytesToRead(m) < kHeaderSize) {
        return 0;
    }
    uint8_t hdr[kHeaderSize];
    if (peekBytes(m, hdr, kHeaderSize) != kHeaderSize) {
        return 0;
    }
    uint32_t payload = (uint32_t)hdr[8] | ((uint32_t)hdr[9] << 8)
                     | ((uint32_t)hdr[10] << 16) | ((uint32_t)hdr[11] << 24);
    return (uint16_t)(payload + kHeaderSize);
}

// `_bcm_olmsg_readmsg`: read one full message (header+payload) into `out`.
// `outLen` must be >= the required message length. Returns bytes read.
inline uint16_t readMsg(Info *m, uint8_t *out, uint16_t outLen) {
    uint16_t len = peekMsgLen(m);
    if (!out || len == 0 || (uint16_t)(len - 1) >= outLen) {
        return 0;
    }
    uint32_t n = readBytes(m, out, len);
    return (n == len) ? len : 0;
}

} // namespace ol_msg

#endif /* OlMsg_hpp */
