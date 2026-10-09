/*
 * X protocol: drive-timed transfers, two bits per bus write on CLK+DATA
 * Copyright (c) 2026 Josh Bailey
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Per byte the adapter asserts DATA ("go") with interrupts masked, waits for
 * CLK to read released and then asserted (the drive's SYNC write), and from
 * that poll samples or drives the four (DATA, CLK) pairs at fixed clock
 * offsets. The offsets sit in the middle of the windows given by the drive's
 * cycle schedule (nybulah drive/proto_x.inc, docs/protocol.md), widened by the
 * budgets below. If no SYNC arrives within a slice, go is withdrawn and CLK
 * is watched for X_GRACE more drive cycles, so a drive that already saw go is
 * still served; otherwise interrupts and TimerWorker() run before retrying.
 */
#include "x_timing.h"
#include "xb_ready.h"
#include "xum1541.h"

#ifdef X_SUPPORT

#if F_CPU != 16000000
#error "X protocol offsets assume a 16 MHz adapter clock"
#endif

/* Drive -> host: drive cycles after SYNC of P0..P3 and the earliest REL. */
#define XW0 13
#define XW1 25
#define XW2 35
#define XW3 47
#define XW4 57
/* Host -> drive: SYNC release and the drive's reads of P0..P3. */
#define XR0 6
#define XR1 12
#define XR2 21
#define XR3 34
#define XR4 43

__asm__(".macro xdelay n\n"
        " .if (\\n) < 0\n"
        "  .error \"negative X delay\"\n"
        " .endif\n"
        " .if (\\n) / 3\n"
        "  ldi r23, (\\n) / 3\n"
        "1: dec r23\n"
        "  brne 1b\n"
        " .endif\n"
        " .if (\\n) % 3 == 1\n"
        "  nop\n"
        " .elseif (\\n) % 3 == 2\n"
        "  rjmp .+0\n"
        " .endif\n"
        ".endm\n"
        /*
         * Wait for SYNC (pin bit sb, CLK by default) released then asserted,
         * 6 clocks per poll, counter in \c (sbiw pair) with \lo:\hi its
         * halves. Falls through 4 clocks after the detecting sbic, or branches
         * to \fail with go (port bit gb, DATA by default) withdrawn.
         */
        ".macro xsync c, lo, hi, grace, fail, pin, port, sb=1, gb=3\n"
        "5: sbiw \\c, 1\n"
        "  breq 8f\n"
        "  sbis \\pin, \\sb\n"
        "  rjmp 5b\n"
        "6: sbiw \\c, 1\n"
        "  breq 7f\n"
        "  sbic \\pin, \\sb\n"
        "  rjmp 6b\n"
        "  rjmp 9f\n"
        "7: cbi \\port, \\gb\n"
        "  ldi \\lo, lo8(\\grace)\n"
        "  ldi \\hi, hi8(\\grace)\n"
        "4: sbiw \\c, 1\n"
        "  breq \\fail\n"
        "  sbic \\pin, \\sb\n"
        "  rjmp 4b\n"
        "  rjmp 9f\n"
        "8: cbi \\port, \\gb\n"
        "  rjmp \\fail\n"
        "9:\n"
        ".endm\n");

#if IO_CLK_IN != _BV(1) || IO_DATA != _BV(3) || IO_CLK != _BV(0) ||            \
    IO_DATA_IN != _BV(2)
#error "X protocol assumes the ZoomFloppy port D pin assignment"
#endif

/*
 * One drive -> host byte at f clocks per drive cycle. Returns the four raw
 * PIND samples (P0 in the low byte), or X_NOSYNC (go is released in every
 * sample, so its bit is never all ones) if the drive never sent SYNC.
 */
#define X_RX(f)                                                                \
  static __attribute__((noinline)) uint32_t x_rx##f(uint16_t n) {              \
    uint32_t s;                                                                \
    __asm__ volatile(                                                          \
        "xsync %A[n], %A[n], %B[n], %[g], 2f, %[pin], %[port]\n"               \
        "  cbi %[port], 3\n"                                                   \
        "  xdelay %[d0]\n"                                                     \
        "  in %A[s], %[pin]\n"                                                 \
        "  xdelay %[d1]\n"                                                     \
        "  in %B[s], %[pin]\n"                                                 \
        "  xdelay %[d2]\n"                                                     \
        "  in %C[s], %[pin]\n"                                                 \
        "  xdelay %[d3]\n"                                                     \
        "  in %D[s], %[pin]\n"                                                 \
        "  rjmp 3f\n"                                                          \
        "2: clr %A[s]\n"                                                       \
        "  com %A[s]\n"                                                        \
        "  mov %B[s], %A[s]\n"                                                 \
        "  movw %C[s], %A[s]\n"                                                \
        "3:\n"                                                                 \
        : [s] "=&r"(s), [n] "+w"(n)                                            \
        : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),      \
          [g] "i"(X_GRACE * (f) / X_POLL),                                     \
          [d0] "i"(X_SAMPLE(XW0, XW1, f) - X_FOUND - 2),                       \
          [d1] "i"(X_SAMPLE(XW1, XW2, f) - X_SAMPLE(XW0, XW1, f) - 1),         \
          [d2] "i"(X_SAMPLE(XW2, XW3, f) - X_SAMPLE(XW1, XW2, f) - 1),         \
          [d3] "i"(X_SAMPLE(XW3, XW4, f) - X_SAMPLE(XW2, XW3, f) - 1)          \
        : "r23");                                                              \
    return s;                                                                  \
  }

/*
 * One host -> drive byte: p holds PORTD for P0..P3 and the final release.
 * Returns false if the drive never sent SYNC.
 */
#define X_TX(f)                                                                \
  static                                                                       \
      __attribute__((noinline)) bool x_tx##f(uint16_t n, const uint8_t *p) {   \
    uint8_t ok, v0, v1, v2, v3, v4;                                            \
    v0 = p[0];                                                                 \
    v1 = p[1];                                                                 \
    v2 = p[2];                                                                 \
    v3 = p[3];                                                                 \
    v4 = p[4];                                                                 \
    __asm__ volatile(                                                          \
        "ldi %[ok], 0\n"                                                       \
        "xsync %A[n], %A[n], %B[n], %[g], 2f, %[pin], %[port]\n"               \
        "  out %[port], %[v0]\n"                                               \
        "  xdelay %[d1]\n"                                                     \
        "  out %[port], %[v1]\n"                                               \
        "  xdelay %[d2]\n"                                                     \
        "  out %[port], %[v2]\n"                                               \
        "  xdelay %[d3]\n"                                                     \
        "  out %[port], %[v3]\n"                                               \
        "  xdelay %[d4]\n"                                                     \
        "  out %[port], %[v4]\n"                                               \
        "  ldi %[ok], 1\n"                                                     \
        "2:\n"                                                                 \
        : [ok] "=&d"(ok), [n] "+w"(n)                                          \
        : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),      \
          [g] "i"(X_GRACE * (f) / X_POLL), [v0] "r"(v0), [v1] "r"(v1),         \
          [v2] "r"(v2), [v3] "r"(v3), [v4] "r"(v4),                            \
          [d1] "i"(X_CHANGE(XR1, XR2, f) - X_FOUND - 1),                       \
          [d2] "i"(X_CHANGE(XR2, XR3, f) - X_CHANGE(XR1, XR2, f) - 1),         \
          [d3] "i"(X_CHANGE(XR3, XR4, f) - X_CHANGE(XR2, XR3, f) - 1),         \
          [d4] "i"((XR4 + 4) * (f) - X_CHANGE(XR3, XR4, f) - 1)                \
        : "r23");                                                              \
    return ok;                                                                 \
  }

X_RX(16)
X_RX(8)
X_TX(16)
X_TX(8)

#define X_NOSYNC 0xffffffffUL

/* Drive -> host pairs: (DATA, CLK) carry (b1,b3) (b5,b7) (b0,b2) (b4,b6). */
static uint8_t x_decode(uint32_t s) {
  static const uint8_t d[4] = {1, 5, 0, 4};
  uint8_t b = 0, i;

  for (i = 0; i < 4; i++, s >>= 8) {
    if (!(s & IO_DATA_IN))
      b |= 1 << d[i];
    if (!(s & IO_CLK_IN))
      b |= 4 << d[i];
  }
  return b;
}

/* Host -> drive pairs: (DATA, CLK) carry (b0,b2) (b1,b3) (b4,b6) (b5,b7). */
static void x_encode(uint8_t b, uint8_t *p) {
  static const uint8_t bits[4] = {0, 1, 4, 5};
  uint8_t base = PORTD & ~(IO_CLK | IO_DATA), i;

  for (i = 0; i < 4; i++)
    p[i] = base | ((b >> bits[i]) & 1 ? IO_DATA : 0) |
           ((b >> (bits[i] + 2)) & 1 ? IO_CLK : 0);
  p[4] = base;
}

/* Slice of SYNC polls between interrupt windows (about 24 ms). */
#define X_SLICE 0

uint16_t x_read_loop(uint16_t len, uint8_t flags, bool *ok) {
  uint16_t n = 0;
  uint32_t s;

  usbInitIo(len, ENDPOINT_DIR_IN);
  iec_release(IO_ATN | IO_CLK | IO_DATA);
  while (n < len && TimerWorker()) {
    if (!Endpoint_IsReadWriteAllowed())
      continue;
    cli();
    iec_set(IO_DATA);
    s = (flags & XUM_X_2MHZ) ? x_rx8(X_SLICE) : x_rx16(X_SLICE);
    sei();
    if (s == X_NOSYNC)
      continue;
    IoProgress();
    Endpoint_Write_Byte(x_decode(s));
    n++;
    if (!Endpoint_IsReadWriteAllowed())
      Endpoint_ClearIN();
  }
  iec_release(IO_DATA);
  Set_usbDataLen(len - n);
  usbIoDone();
  *ok = n == len;
  return n;
}

uint16_t x_write_loop(uint16_t len, uint8_t flags, bool *ok) {
  uint16_t n = 0;
  uint8_t data, p[5];
  bool sent;

  usbInitIo(len, ENDPOINT_DIR_OUT);
  iec_release(IO_ATN | IO_CLK | IO_DATA);
  while (n < len && usbRecvByte(&data) == 0) {
    x_encode(data, p);
    do {
      cli();
      iec_set(IO_DATA);
      sent = (flags & XUM_X_2MHZ) ? x_tx8(X_SLICE, p) : x_tx16(X_SLICE, p);
      sei();
    } while (!sent && TimerWorker());
    if (!sent)
      break;
    IoProgress();
    n++;
  }
  iec_release(IO_CLK | IO_DATA);
  usbIoDone();
  *ok = n == len;
  return n;
}

/*
 * Burst X (firmware v10): one go/SYNC per burst of up to XB_BURST bytes. The
 * drive (nybulah drive/proto_xb.inc) then runs a loop with a fixed cycle count
 * per byte, which these routines follow open-loop from the detecting poll.
 * Bursts split a transfer into XB_BURST-byte pieces from its start, both banks
 * of the double-banked endpoint each, switched at a fixed point of the loop;
 * the drive ends its bursts at XB_BURST-aligned addresses, so the host moves an
 * unaligned head in a transfer of its own.
 */
#define XB_BANK XUM_ENDPOINT_BULK_SIZE
#define XB_BURST (2 * XB_BANK)
#if XB_BURST != 64
#error "nybulah drive/proto_xb.inc assumes 64-byte bursts"
#endif

/* Drive -> host: writes of P0..P3, release after the last byte, period. */
#define XBW0 14
#define XBW1 26
#define XBW2 50
#define XBW3 62
#define XBW4 74
#define XBWN 67
/* Host -> drive: CLK release, reads of P0..P3, period. */
#define XBR0 6
#define XBR1 12
#define XBR2 20
#define XBR3 30
#define XBR4 38
#define XBRN 52

/* Clocks of xbbank, of the receive loop's decode and sts plus xbbank, and of
 * the send loop's dec, brne, lds and encode plus xbbank. */
#define XB_BANKSW 9
#define XB_DECODE (19 + XB_BANKSW)
#define XB_ENCODE (25 + XB_BANKSW)

__asm__(
    ".macro xbenc p, b, base, d, c\n"
    "  mov \\p, \\base\n"
    "  bst \\b, \\d\n"
    "  bld \\p, 3\n"
    "  bst \\b, \\c\n"
    "  bld \\p, 0\n"
    ".endm\n"
    /* Count down c; at zero clear flags in UEINTX (at ue) to switch banks. */
    ".macro xbbank c, t, flags, ue\n"
    "  dec \\c\n"
    "  brne 11f\n"
    "  lds \\t, \\ue\n"
    "  andi \\t, ~(\\flags) & 0xff\n"
    "  sts \\ue, \\t\n"
    "  rjmp 12f\n"
    "11: rjmp .+0\n"
    "  rjmp .+0\n"
    "  rjmp .+0\n"
    "12:\n"
    ".endm\n");

/*
 * Drive -> host burst of k bytes at f clocks per drive cycle, stored into the
 * IN endpoint as they arrive; the bank is sent after c bytes if c is not 0.
 * Pairs (DATA, CLK) carry (b1,b3) (b5,b7) (b0,b2) (b4,b6). Returns false if
 * the drive never sent SYNC.
 */
#define XB_RX(f)                                                               \
  static __attribute__((noinline)) bool xb_rx##f(uint16_t n, uint8_t k,        \
                                                 uint8_t c) {                  \
    uint8_t ok, s0, s1, s2, s3, b, t;                                          \
    __asm__ volatile(                                                          \
        "ldi %[ok], 0\n"                                                       \
        "xsync %A[n], %A[n], %B[n], %[g], 2f, %[pin], %[port]\n"               \
        "  cbi %[port], 3\n"                                                   \
        "  xdelay %[d0]\n"                                                     \
        "10: in %[s0], %[pin]\n"                                               \
        "  xdelay %[d1]\n"                                                     \
        "  in %[s1], %[pin]\n"                                                 \
        "  xdelay %[d2]\n"                                                     \
        "  in %[s2], %[pin]\n"                                                 \
        "  xdelay %[d3]\n"                                                     \
        "  in %[s3], %[pin]\n"                                                 \
        "  bst %[s0], 2\n  bld %[b], 1\n  bst %[s0], 1\n  bld %[b], 3\n"       \
        "  bst %[s1], 2\n  bld %[b], 5\n  bst %[s1], 1\n  bld %[b], 7\n"       \
        "  bst %[s2], 2\n  bld %[b], 0\n  bst %[s2], 1\n  bld %[b], 2\n"       \
        "  bst %[s3], 2\n  bld %[b], 4\n  bst %[s3], 1\n  bld %[b], 6\n"       \
        "  com %[b]\n"                                                         \
        "  sts %[fifo], %[b]\n"                                                \
        "  xbbank %[c], %[t], %[in], %[ue]\n"                                  \
        "  xdelay %[dl]\n"                                                     \
        "  dec %[k]\n"                                                         \
        "  brne 10b\n"                                                         \
        "  ldi %[ok], 1\n"                                                     \
        "2:\n"                                                                 \
        : [ok] "=&d"(ok), [n] "+w"(n), [k] "+r"(k), [c] "+r"(c),               \
          [s0] "=&r"(s0), [s1] "=&r"(s1), [s2] "=&r"(s2), [s3] "=&r"(s3),      \
          [b] "=&r"(b), [t] "=&d"(t)                                           \
        : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),      \
          [fifo] "n"(_SFR_MEM_ADDR(UEDATX)), [ue] "n"(_SFR_MEM_ADDR(UEINTX)),  \
          [in] "n"(_BV(TXINI) | _BV(FIFOCON)),                                 \
          [g] "i"(X_GRACE * (f) / X_POLL),                                     \
          [d0] "i"(X_SAMPLE(XBW0, XBW1, f) - X_FOUND - 2),                     \
          [d1] "i"(X_SAMPLE(XBW1, XBW2, f) - X_SAMPLE(XBW0, XBW1, f) - 1),     \
          [d2] "i"(X_SAMPLE(XBW2, XBW3, f) - X_SAMPLE(XBW1, XBW2, f) - 1),     \
          [d3] "i"(X_SAMPLE(XBW3, XBW4, f) - X_SAMPLE(XBW2, XBW3, f) - 1),     \
          [dl] "i"(XBWN * (f) - X_SAMPLE(XBW3, XBW4, f) +                      \
                   X_SAMPLE(XBW0, XBW1, f) - 1 - XB_DECODE - 1 - 2)            \
        : "r23", "memory");                                                    \
    return ok;                                                                 \
  }

/*
 * Host -> drive burst of k bytes: b0 is the first byte, the rest are read from
 * the OUT endpoint between bytes, switching banks after c more if c is not 0;
 * base is PORTD with CLK and DATA released. Pairs (DATA, CLK) carry (b5,b7)
 * (b4,b6) (b1,b3) (b0,b2); P0 of the first byte goes out at detection.
 * Returns false if the drive never sent SYNC.
 */
#define XB_TX(f)                                                               \
  static __attribute__((noinline)) bool xb_tx##f(                              \
      uint16_t n, uint8_t k, uint8_t c, uint8_t b0, uint8_t base) {            \
    uint8_t ok, p0, p1, p2, p3, t;                                             \
    __asm__ volatile(                                                          \
        "ldi %[ok], 0\n"                                                       \
        "xbenc %[p0], %[b], %[base], 5, 7\n"                                   \
        "xbenc %[p1], %[b], %[base], 4, 6\n"                                   \
        "xbenc %[p2], %[b], %[base], 1, 3\n"                                   \
        "xbenc %[p3], %[b], %[base], 0, 2\n"                                   \
        "rjmp 15f\n"                                                           \
        "14: rjmp 2f\n"                                                        \
        "15: xsync %A[n], %A[n], %B[n], %[g], 14b, %[pin], %[port]\n"          \
        "  out %[port], %[p0]\n"                                               \
        "  xdelay %[e1]\n"                                                     \
        "10: out %[port], %[p1]\n"                                             \
        "  xdelay %[d2]\n"                                                     \
        "  out %[port], %[p2]\n"                                               \
        "  xdelay %[d3]\n"                                                     \
        "  out %[port], %[p3]\n"                                               \
        "  dec %[k]\n"                                                         \
        "  brne 13f\n"                                                         \
        "  rjmp 3f\n"                                                          \
        "13: lds %[b], %[fifo]\n"                                              \
        "  xbbank %[c], %[t], %[out], %[ue]\n"                                 \
        "  xbenc %[p0], %[b], %[base], 5, 7\n"                                 \
        "  xbenc %[p1], %[b], %[base], 4, 6\n"                                 \
        "  xbenc %[p2], %[b], %[base], 1, 3\n"                                 \
        "  xbenc %[p3], %[b], %[base], 0, 2\n"                                 \
        "  xdelay %[da]\n"                                                     \
        "  out %[port], %[p0]\n"                                               \
        "  xdelay %[db]\n"                                                     \
        "  rjmp 10b\n"                                                         \
        "3: xdelay %[dr]\n"                                                    \
        "  out %[port], %[base]\n"                                             \
        "  ldi %[ok], 1\n"                                                     \
        "2:\n"                                                                 \
        : [ok] "=&d"(ok), [n] "+w"(n), [k] "+r"(k), [c] "+r"(c), [b] "+r"(b0), \
          [p0] "=&r"(p0), [p1] "=&r"(p1), [p2] "=&r"(p2), [p3] "=&r"(p3),      \
          [t] "=&d"(t)                                                         \
        : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),      \
          [fifo] "n"(_SFR_MEM_ADDR(UEDATX)), [ue] "n"(_SFR_MEM_ADDR(UEINTX)),  \
          [out] "n"(_BV(RXOUTI) | _BV(FIFOCON)), [base] "r"(base),             \
          [g] "i"(X_GRACE * (f) / X_POLL),                                     \
          [e1] "i"(X_CHANGE(XBR1, XBR2, f) - X_FOUND - 1),                     \
          [d2] "i"(X_CHANGE(XBR2, XBR3, f) - X_CHANGE(XBR1, XBR2, f) - 1),     \
          [d3] "i"(X_CHANGE(XBR3, XBR4, f) - X_CHANGE(XBR2, XBR3, f) - 1),     \
          [da] "i"(X_CHANGE(XBR4, XBR1 + XBRN, f) - X_CHANGE(XBR3, XBR4, f) -  \
                   1 - XB_ENCODE),                                             \
          [db] "i"(X_CHANGE(XBR1, XBR2, f) + XBRN * (f) -                      \
                   X_CHANGE(XBR4, XBR1 + XBRN, f) - 1 - 2),                    \
          [dr] "i"((XBR4 + 4) * (f) - X_CHANGE(XBR3, XBR4, f) - 1 - 1 - 1 - 2) \
        : "r23", "memory");                                                    \
    return ok;                                                                 \
  }

XB_RX(16)
XB_RX(8)
XB_TX(16)
XB_TX(8)

/* Busy banks of the selected endpoint, a count (0..2). */
#define XB_BUSY() (UESTA0X & (_BV(NBUSYBK0) | _BV(NBUSYBK1)))

uint16_t xb_read_loop(uint16_t len, uint8_t flags, bool *ok) {
  uint16_t n = 0;
  uint8_t k, c;
  bool got;

  usbInitIo(len, ENDPOINT_DIR_IN);
  iec_release(IO_ATN | IO_CLK | IO_DATA);
  while (n < len && TimerWorker()) {
    if (!Endpoint_IsReadWriteAllowed() || !xb_in_ready(XB_BUSY()))
      continue;
    k = len - n > XB_BURST ? XB_BURST : len - n;
    c = k > XB_BANK ? XB_BANK : 0;
    cli();
    iec_set(IO_DATA);
    got = (flags & XUM_X_2MHZ) ? xb_rx8(X_SLICE, k, c) : xb_rx16(X_SLICE, k, c);
    sei();
    if (!got)
      continue;
    IoProgress();
    n += k;
    if (Endpoint_BytesInEndpoint() != 0)
      Endpoint_ClearIN();
  }
  iec_release(IO_DATA);
  Set_usbDataLen(len - n);
  usbIoDone();
  *ok = n == len;
  return n;
}

uint16_t xb_write_loop(uint16_t len, uint8_t flags, bool *ok) {
  uint16_t n = 0, used = 0;
  uint8_t k, c, b0, base;
  bool sent = false;

  usbInitIo(len, ENDPOINT_DIR_OUT);
  iec_release(IO_ATN | IO_CLK | IO_DATA);
  base = PORTD & ~(IO_CLK | IO_DATA);
  while (n < len && TimerWorker()) {
    k = len - n > XB_BURST ? XB_BURST : len - n;
    if (!Endpoint_IsReadWriteAllowed()) {
      Endpoint_ClearOUT();
      while (!Endpoint_IsReadWriteAllowed() && TimerWorker())
        ;
      continue;
    }
    if (!xb_out_ready(k, Endpoint_BytesInEndpoint(), XB_BUSY(), XB_BANK))
      continue;
    c = k > XB_BANK ? XB_BANK - 1 : 0;
    b0 = Endpoint_Read_Byte();
    used++;
    do {
      cli();
      iec_set(IO_DATA);
      sent = (flags & XUM_X_2MHZ) ? xb_tx8(X_SLICE, k, c, b0, base)
                                  : xb_tx16(X_SLICE, k, c, b0, base);
      sei();
    } while (!sent && TimerWorker());
    if (!sent)
      break;
    IoProgress();
    n += k;
    used = n;
  }
  iec_release(IO_CLK | IO_DATA);
  Set_usbDataLen(len - used);
  usbIoDone();
  *ok = n == len;
  return n;
}

/*
 * SRQ fast serial (firmware v11, 1571 only): the drive's 6526 shifts bytes MSB
 * first on DATA, clocked by SRQ (its CNT), both through the bus drivers that
 * VIA1 PA1 turns (nybulah drive/proto_srq.inc, docs/protocol.md). With timer
 * A at latch 1 CNT falls with each new bit on DATA and rises SRQ_U cycles
 * later. Drive -> host: go on CLK, then the adapter times each byte from its
 * first SRQ fall and samples DATA in mid-window; it then polls for the next
 * byte's fall between the last rise settling and the drive's earliest next
 * byte. Host -> drive: go on DATA and SYNC on CLK as burst X, then the
 * adapter clocks SRQ with DATA valid on each rise, at a bit period giving the
 * DATA set-up (SRQ low >= X_RISE) and hold (SRQ high >= X_RISE plus a drive
 * cycle of sampling) the slack of the tightest 2 MHz read window, SRQ_SIGMA,
 * and a byte period no shorter than the drive's receive loop.
 */
#if IO_SRQ != _BV(4) || IO_SRQ_IN != _BV(5)
#error "SRQ protocol assumes the ZoomFloppy SRQ pins"
#endif

/* srq_rx results besides the bytes still missing: no first fall at all. */
#define SRQ_NOSYNC 0xff

__asm__(".macro srqin s, b, j, d, pin\n"
        "  in \\s, \\pin\n"
        "  bst \\s, 2\n"
        "  bld \\b, \\j\n"
        "  xdelay \\d\n"
        ".endm\n"
        /* Bit j of ~b on DATA with SRQ asserted, SRQ released after lo + 2
         * clocks, next bit's port write hi + 4 clocks later. */
        ".macro srqbit p, bs, b, j, lo, hi, port\n"
        "  mov \\p, \\bs\n"
        "  bst \\b, \\j\n"
        "  bld \\p, 3\n"
        "  out \\port, \\p\n"
        "  xdelay \\lo\n"
        "  andi \\p, 0xef\n"
        "  out \\port, \\p\n"
        "  xdelay \\hi\n"
        ".endm\n");

/*
 * Drive -> host SRQ burst of k bytes at f clocks per drive cycle into the IN
 * endpoint, its bank sent after c bytes if c is not 0. Returns the bytes not
 * received (0 when complete) or SRQ_NOSYNC if the first fall never came.
 */
#define SRQ_RX(f)                                                              \
  static __attribute__((noinline)) uint8_t srq_rx##f(uint16_t n, uint8_t k,    \
                                                     uint8_t c) {              \
    uint8_t s, b, t, w;                                                        \
    __asm__ volatile(                                                          \
        "rjmp 15f\n"                                                           \
        "14: rjmp 2f\n"                                                        \
        "15: xsync %A[n], %A[n], %B[n], %[g], 14b, %[pin], %[port], 5, 0\n"    \
        "21: cbi %[port], 0\n"                                                 \
        "  xdelay %[d0]\n"                                                     \
        "  srqin %[s], %[b], 7, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 6, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 5, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 4, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 3, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 2, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 1, %[dj], %[pin]\n"                               \
        "  srqin %[s], %[b], 0, 0, %[pin]\n"                                   \
        "  sts %[fifo], %[b]\n"                                                \
        "  xbbank %[c], %[t], %[in], %[ue]\n"                                  \
        "  dec %[k]\n"                                                         \
        "  breq 3f\n"                                                          \
        "  xdelay %[dp]\n"                                                     \
        "  ldi %[w], %[wn]\n"                                                  \
        "22: dec %[w]\n"                                                       \
        "  breq 3f\n"                                                          \
        "  sbis %[pin], 5\n"                                                   \
        "  rjmp 22b\n"                                                         \
        "23: dec %[w]\n"                                                       \
        "  breq 3f\n"                                                          \
        "  sbic %[pin], 5\n"                                                   \
        "  rjmp 23b\n"                                                         \
        "  rjmp 21b\n"                                                         \
        "2: ldi %[k], %[nosync]\n"                                             \
        "3:\n"                                                                 \
        : [n] "+w"(n), [k] "+d"(k), [c] "+r"(c), [s] "=&r"(s), [b] "=&r"(b),   \
          [t] "=&d"(t), [w] "=&d"(w)                                           \
        : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),      \
          [fifo] "n"(_SFR_MEM_ADDR(UEDATX)), [ue] "n"(_SFR_MEM_ADDR(UEINTX)),  \
          [in] "n"(_BV(TXINI) | _BV(FIFOCON)), [nosync] "n"(SRQ_NOSYNC),       \
          [g] "i"(X_GRACE * (f) / X_POLL), [wn] "n"(SRQ_POLLS(f)),             \
          [d0] "i"(SRQ_SAMPLE(0, f) - X_FOUND - 2),                            \
          [dj] "i"(SRQ_SAMPLE(1, f) - SRQ_SAMPLE(0, f) - 3),                   \
          [dp] "i"(SRQ_START(f) - SRQ_SAMPLE(7, f) - 3 - 2 - XB_BANKSW - 2 -   \
                   1 - 2)                                                      \
        : "r23", "memory");                                                    \
    return k;                                                                  \
  }

/*
 * Host -> drive SRQ burst of k bytes: b0 is the first byte, the rest come from
 * the OUT endpoint, switching banks after c more if c is not 0; base is PORTD
 * with SRQ, CLK and DATA released. Returns false if the drive never sent SYNC.
 */
#define SRQ_TX(f)                                                              \
  static __attribute__((noinline)) bool srq_tx##f(                             \
      uint16_t n, uint8_t k, uint8_t c, uint8_t b0, uint8_t base) {            \
    uint8_t ok, p, t, bs = base | IO_SRQ;                                      \
    __asm__ volatile(                                                          \
        "ldi %[ok], 0\n"                                                       \
        "com %[b]\n"                                                           \
        "rjmp 15f\n"                                                           \
        "14: rjmp 2f\n"                                                        \
        "15: xsync %A[n], %A[n], %B[n], %[g], 14b, %[pin], %[port]\n"          \
        "20: srqbit %[p], %[bs], %[b], 7, %[lo], %[hi], %[port]\n"             \
        "  srqbit %[p], %[bs], %[b], 6, %[lo], %[hi], %[port]\n"               \
        "  srqbit %[p], %[bs], %[b], 5, %[lo], %[hi], %[port]\n"               \
        "  srqbit %[p], %[bs], %[b], 4, %[lo], %[hi], %[port]\n"               \
        "  srqbit %[p], %[bs], %[b], 3, %[lo], %[hi], %[port]\n"               \
        "  srqbit %[p], %[bs], %[b], 2, %[lo], %[hi], %[port]\n"               \
        "  srqbit %[p], %[bs], %[b], 1, %[lo], %[hi], %[port]\n"               \
        "  srqbit %[p], %[bs], %[b], 0, %[lo], 0, %[port]\n"                   \
        "  dec %[k]\n"                                                         \
        "  breq 3f\n"                                                          \
        "  lds %[b], %[fifo]\n"                                                \
        "  com %[b]\n"                                                         \
        "  xbbank %[c], %[t], %[out], %[ue]\n"                                 \
        "  xdelay %[dn]\n"                                                     \
        "  rjmp 20b\n"                                                         \
        "3: xdelay %[dr]\n"                                                    \
        "  out %[port], %[base]\n"                                             \
        "  ldi %[ok], 1\n"                                                     \
        "2:\n"                                                                 \
        : [ok] "=&d"(ok), [n] "+w"(n), [k] "+r"(k), [c] "+r"(c), [b] "+r"(b0), \
          [p] "=&d"(p), [t] "=&d"(t)                                           \
        : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),      \
          [fifo] "n"(_SFR_MEM_ADDR(UEDATX)), [ue] "n"(_SFR_MEM_ADDR(UEINTX)),  \
          [out] "n"(_BV(RXOUTI) | _BV(FIFOCON)), [base] "r"(base),             \
          [bs] "r"(bs), [g] "i"(X_GRACE * (f) / X_POLL),                       \
          [lo] "i"(SRQ_LOW(f) - 2), [hi] "i"(SRQ_HIGH(f) - 4),                 \
          [dn] "i"(SRQ_HIGH(f) - 1 - 1 - 1 - 2 - 1 - XB_BANKSW - 2 -           \
                   SRQ_ENCODE),                                                \
          [dr] "i"(SRQ_HIGH(f) - 1 - 2 - 1)                                    \
        : "r23", "memory");                                                    \
    return ok;                                                                 \
  }

SRQ_RX(16)
SRQ_RX(8)
SRQ_TX(16)
SRQ_TX(8)

/*
 * SRQ streaming receive (firmware v12, 1571 at 2 MHz, nybulah drive/stream.s):
 * after go and the first fall (as srq_rx) the drive sends bytes unprompted,
 * each timed from its own first SRQ fall and sampled as srq_rx samples them;
 * CLK asserted at bit 7's sample marks metadata. Each byte goes into the IN
 * endpoint at once (XUM_X_STREAM in xum1541_types.h gives the framing); a
 * bank is handed to the host when full, and a bank still waiting for the host
 * when the next byte needs it is an overrun. After the byte, a poll at
 * SRQ_FRAME or later must see SRQ released (else the next byte started too
 * early: framing), and the wait for the next fall starts by SRQ_WAIT. Returns
 * an XUM_STREAM_* code (bit 0: an ESC was left without its second byte) in
 * the low byte and the bytes still free in the current bank in the high byte,
 * or SRQ_NOSYNC if the first fall never came; *left counts the banks still
 * allowed.
 */
__asm__(
    /* Byte r into the IN endpoint (c free bytes in the bank, t scratch): a
     * fresh bank must be free (else ovr), a full one goes to the host and
     * counts down left (a pointer pair) banks, ending at end on 0. */
    ".macro sput r, c, t, left, ue, fifo, flags, ovr, end\n"
    "  cpi \\c, 32\n"
    "  brne 1f\n"
    "  lds \\t, \\ue\n"
    "  sbrs \\t, 5\n" /* RWAL */
    "  rjmp \\ovr\n"
    "1: sts \\fifo, \\r\n"
    "  dec \\c\n"
    "  brne 2f\n"
    "  lds \\t, \\ue\n"
    "  andi \\t, ~(\\flags) & 0xff\n"
    "  sts \\ue, \\t\n"
    "  ldi \\c, 32\n"
    "  sbiw \\left, 1\n"
    "  brne 2f\n"
    "  rjmp \\end\n"
    "2:\n"
    ".endm\n");

static __attribute__((noinline)) uint16_t srq_stream8(uint16_t n, uint8_t c,
                                                      uint16_t *left) {
  uint8_t s, s0, b, t, code;
  uint16_t l = *left;
  __asm__ volatile(
      "rjmp 15f\n"
      "14: rjmp 2f\n"
      "15: xsync %A[n], %A[n], %B[n], %[g], 14b, %[pin], %[port], 5, 0\n"
      "21: cbi %[port], 0\n"
      "  xdelay %[d0]\n"
      "  srqin %[s0], %[b], 7, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 6, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 5, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 4, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 3, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 2, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 1, %[dj], %[pin]\n"
      "  srqin %[s], %[b], 0, 0, %[pin]\n"
      "  wdr\n"
      "  sbrs %[s0], 1\n" /* CLK asserted: metadata */
      "  rjmp 30f\n"
      "  tst %[b]\n"
      "  brne 31f\n"
      "  sput __zero_reg__, %[c], %[t], %[l], %[ue], %[fifo], %[in], 40f, 46f\n"
      "  sput __zero_reg__, %[c], %[t], %[l], %[ue], %[fifo], %[in], 41f, 45f\n"
      "  rjmp 32f\n"
      "31: sput %[b], %[c], %[t], %[l], %[ue], %[fifo], %[in], 40f, 45f\n"
      "  rjmp 32f\n"
      "30: sput __zero_reg__, %[c], %[t], %[l], %[ue], %[fifo], %[in], 40f, "
      "46f\n"
      "  sput %[b], %[c], %[t], %[l], %[ue], %[fifo], %[in], 41f, 45f\n"
      "  mov %[t], %[b]\n"
      "  andi %[t], 0xf3\n"
      "  cpi %[t], 0x40\n" /* END metadata */
      "  breq 43f\n"
      "32: sbis %[pin], 5\n" /* SRQ_FRAME or later: released */
      "  rjmp 42f\n"
      "  ldi %A[n], lo8(%[wn])\n"
      "  ldi %B[n], hi8(%[wn])\n"
      "22: sbiw %A[n], 1\n" /* SRQ_WAIT or earlier */
      "  breq 44f\n"
      "  sbic %[pin], 5\n"
      "  rjmp 22b\n"
      "  rjmp 21b\n"
      "40: ldi %[code], %[ovr]\n"
      "  rjmp 3f\n"
      "41: ldi %[code], %[ovr] | 1\n"
      "  rjmp 3f\n"
      "42: ldi %[code], %[frm]\n"
      "  rjmp 3f\n"
      "43: ldi %[code], %[done]\n"
      "  rjmp 3f\n"
      "44: ldi %[code], %[tmo]\n"
      "  rjmp 3f\n"
      "45: ldi %[code], %[trn]\n"
      "  rjmp 3f\n"
      "46: ldi %[code], %[trn] | 1\n"
      "  rjmp 3f\n"
      "2: ldi %[code], %[nosync]\n"
      "3:\n"
      : [n] "+w"(n), [c] "+d"(c), [l] "+e"(l), [s] "=&r"(s), [s0] "=&r"(s0),
        [b] "=&r"(b), [t] "=&d"(t), [code] "=&d"(code)
      : [pin] "I"(_SFR_IO_ADDR(PIND)), [port] "I"(_SFR_IO_ADDR(PORTD)),
        [fifo] "n"(_SFR_MEM_ADDR(UEDATX)), [ue] "n"(_SFR_MEM_ADDR(UEINTX)),
        [in] "n"(_BV(TXINI) | _BV(FIFOCON)), [g] "i"(X_GRACE * 8 / X_POLL),
        [wn] "i"(SRQ_STREAM_POLLS), [nosync] "n"(SRQ_NOSYNC),
        [ovr] "n"(XUM_STREAM_OVERRUN), [frm] "n"(XUM_STREAM_FRAMING),
        [done] "n"(XUM_STREAM_DONE), [tmo] "n"(XUM_STREAM_TIMEOUT),
        [trn] "n"(XUM_STREAM_TRUNCATED),
        [d0] "i"(SRQ_SAMPLE(0, 8) - X_FOUND - 2),
        [dj] "i"(SRQ_SAMPLE(1, 8) - SRQ_SAMPLE(0, 8) - 3)
      : "r23", "memory");
  *left = l;
  return code | (uint16_t)c << 8;
}

/* Drive gone quiet after ATN: SRQ released this many polls in a row. */
#define STREAM_QUIET 1000
/* Most polls (1 us apart or more) of ATN held. */
#define STREAM_ATN 50000U

/* ATN stops the drive's stream (drive/stream.s checks it); release it once SRQ
 * has stayed released STREAM_QUIET polls, or after STREAM_ATN. */
static void stream_stop(void) {
  uint16_t quiet = 0, n;

  iec_set(IO_ATN);
  for (n = 0; quiet < STREAM_QUIET && n < STREAM_ATN; n++) {
    wdt_reset();
    DELAY_US(1);
    quiet = iec_get(IO_SRQ) ? 0 : quiet + 1;
  }
  iec_release(IO_ATN);
}

/* One more output byte once the current bank is writable. */
static bool stream_put(uint8_t b) {
  while (!Endpoint_IsReadWriteAllowed())
    if (!TimerWorker())
      return false;
  Endpoint_Write_Byte(b);
  if (Endpoint_BytesInEndpoint() == XB_BANK)
    Endpoint_ClearIN();
  return true;
}

/*
 * Streaming receive (2 MHz only) of at most units * XUM_STREAM_UNIT output
 * bytes, the last bank kept for the trailer: go and the first fall as
 * srq_read_loop, then srq_stream8 with interrupts masked; a stop other than
 * the drive's END holds ATN to end the drive's stream. Then the trailer (ESC
 * and the code, or the code alone after a dangling ESC) and a short packet.
 * No status block follows.
 */
void srq_stream_loop(uint16_t units, uint8_t flags) {
  uint32_t banks = (uint32_t)units * (XUM_STREAM_UNIT / XB_BANK) - 1;
  uint16_t left = banks > 0xffff ? 0xffff : banks, r = SRQ_NOSYNC;
  uint8_t code;

  usbInitIo(0, ENDPOINT_DIR_IN);
  iec_release(IO_ATN | IO_CLK | IO_DATA | IO_SRQ);
  if (units == 0 || !(flags & XUM_X_2MHZ))
    left = 0;
  while (left != 0 && (uint8_t)r == SRQ_NOSYNC && TimerWorker()) {
    if (!Endpoint_IsReadWriteAllowed())
      continue;
    cli();
    iec_set(IO_CLK);
    r = srq_stream8(X_SLICE, XB_BANK - Endpoint_BytesInEndpoint(), &left);
    sei();
  }
  iec_release(IO_CLK);
  code = (uint8_t)r == SRQ_NOSYNC ? XUM_STREAM_TIMEOUT : (uint8_t)r;
  if ((code & ~1) != XUM_STREAM_DONE)
    stream_stop();
  if ((code & 1) || stream_put(XUM_STREAM_ESC))
    stream_put(code & ~1);
  if (Endpoint_BytesInEndpoint() != 0)
    Endpoint_ClearIN();
  else {
    while (!Endpoint_IsReadWriteAllowed() && TimerWorker())
      ;
    Endpoint_ClearIN();
  }
  Set_usbDataLen(0);
  usbIoDone();
}

uint16_t srq_read_loop(uint16_t len, uint8_t flags, bool *ok) {
  uint16_t n = 0;
  uint8_t k, c, r = 0;

  usbInitIo(len, ENDPOINT_DIR_IN);
  iec_release(IO_ATN | IO_CLK | IO_DATA | IO_SRQ);
  while (n < len && r == 0 && TimerWorker()) {
    if (!Endpoint_IsReadWriteAllowed() || !xb_in_ready(XB_BUSY()))
      continue;
    k = len - n > XB_BURST ? XB_BURST : len - n;
    c = k > XB_BANK ? XB_BANK : 0;
    cli();
    iec_set(IO_CLK);
    r = (flags & XUM_X_2MHZ) ? srq_rx8(X_SLICE, k, c) : srq_rx16(X_SLICE, k, c);
    sei();
    if (r == SRQ_NOSYNC) {
      r = 0;
      continue;
    }
    IoProgress();
    n += k - r;
    if (Endpoint_BytesInEndpoint() != 0)
      Endpoint_ClearIN();
  }
  iec_release(IO_CLK);
  Set_usbDataLen(len - n);
  usbIoDone();
  *ok = n == len;
  return n;
}

uint16_t srq_write_loop(uint16_t len, uint8_t flags, bool *ok) {
  uint16_t n = 0, used = 0;
  uint8_t k, c, b0, base;
  bool sent = false;

  usbInitIo(len, ENDPOINT_DIR_OUT);
  iec_release(IO_ATN | IO_CLK | IO_DATA | IO_SRQ);
  base = PORTD & ~(IO_CLK | IO_DATA | IO_SRQ);
  while (n < len && TimerWorker()) {
    k = len - n > XB_BURST ? XB_BURST : len - n;
    if (!Endpoint_IsReadWriteAllowed()) {
      Endpoint_ClearOUT();
      while (!Endpoint_IsReadWriteAllowed() && TimerWorker())
        ;
      continue;
    }
    if (!xb_out_ready(k, Endpoint_BytesInEndpoint(), XB_BUSY(), XB_BANK))
      continue;
    c = k > XB_BANK ? XB_BANK - 1 : 0;
    b0 = Endpoint_Read_Byte();
    used++;
    do {
      cli();
      iec_set(IO_DATA);
      sent = (flags & XUM_X_2MHZ) ? srq_tx8(X_SLICE, k, c, b0, base)
                                  : srq_tx16(X_SLICE, k, c, b0, base);
      sei();
    } while (!sent && TimerWorker());
    if (!sent)
      break;
    IoProgress();
    n += k;
    used = n;
  }
  iec_release(IO_SRQ | IO_CLK | IO_DATA);
  Set_usbDataLen(len - used);
  usbIoDone();
  *ok = n == len;
  return n;
}

#else

uint16_t x_read_loop(uint16_t len, uint8_t flags, bool *ok) {
  (void)flags;
  usbInitIo(len, ENDPOINT_DIR_IN);
  usbIoDone();
  *ok = false;
  return 0;
}

uint16_t x_write_loop(uint16_t len, uint8_t flags, bool *ok) {
  (void)flags;
  usbInitIo(len, ENDPOINT_DIR_OUT);
  usbIoDone();
  *ok = false;
  return 0;
}

uint16_t xb_read_loop(uint16_t len, uint8_t flags, bool *ok) {
  return x_read_loop(len, flags, ok);
}

uint16_t xb_write_loop(uint16_t len, uint8_t flags, bool *ok) {
  return x_write_loop(len, flags, ok);
}

uint16_t srq_read_loop(uint16_t len, uint8_t flags, bool *ok) {
  return x_read_loop(len, flags, ok);
}

uint16_t srq_write_loop(uint16_t len, uint8_t flags, bool *ok) {
  return x_write_loop(len, flags, ok);
}

void srq_stream_loop(uint16_t units, uint8_t flags) {
  usbInitIo(0, ENDPOINT_DIR_IN);
  (void)units;
  (void)flags;
  usbIoDone();
}

#endif
