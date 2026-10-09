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
#include "xum1541.h"

#ifdef X_SUPPORT

#if F_CPU != 16000000
#error "X protocol offsets assume a 16 MHz adapter clock"
#endif

/* Adapter clocks: release settling budget, SYNC poll loop, synchroniser. */
#define X_RISE 16
#define X_POLL 6
#define X_SYNC 1
/* Drive cycles watched after withdrawing go; the drive needs <= 18. */
#define X_GRACE 32

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

/* Centre of window [a, b) (drive cycles, f clocks each), minus poll jitter. */
#define X_SAMPLE(a, b, f) ((((a) + (b)) * (f) + X_RISE - X_POLL) / 2)
/* Change between drive reads a and b, settled before b, after a. */
#define X_CHANGE(a, b, f) ((((a) + (b)) * (f) - X_POLL - X_RISE) / 2 - X_SYNC)

/* Clocks from the detecting sbic to the first instruction after it. */
#define X_FOUND 4

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
         * Wait for CLK released then asserted, 6 clocks per poll, counter in
         * \c (sbiw pair) with \lo:\hi its halves. Falls through 4 clocks after
         * the detecting sbic, or branches to \fail with go withdrawn.
         */
        ".macro xsync c, lo, hi, grace, fail, pin, port\n"
        "5: sbiw \\c, 1\n"
        "  breq 8f\n"
        "  sbis \\pin, 1\n"
        "  rjmp 5b\n"
        "6: sbiw \\c, 1\n"
        "  breq 7f\n"
        "  sbic \\pin, 1\n"
        "  rjmp 6b\n"
        "  rjmp 9f\n"
        "7: cbi \\port, 3\n"
        "  ldi \\lo, lo8(\\grace)\n"
        "  ldi \\hi, hi8(\\grace)\n"
        "4: sbiw \\c, 1\n"
        "  breq \\fail\n"
        "  sbic \\pin, 1\n"
        "  rjmp 4b\n"
        "  rjmp 9f\n"
        "8: cbi \\port, 3\n"
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

#endif
