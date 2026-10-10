/*
 * X and SRQ protocol timing budgets and the schedules derived from them,
 * shared by x.c and misc/srq_timing_test.c
 * Copyright (c) 2026 Josh Bailey
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Adapter clocks are 1/16 us; f is adapter clocks per drive cycle.
 */
#ifndef X_TIMING_H
#define X_TIMING_H

#include <stdint.h>

/* Adapter clocks: release settling budget, SYNC poll loop, synchroniser. */
#define X_RISE 16
#define X_POLL 6
#define X_SYNC 1
/* Drive cycles watched after withdrawing go; the drive needs <= 18. */
#define X_GRACE 32

/* Centre of window [a, b) (drive cycles, f clocks each), minus poll jitter. */
#define X_SAMPLE(a, b, f) ((((a) + (b)) * (f) + X_RISE - X_POLL) / 2)
/* Change between drive reads a and b, settled before b, after a. */
#define X_CHANGE(a, b, f) ((((a) + (b)) * (f) - X_POLL - X_RISE) / 2 - X_SYNC)

/* Clocks from the detecting sbic to the first instruction after it. */
#define X_FOUND 4

/* Drive cycles: CNT phase, a byte's first fall to last rise, the drive's
 * earliest and latest next fall after that, its receive loop. */
#define SRQ_U 2
#define SRQ_LAST (15 * SRQ_U)
#define SRQ_GMIN 7
#define SRQ_GMAX 14
#define SRQ_RLOOP 39
/* Adapter clocks: in-burst SRQ poll loop, building a bit's port value. */
#define SRQ_POLL 5
#define SRQ_ENCODE 3

/* Slack each side of the 2 MHz read windows: the write design slack. */
#define SRQ_SIGMA ((2 * SRQ_U * 8 - X_RISE - X_POLL) / 2)
/* Read: sample of bit 7 - j from the detected first fall. */
#define SRQ_SAMPLE(j, f) X_SAMPLE(2 * (j) * SRQ_U, (2 * (j) + 2) * SRQ_U, f)
/* Read: first poll for the next byte, between the last rise and its fall. */
#define SRQ_START(f) X_SAMPLE(SRQ_LAST, SRQ_LAST + SRQ_GMIN, f)
/* Read: polls (both loops) up to one byte beyond the latest next fall. */
#define SRQ_POLLS(f)                                                           \
  (((SRQ_LAST + SRQ_GMAX + 16 * SRQ_U) * (f) - SRQ_START(f)) / SRQ_POLL + 2)
/* Write: bit period for DATA set-up and hold slack, and for the drive loop. */
#define SRQ_MIN(f) (2 * (X_RISE + SRQ_SIGMA) + (f))
#define SRQ_FLOOR(f) (((SRQ_RLOOP + 1) * (f) + X_RISE + SRQ_SIGMA + 7) / 8)
#define SRQ_BIT(f) (SRQ_MIN(f) > SRQ_FLOOR(f) ? SRQ_MIN(f) : SRQ_FLOOR(f))
/* Write: SRQ low and high per bit, set-up and hold slack equal. */
#define SRQ_LOW(f) ((SRQ_BIT(f) - (f)) / 2)
#define SRQ_HIGH(f) (SRQ_BIT(f) - SRQ_LOW(f))

/*
 * Stream (firmware v12, 2 MHz, f = 8): the drive writes a byte SRQ_PERIOD or
 * more cycles after the previous one, so the next byte's first fall comes
 * (SRQ_PERIOD - 1) * f or more clocks after this one's (one cycle of timer
 * phase), at most X_POLL before its detection. After a byte the adapter's poll
 * that must see SRQ released comes once the last rise has settled; its first
 * poll for the next fall no later than that fall.
 */
#define SRQ_PERIOD 40
#define SRQ_FRAME(f) (SRQ_LAST * (f) + X_RISE)
#define SRQ_WAIT(f) ((SRQ_PERIOD - 1) * (f) - X_POLL)
/* Polls (6 clocks) of the fall wait: 20 ms, over 20 SRQ_PERIODs of metadata
 * gap in the drive's sync loop and the 1581 stream's keepalive interval, under
 * the host's patience. Long arithmetic: the AVR's unsigned int is 16 bits, and
 * the product overflows it. The count itself fits the 16-bit register pair. */
#define SRQ_STREAM_POLLS ((uint16_t)(20000UL * 16 / X_POLL))

#endif
