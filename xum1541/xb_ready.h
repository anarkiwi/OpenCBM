/*
 * Burst X endpoint readiness, shared by x.c and misc/xb_ready_test.c
 * Copyright (c) 2026 Josh Bailey
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * busy is UESTA0X & (NBUSYBK1 | NBUSYBK0): the number of busy banks (0..2),
 * not a bit mask. bytes is UEBCLX of the current bank, the only one the CPU
 * sees until FIFOCON is cleared.
 */
#ifndef XB_READY_H
#define XB_READY_H

#include <stdbool.h>
#include <stdint.h>

/* Read: a k-byte burst may start once no IN bank is waiting for the host. */
static inline bool xb_in_ready(uint8_t busy) { return busy == 0; }

/*
 * Write: k <= bank bytes must be in the current bank; a longer burst needs a
 * full current bank and the second bank filled too (its packet may be short).
 */
static inline bool xb_out_ready(uint8_t k, uint8_t bytes, uint8_t busy,
                                uint8_t bank) {
  return k > bank ? bytes == bank && busy == 2 : bytes >= k;
}

#endif
