/*
 * Host check of the burst X endpoint readiness rules (xb_ready.h) against a
 * model of a double-banked OUT endpoint: NBUSYBK counts filled banks.
 * usage: cc -I.. xb_ready_test.c && ./a.out
 */
#include <stdio.h>

#include "xb_ready.h"

#define BANK 32

int main(void) {
  int bad = 0, len, k;

  for (len = 1; len <= 2 * BANK; len++) {
    /* The host sends len bytes as packets of BANK; both banks fill. */
    uint8_t first = len > BANK ? BANK : len;
    uint8_t busy = len > BANK ? 2 : 1;

    k = len;
    if (!xb_out_ready(k, first, busy, BANK)) {
      printf("burst of %d never starts\n", len);
      bad++;
    }
    if (len > BANK && xb_out_ready(k, first, 1, BANK)) {
      printf("burst of %d starts before its second packet\n", len);
      bad++;
    }
  }
  bad += !xb_in_ready(0) + xb_in_ready(1) + xb_in_ready(2);
  printf("%s\n", bad ? "FAIL" : "ok");
  return bad != 0;
}
