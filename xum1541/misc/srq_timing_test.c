/*
 * Host check of the SRQ schedule (x_timing.h): every read sample and the next
 * byte's first poll inside their windows, the poll count within 8 bits and
 * covering the latest next byte, and the write bit and byte periods meeting
 * DATA set-up, hold and the drive's receive loop with SRQ_SIGMA to spare.
 * usage: cc -I.. srq_timing_test.c && ./a.out
 */
#include <stdio.h>

#include "x_timing.h"

static int check(const char *what, int f, int slack) {
  if (*what || slack < 0)
    printf("  f=%2d %-22s %4d\n", f, *what ? what : "sample", slack);
  return slack < 0;
}

static int schedule(int f) {
  int bad = 0, j, s, u = 2 * SRQ_U;

  for (j = 0; j < 8; j++) {
    s = SRQ_SAMPLE(j, f);
    bad += check(j ? "" : "sample after rise", f, s - u * j * f - X_RISE);
    bad +=
        check(j ? "" : "sample before change", f, u * (j + 1) * f - s - X_POLL);
  }
  s = SRQ_START(f);
  bad += check("poll after last rise", f, s - SRQ_LAST * f - X_RISE);
  bad +=
      check("poll before next fall", f, (SRQ_LAST + SRQ_GMIN) * f - s - X_POLL);
  bad += check("polls fit a byte", f, 255 - SRQ_POLLS(f));
  /* the high poll at s, then low polls from s + 4, every SRQ_POLL */
  bad += check("polls cover next byte", f,
               s + 4 + (SRQ_POLLS(f) - 2) * SRQ_POLL -
                   (SRQ_LAST + SRQ_GMAX + u * 8) * f);
  bad += check("write set-up", f, SRQ_LOW(f) - X_RISE - SRQ_SIGMA);
  bad += check("write hold", f, SRQ_HIGH(f) - X_RISE - f - SRQ_SIGMA);
  bad += check("write drive loop", f,
               8 * SRQ_BIT(f) - (SRQ_RLOOP + 1) * f - X_RISE - SRQ_SIGMA);
  return bad;
}

int main(void) {
  int bad = schedule(16) + schedule(8);

  printf("sigma %d, bit %d/%d clocks\n%s\n", SRQ_SIGMA, SRQ_BIT(16), SRQ_BIT(8),
         bad ? "FAIL" : "ok");
  return bad != 0;
}
