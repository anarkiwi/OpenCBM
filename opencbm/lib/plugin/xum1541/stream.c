/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

/*! **************************************************************
** \file lib/plugin/xum1541/stream.c \n
** \brief SRQ streaming receive with queued asynchronous bulk IN transfers
****************************************************************/

#include <stdio.h>
#include <string.h>

#include "opencbm.h"

#include "arch.h"
#include "dynlibusb.h"
#include "xum1541.h"

#if HAVE_LIBUSB1

/** Bulk IN transfers kept in flight. */
#define STREAM_XFERS 8
/** Bytes per bulk IN transfer, a multiple of XUM_STREAM_UNIT. */
#define STREAM_XFER_SIZE 4096

enum stream_state { STREAM_RUN, STREAM_END, STREAM_ERROR };

/** Shared state of one stream: the next region to submit and the result. */
struct stream {
  unsigned char *data;
  unsigned int size, next, total;
  int inflight, sentinel, state;
  unsigned char scratch[XUM_STREAM_UNIT];
};

/*
 * Point a transfer at the next unfilled region of the buffer, or, once the
 * buffer is covered, at a scratch buffer that absorbs the terminating ZLP.
 */
static int stream_submit(struct stream *s, struct libusb_transfer *t) {
  unsigned char *buf;
  unsigned int len;

  if (s->next < s->size) {
    buf = s->data + s->next;
    len = s->size - s->next;
    if (len > STREAM_XFER_SIZE)
      len = STREAM_XFER_SIZE;
    s->next += len;
  } else if (!s->sentinel) {
    buf = s->scratch;
    len = sizeof(s->scratch);
    s->sentinel = 1;
  } else {
    return 0;
  }
  t->buffer = buf;
  t->length = (int)len;
  if (usb.submit_transfer(t) != LIBUSB_SUCCESS) {
    s->state = STREAM_ERROR;
    return -1;
  }
  s->inflight++;
  return 1;
}

/*
 * Bulk IN transfers complete in submission order, so each completion extends
 * the contiguous prefix of the buffer; a short one ends the stream.
 */
static void LIBUSB_CALL stream_done(struct libusb_transfer *t) {
  struct stream *s = t->user_data;

  s->inflight--;
  t->user_data = NULL;
  if (s->state != STREAM_RUN)
    return;
  if (t->status != LIBUSB_TRANSFER_COMPLETED) {
    s->state = STREAM_ERROR;
    return;
  }
  if (t->buffer == s->scratch) {
    s->state = t->actual_length == 0 ? STREAM_END : STREAM_ERROR;
    return;
  }
  s->total += (unsigned int)t->actual_length;
  if (t->actual_length < t->length) {
    s->state = STREAM_END;
    return;
  }
  t->user_data = s;
  if (stream_submit(s, t) <= 0)
    t->user_data = NULL;
}

/* Cancel what is still queued and run events until every transfer is back. */
static int stream_drain(struct libusb_context *ctx, struct stream *s,
                        struct libusb_transfer **xfer) {
  struct timeval tv = {1, 0};
  int i, cancelled = 0, ret;

  while (s->inflight > 0) {
    if (s->state != STREAM_RUN && !cancelled) {
      for (i = 0; i < STREAM_XFERS; i++)
        if (xfer[i] != NULL && xfer[i]->user_data != NULL)
          usb.cancel_transfer(xfer[i]);
      cancelled = 1;
    }
    ret = usb.handle_events_timeout_completed(ctx, &tv, NULL);
    if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_INTERRUPTED) {
      fprintf(stderr, "USB error in stream events: %s\n", usb.error_name(ret));
      return -1;
    }
  }
  return 0;
}

static int stream_run(struct opencbm_usb_handle *h, unsigned char *data,
                      unsigned int units) {
  struct libusb_transfer *xfer[STREAM_XFERS] = {NULL};
  struct stream s;
  unsigned char cmd[XUM_CMDBUF_SIZE];
  unsigned int timeout = xum1541_timeout(0, 0);
  int i, n = 0, ret, drained;

  memset(&s, 0, sizeof(s));
  s.data = data;
  s.size = units * XUM_STREAM_UNIT;
  s.state = STREAM_RUN;

  for (i = 0; i < STREAM_XFERS && s.state == STREAM_RUN; i++) {
    xfer[i] = usb.alloc_transfer(0);
    if (xfer[i] == NULL) {
      s.state = STREAM_ERROR;
      break;
    }
    libusb_fill_bulk_transfer(xfer[i], h->devh,
                              XUM_BULK_IN_ENDPOINT | LIBUSB_ENDPOINT_IN, NULL,
                              0, stream_done, &s, timeout);
    if (stream_submit(&s, xfer[i]) <= 0)
      xfer[i]->user_data = NULL;
  }

  if (s.state == STREAM_RUN) {
    cmd[0] = XUM1541_READ;
    cmd[1] = XUM1541_X | XUM_X_STREAM | XUM_X_2MHZ;
    cmd[2] = (unsigned char)(units & 0xff);
    cmd[3] = (unsigned char)(units >> 8);
    ret =
        usb.bulk_transfer(h->devh, XUM_BULK_OUT_ENDPOINT | LIBUSB_ENDPOINT_OUT,
                          cmd, sizeof(cmd), &n, timeout);
    if (ret != LIBUSB_SUCCESS || n != (int)sizeof(cmd)) {
      fprintf(stderr, "USB error in stream cmd: %s\n", usb.error_name(ret));
      s.state = STREAM_ERROR;
    }
  }

  drained = stream_drain(h->ctx, &s, xfer) == 0;
  if (drained)
    for (i = 0; i < STREAM_XFERS; i++)
      if (xfer[i] != NULL)
        usb.free_transfer(xfer[i]);
  if (s.state == STREAM_ERROR || !drained) {
    xum1541_resync(h);
    return -1;
  }
  return (int)s.total;
}

#endif

/*! \brief Receive an SRQ stream from a 1571 at 2 MHz

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the buffer which will hold the stream, including its
    two-byte trailer (XUM_STREAM_ESC and an XUM_STREAM_* code).

  \param size
    The size of the buffer, used in whole XUM_STREAM_UNIT units.
    Below one unit, nothing is transferred.

  \return
    The number of bytes received, 0 if size is below one unit, or -1 on
    error or if the firmware lacks SRQ streaming (version 12).
*/
int CBMAPIDECL opencbm_plugin_srq2_stream(CBM_FILE HandleDevice,
                                          unsigned char *data,
                                          unsigned int size) {
#if HAVE_LIBUSB1
  unsigned int units = size / XUM_STREAM_UNIT;

  if (DeviceFirmwareVersion < 12 ||
      (DeviceCapabilities & XUM1541_CAP_STREAM) == 0)
    return -1;
  if (units == 0)
    return 0;
  if (units > 0xffff)
    units = 0xffff;
  return stream_run((struct opencbm_usb_handle *)HandleDevice, data, units);
#else
  (void)HandleDevice;
  (void)data;
  (void)size;
  return -1;
#endif
}
