# xum1541 hang analysis

Why a stalled drive transfer wedges the ZoomFloppy until power cycle, why
`cbmctrl reset` sometimes needs two tries, and what this branch changes.
Line numbers refer to the parent commit (`git show HEAD~:<file>` once this
branch is committed on top of it).

## Failure chain

1. **Firmware waits are unbounded.** Every S1/S2 handshake loop spins on a
   line state and only leaves when `TimerWorker()` returns false
   (`xum1541/s1.c:34-57`, `s1.c:68-87`, `xum1541/s2.c:30-46`, `s2.c:61-75`), as
   does `iec_wait()` for `XUM1541_IEC_WAIT` (`xum1541/iec.c:565`).
   `TimerWorker()` (`xum1541/main.c:256-269`) kicks the 1 s watchdog
   (`cpu-zoomfloppy.h:21`) on every iteration, so the watchdog never fires, and
   it only returns false when `doDeviceReset` is set.
2. **Nothing sets `doDeviceReset` within a session.** Its only writer is
   `SetAbortState()` (`main.c:235-247`), called only from the `XUM1541_INIT`
   control request when the previous session never sent `XUM1541_SHUTDOWN`
   (`xum1541/commands.c:627-632`). A drive that stops toggling CLK/DATA (crashed
   drive code, wrong protocol, a second drive holding a line) therefore pins the
   main loop forever.
3. **The host waits forever too.** Every bulk transfer in the plugin uses
   `LIBUSB_NO_TIMEOUT` (`opencbm/lib/plugin/xum1541/xum1541.c:808/813` status,
   `922/926` ioctl, `1005/1009` and `1030/1035` write, `1189/1193` and
   `1214/1218` read). A Python caller blocked inside ctypes cannot be
   interrupted: CPython only runs its SIGINT handler between bytecodes, so
   Ctrl-C is ignored until the C call returns, which it never does. The user
   ends up killing the process or pulling the cable.
4. **Errors kill the caller.** `xum1541_control_msg`, `xum1541_wait_status` and
   `xum1541_ioctl` call `exit(-1)` on any USB error (`xum1541.c:790, 829, 834,
   936`), so even a detected stall (EPIPE after the device stalls the endpoint)
   terminates the host process instead of being recoverable.
5. **Recovery only happens on the next `XUM1541_INIT`, inside the USB ISR.**
   The firmware is built with `-DINTERRUPT_CONTROL_ENDPOINT` (`Makefile:113`),
   so control requests run in `USB_COM_vect`. `XUM1541_INIT` and
   `XUM1541_RESET` call `cmds->cbm_reset()` there (`commands.c:630, 643`):
   `DELAY_MS(100)` (`iec.c:216`) plus `wait_for_free_bus()`, up to 7500 probes
   of up to 200 us (`iec.c:171`), i.e. up to about 1.6 s before the INIT reply
   is sent. The host allows `USB_TIMEOUT` = 1.5 s x 1100 = 1650 ms
   (`opencbm/lib/plugin/xum1541/xum1541.h:51`). Whenever the bus is not free
   quickly (drives still booting after RESET, a 1571 with a longer reset, a
   drive holding DATA), the INIT control transfer times out, `cbm_driver_open`
   fails, and the endpoints are left stalled by `SetAbortState()` with nobody
   to clear them. The next attempt sees `cmdSeqInProgress` still set, resets
   again and usually wins the race: hence "`cbmctrl reset` needs two tries".
6. **Aborted reads emit garbage.** `s1_read_byte`/`s2_read_byte` return `-1`
   from a `uint8_t` function (`s1.c:70, 79, 86`, `s2.c:63, 74`); `ioReadLoop`
   (`commands.c:216-228`) passes that 0xFF to `usbSendByte()`, which writes it
   into the IN FIFO before noticing the abort (`commands.c:161-175`), and
   `usbIoDone()` then commits the partial bank (`commands.c:123-125`). S1/S2
   transfers carry no status, so the host cannot tell a short or corrupted
   read from a good one.
7. **`XUM1541_IEC_WAIT` aborts silently.** When `cbm_wait()` returns false the
   handler sets `ret = 0` (`commands.c:816-818`) so no status block is sent,
   leaving the host's status read (item 3) blocked forever.
8. **Other unbounded spins.** `usbInitIo()` waits for the endpoint with no exit
   (`commands.c:109-110`); `usbSendByte`/`usbRecvByte` spin until the host
   reads/writes with only `doDeviceReset` as an exit (`commands.c:167, 198`).
9. Unrelated bug found on the way: `iec_poll_pins()` reads RESET from `PINC`
   although `IO_RESET_IN` is on port D (`board-zoomfloppy.c:79`).

## Changes on this branch

| Problem | Fix |
|---|---|
| 1, 2, 8 | 100 ms tick deadline in `TimerWorker()` (`IoArm`/`IoProgress`/`IoAborted`, `main.c`); armed for every bulk command outside tape mode, restarted on each byte of progress; default 30 s, settable with the new `XUM1541_SET_TIMEOUT` control request. All USB endpoint spins go through `TimerWorker()`. On expiry the IEC lines are released. |
| 3 | Finite libusb timeouts everywhere: firmware timeout + 3 s for commands/status, plus 4 ms per byte for data phases. `opencbm_plugin_xum1541_set_timeout()` changes both sides. |
| 4 | `exit(-1)` replaced by `xum1541_resync()` + `return -1`. |
| 5 | INIT/RESET only set `pendingReset`; the main loop performs the bus reset, so the INIT reply is immediate. New `XUM1541_ABORT` control request: wValue 1 stalls both bulk endpoints and flags the abort, wValue 0 polls until the main loop has unwound (FIFOs reset, lines released); the host then clears both halts. Stale FIFO data cannot leak into the next command. |
| 6 | Loops check `IoAborted()` before queueing a byte and return the count; S1/S2 send a status block (`IO_READY`/`IO_ERROR` + count) when the host sets `XUM_RW_STATUS`, which the plugin does for firmware >= 9. |
| 7 | A failed `IEC_WAIT` returns `XUM1541_IO_ERROR` with status. |
| 9 | Reads `PIND`. |

Firmware version is 9 (`XUM1541_VERSION`, `XUMFW_VERSION`); the plugin gates
the new behaviour on it, so old firmware keeps working (with host timeouts but
no device-side abort). The X protocol (`xum1541/x.c`, capability
`XUM1541_CAP_X`, `XUM1541_X` read/write, plugin `opencbm_plugin_x[2]_read_n` /
`x[2]_write_n`) is specified in nybulah `docs/protocol.md`;
`xum1541/misc/x_timing.py` checks the compiled sample/drive clocks against the
schedule and runs in `Dockerfile.nybulah`.

## Remaining limits

- Old firmware (< 9) cannot be aborted from the host; a wedge still needs a new
  session or replug.
- A resync during a deferred reset cuts the reset's bus wait short.
- Tape mode keeps infinite waits (it waits on the user by design).
