#!/usr/bin/env python3
"""Check the X protocol's compiled port timing against the schedules in x.c.

Steps each x_rx/x_tx, burst xb_rx/xb_tx and SRQ srq_rx/srq_tx routine from every
detecting sbic and compares each in/out clock (and srq_rx's next-byte poll, an sbis
seeing SRQ released) with the x.c formulas; bursts run BURST bytes and switch USB
banks after BANK (a counter whose dec guards an lds). For srq_stream8 it also takes
every branch after the samples: the poll that must see SRQ released comes at
SRQ_FRAME or later, the first poll for the next fall at SRQ_WAIT or earlier, and
that wait polls every X_POLL clocks and its compiled count is the 20 ms of
x_timing.h (the AVR's 16-bit int would silently shorten it). usage: x_timing.py ELF
[x.c] (x_timing.h beside x.c is read too)
"""

import os
import re
import subprocess
import sys

BURST = 3
BANK = 2
STREAM_GAP_US = 20000  # x_timing.h SRQ_STREAM_POLLS: the fall wait
CYCLES = {"cbi": 2, "sbi": 2, "rjmp": 2, "lds": 2, "sts": 2, "sbiw": 2}
SKIPS = ("sbrs", "sbrc", "sbis", "sbic", "cpse")
BRANCHES = ("breq", "brne", "brcs", "brcc", "brmi", "brpl", "brvs", "brvc")
LINE = re.compile(
    r"\s+([0-9a-f]+):\s+(?:[0-9a-f]{2} )+\s*(\w+)\s*([^;]*)(?:;\s*0x([0-9a-f]+))?"
)


def defines(src):
    """Integer #defines of x.c."""
    return {k: int(v) for k, v in re.findall(r"#define ((?:X|SRQ)_?\w+) (\d+)\b", src)}


def sample(d, a, b, f):
    """X_SAMPLE: centre of drive window [a, b) in clocks."""
    return ((a + b) * f + d["X_RISE"] - d["X_POLL"]) // 2


def change(d, a, b, f):
    """X_CHANGE: change between drive reads a and b in clocks."""
    return ((a + b) * f - d["X_POLL"] - d["X_RISE"]) // 2 - d["X_SYNC"]


def srq(d, f):
    """SRQ clocks: read samples, next-byte poll, write bit/low/byte periods."""
    last, sig = 15 * d["SRQ_U"], (16 * d["SRQ_U"] - d["X_RISE"] - d["X_POLL"]) // 2
    loop = ((d["SRQ_RLOOP"] + 1) * f + d["X_RISE"] + sig + 7) // 8
    bit = max(2 * (d["X_RISE"] + sig) + f, loop)
    u = 2 * d["SRQ_U"]
    return (
        [sample(d, u * j, u * (j + 1), f) for j in range(8)],
        sample(d, last, last + d["SRQ_GMIN"], f),
        bit,
        (bit - f) // 2,
    )


def expected(d, name):
    """[(op, clock)] of the port accesses of routine name."""
    f = int(re.search(r"(\d+)$", name).group(1))
    if name.startswith("srq_"):
        samples, start, bit, low = srq(d, f)
        if "rx" in name:
            return [("in", t) for t in samples] + [("poll", start)]
        first, out = d["X_FOUND"] + d["SRQ_ENCODE"], []
        for i in range(BURST):
            for k in range(8):
                t = first + (8 * i + k) * bit
                out += [t, t + low]
        return [("out", t) for t in out + [out[-1] + bit - low]]
    if name.startswith("xb_"):
        w = [d[f"XBW{i}"] for i in range(5)]
        r = [d[f"XBR{i}"] for i in range(5)]
        if "rx" in name:
            per = d["XBWN"] * f
            return [
                ("in", sample(d, w[k], w[k + 1], f) + i * per)
                for i in range(BURST)
                for k in range(4)
            ]
        per, out = d["XBRN"] * f, [d["X_FOUND"]]
        for i in range(BURST):
            out += [change(d, r[k], r[k + 1], f) + i * per for k in (1, 2, 3)]
            nxt = change(d, r[4], r[1] + d["XBRN"], f) + i * per
            out.append(nxt if i + 1 < BURST else (r[4] + 4) * f + i * per)
        return [("out", t) for t in out]
    w = [d[f"XW{i}"] for i in range(5)]
    r = [d[f"XR{i}"] for i in range(5)]
    if "rx" in name:
        return [("in", sample(d, w[k], w[k + 1], f)) for k in range(4)]
    out = [d["X_FOUND"]] + [change(d, r[k], r[k + 1], f) for k in range(1, 4)]
    return [("out", t) for t in out + [(r[4] + 4) * f]]


def routines(elf):
    """{name: [(addr, mnemonic, operands, target)]} for the X routines."""
    out = subprocess.run(
        ["avr-objdump", "-d", elf], check=True, capture_output=True, text=True
    ).stdout
    funcs, cur = {}, None
    for line in out.splitlines():
        m = re.match(r"[0-9a-f]+ <((?:xb?|srq)_[rt]x(?:8|16)|srq_stream8)[.\w]*>:", line)
        if m:
            cur = funcs.setdefault(m.group(1), [])
        elif cur is not None and (m := LINE.match(line)):
            target = int(m.group(4), 16) if m.group(4) else None
            cur.append((int(m.group(1), 16), m.group(2), m.group(3).strip(), target))
        elif not line.strip():
            cur = None
    return funcs


def run(code, start, count):
    """Clocks of the first count in/out after the detecting sbic at index start."""
    at = {a: i for i, (a, *_) in enumerate(code)}
    regs, zero, t, i, events = {}, False, 2, start + 2, []
    while len(events) < count:
        _, op, args, target = code[i]
        if op in ("in", "out"):
            events.append((op, t))
        if op == "sbis":
            events.append(("poll", t))
            t, i = t + 2, i + 2
            continue
        if op == "ldi":
            reg, val = args.split(",")
            regs[reg.strip()] = int(val, 0) & 0xFF
        elif op == "dec":
            bank = code[i + 1][1] == "brne" and code[i + 2][1] == "lds"
            regs[args] = (regs.get(args, BANK if bank else BURST) - 1) & 0xFF
            zero = regs[args] == 0
        if op in ("brne", "breq") and (op == "breq") == zero:
            t, i = t + 2, at[target]
        elif op == "rjmp":
            t, i = t + 2, at[target]
        else:
            t, i = t + CYCLES.get(op, 1), i + 1
    return events


def _step(code, at, i):
    """(clocks, next indices) of instruction i with every branch outcome."""
    addr, op, _, target = code[i]
    if op == "rjmp":
        return [(2, at[target])]
    if op in BRANCHES:
        return [(2, at[target]), (1, i + 1)]
    if op in SKIPS:
        size = code[i + 2][0] - code[i + 1][0] if i + 2 < len(code) else 2
        return [(1 + size // 2, i + 2), (1, i + 1)]
    return [(CYCLES.get(op, 1), i + 1)]


def spans(code, start, t0, stop):
    """{index: (min, max)} clocks at which every path from start (at t0) reaches an
    instruction where stop(index) holds; paths branching backwards end there."""
    at = {a: i for i, (a, *_) in enumerate(code)}
    out, todo = {}, [(start, t0)]
    while todo:
        i, t = todo.pop()
        if i >= len(code) or code[i][1] == "ret":
            continue
        if stop(i):
            lo, hi = out.get(i, (t, t))
            out[i] = (min(lo, t), max(hi, t))
            continue
        for dt, j in _step(code, at, i):
            if j > i:
                todo.append((j, t + dt))
    return out


def stream(d, code):
    """Mismatches of srq_stream8 against x_timing.h, with the measured clocks."""
    bad, notes = 0, []
    want = [("in", sample(d, 2 * j * d["SRQ_U"], (2 * j + 2) * d["SRQ_U"], 8))
            for j in range(8)]
    det = [s for s, c in enumerate(code) if c[1] == "sbic" and code[s + 1][1] == "rjmp"]
    for s in det:
        got = run(code, s, 8)
        bad += got != want
        notes.append(f"sbic@{code[s][0]:04x} samples {got == want}")
    last = max(i for i, c in enumerate(code) if c[1] == "in")
    t_last = want[-1][1]
    frames = spans(code, last + 1, t_last + 1, lambda i: code[i][1] == "sbis")
    lo, hi = min(v[0] for v in frames.values()), max(v[1] for v in frames.values())
    frame = 8 * 15 * d["SRQ_U"] + d["X_RISE"]
    wait = (d["SRQ_PERIOD"] - 1) * 8 - d["X_POLL"]
    poll = [i for i in range(len(code)) if code[i][1] == "sbic" and i > min(frames)]
    first = spans(code, min(frames), lo, lambda i: i == poll[0])
    sbis_ok = 2 + sum(CYCLES.get(c[1], 1) for c in code[min(frames) + 2 : poll[0]])
    late = hi + sbis_ok
    loop = sum(CYCLES.get(c[1], 1) for c in code[poll[0] - 2 : poll[0] + 2])
    polls = wait_count(code, poll[0] - 4)
    want_polls = STREAM_GAP_US * 16 // d["X_POLL"]
    ok = lo >= frame and late <= wait and loop == d["X_POLL"] and first
    ok = ok and polls == want_polls
    notes.append(f"frame [{lo}, {hi}] >= {frame}, next poll <= {late} <= {wait}")
    notes.append(f"wait loop {loop} clocks per poll")
    notes.append(f"fall wait {polls} polls == {want_polls}, {polls * loop / 16000:.1f} ms")
    return bad + (not ok), notes


def wait_count(code, i):
    """The 16-bit immediate two ldi at index i load (the compiled fall-wait count), or
    None when the instructions there are not that pair."""
    try:
        lo, hi = (int(code[i + k][2].split(",")[1], 0) for k in (0, 1))
    except (ValueError, IndexError):
        return None
    if code[i][1] != "ldi" or code[i + 1][1] != "ldi":
        return None
    return lo | hi << 8


def main(argv):
    """Exit status 1 on any mismatch."""
    src = argv[2] if len(argv) > 2 else "x.c"
    head = os.path.join(os.path.dirname(src), "x_timing.h")
    with open(src, encoding="utf-8") as fh, open(head, encoding="utf-8") as fh2:
        d, bad = defines(fh.read() + fh2.read()), 0
    for name, code in sorted(routines(argv[1]).items()):
        if name == "srq_stream8":
            n, notes = stream(d, code)
            bad += n
            print(f"{name}: {'ok' if not n else 'MISMATCH'} {'; '.join(notes)}")
            continue
        want = expected(d, name)
        for s, c in enumerate(code):
            if c[1] == "sbic" and code[s + 2][1] == "rjmp":
                got = run(code, s, len(want))
                bad += got != want
                print(f"{name} sbic@{c[0]:04x}: {'ok' if got == want else 'MISMATCH'} {got}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
