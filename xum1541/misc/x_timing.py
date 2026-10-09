#!/usr/bin/env python3
"""Check the X protocol's compiled port timing against the schedules in x.c.

Steps each x_rx/x_tx, burst xb_rx/xb_tx and SRQ srq_rx/srq_tx routine from every
detecting sbic and compares each in/out clock (and srq_rx's next-byte poll, an sbis
seeing SRQ released) with the x.c formulas; bursts run BURST bytes and switch USB
banks after BANK (a counter whose dec guards an lds). usage: x_timing.py ELF [x.c] (x_timing.h beside x.c is read too)
"""

import os
import re
import subprocess
import sys

BURST = 3
BANK = 2
CYCLES = {"cbi": 2, "sbi": 2, "rjmp": 2, "lds": 2, "sts": 2}
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
        m = re.match(r"[0-9a-f]+ <((?:xb?|srq)_[rt]x(?:8|16))[.\w]*>:", line)
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


def main(argv):
    """Exit status 1 on any mismatch."""
    src = argv[2] if len(argv) > 2 else "x.c"
    head = os.path.join(os.path.dirname(src), "x_timing.h")
    with open(src, encoding="utf-8") as fh, open(head, encoding="utf-8") as fh2:
        d, bad = defines(fh.read() + fh2.read()), 0
    for name, code in sorted(routines(argv[1]).items()):
        want = expected(d, name)
        for s, c in enumerate(code):
            if c[1] == "sbic" and code[s + 2][1] == "rjmp":
                got = run(code, s, len(want))
                bad += got != want
                print(f"{name} sbic@{c[0]:04x}: {'ok' if got == want else 'MISMATCH'} {got}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
