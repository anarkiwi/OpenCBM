#!/usr/bin/env python3
"""Check the X protocol's compiled port timing against the schedule in x.c.

Steps each x_rx/x_tx routine of the firmware ELF from every SYNC-detecting sbic
and compares the clock of each in/out with the X_SAMPLE/X_CHANGE offsets.
usage: x_timing.py ELF [x.c]
"""

import re
import subprocess
import sys

CYCLES = {"cbi": 2, "rjmp": 2, "ldi": 1, "dec": 1, "nop": 1, "in": 1, "out": 1}
LINE = re.compile(
    r"\s+([0-9a-f]+):\s+(?:[0-9a-f]{2} )+\s*(\w+)\s*([^;]*)(?:;\s*0x([0-9a-f]+))?"
)


def defines(src):
    """Integer #defines of x.c."""
    return {k: int(v) for k, v in re.findall(r"#define (X\w+) (\d+)\b", src)}


def expected(d, f):
    """Clocks of the four samples and of the five drive changes at f clocks/cycle."""
    w = [d[f"XW{i}"] for i in range(5)]
    r = [d[f"XR{i}"] for i in range(5)]
    rise, poll, sync = d["X_RISE"], d["X_POLL"], d["X_SYNC"]
    sample = [((w[k] + w[k + 1]) * f + rise - poll) // 2 for k in range(4)]
    change = [((r[k] + r[k + 1]) * f - poll - rise) // 2 - sync for k in range(1, 4)]
    return sample, [d["X_FOUND"]] + change + [(r[4] + 4) * f]


def routines(elf):
    """{name: [(addr, mnemonic, operands, target)]} for the x_rx/x_tx routines."""
    out = subprocess.run(
        ["avr-objdump", "-d", elf], check=True, capture_output=True, text=True
    ).stdout
    funcs, cur = {}, None
    for line in out.splitlines():
        m = re.match(r"[0-9a-f]+ <(x_[rt]x(?:8|16))[.\w]*>:", line)
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
    regs, t, i, events = {}, 2, start + 2, []
    while len(events) < count:
        _, op, args, target = code[i]
        if op in ("in", "out"):
            events.append((op, t))
        if op == "ldi":
            reg, val = args.split(",")
            regs[reg.strip()] = int(val, 0)
        elif op == "dec":
            regs[args] -= 1
        if op == "brne" and regs["r23"]:
            t, i = t + 2, at[target]
        elif op == "rjmp":
            t, i = t + 2, at[target]
        else:
            t, i = t + CYCLES.get(op, 1), i + 1
    return events


def main(argv):
    """Exit status 1 on any mismatch."""
    with open(argv[2] if len(argv) > 2 else "x.c", encoding="utf-8") as fh:
        d, bad = defines(fh.read()), 0
    for name, code in sorted(routines(argv[1]).items()):
        sample, change = expected(d, int(name[4:]))
        want = [("in", t) for t in sample] if "rx" in name else [("out", t) for t in change]
        for s, c in enumerate(code):
            if c[1] == "sbic" and code[s + 2][1] == "rjmp":
                got = run(code, s, len(want))
                bad += got != want
                print(f"{name} sbic@{c[0]:04x}: {'ok' if got == want else 'MISMATCH'} {got}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
