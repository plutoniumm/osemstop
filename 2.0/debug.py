#!/usr/bin/env python3
import argparse
import sys
import time
import numpy as np
from rich.text import Text
import osem
from osem.session import STEP, Hold, check, guard, need_room, ready, say

NEAR, NSIG, FLOOR = (60, 4.0, 0.3)
REMEASURE = "run `python run.py calib` before bench"
WORD = dict(
    OK="same",
    SKIP="too weak to judge",
    DEAD="DEAD",
    WEAK="weaker",
    FLIPPED="FLIPPED",
    CHANGED="changed",
)
ALL_DEAD = "every coil dead: DAC wiring or DAC power; the firmware enables the DAC reference only at boot, so power the DAC then reset the board"


def commands(board, rig):
    board._ask(b"ACK 1")
    bad = []
    for j, (d, v) in enumerate(zip(rig.dac_map, rig.bias)):
        got = board._ask(b"SET %d %.4f" % (d, v))
        if b"OK ch=%d v=%.4f" % (d, v) not in got:
            bad.append("coil %d (DAC %d) answered %r" % (j, d, got.strip()[:30]))
    refuses = b"ERR" in board._ask(b"BOGUS")
    board._ask(b"ACK 0")
    if bad or not refuses:
        return (False, "; ".join(bad) or "board accepted a nonsense command: wrong firmware?")
    return (True, "the board accepts a voltage for all %d coils" % rig.n)


def sensors(T, C, cfg):
    hz = (len(T) - 1) / (T[-1] - T[0])
    lo, hi, sd = (C.min(0), C.max(0), C.std(0))
    near = (lo < NEAR) | (hi > cfg.adc_max - NEAR)
    flag = np.where(sd < 0.05, "stuck", np.where(near, "near the ADC limit", ""))
    flag = np.where((lo <= 0) | (hi >= cfg.adc_max), "railed", flag)
    bad = flag != ""
    hint = ", ".join(
        "a%d %s (%.0f..%.0f)" % (i, flag[i], lo[i], hi[i]) for i in np.flatnonzero(bad)
    )
    if bad.any():
        hint += ": check that sensor's lead, LED and flag position"
    return (not hint, hint or "all %d read, %.0f frames/s" % (C.shape[1], hz), bad, sd)


def drain(board, seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        board.read()


def link(board, rig, s0):
    drain(board, 0.2)
    lat = []
    for _ in range(50):
        ok, t = (board.cmd_ok, time.perf_counter())
        board.write(rig.bias)
        while board.cmd_ok == ok and time.perf_counter() - t < 0.2:
            board.read()
        lat.append(time.perf_counter() - t)
    drain(board, 0.2)
    med = np.median(lat) * 1000.0
    sent = board.sent - s0
    ok = not (board.lost or board.bad or board.cmd_bad) and sent - board.cmd_ok <= 2
    if ok:
        return (True, "0 of %d frames lost, command round trip %.1f ms" % (board.frames, med))
    return (
        False,
        "%d frames lost, %d corrupt, %d commands rejected: USB cable, hub, or a busy host"
        % (board.lost, board.bad, board.cmd_bad),
    )


def verdict(m, sg, e, k, use):
    seen = use & (np.abs(m) > NSIG * sg)
    big = use & (np.abs(e) >= 0.3 * abs(e[k]))
    if abs(e[k]) <= NSIG * sg[k]:
        return "SKIP"
    if not seen.any():
        return "DEAD"
    if (seen & big).any() and (m * e < 0)[seen & big].all():
        return "FLIPPED"
    if 0 <= m[k] / e[k] < 0.5:
        return "WEAK"
    off = np.abs(m - e) > np.maximum(np.maximum(0.5 * np.abs(e), NSIG * sg), 5.0)
    return "CHANGED" if off[big].any() else "OK"


def coils(hold, rig, which, use, win):
    say(
        "\ncoils: each one stepped +-%.2f V; response of its strongest sensor, counts per volt"
        % STEP
    )
    say("  coil  sensor   at calib      now          verdict", "dim")
    out = {}
    for j in which:
        lv, ring = ([], 0.0)
        for s in (-1, 1, -1):
            v = rig.bias.copy()
            v[j] += s * STEP
            C = hold(v, win, ramp_s=1.0)[1]
            lv.append(C.mean(0))
            ring = np.maximum(ring, C.std(0))
        m = (lv[1] - 0.5 * (lv[0] + lv[2])) / (2 * STEP)
        leak = ring * np.sqrt(2) / (np.pi * rig.f_hz.min() * win)
        sg = np.maximum(np.maximum(np.abs(lv[0] - lv[2]) / 2, FLOOR), leak) / (2 * STEP)
        e = rig.dc[:, j]
        k = int(np.argmax(np.abs(e) * use))
        out[j] = verdict(m, sg, e, k, use)
        row = "  %4d     a%d   %+7.1f   %+7.1f +-%-5.1f " % (j, k, e[k], m[k], sg[k])
        say(Text.assemble(row, (WORD[out[j]], "green" if out[j] == "OK" else "red")))
    hold(rig.bias, 0.0, ramp_s=1.0)
    return out


def coil_hints(rig, v):
    judged = [j for j in v if v[j] != "SKIP"]
    bad = [j for j in judged if v[j] != "OK"]
    if len(judged) > 1 and all(v[j] == "DEAD" for j in judged):
        return ALL_DEAD
    if not bad:
        return "all %d respond as they did at calib" % len(judged)
    return "%s differ from calib: %s" % (
        ", ".join("coil %d" % j for j in bad),
        "check the leads of the dead ones" if any(v[j] == "DEAD" for j in bad) else REMEASURE,
    )


class Report(list):
    def append(self, row):
        name, ok, hint = row
        check(ok, name, hint, 9)
        super().append(row)


def checks(board, rig, cfg, R, which=None, fast=False):
    R.append(("commands",) + commands(board, rig))
    board.start(rig.bias)
    s0 = board.sent
    hold = Hold(board, cfg, rig.n)
    hold.v = rig.bias.copy()
    T, C = hold(rig.bias, 5.0, ramp_s=0.0)
    if len(T) < 2:
        return R.append(("sensors", False, "no sample frames: firmware stream or USB"))
    ok, hint, bad, sd = sensors(T, C, cfg)
    R.append(("sensors", ok, hint))
    if sd[:4].max() > 10:
        say(
            "  note  plate is swinging (a%d moves %.0f counts rms): coil readings below are rough"
            % (sd[:4].argmax(), sd[:4].max()),
            "yellow",
        )
    R.append(("link",) + link(board, rig, s0))
    if not board.cmd_ok:
        return R.append(("coils", False, "skipped: the board accepted no command frame"))
    use = ~bad & (np.arange(rig.n) >= (4 if fast else 0))
    if not use.any():
        return R.append(("coils", False, "skipped: no usable sensor"))
    if fast:
        say("--fast: 1.5 s windows do not average the plate's ring out; judging on a4..a7 only")
    v = coils(hold, rig, range(rig.n) if which is None else which, use, 1.5 if fast else 4.05)
    R.append(
        ("coils", set(v.values()) <= {"OK", "SKIP"} and "OK" in v.values(), coil_hints(rig, v))
    )


def main():
    ap = argparse.ArgumentParser(description="walk port -> coils once; never closes a loop")
    ap.add_argument("--port")
    ap.add_argument("--coils", type=lambda s: [int(x) for x in s.split(",")])
    ap.add_argument("--fast", action="store_true")
    a = ap.parse_args()
    rig, cfg = (osem.Rig.load(), osem.Config())
    need_room(rig.bias, cfg)
    if any(not 0 <= j < rig.n for j in a.coils or ()):
        sys.exit("--coils are numbered 0..%d" % (rig.n - 1))
    guard()
    say("checking the rig, about 2.5 minutes")
    R, board = (Report(), None)
    try:
        port = a.port or osem.Board.find_port()
        R.append(("port", True, port))
        board = osem.Board(rig, cfg, port, say=lambda *_: None)
        R.append(("firmware", True, "version %s, matches this code" % board.fw))
    except (SystemExit, OSError) as e:
        hint = "old firmware: python flash.py" if "ERR unknown" in str(e) else str(e)
        R.append((("port", "firmware")[len(R)], False, hint))
    if board:
        try:
            checks(board, rig, cfg, R, a.coils, a.fast)
        except KeyboardInterrupt:
            R.append(("interrupted", False, "coils parked at bias"))
        finally:
            board.park(rig.bias)
            board.close()
    bad = [(n, h) for n, ok, h in R if not ok]
    sys.exit(not ready("%s: %s" % bad[0] if bad else ""))


if __name__ == "__main__":
    main()
