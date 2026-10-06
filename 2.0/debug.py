#!/usr/bin/env python3
import argparse
import signal
import sys
import time
import numpy as np
import osem
from osem.procedures import Hold
from osem.session import keep_awake
from osem.term import STATE, c

STEP, NEAR, NSIG, FLOOR = (0.15, 60, 4.0, 0.3)
REMEASURE = "re-run `run.py measure` before closing a loop"
HINT = {
    "DEAD": "check DAC output %d and that coil's lead",
    "WEAK": "check DAC output %d and that coil's lead, then " + REMEASURE,
    "FLIPPED": "polarity reversed against rig.json (DAC %d): " + REMEASURE,
    "CHANGED": "response moved against rig.json (DAC %d): " + REMEASURE,
}
ALL_DEAD = "every coil dead: DAC wiring or DAC power; the firmware enables the DAC reference only at boot, so power the DAC then reset the board"


def commands(board, rig, say):
    board._ask(b"ACK 1")
    echo = errs = 0
    say("commands\n  coil dac  volts   reply")
    for c, (d, v) in enumerate(zip(rig.dac_map, rig.bias)):
        got = board._ask(b"SET %d %.4f" % (d, v))
        echo += b"OK ch=%d v=%.4f" % (d, v) in got
        errs += b"ERR" in got
        say("  %4d %3d  %.4f  %s" % (c, d, v, got.strip().decode(errors="replace")[:40]))
    bogus = board._ask(b"BOGUS")
    board._ask(b"ACK 0")
    say("  bad command -> %s; %d ERR on good ones" % (bogus.strip().decode(errors="replace"), errs))
    ok = echo == rig.n and errs == 0 and b"ERR" in bogus
    return (
        ok,
        "" if ok else "%d of %d SET echoes matched: serial link or firmware" % (echo, rig.n),
    )


def sensors(T, C, cfg, say):
    hz = (len(T) - 1) / (T[-1] - T[0])
    lo, hi, sd = (C.min(0), C.max(0), C.std(0))
    near = (lo < NEAR) | (hi > cfg.adc_max - NEAR)
    flag = np.where(sd < 0.05, "STUCK", np.where(near, "NEAR RAIL", ""))
    flag = np.where((lo <= 0) | (hi >= cfg.adc_max), "RAILED", flag)
    say("sensors  %.1f frames/s\n  sensor    mean    std    min    max" % hz)
    for i, row in enumerate(zip(C.mean(0), sd, lo, hi, flag)):
        say(("  a%d     %7.1f %6.2f %6.0f %6.0f  %s" % ((i,) + row)).rstrip())
    bad = flag != ""
    hint = ", ".join("a%d %s" % (i, flag[i].lower()) for i in np.flatnonzero(bad))
    if bad.any():
        hint += ": check that OSEM's lead, LED and flag position"
    if hz < 1.5 * cfg.control_hz:
        hint = "only %.0f frames/s. " % hz + hint
    return (not hint, hint, bad)


def drain(board, seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        board.read()


def link(board, rig, s0, say):
    drain(board, 0.2)
    lat = []
    for _ in range(50):
        ok, t = (board.cmd_ok, time.perf_counter())
        board.write(rig.bias)
        while board.cmd_ok == ok and time.perf_counter() - t < 0.2:
            board.read()
        lat.append(time.perf_counter() - t)
    drain(board, 0.2)
    med, p95 = np.percentile(lat, [50, 95]) * 1000.0
    sent = board.sent - s0
    say("link\n  frames %d  lost %d  bad %d" % (board.frames, board.lost, board.bad))
    say("  commands sent %d  accepted %d  rejected %d" % (sent, board.cmd_ok, board.cmd_bad))
    say("  round trip median %.1f ms  p95 %.1f ms" % (med, p95))
    ok = not (board.lost or board.bad or board.cmd_bad) and sent - board.cmd_ok <= 2
    return (ok, "" if ok else "frames or commands dropped: USB cable, hub, or a busy host")


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


def coils(hold, rig, which, use, win, say):
    say("coils  +-%.2f V, %.2f s windows" % (STEP, win))
    say("  coil dac sensor  expected  measured     +-  verdict   (counts/V)")
    out = {}
    for j in which:
        lv = []
        for s in (-1, 1, -1):
            v = rig.bias.copy()
            v[j] += s * STEP
            lv.append(hold(v, win, ramp_s=1.0)[1].mean(0))
        m = (lv[1] - 0.5 * (lv[0] + lv[2])) / (2 * STEP)
        sg = np.maximum(np.abs(lv[0] - lv[2]) / 2, FLOOR) / (2 * STEP)
        e = rig.dc[:, j]
        k = int(np.argmax(np.abs(e) * use))
        out[j] = verdict(m, sg, e, k, use)
        row = (j, rig.dac_map[j], k, e[k], m[k], sg[k], out[j])
        say("  %4d %3d     a%d  %+8.1f  %+8.1f %6.1f  %s" % row)
    hold(rig.bias, 0.0, ramp_s=1.0)
    return out


def coil_hints(rig, v):
    judged = [j for j in v if v[j] != "SKIP"]
    skip = ["coils %s not judged: too weak on the sensors used" % sorted(set(v) - set(judged))]
    if len(judged) > 1 and all(v[j] == "DEAD" for j in judged):
        return ALL_DEAD
    bad = ["coil %d %s: " % (j, v[j]) + HINT[v[j]] % rig.dac_map[j] for j in judged if v[j] != "OK"]
    return "; ".join(bad + skip * (len(judged) < len(v)))


def checks(board, rig, cfg, R, which=None, fast=False, say=print):
    R.append(("commands",) + commands(board, rig, say))
    board.start(rig.bias)
    s0 = board.sent
    hold = Hold(board, cfg, rig.n)
    hold.v = rig.bias.copy()
    T, C = hold(rig.bias, 5.0, ramp_s=0.0)
    if len(T) < 2:
        return R.append(("sensors", False, "no sample frames: firmware stream or USB"))
    ok, hint, bad = sensors(T, C, cfg, say)
    R.append(("sensors", ok, hint))
    R.append(("link",) + link(board, rig, s0, say))
    if not board.cmd_ok:
        return R.append(("coils", False, "skipped: the board accepted no command frame"))
    use = ~bad & (np.arange(rig.n) >= (4 if fast else 0))
    if not use.any():
        return R.append(("coils", False, "skipped: no usable sensor"))
    if fast:
        say("--fast: 1.5 s windows do not average the plate's ring out; judging on a4..a7 only")
    v = coils(hold, rig, range(rig.n) if which is None else which, use, 1.5 if fast else 4.05, say)
    R.append(
        ("coils", set(v.values()) <= {"OK", "SKIP"} and "OK" in v.values(), coil_hints(rig, v))
    )


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description="walk port -> coils once; never closes a loop")
    ap.add_argument("--port")
    ap.add_argument("--coils", type=lambda s: [int(x) for x in s.split(",")])
    ap.add_argument("--fast", action="store_true")
    a = ap.parse_args()
    rig, cfg = (osem.Rig.load(), osem.Config())
    if (rig.bias - STEP < 0).any() or rig.bias.sum() + STEP > cfg.sum_v_max:
        sys.exit("bias +- %.2f V does not fit under %.2f V summed" % (STEP, cfg.sum_v_max))
    if any(not 0 <= j < rig.n for j in a.coils or ()):
        sys.exit("--coils are numbered 0..%d" % (rig.n - 1))
    keep_awake()
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    R, board = ([], None)
    try:
        port = a.port or osem.Board.find_port()
        R.append(("port", True, port))
        board = osem.Board(rig, cfg, port, say=lambda *_: None)
        R.append(("firmware", True, "READY at 115200, fw=2"))
    except (SystemExit, OSError) as e:
        hint = "old firmware: run.py flash" if "ERR unknown" in str(e) else str(e)
        R.append((("port", "firmware")[len(R)], False, hint))
    if board:
        try:
            checks(board, rig, cfg, R, a.coils, a.fast)
        except KeyboardInterrupt:
            R.append(("interrupted", False, "coils parked at bias"))
        finally:
            board.park(rig.bias)
            board.close()
    print("summary")
    for name, ok, hint in R:
        tag = "PASS" if ok else "FAIL"
        print("  %s %-9s %s" % (c(tag, STATE[tag]), name, hint))
    sys.exit(not all(ok for _, ok, _ in R))


if __name__ == "__main__":
    main()
