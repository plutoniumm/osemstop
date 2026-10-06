#!/usr/bin/env python3
"""
run.py measure            ON HARDWARE  sign test + passive record -> rig.json -> preflight
run.py auto [--hours H]   ON HARDWARE  measure, then run until Ctrl+C (or H hours), light log
run.py bench [--kicks 3]  ON HARDWARE  close the loop; --kicks says "jerk it" after lock
run.py link               ON HARDWARE  hold bias, check the link
run.py stairs             ON HARDWARE  raise the bias in steps, no feedback
run.py flash              ON HARDWARE  compile and upload firmware/
run.py preflight          every check that needs no port
run.py sim                the loop against a synthetic plate
run.py replay FILE        the estimator on a recorded CSV
"""

import argparse
from dataclasses import replace
import signal
import sys
import osem
from osem import procedures
from osem.session import keep_awake, stamp


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument(
        "cmd", choices="auto measure bench link stairs flash preflight sim replay".split()
    )
    ap.add_argument("file", nargs="?")
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--bias", type=float, default=0.3)
    ap.add_argument("--kicks", type=int, default=0)
    ap.add_argument("--every", type=float, default=25.0)
    ap.add_argument("--pid-only", action="store_true")
    ap.add_argument("--hours", type=float)
    ap.add_argument("--gain", type=float)
    ap.add_argument("--pulse", action="store_true")
    ap.add_argument("--coil", type=int)
    ap.add_argument("--top", type=float, default=0.5)
    ap.add_argument("--stale-ok", action="store_true")
    a = ap.parse_args()
    if a.cmd == "flash":
        return procedures.flash(a.port)
    rig, cfg = osem.Rig.load(), replace(
        osem.Config(), **({"modal_scale": a.gain} if a.gain else {})
    )
    if a.cmd == "preflight":
        return sys.exit(not osem.show_preflight(rig, cfg))
    if a.cmd == "auto":
        if not procedures.measure(rig, cfg, a.port, bias=a.bias):
            sys.exit("preflight failed after measuring; not closing the loop.")
        rig, cfg = osem.Rig.load(), replace(cfg, status_period_s=60.0)
    if a.cmd == "measure":
        return sys.exit(not procedures.measure(rig, cfg, a.port, bias=a.bias, dc=a.file))
    if a.cmd == "link":
        return procedures.link(rig, cfg, a.port, a.seconds)
    if a.cmd == "stairs":
        return procedures.staircase(
            rig,
            cfg,
            a.port,
            coil=a.coil,
            top=a.top,
            rest=a.bias,
            bottom=0.0 if a.coil is not None else a.bias,
        )
    regulators = [osem.Pid(rig, cfg)] if a.pid_only else None
    ctl = osem.Controller(rig, cfg, regulators)
    print("\n".join(ctl.banner()))
    if a.cmd == "sim":
        return osem.run(osem.Sim(rig, a.seconds), ctl)
    if a.cmd == "replay":
        return osem.run(osem.Replay(a.file, rig.n), ctl)
    if not osem.show_preflight(rig, cfg, waive_age=a.stale_ok):
        sys.exit("preflight failed; not opening the port.")
    keep_awake()
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    board = osem.Board(rig, cfg, a.port)
    log = stamp("")
    print("logging to %s" % log)
    board.start(ctl.bias)
    pulse = (0, 0.15, 0.5) if a.pulse else None
    long = a.cmd == "auto"
    osem.run(
        board,
        ctl,
        log,
        kicks=a.kicks,
        every=a.every,
        pulse=pulse,
        log_hz=10 if long else None,
        until=a.hours * 3600 if a.hours else None,
    )


if __name__ == "__main__":
    main()
