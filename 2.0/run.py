#!/usr/bin/env python3
"""
run.py calib [dc.csv]     step each coil, listen to the free plate -> rig.json, then the checks
run.py bench [--kicks 3]  close the loop; --kicks says "jerk it" after lock, --pulse kicks with coil 0
"""

import argparse
import sys
import osem
from osem.session import guard, measure, say, stamp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("cmd", choices="calib bench".split())
    ap.add_argument("file", nargs="?")
    ap.add_argument("--port")
    ap.add_argument("--bias", type=float, default=0.3)
    ap.add_argument("--kicks", type=int, default=0)
    ap.add_argument("--every", type=float, default=25.0)
    ap.add_argument("--hours", type=float)
    ap.add_argument("--pulse", type=float, nargs="?", const=0.15)
    ap.add_argument("--stale-ok", action="store_true")
    a = ap.parse_args()
    rig, cfg = (osem.Rig.load(), osem.Config())
    guard()
    if a.cmd == "calib":
        sys.exit(not measure(rig, cfg, a.port, a.bias, a.file))
    ctl = osem.Controller(rig, cfg)
    say(ctl.banner())
    for note in rig.notes:
        say("note    " + note, "yellow")
    if not osem.show_preflight(rig, cfg, waive_age=a.stale_ok):
        sys.exit("preflight failed; not opening the port.")
    board = osem.Board(rig, cfg, a.port)
    log = stamp("")
    say("log     %s" % log)
    board.start(ctl.bias)
    osem.run(
        board,
        ctl,
        log,
        kicks=a.kicks,
        every=a.every,
        pulse=a.pulse or None,
        log_hz=10 if a.hours else None,
        until=a.hours * 3600 if a.hours else None,
    )


if __name__ == "__main__":
    main()
