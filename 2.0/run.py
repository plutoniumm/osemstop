#!/usr/bin/env python3
"""run -- the one entry point.

    run.py preflight                  every check that needs no port. READ IT.
    run.py sim [--seconds 90]         the loop against the synthetic plate
    run.py replay FILE                the estimator, open loop, on a recorded CSV
    run.py identify CENSUS DC         rebuild rig.json from two 1.0 records
    run.py bench [--port P]           ON HARDWARE
    run.py measure [--bias 0.25] [DC] ON HARDWARE: the DC step pass on all eight coils,
                                      then a 300 s passive census, then identify.
                                      Give an existing DC record to redo the census only.
    run.py link [--seconds 30]        ON HARDWARE: hold bias, exercise the link both
                                      ways, report rate / loss / noise. No control.
    run.py flash [--port P]           compile firmware/ and upload it

Options that change the law: --no-modal (PID only), --tilt X (arm the budget ramp).
"""

import argparse
import os
import signal
import sys
from dataclasses import replace
from datetime import datetime

import control
import rig as rigmod
import stdlib as sl
import transport

DATA = os.path.join(os.path.dirname(rigmod.HERE), "data")


def loop(tr, ctl, rec=None, say=print):
    """Samples in, volts out, until the transport ends or somebody stops it.

    PARK FIRST and let nothing before it fail: each teardown step is isolated,
    because one raise ahead of the park leaves every coil energised.
    """
    console = sl.Console(ctl.cfg.status_period_s)
    try:
        while True:
            s = tr.read()
            if s is None:
                continue
            t, counts = s
            out = ctl.step(counts, t)
            if out is not None:
                tr.write(out)
            for msg in ctl.drain_events():
                say(msg)
            if console.due(t):
                say(ctl.status_line(t))
            if rec is not None:
                rec.write(ctl.csv_row(t, counts))       # every raw sample
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        for what, fn in (("park the coils", lambda: tr.park(ctl.bias)),
                         ("close the log", rec.close if rec else lambda: None),
                         ("close the transport", tr.close)):
            try:
                fn()
            except BaseException as e:                   # incl. a second Ctrl+C
                say("!! could not %s: %r -- carrying on with the shutdown" % (what, e))
    say("\n%d control steps, wire %.0f Hz, %d fault(s), peak demand %.3f V of a "
        "%.3f V budget, peak total %.2f V of %.2f (capped on %d steps), lock %s"
        % (ctl.dec.steps, ctl.dec.wire_hz, ctl.fault_count, ctl.peak_demand,
           ctl.cfg.budget_v, ctl.peak_sum, ctl.cfg.sum_v_max, ctl.sum_limited,
           "never" if not ctl.locked_announced else "announced"))
    if ctl.mkf is not None:
        say("chi2/dof (logged, acted on by nothing):")
        for st in control.STATES:
            say(ctl.chi2_log[st].line(st))
    return ctl


def link(rig, cfg, port, seconds):
    """Hold every coil at bias and exercise the link exactly as a run would:
    one command frame per control step, samples streaming. Nothing moves."""
    import time
    import numpy as np
    tr = transport.Serial(rig, cfg, port)
    T, C = [], []
    try:
        tr.start(rig.bias)
        nxt = 0.0
        while True:
            s = tr.read()
            t = time.perf_counter() - tr.t0
            if t >= nxt:
                tr.write(rig.bias)
                nxt += 1.0 / cfg.control_hz
            if s is not None:
                T.append(s[0]), C.append(s[1])
            if t > seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        tr.park(rig.bias)
        tr.close()
    T, C = np.array(T), np.array(C)
    os.makedirs(DATA, exist_ok=True)
    path = os.path.join(DATA, datetime.now().strftime("%Y%m%d_%H%M%S") + "_v2_link.csv")
    np.savetxt(path, np.column_stack([T, C]), delimiter=",", fmt="%.5f",
               header="t," + ",".join("a%d" % i for i in range(rig.n)), comments="")
    fs = (len(T) - 1) / (T[-1] - T[0])
    Y = np.fft.rfft(C - C.mean(0), axis=0)
    Y[np.fft.rfftfreq(len(C), 1.0 / fs) < 5.0] = 0
    hf = np.fft.irfft(Y, n=len(C), axis=0).std(0)
    mid = 0.5 * (C[:-2] + C[2:])
    print("%.1f frames/s (%.1f ADC scans/s), largest gap %.1f ms"
          % (fs, fs * tr.os, 1e3 * np.diff(T).max()))
    print("rest  " + " ".join("%7.1f" % x for x in C.mean(0)))
    print("std   " + " ".join("%7.2f" % x for x in C.std(0)))
    print(">5 Hz " + " ".join("%7.2f" % x for x in hf) + "   counts rms")
    print("%d samples jump >150 counts from their neighbours; raw data in %s"
          % (int((np.abs(C[1:-1] - mid) > 150).sum()), path))
    return 0


def measure(rig, cfg, args, step=0.15, dwell=12.0, census_s=300.0):
    """The two records `identify` needs, taken in ONE session at the bias the loop
    will run at, over firmware 2, then identify. One coil at a time is ramped to
    bias + step, held, ramped to bias - step, held, and returned. Ramped, not
    stepped: the ring a step leaves is the noise in this measurement."""
    import time
    import numpy as np
    from dataclasses import replace as _replace
    n, bias = rig.n, np.full(rig.n, args.bias)
    if bias.sum() + step > cfg.sum_v_max or args.bias - step < 0:
        sys.exit("bias %.2f V +- %.2f V does not fit the window" % (args.bias, step))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(DATA, exist_ok=True)
    paths = [os.path.join(DATA, stamp + "_v2_%s.csv" % k) for k in ("dc", "census")]
    head = "time_s,seg,phase,coil,freq_hz,amp_v,u_v," + ",".join(
        "a%d" % i for i in range(n)) + "\n"
    tr = transport.Serial(rig, cfg, args.port)
    state = dict(v=np.zeros(n), nxt=0.0, seg=0)

    def hold(target, seconds, fh, phase, coil, ramp_s=2.0):
        v0, t0 = state["v"].copy(), time.perf_counter() - tr.t0
        state["seg"] += 1
        while True:
            t = time.perf_counter() - tr.t0
            if t >= state["nxt"]:
                state["v"] = v0 + (target - v0) * min((t - t0) / max(ramp_s, 1e-6), 1.0)
                tr.write(state["v"])
                state["nxt"] = t + 1.0 / cfg.control_hz
            s = tr.read()
            if s is not None and fh is not None:
                settled = s[0] - t0 > ramp_s + 1.0     # only the hold carries the tag
                fh.write("%.5f,%d,%s,%d,-1,0,%.4f,%s\n" % (
                    s[0], state["seg"], phase if settled else "ramp",
                    coil if settled else -1, target[coil] if coil >= 0 else args.bias,
                    ",".join("%.2f" % x for x in s[1])))
            if t - t0 > ramp_s + 1.0 + seconds:
                return

    try:
        tr.start(np.zeros(n))
        if args.files:                                  # reuse a DC pass already taken
            paths[0] = args.files[0]
            hold(bias, 8.0, None, "passive", -1, ramp_s=5.0)
        with open(os.devnull if args.files else paths[0], "w", buffering=1) as fh:
            fh.write(head)
            hold(bias, 8.0, fh, "passive", -1, ramp_s=5.0)
            for j in ([] if args.files else range(n)):
                for sign in (+1, -1):
                    v = bias.copy()
                    v[j] += sign * step
                    hold(v, dwell, fh, "dc", j)
                hold(bias, 2.0, fh, "passive", -1)
                print("  coil %d stepped" % j)
        print("DC pass done -> %s\ncensus, %.0f s, nothing driven..." % (paths[0], census_s))
        with open(paths[1], "w", buffering=1) as fh:
            fh.write(head)
            hold(bias, 20.0, None, "passive", -1)       # let the last ramp settle
            hold(bias, census_s, fh, "passive", -1, ramp_s=0.0)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        tr.park(bias)
        tr.close()
    prior = _replace(rig, bias=bias, source=dict(
        rig.source, bias="%.2f V uniform (the supply folds back near %.1f V summed)"
        % (args.bias, cfg.sum_v_limit)))
    new = rigmod.identify(paths[1], paths[0], prior,
                          volts_per_count=cfg.vcc / cfg.adc_max,
                          control_hz=cfg.control_hz, t_amp_s=cfg.t_amp_s,
                          t_dc_s=cfg.t_dc_s)
    new.save(args.rig)
    print("\nwrote %s" % args.rig)
    return 0 if show_preflight(new, cfg) else 1


def show_preflight(rig, cfg, waive_age=False):
    ok = True
    for name, good, detail in control.preflight(rig, cfg):
        waived = waive_age and not good and name.startswith("rig.json is fresh")
        print("  %-6s %s%s" % ("PASS" if good else "WAIVED" if waived else "FAIL",
                               name, "   " + detail if detail else ""))
        ok &= good or waived
    return ok


def main(argv=None):
    sys.stdout.reconfigure(line_buffering=True)          # logs get piped
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("cmd", choices="preflight sim replay identify bench link "
                                   "flash measure".split())
    ap.add_argument("--bias", type=float, default=0.25)
    ap.add_argument("files", nargs="*")
    ap.add_argument("--rig", default=rigmod.PATH)
    ap.add_argument("--port", default=os.environ.get("PORT"))
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--no-modal", action="store_true")
    ap.add_argument("--tilt", type=float, default=0.0)
    ap.add_argument("--stale-ok", action="store_true",
                    help="bench: run on a rig.json past its age limit")
    args = ap.parse_args(argv)

    if args.cmd == "flash":
        import subprocess
        sketch = os.path.join(rigmod.HERE, "firmware")
        port = args.port or transport.Serial.find_port()
        for step in (["compile", "--fqbn", "arduino:avr:mega", sketch],
                     ["upload", "--fqbn", "arduino:avr:mega", "-p", port, sketch]):
            if subprocess.run(["arduino-cli"] + step).returncode:
                sys.exit("arduino-cli %s failed" % step[0])
        print("flashed. setup() zeroed all eight coils. Now `run.py link`.")
        return 0

    rig = rigmod.Rig.load(args.rig)
    cfg = replace(control.Config(), modal=not args.no_modal, budget_tilt=args.tilt)

    if args.cmd in ("bench", "link", "measure") and sys.platform == "darwin":
        # A sleeping Mac freezes this process with the coils held at whatever was
        # last commanded, and the board streams into the void. It happened on
        # 2026-10-05: a 300 s census took 50 minutes and came out in pieces.
        import subprocess
        subprocess.Popen(["caffeinate", "-dimsu", "-w", str(os.getpid())])

    if args.cmd == "identify":
        if len(args.files) != 2:
            ap.error("identify CENSUS.csv DC_PASS.csv")
        new = rigmod.identify(args.files[0], args.files[1], rig,
                              volts_per_count=cfg.vcc / cfg.adc_max,
                              control_hz=cfg.control_hz, t_amp_s=cfg.t_amp_s,
                              t_dc_s=cfg.t_dc_s)
        new.save(args.rig)
        print("\nwrote %s -- now run `run.py preflight`" % args.rig)
        return 0
    if args.cmd == "preflight":
        return 0 if show_preflight(rig, cfg) else 1

    if args.cmd == "link":
        return link(rig, cfg, args.port, args.seconds)
    if args.cmd == "measure":
        return measure(rig, cfg, args)

    ctl = control.Controller(rig, cfg)
    print("\n".join(ctl.banner()))
    if args.cmd == "sim":
        loop(transport.Sim(rig, args.seconds), ctl)
        return 0
    if args.cmd == "replay":
        if len(args.files) != 1:
            ap.error("replay FILE.csv")
        loop(transport.Replay(args.files[0], rig.n), ctl)
        return 0

    # bench
    if not show_preflight(rig, cfg, waive_age=args.stale_ok):
        sys.exit("\npreflight FAILED -- not opening the port.")
    for name in ("SIGTERM", "SIGHUP"):                   # a plain `kill` must park
        def _term(signum, frame):
            raise KeyboardInterrupt
        signal.signal(getattr(signal, name), _term)
    tr = transport.Serial(rig, cfg, args.port)
    tr.park(ctl.bias)
    if sys.stdin.isatty():
        input("Coils at bias. Enter to start, Ctrl+C to stop... ")
    os.makedirs(DATA, exist_ok=True)
    path = os.path.join(DATA, datetime.now().strftime("%Y%m%d_%H%M%S") + "_v2.csv")
    rec = sl.Recorder(path, ctl.csv_cols())
    print("logging every raw sample to %s\nREAD THE FIRST STATUS LINE." % path)
    tr.start(ctl.bias)
    loop(tr, ctl, rec)
    return 0


if __name__ == "__main__":
    sys.exit(main())
