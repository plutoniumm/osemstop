#!/usr/bin/env python3
"""staircase -- how much bias can the supply and the front end take? ON HARDWARE.

    python staircase.py [--top 0.90] [--step 0.05] [--dwell 15] [--port P]
    python staircase.py --coil 3 --bottom 0.0     # ONE coil's transfer curve, the
                                                  # other seven held at 0.50 V

Every coil is ramped to 0.50 V, held, then raised together one step at a time
with NO feedback. After each dwell the sensors are compared with what the DC
matrix says a bias change alone should do; what is left over is the supply or
the front end. It stops and ramps down to 0.25 V at the first of:

  * a sensor reading under 20 counts, or all of them frozen   (front end gone;
    1.10 V on all eight did this on 2026-08-20 and it did not come back)
  * the DC levels falling TOGETHER beyond the prediction      (supply sagging;
    -185..-286 counts on every channel when it alarmed at 0.75 V + 0.75 V swing)
  * noise shared by all eight sensors tripling                (same event: 15 -> 132)
  * a sensor on a rail for over 5 % of a dwell                (nothing left to read)

It is a test of the hardware, not of the plate: bias moves are ramped over 2 s so
they ring it as little as possible. Every frame goes to disk.
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np

import control
import rig as rigmod
import transport


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--top", type=float, default=0.90)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--dwell", type=float, default=15.0)
    ap.add_argument("--bottom", type=float, default=0.50)
    ap.add_argument("--coil", type=int, default=None,
                    help="step this coil only; the rest hold --rest")
    ap.add_argument("--rest", type=float, default=0.50,
                    help="what the other coils hold during a --coil sweep")
    ap.add_argument("--port", default=os.environ.get("PORT"))
    a = ap.parse_args()
    if a.top > 1.0:
        sys.exit("--top above 1.0 V: 1.10 V on all eight took the front end down.")
    sys.stdout.reconfigure(line_buffering=True)
    rig, cfg = rigmod.Rig.load(), control.Config()
    n, hz = rig.n, cfg.control_hz
    levels = np.round(np.arange(a.bottom, a.top + 1e-9, a.step), 3)
    who = slice(None) if a.coil is None else a.coil

    def volts(v):
        out = np.full(n, a.rest)
        out[who] = v
        return out

    tr = transport.Serial(rig, cfg, a.port)
    path = os.path.join(os.path.dirname(rigmod.HERE), "data",
                        datetime.now().strftime("%Y%m%d_%H%M%S") + "_v2_staircase.csv")
    log = open(path, "w", buffering=1)
    log.write("t,bias," + ",".join("a%d" % i for i in range(n)) + "\n")
    state = dict(v=np.zeros(n), next=0.0)

    def hold(target, seconds, ramp_s=2.0):
        """Ramp every coil to `target`, hold; return (t, counts) of the hold."""
        v0, t_start, T, C = state["v"].copy(), time.perf_counter() - tr.t0, [], []
        target = np.asarray(target, float)
        while True:
            t = time.perf_counter() - tr.t0
            if t >= state["next"]:
                state["v"] = v0 + (target - v0) * min((t - t_start) / ramp_s, 1.0)
                tr.write(state["v"])
                state["next"] = t + 1.0 / hz
            s = tr.read()
            if s is not None:
                log.write("%.5f,%.4f,%s\n" % (s[0], state["v"][who if a.coil is not None else 0],
                                              ",".join("%.2f" % x for x in s[1])))
                if s[0] - t_start > ramp_s + 1.0:
                    T.append(s[0]), C.append(s[1])
                    if len(C) % 200 == 0:                # fast check, ~1 s
                        last = np.array(C[-200:])
                        if (last.mean(0) < 20).any() or (last.std(0) < 0.02).all():
                            return np.array(T), np.array(C), "FRONT END: a sensor " \
                                "under 20 counts or all frozen"
            if t - t_start > ramp_s + 1.0 + seconds:
                return np.array(T), np.array(C), None

    def stats(T, C):
        fs = (len(T) - 1) / (T[-1] - T[0])
        G = [np.ones(len(T))]
        for f in rig.f_hz:                               # fit the ring out of the level
            G += [np.cos(2 * np.pi * f * T), np.sin(2 * np.pi * f * T)]
        level = np.linalg.lstsq(np.column_stack(G), C, rcond=None)[0][0]
        Y = np.fft.rfft(C - C.mean(0), axis=0)
        Y[np.fft.rfftfreq(len(C), 1.0 / fs) < 5.0] = 0
        hf = np.fft.irfft(Y, n=len(C), axis=0)
        rail = ((C >= cfg.rail_hi) | (C <= cfg.rail_lo)).mean(0)
        return level, hf.std(0), float(hf.mean(1).std()), rail

    end, why = 0.25, None                 # 0.25 V on all eight ran for months
    try:
        tr.start(np.zeros(n))
        print("%s: %.2f -> %.2f V in %d steps, %.0f s each\nraw data: %s"
              % ("all eight coils" if a.coil is None else
                 "coil %d, the rest at %.2f V" % (a.coil, a.rest), levels[0], levels[-1],
                 len(levels) - 1, a.dwell, path))
        print(" bias   sumV^2 |  level, counts a0..a7                      |"
              "  beyond prediction, counts a0..a7          | shared noise | >5 Hz worst")
        base = None
        for v in levels:
            T, C, why = hold(volts(v), a.dwell if base is not None else a.dwell + 5.0,
                             ramp_s=5.0 if base is None else 2.0)
            if why is None:
                level, hf, cm, rail = stats(T, C)
                if base is None:
                    base = (level, cm)
                extra = level - base[0] - rig.dc @ (volts(v) - volts(levels[0]))
                print(" %.2f   %5.2f  | %s | %s |   %5.2f      | %5.2f"
                      % (v, float((volts(v) ** 2).sum()),
                         " ".join("%5.0f" % x for x in level),
                         " ".join("%+5.0f" % x for x in extra), cm, hf.max()))
                if (extra < 0).sum() >= n - 1 and np.median(extra) < -15.0:
                    why = "SUPPLY: DC levels fell together, median %+.0f counts " \
                          "beyond what the bias change explains" % np.median(extra)
                elif cm > 3.0 * base[1] and cm > 3.0:
                    why = "SUPPLY: shared noise %.1f -> %.1f counts" % (base[1], cm)
                elif (rail > 0.05).any():
                    why = "RAIL: a%d on a rail %.0f %% of the dwell" \
                          % (int(rail.argmax()), 100 * rail.max())
            if why:
                end = 0.25
                print("\n!! STOPPED at %.2f V -- %s. Ramping down to 0.25 V." % (v, why))
                break
        else:
            print("\nreached %.2f V with none of the stop conditions." % levels[-1])
    except KeyboardInterrupt:
        end, why = 0.25, "interrupted"
        print("\ninterrupted -- ramping down to 0.25 V.")
    finally:
        try:
            hold(np.full(n, end), 2.0, ramp_s=3.0)
            print("coils left at %.2f V." % end)
        finally:
            log.close()
            tr.close()


if __name__ == "__main__":
    main()
