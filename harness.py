#!/usr/bin/env python3
"""
harness.py -- the one entry point for running a controller off the bench.
=========================================================================
Every `osem.vN.py` in this directory is a complete, standalone controller that
also runs on the hardware. This file runs one of them against the simulated
suspension in sim/, either interactively in a browser or as a behavioural test
suite. It does not reimplement any part of the control law: it imports the
version you name and steps that module's own `Controller`.

    python3 harness.py                     # serve the newest version
    python3 harness.py v0                  # serve a specific one
    python3 harness.py v3 --port 8771
    python3 harness.py --list              # what versions exist
    python3 harness.py --test              # test the newest version
    python3 harness.py --test v0
    python3 harness.py --test all          # the whole ladder, one after another

Version contract. To be runnable here a controller must expose:
    ENABLE_CHANNEL, STEADY_GAIN, CAPTURE_GAIN, KI_GAIN, KD_GAIN,
    BIAS, DAC_CHANNELS, ADC_MAX_COUNTS, A_VCC, ENVELOPE_WINDOW_S,
    SlidingRMS, Controller(dac, enable=, steady=, capture=, ki=, kd=)
and `Controller` must offer .step(counts, volts, t, dt) -> state,
.drain_events(), .channels, .state, .lock_time, .fault_count.
Optionally VERSION_TAG and FIXES; absent means "v0-era, nothing fixed".

The suite below derives what to assert from the loaded module -- how many
channels are enabled, whether I/D are live, which defects it declares fixed --
so one set of checks covers the whole ladder and a version that claims a fix
has to actually demonstrate it.
"""

import argparse
import contextlib
import glob
import io
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim import server as S           # noqa: E402


# --------------------------------------------------------------------------
# version discovery
# --------------------------------------------------------------------------
def versions():
    """Every osem.vN.py here, ordered by N."""
    found = []
    for path in glob.glob(os.path.join(HERE, "osem.v*.py")):
        m = re.match(r"osem\.v(\d+)$", os.path.basename(path)[:-3])
        if m:
            found.append((int(m.group(1)), path))
    return [p for _, p in sorted(found)]


def resolve(name):
    """'v2', 'osem.v2', 'osem.v2.py' or a path -> an absolute path."""
    all_versions = versions()
    if not all_versions:
        sys.exit("No osem.v*.py found next to harness.py.")
    if name is None:
        return all_versions[-1]
    if os.path.isfile(name):
        return os.path.abspath(name)
    stem = name if name.startswith("osem.") else "osem." + name
    cand = os.path.join(HERE, stem if stem.endswith(".py") else stem + ".py")
    if os.path.isfile(cand):
        return cand
    sys.exit(f"No such version: {name}. Have: "
             + ", ".join(os.path.basename(p)[5:-3] for p in all_versions))


def describe(path):
    osem = S.load(path)
    return dict(
        path=path,
        name=os.path.basename(path)[:-3],
        tag=getattr(osem, "VERSION_TAG", "v0"),
        fixes=tuple(getattr(osem, "FIXES", ())),
        n_enabled=sum(bool(v) for v in osem.ENABLE_CHANNEL),
        ki=[float(v) for v in osem.KI_GAIN],
        kd=[float(v) for v in osem.KD_GAIN],
        steady=[float(v) for v in osem.STEADY_GAIN],
    )


# --------------------------------------------------------------------------
# behavioural suite
# --------------------------------------------------------------------------
_pass = _fail = 0

# --- output and pacing hooks -----------------------------------------------
# test/tui.py drives this same suite; rather than fork the checks it installs a
# REPORTER to receive them and a MONITOR to sample the plant while a run is in
# flight. Left as None, everything below prints and runs flat out, which is what
# `harness.py --test` wants.
REPORTER = None      # fn(kind, **fields) for "check" | "note" | "phase"
MONITOR = None       # fn(label, sim) every MONITOR_EVERY simulated samples
MONITOR_EVERY = 25
PENDING_KICKS = []   # dv values pushed by another thread, applied in-loop
KICKS_APPLIED = 0    # count of kicks that actually reached the plant


def _emit(kind, **fields):
    if REPORTER:
        REPORTER(kind, **fields)
        return False
    return True


def check(name, ok, detail=""):
    global _pass, _fail
    if ok:
        _pass += 1
    else:
        _fail += 1
    if _emit("check", name=name, ok=ok, detail=detail):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""))


def note(text):
    if _emit("note", text=text):
        print(text)


def phase(label):
    if _emit("phase", label=label):
        print(label)


BASE_PLANT = dict(f_drive=1.0, drive_amp=0.8, seismic=0.8, Q=50.0, k_act=20.0,
                  meas_noise=20.0, hum_hz=60.0, hum_mv=0.0, hvac_hz=2.5,
                  hvac_amp=0.0, shock_rate=0.0, shock_amp=3.0)


def run(label, seconds, enable, steady, ki=None, kd=None, occlude_at=None,
        auto_kick=False, plant=None, quiet=True):
    """Step the loaded controller through the simulated plant. Returns the
    observations the checks below read."""
    sim = S.SIM
    sim.enable = [bool(v) for v in enable]
    sim.steady = list(steady)
    sim.capture = [g * 1.167 for g in steady]
    sim.ki = list(ki or [0.0] * 4)
    sim.kd = list(kd or [0.0] * 4)
    sim.occlude = [False] * 4
    sim.auto_kick = auto_kick
    sim.plant.update(BASE_PLANT)
    if plant:
        sim.plant.update(plant)
    sim.reset()
    # Anything queued between scenarios was aimed at the previous one. Landing
    # it here would drop a kick into this scenario's calibration window, which
    # is both wrong and confusing to watch, so drop it.
    PENDING_KICKS.clear()

    faults = recovered = 0
    rail_seen = frozen_on_rail = False
    prev = sim.ctl.state
    peak = [0.0] * 4
    t_rail = None
    # Capture the PID terms DURING the run. `sim` is a singleton and reset()
    # replaces sim.ctl, so reading them off sim afterwards reports whatever the
    # most recent run left behind -- which, if that run ended mid-FAULT, is a
    # correctly-zeroed integrator, and the check would fail a healthy version.
    max_p = max_i = max_d = 0.0
    # v0's check_rail prints on every railed sample with no latch, which floods
    # the console during the occlusion sweep. The rail_fault flag is what the
    # assertions read, so swallow it.
    sink = io.StringIO()
    for n in range(int(seconds * S.SAMPLE_HZ)):
        if occlude_at is not None and sim.t >= occlude_at[1]:
            sim.occlude[occlude_at[0]] = True
        # Kicks arrive from the TUI's input thread; apply them here so nothing
        # touches the plant's state concurrently with step().
        while PENDING_KICKS:
            globals()["KICKS_APPLIED"] += 1
            sim.kick(PENDING_KICKS.pop(0))
        with contextlib.redirect_stdout(sink):
            sim.step()
        if MONITOR is not None and n % MONITOR_EVERY == 0:
            MONITOR(label, sim)
        st = sim.ctl.state
        if st != prev:
            if st == "FAULT":
                faults += 1
            if prev == "FAULT" and st == "CALIBRATING":
                recovered += 1
            prev = st
        if any(c.rail_fault for c in sim.ctl.channels):
            if not rail_seen and occlude_at:
                t_rail = sim.t - occlude_at[1]
            rail_seen = True
            if all(abs(float(c.out) - 0.25) < 1e-6 for c in sim.ctl.channels):
                frozen_on_rail = True
        if st == "DAMPING":
            for c in sim.ctl.channels:
                if c.enabled:
                    max_p = max(max_p, abs(float(c.p_term)))
                    max_i = max(max_i, abs(float(c.i_term)))
                    max_d = max(max_d, abs(float(c.d_term)))
        if sim.t > 12:
            for i in range(4):
                peak[i] = max(peak[i], sim.diag_ratio[i])

    ratio = list(sim.diag_ratio)
    if not quiet:
        print(f"\n=== {label} ===")
        print(f"  state={sim.ctl.state}  faults={faults}  recovered={recovered}  "
              f"lock={'never' if sim.ctl.lock_time is None else f'{sim.ctl.lock_time:.1f}s'}")
        print("  ratio  " + "  ".join(f"ch{i}={ratio[i]:.3f}" for i in range(4)))
    return dict(ratio=ratio, peak=peak, faults=faults, recovered=recovered,
                lock=sim.ctl.lock_time, rail=rail_seen, frozen=frozen_on_rail,
                t_rail=t_rail, sim=sim, max_p=max_p, max_i=max_i, max_d=max_d,
                state=sim.ctl.state,
                sat_flags=[bool(c.saturated_flag) for c in sim.ctl.channels],
                baseline=[float(c.baseline_rms or 0) for c in sim.ctl.channels])


def suite(path):
    """Run every check against one version. Expectations come from the module."""
    info = describe(path)
    fixed = set(info["fixes"])
    shipped = info["steady"]
    on = [i for i in range(4) if S.osem.ENABLE_CHANNEL[i]]
    off = [i for i in range(4) if not S.osem.ENABLE_CHANNEL[i]]
    ship_en = [bool(v) for v in S.osem.ENABLE_CHANNEL]

    phase(f"{'=' * 74}\n{info['name']}  ({info['tag']})   "
          f"channels on: {len(on)}/4   "
          f"I/D: {'live' if any(info['ki']) or any(info['kd']) else 'zeroed'}   "
          f"fixes: {', '.join(info['fixes']) or 'none'}\n{'=' * 74}")

    # ---- 1. the shipped configuration damps what it drives -----------------
    a = run("as shipped", 60, ship_en, shipped, ki=info["ki"], kd=info["kd"])
    check("every enabled channel damps below the lock threshold",
          all(a["ratio"][i] < 0.40 for i in on),
          " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in on))
    check("the run reports LOCKED", a["lock"] is not None,
          f"{a['lock']:.1f}s" if a["lock"] else "never")
    if off:
        check("disabled channels stay at their baseline",
              all(0.80 < a["ratio"][i] < 1.20 for i in off),
              " ".join(f"ch{i}={a['ratio'][i]:.2f}" for i in off))
    check("no faults in a quiet lab", a["faults"] == 0, f"{a['faults']}")

    # ---- 2. a wrong-signed gain is caught by the interlock ------------------
    # ch2 is inverted in hardware, so NEGATING its gain is the sign error here.
    bad = list(shipped)
    bad[2] = -abs(bad[2])
    b = run("ch2 gain sign inverted", 120, [True] * 4, bad,
            ki=info["ki"], kd=info["kd"])
    check("a wrong-signed channel is pumped past the runaway line",
          b["peak"][2] > 1.8, f"ch2 peak={b['peak'][2]:.2f}")
    check("the global interlock trips on it", b["faults"] > 0, f"{b['faults']} fault(s)")
    check("a runaway trip auto-recovers", b["recovered"] > 0,
          f"{b['recovered']} recovery(ies)")

    # ---- 3. I and D behave as the physics says ------------------------------
    if any(info["ki"]):
        check("the I term accumulates", a["max_i"] > 1e-4,
              f"max |I| = {a['max_i']:.4f} V over the run")
        check("the I term respects the anti-windup clamp",
              a["max_i"] <= S.osem.I_CLAMP_V + 1e-9,
              f"peak {a['max_i']:.4f} V vs clamp {S.osem.I_CLAMP_V} V")
        check("P still dominates -- neither I nor D takes over the loop",
              a["max_p"] > a["max_i"] and a["max_p"] > a["max_d"],
              f"P={a['max_p']:.4f} I={a['max_i']:.4f} D={a['max_d']:.4f} V")
    else:
        check("with KI_GAIN and KD_GAIN zeroed the law is pure P",
              a["max_i"] < 1e-12 and a["max_d"] < 1e-12,
              f"I={a['max_i']:.2e} D={a['max_d']:.2e}")

    # ---- 4. saturation trip: recovers only if the version fixed it ----------
    over = list(shipped)
    over[0] = -0.600
    d = run("over-gain ch0 + a kick", 90, [True, False, False, False], over,
            ki=info["ki"], kd=info["kd"], auto_kick=True)
    check("over-gain plus a disturbance trips a fault", d["faults"] > 0,
          f"{d['faults']} fault(s)")
    stuck = d["state"] == "FAULT" and not d["recovered"]
    if "saturation-latch" in fixed:
        check("FIXED saturation-latch: the trip clears and the loop recovers",
              not stuck, f"state={d['state']}, {d['recovered']} recovery(ies)")
        check("FIXED saturation-latch: saturated_flag is not left latched",
              not any(d["sat_flags"]), "refreshed every sample in actuate()")
    else:
        check("KNOWN BUG saturation-latch: FAULT is permanent", stuck,
              f"still {d['state']} 70 s after the trip")
        check("KNOWN BUG saturation-latch: a stale flag is what holds it there",
              any(d["sat_flags"]),
              "cleared only by check_runaway(), which FAULT never reaches")

    # ---- 5. rail interlock against the sensor noise floor -------------------
    note("\n  rail interlock: ch1 blinded at t=20s, sweeping measurement noise")
    note("  noise |  rail  frozen |  armed after")
    rows = []
    for nv in (0, 5, 10, 20):
        r = run(f"blind ch1 @ {nv} mV", 45, [True, False, False, False], shipped,
                ki=info["ki"], kd=info["kd"], occlude_at=(1, 20.0),
                plant={"meas_noise": nv})
        rows.append((nv, r))
        delay = "never" if r["t_rail"] is None else f"{r['t_rail']:.2f} s"
        note(f"  {nv:2d} mV |  {str(r['rail']):5s}  {str(r['frozen']):5s} |  {delay}")
    quiet_rows = [r for nv, r in rows if nv <= 5]
    loud_rows = [r for nv, r in rows if nv >= 10]
    check("the interlock arms on a blind sensor when the signal is quiet",
          all(r["rail"] and r["frozen"] for r in quiet_rows))
    check("blinding one OSEM freezes all four channels",
          all(r["frozen"] for r in quiet_rows))
    if "rail-threshold" in fixed:
        check("FIXED rail-threshold: it still arms at a realistic noise floor",
              all(r["rail"] and r["frozen"] for r in loud_rows),
              f"threshold {S.osem.RAIL_LOW_COUNTS} counts, "
              f"{S.osem.RAIL_FRACTION:.0%} of a {S.osem.RAIL_SUSTAIN_S}s window")
    else:
        check("KNOWN GAP rail-threshold: it stops arming as noise approaches "
              "the 3-count line", all(not r["rail"] for r in loud_rows),
              "not silently patched -- see versions.md")

    # ---- 6. baseline robustness against a shock during calibration ----------
    # 8 shocks/min lands exactly ONE inside the calibration window for every
    # version on the ladder, despite v3 calibrating for 20 s against v0's 8 s,
    # so this compares like with like. An isolated transient is the case the
    # median-of-subwindows fix addresses; measured, v0 skews 20.5% and v3 14.6%,
    # so 18% separates them. Under CONTINUOUS shocking neither is better and v3
    # is slightly worse -- see versions.md, it is a partial fix and is labelled
    # as one.
    sh = run("one shock during calibration", 70, ship_en, shipped,
             ki=info["ki"], kd=info["kd"],
             plant={"shock_rate": 8, "shock_amp": 3.0})
    clean = run("same run, no shocks", 70, ship_en, shipped,
                ki=info["ki"], kd=info["kd"])
    base, base0 = sh["baseline"], clean["baseline"]
    skew = max(abs(base[i] / base0[i] - 1.0) for i in range(4) if base0[i] > 0)
    if "runaway-baseline" in fixed:
        check("FIXED runaway-baseline: an isolated shock is outvoted by the "
              "median", skew < 0.18,
              f"worst channel skewed {skew * 100:.1f}% (v0 skews 20.5% here)")
    else:
        check("KNOWN GAP runaway-baseline: one shock in the calibration window "
              "skews the baseline", skew > 0.18,
              f"worst channel skewed {skew * 100:.1f}%")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Run an osem.vN controller against the simulated suspension.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("version", nargs="?", default=None,
                    help="v0, v1, ... (default: the newest)")
    ap.add_argument("--port", type=int, default=S.DEFAULT_PORT)
    ap.add_argument("--test", nargs="?", const="__self__", default=None,
                    metavar="VERSION", help="run the suite instead of serving; "
                                            "'all' walks the whole ladder")
    ap.add_argument("--list", action="store_true", help="list versions and exit")
    args = ap.parse_args()

    if args.list:
        print(f"{'version':<10} {'channels':<9} {'I/D':<8} fixes")
        for path in versions():
            d = describe(path)
            print(f"{d['name']:<10} {d['n_enabled']}/4       "
                  f"{'live' if any(d['ki']) or any(d['kd']) else 'zeroed':<8} "
                  f"{', '.join(d['fixes']) or '-'}")
        return 0

    if args.test is not None:
        target = args.version if args.test == "__self__" else args.test
        paths = versions() if target == "all" else [resolve(target)]
        t0 = time.perf_counter()
        for path in paths:
            suite(path)
        print(f"\n{'-' * 20} {_pass} passed, {_fail} failed "
              f"({time.perf_counter() - t0:.0f}s) {'-' * 20}")
        return 1 if _fail else 0

    path = resolve(args.version)
    S.load(path)
    return S.serve(args.port)


if __name__ == "__main__":
    sys.exit(main())
