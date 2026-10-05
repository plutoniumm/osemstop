#!/usr/bin/env python3
"""harness -- the loop against the synthetic plate, one scenario per failure it
must survive. `python harness.py` runs all; `python harness.py kick wrong` runs
the ones whose name contains a word. Exit code 1 on any failure.

Each scenario is a function returning [(claim, ok, number)]. The plant, the rig,
the config and the laws are all arguments, so a new scenario is a dozen lines.

WHAT A PASS MEANS: the code does what the model says. The plant here is linear
and built from the same rig.json the controller reads, so it CANNOT catch a wrong
A, a flipped coil, a shared line or a sagging sample rate. Only the bench can.
"""

import sys
from dataclasses import replace
from datetime import datetime

import numpy as np

import control
import rig as rigmod
import run as runner
import transport

# A FROZEN rig, not the live rig.json: the scenarios must not change meaning
# every time the bench is re-measured. It is the 2026-08-20 plate, on which
# per-channel feedback is safe. TODAY is whatever rig.json holds right now.
NOW = datetime.now().isoformat(timespec="seconds")
RIG = replace(rigmod.Rig.load(rigmod.os.path.join(rigmod.HERE, "harness_rig.json")),
              measured=NOW)
TODAY = replace(rigmod.Rig.load(), measured=NOW)
CFG = control.Config()
SCENARIOS = []


def scenario(fn):
    SCENARIOS.append(fn)
    return fn


def drive(seconds=80.0, rig=RIG, cfg=CFG, modal="auto", **plant):
    """Run the loop; return the controller and per-control-step traces."""
    tr = transport.Sim(rig, seconds, wire_hz=200.0, **plant)
    ctl = control.Controller(rig, cfg, modal=modal)
    t, state, counts, out = [], [], [], []
    try:
        while True:
            now, c = tr.read()
            u = ctl.step(c, now)
            if u is not None:
                tr.write(u)
                t.append(now), state.append(ctl.state), counts.append(c), out.append(u)
    except EOFError:
        pass
    return ctl, dict(t=np.array(t), state=np.array(state), c=np.array(counts),
                     u=np.array(out), events="".join(ctl.drain_events()))


def rms(tr, t0, t1, ch=(0, 1, 2, 3)):
    """Corner-sensor motion in counts over a window, mean removed."""
    m = (tr["t"] >= t0) & (tr["t"] < t1)
    return float(tr["c"][m][:, list(ch)].std(axis=0).mean())


_OPEN = {}


def damped(tr, t0, t1, seconds=80.0, ch=(0, 1, 2, 3), **plant):
    """Motion against the SAME plant realisation with the gain at zero.

    Paired on purpose. The plate's amplitude wanders by a factor ~2 on its own
    over a minute (tau 138 s), so a before/after ratio inside one run measures
    the weather; 1.0 quoted a run-to-run scatter of 2-3x for the same reason.
    """
    key = (seconds, repr(sorted(plant.items())))
    if key not in _OPEN:
        _OPEN[key] = drive(seconds, cfg=replace(CFG, kp=0.0, kd=0.0), modal=None,
                           **plant)[1]
    return rms(tr, t0, t1, ch) / rms(_OPEN[key], t0, t1, ch)


@scenario
def open_loop_is_flat():
    """The control: no gain, no change. Shows the metric measures the loop."""
    ctl, tr = drive(cfg=replace(CFG, kp=0.0, kd=0.0), modal=None)
    r = rms(tr, 60, 80) / rms(tr, 2, 20)
    return [("zero gain never moves a coil", (tr["u"] == RIG.bias).all(), ""),
            ("...and never faults on the plate's own wander", ctl.fault_count == 0,
             "amplitude drifted x%.2f by itself" % r)]


@scenario
def modal_damps_and_locks():
    ctl, tr = drive()
    r = damped(tr, 60, 80)
    return [("motion falls under the modal law", r < 0.5, "x%.2f of open loop" % r),
            ("LOCKED is announced", ctl.locked_announced,
             "%.1f s after engaging" % (ctl.lock_time or np.nan)),
            ("no faults", ctl.fault_count == 0, "%d" % ctl.fault_count),
            ("modal drives exactly the resolved coils",
             list(np.nonzero(ctl.modal_cols)[0]) == RIG.a_coils,
             str(np.nonzero(ctl.modal_cols)[0])),
            ("PID drives no coil the modal law has",
             not (ctl.pid_on & ctl.modal_cols).any(), "")]


@scenario
def pid_only_fallback_damps():
    ctl, tr = drive(modal=None)
    r = damped(tr, 60, 80)
    return [("motion falls under PID alone", r < 0.7, "x%.2f of open loop" % r),
            ("PID is on the corner channels", ctl.pid_on[:4].all(), str(ctl.pid_on)),
            ("no faults", ctl.fault_count == 0, "%d" % ctl.fault_count)]


@scenario
def wrong_sign_plant_trips_the_breaker():
    """Every coil's polarity flipped under the controller, as a re-seating did to
    a3. Both laws now PUMP. The loop must notice and let go."""
    ctl, tr = drive(seconds=200.0, a_scale=-1.0)
    fault = tr["state"] == "FAULT"
    entry = np.nonzero(fault & ~np.r_[False, fault[:-1]])[0]
    since = tr["t"] - tr["t"][entry][np.searchsorted(entry, np.arange(len(fault)),
                                                    "right") - 1] if len(entry) else 0
    settled = fault & (since > 0.5)                       # past the slew to bias
    grew = damped(tr, 180, 200, seconds=200.0, a_scale=-1.0)
    return [("the first runaway is caught", len(entry) >= 1,
             "t=%.1f s, %.1f s after engaging"
             % ((tr["t"][entry[0]], tr["t"][entry[0]] - 20.0) if len(entry)
                else (np.nan, np.nan))),
            ("it LATCHES after the second failed engagement",
             ctl.latched and ctl.fault_count == CFG.max_failed_engagements,
             "%d fault(s)" % ctl.fault_count),
            ("coils sit at bias through every FAULT",
             settled.any() and np.abs(tr["u"][settled] - RIG.bias).max() < 1e-9, ""),
            ("motion it added before letting go", None,
             "x%.1f of open loop -- the breaker takes ~9 s and that is too slow"
             % grew)]


@scenario
def kick_decays_without_a_fault():
    ctl, tr = drive(seconds=100.0, kicks=[(50.0, 4.0)])
    hit, later = rms(tr, 50, 55), rms(tr, 85, 100)
    return [("the kick is visible", hit > 2.0 * rms(tr, 40, 50), "%.1f counts" % hit),
            ("no fault on a hand-kick-sized hit", ctl.fault_count == 0,
             "%d" % ctl.fault_count),
            ("it decays", later < 0.4 * hit, "%.1f -> %.1f counts" % (hit, later))]


@scenario
def budget_is_never_exceeded():
    ctl, tr = drive(seconds=70.0, kicks=[(40.0, 40.0)])
    peak = np.abs(tr["u"] - RIG.bias).max()
    return [("the hit saturates the budget", ctl.peak_demand > 0.95 * CFG.budget_v,
             "%.3f V demanded" % ctl.peak_demand),
            ("no coil ever leaves the budget", peak <= CFG.budget_v + 1e-9,
             "peak %.4f V of %.3f" % (peak, CFG.budget_v))]


@scenario
def budget_ramp_stays_bounded():
    k = RIG.nm + 1
    cfg = replace(CFG, budget_tilt=1.0)
    ctl, tr = drive(seconds=120.0, cfg=cfg, kicks=[(60.0, 4.0)])
    w = ctl.budget.w
    r = damped(tr, 100, 120, seconds=120.0, kicks=[(60.0, 4.0)])
    flat = control.Budget(RIG.nm)
    flat.update(0.01, [np.nan, 1.0, np.inf, 0.0], 1.0, True)
    return [("tilt 1 moves the weights", np.ptp(w) > 0.05, "w %s" % np.round(w, 3)),
            ("weights stay positive and sum to their count",
             w.min() > 0 and abs(w.sum() - k) < 0.2, "sum %.3f" % w.sum()),
            ("the PID block never takes more than its own gain", w[-1] <= 1.0 + 1e-12,
             "%.3f" % w[-1]),
            ("tilt 0 is EXACTLY flat, even on nan/inf need", (flat.w == 1.0).all(),
             str(flat.w)),
            ("the armed ramp still damps, no fault",
             ctl.fault_count == 0 and r < 0.6, "x%.2f of open loop" % r)]


@scenario
def railed_sensor_is_dropped_not_fatal():
    ctl, tr = drive(seconds=90.0, stuck={1: (40.0, 1023)})
    quiet = damped(tr, 70, 90, seconds=90.0, ch=(0, 2, 3), stuck={1: (40.0, 1023)})
    return [("ch1 is demoted", not ctl.health.healthy[1], ""),
            ("its coil is parked", abs(tr["u"][-1][1] - RIG.bias[1]) < 1e-9, ""),
            ("the run does not fault", ctl.fault_count == 0, "%d" % ctl.fault_count),
            ("the other three still damp", quiet < 0.7, "x%.2f of open loop" % quiet)]


@scenario
def todays_rig_damps_and_unsafe_pid_stays_off():
    """The live rig.json. If per-channel feedback would pump a mode on it, PID
    must be off on every modal coil -- and the modal law must still damp."""
    ctl, tr = drive(rig=TODAY)
    r = rms(tr, 60, 80) / rms(drive(rig=TODAY, cfg=replace(CFG, kp=0.0, kd=0.0),
                                    modal=None)[1], 60, 80)
    off = not ctl.pid_gain[TODAY.a_coils].any()
    return [("PID fallback is " + ("safe" if ctl.pid_fallback else "UNSAFE and off"),
             ctl.pid_fallback or off, "eigenvalues %s" % np.round(ctl.pid_eig, 3)),
            ("modal drives every resolved coil",
             list(np.nonzero(ctl.modal_cols)[0]) == TODAY.a_coils,
             str(TODAY.a_coils)),
            ("motion falls, no faults", r < 0.6 and ctl.fault_count == 0,
             "x%.2f of open loop, %d fault(s)" % (r, ctl.fault_count)),
            ("no coil leaves the budget",
             np.abs(tr["u"] - TODAY.bias).max() <= CFG.budget_v + 1e-9, "")]


@scenario
def coils_are_parked_on_exit():
    tr = transport.Sim(RIG, 30.0, wire_hz=200.0)
    runner.loop(tr, control.Controller(RIG, CFG), say=lambda *_: None)
    return [("park was called", tr.parked, ""),
            ("every coil is at bias", np.array_equal(tr.u, RIG.bias), str(tr.u))]


@scenario
def signs_follow_the_measured_slope():
    s = RIG.slope.copy()
    s[3], s[0] = -s[3], 0.1 * RIG.slope_sigma[0]
    g0, g1 = RIG.slope_sign, replace(RIG, slope=s).slope_sign
    gain = control.Controller(RIG, CFG).pid_gain
    stale = replace(RIG, measured="2026-08-20T15:56:38")
    fresh = {n: ok for n, ok, _ in control.preflight(RIG, CFG)}
    old = {n: ok for n, ok, _ in control.preflight(stale, CFG)}
    return [("a flipped slope flips that sign", g1[3] == -g0[3] != 0, "%+.0f" % g1[3]),
            ("an unresolved slope gets sign 0, so no gain", g1[0] == 0.0, "%+.0f" % g1[0]),
            ("every gain carries its slope's sign", (np.sign(gain) == g0)[gain != 0].all(),
             str(np.round(gain, 4))),
            ("preflight passes a fresh rig", all(fresh.values()),
             str([n for n, ok in fresh.items() if not ok])),
            ("preflight refuses a stale one", not old["rig.json is fresh"], "")]


def main(argv):
    want = [a for a in argv if not a.startswith("-")]
    bad = 0
    for fn in SCENARIOS:
        if want and not any(w in fn.__name__ for w in want):
            continue
        print("\n%s" % fn.__name__.replace("_", " "))
        for claim, ok, num in fn():
            bad += ok is not None and not ok       # `is False` misses numpy bools
            print("  %-4s %-52s %s" % ("INFO" if ok is None else
                                       "PASS" if ok else "FAIL", claim, num))
    print("\n%s" % ("ALL PASS" if not bad else "%d FAILURE(S)" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
