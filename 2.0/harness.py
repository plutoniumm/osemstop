#!/usr/bin/env python3
"""python harness.py [word ...]   the loop against a synthetic plate.

A pass means the code does what the model says. The plate is built from the same
rig file the controller reads, so it cannot catch a wrong A or a flipped coil.
"""

import os
import sys
from dataclasses import replace
from datetime import datetime
import numpy as np
import osem
from osem.rig import HERE

NOW = datetime.now().isoformat(timespec="seconds")
RIG = replace(osem.Rig.load(os.path.join(HERE, "harness_rig.json")), measured=NOW)
TODAY = replace(osem.Rig.load(), measured=NOW)
CFG = osem.Config()
OFF = replace(CFG, kp=0.0, kd=0.0)
SCENARIOS = []


def scenario(fn):
    SCENARIOS.append(fn)
    return fn


def drive(seconds=80.0, rig=RIG, cfg=CFG, regulators=None, **plant):
    sim = osem.Sim(rig, seconds, wire_hz=200.0, **plant)
    ctl = osem.Controller(rig, cfg, regulators)
    t, state, counts, out = ([], [], [], [])
    try:
        while True:
            now, c = sim.read()
            u = ctl.step(c, now)
            if u is not None:
                sim.write(u)
                (t.append(now), state.append(ctl.state), counts.append(c), out.append(u.copy()))
    except EOFError:
        pass
    return (ctl, dict(t=np.array(t), state=np.array(state), c=np.array(counts), u=np.array(out)))


def rms(tr, t0, t1, ch=(0, 1, 2, 3)):
    m = (tr["t"] >= t0) & (tr["t"] < t1)
    return float(tr["c"][m][:, list(ch)].std(axis=0).mean())


_OPEN = {}


def damped(tr, t0, t1, seconds=80.0, rig=RIG, ch=(0, 1, 2, 3), **plant):
    key = (seconds, id(rig), repr(sorted(plant.items())))
    if key not in _OPEN:
        _OPEN[key] = drive(seconds, rig=rig, cfg=OFF, regulators=[osem.Pid(rig, OFF)], **plant)[1]
    return rms(tr, t0, t1, ch) / rms(_OPEN[key], t0, t1, ch)


@scenario
def zero_gain_moves_nothing():
    ctl, tr = drive(cfg=OFF, regulators=[osem.Pid(RIG, OFF)])
    return [
        ("no coil moves", (tr["u"] == RIG.bias).all(), ""),
        ("no fault", ctl.fault_count == 0, ""),
    ]


@scenario
def modal_damps_and_locks():
    ctl, tr = drive()
    r = damped(tr, 60, 80)
    modal = ctl.find(osem.Modal)
    return [
        ("motion falls", r < 0.5, "x%.2f of open loop" % r),
        ("locks", ctl.locked_announced, "%.1f s" % (ctl.lock_time or np.nan)),
        ("no faults", ctl.fault_count == 0, ""),
        ("modal drives the resolved coils", list(np.nonzero(modal.claimed)[0]) == RIG.a_coils, ""),
    ]


@scenario
def pid_only_damps():
    ctl, tr = drive(regulators=[osem.Pid(RIG, CFG)])
    r = damped(tr, 60, 80)
    return [
        ("motion falls", r < 0.7, "x%.2f of open loop" % r),
        ("no faults", ctl.fault_count == 0, ""),
    ]


@scenario
def flipped_plant_latches_off():
    ctl, tr = drive(seconds=200.0, a_scale=-1.0)
    fault = tr["state"] == "FAULT"
    entry = np.nonzero(fault & ~np.r_[False, fault[:-1]])[0]
    since = tr["t"] - tr["t"][entry][np.searchsorted(entry, np.arange(len(fault)), "right") - 1]
    settled = fault & (since > 0.5)
    grew = damped(tr, 180, 200, seconds=200.0, a_scale=-1.0)
    return [
        ("runaway caught", len(entry) >= 1, "%.1f s after engaging" % (tr["t"][entry[0]] - 20.0)),
        (
            "latches after the second failure",
            ctl.latched and ctl.fault_count == CFG.max_failed_engagements,
            "",
        ),
        ("coils at bias in FAULT", np.abs(tr["u"][settled] - RIG.bias).max() < 1e-09, ""),
        ("motion added before letting go", None, "x%.1f of open loop" % grew),
    ]


@scenario
def kick_decays_without_a_fault():
    ctl, tr = drive(seconds=100.0, kicks=[(50.0, 4.0)])
    hit, later = (rms(tr, 50, 55), rms(tr, 85, 100))
    return [
        ("no fault", ctl.fault_count == 0, ""),
        ("decays", later < 0.4 * hit, "%.1f -> %.1f counts" % (hit, later)),
    ]


@scenario
def limits_hold():
    ctl, tr = drive(seconds=70.0, kicks=[(40.0, 40.0)])
    peak = np.abs(tr["u"] - RIG.bias).max()
    return [
        (
            "the kick reaches the budget",
            ctl.peak_demand > 0.95 * CFG.budget_v,
            "%.3f V" % ctl.peak_demand,
        ),
        ("no coil leaves it", peak <= CFG.budget_v + 1e-09, "%.4f V" % peak),
        (
            "summed command under the cap",
            tr["u"].sum(1).max() <= CFG.sum_v_max + 1e-09,
            "%.2f V" % tr["u"].sum(1).max(),
        ),
    ]


@scenario
def railed_sensor_is_dropped():
    ctl, tr = drive(seconds=90.0, stuck={1: (40.0, 1023)})
    r = damped(tr, 70, 90, seconds=90.0, ch=(0, 2, 3), stuck={1: (40.0, 1023)})
    return [
        (
            "ch1 demoted, its coil parked",
            not ctl.health.healthy[1] and abs(tr["u"][-1][1] - RIG.bias[1]) < 1e-09,
            "",
        ),
        ("no fault", ctl.fault_count == 0, ""),
        ("the rest still damp", r < 0.7, "x%.2f of open loop" % r),
    ]


@scenario
def todays_rig():
    ctl, tr = drive(rig=TODAY)
    r = damped(tr, 60, 80, rig=TODAY)
    pid = osem.Pid(TODAY, CFG)
    return [
        (
            "PID fallback safe, or off on the modal coils",
            pid.safe or not pid.target[TODAY.a_coils].any(),
            "eig %s" % np.round(pid.eig, 3),
        ),
        ("motion falls, no faults", r < 0.6 and ctl.fault_count == 0, "x%.2f of open loop" % r),
        ("limits hold", np.abs(tr["u"] - TODAY.bias).max() <= CFG.budget_v + 1e-09, ""),
    ]


@scenario
def parked_on_exit():
    sim = osem.Sim(RIG, 30.0, wire_hz=200.0)
    osem.run(sim, osem.Controller(RIG, CFG), say=lambda *_: None)
    return [("parked at bias", sim.parked and np.array_equal(sim.u, RIG.bias), "")]


@scenario
def signs_follow_the_slope():
    s = RIG.slope.copy()
    s[3], s[0] = (-s[3], 0.1 * RIG.slope_sigma[0])
    g0, g1 = (RIG.slope_sign, replace(RIG, slope=s).slope_sign)
    gain = osem.Pid(RIG, CFG).target
    stale = {
        n: ok for n, ok, _ in osem.preflight(replace(RIG, measured="2026-08-20T15:56:38"), CFG)
    }
    return [
        ("a flipped slope flips the sign", g1[3] == -g0[3] != 0, ""),
        ("an unresolved slope gets none", g1[0] == 0.0, ""),
        ("gains carry the slope's sign", (np.sign(gain) == g0)[gain != 0].all(), ""),
        ("preflight passes a fresh rig", all((ok for _, ok, _ in osem.preflight(RIG, CFG))), ""),
        ("preflight refuses a stale one", not stale["rig.json is fresh"], ""),
    ]


def main(words):
    bad = 0
    for fn in SCENARIOS:
        if words and (not any((w in fn.__name__ for w in words))):
            continue
        print("\n" + fn.__name__.replace("_", " "))
        for claim, ok, num in fn():
            bad += ok is not None and (not ok)
            print(
                "  %-4s %-44s %s" % ("INFO" if ok is None else "PASS" if ok else "FAIL", claim, num)
            )
    print("\n" + ("ALL PASS" if not bad else "%d FAILURE(S)" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
