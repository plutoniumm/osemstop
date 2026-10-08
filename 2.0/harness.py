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
from osem.filters import Clock, Estimator
from osem.rig import GEOMETRY, HERE, identify
from osem.session import check, say

NOW = datetime.now().isoformat(timespec="seconds")
RIG = replace(osem.Rig.load(os.path.join(HERE, "harness_rig.json")), measured=NOW)
TODAY = replace(osem.Rig.load(), measured=NOW)
CFG = osem.Config()
OFF = replace(CFG, modal_kp=0.0)
CORE = len(GEOMETRY)
SCENARIOS = []


def scenario(fn):
    SCENARIOS.append(fn)
    return fn


def drive(seconds=80.0, rig=RIG, cfg=CFG, **plant):
    sim = osem.Sim(rig, seconds, **plant)
    ctl = osem.Controller(rig, cfg)
    t, state, counts, out = ([], [], [], [])
    try:
        while True:
            now, c = sim.read()
            u = ctl.step(c, now)
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
        _OPEN[key] = drive(seconds, rig=rig, cfg=OFF, **plant)[1]
    return rms(tr, t0, t1, ch) / rms(_OPEN[key], t0, t1, ch)


@scenario
def zero_gain_moves_nothing():
    ctl, tr = drive(cfg=OFF)
    return [
        ("no coil moves", (tr["u"] == RIG.bias).all(), ""),
        ("no fault", ctl.fault_count == 0, ""),
    ]


@scenario
def modal_damps_and_locks():
    ctl, tr = drive()
    r = damped(tr, 60, 80)
    return [
        ("motion falls", r < 0.5, "x%.2f of open loop" % r),
        ("locks", ctl.locked_announced, "%.1f s" % (ctl.lock_time or np.nan)),
        ("no faults", ctl.fault_count == 0, ""),
        ("modal drives the resolved coils", list(np.nonzero(ctl.claimed)[0]) == RIG.a_coils, ""),
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
        (
            "runaway caught",
            len(entry) >= 1,
            "%.1f s after engaging" % (tr["t"][entry[0]] - ctl.damping_start),
        ),
        (
            "latches on the first failure before any lock",
            ctl.latched and ctl.fault_count == 1,
            "",
        ),
        ("coils at bias in FAULT", np.abs(tr["u"][settled] - RIG.bias).max() < 1e-09, ""),
        ("motion added before letting go", None, "x%.1f of open loop" % grew),
    ]


@scenario
def starts_in_the_port_open_ring():
    ctl, tr = drive(
        seconds=50.0, rig=TODAY, mode_rms=4.0, kicks=[(0.0, 3.0), (25.0, 4.5)], delay=0.012
    )
    lock = ctl.damping_start + (ctl.lock_time or np.nan)
    quiet = CFG.quiet_sigma * np.sqrt(TODAY.kalman_r[:4]) / CFG.volts_per_count
    return [
        ("rings like the rig at start", None, "corners %.0f counts rms" % rms(tr, 0, CFG.warmup_s)),
        (
            "gain on after the warm-up",
            ctl.damping_start < CFG.warmup_s + 0.02,
            "%.2f s" % ctl.damping_start,
        ),
        ("locked within 10 s of starting", lock < 10.0, "%.1f s" % lock),
        (
            "locked means quiet",
            rms(tr, lock, lock + 2) < quiet.mean(),
            "%.1f counts" % rms(tr, lock, lock + 2),
        ),
        ("a hard kick does not fault", ctl.fault_count == 0, ""),
        (
            "and is quiet again",
            bool(ctl.locked[:4].all()),
            "%.1f counts at the end" % rms(tr, 45, 50),
        ),
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
    return [
        ("motion falls, no faults", r < 0.6 and ctl.fault_count == 0, "x%.2f of open loop" % r),
        ("limits hold", np.abs(tr["u"] - TODAY.bias).max() <= CFG.budget_v + 1e-09, ""),
    ]


@scenario
def estimator_takes_the_coil_force():
    unit = np.linalg.norm(TODAY.phi, axis=0) * CFG.volts_per_count
    rows, err = (np.linalg.norm(TODAY.phi, axis=1) > 0, {})
    for sign in (1.0, 0.0, -1.0):
        sim = osem.Sim(TODAY, 40.0, mode_rms=1.0)
        est, clock = (Estimator(TODAY, CFG), Clock(CFG.control_hz))
        est.mkf.g = sign * est.mkf.g
        u, e = (np.zeros(TODAY.n), [])
        for _ in range(int(40.0 * CFG.control_hz) - 1):
            t, c = sim.read()
            est.update(c * CFG.volts_per_count, clock.tick(t), rows, u)
            if t > 10:
                e.append(est.qdot - sim.v * unit)
            u = np.zeros(TODAY.n)
            u[0] = 0.1 * (int(t / 1.3) % 2)
            sim.write(TODAY.bias + u)
        err[sign] = float(np.sqrt(np.mean(np.square(e))))
    ctl, tr = drive(rig=TODAY, delay=0.012)
    r = damped(tr, 60, 80, rig=TODAY, delay=0.012)
    return [
        (
            "knowing the coil beats ignoring it beats the wrong sign",
            err[1.0] < 0.5 * err[0.0] and err[0.0] < err[-1.0],
            "velocity error %.4f / %.4f / %.4f" % (err[1.0], err[0.0], err[-1.0]),
        ),
        (
            "loop with a 12 ms actuator delay damps, locks, no faults",
            r < 0.6 and ctl.locked_announced and ctl.fault_count == 0,
            "x%.2f of open loop" % r,
        ),
        ("limits hold", np.abs(tr["u"] - TODAY.bias).max() <= CFG.budget_v + 1e-09, ""),
    ]


@scenario
def identify_reproduces_todays_rig():
    src = TODAY.source
    if not os.path.exists(src["kalman_r"]):
        return [("rig.json's own records are on disk", None, "not found")]
    new = identify(src["kalman_r"], src["slope"], TODAY, CFG, say=lambda *_: None)
    if new.dof != TODAY.dof:
        return [("same modes", False, "%s, rig.json has %s" % (new.dof, TODAY.dof))]

    def off(*keys):
        return max((np.abs(getattr(new, k) - getattr(TODAY, k)).max() for k in keys))

    return [
        ("same modes", True, " ".join(new.dof)),
        ("same frequencies, Phi and A", off("f_hz", "phi", "a_dc") < 1e-09, ""),
        ("same Kalman gain", None, "largest difference %.1e" % off("kalman_k")),
    ]


@scenario
def extra_modes_are_damped():
    if TODAY.nm == CORE:
        return [("today's rig holds modes beyond the three", None, "none")]
    ch = tuple((int(np.argmax(np.abs(TODAY.phi[:, m]))) for m in range(CORE, TODAY.nm)))
    plant = dict(kicks=[(40.0, 1.0)], delay=0.009)
    ctl, tr = drive(seconds=80.0, rig=TODAY, **plant)
    r = [damped(tr, 55, 80, rig=TODAY, ch=(i,), **plant) for i in ch]
    return [
        ("today's rig holds modes beyond the three", True, " ".join(TODAY.dof[CORE:])),
        ("every coil is in the loop", None, "%s" % TODAY.a_coils),
        (
            "a kick on them decays",
            max(r) < 0.5,
            " ".join(("a%d x%.2f" % (i, x) for i, x in zip(ch, r))) + " of open loop",
        ),
        ("locks, no faults", ctl.locked_announced and ctl.fault_count == 0, ""),
    ]


@scenario
def wrong_sign_on_an_extra_mode_latches_off():
    out = []
    for m in range(CORE, TODAY.nm):
        flip = np.ones((TODAY.nm, 1))
        flip[m] = -1.0
        ctl, tr = drive(seconds=150.0, rig=TODAY, a_scale=flip, delay=0.009)
        i = int(np.argmax(np.abs(TODAY.phi[:, m])))
        grew = tr["c"][:, i].std() / tr["c"][tr["t"] < CFG.warmup_s, i].std()
        out += [
            (
                "%s pumped: caught and latched off" % TODAY.dof[m],
                ctl.latched and np.abs(tr["u"][-1] - TODAY.bias).max() < 1e-09,
                "%d faults, a%d reached x%.0f of its free motion" % (ctl.fault_count, i, grew),
            )
        ]
    return out or [("today's rig holds modes beyond the three", None, "none")]


@scenario
def parked_on_exit():
    sim = osem.Sim(RIG, 30.0)
    osem.run(sim, osem.Controller(RIG, CFG), say=lambda *_: None)
    return [("parked at bias", sim.parked and np.array_equal(sim.u, RIG.bias), "")]


@scenario
def signs_follow_the_slope():
    s = RIG.slope.copy()
    s[3], s[0] = (-s[3], 0.1 * RIG.slope_sigma[0])
    g0, g1 = (RIG.slope_sign, replace(RIG, slope=s).slope_sign)
    stale = {
        n: ok for n, ok, _ in osem.preflight(replace(RIG, measured="2026-08-20T15:56:38"), CFG)
    }
    return [
        ("a flipped slope flips the sign", g1[3] == -g0[3] != 0, ""),
        ("an unresolved slope gets none", g1[0] == 0.0, ""),
        ("preflight passes a fresh rig", all((ok for _, ok, _ in osem.preflight(RIG, CFG))), ""),
        ("preflight refuses a stale one", not stale["rig.json is fresh"], ""),
    ]


def main(words):
    bad = 0
    for fn in SCENARIOS:
        if words and (not any((w in fn.__name__ for w in words))):
            continue
        say("\n" + fn.__name__.replace("_", " "))
        for claim, ok, num in fn():
            bad += ok is not None and (not ok)
            check(ok, claim, num, 44)
    say("\n" + ("ALL PASS" if not bad else "%d FAILURE(S)" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
