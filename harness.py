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
    """Every osem.vN.py here, ordered by N. Half-steps (osem.v5.5.py) sort
    between their neighbours -- they are commissioning variants of the version
    they branch from, not new rungs of the ladder."""
    found = []
    for path in glob.glob(os.path.join(HERE, "osem.v*.py")):
        m = re.match(r"osem\.v(\d+)(?:\.(\d+))?$", os.path.basename(path)[:-3])
        if m:
            found.append(((int(m.group(1)), int(m.group(2) or 0)), path))
    return [p for _, p in sorted(found)]


def resolve(name):
    """'v2', 'osem.v2', 'osem.v2.py' or a path -> an absolute path."""
    all_versions = versions()
    if not all_versions:
        sys.exit("No osem.v*.py found next to harness.py.")
    if name is None:
        # The newest FILE is now osem.v6.6.py, a bench measurement tool, so a
        # bare `make sim` or `make test` would land on something the simulator
        # refuses to run. Default to the newest thing it can actually drive.
        drivable = simulatable()
        return drivable[-1] if drivable else all_versions[-1]
    if os.path.isfile(name):
        return os.path.abspath(name)
    stem = name if name.startswith("osem.") else "osem." + name
    cand = os.path.join(HERE, stem if stem.endswith(".py") else stem + ".py")
    if os.path.isfile(cand):
        return cand
    sys.exit(f"No such version: {name}. Have: "
             + ", ".join(os.path.basename(p)[5:-3] for p in all_versions))


_DECL_RE = r'^%s\s*=\s*["\']([^"\']*)["\']'


def declared(path, name, default=""):
    """Read a string constant WITHOUT executing the module.

    Necessary, not merely cheap: `sim/server.py`'s load() derives constants from
    the module as it imports it and raises on anything that is not a controller,
    so a measurement tool cannot be introspected by loading it.
    """
    with open(path) as fh:
        m = re.search(_DECL_RE % name, fh.read(), re.M)
    return m.group(1) if m else default


def simulatable():
    """The versions the simulator can actually drive: real controllers with the
    four channels sim/server.py models. `versions()` still returns everything,
    because --list should show the bench tools and the 8-channel builds even
    though nothing here can run them."""
    return [p for p in versions()
            if declared(p, "KIND", "controller") == "controller"
            and describe(p)["n_channels"] == 4]


def describe(path):
    """osem.v6*.py are measurement tools, not controllers -- they declare
    KIND = "sysid", close no loop and expose no Controller. They are described
    from source text and never handed to the simulator."""
    name = os.path.basename(path)[:-3]
    kind = declared(path, "KIND", "controller")
    if kind != "controller":
        return dict(path=path, name=name, kind=kind, bench="bench tool",
                    tag=declared(path, "VERSION_TAG", name[5:]), fixes=(),
                    n_enabled=0, n_channels=0, ki=[], kd=[], steady=[])
    osem = S.load(path)
    enable = list(getattr(osem, "ENABLE_CHANNEL", []))
    return dict(
        path=path,
        name=os.path.basename(path)[:-3],
        tag=getattr(osem, "VERSION_TAG", "v0"),
        kind=getattr(osem, "KIND", "controller"),
        fixes=tuple(getattr(osem, "FIXES", ())),
        bench=getattr(osem, "BENCH_STATUS", "unknown"),
        n_enabled=sum(bool(v) for v in enable),
        n_channels=len(enable),
        ki=[float(v) for v in getattr(osem, "KI_GAIN", [])],
        kd=[float(v) for v in getattr(osem, "KD_GAIN", [])],
        steady=[float(v) for v in getattr(osem, "STEADY_GAIN", [])],
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


# ack_drain = 0 here ON PURPOSE, and it is the one place this suite departs from
# the simulator's own default (3.5, calibrated to the ~24 Hz the bench actually
# runs at while driving coils -- see sim/server.py ACK_DRAIN).
#
# This suite characterises the CONTROL LAW. The ack drain is a property of the
# TRANSPORT, and a large one: at 24 Hz the sample-counted
# MAX_CONSECUTIVE_SATURATED = 30 means 1.2 s rather than 86 ms, so saturation
# stops tripping at all, and the actuator updates 4x less often so a wrong-signed
# channel barely pumps. Twenty of the checks below invert under it -- not because
# the controllers changed, but because they would be measuring pyDAC's ack
# round-trip instead of the loop. Pinning it to 0 keeps each check about the
# thing it names.
#
# `make sim` does NOT pin it -- it runs at the simulator's honest default, so
# what you watch there is the loop the hardware has. `make test` DOES, because it
# drives this same suite through run() and therefore inherits BASE_PLANT; its `d`
# key turns the drain on so the effect is still watchable. When the controllers
# move to pyDAC2 (fire-and-forget, no ack) 0 becomes the truthful value here too.
BASE_PLANT = dict(ack_drain=0.0,
                  f_drive=1.0, drive_amp=0.8, seismic=0.8, Q=50.0, k_act=20.0,
                  meas_noise=20.0, hum_hz=60.0, hum_mv=0.0, hvac_hz=2.5,
                  hvac_amp=0.0, shock_rate=0.0, shock_amp=3.0)


def run(label, seconds, enable, steady, ki=None, kd=None, occlude_at=None,
        auto_kick=False, plant=None, quiet=True, shock_at=None,
        stop_on_recover=False, until_baseline=False, occlusions=None):
    """Step the loaded controller through the simulated plant. Returns the
    observations the checks below read.

    `seconds` is an upper bound, not a duration: `stop_on_recover` cuts the run
    at the first FAULT -> CALIBRATING transition and `until_baseline` at the end
    of the first calibration. Both exist so a check that only cares about one
    event does not pay for a version whose CALIBRATION_S is 20 s (v3) as if
    it were 8 s (v0..v2).

    `shock_at` is `(t, dv)`: one deterministic velocity impulse, the same one
    `sim.kick()` applies from the TUI. Note it also draws two random numbers for
    the pitch/yaw split, so a run with a shock and a run without diverge in the
    noise stream after it -- the no-shock run is a reference, not an exact
    counterfactual.

    `occlude_at` is `(idx, t)`: blind one OSEM from `t` to the end of the run.
    `occlusions` is the general form, `[(idx, t_on, t_off_or_None), ...]`, and
    exists because v4's auto-disable is only half-testable without an un-blind:
    "it drops the axis" and "it puts the axis back" are separate claims, and the
    second one needs the sensor to come back. Several entries can overlap, which
    is how the quorum is exercised.
    """
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

    faults = recovered = reengaged = 0
    state_n = {}
    rail_seen = frozen_on_rail = False
    prev = sim.ctl.state
    peak = [0.0] * 4
    t_rail = None
    shock_fired = False
    # Baselines as they were at the END OF THE FIRST CALIBRATION. Reading
    # `Channel.baseline_rms` after the run instead reports whatever the LAST
    # calibration produced, which for a version that faulted and re-calibrated
    # mid-run is a different window under different conditions -- that is what
    # once made a version read 58.6% on the calibration-skew check while running
    # v3's calibration code byte for byte.
    baseline0 = None
    # saturated_flag anywhere, any sample: a saturation trip is what latches
    # FAULT in v0/v1/v2, so "did this scenario saturate?" decides whether it is
    # testing the runaway path or the saturation path.
    sat_ever = [False] * 4
    # Peak |out - bias| per channel, against the +-0.25 V the controller clips
    # to. Two uses: proving a disabled channel never actuates, and proving a
    # runaway scenario tripped on amplitude rather than on a pinned actuator.
    bias = [float(b) for b in S.osem.BIAS]
    out_dev = [0.0] * 4
    # Capture the PID terms DURING the run. `sim` is a singleton and reset()
    # replaces sim.ctl, so reading them off sim afterwards reports whatever the
    # most recent run left behind -- which, if that run ended mid-FAULT, is a
    # correctly-zeroed integrator, and the check would fail a healthy version.
    max_p = max_i = max_d = 0.0
    # --- v4 auto-disable observations ---------------------------------------
    # `healthy` is runtime health, distinct from the static `enabled`; versions
    # before v4 do not have it, so getattr defaults to True and every field
    # below reduces to a constant for them rather than needing a second suite.
    occlusions = list(occlusions or [])
    prev_healthy = [True] * 4
    rearms = 0
    t_demote = t_rearm = None
    min_actuating = 4
    # Peak |out - bias| per channel WHILE some channel is demoted. The 0.5 s
    # offset from t_demote skips the slew: MAX_SLEW_PER_S is 2.0 V/s, so a
    # channel dropping from a 0.25 V excursion needs 0.125 s to reach bias and
    # would otherwise look like it kept actuating after being dropped.
    dev_blind = [0.0] * 4
    # v0's check_rail prints on every railed sample with no latch, which floods
    # the console during the occlusion sweep. The rail_fault flag is what the
    # assertions read, so swallow it.
    sink = io.StringIO()
    for n in range(int(seconds * S.SAMPLE_HZ)):
        if occlude_at is not None and sim.t >= occlude_at[1]:
            sim.occlude[occlude_at[0]] = True
        for oi, t_on, t_off in occlusions:
            sim.occlude[oi] = sim.t >= t_on and (t_off is None or sim.t < t_off)
        if shock_at is not None and not shock_fired and sim.t >= shock_at[0]:
            sim.kick(shock_at[1])
            shock_fired = True
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
        state_n[st] = state_n.get(st, 0) + 1
        if st != prev:
            if st == "FAULT":
                faults += 1
            # Recovery has TWO exits. v0..v5-pre always went FAULT -> CALIBRATING;
            # a version that reuses its previous baseline instead of re-measuring
            # it goes FAULT -> DAMPING. Both are recoveries, and counting only the
            # first reports a working loop as permanently faulted.
            if prev == "FAULT" and st in ("CALIBRATING", "DAMPING"):
                recovered += 1
                if st == "DAMPING":
                    reengaged += 1
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
        for i, c in enumerate(sim.ctl.channels):
            if c.saturated_flag:
                sat_ever[i] = True
            out_dev[i] = max(out_dev[i], abs(float(c.out) - bias[i]))

        chans = sim.ctl.channels
        healthy = [bool(getattr(c, "healthy", True)) for c in chans]
        for i in range(4):
            if not healthy[i] and prev_healthy[i] and t_demote is None:
                t_demote = sim.t
            if healthy[i] and not prev_healthy[i]:
                rearms += 1
                if t_rearm is None:
                    t_rearm = sim.t
        prev_healthy = healthy
        # Counted in EVERY state, not just DAMPING. When the last healthy
        # channel drops, the demotion and the quorum fault happen inside one
        # step(), so by the time the state is read back it is already FAULT --
        # gating on DAMPING would miss the only sample where the count was zero
        # and report the run as never having lost an axis.
        min_actuating = min(min_actuating,
                            sum(1 for i, c in enumerate(chans)
                                if c.enabled and healthy[i]))
        if st == "DAMPING":
            if not all(healthy) and t_demote is not None and sim.t >= t_demote + 0.5:
                for i, c in enumerate(chans):
                    dev_blind[i] = max(dev_blind[i], abs(float(c.out) - bias[i]))
        if baseline0 is None and all(c.baseline_rms for c in sim.ctl.channels):
            baseline0 = [float(c.baseline_rms) for c in sim.ctl.channels]
            if until_baseline:
                break
        if sim.t > 12:
            for i in range(4):
                peak[i] = max(peak[i], sim.diag_ratio[i])
        if stop_on_recover and recovered:
            break

    ratio = list(sim.diag_ratio)
    if not quiet:
        print(f"\n=== {label} ===")
        print(f"  state={sim.ctl.state}  faults={faults}  recovered={recovered}  "
              f"lock={'never' if sim.ctl.lock_time is None else f'{sim.ctl.lock_time:.1f}s'}")
        print("  ratio  " + "  ".join(f"ch{i}={ratio[i]:.3f}" for i in range(4)))
    return dict(ratio=ratio, peak=peak, faults=faults, recovered=recovered,
                lock=sim.ctl.lock_time, rail=rail_seen, frozen=frozen_on_rail,
                t_rail=t_rail, sim=sim, max_p=max_p, max_i=max_i, max_d=max_d,
                state=sim.ctl.state, sat_ever=sat_ever, out_dev=out_dev,
                reengaged=reengaged,
                pct_damping=100.0 * state_n.get('DAMPING', 0) / max(sum(state_n.values()), 1),
                rearms=rearms, t_demote=t_demote, t_rearm=t_rearm,
                min_actuating=min_actuating, dev_blind=dev_blind,
                demotes=int(getattr(sim.ctl, "demote_count", 0)),
                sat_flags=[bool(c.saturated_flag) for c in sim.ctl.channels],
                baseline=[float(c.baseline_rms or 0) for c in sim.ctl.channels],
                # falls back to the end-of-run value only if calibration never
                # completed, which is itself a failure the checks will show
                baseline0=(baseline0 if baseline0 is not None
                           else [float(c.baseline_rms or 0) for c in sim.ctl.channels]))


def suite(path):
    """Run every check against one version. Expectations come from the module."""
    info = describe(path)
    fixed = set(info["fixes"])
    if info["kind"] != "controller":
        phase(f"{'=' * 74}\n{info['name']}  ({info['tag']})   kind: {info['kind']}\n{'=' * 74}")
        note("  SKIPPED -- a bench measurement tool, not a controller. It closes no\n"
             "  loop and exposes no Controller, so there is nothing here to simulate.")
        return
    # The simulated plant is ONE rigid body carrying exactly four OSEMs -- GEOM
    # is four corners and SENSE is a 4x4 modal projection. A controller built for
    # a different channel count cannot be driven by it at all, and inventing the
    # extra geometry would mean asserting against made-up physics. Skip loudly
    # rather than fail, or pass on a fiction.
    n_ch = len(S.osem.ENABLE_CHANNEL)
    if n_ch != 4:
        phase(f"{'=' * 74}\n{info['name']}  ({info['tag']})   channels: {n_ch}\n{'=' * 74}")
        note(f"  SKIPPED -- this version has {n_ch} channels and sim/server.py models a "
             f"4-OSEM body.\n  It is a BENCH artefact: no simulator coverage exists for "
             f"it. See versions.md.")
        return
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
        # NOT "disabled channels stay at their baseline" any more. That assertion
        # encoded four independent oscillators; the plant is one rigid body, so
        # damping the driven corner takes energy out of the whole mass and every
        # OSEM sees it decay. provenance.md §3 measured exactly that on the
        # bench -- "although only one channel is being driven, all four visibly
        # decay" -- and it is why v1 turned the other three on. Measured here on
        # v0: driven ch0 = 0.197, undriven 0.265 / 0.323 / 0.355.
        #
        # What is still asserted, and what breaks it:
        #   * a disabled channel must never actuate -- fails the moment
        #     ENABLE_CHANNEL stops gating actuate(); measured, all four enabled
        #     instead moves the other three by 0.047-0.099 V off bias;
        #   * it must still decay well below its own baseline -- fails if the
        #     rigid-body coupling is lost or the loop stops removing energy;
        #   * but it must decay LESS than the driven one, because it can only
        #     lose energy indirectly through that coupling -- provenance.md section
        #     3 again, the targeted channel has "the best flatness".
        check("disabled channels never actuate -- output pinned to bias",
              all(a["out_dev"][i] < 1e-9 for i in off),
              " ".join(f"ch{i}={a['out_dev'][i]:.1e}V" for i in off))
        check("rigid body: undriven channels decay too, but less than the driven one",
              all(a["ratio"][i] < 0.75 for i in off) and bool(on)
              and min(a["ratio"][i] for i in off) > max(a["ratio"][i] for i in on),
              "driven " + " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in on)
              + " | undriven " + " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in off))
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
    # On the rigid-body plant this scenario trips on a PINNED ACTUATOR, not on a
    # runaway, and the distinction decides which defect it exercises. Inverting
    # ch2 does not make the mass unstable -- the other three still out-damp it
    # (sum of gain_i * COIL_GAIN_i goes from -0.1415 to -0.0419, so about a third
    # of the damping survives) -- it just leaves the optic near full amplitude,
    # where the velocity feedback demands more than the +-0.25 V it is clipped
    # to. Measured: v0/v1 trip at t=10.74 s on ch3 with sat_streak=31 and
    # saturated_flag set, v2 at t=17.23 s the same way. That is documented
    # defect #1, so v0/v1/v2 latch and only a version that fixed it recovers.
    # The pure-runaway path is exercised separately below.
    stuck_b = b["state"] == "FAULT" and not b["recovered"]
    if "saturation-latch" in fixed:
        check("FIXED saturation-latch: the sign-error trip clears and the loop "
              "recovers", b["recovered"] > 0, f"{b['recovered']} recovery(ies)")
    else:
        check("KNOWN BUG saturation-latch: the sign-error trip pins an actuator "
              "and latches", stuck_b and any(b["sat_ever"]),
              f"state={b['state']}, {b['recovered']} recovery(ies), "
              f"saturated={[i for i, s in enumerate(b['sat_ever']) if s]}")

    # A runaway with NO saturation, so the recovery path is tested on its own.
    # Only the wrong-signed channel drives, at half the shipped magnitude: that
    # halves the growth rate (so the 2 s RUNAWAY_SUSTAIN_S elapses before the
    # amplitude has run far past the 1.8x line) and doubles the amplitude the
    # actuator would need to clip, which is what keeps the two mechanisms apart.
    # Measured peak |out - bias| against the 0.25 V clip: v0/v1 0.112 V, v2
    # 0.057 V, v3 0.055 V -- nowhere near pinned, so this is unambiguously the
    # runaway breaker firing. Every version must recover from it, including
    # v0/v1/v2: saturated_flag is never set, so nothing holds all_clear False.
    # That is the exact claim README section 7, versions.md and versions.md hazard 1
    # all make -- "a runaway trip recovers fine; only saturation deadlocks" --
    # and for the un-fixed versions recovery is itself the proof that no
    # actuator pinned, since one that did would latch FAULT forever.
    solo = [False, False, True, False]
    weak = list(shipped)
    weak[2] = -abs(weak[2]) * 0.5
    r = run("ch2 alone, wrong-signed at half magnitude", 60, solo, weak,
            ki=info["ki"], kd=info["kd"], stop_on_recover=True)
    check("the runaway breaker trips on a pumped resonance", r["faults"] > 0,
          f"{r['faults']} fault(s), peak ch2 ratio={r['peak'][2]:.2f}")
    check("a runaway trip auto-recovers", r["recovered"] > 0,
          f"{r['recovered']} recovery(ies), peak |out-bias| = "
          f"{max(r['out_dev']):.3f} V of the 0.25 V clip")

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
    # DETECTION and RESPONSE are two claims, and v4 keeps the first while
    # deliberately inverting the second, so they can no longer be asserted as
    # one `rail and frozen`. Every version must still SEE the blind sensor.
    # `frozen` is also a coincidence detector -- "all four outputs within 1 uV
    # of bias at some sample" -- which a single locked, quiet channel satisfies
    # by accident, so for v4 the response is read off `faults` instead, which
    # is unambiguous.
    needs_freeze = "auto-disable" not in fixed
    check("the interlock arms on a blind sensor when the signal is quiet",
          all(r["rail"] for r in quiet_rows),
          " ".join("armed" if r["rail"] else "MISSED" for r in quiet_rows))
    if needs_freeze:
        check("blinding one OSEM freezes all four channels",
              all(r["frozen"] for r in quiet_rows))
    else:
        # ch1 is not even driving in this scenario (enable is [T,F,F,F]), so v4
        # demotes a channel that was already parked and the rig simply carries
        # on. v3 faulted the whole optic here.
        check("FIXED auto-disable: one blind OSEM no longer faults the whole rig",
              all(r["faults"] == 0 and r["state"] == "DAMPING" for _, r in rows),
              " ".join(f"{nv}mV:{r['faults']}f/{r['state']}" for nv, r in rows))
    if "rail-threshold" in fixed:
        check("FIXED rail-threshold: it still arms at a realistic noise floor",
              all(r["rail"] and (r["frozen"] or not needs_freeze) for r in loud_rows),
              f"threshold {S.osem.RAIL_LOW_COUNTS} counts, "
              f"{S.osem.RAIL_FRACTION:.0%} of a {S.osem.RAIL_SUSTAIN_S}s window")
    else:
        check("KNOWN GAP rail-threshold: it stops arming as noise approaches "
              "the 3-count line", all(not r["rail"] for r in loud_rows),
              "not silently patched -- see versions.md")

    # ---- 6. baseline robustness against a shock during calibration ----------
    # ONE deterministic velocity impulse inside the calibration window, SWEPT
    # across the window, because where it lands dominates the answer and a
    # single fixed shock time measures the sampling instant rather than the fix.
    # Where the shock lands dominates the answer, and on the old plant v0 could
    # look BETTER than v3 at some shock times -- so a one-shot comparison ranks
    # them backwards. What the median of sub-windows buys is a bound on the
    # damage wherever the transient lands, and the WORST case over the sweep is
    # what is asserted.
    #
    # The window swept is v0's 8 s one, which lies inside every longer window on
    # the ladder (v3+ calibrate for 20 s), so every version is handed the same
    # transient at the same absolute time. Measured worst case over t = 2/4/6 s:
    #     v0 / v1 / v2  (one 8 s window, RMS)          39.3%
    #     v3 / v4 / v5  (20 s, median of 5 windows)    19.7%
    # 35% separates them with ~1.1x / ~1.8x margin. Re-measured 2026-08-03 after
    # the simulator moved to the bench sample rate and gave scripted shocks their
    # own RNG; the previous 45.3% / 25.0% came from a run where the shock
    # direction was drawn from the shared noise stream and so depended on the
    # sample rate. Simulator numbers, never hardware.
    # Simulator numbers, never hardware.
    #
    # Both runs stop at the end of the first calibration and read baseline0, not
    # the end-of-run baseline: this is a claim about ONE calibration window, and
    # a version that faults and re-calibrates later would otherwise be scored on
    # a window this scenario never controlled.
    clean = run("no shock, reference baseline", 40, ship_en, shipped,
                ki=info["ki"], kd=info["kd"], until_baseline=True)
    base0 = clean["baseline0"]
    skew, worst_t = 0.0, None
    for t_shock in (2.0, 4.0, 6.0):
        sh = run(f"one shock at t={t_shock:.0f}s", 40, ship_en, shipped,
                 ki=info["ki"], kd=info["kd"], shock_at=(t_shock, 6.0),
                 until_baseline=True)
        base = sh["baseline0"]
        s = max(abs(base[i] / base0[i] - 1.0) for i in range(4) if base0[i] > 0)
        if s > skew:
            skew, worst_t = s, t_shock
    where = f"worst {skew * 100:.1f}% (shock at t={worst_t:.0f}s)"
    if "runaway-baseline" in fixed:
        check("FIXED runaway-baseline: wherever the shock lands, the median "
              "bounds the skew", skew < 0.35,
              f"{where}; v0 reaches 39.3% on the same sweep")
    else:
        check("KNOWN GAP runaway-baseline: one shock in the calibration window "
              "skews the baseline", skew > 0.35, where)

    # ---- 7. graceful degradation: a blind axis drops out, the rest carry on --
    # Only v4 claims this, and only v4 has the `healthy` attribute the run()
    # observations read, so the whole section is gated. Everything asserted here
    # is a behaviour v3 does NOT have: v3's `any_rail` freezes all four.
    #
    # All four occlusions land at t=30, comfortably past the longest calibration
    # on the ladder (v3/v4 are 20 s) and past lock, so these are claims about a
    # RUNNING loop losing an axis -- which is the case the version was written
    # for. A rail during calibration is a different scenario and is checked
    # separately at the bottom.
    if "auto-disable" in fixed:
        note("\n  auto-disable: losing axes from a running loop")

        # (a) one driving channel of four goes blind at t=30 and comes back at
        # t=45. The shock at t=31 is there so "the others kept damping" is a
        # measurement rather than an artefact of a quiet lab -- a locked, still
        # optic parks every output at bias whether or not the loop is live.
        g = run("ch1 blind t=30..45s, all four driving", 75, ship_en, shipped,
                ki=info["ki"], kd=info["kd"], occlusions=[(1, 30.0, 45.0)],
                shock_at=(31.0, 4.0))
        check("a blind axis is demoted, not escalated to a whole-rig fault",
              g["demotes"] >= 1 and g["faults"] == 0,
              f"{g['demotes']} demotion(s), {g['faults']} fault(s), "
              f"state={g['state']}")
        check("exactly one axis drops out -- the other three stay in the loop",
              g["min_actuating"] == 3, f"fell to {g['min_actuating']}/4")
        check("the demoted axis is parked at bias while it is blind",
              g["dev_blind"][1] < 1e-9, f"ch1 moved {g['dev_blind'][1]:.1e} V off bias")
        check("the surviving axes keep actuating while it is blind",
              all(g["dev_blind"][i] > 1e-3 for i in (0, 2, 3)),
              " ".join(f"ch{i}={g['dev_blind'][i]:.3f}V" for i in (0, 2, 3)))
        # The re-arm timer starts when the RAIL clears, not when the sensor
        # comes back, so the expected delay is the time for the rail window to
        # drain below RAIL_FRACTION plus REARM_SUSTAIN_S. Bounded rather than
        # pinned: the lower bound proves it waited out the filter transient
        # instead of snapping straight back in, the upper that it did come back.
        lo = S.osem.REARM_SUSTAIN_S
        hi = S.osem.REARM_SUSTAIN_S + S.osem.RAIL_SUSTAIN_S + 1.5
        back = None if g["t_rearm"] is None else g["t_rearm"] - 45.0
        check("it re-arms once the sensor returns, after the hold and not before",
              back is not None and lo <= back <= hi,
              f"{'never' if back is None else f'{back:.2f}s'} after un-blinding "
              f"(hold {lo:.1f}s, window allows {lo:.1f}-{hi:.1f}s)")

        # (b) the quorum, and several axes coming back at once. Three of four
        # blind from t=30 to t=45: one healthy channel is the documented floor,
        # because provenance.md section 3 measured a single OSEM/coil pair
        # damping the whole rigid body. Restoring all three together also checks
        # that re-arming is genuinely per-channel and not a single global latch
        # that happened to look right with one axis down in (a).
        q = run("three of four blind t=30..45s", 75, ship_en, shipped,
                ki=info["ki"], kd=info["kd"],
                occlusions=[(0, 30.0, 45.0), (1, 30.0, 45.0), (2, 30.0, 45.0)])
        check("three blind axes still do not fault the rig -- quorum of one",
              q["faults"] == 0 and q["min_actuating"] == S.osem.MIN_HEALTHY_CHANNELS,
              f"{q['faults']} fault(s), fell to {q['min_actuating']}/4, "
              f"MIN_HEALTHY_CHANNELS={S.osem.MIN_HEALTHY_CHANNELS}")
        check("the last healthy axis is still driving its coil",
              q["dev_blind"][3] > 1e-3 and all(q["dev_blind"][i] < 1e-9 for i in (0, 1, 2)),
              " ".join(f"ch{i}={q['dev_blind'][i]:.3f}V" for i in range(4)))
        check("all three come back independently once their sensors return",
              q["rearms"] == 3 and q["demotes"] == 3,
              f"{q['demotes']} demotion(s), {q['rearms']} re-arm(s)")

        # (c) and the floor is a floor: lose the last one and it IS a fault.
        # Without this, "never faults" would pass for a version that simply
        # deleted the interlock.
        z = run("all four blind from t=30s", 60, ship_en, shipped,
                ki=info["ki"], kd=info["kd"],
                occlusions=[(i, 30.0, None) for i in range(4)])
        check("losing the LAST axis does fault the rig",
              z["faults"] > 0 and z["state"] == "FAULT" and z["min_actuating"] == 0,
              f"{z['faults']} fault(s), state={z['state']}, "
              f"fell to {z['min_actuating']}/4")

        # (d) calibration is deliberately NOT degraded. You cannot measure a
        # baseline from a blind sensor, and a channel demoted mid-calibration
        # would re-arm later with no baseline at all -- so v4 keeps v3's global
        # fault here, and that is what makes a demoted channel's carried-forward
        # baseline safe to trust.
        c0 = run("ch1 blind from t=0, during calibration", 35, ship_en, shipped,
                 ki=info["ki"], kd=info["kd"], occlusions=[(1, 0.0, None)])
        check("a rail during CALIBRATION is still a whole-rig fault",
              c0["faults"] > 0 and c0["demotes"] == 0,
              f"{c0['faults']} fault(s), {c0['demotes']} demotion(s), "
              f"state={c0['state']}")

    # ---- 8. a disturbance that does not go away -----------------------------
    # Everything above faults on a TRANSIENT and recovers. This is the other
    # case: drive_amp 2.5 clips the ADC on every swing and keeps clipping, so the
    # fault recurs the moment the loop re-engages. Recovering by re-calibrating
    # then costs CALIBRATION_S of ZERO gain per attempt, during which the optic
    # rings straight back up to the amplitude that faulted it -- a supervisor
    # feedback loop that spends its life measuring instead of damping.
    #
    # Measured over 200 s at drive_amp 2.5, as % of samples in DAMPING:
    #     v0 / v1 / v2   0.6%   (they latch FAULT outright -- defect #1)
    #     v3 / v4        4.1%   7 faults, never locks
    #     v5            87.5%   1 fault, 1 straight-to-DAMPING recovery, locks
    # The margins either side of the lines below are ~2x and ~7x.
    hard = run("sustained disturbance, drive_amp=2.5", 200, ship_en, shipped,
               ki=info["ki"], kd=info["kd"], plant={"drive_amp": 2.5})
    if "fast-refault" in fixed:
        check("FIXED fast-refault: a repeat fault re-engages instead of re-calibrating",
              hard["reengaged"] > 0,
              f"{hard['reengaged']} straight-to-DAMPING recovery(ies), "
              f"{hard['faults']} fault(s)")
        check("FIXED fast-refault: it spends the run damping, not re-measuring",
              hard["pct_damping"] > 60.0,
              f"{hard['pct_damping']:.0f}% of samples in DAMPING, "
              f"lock {'%.1fs' % hard['lock'] if hard['lock'] else 'never'}")
    else:
        check("KNOWN GAP fast-refault: a sustained disturbance leaves it not damping",
              hard["pct_damping"] < 30.0 and hard["reengaged"] == 0,
              f"{hard['pct_damping']:.0f}% of samples in DAMPING, "
              f"{hard['faults']} fault(s), lock "
              f"{'%.1fs' % hard['lock'] if hard['lock'] else 'never'}")


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
        print(f"{'version':<10} {'on bench':<11} {'channels':<9} {'I/D':<8} fixes")
        for path in versions():
            d = describe(path)
            if d["kind"] != "controller":
                print(f"{d['name']:<10} {'bench tool':<11} {'-':<9} {'-':<8} "
                      f"{d['kind']}: actuation-matrix measurement")
                continue
            print(f"{d['name']:<10} {d['bench']:<11} {d['n_enabled']}/{d['n_channels']}       "
                  f"{'live' if any(d['ki']) or any(d['kd']) else 'zeroed':<8} "
                  f"{', '.join(d['fixes']) or '-'}")
        print("\n  validated = confirmed on hardware   reported = provenance.md only, this file untested")
        print("  untested  = never on hardware       broken   = does not damp, do not flash")
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
    info = describe(path)
    if info["kind"] != "controller":
        sys.exit(f"  {info['name']} is a bench measurement tool, not a controller -- "
                 f"there is no plant to serve it against.\n  Run it on hardware: "
                 f"make run V={info['tag']}")
    if info["n_channels"] != 4:
        sys.exit(f"  {info['name']} has {info['n_channels']} channels and sim/server.py "
                 f"models a 4-OSEM body.\n  Serving it crashes on the first request "
                 f"rather than telling you anything.\n  Run it on hardware: "
                 f"make run V={info['tag']}")
    if info["bench"] != "validated":
        print(f"  NOTE: {info['name']} is '{info['bench']}' on hardware -- fine in "
              f"here, see README.md before flashing it.", flush=True)
    S.load(path)
    return S.serve(args.port)


if __name__ == "__main__":
    sys.exit(main())
