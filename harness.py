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

import ladder

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim import server as S           # noqa: E402


# --------------------------------------------------------------------------
# version discovery
# --------------------------------------------------------------------------
def versions():
    """Every controller here, in ladder order, tools last.

    The name list is `ladder.LADDER` and this function derives nothing from the
    filename beyond looking it up there -- see ladder.py for why the order is an
    explicit tuple. `bench.py` calls the same helper, which is the point: the two
    used to carry a regex each and the copies drifted."""
    found = []
    for path in glob.glob(os.path.join(HERE, ladder.PREFIX + "*" + ladder.SUFFIX)):
        k = ladder.sort_key(os.path.basename(path))
        if k is not None:
            found.append((k, path))
    return [p for _, p in sorted(found)]


def resolve(name):
    """'delta', 'osem.delta', 'osem.delta.py' or a path -> an absolute path."""
    all_versions = versions()
    if not all_versions:
        sys.exit("No controllers found next to harness.py.")
    if name is None:
        # The sysid tool sorts after every rung, so the last file is not always
        # drivable -- a bare `make sim` or `make test` could land on something
        # the simulator refuses to run. Default to the newest thing it can
        # actually drive.
        drivable = simulatable()
        return drivable[-1] if drivable else all_versions[-1]
    if os.path.isfile(name):
        return os.path.abspath(name)
    stem = ladder.resolve_name(name)
    if stem:
        cand = os.path.join(HERE, ladder.filename(stem))
        if os.path.isfile(cand):
            return cand
    # Numbered names get redirected, not refused. Every log and docstring
    # written before the rename says "v12", and "no such version: v12" is a
    # true statement that helps nobody.
    new, was_numbered = ladder.redirect(name)
    if new:
        cand = os.path.join(HERE, ladder.filename(new))
        if os.path.isfile(cand):
            print(f"note: {name} is now `{new}` -- running that.")
            return cand
    if was_numbered:
        sys.exit(f"{name} was deleted, not renamed. It is in git history; "
                 f"versions.md says what it was. Have: "
                 + ", ".join(ladder.stem(os.path.basename(p)) for p in all_versions))
    sys.exit(f"No such version: {name}. Have: "
             + ", ".join(ladder.stem(os.path.basename(p)) for p in all_versions))


_DECL_RE = r'^%s\s*=\s*["\']([^"\']*)["\']'


def declared(path, name, default=""):
    """Read a string constant WITHOUT executing the module.

    Necessary, not merely cheap: `sim/server.py`'s load() derives constants from
    the module as it imports it and raises on anything that is not a controller,
    so a measurement tool cannot be introspected by loading it.
    """
    with open(path) as fh:
        src = fh.read()
    m = re.search(_DECL_RE % name, src, re.M)
    if m:
        return m.group(1)
    # Every controller declares `VERSION_TAG, BENCH_STATUS = "vN", "status"` on
    # one line, so the single-name pattern above misses both of them. It only
    # ever mattered for files that get loaded anyway; a skeleton is read from
    # source alone, so the tuple form has to be readable too.
    m = re.search(r'^([A-Z_]+)\s*,\s*([A-Z_]+)\s*=\s*'
                  r'["\']([^"\']*)["\']\s*,\s*["\']([^"\']*)["\']', src, re.M)
    if m and name in (m.group(1), m.group(2)):
        return m.group(3) if name == m.group(1) else m.group(4)
    return default


# The channel counts sim/server.py has a measured body for. Four is the
# 2026-08-03 rig; eight is the 2026-08-04/06 one. Anything else has no geometry
# behind it, and inventing some would mean asserting against fabricated physics.
SIM_CHANNELS = (4, 8)


def simulatable():
    """The versions the simulator can actually drive: real controllers whose
    channel count matches a body sim/server.py has measurements for.
    `versions()` still returns everything, because --list should show the bench
    tools too even though nothing here can run them."""
    return [p for p in versions()
            if declared(p, "KIND", "controller") == "controller"
            and describe(p)["n_channels"] in SIM_CHANNELS]


def describe(path):
    """osem.sysid.py is a measurement tool, not a controller -- it declares
    KIND = "sysid", close no loop and expose no Controller. They are described
    from source text and never handed to the simulator."""
    name = os.path.basename(path)[:-3]
    kind = declared(path, "KIND", "controller")
    if kind != "controller":
        # Read the status out of the source too. Hardcoding "bench tool" was
        # right while `sysid` was the only non-controller kind; a `skeleton`
        # is not a bench tool and saying so in the table is the whole point.
        return dict(path=path, name=name, kind=kind,
                    bench=declared(path, "BENCH_STATUS", "bench tool"),
                    tag=declared(path, "VERSION_TAG", name[5:]), fixes=(),
                    n_enabled=0, n_channels=0, ki=[], kd=[], steady=[])
    osem = S.load(path)
    enable = list(getattr(osem, "ENABLE_CHANNEL", []))
    return dict(
        path=path,
        name=os.path.basename(path)[:-3],
        tag=getattr(osem, "VERSION_TAG", "?"),
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
#
# `f_drive` is NOT in here, and `line_scale` is. The coherent lab tone has to
# sit on the body's own resonance or it excites nothing -- 1.0 Hz is the
# four-OSEM body's first mode and 1.046 Hz is the eight-OSEM body's, so it comes
# from S.DRIVE_HZ per body rather than being pinned here. `line_scale` is pinned
# because the out-of-band interference on a4/a6/a7 was measured 4-20x louder in
# one session than the other, and a suite should not silently inherit whichever
# a previous scenario left behind.
BASE_PLANT = dict(ack_drain=0.0,
                  drive_amp=0.8, seismic=0.8, Q=50.0, k_act=20.0,
                  meas_noise=20.0, hum_hz=60.0, hum_mv=0.0, hvac_hz=2.5,
                  hvac_amp=0.0, shock_rate=0.0, shock_amp=3.0, line_scale=1.0)


def run(label, seconds, enable, steady, ki=None, kd=None, occlude_at=None,
        auto_kick=False, plant=None, quiet=True, shock_at=None,
        stop_on_recover=False, until_baseline=False, occlusions=None,
        sensor_gain=None):
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
    counterfactual. A LIST of `(t, dv)` pairs, in time order, schedules several:
    the baseline-reuse checks need a second kick after the loop has recovered
    from the first, because "it re-engaged" and "the interlocks still work
    against the baseline it re-engaged on" are separate claims.

    `occlude_at` is `(idx, t)`: blind one OSEM from `t` to the end of the run.
    `occlusions` is the general form, `[(idx, t_on, t_off_or_None), ...]`, and
    exists because v4's auto-disable is only half-testable without an un-blind:
    "it drops the axis" and "it puts the axis back" are separate claims, and the
    second one needs the sensor to come back. Several entries can overlap, which
    is how the quorum is exercised.

    `sensor_gain` is one counts-per-metre multiplier per channel, default all
    1.0. It is the OTHER sensor failure -- mis-alignment rather than occlusion.
    An occluded OSEM pins at a rail and the rail interlock sees it; a mis-aligned
    one keeps reporting a healthy DC level and a healthy noise floor with the
    optic simply absent from it, and NOTHING in v9 looks for that. 0.0 is
    a4/a6/a7 on the bench 2026-08-06, 1/12 is a5. On the eight-OSEM body those
    channels are already like that without any injection -- it is measured
    geometry there, not a fault -- so this is mostly a four-channel tool.
    See Sim.sens_gain.
    """
    sim = S.SIM
    nch = S.N          # channels on the body this controller loaded
    sim.enable = [bool(v) for v in enable]
    sim.steady = list(steady)
    sim.capture = [g * 1.167 for g in steady]
    sim.ki = list(ki or [0.0] * nch)
    sim.kd = list(kd or [0.0] * nch)
    sim.occlude = [False] * nch
    sim.sens_gain = [1.0] * nch if sensor_gain is None else [float(g) for g in sensor_gain]
    sim.auto_kick = auto_kick
    sim.plant.update(BASE_PLANT)
    # On the body's own resonance unless a scenario says otherwise; see
    # BASE_PLANT. Set explicitly rather than left over from the last run.
    sim.plant["f_drive"] = S.DRIVE_HZ
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
    peak = [0.0] * nch
    t_rail = None
    # one (t, dv) pair or a list of them, consumed in time order
    sched = ([] if shock_at is None
             else [tuple(shock_at)] if not isinstance(shock_at[0], (list, tuple))
             else [tuple(s) for s in shock_at])
    # Baselines as they were at the END OF THE FIRST CALIBRATION. Reading
    # `Channel.baseline_rms` after the run instead reports whatever the LAST
    # calibration produced, which for a version that faulted and re-calibrated
    # mid-run is a different window under different conditions -- that is what
    # once made a version read 58.6% on the calibration-skew check while running
    # v3's calibration code byte for byte.
    baseline0 = None
    # When that first calibration ENDED. `CALIBRATION_S` is a fixed duration on
    # v4/v5 and a ceiling on a version claiming `fast-calib`, so how long
    # calibration actually took is an observation now, not a constant.
    t_calib = None
    # saturated_flag anywhere, any sample: a saturation trip is what latches
    # FAULT in v0/v1/v2, so "did this scenario saturate?" decides whether it is
    # testing the runaway path or the saturation path.
    sat_ever = [False] * nch
    # Peak |out - bias| per channel, against the +-0.25 V the controller clips
    # to. Two uses: proving a disabled channel never actuates, and proving a
    # runaway scenario tripped on amplitude rather than on a pinned actuator.
    # Each channel's CURRENT bias, not the module constant. `bias-trim` moves it
    # one 0.25 V quantum at a time, so on v7/v10/v11 "did this channel actuate?"
    # asked against BIAS reports a whole trim step as actuation and a parked
    # channel looks like it is still driving. Read per sample, because a trim can
    # land mid-run. Versions without the trim have no `ctl.bias` and fall back to
    # the constant, which is what they never move off.
    bias = [float(b) for b in S.osem.BIAS]
    out_dev = [0.0] * nch
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
    prev_healthy = [True] * nch
    t_engaged = None         # when the loop first reached DAMPING
    rearms = 0
    t_demote = t_rearm = None
    min_actuating = nch
    # Peak |out - bias| per channel WHILE some channel is demoted. The 0.5 s
    # offset from t_demote skips the slew: MAX_SLEW_PER_S is 2.0 V/s, so a
    # channel dropping from a 0.25 V excursion needs 0.125 s to reach bias and
    # would otherwise look like it kept actuating after being dropped.
    dev_blind = [0.0] * nch
    # --- baseline-floor observations ----------------------------------------
    # Whether each channel was EVER demoted for calibrating an implausible
    # baseline, which is a different demotion from the rail one and has to be
    # counted separately: it is decided once at the end of calibration and is
    # not visible in `demotes`, which counts rails. Versions without the fix have
    # no `floor_bad` at all, so this stays all-False for them and the checks
    # below read as "nothing demoted it", which is exactly the claim.
    floor_ever = [False] * nch
    # v0's check_rail prints on every railed sample with no latch, which floods
    # the console during the occlusion sweep. The rail_fault flag is what the
    # assertions read, so swallow it.
    sink = io.StringIO()
    for n in range(int(seconds * S.SAMPLE_HZ)):
        if occlude_at is not None and sim.t >= occlude_at[1]:
            sim.occlude[occlude_at[0]] = True
        for oi, t_on, t_off in occlusions:
            sim.occlude[oi] = sim.t >= t_on and (t_off is None or sim.t < t_off)
        while sched and sim.t >= sched[0][0]:
            sim.kick(sched.pop(0)[1])
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
        live_bias = getattr(sim.ctl, "bias", None)
        if live_bias is not None:
            bias = [float(b) for b in live_bias]
        for i, c in enumerate(sim.ctl.channels):
            if c.saturated_flag:
                sat_ever[i] = True
            out_dev[i] = max(out_dev[i], abs(float(c.out) - bias[i]))

        fb = getattr(sim.ctl, "floor_bad", None)
        if fb is not None:
            floor_ever = [a or bool(b) for a, b in zip(floor_ever, fb)]

        chans = sim.ctl.channels
        healthy = [bool(getattr(c, "healthy", True)) for c in chans]
        # `baseline-floor` demotes at the END OF CALIBRATION, in the same step
        # that enters DAMPING. That is a config decision, not an axis lost from a
        # running loop, and counting it starts `t_demote` at t ~ 20 s -- after
        # which dev_blind accumulates through the whole healthy stretch and every
        # channel looks like it kept driving while blind. So the demotion clock
        # starts from the channel set the loop actually ENGAGES with. Nothing
        # changes on a body with no signal-less channels: the set is all-True
        # there and this is a no-op.
        if st == "DAMPING" and t_engaged is None:
            t_engaged = sim.t
        if t_engaged is not None and sim.t < t_engaged + 1.0:
            prev_healthy = healthy
        for i in range(nch):
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
        # Only WHILE A SENSOR IS ACTUALLY BLINDED. It used to be "while any
        # channel is unhealthy", which on the eight-OSEM body is the whole run:
        # a4/a6/a7 are demoted by `baseline-floor` at calibration and never come
        # back, so every channel's post-re-arm driving was being counted as
        # driving-while-blind. Unchanged on a body where the only demotions are
        # the scripted occlusions.
        if st == "DAMPING" and any(sim.occlude):
            if not all(healthy) and t_demote is not None and sim.t >= t_demote + 0.5:
                for i, c in enumerate(chans):
                    dev_blind[i] = max(dev_blind[i], abs(float(c.out) - bias[i]))
        if baseline0 is None and all(c.baseline_rms for c in sim.ctl.channels):
            baseline0 = [float(c.baseline_rms) for c in sim.ctl.channels]
            t_calib = sim.t
            if until_baseline:
                break
        if sim.t > 12:
            for i in range(nch):
                peak[i] = max(peak[i], sim.diag_ratio[i])
        if stop_on_recover and recovered:
            break

    ratio = list(sim.diag_ratio)
    if not quiet:
        print(f"\n=== {label} ===")
        print(f"  state={sim.ctl.state}  faults={faults}  recovered={recovered}  "
              f"lock={'never' if sim.ctl.lock_time is None else f'{sim.ctl.lock_time:.1f}s'}")
        print("  ratio  " + "  ".join(f"ch{i}={ratio[i]:.3f}" for i in range(nch)))
    return dict(ratio=ratio, peak=peak, faults=faults, recovered=recovered,
                lock=sim.ctl.lock_time, rail=rail_seen, frozen=frozen_on_rail,
                t_rail=t_rail, sim=sim, max_p=max_p, max_i=max_i, max_d=max_d,
                state=sim.ctl.state, sat_ever=sat_ever, out_dev=out_dev,
                reengaged=reengaged,
                pct_damping=100.0 * state_n.get('DAMPING', 0) / max(sum(state_n.values()), 1),
                t_calib=t_calib,
                rearms=rearms, t_demote=t_demote, t_rearm=t_rearm,
                floor=floor_ever,
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
    # The simulated plant is ONE rigid body, and sim/server.py has a MEASURED
    # geometry for two of them: the four-OSEM rig of 2026-08-03 and the
    # eight-OSEM rig of 2026-08-04/06. A controller built for any other channel
    # count cannot be driven by either, and inventing the extra geometry would
    # mean asserting against made-up physics. Skip loudly rather than fail, or
    # pass on a fiction.
    n_ch = len(S.osem.ENABLE_CHANNEL)
    if n_ch not in SIM_CHANNELS:
        phase(f"{'=' * 74}\n{info['name']}  ({info['tag']})   channels: {n_ch}\n{'=' * 74}")
        note(f"  SKIPPED -- this version has {n_ch} channels and sim/server.py has a "
             f"measured body for {' or '.join(map(str, SIM_CHANNELS))}.\n  It is a BENCH "
             f"artefact: no simulator coverage exists for it. See versions.md.")
        return
    shipped = info["steady"]
    on = [i for i in range(n_ch) if S.osem.ENABLE_CHANNEL[i]]
    off = [i for i in range(n_ch) if not S.osem.ENABLE_CHANNEL[i]]
    ship_en = [bool(v) for v in S.osem.ENABLE_CHANNEL]

    # --- what the PLANT says about each channel, as distinct from the config --
    # `enabled` is the controller's opinion. These three come from the measured
    # body and are what makes the eight-channel case different in kind rather
    # than merely in size:
    #
    #   sensing   the channel is coupled to the optic at all. a4/a6/a7 are not:
    #             measured 0.1-9% of their power in the 0.4-3 Hz loop band,
    #             lock-in SNR 1.1-1.5, bandpassed RMS flat through 40 s of
    #             damping. Every check about DAMPING has to be asked of the
    #             channels that can damp; asking it of these measures nothing.
    #   live      enabled AND sensing -- the loops that actually exist.
    #   weak      sensing, but at a small fraction of the others'
    #             counts-per-metre. a5 is measured at ~1/12 of a0. Such a
    #             channel damps the optic perfectly well and STILL cannot pass a
    #             per-channel ratio test, because ratio is against its own
    #             floor. v10 measured exactly that on hardware.
    strength = [math.sqrt(sum(v * v for v in S.SHAPE[i])) for i in range(n_ch)]
    sensing = [i for i in range(n_ch) if strength[i] > 0]
    typical = sorted(strength[i] for i in sensing)[len(sensing) // 2]
    weak = [i for i in sensing if strength[i] < 0.5 * typical]
    live = [i for i in on if i in sensing]
    solid = [i for i in live if i not in weak]      # enabled, sensing, full gain
    dead_on = [i for i in on if i not in sensing]

    # --- what a scenario gain MEANS on this body -----------------------------
    # Several scenarios below name a gain: -0.600 for "over-gain", half the
    # shipped magnitude for "wrong-signed but weakly". Those numbers were chosen
    # against the four-OSEM body, where ch0's coil moves its own OSEM by
    # k_act * 1.0 per volt. The eight-OSEM body's coil authority is not a free
    # parameter -- it is the part of the measured DC matrix that lies in the two
    # modes the loop can see, which for ch0 is 0.30 of the DC total -- so the
    # SAME Kp is a 3.3x weaker loop there. (That is the more faithful number, not
    # a defect: at Kp = -0.030 it gives ch0 a total decay rate of 0.157 s^-1
    # against the 0.191 measured on the bench, where the four-OSEM body gives
    # 0.363.) Scenario gains are therefore quoted against ch0's authority and
    # rescaled per body, so "over-gain" means over-gain on both.
    #
    # AUTH_REF is the four-OSEM body's value, so this is exactly 1.0 there --
    # exactly, including in floating point, since x * (20.0 / 20.0) is x.
    # Per channel, because the two bodies do not merely differ by a scale: ch2's
    # authority is 4.15x ch0's on the four-OSEM body (inferred from provenance.md
    # Table 1's damping rates) and 2.58x here (measured from the DC matrix), so
    # one factor cannot carry both.
    _ref = S._BODY4

    def _auth(coil, act, shape, i):
        return S.K_ACT_REF * abs(coil[i] * act[0][i] * shape[i][0])

    gscale = [(_auth(_ref["COIL_GAIN"], _ref["ACT"], _ref["SHAPE"], i)
               / _auth(S.COIL_GAIN, S.ACT, S.SHAPE, i)) if i < 4 else 1.0
              for i in range(n_ch)]
    if any(abs(g - 1.0) > 1e-12 for g in gscale):
        note("  scenario gains x" + " ".join(f"ch{i}:{g:.2f}" for i, g in
                                            enumerate(gscale) if i < 4)
             + " -- this body's in-band coil authority against the four-OSEM "
               "body the literals below were chosen at")

    phase(f"{'=' * 74}\n{info['name']}  ({info['tag']})   "
          f"channels on: {len(on)}/{n_ch}   "
          f"I/D: {'live' if any(info['ki']) or any(info['kd']) else 'zeroed'}   "
          f"fixes: {', '.join(info['fixes']) or 'none'}\n{'=' * 74}")
    if len(sensing) != n_ch or weak:
        note(f"  plant: {len(sensing)}/{n_ch} OSEMs see the optic"
             + (f"; ch{','.join(map(str, weak))} at ~1/{round(typical / strength[weak[0]])}"
                f" of the others' counts-per-metre" if weak else "")
             + (f"; ch{','.join(str(i) for i in range(n_ch) if i not in sensing)}"
                f" read out-of-band interference and no motion" if len(sensing) != n_ch else ""))

    # ---- 1. the shipped configuration damps what it drives -----------------
    a = run("as shipped", 60, ship_en, shipped, ki=info["ki"], kd=info["kd"])
    check("every enabled channel damps below the lock threshold",
          all(a["ratio"][i] < 0.40 for i in solid),
          " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in solid))
    if weak and set(weak) & set(on):
        # The bench result this reproduces, from v10's own docstring: "a5 reads
        # ratio 0.45 and so the rig does not announce LOCKED". Nothing is wrong
        # with the channel -- its ABSOLUTE envelope is comparable to ch0's and
        # falling. `ratio` is env over that channel's OWN baseline, and a5's
        # baseline is a twelfth of everyone else's, so the same residual motion
        # reads as a much larger ratio. Asserted from the limited side, like
        # every other known defect here, so that a version which fixes it (by
        # realigning the flag, or by weighting the lock claim) has to say so.
        w0 = weak[0]
        check("KNOWN LIMIT: a low-gain OSEM in the loop holds the rig out of "
              "LOCKED however well the optic is damped",
              a["lock"] is None and a["ratio"][w0] > 0.40 and a["faults"] == 0,
              f"ch{w0} ratio {a['ratio'][w0]:.2f} against the "
              f"{S.osem.LOCK_RMS_FACTOR} lock line while "
              + " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in solid)
              + f"; {a['faults']} fault(s)")
    else:
        check("the run reports LOCKED", a["lock"] is not None,
              f"{a['lock']:.1f}s" if a["lock"] else "never")
    if dead_on:
        # An enabled channel the plant gives no motion to. It cannot damp and it
        # cannot be asked to; what it must NOT do is fault the rig. That is the
        # whole v5.5 failure and it is checked properly in section 7b.
        check("an enabled channel that senses nothing does not fault the rig",
              a["faults"] == 0 and a["state"] == "DAMPING",
              f"ch{','.join(map(str, dead_on))} enabled with no motion coupling, "
              f"{a['faults']} fault(s), state={a['state']}")
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
        # Restricted to the undriven channels that WATCH the optic. On the
        # eight-OSEM body the off list is a4/a6/a7, which are not coupled to it
        # at all -- their bandpassed RMS is flat through a ringdown that takes a
        # factor of 3-12 out of every real channel -- so "it decayed too" is not
        # a claim about them and asserting it would be asserting noise.
        quiet = [i for i in off if i in sensing]
        if quiet and solid:
            check("rigid body: undriven channels decay too, but less than the driven one",
                  all(a["ratio"][i] < 0.75 for i in quiet)
                  and min(a["ratio"][i] for i in quiet) > max(a["ratio"][i] for i in solid),
                  "driven " + " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in solid)
                  + " | undriven " + " ".join(f"ch{i}={a['ratio'][i]:.3f}" for i in quiet))
    check("no faults in a quiet lab", a["faults"] == 0, f"{a['faults']}")

    # ---- 2. a wrong-signed gain is caught by the interlock ------------------
    # ch2 is inverted in hardware, so NEGATING its gain is the sign error here.
    bad = list(shipped)
    bad[2] = -abs(bad[2]) * gscale[2]
    # Every channel the plant actually couples to the optic, so the scenario is
    # "three good loops against one wrong-signed one" on either body rather than
    # "however many this version happens to ship".
    all_sensing = [i in sensing for i in range(n_ch)]
    b = run("ch2 gain sign inverted", 120, all_sensing, bad,
            ki=info["ki"], kd=info["kd"])
    if n_ch == 4:
        check("a wrong-signed channel is pumped past the runaway line",
              b["peak"][2] > 1.8, f"ch2 peak={b['peak'][2]:.2f}")
    else:
        # Same claim, measured rather than compared against a four-OSEM number.
        # RUNAWAY_MULTIPLE is a ratio against a baseline, and a version that
        # faults on SATURATION first re-calibrates before the ratio can climb --
        # so on a body whose coils are weaker the trip happens with the envelope
        # only a few times its floor. What is body-independent is that flipping
        # the sign makes that channel far worse than it is when shipped.
        check("a wrong-signed channel turns a fault-free run into a repeatedly "
              "faulting one",
              b["faults"] >= 3 and a["faults"] == 0,
              f"{b['faults']} fault(s) inverted against {a['faults']} as shipped; "
              f"ch2 peak {b['peak'][2]:.2f} -- the envelope RATIO stays low here "
              f"because the trip comes before it can climb, not because the "
              f"channel is behaving")
    check("the global interlock trips on it", b["faults"] > 0, f"{b['faults']} fault(s)")
    # The safety complement to the soft-saturation check further down, and the
    # reason that one is not just "the interlock was switched off". This scenario
    # ALSO clips -- see the note below -- but here the optic sits near full
    # amplitude instead of coming down, so the envelope is not falling and the
    # fault must still fire. Clipped-and-winning keeps damping; clipped-and-not-
    # winning still faults. Both halves are asserted, on two different runs.
    if "soft-saturation" in fixed:
        check("soft-saturation did NOT disable the interlock: clipped and NOT "
              "winning still faults",
              b["faults"] > 0 and any(b["sat_ever"]),
              f"{b['faults']} fault(s) with the actuator pinned, ch2 peak="
              f"{b['peak'][2]:.2f}")
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
    solo = [i == 2 for i in range(n_ch)]
    half = list(shipped)
    half[2] = -abs(half[2]) * 0.5 * gscale[2]
    r = run("ch2 alone, wrong-signed at half magnitude", 60, solo, half,
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
    over[0] = -0.600 * gscale[0]
    only0 = [i == 0 for i in range(n_ch)]
    # ch0 ALONE on the four-OSEM body, where the other three are ordinary OSEMs
    # sitting idle. On the eight-OSEM body that same configuration leaves a5 --
    # real, weak, and NOT enabled, so undamped -- inside the runaway breaker with
    # nothing holding it, and the rig trips on a5 rather than on ch0's clipping,
    # which is a different scenario wearing this one's name. So there the rig
    # runs as SHIPPED with ch0 over-gained on top.
    over_en = only0 if n_ch == 4 else ship_en
    d = run("over-gain ch0 + a kick", 90, over_en, over,
            ki=info["ki"], kd=info["kd"], auto_kick=True)
    # What this scenario actually produces is a CLIPPED loop, not a diverging
    # one: the simulated plant is linear apart from the ADC, so -0.600 damps
    # monotonically harder than -0.030 (versions.md: swept to -0.600, zero
    # faults). Up to v9 the rig faulted here anyway, on the 30-consecutive-pinned
    # streak alone. `soft-saturation` asks the second question -- is the clipped
    # output winning? -- and here it is, so v10 keeps damping. Measured on this
    # exact run: v9 faults once and peaks at 1.527 with 87.8% of samples in
    # DAMPING; v10 never faults, peaks at 0.853, and reaches 93.3%. Freezing a
    # winning actuator let the optic ring 1.8x higher. Both versions really did
    # clip (sat_ever), so this is not a scenario that stopped saturating.
    if "soft-saturation" in fixed and n_ch == 4:
        check("FIXED soft-saturation: a clipped actuator that is still WINNING "
              "keeps damping instead of faulting",
              d["faults"] == 0 and any(d["sat_ever"]) and d["ratio"][0] < 0.35,
              f"{d['faults']} fault(s), saturated={any(d['sat_ever'])}, "
              f"ch0 ratio {d['ratio'][0]:.3f} vs the 0.35 lock line, "
              f"peak {d['peak'][0]:.3f}, {d['pct_damping']:.1f}% damping")
    elif "soft-saturation" in fixed:
        # "Zero faults" is a four-OSEM claim: there, every other channel is a
        # healthy idle OSEM. Here the rig runs as shipped and a5 is in the
        # interlocks with a floor that is mostly its own noise, so it can trip
        # for reasons that have nothing to do with ch0's rail. What the fix
        # itself has to show is that the CLIPPED channel keeps winning and the
        # run keeps damping -- a frozen actuator does neither.
        check("FIXED soft-saturation: the clipped actuator keeps winning and the "
              "run keeps damping",
              any(d["sat_ever"]) and d["ratio"][0] < 0.35
              and d["pct_damping"] > 60.0,
              f"ch0 ratio {d['ratio'][0]:.3f} vs the 0.35 lock line while pinned, "
              f"peak {d['peak'][0]:.3f}, {d['pct_damping']:.1f}% of samples "
              f"damping, {d['faults']} fault(s) from the rest of the rig")
    else:
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
        r = run(f"blind ch1 @ {nv} mV", 45, only0, shipped,
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
    took = {}
    for t_shock in (2.0, 4.0, 6.0):
        sh = run(f"one shock at t={t_shock:.0f}s", 40, ship_en, shipped,
                 ki=info["ki"], kd=info["kd"], shock_at=(t_shock, 6.0),
                 until_baseline=True)
        base = sh["baseline0"]
        took[t_shock] = sh["t_calib"]
        # Only over the channels whose baseline is the OPTIC. A channel that
        # measures its own noise floor has a baseline made of that floor, and a
        # low-gain one is mostly its own floor too (a5 is measured at ~70% own
        # content), so a window that ends at a different instant re-rolls it.
        # Including those would report window placement, not the shock.
        s = max(abs(base[i] / base0[i] - 1.0) for i in solid if base0[i] > 0)
        if s > skew:
            skew, worst_t = s, t_shock
    where = f"worst {skew * 100:.1f}% (shock at t={worst_t:.0f}s)"
    if n_ch != 4:
        # The 35% line is a FOUR-OSEM number: it separates v0's 39.3% from
        # v3+'s 19.7% on that body's sweep, and both halves of that comparison
        # were measured there. No v0 exists for this body, so the separation has
        # never been measured on it and asserting the same line would be
        # borrowing a threshold across a plant change. What is still asserted is
        # that the estimator does not come apart -- and the number is printed so
        # a future v0-equivalent run has something to be compared against.
        check("runaway-baseline: a shock in the window skews the floor, bounded "
              "-- the 35% line is a four-OSEM figure and is NOT applied here",
              skew < 1.0, where + "; no same-body reference measurement exists")
    elif "runaway-baseline" in fixed:
        check("FIXED runaway-baseline: wherever the shock lands, the median "
              "bounds the skew", skew < 0.35,
              f"{where}; v0 reaches 39.3% on the same sweep")
    else:
        check("KNOWN GAP runaway-baseline: one shock in the calibration window "
              "skews the baseline", skew > 0.35, where)

    # ---- 6b. what that calibration COSTS ------------------------------------
    # Time-to-lock is calibration + lock-after-gain, and on v4/v5 the first term
    # is a fixed CALIBRATION_S = 20 s with the gain forced to zero -- about 65%
    # of the wall clock, measured on the bench 2026-08-03/04 (lock at t ~ 31 s on
    # four channels, 20 s of it calibrating). `fast-calib` makes CALIBRATION_S a
    # CEILING: sub-windows accumulate and calibration ends as soon as the
    # trailing ones say the floor has stopped moving.
    #
    # Both runs are already in hand from the sweep above, so these two checks are
    # free. They are the two halves of the claim and neither is sufficient alone:
    # stopping early is worthless if it stops early on a transient, and refusing
    # to stop early is what v4/v5 already do.
    calib_s = float(S.osem.CALIBRATION_S)
    if "fast-calib" in fixed and len(sensing) == n_ch and not weak:
        check("FIXED fast-calib: a quiet lab does not pay the whole calibration "
              "window", clean["t_calib"] is not None and clean["t_calib"] <= calib_s * 0.5,
              f"{clean['t_calib']:.2f}s of a {calib_s:.0f}s ceiling"
              if clean["t_calib"] else "never calibrated")
    elif "fast-calib" in fixed:
        # THE BENCH ALREADY FOUND THIS, and the eight-OSEM body reproduces it.
        # `fast-calib` ends calibration early only when the trailing sub-window
        # floors agree to within CALIB_AGREE_TOL across EVERY channel. Put a
        # channel in the set whose floor is its own narrowband noise rather than
        # the optic and they never agree, so it runs the ceiling. On hardware
        # 2026-08-04 v8 printed exactly that -- "ran the full 20 s ceiling, the
        # floor never settled" -- and versions.md records the 20 s -> 6 s saving
        # as simulator-only and unreproduced. What must still hold is the
        # designed fallback: it degrades to v5's estimator and still calibrates,
        # rather than hanging or exiting on a transient.
        check("KNOWN LIMIT fast-calib: with a channel whose floor is not the "
              "optic it pays the whole ceiling, and degrades safely to it",
              clean["t_calib"] is not None and clean["t_calib"] >= calib_s - 0.2,
              f"{clean['t_calib']:.2f}s of a {calib_s:.0f}s ceiling -- same "
              f"'floor never settled' the bench saw on v8, 2026-08-04"
              if clean["t_calib"] else "never calibrated")
    else:
        check("calibration always costs the full CALIBRATION_S",
              clean["t_calib"] is not None and clean["t_calib"] >= calib_s - 0.2,
              f"{clean['t_calib']:.2f}s of {calib_s:.0f}s"
              if clean["t_calib"] else "never calibrated")
    # The safety half, asserted for EVERY version: a shock inside the window must
    # never be mistaken for a settled noise floor. t = 2 s and t = 4 s are the two
    # sweep points that land before the earliest exit any version can take, so
    # this is the same transient in the same place for all of them. A version
    # that stopped early here would be storing the ringdown as its floor, which
    # de-sensitises the runaway breaker AND makes LOCKED easier to declare.
    early_on_shock = [t for t in (2.0, 4.0)
                      if took[t] is None or took[t] < calib_s - 0.2]
    check("a transient inside the window is never read as a settled floor",
          not early_on_shock,
          " ".join(f"shock@{t:.0f}s -> {took[t]:.2f}s" for t in (2.0, 4.0)
                   if took[t] is not None)
          + f" of a {calib_s:.0f}s window")

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
        # How many loops the plant leaves standing once calibration has thrown
        # out the channels that measure nothing. On the four-OSEM body that is
        # simply the enabled set; on the eight-OSEM one it is the enabled set
        # minus a4/a6/a7, which `baseline-floor` demotes before DAMPING starts.
        n_live = len(live)
        others = [i for i in live if i != 1]

        # (a) one driving channel goes blind at t=30 and comes back at t=45. The
        # shock at t=31 is there so "the others kept damping" is a measurement
        # rather than an artefact of a quiet lab -- a locked, still optic parks
        # every output at bias whether or not the loop is live.
        g = run(f"ch1 blind t=30..45s, {n_live} loops driving", 75, ship_en, shipped,
                ki=info["ki"], kd=info["kd"], occlusions=[(1, 30.0, 45.0)],
                shock_at=(31.0, 4.0))
        check("a blind axis is demoted, not escalated to a whole-rig fault",
              g["demotes"] >= 1 and g["faults"] == 0,
              f"{g['demotes']} demotion(s), {g['faults']} fault(s), "
              f"state={g['state']}")
        check("exactly one axis drops out -- the rest stay in the loop",
              g["min_actuating"] == n_live - 1,
              f"fell to {g['min_actuating']}/{n_live}")
        check("the demoted axis is parked at bias while it is blind",
              g["dev_blind"][1] < 1e-9, f"ch1 moved {g['dev_blind'][1]:.1e} V off bias")
        check("the surviving axes keep actuating while it is blind",
              all(g["dev_blind"][i] > 1e-3 for i in others),
              " ".join(f"ch{i}={g['dev_blind'][i]:.3f}V" for i in others))
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
        blinded, last = live[:-1], live[-1]
        q = run(f"{len(blinded)} of {n_live} blind t=30..45s", 75, ship_en, shipped,
                ki=info["ki"], kd=info["kd"],
                occlusions=[(i, 30.0, 45.0) for i in blinded])
        check(f"{len(blinded)} blind axes still do not fault the rig -- quorum of one",
              q["faults"] == 0 and q["min_actuating"] == S.osem.MIN_HEALTHY_CHANNELS,
              f"{q['faults']} fault(s), fell to {q['min_actuating']}/{n_live}, "
              f"MIN_HEALTHY_CHANNELS={S.osem.MIN_HEALTHY_CHANNELS}")
        check("the last healthy axis is still driving its coil",
              q["dev_blind"][last] > 1e-3
              and all(q["dev_blind"][i] < 1e-9 for i in range(n_ch) if i != last),
              " ".join(f"ch{i}={q['dev_blind'][i]:.3f}V" for i in range(n_ch)))
        check("all of them come back independently once their sensors return",
              q["rearms"] == len(blinded) and q["demotes"] == len(blinded),
              f"{q['demotes']} demotion(s), {q['rearms']} re-arm(s)")

        # (c) and the floor is a floor: lose the last one and it IS a fault.
        # Without this, "never faults" would pass for a version that simply
        # deleted the interlock.
        z = run("every channel blind from t=30s", 60, ship_en, shipped,
                ki=info["ki"], kd=info["kd"],
                occlusions=[(i, 30.0, None) for i in range(n_ch)])
        check("losing the LAST axis does fault the rig",
              z["faults"] > 0 and z["state"] == "FAULT" and z["min_actuating"] == 0,
              f"{z['faults']} fault(s), state={z['state']}, "
              f"fell to {z['min_actuating']}/{n_live}")

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

    # ---- 7b. the OTHER sensor failure: one that never rails ------------------
    # `auto-disable` above catches a BLIND sensor, which pins at a rail. This is
    # the failure it cannot see: an OSEM that is powered, reading, not railed and
    # not measuring the optic, because its flag sits outside the partial shadow
    # where a shadow sensor is linear. It reports a healthy DC level and a
    # healthy noise floor with the optic simply absent from them, so it
    # calibrates a baseline near zero -- and EVERY test downstream of calibration
    # is a ratio against that baseline.
    #
    # Measured on the bench 2026-08-04, all eight OSEMs streaming
    # (bench/20260804/v55_all8.log): a4/a6/a7 calibrated 0.0014/0.0050/0.0020 V
    # against ch0's 0.3109 V, and the rig faulted TEN times in 145 s -- nine of
    # them naming a4, a6 or a7 -- while ch0-ch3 were damping at ratio 0.31-0.39.
    # a4 read ratio 2.73 on a bandpassed signal of +0.000 V. Four working loops,
    # frozen repeatedly by three channels that were measuring nothing.
    #
    # WHAT THE SIMULATOR REPRODUCES, AND WHAT IT DOES NOT. It reproduces the
    # cause exactly: with sens_gain = 0 the channel calibrates 0.0031 V against a
    # 0.613 V median, a factor of 200, matching the bench's 0.0014 vs 0.3109. It
    # does NOT reproduce the whole-rig FAULT, and the reason is worth knowing
    # before reading anything into that. Here the residual is stationary white
    # sensor noise, so env/baseline is the ratio of two estimates of the same
    # stationary quantity: it sits at ~1.0 and peaks at 1.28-1.58 against the
    # RUNAWAY_MULTIPLE = 1.8 line without crossing it. The bench's dead channels
    # are dominated by drift and by a 6.19 Hz interference line (versions.md:
    # 91-99% of their power at 3-20 Hz), which is NOT stationary in band, and
    # they reached 2.73. So the simulator understates this defect. What it does
    # show deterministically is the second consequence, which needs no luck at
    # all: a channel whose ratio sits at ~1.0 can never satisfy
    # `env < LOCK_RMS_FACTOR x baseline`, so the rig NEVER REPORTS LOCKED -- the
    # run's actual deliverable -- and no version before v10 recovers from that.
    note("\n  baseline-floor: an OSEM that reads but does not sense")
    if dead_on or (len(sensing) < n_ch):
        # THE EIGHT-OSEM BODY DOES NOT NEED THE INJECTION. a4/a6/a7 are like
        # this as measured -- powered, reading a 6.19 Hz interference line,
        # 0.1-9% of their power in the loop band, bandpassed RMS flat through a
        # ringdown that took a factor of 3-12 out of every real channel. So this
        # is the v5.5 failure run verbatim, and `baseline-floor` is the thing
        # that has to survive it. Bench 2026-08-06, v10, 8 channels: a4/a6/a7
        # calibrated 3% / 5% / 1% of the median and were demoted; the rig then
        # ran 40 s of unbroken DAMPING where v5.5 faulted ten times in 145 s.
        nosig = run("as shipped, eight OSEMs, three of them measuring nothing",
                    90, ship_en, shipped, ki=info["ki"], kd=info["kd"])
        blind_ch = [i for i in range(n_ch) if i not in sensing]
    else:
        nosig = run("ch3 sensing nothing at all (sens_gain 0)", 90, ship_en, shipped,
                    ki=info["ki"], kd=info["kd"],
                    sensor_gain=[1.0] * (n_ch - 1) + [0.0])
        blind_ch = [n_ch - 1]
    _b = sorted(nosig["baseline0"][i] for i in on) or sorted(nosig["baseline0"])
    med = (_b[len(_b) // 2] if len(_b) % 2
           else 0.5 * (_b[len(_b) // 2 - 1] + _b[len(_b) // 2]))
    note(f"  baselines {' '.join(f'ch{i}={b:.4f}V' for i, b in enumerate(nosig['baseline0']))}"
         f"  (median over enabled {med:.4f}V)")
    survivors = [i for i in on if i not in blind_ch]
    if "baseline-floor" in fixed and blind_ch != [n_ch - 1]:
        # The eight-OSEM form. There is no injected fault and no counterfactual
        # run to build: the plant IS the bench's channel set, so the two halves
        # -- throw away what measures nothing, keep what measures a little --
        # are both read off one run of the shipped configuration.
        w0 = weak[0] if weak else None
        check("FIXED baseline-floor: every OSEM that measures nothing is demoted "
              "and the rig keeps damping instead of faulting",
              all(nosig["floor"][i] for i in blind_ch) and nosig["faults"] == 0
              and nosig["state"] == "DAMPING"
              and all(nosig["ratio"][i] < 0.40 for i in solid),
              " ".join(f"ch{i}={nosig['baseline0'][i]:.4f}V="
                       f"{100.0 * nosig['baseline0'][i] / med:.1f}%" for i in blind_ch)
              + f" of the median, under the {S.osem.BASELINE_FLOOR_FRAC:.0%} line; "
              + f"{nosig['faults']} fault(s), survivors "
              + " ".join(f"ch{i}={nosig['ratio'][i]:.3f}" for i in solid))
        check("FIXED baseline-floor: it is a floor, not a cull -- the WEAK but "
              "real OSEM is kept",
              w0 is not None and not nosig["floor"][w0]
              and nosig["baseline0"][w0] > S.osem.BASELINE_FLOOR_FRAC * med,
              f"ch{w0} calibrated {nosig['baseline0'][w0]:.4f}V = "
              f"{100.0 * nosig['baseline0'][w0] / med:.0f}% of the median at "
              f"~1/{round(typical / strength[w0])} of the others' sensor gain -- "
              f"kept, {100.0 * S.osem.BASELINE_FLOOR_FRAC:.0f}% line"
              if w0 is not None else "this build has no low-gain channel")
        # The consequence, and the one that killed v5.5: a channel calibrated at
        # ~1% of the median has a ratio that sits at 1.0 whatever the optic does,
        # so it can never satisfy the lock test and, in every version before the
        # guard, drags the runaway breaker with it. Demotion has to take it out
        # of BOTH -- the coil and the interlocks -- not just quieten it.
        check("FIXED baseline-floor: a demoted channel is out of the loop AND "
              "out of the interlocks",
              all(nosig["out_dev"][i] < 1e-9 for i in blind_ch if i in on)
              and all(nosig["ratio"][i] > 0.5 for i in blind_ch)
              and nosig["faults"] == 0,
              "ratios "
              + " ".join(f"ch{i}={nosig['ratio'][i]:.2f}" for i in blind_ch)
              + f" (peak {max(nosig['peak'][i] for i in blind_ch):.2f} against the "
              f"{S.osem.RUNAWAY_MULTIPLE} runaway line) and {nosig['faults']} "
              "fault(s) -- v5.5 faulted ten times in 145 s on exactly this")
        # AND THE GUARD IS AMPLITUDE-DEPENDENT, which is not obvious and is worth
        # a bench session. BASELINE_FLOOR_FRAC compares a bandpassed RMS against
        # the median of one, and what a dead channel contributes to that RMS is
        # its interference line leaking through a SINGLE-POLE 3.0 Hz lowpass --
        # 6.19 Hz survives that at 0.44, attenuated but not gone. The line was
        # measured 4-20x louder on 2026-08-04 than on 2026-08-06, so this reruns
        # calibration at the louder session's amplitude. It stops at the end of
        # the first calibration, so it costs one window rather than a whole run.
        loud = run("same rig, the 2026-08-04 interference amplitude", 40,
                   ship_en, shipped, ki=info["ki"], kd=info["kd"],
                   plant={"line_scale": 10.0}, until_baseline=True)
        lb = sorted(loud["baseline0"][i] for i in on)
        lmed = (lb[len(lb) // 2] if len(lb) % 2
                else 0.5 * (lb[len(lb) // 2 - 1] + lb[len(lb) // 2]))
        kept_loud = [i for i in blind_ch
                     if loud["baseline0"][i] >= S.osem.BASELINE_FLOOR_FRAC * lmed]
        check("KNOWN LIMIT baseline-floor: the demotion depends on how loud the "
              "out-of-band interference is, not only on whether the OSEM senses",
              bool(kept_loud),
              "at 10x the 2026-08-06 line "
              + " ".join(f"ch{i}={100.0 * loud['baseline0'][i] / lmed:.0f}%"
                         for i in blind_ch)
              + f" of the median against a {S.osem.BASELINE_FLOOR_FRAC:.0%} line -- "
              + f"ch{','.join(map(str, kept_loud))} would be KEPT. A brick-wall "
              "band-limited floor estimate would not have this failure mode")
    elif "baseline-floor" in fixed:
        # The second run is the half without which this whole section could pass
        # by demoting everything. A quarter of nominal sensor gain is the a5 case
        # as it was actually measured on 2026-08-06: a5's bandpassed RMS is 0.26
        # of a0's (0.0801 V against 0.3109 V), which is what a real OSEM near the
        # edge of its linear range looks like from here. It must survive, damp
        # and lock -- v10.5 ships a5 enabled and the guard is what stands between
        # it and being thrown away.
        #
        # Note the sim is HARSHER than the bench on this point and deliberately
        # not tuned around: its low-gain channel is nearly pure scaled motion, so
        # 1/12 of nominal gain lands at 0.087 of the median here where the
        # bench's a5, with its own electronics floor underneath it, measured
        # 0.579. Set sensor_gain to 1/12 and this check fails -- correctly, for
        # a plant that has no per-channel electronics.
        bc = blind_ch[0]
        real = run(f"ch{bc} at a QUARTER of nominal sensor gain -- weak but real", 90,
                   ship_en, shipped, ki=info["ki"], kd=info["kd"],
                   sensor_gain=[1.0] * (n_ch - 1) + [0.25])
        check("FIXED baseline-floor: a channel that senses nothing is demoted and "
              "the rig keeps damping instead of faulting",
              nosig["floor"][bc] and nosig["faults"] == 0
              and nosig["state"] == "DAMPING"
              and all(nosig["ratio"][i] < 0.40 for i in survivors),
              f"ch{bc} calibrated {nosig['baseline0'][bc]:.4f}V = "
              f"{100.0 * nosig['baseline0'][bc] / med:.1f}% of the median, under the "
              f"{S.osem.BASELINE_FLOOR_FRAC:.0%} line; {nosig['faults']} fault(s), "
              "survivors "
              + " ".join(f"ch{i}={nosig['ratio'][i]:.3f}" for i in survivors))
        check("FIXED baseline-floor: with it demoted the rig can claim LOCKED again",
              nosig["lock"] is not None,
              f"{nosig['lock']:.1f}s, degraded to {len(survivors)}/{len(on)} channels"
              if nosig["lock"]
              else "never -- the demotion did not restore the lock claim")
        check("FIXED baseline-floor: a quiet but REAL channel is not demoted",
              not real["floor"][bc] and real["faults"] == 0
              and real["ratio"][bc] < 0.40 and real["lock"] is not None,
              f"ch{bc} at 1/4 gain calibrated {real['baseline0'][bc]:.4f}V, "
              f"{100.0 * real['baseline0'][bc] / med:.0f}% of a healthy median -- kept, "
              f"damps to {real['ratio'][bc]:.3f}, locks in "
              + (f"{real['lock']:.1f}s" if real["lock"] else "never"))
    else:
        bc = blind_ch[0]
        check("KNOWN GAP baseline-floor: nothing demotes a signal-less channel, and "
              "it holds the whole rig out of LOCKED for the entire run",
              not nosig["floor"][bc] and nosig["lock"] is None
              and nosig["ratio"][bc] > 0.5,
              f"ch{bc} baseline {nosig['baseline0'][bc]:.4f}V vs a {med:.4f}V median, "
              f"ratio stuck at {nosig['ratio'][bc]:.3f} (peak {nosig['peak'][bc]:.2f} "
              f"against the {S.osem.RUNAWAY_MULTIPLE} runaway line), "
              f"lock {'never' if nosig['lock'] is None else '%.1fs' % nosig['lock']}")

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
    hard = run(f"sustained disturbance, drive_amp={S.DRIVE_HARD}", 200, ship_en,
               shipped, ki=info["ki"], kd=info["kd"],
               plant={"drive_amp": S.DRIVE_HARD})
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

    # ---- 9. what coming back from a fault costs ------------------------------
    # Section 8 is the fault that keeps happening. This is the other one: a large
    # transient after the loop has been damping happily for a long time. Every
    # version faults on it; what differs is the price of coming back.
    #
    #   v0..v5        FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S = 25 s of open loop,
    #                 and the window it then measures is full of the ringdown the
    #                 kick left behind (tau = Q/(pi*f0) ~ 16 s, comparable to the
    #                 window), so the new floor comes out INFLATED. That is not
    #                 only slow: an inflated floor de-sensitises the runaway
    #                 breaker and makes LOCKED -- the run's deliverable -- easier
    #                 to declare.
    #   warm-restart  re-engage on the baseline in hand, which the loop had just
    #                 spent the whole engagement demonstrating was right.
    #
    # TWO kicks, not one, because "it re-engaged" and "the interlocks still work
    # against the baseline it re-engaged on" are separate claims and the second
    # one needs a disturbance after the recovery. t = 45 s and t = 90 s leave
    # every version on the ladder an engagement of at least FAST_REFAULT_S before
    # each fault, so this is the healthy-engagement case for all of them and not
    # v5's `fast-refault` under another name.
    w = run("two 15 V/s kicks, at t=45s and t=90s", 115, ship_en, shipped,
            ki=info["ki"], kd=info["kd"], shock_at=[(45.0, 15.0), (90.0, 15.0)])
    if "warm-restart" in fixed:
        check("FIXED warm-restart: a fault after a healthy engagement re-engages "
              "instead of re-measuring", w["reengaged"] >= 2,
              f"{w['reengaged']} straight-to-DAMPING of {w['recovered']} "
              f"recovery(ies), {w['faults']} fault(s)")
        # LOCKED is only available if every live channel CAN reach the lock line.
        # A low-gain OSEM in the loop cannot -- see the KNOWN LIMIT in section 1 --
        # so on that build the surviving claim is the breaker half: the carried
        # baseline still had to be tight enough for the second kick to trip it.
        if weak and set(weak) & set(on):
            check("FIXED warm-restart: the carried baseline still arms the breaker",
                  w["faults"] >= 2 and w["reengaged"] >= 2,
                  f"the second kick still tripped it ({w['faults']} faults, "
                  f"{w['reengaged']} reuses); LOCKED is out of reach on this build "
                  f"for the reason in section 1, not because of the reuse")
        else:
            check("FIXED warm-restart: the carried baseline still arms the breaker "
                  "and still gates LOCKED",
                  w["faults"] >= 2 and w["lock"] is not None,
                  f"the second kick still tripped it ({w['faults']} faults) and it "
                  f"re-locked in {w['lock']:.1f}s on the carried baseline"
                  if w["lock"] else f"{w['faults']} fault(s), never re-locked")
    else:
        check("every fault pays a full re-calibration, transient or not",
              w["reengaged"] == 0 and w["recovered"] >= 2,
              f"{w['recovered']} recovery(ies), {w['reengaged']} of them straight "
              f"to DAMPING, lock {'%.1fs' % w['lock'] if w['lock'] else 'never'}")

    # A RAIL is the exception, and it has to be: the interlock fires because the
    # SENSOR stopped reporting, and a floor measured through a sensor that has
    # since failed is suspect with it -- the OSEM's DC operating point may simply
    # have moved, which changes the floor without changing anything about the
    # lab. All four blind and then back is the one scenario that faults the rig
    # on a rail and still lets it recover, so the path is reachable at all.
    rr = run("every channel blind t=30..45s, then back", 75, ship_en, shipped,
             ki=info["ki"], kd=info["kd"],
             occlusions=[(i, 30.0, 45.0) for i in range(n_ch)])
    if "warm-restart" in fixed:
        check("FIXED warm-restart: a RAIL-caused fault still re-calibrates from "
              "scratch", rr["reengaged"] == 0 and rr["recovered"] > 0,
              f"{rr['recovered']} recovery(ies), {rr['reengaged']} of them "
              f"straight to DAMPING")
    elif "fast-refault" in fixed:
        # Not hypothetical. v5's fast-refault keys only on how long the last
        # engagement lasted, so a rail 10 s after engaging reuses the floor
        # measured through the sensor that has just failed. Asserted from the
        # broken side, like every other defect here, so it is visible rather than
        # argued -- it is what `warm-restart` had to carve out.
        check("KNOWN GAP fast-refault: it reuses the baseline even after a RAIL",
              rr["reengaged"] > 0,
              f"{rr['reengaged']} straight-to-DAMPING of {rr['recovered']} "
              f"recovery(ies) -- the failed sensor's floor is kept")
    else:
        check("without fast-refault a rail fault re-calibrates like any other",
              rr["reengaged"] == 0 and rr["recovered"] > 0,
              f"{rr['recovered']} recovery(ies)")

    # And the reuse budget is a real bound. drive_amp 3.5 clips over half the
    # samples -- a sensor problem no supervisor policy rescues -- so the loop
    # faults on contact, runs MAX_BASELINE_REUSE out and pays for a window
    # anyway. Without this, "it re-engages" would also pass for a version that
    # could never be made to re-measure at all.
    if "fast-refault" in fixed:
        bud = run(f"drive_amp={S.DRIVE_UNDAMPABLE}, a disturbance it cannot damp",
                  120, ship_en, shipped, ki=info["ki"], kd=info["kd"],
                  plant={"drive_amp": S.DRIVE_UNDAMPABLE})
        check("the baseline-reuse budget is a bound -- it does re-calibrate in "
              "the end",
              bud["reengaged"] >= 1 and bud["recovered"] > bud["reengaged"],
              f"{bud['reengaged']} reuse(s) of {bud['recovered']} recovery(ies), "
              f"MAX_BASELINE_REUSE={getattr(S.osem, 'MAX_BASELINE_REUSE', '-')}")

    # ---- 12. the loop's own clock (v12 `decimate`) ---------------------------
    # The rate the control law runs at has never been a control decision in this
    # repo: it is the ARRIVAL rate, and that has been 12.5, 18, 23.5, 73 and
    # ~1111 Hz depending on how many coils are driven and which transport is
    # compiled in. Every gain, every filter and every interlock is exercised at
    # whatever that happens to be.
    #
    # SECTIONS 12-14 RUN ONLY FOR VERSIONS THAT DECLARE THE FIX, which is a
    # departure from the rest of this file and is deliberate. Everything else
    # here asserts the un-fixed side too, so a defect is visible rather than
    # argued; these three cannot, for two different reasons. `decimate` has no
    # observable on a version with no control clock other than "the loop rate is
    # the wire rate", which is a tautology in a simulator that has exactly one
    # wire rate. `persist-baseline` needs a process restart, which this suite
    # cannot do. Asserting the gap would also have changed the CHECK COUNT of
    # every version below v12 -- v9 is scored 37/37 across the repo -- for
    # statements no simulator run actually tests.
    #
    # What IS testable in here: that the control rate is CONTROL_HZ while the
    # wire runs at SAMPLE_HZ, i.e. the two are different numbers, and that each
    # step averages the samples that arrived rather than sub-sampling them. The
    # cross-wire-rate claim -- that the control rate holds when the BAUD changes
    # -- is measured offline on the bench logs and argued in the file's
    # docstring; the simulator has one wire rate and cannot show it.
    if "decimate" in fixed:
        phase("\n  the control rate against the wire rate")
        fast = run("wire at SAMPLE_HZ", 25, ship_en, shipped,
                   ki=info["ki"], kd=info["kd"])
        hz = getattr(S.osem, "CONTROL_HZ", 0.0)
        steps = getattr(fast["sim"].ctl, "ctl_steps", None)
        cf = None if steps is None else steps / max(fast["sim"].t, 1e-9)
        check("FIXED decimate: the control step runs at CONTROL_HZ, not at "
              "whatever the wire delivers",
              cf is not None and hz > 0 and abs(cf - hz) / hz < 0.05,
              f"CONTROL_HZ={hz:.0f}, measured {cf:.1f} Hz on a "
              f"{S.SAMPLE_HZ:.0f} Hz wire" if cf else "no ctl_steps counter")
        # ...and it AVERAGES, rather than sub-sampling. n_avg is how many raw
        # samples the last step consumed, and SAMPLE_HZ / CONTROL_HZ is how many
        # there should be. > 1 is the sqrt(N) the whole change is for.
        na = int(getattr(fast["sim"].ctl, "n_avg", 0))
        want = S.SAMPLE_HZ / max(hz, 1e-9)
        check("FIXED decimate: each step consumes the samples that arrived since "
              "the last one, and averages them",
              na > 1 and abs(na - want) <= 1.0,
              f"{na} samples/step against {want:.1f} expected at "
              f"{S.SAMPLE_HZ:.0f} Hz wire / {hz:.0f} Hz control")

    # ---- 13. torn rows off the wire (v12 `sample-guard`) ---------------------
    # Driven directly rather than through the plant, because the plant cannot
    # produce this: `sim/server.py` builds counts by construction and clips them
    # to 0..1023, so a framing error has to be injected by hand. It is not
    # hypothetical -- v11's own 240 s bench log has ten rows carrying counts
    # outside the converter's range (5659, 65690, 522676, ...), against zero in
    # each of the four slower logs from the same session, because
    # `fast-transport` stopped draining the board's `OK` replies and they now
    # tear the data rows they interleave with.
    #
    # One of them is worth ~1500 V/s of velocity into a loop whose whole scale is
    # under 1 V/s.
    #
    # Fixed-side only, for the reason given at section 12 -- and note this one
    # COULD be asserted from the un-fixed side (it is a direct-drive test and
    # v3/v7/v9 do spike on it), but doing so would move their check counts, and
    # the defect it names was created by `fast-transport`, which none of them
    # carry. On a DACController build the ack is drained and the rows do not tear.
    nch_ = len(S.osem.ENABLE_CHANNEL)
    tctl = S.osem.Controller(S.FakeDAC())
    dtw, rest = 1.0 / S.SAMPLE_HZ, 615.0

    def _feed(ctl, t0, secs, torn=None):
        pk, t = 0.0, t0
        for k in range(int(secs * S.SAMPLE_HZ)):
            t = t0 + k * dtw
            c = [float(round(rest + 40.0 * math.sin(2 * math.pi * 1.046 * t + 0.3 * i)))
                 for i in range(nch_)]
            if torn is not None and k == 0:
                c[0] = float(torn)
            ctl.step(c, [x * (S.osem.A_VCC / S.osem.ADC_MAX_COUNTS) for x in c],
                     t, dtw)
            pk = max(pk, max(abs(float(ch.vel)) for ch in ctl.channels))
        return pk, t + dtw

    if "sample-guard" in fixed:
        phase("\n  one torn row off the wire")
        quiet, t_end = _feed(tctl, 0.0, 4.0)
        quiet, t_end = _feed(tctl, t_end, 1.0)      # settled: the reference
        spike, _ = _feed(tctl, t_end, 1.0, torn=522676)
        bad = int(getattr(tctl, "bad_samples", 0))
        check("FIXED sample-guard: a count outside 0..1023 is dropped, not "
              "filtered", bad >= 1 and spike < 3.0 * max(quiet, 1e-9),
              f"{bad} row(s) rejected; peak |vel| {quiet:.3f} -> {spike:.3f} V/s, "
              f"i.e. the loop never saw it")

    # ---- 14. the floor across restarts (v12 `persist-baseline`) --------------
    # The lab's noise floor is a property of the room, and this suite cannot
    # restart a process -- so what is tested is the two halves that decide
    # whether a stored floor may be trusted: the fingerprint (refuse anything
    # measured under a different setup) and the warm-up check (refuse anything
    # measured in a different room). Fixed-side only; see section 12.
    if "persist-baseline" in fixed:
        phase("\n  the stored floor")
        import json
        import tempfile
        bpath = os.path.join(tempfile.mkdtemp(prefix="osem-baseline-"),
                             "baseline.json")
        # A real calibration first, so the thing being stored is a floor this
        # controller actually measured.
        w = S.osem.Controller(S.FakeDAC())
        _feed(w, 0.0, float(getattr(S.osem, "CALIBRATION_S", 20.0)) + 1.0)
        S.osem.save_baseline(w, bpath)
        got, _rep = S.osem.load_baseline(bpath)
        check("FIXED persist-baseline: a floor round-trips through the file",
              got is not None and w.state == "DAMPING",
              f"stored {len(getattr(w, 'baseline', []))} channels, "
              f"calibration took {w.calib_took:.1f}s")
        # Adopted: DAMPING inside the warm-up window instead of paying the
        # calibration again.
        warm = S.osem.Controller(S.FakeDAC(), baseline_file=got)
        _feed(warm, 0.0, float(getattr(S.osem, "BASELINE_WARMUP_S", 2.0)) + 0.5)
        check("FIXED persist-baseline: a valid stored floor engages the loop in "
              "the warm-up window, not in CALIBRATION_S",
              warm.state == "DAMPING" and getattr(warm, "warm_used", False),
              f"DAMPING after {warm.calib_took:.1f}s against a "
              f"{getattr(S.osem, 'CALIBRATION_S', 0):.0f}s calibration")
        # ...and refused when the room is not the one it was measured in. 10x
        # the amplitude is well outside BASELINE_SANITY_RATIO and well inside
        # what a moved OSEM flag actually does (a5 reads 1/12 of a0).
        loud = S.osem.Controller(S.FakeDAC(), baseline_file=got)
        for k in range(int(3.0 * S.SAMPLE_HZ)):
            t = k * dtw
            c = [float(min(1023, max(0, round(rest + 400.0 * math.sin(
                2 * math.pi * 1.046 * t + 0.3 * i))))) for i in range(nch_)]
            loud.step(c, [x * (S.osem.A_VCC / S.osem.ADC_MAX_COUNTS) for x in c],
                      t, dtw)
        check("FIXED persist-baseline: a stored floor that disagrees with the "
              "live signal is refused, and the run calibrates instead",
              not getattr(loud, "warm_used", False) and loud.state == "CALIBRATING",
              f"warm-up saw ~10x the stored floor, outside "
              f"{getattr(S.osem, 'BASELINE_SANITY_RATIO', 0):.1f}x; state "
              f"{loud.state}")
        # The fingerprint, one field at a time. Each of these changes what a
        # baseline MEANS, so each must refuse rather than adapt.
        base = json.load(open(bpath))
        rejects = []
        for label, mutate in (
                ("age", lambda d: d.__setitem__(
                    "measured_unix",
                    d["measured_unix"] - S.osem.BASELINE_FILE_MAX_AGE_S - 10)),
                ("schema", lambda d: d.__setitem__("schema", 999)),
                ("bias", lambda d: d["config"]["bias"].__setitem__(0, 0.75)),
                ("gains", lambda d: d["config"]["steady"].__setitem__(0, -0.02)),
                ("enable", lambda d: d["config"]["enable"].__setitem__(0, False)),
                ("control_hz", lambda d: d["config"].__setitem__("control_hz", 37.0)),
                ("filter band", lambda d: d["config"].__setitem__("bp_low_hz", 0.7)),
                ("channel count", lambda d: d.__setitem__(
                    "baseline_v", d["baseline_v"][:-1]))):
            d = json.loads(json.dumps(base))
            mutate(d)
            json.dump(d, open(bpath, "w"))
            if S.osem.load_baseline(bpath)[0] is None:
                rejects.append(label)
        check("FIXED persist-baseline: every fingerprint field refuses a floor "
              "measured under a different configuration",
              len(rejects) == 8, f"refused on: {', '.join(rejects)}")
        json.dump(base, open(bpath, "w"))
        check("FIXED persist-baseline: and it can always be overridden by hand",
              S.osem.load_baseline(bpath)[0] is not None
              and S.osem.load_baseline(bpath, fresh=True)[0] is None,
              "--fresh / OSEM_FRESH_CALIB forces a full calibration on a file "
              "that would otherwise be accepted")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Run a controller against the simulated suspension.",
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
                what = {"sysid": "actuation-matrix measurement, closes no loop",
                        "skeleton": "not a controller yet -- main() raises"}
                print(f"{d['name']:<10} {d['bench']:<11} {'-':<9} {'-':<8} "
                      f"{d['kind']}: {what.get(d['kind'], 'not a controller')}")
                continue
            print(f"{d['name']:<10} {d['bench']:<11} {d['n_enabled']}/{d['n_channels']}       "
                  f"{'live' if any(d['ki']) or any(d['kd']) else 'zeroed':<8} "
                  f"{', '.join(d['fixes']) or '-'}")
        print("\n  validated = confirmed on hardware    untested = never on hardware")
        print("  Ladder order is zero -> alpha -> beta -> delta, then epsilon when it")
        print("  exists (ladder.py). Old numbered names still resolve, with a note.")
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
    if info["n_channels"] not in SIM_CHANNELS:
        sys.exit(f"  {info['name']} has {info['n_channels']} channels and sim/server.py "
                 f"has a measured body for {' or '.join(map(str, SIM_CHANNELS))}.\n"
                 f"  Serving it crashes on the first request rather than telling you "
                 f"anything.\n  Run it on hardware: make run V={info['tag']}")
    if info["bench"] != "validated":
        print(f"  NOTE: {info['name']} is '{info['bench']}' on hardware -- fine in "
              f"here, see README.md before flashing it.", flush=True)
    S.load(path)
    return S.serve(args.port)


if __name__ == "__main__":
    sys.exit(main())
