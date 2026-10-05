#!/usr/bin/env python3
"""Behavioural suite for the controllers, against the simulated plant in sim/.

It reimplements no part of the control law: it imports the version named and steps
that module's own `Controller`. Which names exist is `ladder.py`'s business.
Expectations are derived from the loaded module -- channels enabled, whether I/D
are live, which defects it declares fixed -- so one set of checks covers the ladder
and a version claiming a fix has to demonstrate it.

    python3 harness.py --test all          # the whole ladder
    python3 harness.py --test eta
    python3 harness.py --list

No interactive front-end here: `scope.py` plots the real board or a live bench
run.

THE SIMULATOR IS NOT THE RIG, AND MOST OF `make check`'s FAILURES ARE THAT.
`sim/server.py`'s eight-OSEM body was built from the 2026-08-04/06 sessions: TWO
modes at 1.046 and 1.657 Hz. The rig has THREE, measured 0.71519 / 0.99231 /
1.65307 Hz (`status.py` MODES), so eta's and theta's modal path is not reachable
from here at all and nothing below should be read as evidence about the plant.
What this suite does test is BEHAVIOUR -- interlocks, demotion, recovery,
schedule -- against a body that is stable, seeded and reproducible. Refitting the
body is a bench job, not a harness one.

Version contract. To be runnable here a controller must expose:
    ENABLE_CHANNEL, STEADY_GAIN, CAPTURE_GAIN, KI_GAIN, KD_GAIN,
    BIAS, DAC_CHANNELS, ADC_MAX_COUNTS, A_VCC, ENVELOPE_WINDOW_S,
    SlidingRMS, Controller(dac, enable=, steady=, capture=, ki=, kd=)
and `Controller` must offer .step(counts, volts, t, dt) -> state,
.drain_events(), .channels, .state, .lock_time, .fault_count.
Optionally VERSION_TAG and FIXES; absent means "v0-era, nothing fixed".
"""

import argparse
import contextlib
import io
import math
import os
import re
import sys
import time

# Before the imports below: sim/server.py loads controllers by PATH, which puts
# nothing on sys.path, and they import `stdlib`/`pyDAC2` by bare name.
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ladder                         # noqa: E402
from sim import server as S           # noqa: E402


# --------------------------------------------------------------------------
# version discovery
# --------------------------------------------------------------------------
def versions():
    """Every controller next to this file, in ladder order, tools last.

    `ladder.discover` is the only copy of this scan -- see ladder.py."""
    return ladder.discover(HERE)


def resolve(name):
    """'eta', 'osem.eta', 'osem.eta.py' or a path -> an absolute path."""
    all_versions = versions()
    if not all_versions:
        sys.exit("No controllers found next to harness.py.")
    if name is None:
        # The sysid tool sorts after every rung, so the newest file is not always
        # drivable.
        drivable = simulatable()
        return drivable[-1] if drivable else all_versions[-1]
    if os.path.isfile(name):
        return os.path.abspath(name)
    stem = ladder.resolve_name(name)
    if stem:
        cand = os.path.join(HERE, ladder.filename(stem))
        if os.path.isfile(cand):
            return cand
    # Numbered names are redirected, not refused: every log predating the rename
    # says "v12".
    new, was_numbered = ladder.redirect(name)
    if new:
        cand = os.path.join(HERE, ladder.filename(new))
        if os.path.isfile(cand):
            print(f"note: {name} is now `{new}` -- running that.")
            return cand
    if was_numbered:
        sys.exit(f"{name} is not on the ladder any more. Git history still has "
                 f"the file; versions.md says what it was. Have: "
                 + ", ".join(ladder.stem(os.path.basename(p)) for p in all_versions))
    sys.exit(f"No such version: {name}. Have: "
             + ", ".join(ladder.stem(os.path.basename(p)) for p in all_versions))


_DECL_RE = r'^%s\s*=\s*["\']([^"\']*)["\']'


def declared(path, name, default=""):
    """Read a string constant WITHOUT executing the module.

    Necessary: sim/server.py's load() derives constants while importing and raises
    on anything that is not a controller, so a bench tool cannot be loaded."""
    with open(path) as fh:
        src = fh.read()
    m = re.search(_DECL_RE % name, src, re.M)
    if m:
        return m.group(1)
    # Some versions declare two constants on one line, which the pattern misses.
    m = re.search(r'^([A-Z_]+)\s*,\s*([A-Z_]+)\s*=\s*'
                  r'["\']([^"\']*)["\']\s*,\s*["\']([^"\']*)["\']', src, re.M)
    if m and name in (m.group(1), m.group(2)):
        return m.group(3) if name == m.group(1) else m.group(4)
    return default


# Channel counts sim/server.py has a measured body for: four is the 2026-08-03
# rig, eight the 2026-08-04/06 one.
SIM_CHANNELS = (4, 8)


def simulatable():
    """Versions the simulator can drive: controllers whose channel count matches a
    measured body. `versions()` still returns everything, for --list."""
    return [p for p in versions()
            if declared(p, "KIND", "controller") == "controller"
            and describe(p)["n_channels"] in SIM_CHANNELS]


def describe(path):
    """Facts about one version. Non-controllers (KIND != "controller", e.g.
    osem.sysid.py) are read from source text and never handed to the simulator."""
    name = os.path.basename(path)[:-3]
    kind = declared(path, "KIND", "controller")
    if kind != "controller":
        return dict(path=path, name=name, kind=kind,
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

# Left None, everything prints and runs flat out.
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


def median(xs):
    """Plain median of a sequence. One copy: the baseline-floor section computed
    it three different ways inline, which is three places for one off-by-one."""
    v = sorted(xs)
    if not v:
        return 0.0
    k = len(v) // 2
    return v[k] if len(v) % 2 else 0.5 * (v[k - 1] + v[k])


def note(text):
    if _emit("note", text=text):
        print(text)


def phase(label):
    if _emit("phase", label=label):
        print(label)


# ack_drain = 0 ON PURPOSE, against sim/server.py's 3.5 (the bench's ~24 Hz while
# driving coils): at 24 Hz the sample-counted MAX_CONSECUTIVE_SATURATED = 30 means
# 1.2 s rather than 86 ms and twenty checks below invert. `line_scale` pinned --
# a4/a6/a7's out-of-band interference measured 4-20x louder in one session than
# the other. `f_drive` comes from S.DRIVE_HZ per body rather than a literal, so
# the tone always sits on whichever body's first resonance is loaded.
BASE_PLANT = dict(ack_drain=0.0,
                  drive_amp=0.8, seismic=0.8, Q=50.0, k_act=20.0,
                  meas_noise=20.0, hum_hz=60.0, hum_mv=0.0, hvac_hz=2.5,
                  hvac_amp=0.0, shock_rate=0.0, shock_amp=3.0, line_scale=1.0)


def run(label, seconds, enable, steady, ki=None, kd=None, occlude_at=None,
        auto_kick=False, plant=None, quiet=True, shock_at=None,
        stop_on_recover=False, until_baseline=False, occlusions=None,
        sensor_gain=None):
    """Step the loaded controller through the simulated plant, returning the
    observations the checks read.

    `seconds` is an upper bound: `stop_on_recover` cuts at the first
    FAULT -> CALIBRATING and `until_baseline` at the end of the first calibration,
    so a check does not pay for v3's 20 s CALIBRATION_S.

    `shock_at` is `(t, dv)` or a time-ordered LIST. It draws two random numbers
    for the pitch/yaw split, so a shocked run and an unshocked one diverge in the
    noise stream after it -- a reference, not a counterfactual.

    `occlude_at` is `(idx, t)`, blind to the end of the run; `occlusions` is the
    general `[(idx, t_on, t_off_or_None), ...]`, where the un-blind is what makes
    "it puts the axis back" testable and overlaps exercise the quorum.

    `sensor_gain` is a counts-per-metre multiplier per channel -- the OTHER sensor
    failure, mis-alignment, which keeps reporting a healthy DC level and noise
    floor with the optic absent. 0.0 is a4/a6/a7 (bench 2026-08-06), 1/12 is a5.
    Mostly a four-channel tool: eight-OSEM already has those as measured."""
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
    # Set explicitly rather than left over from the last run. See BASE_PLANT.
    sim.plant["f_drive"] = S.DRIVE_HZ
    if plant:
        sim.plant.update(plant)
    sim.reset()
    # Anything queued between scenarios was aimed at the previous one.
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
    # Baselines at the END OF THE FIRST CALIBRATION. Reading `baseline_rms`
    # afterwards reports the LAST one, which once made a version read 58.6% on the
    # calibration-skew check while running v3's code byte for byte.
    baseline0 = None
    # When that calibration ENDED: fixed on v4/v5, a ceiling under `fast-calib`.
    t_calib = None
    # saturated_flag anywhere: it latches FAULT in v0/v1/v2, so it decides whether
    # a scenario tests the runaway path or the saturation one.
    sat_ever = [False] * nch
    # Peak |out - bias| against the +-0.25 V clip, per sample against the CURRENT
    # bias: `bias-trim` moves it 0.25 V at a time and can land mid-run, so the
    # module constant would report a trim step as actuation.
    bias = [float(b) for b in S.osem.BIAS]
    out_dev = [0.0] * nch
    # PID terms captured DURING the run: reset() replaces sim.ctl, so reading them
    # afterwards reports the previous run, and a run that ended mid-FAULT has a
    # correctly-zeroed integrator that would fail a healthy version.
    max_p = max_i = max_d = 0.0
    # `healthy` is runtime health, distinct from static `enabled`. Pre-v4 lacks it,
    # so getattr defaults True and these fields go constant instead of needing a
    # second suite.
    occlusions = list(occlusions or [])
    prev_healthy = [True] * nch
    t_engaged = None         # when the loop first reached DAMPING
    rearms = 0
    t_demote = t_rearm = None
    min_actuating = nch
    # Peak |out - bias| WHILE some channel is demoted. The 0.5 s offset skips the
    # slew: at MAX_SLEW_PER_S = 2.0 V/s, 0.25 V takes 0.125 s to reach bias.
    dev_blind = [0.0] * nch
    # Ever demoted for an implausible baseline: decided once at the end of
    # calibration, so invisible in `demotes`, which counts rails.
    floor_ever = [False] * nch
    # v0's check_rail prints unlatched on every railed sample, flooding the console
    # through the occlusion sweep. The assertions read rail_fault, not this.
    sink = io.StringIO()
    for n in range(int(seconds * S.SAMPLE_HZ)):
        if occlude_at is not None and sim.t >= occlude_at[1]:
            sim.occlude[occlude_at[0]] = True
        for oi, t_on, t_off in occlusions:
            sim.occlude[oi] = sim.t >= t_on and (t_off is None or sim.t < t_off)
        while sched and sim.t >= sched[0][0]:
            sim.kick(sched.pop(0)[1])
        # Applied here, never concurrently with step().
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
            # Recovery has TWO exits: v0..v5-pre went FAULT -> CALIBRATING, a
            # version reusing its baseline goes FAULT -> DAMPING.
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
        # The clock starts from the set the loop ENGAGES with: `baseline-floor`
        # demotes at the end of calibration, which would start t_demote at t ~ 20 s
        # and make every channel look like it drove blind.
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
        # Counted in EVERY state: the last demotion and the quorum fault happen
        # inside one step(), so gating on DAMPING misses the zero.
        min_actuating = min(min_actuating,
                            sum(1 for i, c in enumerate(chans)
                                if c.enabled and healthy[i]))
        # Only WHILE A SENSOR IS BLINDED: "any channel unhealthy" is the whole run
        # on the eight-OSEM body, where a4/a6/a7 never come back.
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
                # end-of-run value only if calibration never completed at all
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
    # Two measured geometries only: four-OSEM 2026-08-03, eight-OSEM 2026-08-04/06.
    # Any other count would need invented physics, so skip loudly.
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

    # What the PLANT says, as distinct from the config. `sensing` = coupled to the
    # optic; a4/a6/a7 are not (0.1-9% of their power in the 0.4-3 Hz loop band,
    # lock-in SNR 1.1-1.5). `weak` = sensing at ~1/12 of a0 (a5, measured): damps
    # fine, still cannot pass a ratio test against its own floor -- v10 saw this.
    strength = [math.sqrt(sum(v * v for v in S.SHAPE[i])) for i in range(n_ch)]
    sensing = [i for i in range(n_ch) if strength[i] > 0]
    typical = median(strength[i] for i in sensing)
    weak = [i for i in sensing if strength[i] < 0.5 * typical]
    live = [i for i in on if i in sensing]
    solid = [i for i in live if i not in weak]      # enabled, sensing, full gain
    dead_on = [i for i in on if i not in sensing]

    # Scenario gain literals (-0.600, half magnitude) were chosen four-OSEM. The
    # eight-OSEM ch0 authority is 0.30 of its DC total -- only the part in the two
    # visible modes -- so the same Kp is 3.3x weaker (at Kp = -0.030 ch0 decays
    # 0.157 /s against 0.191 on the bench, 0.363 four-OSEM). Rescaled per channel,
    # since ch2's authority is 4.15x ch0's four-OSEM (provenance.md Table 1) and
    # 2.58x here. gscale is exactly 1.0 on the four-OSEM body.
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
        # v10's docstring: "a5 reads ratio 0.45 and so the rig does not announce
        # LOCKED". Its ABSOLUTE envelope is fine; ratio is against its own
        # baseline, a twelfth of the others'.
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
        # Such a channel cannot damp; what it must NOT do is fault the rig. That
        # is the v5.5 failure, checked properly in section 7b.
        check("an enabled channel that senses nothing does not fault the rig",
              a["faults"] == 0 and a["state"] == "DAMPING",
              f"ch{','.join(map(str, dead_on))} enabled with no motion coupling, "
              f"{a['faults']} fault(s), state={a['state']}")
    if off:
        # NOT "disabled channels stay at their baseline": one rigid body, so
        # damping one corner drains the mass. provenance.md section 3: "although
        # only one channel is being driven, all four visibly decay". Measured on
        # v0: driven ch0 = 0.197, undriven 0.265/0.323/0.355; all four enabled
        # moves the other three 0.047-0.099 V off bias.
        check("disabled channels never actuate -- output pinned to bias",
              all(a["out_dev"][i] < 1e-9 for i in off),
              " ".join(f"ch{i}={a['out_dev'][i]:.1e}V" for i in off))
        # Only the undriven channels that WATCH the optic: eight-OSEM `off` is
        # a4/a6/a7, flat through a ringdown taking a factor of 3-12 out of the rest.
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
    # "Three good loops against one wrong-signed one" on either body.
    all_sensing = [i in sensing for i in range(n_ch)]
    b = run("ch2 gain sign inverted", 120, all_sensing, bad,
            ki=info["ki"], kd=info["kd"])
    if n_ch == 4:
        check("a wrong-signed channel is pumped past the runaway line",
              b["peak"][2] > 1.8, f"ch2 peak={b['peak'][2]:.2f}")
    else:
        # Measured rather than borrowed: a version faulting on SATURATION first
        # re-calibrates before the ratio can climb, so with weaker coils the trip
        # comes while the envelope is only a few times its floor.
        check("a wrong-signed channel turns a fault-free run into a repeatedly "
              "faulting one",
              b["faults"] >= 3 and a["faults"] == 0,
              f"{b['faults']} fault(s) inverted against {a['faults']} as shipped; "
              f"ch2 peak {b['peak'][2]:.2f} -- the envelope RATIO stays low here "
              f"because the trip comes before it can climb, not because the "
              f"channel is behaving")
    check("the global interlock trips on it", b["faults"] > 0, f"{b['faults']} fault(s)")
    # The safety complement to soft-saturation: clipped-and-winning keeps damping,
    # clipped-and-not-winning must still fault.
    if "soft-saturation" in fixed:
        check("soft-saturation did NOT disable the interlock: clipped and NOT "
              "winning still faults",
              b["faults"] > 0 and any(b["sat_ever"]),
              f"{b['faults']} fault(s) with the actuator pinned, ch2 peak="
              f"{b['peak'][2]:.2f}")
    # Trips on a PINNED ACTUATOR, not a runaway: inverting ch2 leaves a third of
    # the damping (sum of gain_i * COIL_GAIN_i, -0.1415 -> -0.0419), so the optic
    # sits near full amplitude and demands more than the +-0.25 V clip. v0/v1 trip
    # at t=10.74 s on ch3 with sat_streak=31, v2 at t=17.23 s: defect #1.
    stuck_b = b["state"] == "FAULT" and not b["recovered"]
    if "saturation-latch" in fixed:
        check("FIXED saturation-latch: the sign-error trip clears and the loop "
              "recovers", b["recovered"] > 0, f"{b['recovered']} recovery(ies)")
    else:
        check("KNOWN BUG saturation-latch: the sign-error trip pins an actuator "
              "and latches", stuck_b and any(b["sat_ever"]),
              f"state={b['state']}, {b['recovered']} recovery(ies), "
              f"saturated={[i for i, s in enumerate(b['sat_ever']) if s]}")

    # A runaway with NO saturation, so recovery is tested alone. Half magnitude
    # halves the growth rate (RUNAWAY_SUSTAIN_S = 2 s elapses before the amplitude
    # passes the 1.8x line) and doubles the amplitude needed to clip: peak
    # |out - bias| is v0/v1 0.112 V, v2 0.057 V, v3 0.055 V of a 0.25 V clip.
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
    elif any(info["kd"]):
        # Ki zeroed, Kd live -- epsilon, whose Kalman filter carries drift as a
        # state. Without this branch it falls into the pure-P `else` and fails
        # max_d < 1e-12, which was a gap in the check, not a controller defect.
        check("with KI_GAIN zeroed the I term is identically zero",
              a["max_i"] < 1e-12, f"max |I| = {a['max_i']:.2e} V")
        check("P still dominates -- D does not take over the loop",
              a["max_p"] > a["max_d"],
              f"P={a['max_p']:.4f} D={a['max_d']:.4f} V")
    else:
        check("with KI_GAIN and KD_GAIN zeroed the law is pure P",
              a["max_i"] < 1e-12 and a["max_d"] < 1e-12,
              f"I={a['max_i']:.2e} D={a['max_d']:.2e}")

    # ---- 4. saturation trip: recovers only if the version fixed it ----------
    over = list(shipped)
    over[0] = -0.600 * gscale[0]
    only0 = [i == 0 for i in range(n_ch)]
    # ch0 ALONE only works four-OSEM: eight-OSEM it leaves a5 real, weak and
    # undamped inside the runaway breaker, so the rig trips on a5 instead of on
    # ch0's clipping. There it runs as SHIPPED with ch0 over-gained on top.
    over_en = only0 if n_ch == 4 else ship_en
    d = run("over-gain ch0 + a kick", 90, over_en, over,
            ki=info["ki"], kd=info["kd"], auto_kick=True)
    # A CLIPPED loop, not a diverging one: the plant is linear apart from the ADC,
    # so -0.600 damps harder than -0.030 (versions.md: swept to -0.600, zero
    # faults) and up to v9 it faulted on the 30-consecutive-pinned streak alone.
    # Here v9 faults once, peaks 1.527, 87.8% DAMPING; v10 never faults, peaks
    # 0.853, 93.3% -- freezing a winning actuator let it ring 1.8x higher.
    if "soft-saturation" in fixed and n_ch == 4:
        check("FIXED soft-saturation: a clipped actuator that is still WINNING "
              "keeps damping instead of faulting",
              d["faults"] == 0 and any(d["sat_ever"]) and d["ratio"][0] < 0.35,
              f"{d['faults']} fault(s), saturated={any(d['sat_ever'])}, "
              f"ch0 ratio {d['ratio'][0]:.3f} vs the 0.35 lock line, "
              f"peak {d['peak'][0]:.3f}, {d['pct_damping']:.1f}% damping")
    elif "soft-saturation" in fixed:
        # "Zero faults" is a four-OSEM claim: here a5 sits in the interlocks on a
        # floor that is mostly its own noise and can trip for unrelated reasons.
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
    # DETECTION and RESPONSE are two claims -- v4 keeps the first and inverts the
    # second -- so not one `rail and frozen`. `frozen` ("all four outputs within
    # 1 uV of bias") is a coincidence detector one locked quiet channel satisfies
    # by accident, so v4's response is read off `faults`.
    needs_freeze = "auto-disable" not in fixed
    check("the interlock arms on a blind sensor when the signal is quiet",
          all(r["rail"] for r in quiet_rows),
          " ".join("armed" if r["rail"] else "MISSED" for r in quiet_rows))
    if needs_freeze:
        check("blinding one OSEM freezes all four channels",
              all(r["frozen"] for r in quiet_rows))
    else:
        # ch1 is not driving (enable is [T,F,F,F]), so v4 demotes an already-parked
        # channel and carries on where v3 faulted the whole optic.
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
    # ONE impulse SWEPT across v0's 8 s window (inside every longer one on the
    # ladder), because where it lands dominates the answer -- v0 can look BETTER
    # than v3 at some shock times. The WORST case is asserted. Measured over
    # t = 2/4/6 s: v0/v1/v2 (one 8 s window, RMS) 39.3%; v3/v4/v5 (20 s, median of
    # 5) 19.7%; 35% separates them at ~1.1x / ~1.8x. Re-measured 2026-08-03 after
    # the simulator took the bench sample rate and scripted shocks got their own
    # RNG -- the previous 45.3% / 25.0% depended on the rate. Simulator, never
    # hardware.
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
        # Only channels whose baseline is the OPTIC: one made of its own noise
        # floor (a5 measures ~70% own content) re-rolls with the window, so
        # including it would report window placement rather than the shock.
        s = max(abs(base[i] / base0[i] - 1.0) for i in solid if base0[i] > 0)
        if s > skew:
            skew, worst_t = s, t_shock
    where = f"worst {skew * 100:.1f}% (shock at t={worst_t:.0f}s)"
    if n_ch != 4:
        # The 35% line is FOUR-OSEM (v0's 39.3% against v3+'s 19.7%) and no v0
        # exists for this body, so applying it would borrow a threshold across a
        # plant change. Printed anyway, for a future v0-equivalent to compare to.
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
    # v4/v5 pay a fixed CALIBRATION_S = 20 s at zero gain: ~65% of the wall clock,
    # bench 2026-08-03/04 (lock at t ~ 31 s on four channels). `fast-calib` makes
    # it a CEILING; neither check below is sufficient alone.
    calib_s = float(S.osem.CALIBRATION_S)
    if "fast-calib" in fixed and len(sensing) == n_ch and not weak:
        check("FIXED fast-calib: a quiet lab does not pay the whole calibration "
              "window", clean["t_calib"] is not None and clean["t_calib"] <= calib_s * 0.5,
              f"{clean['t_calib']:.2f}s of a {calib_s:.0f}s ceiling"
              if clean["t_calib"] else "never calibrated")
    elif "fast-calib" in fixed:
        # THE BENCH ALREADY FOUND THIS. `fast-calib` exits early only when
        # trailing sub-window floors agree within CALIB_AGREE_TOL on EVERY channel,
        # which one whose floor is its own narrowband noise never does. Hardware
        # 2026-08-04, v8: "ran the full 20 s ceiling, the floor never settled";
        # versions.md has the 20 s -> 6 s saving as simulator-only.
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
    # The safety half, for EVERY version: t = 2 s and 4 s land before the earliest
    # exit any version can take, and a ringdown stored as the floor both
    # de-sensitises the breaker and makes LOCKED easier to declare.
    early_on_shock = [t for t in (2.0, 4.0)
                      if took[t] is None or took[t] < calib_s - 0.2]
    check("a transient inside the window is never read as a settled floor",
          not early_on_shock,
          " ".join(f"shock@{t:.0f}s -> {took[t]:.2f}s" for t in (2.0, 4.0)
                   if took[t] is not None)
          + f" of a {calib_s:.0f}s window")

    # ---- 7. graceful degradation: a blind axis drops out, the rest carry on --
    # Gated on the fix, since only v4 has the `healthy` attribute; v3's `any_rail`
    # freezes all four. Occlusions land at t=30, past the longest calibration
    # (v3/v4 are 20 s) and past lock, so these are claims about a RUNNING loop.
    if "auto-disable" in fixed:
        note("\n  auto-disable: losing axes from a running loop")
        # Loops left standing after calibration throws out what measures nothing:
        # the enabled set, minus a4/a6/a7 on the eight-OSEM body.
        n_live = len(live)
        others = [i for i in live if i != 1]

        # (a) one driving channel blind t=30..45. The shock at t=31 makes "the
        # others kept damping" a measurement: a locked still optic parks every
        # output at bias whether or not the loop is live.
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
        # returns: the window draining below RAIL_FRACTION plus REARM_SUSTAIN_S.
        # Bounded both sides -- it waited, and it came back at all.
        lo = S.osem.REARM_SUSTAIN_S
        hi = S.osem.REARM_SUSTAIN_S + S.osem.RAIL_SUSTAIN_S + 1.5
        back = None if g["t_rearm"] is None else g["t_rearm"] - 45.0
        check("it re-arms once the sensor returns, after the hold and not before",
              back is not None and lo <= back <= hi,
              f"{'never' if back is None else f'{back:.2f}s'} after un-blinding "
              f"(hold {lo:.1f}s, window allows {lo:.1f}-{hi:.1f}s)")

        # (b) the quorum. One healthy channel is the documented floor --
        # provenance.md section 3 measured a single OSEM/coil pair damping the whole
        # body -- and restoring all three at once proves re-arm is per-channel.
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

        # (c) without this, "never faults" passes for a deleted interlock.
        z = run("every channel blind from t=30s", 60, ship_en, shipped,
                ki=info["ki"], kd=info["kd"],
                occlusions=[(i, 30.0, None) for i in range(n_ch)])
        check("losing the LAST axis does fault the rig",
              z["faults"] > 0 and z["state"] == "FAULT" and z["min_actuating"] == 0,
              f"{z['faults']} fault(s), state={z['state']}, "
              f"fell to {z['min_actuating']}/{n_live}")

        # (d) calibration is deliberately NOT degraded: a channel demoted
        # mid-calibration would re-arm with no baseline, so v4 keeps v3's fault.
        c0 = run("ch1 blind from t=0, during calibration", 35, ship_en, shipped,
                 ki=info["ki"], kd=info["kd"], occlusions=[(1, 0.0, None)])
        check("a rail during CALIBRATION is still a whole-rig fault",
              c0["faults"] > 0 and c0["demotes"] == 0,
              f"{c0['faults']} fault(s), {c0['demotes']} demotion(s), "
              f"state={c0['state']}")

    # ---- 7b. the OTHER sensor failure: one that never rails ------------------
    # The failure `auto-disable` cannot see: an OSEM powered, reading, not railed,
    # not measuring the optic, so it calibrates a baseline near zero -- and every
    # test downstream of calibration is a ratio against that baseline.
    #
    # Bench 2026-08-04, all eight streaming (bench/20260804/v55_all8.log):
    # a4/a6/a7 calibrated 0.0014/0.0050/0.0020 V against ch0's 0.3109 V, the rig
    # faulted TEN times in 145 s (nine naming a4/a6/a7) while ch0-ch3 damped at
    # 0.31-0.39, and a4 read ratio 2.73 on a bandpassed +0.000 V.
    #
    # The simulator reproduces the CAUSE -- sens_gain = 0 calibrates 0.0031 V
    # against a 0.613 V median, a factor of 200 against the bench's 0.0014 vs
    # 0.3109 -- but UNDERSTATES it: stationary white residual puts env/baseline at
    # ~1.0, peaking 1.28-1.58 against RUNAWAY_MULTIPLE = 1.8 without crossing,
    # where the bench's drift and 6.19 Hz line (versions.md: 91-99% of their power
    # at 3-20 Hz) reached 2.73. What it does show deterministically: a ratio stuck
    # at ~1.0 never satisfies `env < LOCK_RMS_FACTOR x baseline`, so the rig NEVER
    # REPORTS LOCKED.
    note("\n  baseline-floor: an OSEM that reads but does not sense")
    if dead_on or (len(sensing) < n_ch):
        # NO INJECTION NEEDED -- a4/a6/a7 are like this as measured, so this is
        # the v5.5 failure run verbatim. Bench 2026-08-06, v10, 8 channels:
        # a4/a6/a7 calibrated 3% / 5% / 1% of the median and were demoted, then
        # 40 s of unbroken DAMPING where v5.5 faulted ten times in 145 s.
        nosig = run("as shipped, eight OSEMs, three of them measuring nothing",
                    90, ship_en, shipped, ki=info["ki"], kd=info["kd"])
        blind_ch = [i for i in range(n_ch) if i not in sensing]
    else:
        nosig = run("ch3 sensing nothing at all (sens_gain 0)", 90, ship_en, shipped,
                    ki=info["ki"], kd=info["kd"],
                    sensor_gain=[1.0] * (n_ch - 1) + [0.0])
        blind_ch = [n_ch - 1]
    med = median([nosig["baseline0"][i] for i in on] or nosig["baseline0"])
    note(f"  baselines {' '.join(f'ch{i}={b:.4f}V' for i, b in enumerate(nosig['baseline0']))}"
         f"  (median over enabled {med:.4f}V)")
    survivors = [i for i in on if i not in blind_ch]
    if "baseline-floor" in fixed and blind_ch != [n_ch - 1]:
        # The eight-OSEM form: the plant IS the bench's channel set, so both
        # halves come off one run with no injection and no counterfactual.
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
        # What killed v5.5: a channel calibrated at ~1% of the median has a ratio
        # stuck at 1.0 whatever the optic does, so demotion must remove it from
        # BOTH the coil and the interlocks.
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
        # AND THE GUARD IS AMPLITUDE-DEPENDENT: a dead channel contributes its
        # interference line through a SINGLE-POLE 3.0 Hz lowpass, and 6.19 Hz
        # survives that at 0.44. The line was 4-20x louder on 2026-08-04 than
        # 2026-08-06, so this reruns calibration at the louder amplitude.
        loud = run("same rig, the 2026-08-04 interference amplitude", 40,
                   ship_en, shipped, ki=info["ki"], kd=info["kd"],
                   plant={"line_scale": 10.0}, until_baseline=True)
        lmed = median(loud["baseline0"][i] for i in on)
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
        # Without this half the section could pass by demoting everything. A
        # quarter of nominal gain is a5 measured 2026-08-06: bandpassed RMS 0.26 of
        # a0's, 0.0801 V against 0.3109 V, and v10.5 ships a5 enabled. The sim is
        # HARSHER here and deliberately not tuned around -- its low-gain channel is
        # nearly pure scaled motion, so 1/12 of nominal lands at 0.087 of the
        # median where the bench's a5, with its own electronics floor, gave 0.579.
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
    # Everything above faults on a TRANSIENT. drive_amp 2.5 keeps clipping, so the
    # fault recurs the moment the loop re-engages and each re-calibration costs
    # CALIBRATION_S at ZERO gain. Measured over 200 s, % of samples DAMPING:
    # v0/v1/v2 0.6% (they latch FAULT outright, defect #1); v3/v4 4.1%, 7 faults,
    # never locks; v5 87.5%, 1 fault, 1 straight-to-DAMPING recovery, locks. The
    # margins either side of the lines below are ~2x and ~7x.
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
    # A large transient after a long healthy engagement: every version faults, the
    # price differs. v0..v5 pay FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S = 25 s of
    # open loop, then measure a window full of ringdown (tau = Q/(pi*f0) ~ 16 s,
    # comparable to the window), so the new floor comes out INFLATED -- which
    # de-sensitises the breaker and makes LOCKED easier to declare. TWO kicks,
    # because "it re-engaged" and "the interlocks still work against that
    # baseline" are separate claims; t = 45 s and 90 s leave every version at
    # least FAST_REFAULT_S of engagement first.
    w = run("two 15 V/s kicks, at t=45s and t=90s", 115, ship_en, shipped,
            ki=info["ki"], kd=info["kd"], shock_at=[(45.0, 15.0), (90.0, 15.0)])
    if "warm-restart" in fixed:
        check("FIXED warm-restart: a fault after a healthy engagement re-engages "
              "instead of re-measuring", w["reengaged"] >= 2,
              f"{w['reengaged']} straight-to-DAMPING of {w['recovered']} "
              f"recovery(ies), {w['faults']} fault(s)")
        # LOCKED needs every live channel to reach the lock line and a low-gain
        # OSEM cannot (section 1), so there only the breaker half survives.
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

    # A RAIL is the exception: the interlock fires because the SENSOR stopped
    # reporting, so a floor measured through it is suspect -- its DC operating
    # point may simply have moved.
    rr = run("every channel blind t=30..45s, then back", 75, ship_en, shipped,
             ki=info["ki"], kd=info["kd"],
             occlusions=[(i, 30.0, 45.0) for i in range(n_ch)])
    if "warm-restart" in fixed:
        check("FIXED warm-restart: a RAIL-caused fault still re-calibrates from "
              "scratch", rr["reengaged"] == 0 and rr["recovered"] > 0,
              f"{rr['recovered']} recovery(ies), {rr['reengaged']} of them "
              f"straight to DAMPING")
    elif "fast-refault" in fixed:
        # Not hypothetical: v5's fast-refault keys only on engagement length, so a
        # rail 10 s after engaging reuses the failed sensor's floor. This is what
        # `warm-restart` had to carve out.
        check("KNOWN GAP fast-refault: it reuses the baseline even after a RAIL",
              rr["reengaged"] > 0,
              f"{rr['reengaged']} straight-to-DAMPING of {rr['recovered']} "
              f"recovery(ies) -- the failed sensor's floor is kept")
    else:
        check("without fast-refault a rail fault re-calibrates like any other",
              rr["reengaged"] == 0 and rr["recovered"] > 0,
              f"{rr['recovered']} recovery(ies)")

    # The reuse budget is a real bound: drive_amp 3.5 clips over half the samples,
    # so the loop runs MAX_BASELINE_REUSE out and pays for a window anyway.
    if "fast-refault" in fixed:
        bud = run(f"drive_amp={S.DRIVE_UNDAMPABLE}, a disturbance it cannot damp",
                  120, ship_en, shipped, ki=info["ki"], kd=info["kd"],
                  plant={"drive_amp": S.DRIVE_UNDAMPABLE})
        check("the baseline-reuse budget is a bound -- it does re-calibrate in "
              "the end",
              bud["reengaged"] >= 1 and bud["recovered"] > bud["reengaged"],
              f"{bud['reengaged']} reuse(s) of {bud['recovered']} recovery(ies), "
              f"MAX_BASELINE_REUSE={getattr(S.osem, 'MAX_BASELINE_REUSE', '-')}")

    # ---- 12. the loop's own clock (`decimate`) -------------------------------
    # The control rate was never a control decision here: it is the ARRIVAL rate,
    # measured at 12.5, 18, 23.5, 73 and ~1111 Hz depending on coils driven and
    # transport compiled in.
    #
    # SECTIONS 12-14 ASSERT THE FIXED SIDE ONLY, unlike the rest of this file.
    # `decimate` has no observable on a version whose only clock is the wire rate;
    # `persist-baseline` needs a process restart this suite cannot do; and
    # asserting the gap would move the CHECK COUNT of every earlier version (v9 is
    # scored 37/37 across the repo) for statements no run tests. Testable here:
    # control rate == CONTROL_HZ against a SAMPLE_HZ wire, and that each step
    # averages. The cross-baud claim is measured offline on the bench logs.
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
        # ...and AVERAGES rather than sub-samples. n_avg > 1 is the sqrt(N).
        na = int(getattr(fast["sim"].ctl, "n_avg", 0))
        want = S.SAMPLE_HZ / max(hz, 1e-9)
        check("FIXED decimate: each step consumes the samples that arrived since "
              "the last one, and averages them",
              na > 1 and abs(na - want) <= 1.0,
              f"{na} samples/step against {want:.1f} expected at "
              f"{S.SAMPLE_HZ:.0f} Hz wire / {hz:.0f} Hz control")

    # ---- 13. torn rows off the wire (`sample-guard`) -------------------------
    # Driven directly, since sim/server.py clips counts to 0..1023 by construction.
    # Not hypothetical: v11's own 240 s bench log has ten rows outside the
    # converter's range (5659, 65690, 522676, ...) against zero in each of the four
    # slower logs from the same session -- `fast-transport` stopped draining the
    # board's `OK` replies and they now tear the rows they interleave with. One is
    # worth ~1500 V/s into a loop whose whole scale is under 1 V/s.
    nch_ = len(S.osem.ENABLE_CHANNEL)
    tctl = S.osem.Controller(S.FakeDAC())
    dtw, rest = 1.0 / S.SAMPLE_HZ, 615.0
    tone = S.MODE_F0[0]          # the loaded body's own first mode, never a copy

    def _feed(ctl, t0, secs, torn=None):
        pk, t = 0.0, t0
        for k in range(int(secs * S.SAMPLE_HZ)):
            t = t0 + k * dtw
            c = [float(round(rest + 40.0 * math.sin(2 * math.pi * tone * t + 0.3 * i)))
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

    # ---- 14. the floor across restarts (`persist-baseline`) ------------------
    # No process restart is possible here, so what is tested is the two halves that
    # decide whether a stored floor may be trusted: the fingerprint (a different
    # setup) and the warm-up check (a different room).
    if "persist-baseline" in fixed:
        phase("\n  the stored floor")
        import json
        import tempfile
        bpath = os.path.join(tempfile.mkdtemp(prefix="osem-baseline-"),
                             "baseline.json")
        # A real calibration first, so the stored floor is a measured one.
        w = S.osem.Controller(S.FakeDAC())
        _feed(w, 0.0, float(getattr(S.osem, "CALIBRATION_S", 20.0)) + 1.0)
        S.osem.save_baseline(w, bpath)
        got, _rep = S.osem.load_baseline(bpath)
        check("FIXED persist-baseline: a floor round-trips through the file",
              got is not None and w.state == "DAMPING",
              f"stored {len(getattr(w, 'baseline', []))} channels, "
              f"calibration took {w.calib_took:.1f}s")
        warm = S.osem.Controller(S.FakeDAC(), baseline_file=got)
        _feed(warm, 0.0, float(getattr(S.osem, "BASELINE_WARMUP_S", 2.0)) + 0.5)
        check("FIXED persist-baseline: a valid stored floor engages the loop in "
              "the warm-up window, not in CALIBRATION_S",
              warm.state == "DAMPING" and getattr(warm, "warm_used", False),
              f"DAMPING after {warm.calib_took:.1f}s against a "
              f"{getattr(S.osem, 'CALIBRATION_S', 0):.0f}s calibration")
        # ...and refused when the room is not the one it was measured in. 10x is
        # outside BASELINE_SANITY_RATIO and inside what a moved flag does (a5
        # reads 1/12 of a0).
        loud = S.osem.Controller(S.FakeDAC(), baseline_file=got)
        for k in range(int(3.0 * S.SAMPLE_HZ)):
            t = k * dtw
            c = [float(min(1023, max(0, round(rest + 400.0 * math.sin(
                2 * math.pi * tone * t + 0.3 * i))))) for i in range(nch_)]
            loud.step(c, [x * (S.osem.A_VCC / S.osem.ADC_MAX_COUNTS) for x in c],
                      t, dtw)
        check("FIXED persist-baseline: a stored floor that disagrees with the "
              "live signal is refused, and the run calibrates instead",
              not getattr(loud, "warm_used", False) and loud.state == "CALIBRATING",
              f"warm-up saw ~10x the stored floor, outside "
              f"{getattr(S.osem, 'BASELINE_SANITY_RATIO', 0):.1f}x; state "
              f"{loud.state}")
        # The fingerprint, one field at a time: each must refuse, not adapt.
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
        description="Run the behavioural suite for a controller against the "
                    "simulated suspension.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("version", nargs="?", default=None,
                    help="epsilon, zeta, eta, ... (default: the newest)")
    ap.add_argument("--test", nargs="?", const="__self__", default=None,
                    metavar="VERSION", help="which version to test; 'all' walks "
                                            "the whole ladder (default: the "
                                            "newest, or the positional version)")
    ap.add_argument("--list", action="store_true", help="list versions and exit")
    args = ap.parse_args()

    if args.list:
        print(f"{'version':<13} {'channels':<9} {'I/D':<8} fixes")
        for path in versions():
            d = describe(path)
            if d["kind"] != "controller":
                what = {"sysid": "actuation-matrix measurement, closes no loop",
                        "skeleton": "not a controller yet -- main() raises"}
                print(f"{d['name']:<13} {'-':<9} {'-':<8} "
                      f"{d['kind']}: {what.get(d['kind'], 'not a controller')}")
                continue
            print(f"{d['name']:<13} {d['n_enabled']}/{d['n_channels']}       "
                  f"{'live' if any(d['ki']) or any(d['kd']) else 'zeroed':<8} "
                  f"{', '.join(d['fixes']) or '-'}")
        print("\n  Ladder order: " + " -> ".join(ladder.LADDER) + " (ladder.py).")
        print("  %s are deleted. Those names and the old vN numbers still get an "
              "answer:\n  v6 -> sysid, the rest a pointer to git "
              "history." % ", ".join(ladder.RETIRED))
        return 0

    # Testing is all this file does, so a bare version name means "test that one".
    target = args.test if args.test not in (None, "__self__") else args.version
    paths = versions() if target == "all" else [resolve(target)]
    t0 = time.perf_counter()
    for path in paths:
        suite(path)
    print(f"\n{'-' * 20} {_pass} passed, {_fail} failed "
          f"({time.perf_counter() - t0:.0f}s) {'-' * 20}")
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
