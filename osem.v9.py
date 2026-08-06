"""
v8 -- stop paying 20 s of zero gain for a baseline.
===================================================
v5 with two changes to the SUPERVISOR and nothing else. Every gain, filter,
threshold and state transition v5 ships is here unchanged: the PID, the
bandpass, the gain schedule, the rail interlock, the runaway breaker and the
lock detector are untouched, and the four new constants below are the only
additions to SETTINGS. Both changes are about WHEN the loop is allowed to be
closed, not about what it does once it is -- which is why the damping numbers
are v5's to three decimals (ch0 = 0.029, lock 10.5 s).

WHY. Time-to-lock is `calibration + lock-after-gain`, and on the bench
(2026-08-03/04, four channels) that is 20 s + ~11 s: **65% of the wall clock is
spent with the gain forced to zero and nothing damping.** Worse on recovery --
FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S = 25 s of open loop before the loop
re-engages, so one kick costs ~40 s of not damping. v5's `fast-refault` already
elides the case where the fault repeats immediately; this is the rest of it.
research.md item 6 is the write-up of the problem and ranks these two first.

  1. `fast-calib`. CALIBRATION_S becomes a CEILING rather than a target.
     Sub-windows of CALIB_SUBWINDOW_S accumulate as they complete and
     calibration ends as soon as the trailing ones say the noise floor has
     stopped moving. Quiet lab: out in ~6 s. Anything still ringing down: runs
     to the ceiling and falls back to EXACTLY v5's estimator, median of
     CALIB_SUBWINDOWS sub-windows over the whole window. Measured, in the
     simulator: calibration 20.00 s -> 6.01 s and total time-to-lock 30.6 s ->
     16.5 s, while the swept-shock baseline skew that `runaway-baseline` is
     scored on goes 19.7% -> 17.6% against a 35% line -- it does not degrade,
     because a shock inside the window is exactly the case that refuses to
     converge early and so is still measured v5's way.
     Swept over 16 lab conditions (sensor noise 0-40 mV, seismic 0.2-2.5, drive
     0-2.0 and 0.5-2.5 Hz, Q 10-200, 60 Hz hum, HVAC, shocks up to 60/min), 12
     exit at 6.01 s and land within 7.0% of the floor the full window would have
     measured -- worst case a lab so quiet the drive is off the resonance. The
     other 4 run the ceiling and return v5's number EXACTLY (0.0% difference,
     same code on the same samples): drive off with seismic only, the two
     off-resonance drives, and 60 shocks/min. Those are the cases where the
     floor genuinely is still moving, which is the answer you want.

  2. `warm-restart`. A fault that follows a healthy engagement re-engages on
     the baseline already in hand instead of measuring a new one. v5 does this
     only when the previous engagement lasted under FAST_REFAULT_S (the
     disturbance is evidently still there); v8 does it in the opposite case too
     (the baseline demonstrably supported a working loop, and the thing that
     faulted was a transient). Both are bounded -- see THE INVARIANT below.
     Measured on two 15 V/s kicks into a locked loop, at t = 45 s and t = 90 s:
     v4 and v5 pay FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S = 25 s of open loop for
     each and never get back to LOCKED inside 115 s; v8 re-engages 5 s after the
     all-clear both times and re-locks 10.7 s later.

Everything else is v5 and is documented there and in versions.md: the gain
signs, ch2's positive Kp, the quorum of one, `auto-disable`, `fast-refault`,
and the three v3 defect fixes. This file is the mechanism, not the argument.

  CALIBRATING  gain 0. Sub-windows of CALIB_SUBWINDOW_S; ends EARLY once the
               trailing CALIB_AGREE_N of them agree with each other AND with
               every sub-window measured so far, else at the CALIBRATION_S
               ceiling on the median of CALIB_SUBWINDOWS windows. Any rail
               faults the rig -- you cannot calibrate a blind sensor, which is
               what guarantees a demoted channel has a baseline to come back to.
  DAMPING      u = PID(-vel), gain scheduled by amplitude, slew-limited. A
               railed channel is DEMOTED (held at bias, the rest keep damping)
               and re-arms REARM_SUSTAIN_S after its rail clears. The rig faults
               only on a runaway, a pinned actuator, or losing the last channel.
  FAULT        all outputs at bias until clear for FAULT_CLEAR_SUSTAIN_S, then
               either straight back to DAMPING on the baseline already in hand,
               or a full recalibration (which also clears every demotion).

--------------------------------------------------------------------------
THE INVARIANT THIS BREAKS, AND WHERE THE LINE IS NOW
--------------------------------------------------------------------------
The standing rule is **"baseline RMS is re-measured, never borrowed"** (README,
versions.md). v4 broke it once, per channel, for the re-arm path; v5 broke it
once, for the whole rig, when a fault repeats inside FAST_REFAULT_S. v8 makes
borrowing the DEFAULT on the fault path. That is deliberate and it is the
riskiest thing in this file, so the reasoning is here rather than in a doc:

  * `baseline_rms` is the UNDAMPED noise floor of the lab. It is a property of
    the lab, not of the event that just faulted the loop.
  * Re-measuring after a transient measures the transient. At Q = 50 and
    f0 ~ 1 Hz the ringdown is tau = Q/(pi*f0) ~ 16 s, comparable to the whole
    window, so a post-kick recalibration returns an INFLATED floor. That is not
    merely wasteful, it is unsafe in both directions: the runaway breaker
    (> RUNAWAY_MULTIPLE x baseline) is desensitised, and the lock detector
    (< LOCK_RMS_FACTOR x baseline) becomes EASIER to satisfy -- a false LOCKED
    claim, which is the run's actual deliverable.
  * `fast-calib` narrows but does not close this: after a kick the sub-windows
    do not agree, so the recalibration correctly runs the full ceiling. Correct
    and still 20 s of not damping.

It must STILL re-measure, and does, in all four of these cases:

  1. **First entry at startup.** `self.baseline.all()` is False until a
     calibration has completed, so there is nothing to borrow. No exception.
  2. **After a RAIL-caused fault.** A rail means the SENSOR was suspect, and a
     floor measured through a suspect sensor is suspect with it -- including
     the possibility that the OSEM's DC operating point moved, which changes
     the floor without changing anything about the lab. `fault_railed` is set
     at the two places a rail can fault the rig (a rail during CALIBRATING, and
     losing the quorum in DAMPING) and forces a full recalibration.
  3. **After MAX_BASELINE_REUSE reuses** without an intervening engagement of
     at least FAST_REFAULT_S. This is v5's bound, unchanged, and it is what
     stops a stale baseline surviving a supervisor loop that is faulting on
     contact. A long engagement resets the budget because the baseline has just
     demonstrated that it supports a closed loop.
  4. **Once the baseline is older than BASELINE_MAX_AGE_S.** Rule 3 alone is
     not a bound in wall-clock time -- alternating long and short engagements
     could refresh the budget forever -- and an hour-long run must not close on
     a floor measured once at t = 0. A fault is the natural moment to re-measure.

Ran on the bench 2026-08-04 (`bench/20260804/v9_4ch_3kicks.log`): re-locked after
all three kicks, 4 faults against v8's 10, 26.6% of samples DAMPING. `runaway-trend`
is therefore confirmed on hardware. `fast-calib`, inherited from v8, is NOT --
`CALIB_AGREE_TOL` was tuned on simulator noise and the bench floor never satisfied
it, so it burns the full ceiling and degrades to v5's estimator. The numbers above
are still simulator numbers; what the simulator can and cannot tell you is in
versions.md.

Two conventions inherited from v5 and worth knowing before editing this:
`enabled` is static config, `healthy` is runtime, and `enabled & healthy` gates
the output. Timers that may be "not running" are np.inf, so `t - timer >= X` is
False without a branch; `baseline` is 0.0 rather than None while uncalibrated,
so it stays an array and still reads falsy where the old code tested it.
"""

import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from pyDAC import DACController

# ===================== SETTINGS =====================
PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 4

# Coil DAC channel per sensor 0..3. Rewired 2026-08-03; the full 8-pair map is
# [1,3,5,7,0,2,4,6] (provenance.md). The old [0,2,4,6] now drives the coils for
# sensors 4..7 -- neither the simulator nor the interlocks can catch a wrong map.
DAC_CHANNELS = [1, 3, 5, 7]

VERSION_TAG, BENCH_STATUS = "v9", "validated"   # ran 2026-08-04, see versions.md
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend")
ENABLE_CHANNEL = [True, True, True, True]

BIAS = np.full(N, 0.25)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 0.5, 2.0

# Kp. STEADY is the bench-validated -0.030 on ch0; CAPTURE is stronger, used only
# at large amplitude, and is NOT independently validated.
#   STEADY_GAIN[2] is POSITIVE -- ch2's coil or OSEM is mounted the other way
#   round (provenance.md Table 1). Not a typo, do not "correct" it.
#   Kp = -0.040 is the documented instability / rail onset. Do not exceed -0.035
#   unattended, and note the simulator provably cannot reproduce that limit.
STEADY_GAIN = np.array([-0.030, -0.030, +0.010, -0.030])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035])

# Ki/Kd on the velocity loop: the integral of velocity is DISPLACEMENT (an added
# spring, which moves the pole rather than damping it) and its derivative is
# ACCELERATION (added negative mass, and the noisiest term). Same sign per
# channel as Kp, because they share one loop.
KI_GAIN = np.array([-0.040, -0.040, +0.040, -0.040])
KD_GAIN = np.array([-0.0015, -0.0015, +0.0015, -0.0015])
D_SMOOTH_HZ, I_CLAMP_V, TRACK_TC_S = 2.0, 0.15, 0.5   # I_CLAMP is only a backstop

BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ = 0.4, 3.0, 5.0
CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
SCHEDULE_WINDOW_S, GAIN_SLEW_PER_S = 1.0, 0.02

# --- calibration: a ceiling and a convergence test, not a fixed duration -----
# CALIBRATION_S is now the LONGEST calibration may take, and CALIB_SUBWINDOWS is
# how the ceiling path divides it -- both unchanged from v5, so a calibration
# that runs to the ceiling produces exactly v5's number from exactly v5's code.
#
# CALIB_SUBWINDOW_S is the granularity of the convergence test. 2.0 s is two
# cycles of the ~1 Hz resonance the loop closes on, which is the shortest window
# whose RMS is not dominated by where in the cycle it started; it also divides
# CALIBRATION_S into 10, so the ceiling path regroups cleanly into the 5 windows
# v5 uses. CALIB_MIN_SUBWINDOWS = 3 keeps v5's "at least three, so one bad one
# can be outvoted" and sets the floor on calibration at 6 s.
#
# CALIB_AGREE_TOL is a RATIO (max/min) and 1.20 comes from a measured gap, not
# from taste: in a quiet lab the trailing three sub-windows agree to 1.09 at the
# earliest point the test can fire and to ~1.03 after that, while under a 6 V/s
# kick inside the window the same statistic runs 1.19-1.66 for the whole 20 s.
# 1.19 is the narrow side of that gap, which is exactly why _calib_stationary
# needs its SECOND test as well. Read that docstring before moving this number.
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # ceiling; subwindows >= 3
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20

LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0
ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE, RUNAWAY_SUSTAIN_S = 2.0, 1.8, 2.0
# v9: the breaker also requires GROWTH. Compare the envelope against itself
# RUNAWAY_TREND_LAG_S ago; a real runaway grows on that timescale, a ringdown
# at Q~50 falls. 1.02 gives 2% headroom so envelope noise alone cannot read as
# growth. Lag is one ENVELOPE_WINDOW_S so the two estimates barely overlap.
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 2.0, 1.02
MAX_CONSECUTIVE_SATURATED = 30                 # samples, not seconds

# Rail: raw counts, so there is no float-rounding ambiguity, and a FRACTION of a
# window rather than an unbroken run, so one noise sample dilutes the evidence
# instead of erasing it.
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80

FAULT_CLEAR_SUSTAIN_S = 5.0
REARM_SUSTAIN_S = 2.0        # 5 tau of the 0.4 Hz highpass; timed from rail-clear
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md §3)

# --- when the fault path may borrow the baseline it already has --------------
# The full argument, and the four cases where it must NOT, is in the docstring.
# FAST_REFAULT_S keeps v5's meaning and gains a second one: an engagement at
# least this long is what RESETS the reuse budget, because a baseline that
# carried a closed loop for 15 s has just been validated by the loop itself.
# MAX_BASELINE_REUSE is v5's bound, unchanged. BASELINE_MAX_AGE_S is the
# wall-clock backstop the budget alone does not give -- a policy choice, not a
# measurement: 5 minutes is long against the 16 s ringdown and short against a
# shift in the lab.
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0

STATUS_PERIOD_S, CSV_FLUSH_EVERY_N = 5.0, 200

_LOG = (("bp", ".5f"), ("vel", ".5f"), ("out", ".4f"), ("gain", ".5f"),
        ("ratio", ".4f"), ("p", ".5f"), ("i", ".5f"), ("d", ".5f"))
CSV_HEADER = "time_s,state," + ",".join(
    f"ch{i}_{c}" for i in range(N)
    for c in ("counts", "V") + tuple(k for k, _ in _LOG) + ("rail", "locked", "healthy"))
# ====================================================


class OnePole:
    """One-pole low/high pass over N independent lanes. `on` is per lane because
    the D-term filter is reset for whichever channels stopped actuating this
    sample, and a lane coming back must re-prime from its first sample rather
    than ramp up from zero."""

    def __init__(self, hz, kind="low", n=N):
        self.tau, self.low = 1.0 / (2.0 * np.pi * hz), kind == "low"
        self.y, self.xp, self.on = np.zeros(n), np.zeros(n), np.zeros(n, bool)

    def update(self, x, dt):
        new = ~self.on
        y0 = np.where(new, x if self.low else 0.0, self.y)
        a = dt / (self.tau + dt) if self.low else self.tau / (self.tau + dt)
        nxt = (y0 + a * (x - y0) if self.low
               else a * (y0 + x - np.where(new, x, self.xp)))
        self.y, self.xp = np.where(new, y0, nxt), x.copy()
        self.on[:] = True
        return self.y

    def reset(self, m=slice(None)):
        self.y[m], self.xp[m], self.on[m] = 0.0, 0.0, False


class SlidingRMS:
    """RMS over a time window. Scalar or array -- sim/server.py uses it scalar for
    its own diagnostic; the controller keeps one per lane so a single channel's
    window can be cleared without resizing everyone else's."""

    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = window_s, deque(), 0.0

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        # np.maximum is not defensive padding. The sum is incremental, so when a
        # large transient ages out of the window it leaves a rounding residue of
        # its own magnitude -- measured at -4.5e-14 after a 6 V/s kick in a quiet
        # lab, i.e. NEGATIVE. sqrt() of that is nan, and every comparison against
        # nan is False, so the runaway breaker would stop tripping and the lock
        # detector would stop firing, both silently and permanently. v0..v4 have
        # the same accumulator and the same latent bug.
        return (np.sqrt(np.maximum(self.sq, 0.0) / len(self.buf)) if self.buf
                else self.sq * 0.0)

    def reset(self):
        self.buf.clear()
        self.sq = 0.0


def _bank(window_s):
    return [SlidingRMS(window_s) for _ in range(N)]


def _rms(bank, t, x, m):
    """A lane outside `m` is not fed at all, so a blind channel's flatlined
    bandpass never enters its window."""
    return np.array([bank[i].update(t, x[i]) if m[i] else 0.0 for i in range(N)])


class Actuator:
    """DAC writes for all N coils: 10 ms throttle, 0.5 mV deadband. A missed ack
    is swallowed on purpose -- the next sample resends."""

    def __init__(self, dac, channels):
        self.dac, self.ch = dac, list(channels)
        self.t, self.v = np.zeros(len(self.ch)), np.full(len(self.ch), np.nan)

    def send(self, volts):
        now = time.time()
        due = (now - self.t >= 0.01) & (np.isnan(self.v) | (abs(volts - self.v) >= 0.0005))
        for i in np.nonzero(due)[0]:
            try:
                self.dac.set_voltage(channel=self.ch[i], voltage=float(volts[i]))
                self.t[i], self.v[i] = now, volts[i]
            except RuntimeError:
                pass


def read_sample(ser):
    """(counts, volts) as two length-N arrays, or None. arduino.ino streams eight
    columns but only A0..A3 carry OSEMs, so take the leading four; four-column
    firmware is unaffected."""
    if not ser.in_waiting:
        return None
    try:
        parts = ser.readline().decode("utf-8").strip().split(",")
        if len(parts) < N:
            return None
        counts = np.array([int(p) for p in parts[:N]], dtype=float)
        return counts, counts * (A_VCC / ADC_MAX_COUNTS)
    except (ValueError, IndexError):
        return None


# Channel attribute -> Controller array.
_VIEW = dict(zip("enabled healthy bias steady_gain capture_gain ki kd bp_out vel out "
                 "active_gain last_ratio p_term i_term d_term rail_fault locked "
                 "saturated_flag baseline_rms".split(),
                 "enabled healthy bias steady capture ki kd bp vel out "
                 "gain ratio p i d rail locked sat baseline".split()))


class Channel:
    """A view of one lane of the controller's arrays. v0..v4 kept per-channel
    state in per-channel objects; here it lives in length-N arrays, but the
    logger, the status line and sim/server.py all read and write
    `ctl.channels[i].<field>` -- including live gain edits from the browser UI --
    so this keeps that surface without a second copy of the state."""

    def __init__(self, ctl, idx):
        object.__setattr__(self, "ctl", ctl)
        object.__setattr__(self, "idx", idx)

    def __getattr__(self, k):
        if k in _VIEW:
            return getattr(self.ctl, _VIEW[k])[self.idx]
        raise AttributeError(k)

    def __setattr__(self, k, v):
        if k in _VIEW:
            getattr(self.ctl, _VIEW[k])[self.idx] = v
        else:
            object.__setattr__(self, k, v)


class Controller:
    """The fast-lock state machine. main() drives it from the serial stream and
    sim/server.py drives the same class from a simulated plant, so there is no
    second implementation of the loop."""

    def __init__(self, dac, dac_channels=None, enable=None, steady=None,
                 capture=None, ki=None, kd=None, bias=None):
        f = lambda v, d: np.array(d if v is None else v, dtype=float)
        self.enabled = np.array(ENABLE_CHANNEL if enable is None else enable, bool)
        self.steady, self.capture = f(steady, STEADY_GAIN), f(capture, CAPTURE_GAIN)
        self.ki, self.kd, self.bias = f(ki, KI_GAIN), f(kd, KD_GAIN), f(bias, BIAS)
        self.act = Actuator(dac, DAC_CHANNELS if dac_channels is None else dac_channels)

        self.hp, self.lp = OnePole(BP_LOW_HZ, "high"), OnePole(BP_HIGH_HZ)
        self.dsm, self.dfilt = OnePole(DERIV_SMOOTH_HZ), OnePole(D_SMOOTH_HZ)
        self.rms_sch, self.rms_run = _bank(SCHEDULE_WINDOW_S), _bank(ENVELOPE_WINDOW_S)
        self.env_hist = deque()          # (t, envelope) for the trend test
        self.rms_lock = _bank(LOCK_WINDOW_S)

        for k in ("bp prev_bp vel p i d prev_vel clip_excess gain ratio baseline "
                  "sat_streak rail_sum").split():
            setattr(self, k, np.zeros(N))
        for k in "primed sat locked rail".split():
            setattr(self, k, np.zeros(N, bool))
        self.healthy = np.ones(N, bool)
        self.clear_t, self.excess_since = np.full(N, np.inf), np.full(N, np.inf)
        self.locked_since = np.full(N, np.inf)
        self.out, self.prev_out = self.bias.copy(), self.bias.copy()
        self.rail_hist, self.calib, self.events = deque(), [], []
        self.channels = [Channel(self, i) for i in range(N)]
        self.state, self.calib_start, self.damping_start = "CALIBRATING", 0.0, None
        self.clear_since = self.lock_time = None
        self.filt_on = self.locked_announced = False
        self.fault_count = self.demote_count = self.reuse_count = 0
        self.damped_for = None       # how long DAMPING lasted before the fault
        # --- fast-calib bookkeeping ---
        self.sub_rms = []            # one RMS vector per COMPLETED sub-window
        self.sub_start, self.sub_n0 = 0.0, 0
        self.calib_took = None       # how long the last calibration actually ran
        self.calib_early = False     # ...and whether it converged or hit the ceiling
        # --- warm-restart bookkeeping ---
        self.baseline_t = None       # when the live baseline was measured
        self.fault_railed = False    # was the fault a SENSOR failure?

    # ---- helpers ----------------------------------------------------------
    def drain_events(self):
        out, self.events = self.events, []
        return out

    def _say(self, msg):
        self.events.append("\n" + msg + "\n")

    def _fault(self, t, msg, railed=False):
        self._say("!! " + msg)
        # Only an engagement that actually happened counts. v5 read damping_start
        # unconditionally, which after a rail during CALIBRATING reports the
        # PREVIOUS engagement's length -- harmless there, load-bearing here,
        # since damped_for now decides whether the reuse budget resets.
        self.damped_for = (t - self.damping_start
                           if self.state == "DAMPING" and self.damping_start is not None
                           else None)
        self.fault_railed = bool(railed)
        self.state, self.clear_since = "FAULT", None
        self.fault_count += 1

    @staticmethod
    def _who(m):
        return ", ".join(f"ch{i}" for i in np.nonzero(m)[0])

    @staticmethod
    def _hold(cond, since, t):
        """Per-lane stopwatch: start where `cond` just became true, clear where it
        is false. np.inf is 'not running'."""
        return np.where(cond, np.where(np.isinf(since), t, since), np.inf)

    def _rail_check(self, counts, t):
        railed = (counts <= RAIL_LOW_COUNTS) | (counts >= RAIL_HIGH_COUNTS)
        self.rail_hist.append((t, railed))
        self.rail_sum = self.rail_sum + railed
        while self.rail_hist and t - self.rail_hist[0][0] > RAIL_SUSTAIN_S:
            self.rail_sum = self.rail_sum - self.rail_hist.popleft()[1]
        # A full window must have elapsed, or the first samples of a run trip it
        # on a sample size of one.
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= RAIL_SUSTAIN_S * 0.9
        self.rail = np.logical_and(
            spanned, self.rail_sum / max(len(self.rail_hist), 1) >= RAIL_FRACTION)

    # ---- calibration ------------------------------------------------------
    def _close_subwindow(self, t):
        """Fold the samples since the last boundary into one RMS vector. The raw
        samples are KEPT as well: the ceiling path re-splits the whole window the
        way v5 does, and must give v5's answer."""
        a = np.asarray(self.calib[self.sub_n0:])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a ** 2).mean(0)))
        self.sub_n0, self.sub_start = len(self.calib), t

    def _calib_stationary(self):
        """Has the noise floor stopped moving? TWO tests, and both are needed.

        (1) the trailing CALIB_AGREE_N sub-windows agree with each other. Catches
            a floor that is still obviously jumping around.
        (2) their median agrees with the median of EVERY sub-window so far.
            Catches the case (1) cannot: a ringdown is slow -- tau = Q/(pi*f0) is
            ~16 s, comparable to the whole window -- so three consecutive
            sub-windows part-way down it can look perfectly stationary while
            sitting well away from the true floor. NOT hypothetical: with (1)
            alone at this tolerance a 6 V/s kick at t = 2 s reads as settled at
            t = 16.01 s (its trailing three agree to 1.187) and stores a baseline
            skewed 37.0% -- past the 35% line the suite asserts, so `fast-calib`
            would have broken `runaway-baseline`. Test (2) sees the same three
            windows sitting 1.400 away from the median of the window as a whole
            and runs to the ceiling instead, for 17.6%.

        Deliberately not a variance or an F-test: this compares the same
        statistic the baseline is made of, so a pass means "the number I am about
        to store has stopped changing" rather than "some other number agrees".
        """
        w = self.sub_rms
        if len(w) < max(CALIB_MIN_SUBWINDOWS, CALIB_AGREE_N):
            return False
        a = np.asarray(w)
        tail = a[-CALIB_AGREE_N:]
        if not bool((tail.max(0) / np.maximum(tail.min(0), 1e-9)
                     <= CALIB_AGREE_TOL).all()):
            return False
        m_tail, m_all = np.median(tail, 0), np.median(a, 0)
        spread = (np.maximum(m_tail, m_all)
                  / np.maximum(np.minimum(m_tail, m_all), 1e-9))
        return bool((spread <= CALIB_AGREE_TOL).all())

    def _set_baseline(self, early):
        """`early` picks the estimator, and the ceiling branch is v5's, unchanged:
        median of CALIB_SUBWINDOWS sub-window RMSs over the whole window, so a
        calibration that had to run the full CALIBRATION_S produces exactly the
        number v5 would have produced from the same samples. The early branch is
        the median of the trailing windows that were just shown to agree --
        deliberately not the median of everything, because on an early exit the
        first sub-window still carries the bandpass filter's own start-up
        transient (measured ~8% high) and there is no reason to average it in."""
        if early and len(self.sub_rms) >= CALIB_AGREE_N:
            self.baseline = np.maximum(
                np.median(np.asarray(self.sub_rms[-CALIB_AGREE_N:]), 0), 1e-6)
        else:
            a = np.asarray(self.calib)
            parts = ([p for p in np.array_split(a, CALIB_SUBWINDOWS) if len(p)]
                     if len(a) else [])
            self.baseline = (
                np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0), 1e-6)
                if parts else np.full(N, 1e-6))
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0

    def _start_calibration(self, t):
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_start = self.sub_start = t

    def _health(self, t):
        """Demote a railed channel; re-arm it REARM_SUSTAIN_S after its rail
        clears. Only RAIL demotes -- a saturated channel's measurement is fine and
        only its output is clipped, so parking it would remove damping authority
        at peak amplitude. Filters and baseline are deliberately carried across
        the gap; the short RMS windows are not. versions.md section v4."""
        drop = self.healthy & self.rail
        self.healthy[drop], self.gain[drop] = False, 0.0   # so gain ramps from zero
        self.clear_t[self.rail] = np.inf
        idle = ~self.healthy & ~self.rail
        self.clear_t[idle & np.isinf(self.clear_t)] = t
        back = idle & (t - self.clear_t >= REARM_SUSTAIN_S)
        self.healthy[back], self.clear_t[back] = True, np.inf
        for i in np.nonzero(back)[0]:
            self.rms_run[i].reset()
            self.rms_sch[i].reset()
        self.excess_since[back], self.ratio[back] = np.inf, 0.0
        return drop, back

    # ---- the loop ---------------------------------------------------------
    def step(self, counts, volts, t, dt):
        counts, volts = np.asarray(counts, float), np.asarray(volts, float)
        self.bp = self.lp.update(self.hp.update(volts, dt), dt).copy()
        if self.filt_on:
            dv = (self.bp - self.prev_bp) / dt if dt > 0 else np.zeros(N)
            self.vel = self.dsm.update(dv, dt).copy()
        self.prev_bp, self.filt_on = self.bp, True
        self._rail_check(counts, t)

        if self.state == "CALIBRATING":
            self.calib.append(self.bp)
            if self.rail.any():
                self._fault(t, f"{self._who(self.rail)} railed during calibration -- "
                                f"check alignment.", railed=True)
            else:
                if t - self.sub_start >= CALIB_SUBWINDOW_S:
                    self._close_subwindow(t)
                ceiling = t - self.calib_start >= CALIBRATION_S
                early = (not ceiling) and self._calib_stationary()
                if early or ceiling:
                    if ceiling and len(self.calib) > self.sub_n0:
                        self._close_subwindow(t)     # flush the partial tail
                    self.calib_took, self.calib_early = t - self.calib_start, early
                    self._set_baseline(early)
                    self.baseline_t = t
                    self.reuse_count = 0
                    self.state, self.damping_start = "DAMPING", t
                    self.locked_announced = False
                    self._say(
                        f"[DAMPING] baseline set in {self.calib_took:.1f}s "
                        + ("(sub-windows agreed -- stopped early, ceiling is "
                           f"{CALIBRATION_S:.0f}s)" if early
                           else f"(ran the full {CALIBRATION_S:.0f}s ceiling -- the "
                                "floor never settled, median of sub-windows used)")
                        + ": "
                        + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                        + f". Gain schedules {CAPTURE_GAIN[0]:+.3f} -> "
                          f"{STEADY_GAIN[0]:+.3f}...")

        elif self.state == "DAMPING":
            self._damping(t, dt)

        elif self.state == "FAULT":
            self.gain[:] = 0.0
            # `sat` is refreshed in _actuate() every sample in every state, so this
            # is a live reading and not a latch: in FAULT the outputs sit at bias
            # and it clears on its own.
            if self.rail.any() or self.sat.any():
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = t
            elif t - self.clear_since >= FAULT_CLEAR_SUSTAIN_S:
                self._recover(t)

        self._actuate(dt)
        return self.state

    def _damping(self, t, dt):
        drop, back = self._health(t)
        live, n_conf = self.enabled & self.healthy, int(self.enabled.sum())
        n = int(live.sum())
        for i in np.nonzero(drop)[0]:
            self.demote_count += 1
            self._say(f"!! ch{i} railed -- held at bias, {n}/{n_conf} still damping.")
        for i in np.nonzero(back)[0]:
            self._say(f"[ch{i} back] rail clear {REARM_SUSTAIN_S:.0f}s -- re-engaging on its "
                      f"pre-event baseline {self.baseline[i]:.4f}V, {n}/{n_conf} damping.")
        if n_conf and n < MIN_HEALTHY_CHANNELS:
            return self._fault(t, f"quorum lost -- {n}/{n_conf} healthy, need "
                                  f"{MIN_HEALTHY_CHANNELS}. Freezing everything.",
                               railed=True)

        # Gain schedule: blend capture -> steady on amplitude over this channel's
        # own baseline, slew-limited. Doubles as the startup soft-start.
        self.ratio = np.where(live, _rms(self.rms_sch, t, self.bp, live) / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)

        # Runaway breaker, gated on `healthy` not `live`: a config-disabled channel
        # still watches its own amplitude, but a blind one measures nothing.
        env = _rms(self.rms_run, t, self.bp, self.healthy)
        # FIXED (v9), "runaway-trend". The old test was env > RUNAWAY_MULTIPLE x
        # baseline sustained RUNAWAY_SUSTAIN_S -- a LEVEL test. A runaway is the
        # loop pumping energy IN, which means amplitude GROWING; a large but
        # decaying envelope is the loop succeeding. The level test cannot tell
        # them apart, so after a kick it trips on a timer while the optic rings
        # down, every RUNAWAY_SUSTAIN_S, until the amplitude happens to fall
        # under the line. Measured on hardware 2026-08-04 (v8, 240 s, 3 kicks):
        # ten faults, and the envelope was falling through every one of them --
        # 5.97 -> 4.61, 3.00 -> 3.14, 2.72 -> 2.07. The only re-engagement that
        # survived started at 1.64, just under the 1.8 line. That is the whole
        # fault thrash, and it was in every version back to v0.
        #
        # So: still require the envelope to be high (a decaying transient is not
        # interesting no matter what it does), but ALSO require it to be growing
        # relative to where it was RUNAWAY_TREND_LAG_S ago. Both, sustained.
        self.env_hist.append((t, env.copy()))
        while self.env_hist and t - self.env_hist[0][0] > RUNAWAY_TREND_LAG_S * 2:
            self.env_hist.popleft()
        past = env
        for ts, e in self.env_hist:                    # oldest sample >= lag old
            if t - ts >= RUNAWAY_TREND_LAG_S:
                past = e
                break
        growing = env > past * RUNAWAY_GROWTH_FRAC
        high = env > self.baseline * RUNAWAY_MULTIPLE
        self.excess_since = self._hold(self.healthy & high & growing, self.excess_since, t)
        trip = self.healthy & ((t - self.excess_since >= RUNAWAY_SUSTAIN_S) | self.sat)
        if trip.any():
            return self._fault(t, f"{self._who(trip)} runaway/saturation -- freezing "
                                  f"all channels, entering FAULT.")

        # Lock detector. A demoted channel sits at bias with a flatlined bandpass,
        # which an RMS-only test reads as the steadiest axis on the rack, so it may
        # neither lock nor block a lock.
        quiet = live & (_rms(self.rms_lock, t, self.bp, live) < self.baseline * LOCK_RMS_FACTOR)
        self.locked_since = self._hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= LOCK_SUSTAIN_S)
        if n and self.locked[live].all() and not self.locked_announced:
            self.lock_time = t - self.damping_start
            self._say(f"*** LOCKED -- {self.lock_time:.1f}s after gain was applied "
                      f"(t={t:.1f}s total)"
                      + ("" if n == n_conf else f" -- DEGRADED, {n}/{n_conf} channels") + " ***")
            self.locked_announced = True

    def _actuate(self, dt):
        live = self.enabled & self.healthy & (self.state == "DAMPING")
        err = -self.vel                                # setpoint is zero velocity
        self.p = self.gain * err
        self.prev_vel[~self.primed] = self.vel[~self.primed]
        self.primed[:] = True
        # Derivative on the MEASUREMENT: identical while the setpoint is a constant
        # zero, but it cannot kick if that ever changes.
        dv = (self.vel - self.prev_vel) / dt if dt > 0 else np.zeros(N)
        self.prev_vel = self.vel.copy()
        self.d = -self.kd * self.dfilt.update(dv, dt)
        # Back-calculation anti-windup: the integrator unwinds at TRACK_TC_S from
        # last sample's clip rather than freezing at a limit.
        self.i = np.clip(self.i + (self.ki * err - self.clip_excess / TRACK_TC_S) * dt,
                         -I_CLAMP_V, I_CLAMP_V)

        raw = self.bias + self.p + self.i + self.d
        clipped = np.clip(raw, VMIN, VMAX)
        self.clip_excess = np.where(live, raw - clipped, 0.0)
        # Bumpless re-entry for every lane not actuating -- FAULT, config-disabled
        # and demoted are all the same case, so none can resume with a stale
        # integral or a derivative step across the gap.
        dead = ~live
        if dead.any():
            self.p[dead] = self.i[dead] = self.d[dead] = self.prev_vel[dead] = 0.0
            self.primed[dead] = False
            self.dfilt.reset(dead)
        cap = MAX_SLEW_PER_S * dt
        target = np.where(live, clipped, self.bias)
        self.out = self.prev_out + np.clip(target - self.prev_out, -cap, cap)
        self.prev_out = self.out.copy()
        # Maintained HERE, where the output is actually known, so it refreshes
        # every sample in every state including FAULT -- which is what stops the
        # v0/v1/v2 saturation latch.
        pinned = (self.out <= VMIN + 1e-6) | (self.out >= VMAX - 1e-6)
        self.sat_streak = np.where(pinned, self.sat_streak + 1, 0)
        self.sat = self.sat_streak > MAX_CONSECUTIVE_SATURATED
        self.act.send(self.out)

    def _why_reuse(self, t):
        """Whether the fault path may re-engage on the baseline in hand, and the
        one-line reason either way. The four re-measure cases are the file
        docstring's; this is where each of them is enforced."""
        if not bool(self.baseline.all()):
            return False, "nothing measured yet"
        if self.fault_railed:
            return False, ("the fault was a RAIL -- a floor measured through a "
                           "suspect sensor is suspect with it")
        age = None if self.baseline_t is None else t - self.baseline_t
        if age is not None and age > BASELINE_MAX_AGE_S:
            return False, (f"the baseline is {age:.0f}s old, over the "
                           f"{BASELINE_MAX_AGE_S:.0f}s limit")
        if self.reuse_count >= MAX_BASELINE_REUSE:
            return False, (f"already reused {self.reuse_count} times without a "
                           f"{FAST_REFAULT_S:.0f}s engagement in between")
        if self.damped_for is not None and self.damped_for < FAST_REFAULT_S:
            return True, (f"re-faulted after only {self.damped_for:.1f}s, so the "
                          f"disturbance is still there and a fresh window would "
                          f"measure it rather than the floor")
        return True, ("the previous engagement was healthy, so this baseline is "
                      "known good and the fault was a transient -- measuring a "
                      "new floor now would measure the ringdown")

    def _recover(self, t):
        # Leaving FAULT. Either straight back to DAMPING on the baseline already
        # in hand, or a full recalibration. CALIBRATION_S of zero gain is not free
        # and is not always even correct -- see _why_reuse and the file docstring.
        if self.damped_for is not None and self.damped_for >= FAST_REFAULT_S:
            self.reuse_count = 0        # the loop itself validated this baseline
        reuse, why = self._why_reuse(t)
        keep, keep_t = self.baseline.copy(), self.baseline_t
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        for k in "baseline gain ratio sat_streak clip_excess p i d prev_vel".split():
            getattr(self, k)[:] = 0.0
        self.primed[:] = self.sat[:] = self.locked[:] = False
        self.locked_since[:] = self.excess_since[:] = self.clear_t[:] = np.inf
        self.healthy[:] = True          # a recovery forgets every demotion
        self.dfilt.reset()
        for b in self.rms_sch + self.rms_run + self.rms_lock:
            b.reset()
        if reuse:
            self.reuse_count += 1
            self.baseline, self.baseline_t = keep, keep_t
            self.state, self.damping_start = "DAMPING", t
            self.locked_announced = False
            self._say(f"[recovered] re-engaging on the baseline already in hand -- "
                      f"{why} ({self.reuse_count}/{MAX_BASELINE_REUSE} before a "
                      f"forced re-calibration). Gain ramps from zero.")
        else:
            self.reuse_count = 0
            self.baseline_t, self.damping_start = None, None
            self._start_calibration(t)
            self.state = "CALIBRATING"
            self._say(f"[recovered] all channels clear for {FAULT_CLEAR_SUSTAIN_S:.0f}s "
                      f"-- re-calibrating ({why}) for up to {CALIBRATION_S:.0f}s.")
        self.fault_railed = False
        self.clear_since = self.lock_time = None

    # ---- reporting --------------------------------------------------------
    def status_line(self, t):
        def one(i):
            if not self.enabled[i]:
                return f"ch{i}:off  bp={self.bp[i]:+.3f}V"
            if not self.healthy[i]:
                return f"ch{i}:DOWN bp={self.bp[i]:+.3f}V"
            flags = ("LOCK" if self.locked[i] else "....") + ("!RAIL" if self.rail[i] else "")
            return (f"ch{i}:{flags} g={self.gain[i]:+.4f} "
                    f"bp={self.bp[i]:+.3f}V ratio={self.ratio[i]:.2f}")
        return f"[{t:7.1f}s] {self.state:11s} " + "  ".join(one(i) for i in range(N))

    def csv_row(self, t, counts):
        """`healthy` is the 13th column and `rail` cannot replace it: a demotion
        outlasts the rail that caused it by REARM_SUSTAIN_S."""
        row = [f"{t:.4f}", self.state]
        for i in range(N):
            row += [str(int(counts[i])), f"{counts[i] * (A_VCC / ADC_MAX_COUNTS):.4f}"]
            row += [format(getattr(self, k)[i], f) for k, f in _LOG]
            row += [str(int(v[i])) for v in (self.rail, self.locked, self.healthy)]
        return ",".join(row)


def main():
    dac = DACController(port=PORT)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))
    input("DAC biases set. Press Enter to start fast-lock damping (Ctrl+C to stop)... ")
    dac.start_stream()
    ctl = Controller(dac)

    os.makedirs("data", exist_ok=True)
    path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    log = open(path, "w", buffering=1)
    log.write(CSV_HEADER + "\n")
    print(f"Logging to {path}\n\n[{ctl.state}] measuring baseline noise (outputs held "
          f"at bias, no damping yet). This stops as soon as the floor settles, and "
          f"after {CALIBRATION_S:.0f}s at the latest...\n")

    start = prev = time.time()
    last_status, rows = 0.0, 0
    try:
        while True:
            sample = read_sample(dac.ser)
            if sample is None:
                continue
            counts, volts = sample
            now = time.time()
            dt, prev = min(max(now - prev, 1e-4), 0.05), now
            t = now - start
            ctl.step(counts, volts, t, dt)
            for msg in ctl.drain_events():
                print(msg)
            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                print(ctl.status_line(t))
            log.write(ctl.csv_row(t, counts) + "\n")
            rows += 1
            if rows % CSV_FLUSH_EVERY_N == 0:
                log.flush()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        log.close()
        dac.stop_stream()
        print("Returning DAC outputs to bias voltages...")
        for c, b in zip(DAC_CHANNELS, BIAS):
            try:
                dac.set_voltage(channel=c, voltage=float(b))
            except RuntimeError:
                pass                      # best effort; the board resets on reconnect
        time.sleep(0.1)
        dac.close()
        print(f"Log saved to {path}")


if __name__ == "__main__":
    main()
