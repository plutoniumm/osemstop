"""
v5 -- v4 as arrays, plus one fix.
=================================
Two changes, kept separate on purpose:

1. THE SHAPE. Four channels are four lanes of one numpy array instead of four
   objects running identical scalar arithmetic, so each filter, the PID, the
   gain schedule and every interlock is one expression rather than four. That
   part is a pure rewrite: it was verified to reproduce v4's output exactly --
   same ratios, lock time, peak P/I/D and re-arm delay, on two different plants
   -- before anything below was added.

2. `fast-refault`. v0..v4 answer every fault by re-calibrating: CALIBRATION_S of
   OPEN LOOP, gain at zero. That is right for a transient and wrong for a
   disturbance that is still there, because the optic rings straight back up to
   the amplitude that faulted it and faults again on contact. Measured over
   200 s of sustained clipping (drive_amp 2.5), v3/v4 spend 4% of samples
   actually damping across 7 faults and never lock; v5 spends 87% and locks in
   10.2 s. See FAST_REFAULT_S below and versions.md section v5.

WHY anything here is what it is -- the gain signs, the quorum of one, the
carried-forward baseline, the three defect fixes -- is in versions.md and
provenance.md. This file is the mechanism, not the argument.

  CALIBRATING  gain 0; baseline = median of CALIB_SUBWINDOWS sub-window RMSs.
               Any rail faults the rig -- you cannot calibrate a blind sensor,
               which is what guarantees a demoted channel has a baseline to
               come back to.
  DAMPING      u = PID(-vel), gain scheduled by amplitude, slew-limited. A
               railed channel is DEMOTED (held at bias, the rest keep damping)
               and re-arms REARM_SUSTAIN_S after its rail clears. The rig faults
               only on a runaway, a pinned actuator, or losing the last channel.
  FAULT        all outputs at bias until clear for FAULT_CLEAR_SUSTAIN_S, then
               either a full recalibration (which also clears every demotion) or
               -- if the last engagement lasted under FAST_REFAULT_S, so the
               cause is evidently still present -- straight back to DAMPING on
               the baseline already in hand, gain ramping from zero.

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

VERSION_TAG, BENCH_STATUS = "v5", "untested"
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault")
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

CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # subwindows >= 3, to outvote one
LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0
ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE, RUNAWAY_SUSTAIN_S = 2.0, 1.8, 2.0
MAX_CONSECUTIVE_SATURATED = 30                 # samples, not seconds

# Rail: raw counts, so there is no float-rounding ambiguity, and a FRACTION of a
# window rather than an unbroken run, so one noise sample dilutes the evidence
# instead of erasing it.
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80

FAULT_CLEAR_SUSTAIN_S = 5.0
REARM_SUSTAIN_S = 2.0        # 5 tau of the 0.4 Hz highpass; timed from rail-clear
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md §3)
# A fault this soon after engaging means whatever caused it is still present, and
# a fresh CALIBRATION_S of open-loop measurement is then 20 s of ZERO gain spent
# letting the optic ring back up -- which is what caused the fault. Reuse the last
# baseline and re-engage immediately instead. Bounded, so a stale baseline cannot
# survive indefinitely: after MAX_BASELINE_REUSE tries it calibrates properly and
# accepts the cost. Measured effect in versions.md section v5.
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4

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

    # ---- helpers ----------------------------------------------------------
    def drain_events(self):
        out, self.events = self.events, []
        return out

    def _say(self, msg):
        self.events.append("\n" + msg + "\n")

    def _fault(self, t, msg):
        self._say("!! " + msg)
        self.damped_for = (None if self.damping_start is None
                           else t - self.damping_start)
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

    def _set_baseline(self):
        a = np.asarray(self.calib)
        parts = [p for p in np.array_split(a, CALIB_SUBWINDOWS) if len(p)] if len(a) else []
        self.baseline = (np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0), 1e-6)
                         if parts else np.full(N, 1e-6))
        self.calib = []

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
                                f"check alignment.")
            elif t - self.calib_start >= CALIBRATION_S:
                self._set_baseline()
                self.state, self.damping_start = "DAMPING", t
                self.locked_announced = False
                self._say("[DAMPING] baseline set ("
                          + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                          + f"). Gain schedules {CAPTURE_GAIN[0]:+.3f} -> "
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
                self._recalibrate(t)

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
                                  f"{MIN_HEALTHY_CHANNELS}. Freezing everything.")

        # Gain schedule: blend capture -> steady on amplitude over this channel's
        # own baseline, slew-limited. Doubles as the startup soft-start.
        self.ratio = np.where(live, _rms(self.rms_sch, t, self.bp, live) / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)

        # Runaway breaker, gated on `healthy` not `live`: a config-disabled channel
        # still watches its own amplitude, but a blind one measures nothing.
        env = _rms(self.rms_run, t, self.bp, self.healthy)
        self.excess_since = self._hold(
            self.healthy & (env > self.baseline * RUNAWAY_MULTIPLE), self.excess_since, t)
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

    def _recalibrate(self, t):
        # If the last engagement lasted no time at all, whatever caused the fault
        # is still out there, and re-measuring the noise floor underneath it is
        # both wrong (the floor is the disturbance) and expensive (CALIBRATION_S
        # of zero gain, during which the optic rings straight back up to the
        # amplitude that faulted it). Re-engage on the baseline we already have.
        reuse = (self.damped_for is not None and self.damped_for < FAST_REFAULT_S
                 and bool(self.baseline.all()) and self.reuse_count < MAX_BASELINE_REUSE)
        keep = self.baseline.copy()
        self.calib = []
        for k in "baseline gain ratio sat_streak clip_excess p i d prev_vel".split():
            getattr(self, k)[:] = 0.0
        self.primed[:] = self.sat[:] = self.locked[:] = False
        self.locked_since[:] = self.excess_since[:] = self.clear_t[:] = np.inf
        self.healthy[:] = True          # a recalibration forgets every demotion
        self.dfilt.reset()
        for b in self.rms_sch + self.rms_run + self.rms_lock:
            b.reset()
        if reuse:
            self.reuse_count += 1
            self.baseline = keep
            self.state, self.damping_start = "DAMPING", t
            self.locked_announced = False
            self._say(f"[recovered] re-engaging on the previous baseline -- re-faulted "
                      f"after only {self.damped_for:.1f}s, so the disturbance is still "
                      f"there ({self.reuse_count}/{MAX_BASELINE_REUSE} before a forced "
                      f"re-calibration). Gain ramps from zero.")
        else:
            self.reuse_count = 0
            self.state, self.calib_start = "CALIBRATING", t
            self._say(f"[recovered] all channels clear for {FAULT_CLEAR_SUSTAIN_S:.0f}s "
                      f"-- re-calibrating and resuming.")
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
    print(f"Logging to {path}\n\n[{ctl.state}] measuring baseline noise for "
          f"{CALIBRATION_S:.0f}s (outputs held at bias, no damping yet)...\n")

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
