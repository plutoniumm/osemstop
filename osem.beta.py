"""
beta: v5's loop, three supervisor changes; only WHEN the loop closes differs.

fast-calib: CALIBRATION_S is a ceiling, exit early once sub-windows agree. Sim
  20.00->6.01 s calib, 30.6->16.5 s lock, 19.7%->17.6% skew (35% line). NOT
  confirmed on hardware: the bench floor never meets CALIB_AGREE_TOL.
warm-restart: a fault after healthy damping reuses the baseline in hand, bounded
  by _why_reuse. Sim, two 15 V/s kicks: v5 never re-locks inside 115 s, beta
  re-locks 10.7 s after a 5 s all-clear.
runaway-trend: breaker requires growth, not level. Bench 2026-08-04, 4 ch,
  3 kicks: 4 faults against v8's 10, 26.6% DAMPING.

CALIBRATING (gain 0, a rail faults the rig) -> DAMPING (u = PID(-vel), a railed
channel is demoted) -> FAULT (outputs at bias). `enabled` config, `healthy`
runtime, both gate output; idle timers are np.inf; `baseline` is 0.0, not None.
"""

import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from pyDAC import DACController

PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 4   # sensing: 5.02 V / 1023 counts

# Coil DAC per sensor 0..3, rewired 2026-08-03; full 8-pair map [1,3,5,7,0,2,4,6]
# (provenance.md). A wrong map is invisible to the simulator and the interlocks.
DAC_CHANNELS = [1, 3, 5, 7]

VERSION_TAG, BENCH_STATUS = "beta", "validated"   # ran 2026-08-04, see versions.md
# Best DAMPING fraction on record, 83.1% (ladder.py).
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend")
ENABLE_CHANNEL = [True, True, True, True]

# Actuation: 2.5 V DAC, held to 0-0.5 V about BIAS = 0.25 (see CLAUDE.md item 3).
BIAS = np.full(N, 0.25)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 0.5, 2.0

# STEADY: bench-validated -0.030 on ch0. CAPTURE: large amplitude only, NOT
# validated. STEADY_GAIN[2] is POSITIVE, ch2 is mounted the other way round
# (provenance.md Table 1): not a typo, a wrong sign PUMPS. Kp = -0.040 is the
# instability / rail onset the simulator cannot reproduce: never exceed -0.035.
STEADY_GAIN = np.array([-0.030, -0.030, +0.010, -0.030])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035])

# On a velocity loop the integral is DISPLACEMENT (a spring, moves the pole) and
# the derivative ACCELERATION (noisiest term). Same sign per channel as Kp.
KI_GAIN = np.array([-0.040, -0.040, +0.040, -0.040])
KD_GAIN = np.array([-0.0015, -0.0015, +0.0015, -0.0015])
D_SMOOTH_HZ, I_CLAMP_V, TRACK_TC_S = 2.0, 0.15, 0.5   # I_CLAMP is only a backstop

BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ = 0.4, 3.0, 5.0
CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
SCHEDULE_WINDOW_S, GAIN_SLEW_PER_S = 1.0, 0.02

# CALIB_AGREE_TOL is a ratio (max/min) off a measured gap: the trailing three
# agree to 1.09 then ~1.03 in a quiet lab, 1.19-1.66 under a 6 V/s kick over 20 s.
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # v5's ceiling path, unchanged
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3   # 2 cycles of ~1 Hz; floor 6 s
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20

LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0
ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE, RUNAWAY_SUSTAIN_S = 2.0, 1.8, 2.0
# A runaway grows over one ENVELOPE_WINDOW_S; a Q~50 ringdown falls.
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 2.0, 1.02   # 1.02 = 2% noise headroom
MAX_CONSECUTIVE_SATURATED = 30                 # samples, not seconds

# Rail: RAW COUNTS, because a railed sensor flatlines the bandpass and an RMS-only
# check calls that perfect stability. A FRACTION of a window, not an unbroken run.
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80

FAULT_CLEAR_SUSTAIN_S = 5.0
REARM_SUSTAIN_S = 2.0        # 5 tau of the 0.4 Hz highpass; timed from rail-clear
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md §3)

# Baseline reuse on the fault path (_why_reuse); FAST_REFAULT_S of damping resets
# the budget. The age cap is policy: long vs the 16 s ringdown, short vs lab drift.
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0

STATUS_PERIOD_S, CSV_FLUSH_EVERY_N = 5.0, 200

_LOG = (("bp", ".5f"), ("vel", ".5f"), ("out", ".4f"), ("gain", ".5f"),
        ("ratio", ".4f"), ("p", ".5f"), ("i", ".5f"), ("d", ".5f"))
CSV_HEADER = "time_s,state," + ",".join(
    f"ch{i}_{c}" for i in range(N)
    for c in ("counts", "V") + tuple(k for k, _ in _LOG) + ("rail", "locked", "healthy"))


class OnePole:
    """One-pole low/high pass, N lanes. `on` is per lane so a reset lane primes
    from its first sample, not zero."""

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
    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = window_s, deque(), 0.0

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        # np.maximum is load-bearing: the incremental sum leaves a residue of the
        # transient's own magnitude, measured -4.5e-14 after a 6 V/s kick. sqrt of
        # a negative is nan, nan compares False, breaker and lock detector die.
        return (np.sqrt(np.maximum(self.sq, 0.0) / len(self.buf)) if self.buf
                else self.sq * 0.0)

    def reset(self):
        self.buf.clear()
        self.sq = 0.0


def _bank(window_s):
    return [SlidingRMS(window_s) for _ in range(N)]


def _rms(bank, t, x, m):
    """A lane outside `m` is not fed, so a blind channel's flatlined bandpass
    never enters the window."""
    return np.array([bank[i].update(t, x[i]) if m[i] else 0.0 for i in range(N)])


class Actuator:
    """DAC writes: 10 ms throttle, 0.5 mV deadband, missed ack swallowed."""

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
    """(counts, volts) or None. arduino.ino streams 8 columns, A0..A3 are OSEMs."""
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


# Channel field -> Controller array.
_VIEW = dict(zip("enabled healthy bias steady_gain capture_gain ki kd bp_out vel out "
                 "active_gain last_ratio p_term i_term d_term rail_fault locked "
                 "saturated_flag baseline_rms".split(),
                 "enabled healthy bias steady capture ki kd bp vel out "
                 "gain ratio p i d rail locked sat baseline".split()))


class Channel:
    """One lane of the arrays; the logger, status line and sim/server.py write
    through `ctl.channels[i].<field>`."""

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
    """State machine; main() drives it from serial, sim/server.py from a plant."""

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
        self.damped_for = None       # DAMPING time before the fault
        self.sub_rms = []            # one RMS vector per COMPLETED sub-window
        self.sub_start, self.sub_n0 = 0.0, 0
        self.calib_took = None       # last calibration's duration...
        self.calib_early = False     # ...and whether it converged or hit the ceiling
        self.baseline_t = None       # when the live baseline was measured
        self.fault_railed = False    # was the fault a SENSOR failure?

    def drain_events(self):
        out, self.events = self.events, []
        return out

    def _say(self, msg):
        self.events.append("\n" + msg + "\n")

    def _fault(self, t, msg, railed=False):
        self._say("!! " + msg)
        # Only a real engagement counts: damped_for gates the reuse budget.
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
    def _hold(cond, since, t):          # per-lane stopwatch; np.inf = not running
        return np.where(cond, np.where(np.isinf(since), t, since), np.inf)

    def _rail_check(self, counts, t):
        railed = (counts <= RAIL_LOW_COUNTS) | (counts >= RAIL_HIGH_COUNTS)
        self.rail_hist.append((t, railed))
        self.rail_sum = self.rail_sum + railed
        while self.rail_hist and t - self.rail_hist[0][0] > RAIL_SUSTAIN_S:
            self.rail_sum = self.rail_sum - self.rail_hist.popleft()[1]
        # A full window must have elapsed, or a run trips on a sample size of one.
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= RAIL_SUSTAIN_S * 0.9
        self.rail = np.logical_and(
            spanned, self.rail_sum / max(len(self.rail_hist), 1) >= RAIL_FRACTION)

    def _close_subwindow(self, t):
        """One RMS vector per sub-window; raw samples are KEPT for the ceiling
        path, which re-splits them v5's way."""
        a = np.asarray(self.calib[self.sub_n0:])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a ** 2).mean(0)))
        self.sub_n0, self.sub_start = len(self.calib), t

    def _calib_stationary(self):
        """TWO tests, both needed: (1) the trailing CALIB_AGREE_N sub-windows
        agree, (2) their median agrees with the median of all so far. A ringdown
        is slow (tau = Q/(pi*f0) ~ 16 s), so windows part-way down one look
        stationary while off the floor: with (1) alone a 6 V/s kick at t = 2 s
        reads settled at 16.01 s (trailing three agree to 1.187) and stores a
        baseline skewed 37.0%, past the suite's 35% line. (2) sees them 1.400 off
        the whole-window median: ceiling, 17.6%."""
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
        """Ceiling branch is v5's: median of CALIB_SUBWINDOWS RMSs. Early branch
        takes only the trailing windows shown to agree; the first carries the
        bandpass start-up transient, ~8% high."""
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
        """Demote a railed channel, re-arm REARM_SUSTAIN_S after its rail clears.
        Only RAIL demotes: a saturated channel still measures, and parking it
        drops authority at peak amplitude. Filters and baseline survive."""
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
            # `sat` is refreshed in _actuate() in every state, so it is live, not
            # a latch: in FAULT it clears on its own.
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

        # Blend capture -> steady on amplitude over baseline; also soft-start.
        self.ratio = np.where(live, _rms(self.rms_sch, t, self.bp, live) / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)

        # Gated on `healthy`, not `live`: a disabled channel still watches, a
        # blind one measures nothing.
        env = _rms(self.rms_run, t, self.bp, self.healthy)
        # "runaway-trend": a LEVEL test cannot tell growth from a decaying
        # envelope, so it trips every RUNAWAY_SUSTAIN_S through a ringdown.
        # Hardware 2026-08-04 (v8, 240 s, 3 kicks): ten faults, envelope falling
        # through all, 5.97 -> 4.61, 3.00 -> 3.14, 2.72 -> 2.07. High AND growing.
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

        # Gated on `live`: a demoted channel's flatlined bandpass reads to RMS as
        # the steadiest axis on the rack.
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
        # Derivative on the MEASUREMENT: no kick if the setpoint ever changes.
        dv = (self.vel - self.prev_vel) / dt if dt > 0 else np.zeros(N)
        self.prev_vel = self.vel.copy()
        self.d = -self.kd * self.dfilt.update(dv, dt)
        # Back-calculation anti-windup: unwinds at TRACK_TC_S, never freezes.
        self.i = np.clip(self.i + (self.ki * err - self.clip_excess / TRACK_TC_S) * dt,
                         -I_CLAMP_V, I_CLAMP_V)

        raw = self.bias + self.p + self.i + self.d
        clipped = np.clip(raw, VMIN, VMAX)
        self.clip_excess = np.where(live, raw - clipped, 0.0)
        # Bumpless re-entry for every idle lane: no stale integral, no D step.
        dead = ~live
        if dead.any():
            self.p[dead] = self.i[dead] = self.d[dead] = self.prev_vel[dead] = 0.0
            self.primed[dead] = False
            self.dfilt.reset(dead)
        cap = MAX_SLEW_PER_S * dt
        target = np.where(live, clipped, self.bias)
        self.out = self.prev_out + np.clip(target - self.prev_out, -cap, cap)
        self.prev_out = self.out.copy()
        # Maintained HERE: refreshed every sample in every state including FAULT,
        # which is what stops the saturation latch.
        pinned = (self.out <= VMIN + 1e-6) | (self.out >= VMAX - 1e-6)
        self.sat_streak = np.where(pinned, self.sat_streak + 1, 0)
        self.sat = self.sat_streak > MAX_CONSECUTIVE_SATURATED
        self.act.send(self.out)

    def _why_reuse(self, t):
        """May the fault path reuse the baseline in hand, and why. Four cases
        re-measure: none yet, RAIL fault, too old, budget spent."""
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
        # Leaving FAULT: back to DAMPING on the baseline in hand, or recalibrate.
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
        """`healthy` (col 13) is not `rail`: a demotion outlasts its rail by
        REARM_SUSTAIN_S."""
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
