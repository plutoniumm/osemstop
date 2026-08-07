"""
alpha: auto-disable. A blind axis is demoted and parked; the rest keep damping.

A railed sensor demotes that channel alone (held at BIAS, gain zeroed); global
FAULT only below MIN_HEALTHY_CHANNELS. A rail during CALIBRATING is still
global, and so are runaway and saturation: four loops on one rigid body cannot
localise growing amplitude, and a saturated channel still measures fine.

Four independent SISO loops, OSEM i drives coil i and reads nothing else, so
one healthy channel is a slow rig, not a broken one (provenance.md section 3:
one pair damps the whole body in ~70 s, against ~17 s for four).

Watch: auto-disable is unconfirmed on the bench. It fires on a full RAIL, not
on PARTIAL clipping (the bench measured 2.19% of samples at a rail, under
RAIL_FRACTION = 0.80); centring the OSEM rest points is the fix, README.md.
"""

import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from pyDAC import DACController

PORT = "COM7"
A_VCC = 5.02                  # sensing scale: 5.02 V across 1023 ADC counts
ADC_MAX_COUNTS = 1023

DAC_CHANNELS = [1, 3, 5, 7]   # coil DAC for sensor 0..3 (A0..A3). Rewired
                              # 2026-08-03; full 8-pair map [1,3,5,7,0,2,4,6].
                              # A WRONG MAP IS INVISIBLE to sim and interlocks.

# provenance.md §4 Table 1: all four damped together, tau = 4.55, 5.49, 3.58,
# 5.05 s for A0..A3. Disabled channels are still filtered, rail-checked, logged.
VERSION_TAG, BENCH_STATUS = "alpha", "validated"   # ran 2026-08-04
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable")

# Bench 2026-08-04: ch0 alone locked in 14.7 s, all four in 11.1 s, rode three
# kicks faulting and RECOVERING each time. Coil map [1,3,5,7] validated there;
# ch2 damped best. auto-disable is unconfirmed on hardware (it needs a RAILED
# sensor; kicks give runaway/saturation). versions.md "On the bench".
BENCH_STATUS = "validated"

ENABLE_CHANNEL = [True, True, True, True]

# Actuation scale: a 2.5 V DAC restricted to the 0.0-0.5 V window around BIAS.
BIAS = np.array([0.25, 0.25, 0.25, 0.25])
VMIN, VMAX = 0.0, 0.5
MAX_SLEW_PER_S = 2.0          # volts/second on the output

# Kp per channel. STEADY_GAIN is sweep-validated on ch0 (-0.030). CAPTURE_GAIN
# is a stronger transient value, NOT independently validated. Kp = -0.040 is the
# documented instability / rail onset: do not exceed -0.035 unattended.
# ch2 is POSITIVE because that OSEM is mounted the other way round. Not a typo:
# a wrong sign PUMPS the optic, it does not merely under-damp.
STEADY_GAIN  = np.array([-0.030, -0.030, +0.010, -0.030])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035])

# Gain schedule: rolling bandpass RMS over the channel's calibrated baseline.
# Above CAPTURE_HIGH_FRAC -> capture gain, below CAPTURE_LOW_FRAC -> steady.
CAPTURE_HIGH_FRAC = 0.6
CAPTURE_LOW_FRAC = 0.2
SCHEDULE_WINDOW_S = 1.0

GAIN_SLEW_PER_S = 0.02        # gain-units/s, so no gain step kicks a transient

# PID on velocity, setpoint zero, so e = -vel. Kp is STEADY/CAPTURE_GAIN above,
# the only term that removes energy. int(vel) is DISPLACEMENT, so Ki is a
# SPRING; d(vel)/dt is ACCELERATION, so Kd is negative MASS and the noisiest
# term. All three share ONE sign per channel: negative on ch0/1/3, plus on ch2.
KI_GAIN = np.array([-0.040, -0.040, +0.040, -0.040])
KD_GAIN = np.array([-0.0015, -0.0015, +0.0015, -0.0015])
D_SMOOTH_HZ = 2.0      # lowpass on the D term (acceleration estimate)
I_CLAMP_V = 0.15       # backstop only; TRACK_TC_S is the working anti-windup
TRACK_TC_S = 0.5       # back-calculation time constant, seconds. Must stay well
                       # above the sample period.

BP_LOW_HZ = 0.4
BP_HIGH_HZ = 3.0
DERIV_SMOOTH_HZ = 5.0

# Baseline is the MEDIAN of CALIB_SUBWINDOWS sub-windows: a high-Q suspension
# wanders on its ringdown timescale (Q/f0 ~ 50 s), so one window is not stable.
CALIBRATION_S = 20.0           # ~20 cycles of the ~1 Hz resonance, gain held at 0
CALIB_SUBWINDOWS = 5           # median across these; must be >= 3 to outvote one
LOCK_RMS_FACTOR = 0.35         # "locked" once rolling RMS < this * baseline
LOCK_SUSTAIN_S = 5.0
LOCK_WINDOW_S = 5.0

ENVELOPE_WINDOW_S = 2.0         # sliding window for the runaway check
RUNAWAY_MULTIPLE = 1.8
RUNAWAY_SUSTAIN_S = 2.0
MAX_CONSECUTIVE_SATURATED = 30

# Rail interlock, judged on RAW ADC COUNTS: a railed sensor flatlines the
# bandpass, which an RMS-only check reads as perfect stability. The old
# 3-count / 250-consecutive form armed in 0.51 s at <=5 mV sensor noise, 6.8 s
# at 7 mV, NEVER at >=10 mV; hence a FRACTION of a window, not an unbroken run.
RAIL_LOW_COUNTS = 12            # raw counts out of 0..1023, not volts, to avoid
RAIL_HIGH_COUNTS = 1011         # float-rounding ambiguity
RAIL_SUSTAIN_S = 0.5
RAIL_FRACTION = 0.80            # of the samples in the window, not consecutive

FAULT_CLEAR_SUSTAIN_S = 5.0     # all-clear for this long -> auto re-arm

# 5.0 tau of the slowest pole, the BP_LOW_HZ = 0.4 Hz highpass (tau = 0.398 s);
# a sensor coming off a rail is a step into it. Timer starts on rail CLEAR, not
# on demotion. Do not go below ~1.2 s (3 tau) without re-checking bumplessness.
REARM_SUSTAIN_S = 2.0
# Quorum. provenance.md section 3: one OSEM/coil pair damps the whole rigid body
# alone (~70 s, against ~17 s for four), so one healthy channel is still a rig.
MIN_HEALTHY_CHANNELS = 1

STATUS_PERIOD_S = 5.0
CSV_FLUSH_EVERY_N = 200


class OnePoleFilter:
    def __init__(self, cutoff_hz, kind="low"):
        self.tau = 1.0 / (2.0 * np.pi * cutoff_hz)
        self.kind = kind
        self.y = 0.0
        self.x_prev = 0.0
        self.initialized = False

    def update(self, x, dt):
        if not self.initialized:
            self.y = x if self.kind == "low" else 0.0
            self.x_prev = x
            self.initialized = True
            return self.y
        if self.kind == "low":
            a = dt / (self.tau + dt)
            self.y = self.y + a * (x - self.y)
        else:
            a = self.tau / (self.tau + dt)
            self.y = a * (self.y + x - self.x_prev)
        self.x_prev = x
        return self.y

    def reset(self):
        """Re-prime from the next sample. State carried across a FAULT would
        step the first sample after recovery."""
        self.y = 0.0
        self.x_prev = 0.0
        self.initialized = False


class SlidingRMS:
    def __init__(self, window_s):
        self.window_s = window_s
        self.buf = deque()
        self.sq_sum = 0.0

    def update(self, t, value):
        self.buf.append((t, value))
        self.sq_sum += value ** 2
        while self.buf and t - self.buf[0][0] > self.window_s:
            _, old_v = self.buf.popleft()
            self.sq_sum -= old_v ** 2
        n = len(self.buf)
        return float(np.sqrt(self.sq_sum / n)) if n > 0 else 0.0

    def reset(self):
        self.buf.clear()
        self.sq_sum = 0.0


class PID:
    """Parallel-form PID on the velocity error, in DAC volts.

    Kp arrives per call because the supervisor schedules and slew-limits it;
    ki/kd are fixed per channel and carry the channel's own loop sign.
    Anti-windup is back-calculation at TRACK_TC_S; I_CLAMP_V is a backstop.
    """

    def __init__(self, ki, kd):
        self.ki = ki
        self.kd = kd
        self.d_filter = OnePoleFilter(D_SMOOTH_HZ, kind="low")
        self.reset()

    def reset(self):
        """Bumpless re-entry. Called on every entry to a non-actuating state."""
        self.i_term = 0.0
        self.p_term = 0.0
        self.d_term = 0.0
        self.prev_meas = 0.0
        self.primed = False
        self.d_filter.reset()

    def update(self, meas, dt, kp, clip_excess=0.0):
        """`clip_excess` is (unclipped - clipped) from the PREVIOUS sample, V."""
        err = -meas                                   # setpoint is zero velocity
        self.p_term = kp * err

        # Derivative on the MEASUREMENT: cannot kick if the setpoint changes.
        if not self.primed:
            self.prev_meas = meas
            self.primed = True
        d_meas = (meas - self.prev_meas) / dt if dt > 0 else 0.0
        self.prev_meas = meas
        self.d_term = -self.kd * self.d_filter.update(d_meas, dt)

        # Back-calculation anti-windup, then the hard clamp as a backstop.
        self.i_term += (self.ki * err - clip_excess / TRACK_TC_S) * dt
        self.i_term = float(np.clip(self.i_term, -I_CLAMP_V, I_CLAMP_V))

        return self.p_term + self.i_term + self.d_term


def slew_limit(prev, target, dt, max_rate):
    max_step = max_rate * dt
    delta = np.clip(target - prev, -max_step, max_step)
    return prev + delta


class RateLimitedActuator:
    def __init__(self, dac, channel, min_interval_s=0.01, deadband_v=0.0005):
        self.dac = dac
        self.channel = channel
        self.min_interval_s = min_interval_s
        self.deadband_v = deadband_v
        self.last_sent_t = 0.0
        self.last_sent_v = None

    def send(self, voltage, force=False):
        now = time.time()
        due_by_time = (now - self.last_sent_t) >= self.min_interval_s
        due_by_change = (self.last_sent_v is None or
                          abs(voltage - self.last_sent_v) >= self.deadband_v)
        if force or (due_by_time and due_by_change):
            try:
                self.dac.set_voltage(channel=self.channel, voltage=voltage)
                self.last_sent_t = now
                self.last_sent_v = voltage
            except RuntimeError:
                pass  # a missed ack here and there is fine; next sample resends


def read_sample(ser):
    """Returns (counts, volts) as two length-4 arrays, or None."""
    if not ser.in_waiting:
        return None
    try:
        raw = ser.readline().decode("utf-8").strip()
        parts = raw.split(",")
        # arduino.ino streams eight columns (A0..A7); only A0..A3 carry OSEMs.
        if len(parts) < 4:
            return None
        counts = np.array([int(p) for p in parts[:4]])
        volts = counts * (A_VCC / ADC_MAX_COUNTS)
        return counts, volts
    except (ValueError, IndexError):
        return None


class Channel:
    """Filter chain, gain and health for one OSEM/coil pair. Filtering,
    rail-checking and calibration run regardless of `enabled`, so a disabled
    channel still logs and still trips the global interlock.

        enabled   static config; operator intent, never written here.
        healthy   runtime, cleared on this channel's rail and restored
                  REARM_SUSTAIN_S after it clears. Owned here.

    `actuating` is the conjunction and gates the output, so sim/server.py can
    assign `ch.enabled` every frame without un-demoting a blind channel.
    """

    def __init__(self, idx, dac, dac_channel, enabled, steady_gain, capture_gain,
                 ki, kd, bias):
        self.idx = idx
        self.enabled = enabled
        self.steady_gain = steady_gain
        self.capture_gain = capture_gain
        self.active_gain = 0.0
        self.bias = bias

        # active_gain is Kp, scheduled and slew-limited; ki/kd are fixed.
        self.pid = PID(ki, kd)
        self.clip_excess = 0.0        # carried to the next sample's anti-windup

        self.hp = OnePoleFilter(BP_LOW_HZ, kind="high")
        self.lp = OnePoleFilter(BP_HIGH_HZ, kind="low")
        self.deriv_smooth = OnePoleFilter(DERIV_SMOOTH_HZ, kind="low")
        self.prev_filt = 0.0
        self.filt_initialized = False
        self.bp_out = 0.0
        self.vel = 0.0

        self.actuator = RateLimitedActuator(dac, dac_channel)
        self.prev_out = bias
        self.out = bias

        self.calib_sq_sum = 0.0
        self.calib_n = 0
        self.calib_windows = []       # one RMS per finished sub-window
        self.baseline_rms = None

        self.lock_rms = SlidingRMS(LOCK_WINDOW_S)
        self.runaway_rms = SlidingRMS(ENVELOPE_WINDOW_S)
        self.schedule_rms = SlidingRMS(SCHEDULE_WINDOW_S)
        self.last_ratio = 0.0

        self.rail_hist = deque()      # (t, railed) over the last RAIL_SUSTAIN_S
        self.rail_fault = False
        self.excess_since = None
        self.sat_streak = 0
        self.saturated_flag = False
        self.locked = False
        self.locked_since = None

        # Runtime health, distinct from the static `enabled` above.
        self.healthy = True
        self.demoted_since = None     # t of the demotion, for the status line
        self.clear_since = None       # t the rail cleared; the re-arm timer

    @property
    def actuating(self):
        """Configured on AND not demoted: the one gate for being in the loop."""
        return self.enabled and self.healthy

    def filter_sample(self, volt, dt):
        hp_out = self.hp.update(volt, dt)
        bp_out = self.lp.update(hp_out, dt)
        if not self.filt_initialized:
            self.prev_filt = bp_out
            self.filt_initialized = True
            self.bp_out, self.vel = bp_out, 0.0
            return
        raw_vel = (bp_out - self.prev_filt) / dt if dt > 0 else 0.0
        self.prev_filt = bp_out
        self.vel = self.deriv_smooth.update(raw_vel, dt)
        self.bp_out = bp_out

    def check_rail(self, counts, now):
        """Rail on RAW COUNTS, over a FRACTION of a sliding window rather than
        an unbroken run, so one noise sample dilutes rather than erases.
        """
        railed = counts <= RAIL_LOW_COUNTS or counts >= RAIL_HIGH_COUNTS
        self.rail_hist.append((now, railed))
        while self.rail_hist and now - self.rail_hist[0][0] > RAIL_SUSTAIN_S:
            self.rail_hist.popleft()

        # Need a full window, or the first sample of a run trips it on n = 1.
        spanned = self.rail_hist and (now - self.rail_hist[0][0]) >= RAIL_SUSTAIN_S * 0.9
        frac = (sum(1 for _, r in self.rail_hist if r) / len(self.rail_hist)
                if self.rail_hist else 0.0)
        now_faulted = bool(spanned and frac >= RAIL_FRACTION)

        if now_faulted and not self.rail_fault:
            print(f"RAIL! ch={self.idx}, counts={counts:.0f}, "
                  f"{frac * 100:.0f}% of the last {RAIL_SUSTAIN_S:.1f}s")
        elif self.rail_fault and not now_faulted:
            print(f"rail cleared on ch={self.idx}")
        self.rail_fault = now_faulted

    def update_health(self, t):
        """Demote while this sensor is blind; re-arm REARM_SUSTAIN_S after the
        rail clears. Returns "demoted", "rearmed" or None, one per transition.

        Only RAIL demotes. A saturated channel's measurement is fine and only
        its output is clipped, so parking it would remove damping authority at
        peak amplitude; saturation keeps the global check_runaway() path.
        """
        if self.rail_fault:
            self.clear_since = None
            if self.healthy:
                self.healthy = False
                self.demoted_since = t
                # Ramp back from zero, not from the gain held when it blinded.
                self.active_gain = 0.0
                return "demoted"
            return None

        if self.healthy:
            return None

        if self.clear_since is None:
            self.clear_since = t
        elif t - self.clear_since >= REARM_SUSTAIN_S:
            self.healthy = True
            self.clear_since = None
            self.demoted_since = None
            # Filters and baseline_rms left alone: re-seeding a one-pole steps
            # it (versions.md hazard 2) and an event-time baseline is inflated.
            # RMS windows ARE cleared: a blind period of flatlined bandpass
            # would dilute the runaway check for one ENVELOPE_WINDOW_S.
            self.runaway_rms.reset()
            self.schedule_rms.reset()
            self.excess_since = None
            self.last_ratio = 0.0
            return "rearmed"
        return None

    def accumulate_calibration(self, frac_done=0.0):
        """Close off CALIB_SUBWINDOWS sub-windows for finish_calibration() to
        median. PARTIAL: one shock in the window skews the worst baseline 14.6%
        here against 20.5% for a single-window RMS, but gain is zero here so a
        shock rings down over Q/f0 ~ 50 s, and at 30 shocks/min this is WORSE
        (33% vs 27%), a 20 s window catching more than an 8 s one.
        """
        self.calib_sq_sum += self.bp_out ** 2
        self.calib_n += 1
        want = min(int(frac_done * CALIB_SUBWINDOWS), CALIB_SUBWINDOWS - 1)
        if want > len(self.calib_windows) and self.calib_n > 0:
            self.calib_windows.append(np.sqrt(self.calib_sq_sum / self.calib_n))
            self.calib_sq_sum, self.calib_n = 0.0, 0

    def finish_calibration(self):
        if self.calib_n > 0:
            self.calib_windows.append(np.sqrt(self.calib_sq_sum / self.calib_n))
        if self.calib_windows:
            # Median, not mean: a transient corrupts one sub-window, discarded.
            self.baseline_rms = max(float(np.median(self.calib_windows)), 1e-6)
        else:
            self.baseline_rms = 1e-6
        self.calib_sq_sum, self.calib_n = 0.0, 0
        self.calib_windows = []

    def update_schedule(self, t_rel, dt):
        """Blend active_gain toward the scheduled target at <= GAIN_SLEW_PER_S.
        Doubles as the startup soft-start, since active_gain begins at 0.0."""
        if self.baseline_rms is None:
            return
        local_rms = self.schedule_rms.update(t_rel, self.bp_out)
        ratio = local_rms / self.baseline_rms
        self.last_ratio = ratio
        frac = np.clip((ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0.0, 1.0)
        target_gain = self.steady_gain + frac * (self.capture_gain - self.steady_gain)
        max_step = GAIN_SLEW_PER_S * dt
        delta = np.clip(target_gain - self.active_gain, -max_step, max_step)
        self.active_gain += delta

    def check_runaway(self, t_rel):
        """Returns True if this channel just tripped (runaway or pinned actuator)."""
        if self.baseline_rms is None:
            return False
        # A demoted channel reads a railed sensor. Return before touching
        # runaway_rms, so the blind period never enters the window.
        if not self.healthy:
            return False
        local_rms = self.runaway_rms.update(t_rel, self.bp_out)
        tripped = False
        if local_rms > self.baseline_rms * RUNAWAY_MULTIPLE:
            if self.excess_since is None:
                self.excess_since = t_rel
            elif t_rel - self.excess_since >= RUNAWAY_SUSTAIN_S:
                tripped = True
        else:
            self.excess_since = None

        # Maintained in actuate(); read only here, so the two trips stay apart.
        if self.saturated_flag:
            tripped = True

        return tripped

    def update_lock(self, t_rel):
        # `actuating`, not `enabled`: a demoted channel's bandpass has flatlined,
        # which an RMS-only test reads as the steadiest axis on the rack.
        if not self.actuating or self.baseline_rms is None:
            self.locked = False
            self.locked_since = None
            return
        local_rms = self.lock_rms.update(t_rel, self.bp_out)
        if local_rms < self.baseline_rms * LOCK_RMS_FACTOR:
            if self.locked_since is None:
                self.locked_since = t_rel
            elif t_rel - self.locked_since >= LOCK_SUSTAIN_S:
                self.locked = True
        else:
            self.locked_since = None
            self.locked = False

    def actuate(self, dt, state):
        # `actuating`, not `enabled`: the one line enforcing a demotion.
        if state == "DAMPING" and self.actuating:
            u = self.pid.update(self.vel, dt, self.active_gain, self.clip_excess)
            unclipped = self.bias + u
            target = float(np.clip(unclipped, VMIN, VMAX))
            self.clip_excess = unclipped - target     # feeds back next sample
        else:
            # One call defines bumpless re-entry across the gap.
            self.pid.reset()
            self.clip_excess = 0.0
            target = self.bias
        self.out = slew_limit(self.prev_out, target, dt, MAX_SLEW_PER_S)
        self.prev_out = self.out

        # saturation-latch: maintained HERE so it updates in every state,
        # including FAULT where the output is at bias and the flag self-clears.
        # In check_runaway(), which never runs in FAULT, it latched forever.
        if self.out <= VMIN + 1e-6 or self.out >= VMAX - 1e-6:
            self.sat_streak += 1
        else:
            self.sat_streak = 0
        self.saturated_flag = self.sat_streak > MAX_CONSECUTIVE_SATURATED

        self.actuator.send(float(self.out))

    # The logger, status line and simulator gain editing use these names. The
    # setters matter: without them, retuning ki/kd would silently do nothing.
    @property
    def p_term(self):
        return self.pid.p_term

    @property
    def i_term(self):
        return self.pid.i_term

    @property
    def d_term(self):
        return self.pid.d_term

    @property
    def ki(self):
        return self.pid.ki

    @ki.setter
    def ki(self, v):
        self.pid.ki = v

    @property
    def kd(self):
        return self.pid.kd

    @kd.setter
    def kd(self, v):
        self.pid.kd = v

    def reset_for_recalibration(self):
        self.calib_sq_sum, self.calib_n = 0.0, 0
        self.calib_windows = []
        self.baseline_rms = None
        self.active_gain = 0.0
        self.pid.reset()
        self.clip_excess = 0.0
        self.last_ratio = 0.0
        self.excess_since = None
        self.sat_streak = 0
        self.saturated_flag = False
        self.locked = False
        self.locked_since = None
        self.lock_rms.reset()
        self.runaway_rms.reset()
        self.schedule_rms.reset()
        # The one place a demotion is forgotten wholesale: everything is
        # re-measured, so a stale one would exclude a recovered channel.
        self.healthy = True
        self.demoted_since = None
        self.clear_since = None

    def status_str(self):
        tag = f"ch{self.idx}"
        if not self.enabled:
            return f"{tag}:off  bp={self.bp_out:+.3f}V"
        if not self.healthy:
            # Distinct from ":off": that is the operator's choice, this the rig's.
            held = "" if self.demoted_since is None else f" {self.demoted_since:.0f}s"
            return f"{tag}:DOWN{held}  bp={self.bp_out:+.3f}V"
        if abs(self.active_gain - self.capture_gain) < 0.1 * abs(self.capture_gain - self.steady_gain):
            mode = "CAP"
        elif abs(self.active_gain - self.steady_gain) < 0.1 * abs(self.capture_gain - self.steady_gain):
            mode = "hold"
        else:
            mode = "tr.."
        flags = ("LOCK" if self.locked else "....") + ("!RAIL" if self.rail_fault else "")
        return (f"{tag}:{flags} {mode} g={self.active_gain:+.4f} "
                f"bp={self.bp_out:+.3f}V ratio={self.last_ratio:.2f}")

    def log_fields(self, counts):
        volt = counts * (A_VCC / ADC_MAX_COUNTS)
        return [str(int(counts)), f"{volt:.4f}", f"{self.bp_out:.5f}", f"{self.vel:.5f}",
                f"{self.out:.4f}", f"{self.active_gain:.5f}", f"{self.last_ratio:.4f}",
                f"{self.p_term:.5f}", f"{self.i_term:.5f}", f"{self.d_term:.5f}",
                str(int(self.rail_fault)), str(int(self.locked)),
                # 13th column: axes in the loop at time t. rail_fault cannot
                # say; a demotion outlasts its rail by REARM_SUSTAIN_S.
                str(int(self.healthy))]


class Controller:
    """The fast-lock state machine, decoupled from its sample source. main()
    drives it from serial, sim/server.py from a simulated plant, so the
    simulator tests this code and not a copy. Prints are events, so a
    non-console caller can consume them.
    """

    def __init__(self, dac, dac_channels=None, enable=None, steady=None,
                 capture=None, ki=None, kd=None, bias=None):
        dac_channels = DAC_CHANNELS if dac_channels is None else dac_channels
        enable = ENABLE_CHANNEL if enable is None else enable
        steady = STEADY_GAIN if steady is None else steady
        capture = CAPTURE_GAIN if capture is None else capture
        ki = KI_GAIN if ki is None else ki
        kd = KD_GAIN if kd is None else kd
        bias = BIAS if bias is None else bias

        self.channels = [Channel(i, dac, dac_channels[i], enable[i],
                                 steady[i], capture[i], ki[i], kd[i], bias[i])
                         for i in range(4)]
        self.state = "CALIBRATING"
        self.calib_start = 0.0
        self.damping_start = None
        self.fault_clear_since = None
        self.locked_announced = False
        self.lock_time = None
        self.fault_count = 0
        # Demotions are not faults; counted apart so they do not inflate it.
        self.demote_count = 0
        self.events = []

    def drain_events(self):
        out, self.events = self.events, []
        return out

    def step(self, counts, volts, t, dt):
        """One iteration of the control loop. `counts` and `volts` are length-4."""
        channels = self.channels
        for ch in channels:
            ch.filter_sample(volts[ch.idx], dt)
            ch.check_rail(counts[ch.idx], t)
        any_rail = any(ch.rail_fault for ch in channels)

        if self.state == "CALIBRATING":
            frac_done = min((t - self.calib_start) / CALIBRATION_S, 1.0)
            for ch in channels:
                ch.accumulate_calibration(frac_done)
            if any_rail:
                self.events.append("\n!! sensor railed during calibration -- check alignment "
                                   "before continuing. Entering FAULT.\n")
                self.state = "FAULT"
                self.fault_clear_since = None
                self.fault_count += 1
            elif t - self.calib_start >= CALIBRATION_S:
                for ch in channels:
                    ch.finish_calibration()
                self.state = "DAMPING"
                self.damping_start = t
                self.locked_announced = False
                baselines = ", ".join(f"ch{ch.idx}={ch.baseline_rms:.4f}V" for ch in channels)
                self.events.append(f"[{self.state}] baseline set ({baselines}). "
                                   f"Gain will schedule between {CAPTURE_GAIN[0]:+.3f} (large amplitude) "
                                   f"and {STEADY_GAIN[0]:+.3f} (near lock)...\n")

        elif self.state == "DAMPING":
            # Health moves in this state alone: CALIBRATING faults globally on
            # any rail, and the FAULT path re-calibrates, clearing demotions.
            transitions = [(ch, ch.update_health(t)) for ch in channels]
            n_conf = sum(1 for ch in channels if ch.enabled)
            healthy_n = sum(1 for ch in channels if ch.actuating)
            for ch, ev in transitions:
                if ev == "demoted":
                    self.demote_count += 1
                    # A config-disabled channel is rail-checked too; do not
                    # claim to have parked an output that was never moving.
                    what = (f"holding ch{ch.idx} at bias" if ch.enabled
                            else f"ch{ch.idx} was not driving")
                    self.events.append(
                        f"\n!! ch{ch.idx} sensor railed -- {what}. "
                        f"{healthy_n}/{n_conf} channels still damping.\n")
                elif ev == "rearmed":
                    self.events.append(
                        f"\n[ch{ch.idx} back] rail clear for {REARM_SUSTAIN_S:.0f}s -- "
                        f"re-engaging on its pre-event baseline "
                        f"({ch.baseline_rms:.4f}V), gain ramping from zero. "
                        f"{healthy_n}/{n_conf} damping.\n")

            if n_conf and healthy_n < MIN_HEALTHY_CHANNELS:
                self.events.append(
                    f"\n!! quorum lost -- {healthy_n}/{n_conf} channels healthy, "
                    f"need {MIN_HEALTHY_CHANNELS}. Freezing everything, entering FAULT.\n")
                self.state = "FAULT"
                self.fault_clear_since = None
                self.fault_count += 1
            else:
                tripped_any = False
                for ch in channels:
                    if ch.actuating:
                        ch.update_schedule(t, dt)
                    else:
                        ch.active_gain = 0.0
                    if ch.check_runaway(t):
                        tripped_any = True
                        self.events.append(f"\n!! ch{ch.idx} runaway/saturation -- freezing all "
                                           f"channels, entering FAULT.\n")
                if tripped_any:
                    self.state = "FAULT"
                    self.fault_clear_since = None
                    self.fault_count += 1
                else:
                    for ch in channels:
                        ch.update_lock(t)
                    # A demoted channel must neither lock nor block a lock.
                    enabled_locked = [ch.locked for ch in channels if ch.actuating]
                    if enabled_locked and all(enabled_locked) and not self.locked_announced:
                        self.lock_time = t - self.damping_start
                        degraded = ("" if healthy_n == n_conf else
                                    f" -- DEGRADED, {healthy_n}/{n_conf} channels")
                        self.events.append(f"\n*** LOCKED -- {self.lock_time:.1f}s after gain "
                                           f"was applied (t={t:.1f}s total){degraded} ***\n")
                        self.locked_announced = True

        elif self.state == "FAULT":
            for ch in channels:
                ch.active_gain = 0.0
            # Refreshed by actuate() in every state, so this is live, not latched.
            all_clear = not any_rail and all(not ch.saturated_flag for ch in channels)
            if all_clear:
                if self.fault_clear_since is None:
                    self.fault_clear_since = t
                elif t - self.fault_clear_since >= FAULT_CLEAR_SUSTAIN_S:
                    self.events.append(f"\n[recovered] all channels clear for "
                                       f"{FAULT_CLEAR_SUSTAIN_S:.0f}s -- re-calibrating and resuming.\n")
                    for ch in channels:
                        ch.reset_for_recalibration()
                    self.state = "CALIBRATING"
                    self.calib_start = t
                    self.fault_clear_since = None
                    self.lock_time = None
            else:
                self.fault_clear_since = None

        for ch in channels:
            ch.actuate(dt, self.state)
        return self.state

    def status_line(self, t):
        return (f"[{t:7.1f}s] {self.state:11s} " +
                "  ".join(ch.status_str() for ch in self.channels))


def main():
    dac = DACController(port=PORT)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))

    input("DAC biases set. Press Enter to start fast-lock damping (Ctrl+C to stop)... ")
    dac.start_stream()
    ser = dac.ser

    ctl = Controller(dac)
    channels = ctl.channels

    os.makedirs("data", exist_ok=True)
    csv_path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    csv_file = open(csv_path, "w", buffering=1)
    header = "time_s,state," + ",".join(
        f"ch{i}_counts,ch{i}_V,ch{i}_bp,ch{i}_vel,ch{i}_out,ch{i}_gain,ch{i}_ratio,"
        f"ch{i}_p,ch{i}_i,ch{i}_d,ch{i}_rail,ch{i}_locked,ch{i}_healthy"
        for i in range(4)
    )
    csv_file.write(header + "\n")
    print(f"Logging to {csv_path}")

    start_time = time.time()
    prev_t = time.time()
    last_status = 0.0
    row_count = 0

    print(f"\n[{ctl.state}] measuring baseline noise for {CALIBRATION_S:.0f}s "
          f"(outputs held at bias, no damping yet)...\n")

    try:
        while True:
            sample = read_sample(ser)
            if sample is None:
                continue
            counts, volts = sample
            now = time.time()
            dt = now - prev_t
            prev_t = now
            dt = min(max(dt, 1e-4), 0.05)
            t_rel = now - start_time

            state = ctl.step(counts, volts, t_rel, dt)
            for msg in ctl.drain_events():
                print(msg)

            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                print(ctl.status_line(t_rel))

            row = [f"{t_rel:.4f}", state]
            for ch in channels:
                row += ch.log_fields(counts[ch.idx])
            csv_file.write(",".join(row) + "\n")
            row_count += 1
            if row_count % CSV_FLUSH_EVERY_N == 0:
                csv_file.flush()

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        csv_file.flush()
        csv_file.close()
        dac.stop_stream()
        print("Returning DAC outputs to bias voltages...")
        for c, b in zip(DAC_CHANNELS, BIAS):
            try:
                dac.set_voltage(channel=c, voltage=float(b))
            except RuntimeError:
                pass
        time.sleep(0.1)
        dac.close()
        print(f"Log saved to {csv_path}")


if __name__ == "__main__":
    main()
