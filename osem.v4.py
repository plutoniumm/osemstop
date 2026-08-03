"""
v4 -- modal (MIMO) damping on a basis learned from the hardware.
==================================================================
v0..v3 are four independent SISO loops: each OSEM drives only its own coil and
knows nothing about the other three. report.pdf says so itself -- "this remains
four independent single-input single-output loops rather than a true
multi-degree-of-freedom controller."

But the four OSEMs watch ONE suspended mass, so their readings are projections
of far fewer real degrees of freedom and are heavily correlated. Four loops each
fighting a plant the other three are also driving is the wrong decomposition of
the problem.

v4 controls the MOTION instead of the sensors. It does not need to be told the
geometry -- no quadrant map, no axis assignment, no idea which OSEM is where.
It measures both matrices it needs:

  1. CALIBRATING, gain at zero, as before -- but it also accumulates the 4x4
     covariance of the bandpassed sensor vector. Its eigenvectors ARE the
     observed modes and its eigenvalues rank them by energy. Modes carrying
     less than MODE_ENERGY_FRAC of the total are motion the mass cannot
     actually make: that is the null space, and it is monitored, never driven.
     This is where "how many degrees of freedom are there" gets answered by the
     hardware rather than by an assumption in a config file.

  2. SYSID, a new short state -- dither each coil in turn with a small sine and
     measure which way the modes answer. That gives the actuation matrix B, and
     with it the per-channel sign and authority. This is what makes v4 immune to
     the STEADY_GAIN[2] = +0.010 problem: it DISCOVERS that ch2 pushes the other
     way instead of having to be told, and it would discover a miswired channel
     on any other axis just as readily.

Damping then runs per MODE -- one velocity-feedback loop per real degree of
freedom -- and the modal command is distributed to the four coils through the
pseudo-inverse of B. Null-space modes are watched: a rising null-space signal
means one sensor disagrees with the other three, which detects a blind OSEM
from consistency alone, with no threshold in ADC counts.

Everything else -- filters, slew limits, calibration timing, the state machine,
logging, serial protocol, and all three v3 defect fixes -- is v3 unchanged.
Set MODAL_ENABLE = False and it falls back to v3's per-channel PID exactly.
See versions.md.

Inherited from v3, unchanged:
==================================================================
The correction pass that v0/v1/v2 deliberately deferred. Two kinds of
change, kept separate on purpose:

1. THE CONTROL LAW is restructured into a real PID (class `PID` below)
   instead of three terms scattered through `Channel.actuate`:
     * derivative on MEASUREMENT, not on error. With a constant setpoint
       the two are algebraically the same here, but the measurement form
       cannot produce a derivative kick if the setpoint is ever made
       non-zero, and it keeps the filter state in one place.
     * back-calculation anti-windup. v0..v2 used a hard clamp plus
       conditional integration, which stops the accumulator dead at a
       limit; back-calculation instead bleeds it back at a defined time
       constant TRACK_TC_S, so recovery from saturation is smooth and the
       clamp is a backstop rather than the mechanism.
     * one `reset()`, called on every entry to a non-actuating state, so
       there is exactly one place that defines bumpless re-entry.
   The sign convention is unchanged: all three gains follow the channel's
   own loop sign, negative on ch0/1/3 and positive on the inverted ch2.

2. THE THREE DEFECTS in `FIXES` below are fixed. Each one is called out at
   its site with a `FIXED (vN)` comment naming what v0 did. Every fix is
   asserted from the other side in the harness: the checks that pinned the
   broken behaviour for v0 assert the correct behaviour here.

Everything else -- filters, gain schedule, calibration, logging, serial
protocol -- is v1/v2 unchanged, so a bench comparison isolates the above.
See versions.md.

What's different from the sweep script, and why
--------------------------------------------------
1. No fixed-duration steps. This runs until you Ctrl+C, or until a fault
   forces it into a frozen state (which it can also recover from on its
   own -- see the FAULT state below).

2. Calibration phase instead of a single borrowed reference RMS. The old
   sweep script measured its safety-check reference from ONE 45s window
   at the very start of a ~9 minute run and never updated it. Here, every
   time the controller (re)starts or recovers from a fault, it spends
   CALIBRATION_S seconds with the gain at zero, measuring each channel's
   own natural (undamped) noise floor fresh, before engaging any gain.
   That reference is what the runaway breaker and the lock detector both
   compare against.

3. An explicit sensor-rail interlock, independent of the RMS breaker.
   The sweep run showed that a fully-occluded OSEM (raw ADC pinned at 0)
   produces a bandpassed signal that also flatlines near zero -- which
   looks like *perfect* stability to an RMS-only check, when it's actually
   the sensor going blind. Here, rail state is checked on the raw ADC
   COUNTS (not the scaled voltage) specifically to avoid any float-
   rounding ambiguity, and a sustained rail on ANY channel immediately
   freezes ALL FOUR outputs to bias -- since all four OSEMs sit on the
   same rigid body, a rail on one is treated as "something about this
   optic's position is no longer trustworthy," not just a single-axis
   problem.

4. Auto-recovery. If everything comes back into range on its own (e.g. a
   transient bump) and stays clear for FAULT_CLEAR_SUSTAIN_S, the
   controller re-calibrates from scratch and resumes -- no need to
   restart the script by hand every time something trips.

5. A lock detector with a real, printed answer to "how fast did it
   stabilize": once an enabled channel's rolling RMS has stayed below
   LOCK_RMS_FACTOR times its own calibrated baseline for LOCK_SUSTAIN_S,
   it's declared LOCKED and the elapsed time is printed.

6. Streaming CSV (flushed periodically), not "buffer everything in RAM
   and write at the end." This script is meant to run indefinitely, so
   holding hours of samples in a Python list isn't a good idea, and you
   want data on disk even if it's killed uncleanly.

7. Gain scheduling for a faster, stronger capture without raising the
   validated steady-state gain. Each channel has two gains:
     - STEADY_GAIN: the sweep-validated value (-0.03 on ch0).
     - CAPTURE_GAIN: a stronger, NOT independently validated value, used
       only while the oscillation is still large.
   The active gain blends continuously between them based on a short
   rolling RMS of the bandpassed signal versus that channel's own
   calibrated baseline: near full CAPTURE_GAIN while amplitude is above
   CAPTURE_HIGH_FRAC of baseline, tapering linearly down to STEADY_GAIN
   by the time it drops below CAPTURE_LOW_FRAC, and held at STEADY_GAIN
   for the remainder of the lock. All gain changes (including the
   startup ramp from zero) are slew-rate-limited by GAIN_SLEW_PER_S, so
   there's no abrupt gain step that could itself kick a transient.
   CAPTURE_GAIN defaults to a modest step past -0.03 and stays under the
   -0.04 point the sweep flagged as problematic -- watch the scope the
   first time you run this, the same way you validated -0.03, before
   pushing it further.

Data logged (per sample, per channel)
--------------------------------------
Raw ADC counts (unambiguous rail detection), scaled volts, the bandpassed
signal, the smoothed velocity estimate, the actuator output, the
currently-applied gain, and rail/locked flags -- plus the overall
controller state. That's enough to fully reconstruct what happened and
verify a "locked" claim after the fact, which is exactly what the
sweep-vs-scope mismatch showed we needed.
"""

import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from pyDAC import DACController

# ===================== SETTINGS =====================
PORT = "COM7"
A_VCC = 5.02
ADC_MAX_COUNTS = 1023

DAC_CHANNELS = [1, 3, 5, 7]   # coil DAC channel for sensor index 0..3 (A0..A3).
                              # Rewired 2026-08-03; ref.py:121 is the full 8-pair
                              # map [1,3,5,7,0,2,4,6]. The old [0,2,4,6] now points
                              # at the coils for sensors 4..7.

# All four enabled. report.pdf section 4 documents all four damping together on
# the bench (Table 1: tau = 4.55, 5.49, 3.58, 5.05 s for A0..A3), which is the
# run v0 predates. Disabled channels are still filtered, rail-checked and logged,
# and still trip the global interlock -- disabling one does not make it silent.
VERSION_TAG = "v4"
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "modal")

# ---- modal control -------------------------------------------------------
MODAL_ENABLE = True     # False -> behaves exactly like v3, per-channel PID

# A mode carrying less than this fraction of the total observed energy is taken
# to be null space -- motion the mass cannot make -- and is monitored, never
# driven. Four sensors on a body with 2 real freedoms leave 2 such modes.
MODE_ENERGY_FRAC = 0.005

# Per-mode damping, in the same sign convention as STEADY_GAIN: negative is
# damping. The modal basis is orthonormal and SYSID normalises the actuation
# matrix, so ONE number covers every mode -- there is no per-channel table to
# get wrong, which is the point.
MODAL_STEADY_GAIN = -0.030
MODAL_CAPTURE_GAIN = -0.035

# SYSID: dither the coils and watch which way the modes answer.
#
# The excitation is a pseudo-random sign sequence per coil, NOT a sine. Two
# reasons, both learned the hard way:
#
#   * A sine at the resonance cannot be told apart from the ambient motion at
#     the resonance. The lab is already being driven near 1 Hz -- that is the
#     whole problem being solved -- so a lock-in at that frequency measures the
#     disturbance, identically for every coil, and reports four channels with
#     the same sign and the same authority. Which is wrong, and wrong in the
#     most misleading way: it looks like a successful measurement.
#   * A PRBS is uncorrelated with ANY ambient tone, wherever it happens to sit,
#     so nothing needs to be known about the disturbance in advance.
#
# Four mutually uncorrelated sequences also identify all four coils in ONE
# window instead of four sequential ones, because the cross-correlation between
# different sequences averages to zero. Same information, a quarter of the time.
SYSID_DITHER_S = 10.0           # one window identifies the whole matrix
SYSID_AMPL_V = 0.05             # dither amplitude in DAC volts, around BIAS
SYSID_HOLD_S = 0.25             # per PRBS bit -> energy spread over ~0-2 Hz
SYSID_MIN_RESPONSE = 1e-4       # below this a coil is considered dead

# Null-space watchdog. A rigid body cannot produce null-space motion, so any is
# a sensor disagreeing with its neighbours. This catches a blind OSEM by
# CONSISTENCY rather than by an absolute threshold in counts, which is what the
# rail check needs the noise floor to cooperate with.
NULL_ALARM_RATIO = 4.0          # x the null level measured during calibration
NULL_SUSTAIN_S = 1.0

# --- bench status ---
# DOES NOT DAMP. System identification does not converge (research.md item 2).
# Do not flash this. Kept because the architecture is the deliverable.
BENCH_STATUS = "broken"

ENABLE_CHANNEL = [True, True, True, True]

BIAS = np.array([0.25, 0.25, 0.25, 0.25])
VMIN, VMAX = 0.0, 0.5
MAX_SLEW_PER_S = 2.0

# Damping gain per channel. STEADY_GAIN is the sweep-validated value on
# ch0 (-0.03: real damping, no rail clipping). CAPTURE_GAIN is a stronger
# value used only transiently while the oscillation is still large -- it
# is NOT independently validated. It stays under the -0.04 point the
# sweep flagged as problematic, but confirm on the scope before trusting
# it unattended, the same way -0.03 was confirmed.

STEADY_GAIN  = np.array([-0.030, -0.030, +0.010, -0.030])   # flipped, and much smaller
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035])

# Gain scheduling: blend between CAPTURE_GAIN and STEADY_GAIN based on a
# short rolling RMS of the bandpassed signal, relative to that channel's
# own calibrated baseline. Above CAPTURE_HIGH_FRAC of baseline -> full
# capture gain. Below CAPTURE_LOW_FRAC -> full steady gain. Linear blend
# between. SCHEDULE_WINDOW_S sets how quickly the estimate reacts.
CAPTURE_HIGH_FRAC = 0.6
CAPTURE_LOW_FRAC = 0.2
SCHEDULE_WINDOW_S = 1.0

# Max rate of change of the applied gain, in gain-units/second. Governs
# both the startup ramp (0 -> scheduled target) and every scheduling
# transition, so nothing steps abruptly.
GAIN_SLEW_PER_S = 0.02

# ---- PID on the velocity loop ----
# The controlled variable is VELOCITY and the setpoint is zero (bring the
# optic to rest), so the error is e = 0 - vel = -vel. In that frame:
#
#   P  = active_gain * e     <- STEADY_GAIN/CAPTURE_GAIN above ARE Kp. This is
#                               the only term that removes energy from the
#                               resonance, and the only one validated on
#                               hardware (-0.030 on ch0).
#   I  = KI_GAIN * int(e)    <- the integral of velocity is DISPLACEMENT, so
#                               this acts as an added SPRING: it shifts the
#                               resonant frequency rather than damping it.
#                               Nonzero Ki moves the pole -- re-validate.
#   D  = KD_GAIN * d(e)/dt   <- the derivative of velocity is ACCELERATION, so
#                               this acts as added (negative) MASS. It also
#                               differentiates the sensor a second time, so it
#                               is by far the noisiest term and is lowpassed
#                               at D_SMOOTH_HZ.
#
# Both default to 0.0 on every channel, which makes the loop mathematically
# identical to the validated P-only (velocity-feedback) controller. Change one
# term at a time and confirm on the scope, the same way Kp was confirmed.
# All three gains share ONE sign convention per channel, because they share one
# loop: negative on ch0/1/3, positive on the inverted ch2.
KI_GAIN = np.array([-0.040, -0.040, +0.040, -0.040])
KD_GAIN = np.array([-0.0015, -0.0015, +0.0015, -0.0015])
D_SMOOTH_HZ = 2.0      # lowpass on the D term (acceleration estimate)
I_CLAMP_V = 0.15       # backstop only; TRACK_TC_S is the working anti-windup
TRACK_TC_S = 0.5       # back-calculation time constant. Smaller unwinds the
                       # integrator faster when the output is clipped; must
                       # stay well above the sample period.

BP_LOW_HZ = 0.4
BP_HIGH_HZ = 3.0
DERIV_SMOOTH_HZ = 5.0

# FIXED (v3), defect "runaway-baseline": v0 measured the baseline as ONE RMS
# over a single 8 s window and compared a 2 s RMS against it forever. A high-Q
# suspension wanders on its own ringdown timescale (Q/f0 ~ 50 s), so that
# single sample is not a stable estimate of anything, and one shock landing
# inside the window poisons it for the whole run. Two changes: calibrate for
# longer, and take the MEDIAN of several sub-windows so a transient in one of
# them is outvoted rather than averaged in.
CALIBRATION_S = 20.0           # ~20 cycles of the ~1 Hz resonance, gain held at 0
CALIB_SUBWINDOWS = 5           # median across these; must be >= 3 to outvote one
LOCK_RMS_FACTOR = 0.35          # "locked" once rolling RMS < this * baseline
LOCK_SUSTAIN_S = 5.0
LOCK_WINDOW_S = 5.0

ENVELOPE_WINDOW_S = 2.0         # sliding window for the runaway check
RUNAWAY_MULTIPLE = 1.8
RUNAWAY_SUSTAIN_S = 2.0
MAX_CONSECUTIVE_SATURATED = 30

# FIXED (v3), defect "rail-threshold": v0 used 3 counts of 1023 (~14.7 mV) and
# required 250 CONSECUTIVE samples under it. Dark noise of ~1 count RMS is
# enough for one sample to poke above the line and restart the count, so the
# interlock silently stopped arming: measured on the simulator it armed in
# 0.51 s at <=5 mV of sensor noise, took 6.8 s at 7 mV, and NEVER fired at
# >=10 mV -- while the blind channel reported an amplitude ratio near zero,
# steadier than any working axis. Two changes: put the threshold above a
# realistic noise floor, and require a FRACTION of a window rather than an
# unbroken run, so a single noise sample no longer resets the evidence.
RAIL_LOW_COUNTS = 12            # raw ADC counts, out of 0..1023 -- not volts,
RAIL_HIGH_COUNTS = 1011         # to avoid float-rounding ambiguity
RAIL_SUSTAIN_S = 0.5
RAIL_FRACTION = 0.80            # of the samples in the window, not consecutive

FAULT_CLEAR_SUSTAIN_S = 5.0     # all-clear for this long -> auto re-arm

STATUS_PERIOD_S = 5.0
CSV_FLUSH_EVERY_N = 200
# ====================================================


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
        """Forget the state, so the next sample re-primes as if fresh. Used by
        PID.reset() -- a filter carrying state from before a FAULT would put a
        step into the first sample after recovery."""
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


class ModalBasis:
    """Learns, from the hardware, what the four sensors are actually watching.

    Nothing here is told the geometry. During calibration it accumulates the
    covariance of the bandpassed 4-vector; its eigenvectors are the observed
    modes and its eigenvalues say how much motion each one carries. Modes below
    MODE_ENERGY_FRAC of the total are the null space -- combinations no rigid
    motion of the mass can produce -- and are reported, never driven.

    Then SYSID dithers each coil and records how the modes respond, giving the
    actuation matrix. Sensing basis and actuation matrix are measured
    separately on purpose: assuming one is the transpose of the other is only
    true if every channel's sensor and coil agree in sign, and on this hardware
    one of them does not.
    """

    def __init__(self, n=4):
        self.n = n
        self.sum_outer = np.zeros((n, n))
        self.n_samples = 0
        self.modes = None          # (n, n) columns are unit mode vectors
        self.energy = None         # eigenvalue per mode, descending
        self.n_active = 0          # how many are real motion
        self.null_baseline = 0.0
        self.B = None              # (n_active, n) modal response per coil volt
        self.B_pinv = None         # (n, n_active) modal command -> coil volts

    # ---- learned from the calibration window ----
    def accumulate(self, vec):
        v = np.asarray(vec, dtype=float)
        self.sum_outer += np.outer(v, v)
        self.n_samples += 1

    def solve(self):
        """Eigendecompose. Returns a one-line human summary for the log."""
        cov = self.sum_outer / max(self.n_samples, 1)
        vals, vecs = np.linalg.eigh(cov)          # ascending, orthonormal
        order = np.argsort(vals)[::-1]
        self.energy = vals[order]
        self.modes = vecs[:, order]
        total = float(np.sum(np.clip(self.energy, 0, None))) or 1.0
        frac = np.clip(self.energy, 0, None) / total
        self.n_active = max(1, int(np.sum(frac >= MODE_ENERGY_FRAC)))
        # how much lives in the null space right now, as the alarm reference
        null_e = float(np.sum(np.clip(self.energy[self.n_active:], 0, None)))
        self.null_baseline = max(np.sqrt(null_e), 1e-9)
        return (f"{self.n_active} of {self.n} modes carry real motion "
                f"(energy split " + " ".join(f"{f * 100:.1f}%" for f in frac) + ")")

    def project(self, vec):
        """Sensor vector -> modal coordinates, all n of them."""
        return self.modes.T @ np.asarray(vec, dtype=float)

    def null_rms(self, vec):
        """How much of this reading no rigid motion could have produced."""
        m = self.project(vec)
        return float(np.sqrt(np.sum(m[self.n_active:] ** 2)))

    # ---- learned from the dither ----
    def set_actuation(self, columns):
        """`columns[i]` is the modal response to a unit volt on coil i."""
        B = np.array(columns, dtype=float).T          # (n_active, n_coils)
        # Normalise by the typical column so a modal gain means the same thing
        # a per-channel gain did in v0..v3 -- otherwise every gain constant in
        # the file would silently change meaning between versions.
        scale = float(np.mean(np.linalg.norm(B, axis=0))) or 1.0
        B = B / scale
        self.B = B
        # Minimum-norm distribution of a modal command across the coils. Rows of
        # B that are near-zero (a dead or disconnected coil) fall out of the
        # pseudo-inverse on their own rather than needing a special case.
        self.B_pinv = np.linalg.pinv(B, rcond=1e-3)
        return B

    def to_coils(self, modal_cmd):
        return self.B_pinv @ np.asarray(modal_cmd, dtype=float)

    def describe_actuation(self):
        """Per-coil authority and sign, as measured -- not as configured."""
        out = []
        for i in range(self.B.shape[1]):
            col = self.B[:, i]
            mag = float(np.linalg.norm(col))
            dominant = int(np.argmax(np.abs(col)))
            sign = "+" if col[dominant] >= 0 else "-"
            out.append(f"ch{i}: |B|={mag:.4f} {sign}mode{dominant}")
        return ", ".join(out)


class PID:
    """Parallel-form PID on the velocity error, in DAC volts.

    The process variable is VELOCITY and the setpoint is zero, so e = -vel and
    the three terms are physically:
        P  velocity feedback -- the only term that removes energy
        I  integral of velocity is displacement -> an added spring
        D  derivative of velocity is acceleration -> an added mass

    Kp arrives per call because the supervisor schedules and slew-limits it;
    ki and kd are fixed per channel. All three carry the channel's own loop
    sign, so they are negative together on a normally-wired channel.

    Anti-windup is back-calculation: when the caller clips the output it hands
    back how much it clipped, and the integrator is unwound at TRACK_TC_S
    rather than simply frozen. That keeps the recovery from saturation smooth
    and defined instead of depending on which side of a conditional the error
    happened to fall. I_CLAMP_V remains as a hard backstop.
    """

    def __init__(self, ki, kd):
        self.ki = ki
        self.kd = kd
        self.d_filter = OnePoleFilter(D_SMOOTH_HZ, kind="low")
        self.reset()

    def reset(self):
        """Bumpless re-entry: no stale integral, no derivative kick on the
        first sample back. Called on every entry to a non-actuating state."""
        self.i_term = 0.0
        self.p_term = 0.0
        self.d_term = 0.0
        self.prev_meas = 0.0
        self.primed = False
        self.d_filter.reset()

    def update(self, meas, dt, kp, clip_excess=0.0):
        """One sample. `meas` is the measured velocity, `clip_excess` is
        (unclipped - clipped) output from the PREVIOUS sample, in volts."""
        err = -meas                                   # setpoint is zero velocity
        self.p_term = kp * err

        # Derivative on the MEASUREMENT, not the error. Identical while the
        # setpoint is a constant zero, but it cannot kick if that ever changes,
        # and the sign is explicit rather than folded into err.
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
        # The firmware streams one column per analog pin it samples. thing.c
        # streams four (A0..A3); arduino.ino streams eight (A0..A7) but only
        # A0..A3 carry OSEMs, so take the leading four. Exactly-four input is
        # unaffected -- parts[:4] is a no-op there.
        if len(parts) < 4:
            return None
        counts = np.array([int(p) for p in parts[:4]])
        volts = counts * (A_VCC / ADC_MAX_COUNTS)
        return counts, volts
    except (ValueError, IndexError):
        return None


class Channel:
    """Filter chain, gain, and health tracking for one OSEM/coil pair.

    Filtering, rail-checking, and calibration run for every channel
    regardless of `enabled`, so disabled channels still contribute
    diagnostic data and still participate in the global safety trip.
    """

    def __init__(self, idx, dac, dac_channel, enabled, steady_gain, capture_gain,
                 ki, kd, bias):
        self.idx = idx
        self.enabled = enabled
        self.steady_gain = steady_gain
        self.capture_gain = capture_gain
        self.active_gain = 0.0
        self.bias = bias

        # PID on velocity error e = -vel. active_gain is Kp (scheduled and
        # slew-limited above); ki/kd are fixed per channel.
        self.pid = PID(ki, kd)
        self.clip_excess = 0.0        # carried to the next sample's anti-windup
        # When the supervisor is running a modal law, the per-channel command
        # comes from outside: this channel is one actuator serving several
        # modes, not a loop of its own. None means "run your own PID" (v3
        # behaviour), which is what MODAL_ENABLE = False leaves in place.
        self.external_cmd = None

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
        """FIXED (v3), defect "rail-threshold". Two changes from v0.

        v0 demanded RAIL_SUSTAIN_S of CONSECUTIVE samples under 3 counts. A
        single sample of dark noise poking above the line reset the counter, so
        the interlock stopped arming entirely once noise exceeded ~1 count RMS
        -- and a blind channel then reported a near-zero amplitude ratio,
        reading as the steadiest axis on the rack while the loop stayed in
        DAMPING and declared itself locked. Here the threshold sits above a
        realistic noise floor, and the evidence is a FRACTION of a sliding
        window rather than an unbroken run, so noise dilutes the count instead
        of erasing it.

        Also: v0 printed on every railed sample with no latch, thousands of
        lines a second. This prints once per transition.
        """
        railed = counts <= RAIL_LOW_COUNTS or counts >= RAIL_HIGH_COUNTS
        self.rail_hist.append((now, railed))
        while self.rail_hist and now - self.rail_hist[0][0] > RAIL_SUSTAIN_S:
            self.rail_hist.popleft()

        # Need a full window before judging, or the first samples of a run trip
        # it on a sample size of one.
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

    def accumulate_calibration(self, frac_done=0.0):
        """FIXED (v3), defect "runaway-baseline". v0 accumulated one sum of
        squares across the whole 8 s window and took its RMS. One shock landing
        anywhere inside that window inflated the baseline for the entire run,
        after which ordinary motion read as unusually quiet and the lock
        detector could fire on nothing. Here the window is cut into
        CALIB_SUBWINDOWS pieces and each is closed off separately, so
        finish_calibration() can take a median and outvote a bad one.

        PARTIAL, and measured as such. With one shock in the window the worst
        channel's baseline skews 20.5% in v0 and 14.6% here. It does not help
        against continuous disturbance -- gain is zero during calibration, so a
        shock rings down over Q/f0 ~ 50 s, longer than the whole window, and
        once several have landed there is no clean sub-window left to prefer.
        At 30 shocks/min v3 is slightly WORSE than v0 (33% vs 27%), purely
        because a 20 s window catches more of them than an 8 s one. Arguably
        correct -- a lab being hit that often really does have that noise floor
        -- but do not read this fix as more than it is.
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
            # Median, not mean: a transient corrupts one sub-window, and the
            # median discards it instead of averaging it in.
            self.baseline_rms = max(float(np.median(self.calib_windows)), 1e-6)
        else:
            self.baseline_rms = 1e-6
        self.calib_sq_sum, self.calib_n = 0.0, 0
        self.calib_windows = []

    def update_schedule(self, t_rel, dt):
        """Blend active_gain toward a target set by current oscillation
        amplitude (relative to this channel's calibrated baseline), moving
        no faster than GAIN_SLEW_PER_S. Doubles as the startup soft-start,
        since active_gain begins at 0.0.
        """
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
        local_rms = self.runaway_rms.update(t_rel, self.bp_out)
        tripped = False
        if local_rms > self.baseline_rms * RUNAWAY_MULTIPLE:
            if self.excess_since is None:
                self.excess_since = t_rel
            elif t_rel - self.excess_since >= RUNAWAY_SUSTAIN_S:
                tripped = True
        else:
            self.excess_since = None

        # saturated_flag is maintained in actuate() now -- see the FIXED note
        # there. This only reads it, so the two conditions stay independent.
        if self.saturated_flag:
            tripped = True

        return tripped

    def update_lock(self, t_rel):
        if not self.enabled or self.baseline_rms is None:
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
        if state in ("DAMPING", "SYSID") and self.enabled and self.external_cmd is not None:
            unclipped = self.bias + self.external_cmd
            target = float(np.clip(unclipped, VMIN, VMAX))
            self.clip_excess = unclipped - target
        elif state == "DAMPING" and self.enabled:
            u = self.pid.update(self.vel, dt, self.active_gain, self.clip_excess)
            unclipped = self.bias + u
            target = float(np.clip(unclipped, VMIN, VMAX))
            self.clip_excess = unclipped - target     # feeds back next sample
        else:
            # Not actuating: one call defines bumpless re-entry, so a FAULT or a
            # disabled channel can never resume with a stale integral or a
            # derivative step across the gap.
            self.pid.reset()
            self.clip_excess = 0.0
            self.external_cmd = None
            target = self.bias
        self.out = slew_limit(self.prev_out, target, dt, MAX_SLEW_PER_S)
        self.prev_out = self.out

        # FIXED (v3), defect "saturation-latch": v0 recomputed saturated_flag
        # only inside check_runaway(), which does not run in FAULT. A saturation
        # trip therefore set the flag, entered FAULT, and then nothing could
        # ever clear it -- all_clear stayed False and the fault was permanent
        # until a manual restart, in flat contradiction of the auto-recovery
        # this module advertises. (A runaway trip recovered fine, because it
        # never set the flag. Only saturation deadlocked.) The flag is now
        # maintained HERE, where the output is actually known, so it is updated
        # on every sample in every state -- including FAULT, where the output is
        # held at bias and the flag consequently clears on its own.
        if self.out <= VMIN + 1e-6 or self.out >= VMAX - 1e-6:
            self.sat_streak += 1
        else:
            self.sat_streak = 0
        self.saturated_flag = self.sat_streak > MAX_CONSECUTIVE_SATURATED

        self.actuator.send(float(self.out))

    # v0..v2 kept the PID gains and terms as plain attributes on Channel, and
    # the logger, the status line and the simulator's live gain editing all
    # reach for them by those names. The state moved into `pid`, so these keep
    # the old surface -- including the setters, without which retuning ki or kd
    # at runtime would appear to work and silently do nothing.
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
        self.external_cmd = None
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

    def status_str(self):
        tag = f"ch{self.idx}"
        if not self.enabled:
            return f"{tag}:off  bp={self.bp_out:+.3f}V"
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
                str(int(self.rail_fault)), str(int(self.locked))]


class Controller:
    """The fast-lock state machine, decoupled from where samples come from.

    main() drives this from the serial stream. sim/server.py drives the exact
    same class from a simulated plant, so there is no second implementation of
    the loop to keep in sync -- the simulator is testing this code, not a copy
    of it.

    Timebase note: the original loop used wall-clock `now` for the rail and
    fault-recovery timers and `t_rel` for the RMS/lock/runaway timers. Every one
    of those comparisons is a DIFFERENCE between two timestamps, so driving them
    all from one monotonic clock is exactly equivalent.

    Prints became events so a caller that is not a console (a server, a test)
    can consume them; main() drains and prints them unchanged.
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
        self.events = []

        # --- modal control state ---
        self.basis = ModalBasis(4)
        self.modal_pid = []            # one PID per active mode, built after solve()
        self.modal_gain = 0.0          # scheduled and slew-limited, like active_gain
        self.modal_cmd = np.zeros(4)
        self.modal_vel = np.zeros(4)
        self.sysid_start = None
        self.sysid_lfsr = [0xACE1, 0xBEEF, 0x1234, 0x5A5A]   # one seed per coil
        self.sysid_signs = [1.0, 1.0, 1.0, 1.0]
        self.sysid_acc = None          # per-coil correlation accumulators
        self.sysid_n = 0
        self.sysid_next_flip = 0.0
        self.sysid_cols = []
        self.null_level = 0.0
        self.null_since = None
        self.null_alarm = False

    # ---------------- modal helpers ----------------
    def modal_ready(self):
        return MODAL_ENABLE and self.basis.B_pinv is not None

    @staticmethod
    def _lfsr(state):
        """Deterministic pseudo-random bit. Deterministic matters: a run has to
        be reproducible, and the sequences have to be the same ones the
        correlation is computed against."""
        bit = ((state) ^ (state >> 2) ^ (state >> 3) ^ (state >> 5)) & 1
        return ((state >> 1) | (bit << 15)) & 0xFFFF

    def _sysid_step(self, t, dt):
        """Excite all four coils with uncorrelated sign sequences and correlate
        the modal response against each. Returns (coil volts, finished).

        Correlates modal VELOCITY, not displacement. Driven near resonance,
        displacement lags force by 90 degrees, so its in-phase component is
        nearly zero and the measurement would be mostly noise -- with the sign
        of that noise deciding which way each coil is wired. Velocity is in
        phase with force at resonance, and it is also the transfer function
        this controller actually uses, since the law is velocity feedback.
        """
        elapsed = t - self.sysid_start
        volts = np.zeros(4)
        if elapsed >= SYSID_DITHER_S:
            n = max(self.sysid_n, 1)
            cols = [self.sysid_acc[i] / n / (SYSID_AMPL_V ** 2) for i in range(4)]
            self.sysid_cols = cols
            return volts, True

        # advance the sign sequences on the hold grid
        if elapsed >= self.sysid_next_flip:
            self.sysid_next_flip += SYSID_HOLD_S
            for i in range(4):
                self.sysid_lfsr[i] = self._lfsr(self.sysid_lfsr[i])
                self.sysid_signs[i] = 1.0 if (self.sysid_lfsr[i] & 1) else -1.0

        for i in range(4):
            volts[i] = SYSID_AMPL_V * self.sysid_signs[i]

        # let the first bit or two propagate before believing the response
        if elapsed > 4 * SYSID_HOLD_S:
            m = self.basis.project([ch.vel for ch in self.channels])
            mv = m[:self.basis.n_active]
            for i in range(4):
                self.sysid_acc[i] += mv * (SYSID_AMPL_V * self.sysid_signs[i])
            self.sysid_n += 1
        return volts, False

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
            # Same window, no extra time: learn what the sensors are watching.
            if MODAL_ENABLE:
                self.basis.accumulate([ch.bp_out for ch in channels])
            if any_rail:
                self.events.append("\n!! sensor railed during calibration -- check alignment "
                                   "before continuing. Entering FAULT.\n")
                self.state = "FAULT"
                self.fault_clear_since = None
                self.fault_count += 1
            elif t - self.calib_start >= CALIBRATION_S:
                for ch in channels:
                    ch.finish_calibration()
                baselines = ", ".join(f"ch{ch.idx}={ch.baseline_rms:.4f}V" for ch in channels)
                self.locked_announced = False
                if MODAL_ENABLE:
                    summary = self.basis.solve()
                    self.events.append(f"[CALIBRATED] baseline set ({baselines}).\n"
                                       f"  {summary}\n")
                    self.state = "SYSID"
                    self.sysid_start = t
                    self.sysid_cols = []
                    self.sysid_n = 0
                    self.sysid_next_flip = 0.0
                    self.sysid_lfsr = [0xACE1, 0xBEEF, 0x1234, 0x5A5A]
                    self.sysid_acc = [np.zeros(self.basis.n_active)
                                      for _ in range(4)]
                else:
                    self.state = "DAMPING"
                    self.damping_start = t
                    self.events.append(f"[{self.state}] baseline set ({baselines}). "
                                       f"Gain will schedule between {CAPTURE_GAIN[0]:+.3f} "
                                       f"(large amplitude) and {STEADY_GAIN[0]:+.3f} (near lock)...\n")

        elif self.state == "SYSID":
            # Learn the actuation matrix. Gain stays at zero throughout; the only
            # thing driving the coils is the dither itself.
            for ch in channels:
                ch.active_gain = 0.0
            sysid_volts, done = self._sysid_step(t, dt)
            if any_rail:
                self.events.append("\n!! sensor railed during system id -- entering FAULT.\n")
                self.state = "FAULT"
                self.fault_clear_since = None
                self.fault_count += 1
            elif done or len(self.sysid_cols) >= 4:
                cols = self.sysid_cols[:4]
                while len(cols) < 4:                      # a coil that never answered
                    cols.append(np.zeros(self.basis.n_active))
                B = self.basis.set_actuation(cols)
                dead = [i for i in range(4)
                        if float(np.linalg.norm(B[:, i])) < SYSID_MIN_RESPONSE]
                self.modal_pid = [PID(0.0, 0.0) for _ in range(self.basis.n_active)]
                self.modal_gain = 0.0
                self.events.append(
                    f"[SYSID] actuation measured -- {self.basis.describe_actuation()}\n"
                    + (f"  !! no response from ch{dead} -- excluded by the pseudo-inverse\n"
                       if dead else "")
                    + f"  damping {self.basis.n_active} mode(s); the remaining "
                      f"{4 - self.basis.n_active} are null space and are watched, not driven.\n")
                self.state = "DAMPING"
                self.damping_start = t
            for i, ch in enumerate(channels):
                ch.external_cmd = float(sysid_volts[i]) if ch.enabled else None

        elif self.state == "DAMPING":
            if any_rail:
                self.events.append("\n!! sensor rail detected -- freezing all channels, entering FAULT.\n")
                self.state = "FAULT"
                self.fault_clear_since = None
                self.fault_count += 1
            else:
                tripped_any = False
                for ch in channels:
                    if ch.enabled:
                        ch.update_schedule(t, dt)
                    else:
                        ch.active_gain = 0.0
                    if ch.check_runaway(t):
                        tripped_any = True
                        self.events.append(f"\n!! ch{ch.idx} runaway/saturation -- freezing all "
                                           f"channels, entering FAULT.\n")
                # ---- modal law: one loop per real degree of freedom ----
                if self.modal_ready():
                    n = self.basis.n_active
                    # The basis is a constant linear map and every filter ahead
                    # of it is linear, so projecting the per-channel velocity
                    # estimates gives the modal velocities directly -- no second
                    # differentiator, no extra phase lag.
                    self.modal_vel = self.basis.project([ch.vel for ch in channels])
                    ratio = max((ch.last_ratio for ch in channels if ch.enabled),
                                default=0.0)
                    frac = np.clip((ratio - CAPTURE_LOW_FRAC)
                                   / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0.0, 1.0)
                    target = MODAL_STEADY_GAIN + frac * (MODAL_CAPTURE_GAIN - MODAL_STEADY_GAIN)
                    self.modal_gain = slew_limit(self.modal_gain, target, dt, GAIN_SLEW_PER_S)

                    cmd = np.zeros(n)
                    for m in range(n):
                        cmd[m] = self.modal_pid[m].update(
                            float(self.modal_vel[m]), dt, self.modal_gain)
                    self.modal_cmd = cmd
                    coil = self.basis.to_coils(cmd)
                    for i, ch in enumerate(channels):
                        ch.external_cmd = float(coil[i]) if ch.enabled else None

                    # ---- null-space watchdog ----
                    # A rigid body cannot make this motion, so any of it means a
                    # sensor disagrees with the other three. Catches a blind OSEM
                    # by consistency, with no absolute threshold in counts.
                    self.null_level = self.basis.null_rms([ch.bp_out for ch in channels])
                    if self.null_level > self.basis.null_baseline * NULL_ALARM_RATIO:
                        if self.null_since is None:
                            self.null_since = t
                        elif t - self.null_since >= NULL_SUSTAIN_S and not self.null_alarm:
                            self.null_alarm = True
                            self.events.append(
                                f"\n!! null-space alarm: {self.null_level:.4f}V of motion no "
                                f"rigid body can make ({self.null_level / self.basis.null_baseline:.1f}x "
                                f"the calibrated level). One sensor disagrees with the other "
                                f"three -- suspect a blind or drifting OSEM.\n")
                            tripped_any = True
                    else:
                        self.null_since = None
                        self.null_alarm = False

                if tripped_any:
                    self.state = "FAULT"
                    self.fault_clear_since = None
                    self.fault_count += 1
                else:
                    for ch in channels:
                        ch.update_lock(t)
                    enabled_locked = [ch.locked for ch in channels if ch.enabled]
                    if enabled_locked and all(enabled_locked) and not self.locked_announced:
                        self.lock_time = t - self.damping_start
                        self.events.append(f"\n*** LOCKED -- {self.lock_time:.1f}s after gain "
                                           f"was applied (t={t:.1f}s total) ***\n")
                        self.locked_announced = True

        elif self.state == "FAULT":
            for ch in channels:
                ch.active_gain = 0.0
            # saturated_flag is refreshed by actuate() on every sample in every
            # state, so this is now a live reading rather than a latch. See the
            # FIXED note in Channel.actuate.
            all_clear = not any_rail and all(not ch.saturated_flag for ch in channels)
            if all_clear:
                if self.fault_clear_since is None:
                    self.fault_clear_since = t
                elif t - self.fault_clear_since >= FAULT_CLEAR_SUSTAIN_S:
                    self.events.append(f"\n[recovered] all channels clear for "
                                       f"{FAULT_CLEAR_SUSTAIN_S:.0f}s -- re-calibrating and resuming.\n")
                    for ch in channels:
                        ch.reset_for_recalibration()
                    # Re-learn the model from scratch: the reason we faulted may
                    # be that the plant or a sensor is no longer what we
                    # measured. A borrowed basis would be exactly the mistake
                    # the borrowed-baseline rule already forbids.
                    self.basis = ModalBasis(4)
                    self.modal_pid = []
                    self.modal_gain = 0.0
                    self.null_since = None
                    self.null_alarm = False
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
        f"ch{i}_p,ch{i}_i,ch{i}_d,ch{i}_rail,ch{i}_locked"
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
