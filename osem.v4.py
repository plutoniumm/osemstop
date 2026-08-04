"""
v4 -- graceful degradation: a blind axis drops out, the rest keep damping.
==========================================================================
v3 with ONE behavioural change, and nothing else touched. Everything below
this section is v3 byte for byte, so a bench comparison isolates it.

THE CHANGE. v3 treats a railed sensor as a whole-optic emergency:

    any_rail = any(ch.rail_fault for ch in channels)   # -> FAULT, all four to bias

One OSEM losing its flag stops ALL damping -- at exactly the moment of
largest excursion, which is when the flag left range in the first place.
During a seismic event that is backwards: the loop gives up precisely when
the optic most needs it.

v4 demotes the blind channel instead of the whole rig:

    rail on ch_i  ->  ch_i alone is held at bias; ch_j != i keep damping
    global FAULT  ->  only when ZERO configured channels are still healthy

That quorum of one is not a guess. provenance.md section 3 measured a single
OSEM/coil pair damping the WHOLE rigid body (~70 s, against ~17 s for all
four in section 4), and v0..v4 run four INDEPENDENT SISO loops -- OSEM i
drives coil i and reads nothing else -- so ch0 does not need ch2's sensor to
compute ch0's output. Three healthy axes are a slower rig, not a broken one.

WHAT IS DEMOTED, AND WHAT IS STILL GLOBAL. Only the rail interlock became
per-channel. This is deliberate, and the distinction is the whole design:

  * RAIL is a local SENSING failure. The measurement is garbage, so the
    output computed from it is garbage. Actuating on it is worse than not
    actuating. Demote.
  * AMPLITUDE RUNAWAY is not localisable. Four loops act on one rigid body,
    so "amplitude is growing under feedback" does not identify which loop is
    putting the energy in. Still global FAULT, unchanged from v3.
  * ACTUATOR SATURATION is deliberately NOT demoted, even though it is the
    other obvious candidate. A saturated channel is still pushing the right
    way with full authority -- its measurement is fine and only its output is
    clipped, which v3's back-calculation anti-windup already handles. Pulling
    it to bias would REMOVE damping authority at peak amplitude, the same
    mistake this version exists to fix. Saturation keeps v3's behaviour
    exactly (check_runaway -> global FAULT). Unchanged, on purpose.

CALIBRATING IS ALSO UNCHANGED: any rail there is still a global FAULT. You
cannot calibrate a blind sensor, and a rig that starts up blind is not ready.
That also means a demoted channel ALWAYS has a valid baseline, because it was
healthy for the whole calibration window -- which is what makes re-arming
cheap.

RE-ARMING. A demoted channel comes back when its rail has been clear
continuously for REARM_SUSTAIN_S. Three things happen on the way back, and
each is there for a measured reason:

  1. The filters are NOT reset. versions.md hazard 2: re-seeding a one-pole
     injects a step. Instead the hold does the work -- BP_LOW_HZ = 0.4 Hz is
     tau = 0.398 s, so REARM_SUSTAIN_S = 2.0 s is ~5 tau of wash-out AFTER the
     sensor comes back, which is why the timer starts on rail-clear and not
     on the demotion.
  2. The baseline is CARRIED FORWARD, not re-measured. This is a deliberate
     exception to the "always measure your own baseline, never borrow one"
     rule the rest of the ladder follows, and it is the one place v4 breaks
     it. Re-measuring means measuring during the event that caused the
     demotion, with this channel's gain at zero while the others damp: an
     inflated noise floor, which then desensitises this channel's runaway
     breaker (RUNAWAY_MULTIPLE is relative to baseline) for the rest of the
     run. A pre-event baseline measured in a quiet lab is the better estimate
     of a quiet lab. If the event was permanent rather than transient, the
     right recovery is a global re-calibration, which the FAULT path already
     does.
  3. active_gain is zeroed on demotion, so GAIN_SLEW_PER_S ramps it back from
     zero (1.75 s to reach capture gain) rather than stepping. The runaway and
     schedule RMS windows are cleared too, so near-zero bandpass samples from
     the blind period cannot sit inside a 2 s window and mask a real runaway.

KNOWN LIMIT, stated because it is easy to over-read this version: it fixes
the FULLY blind case, not partial clipping. The bench measured 2.19% of
samples at a rail on v1; RAIL_FRACTION = 0.80 of a window will not fire on
that, so a partially-clipping channel still reads as healthy and still feeds
flat-topped bandpass and spiking velocity into a live loop. Centring the OSEM
rest points (622/632/598 counts against a mid-scale of 511.5) is the fix for
that, and it is a separate problem. See README.md, "Known open problem".

This is also the `auto-disable` block drawn in the original report's
safety-checks diagram and never implemented in any version -- see
provenance.md, "What the report does NOT contain".

--------------------------------------------------------------------------
Inherited from v3, unchanged
--------------------------------------------------------------------------
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

The three `FIXED (v3)` comments below are left naming v3, because v3 is where
those fixes were made and where the harness first asserted them. The one
change v4 makes is marked `NEW (v4)`.

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

v4 adds a 13th per-channel column, `healthy`, so a log can answer how many
axes were in the loop at any instant. `rail` alone cannot: a demotion
outlasts the rail that caused it by REARM_SUSTAIN_S.
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
PORT = "COM7"
A_VCC = 5.02
ADC_MAX_COUNTS = 1023

DAC_CHANNELS = [1, 3, 5, 7]   # coil DAC channel for sensor index 0..3 (A0..A3).
                              # Rewired 2026-08-03. Full 8-pair map is
                              # [1,3,5,7,0,2,4,6] -- see provenance.md. The old
                              # [0,2,4,6] now points at the coils for sensors 4..7.

# All four enabled. provenance.md §4 documents all four damping together on
# the bench (Table 1: tau = 4.55, 5.49, 3.58, 5.05 s for A0..A3), which is the
# run v0 predates. Disabled channels are still filtered, rail-checked and logged,
# and still trip the global interlock -- disabling one does not make it silent.
VERSION_TAG = "v4"
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable")

# --- bench status ---
# Ran on hardware 2026-08-04. ch0 alone locked in 14.7 s; all four locked in
# 11.1 s and rode three kicks, faulting and RECOVERING every time -- which is
# the saturation-latch fix confirmed under a real disturbance, the thing v2
# latched on. The rewired coil map [1,3,5,7] was validated on the same run, and
# ch2's +0.010 came out the best-damping channel of the four, as Table 1 said.
#
# Still unconfirmed: auto-disable itself. It triggers on a RAILED sensor, and a
# kick produces runaway/saturation, which is deliberately still global -- so no
# bench run has demoted a channel yet. Verified against the simulator only,
# which models occlusion directly. See versions.md section "On the bench".
BENCH_STATUS = "validated"

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

# NEW (v4), "auto-disable". How long a demoted channel's rail must stay clear
# before it re-arms, and how many channels must survive before the whole rig
# faults.
#
# REARM_SUSTAIN_S is set from the filter time constants, not picked. The
# slowest pole in the chain is the BP_LOW_HZ = 0.4 Hz highpass, tau =
# 1/(2*pi*0.4) = 0.398 s. A sensor coming back off a rail is a step into that
# filter, so the hold has to outlast the transient: 2.0 s is 5.0 tau. The timer
# starts when the RAIL CLEARS, not when the channel was demoted, so the whole
# wash-out happens with the sensor already reading properly. Do not shorten it
# below ~1.2 s (3 tau) without re-checking that re-entry is still bumpless.
REARM_SUSTAIN_S = 2.0
# The quorum. provenance.md section 3: one OSEM/coil pair damps the whole rigid
# body on its own (~70 s, against ~17 s for four), so one healthy channel is a
# slow rig rather than no rig. Raising this to 2 or 3 trades that residual
# damping for a tighter definition of "trustworthy" -- a defensible choice, but
# it is a different one, so it is a constant and not a hardcoded `any()`.
MIN_HEALTHY_CHANNELS = 1

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
        # The firmware streams one column per analog pin it samples. arduino.ino
        # streams eight (A0..A7) but only A0..A3 carry OSEMs, so take the
        # leading four. Exactly-four input is unaffected -- parts[:4] is a
        # no-op there, so older four-column firmware still works.
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

    NEW (v4): two separate reasons a channel might not be driving, kept apart
    because conflating them is what makes auto-disable unsafe to reason about:

        enabled   STATIC CONFIG. ENABLE_CHANNEL, or a live edit from the
                  simulator UI. The operator's intent. v4 never writes it.
        healthy   RUNTIME STATE. Cleared when this channel's sensor rails,
                  restored REARM_SUSTAIN_S after it comes back. v4 owns it,
                  and nothing outside this module should touch it.

    `actuating` is the conjunction, and it is what gates the output. Keeping
    them separate means the simulator can keep assigning `ch.enabled` every
    frame (sim/server.py apply_gains) without silently un-demoting a blind
    channel, and means a status line can distinguish "off because you turned
    it off" from "off because it went blind".
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

        # NEW (v4): runtime health, distinct from the static `enabled` above.
        self.healthy = True
        self.demoted_since = None     # t of the demotion, for the status line
        self.clear_since = None       # t the rail cleared; the re-arm timer

    @property
    def actuating(self):
        """Driving its coil right now: configured on AND not demoted. Every
        gate that used to read `enabled` reads this instead, so there is one
        place that decides whether a channel is in the loop."""
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

    def update_health(self, t):
        """NEW (v4), "auto-disable". Demote this channel while its sensor is
        blind; re-arm it once the rail has been clear for REARM_SUSTAIN_S.

        Returns "demoted", "rearmed", or None, so the caller can emit one event
        per transition instead of one per sample.

        Only RAIL demotes. saturated_flag is deliberately not consulted: a
        saturated channel's measurement is fine and only its output is clipped,
        so pulling it to bias would remove damping authority at peak amplitude
        -- the exact failure this version exists to avoid. Saturation keeps v3's
        global path through check_runaway(). See the module docstring.
        """
        if self.rail_fault:
            self.clear_since = None
            if self.healthy:
                self.healthy = False
                self.demoted_since = t
                # Ramp the gain down from zero on the way back rather than
                # stepping in at whatever it held when the sensor went blind.
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
            # The FILTERS are left alone on purpose (hazard 2: re-seeding a
            # one-pole injects a step, and REARM_SUSTAIN_S was sized to let them
            # wash out instead). baseline_rms is left alone too -- carried
            # forward, not re-measured; see the docstring for why.
            #
            # The RMS WINDOWS are cleared, because they are the opposite case:
            # both are short sliding windows (2.0 s and 1.0 s) that spent the
            # blind period accumulating a flatlined bandpass near zero. Left in
            # place, that near-zero history sits inside the runaway window and
            # dilutes a genuine excursion for a full ENVELOPE_WINDOW_S after
            # re-entry -- the breaker would be at its least sensitive exactly
            # when the channel is least trusted.
            self.runaway_rms.reset()
            self.schedule_rms.reset()
            self.excess_since = None
            self.last_ratio = 0.0
            return "rearmed"
        return None

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
        # NEW (v4): a demoted channel is reading a railed sensor, so its
        # amplitude is not a measurement of anything and must not trip the
        # global breaker. Return before touching runaway_rms, so the blind
        # period never enters the window at all.
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

        # saturated_flag is maintained in actuate() now -- see the FIXED note
        # there. This only reads it, so the two conditions stay independent.
        if self.saturated_flag:
            tripped = True

        return tripped

    def update_lock(self, t_rel):
        # NEW (v4): `actuating`, not `enabled` -- a demoted channel is held at
        # bias and its bandpass has flatlined, which an RMS-only test reads as
        # the steadiest axis on the rack. Letting it vote would let a blind
        # sensor declare the optic locked. That is the same failure mode the
        # rail interlock was written for in the first place.
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
        # NEW (v4): `actuating`, not `enabled`. This is the only line that puts
        # the demotion into effect -- everything else is bookkeeping. The
        # else-branch below was already written to define bumpless re-entry for
        # a FAULT or a config-disabled channel, and a demoted channel is the
        # same case, so re-engagement needed no new code.
        if state == "DAMPING" and self.actuating:
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
        # NEW (v4): a global re-calibration is the one place a demotion is
        # forgotten wholesale. Everything is about to be re-measured from
        # scratch anyway, so carrying a stale demotion into the new baseline
        # would keep a since-recovered channel out of a run it belongs in.
        self.healthy = True
        self.demoted_since = None
        self.clear_since = None

    def status_str(self):
        tag = f"ch{self.idx}"
        if not self.enabled:
            return f"{tag}:off  bp={self.bp_out:+.3f}V"
        if not self.healthy:
            # NEW (v4). Distinct from ":off" above -- that is the operator's
            # choice, this is the rig's. Show how long it has been out.
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
                # NEW (v4), 13th column. Without it a log cannot answer "how
                # many axes were actually in the loop at time t", which is the
                # first question you ask of any run this version was written
                # for. rail_fault alone does not answer it: the demotion
                # outlasts the rail by REARM_SUSTAIN_S.
                str(int(self.healthy))]


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
        # NEW (v4): demotions are not faults and must not be counted as them --
        # the whole point is that the rig kept running. Counted separately so a
        # run can be scored on "how often did an axis drop out" without that
        # inflating the fault count every other version is compared on.
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
            # NEW (v4), "auto-disable". v3 read `if any_rail:` here and froze
            # all four. A rail is now a per-channel demotion, and only losing
            # the QUORUM faults the rig. Health is allowed to move in this
            # state alone: CALIBRATING above still faults globally on any rail,
            # and the FAULT path below re-calibrates, which clears demotions
            # wholesale via reset_for_recalibration().
            transitions = [(ch, ch.update_health(t)) for ch in channels]
            n_conf = sum(1 for ch in channels if ch.enabled)
            healthy_n = sum(1 for ch in channels if ch.actuating)
            for ch, ev in transitions:
                if ev == "demoted":
                    self.demote_count += 1
                    # A config-disabled channel is rail-checked like any other,
                    # so say what actually happened rather than claiming to have
                    # parked an output that was never moving.
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
                    # `actuating`, not `enabled`: a demoted channel is held at
                    # bias with a flatlined bandpass, so it can neither lock nor
                    # block a lock. It must not be able to do either.
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
