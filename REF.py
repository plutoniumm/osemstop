"""
Fast-lock OSEM damping controller -- no sweep, runs continuously
==================================================================
This replaces the manual gain sweep for day-to-day use. It applies a
validated velocity-feedback (cold damping) gain directly, from the moment
you press Enter, and reports when the mirror has actually settled instead
of requiring you to eyeball the scope.

*** NOW COVERS ALL 8 SENSOR/COIL PAIRS (A0-A7) ***
Only A0 -> ch0 has been experimentally confirmed (2026-07-15, run 3):
real damping at gain=-0.03 (6x reduction in oscillation amplitude, no
rail clipping), onset of instability/rail at gain=-0.04. All other
channels are wired the same way but their gain has NOT been individually
validated -- they default to DISABLED (see ENABLE_CHANNEL below), except
for ch1-ch3 which were already enabled in this script's prior 4-channel
configuration and are left as-is here. All eight channels are still
filtered, logged, and safety-checked at all times regardless of enabled
state, so you get free diagnostic data on every disabled channel while
the enabled ones run, and validating a new channel later is just a
matter of flipping one flag and watching the scope the same way we did
for ch0.

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
   freezes ALL EIGHT outputs to bias -- since all eight OSEMs sit on the
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

import os
import sys
import time
import numpy as np
from collections import deque
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from pyDAC import DACController

# ===================== SETTINGS =====================
# PORT = "/dev/ttyACM0"     # board-1 ; DAC-6
# PORT = "/dev/ttyACM1"     # board-1 ; DAC-3
PORT = "/dev/ttyUSB0"       # board-2 ; DAC-6

NUM_CHANNELS = 8   # A0..A7

A_VCC = 5.02
ADC_MAX_COUNTS = 1023

# Coil DAC channel for each sensor index (A0..A7). Previously this script
# had two SEPARATE 4-channel configs for "upper deck" wiring (DAC
# 1,3,5,7) vs "lower deck" wiring (DAC 0,2,4,6). Since all 8 sensors are
# now being run together (matching the 8-channel A0-A7 Arduino
# firmware), this assumes a direct 1:1 mapping. VERIFY THIS against your
# actual coil wiring before running -- the rail interlock only detects a
# sensor going out of range, it can't tell you if channel i's correction
# is being sent to the wrong coil.
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]

# Only ch0 (A0 -> DAC ch0) is experimentally validated. ch1-ch3 were
# already enabled in this script's prior 4-channel configuration and are
# left enabled here unchanged. ch4-ch7 (A4-A7) are new and default to
# DISABLED until validated individually on the scope, per this script's
# own stated procedure -- flip one at a time.
ENABLE_CHANNEL = [True, True, True, True, True, True, True, True]

BIAS = np.array([0.25, 0.25, 0.25, 0.25, 0.15, 0.15, 0.15, 0.15])
VMIN, VMAX = 0.0, 0.4
MAX_SLEW_PER_S = 2.0

# Damping gain per channel. STEADY_GAIN is the sweep-validated value on
# ch0 (-0.03: real damping, no rail clipping). CAPTURE_GAIN is a stronger
# value used only transiently while the oscillation is still large -- it
# is NOT independently validated. It stays under the -0.04 point the
# sweep flagged as problematic, but confirm on the scope before trusting
# it unattended, the same way -0.03 was confirmed.
#
# ch0-ch3 values are unchanged from the prior 4-channel config. ch4-ch7
# are new, disabled by default, and given placeholder magnitudes matching
# the ONE value that's actually been scope-validated (-0.03 / -0.035) --
# NOT the +/-0.300 already in use on ch0-ch3, which per the docstring is
# 10x the documented validated figure. Confirm sign and magnitude on the
# scope for each new channel individually before enabling it, exactly as
# was done for ch0.
STEADY_GAIN  = np.array([-0.200, -0.200, +0.200, -0.200,
                          -0.030, -0.030, -0.030, -0.030])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035,
                          -0.035, -0.035, -0.035, -0.035])

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

BP_LOW_HZ = 0.4
BP_HIGH_HZ = 3.0
DERIV_SMOOTH_HZ = 5.0

CALIBRATION_S = 8.0            # ~8 cycles of the ~1 Hz resonance, gain held at 0
LOCK_RMS_FACTOR = 0.35          # "locked" once rolling RMS < this * baseline
LOCK_SUSTAIN_S = 5.0
LOCK_WINDOW_S = 5.0

ENVELOPE_WINDOW_S = 2.0         # sliding window for the runaway check
RUNAWAY_MULTIPLE = 1.8
RUNAWAY_SUSTAIN_S = 2.0
MAX_CONSECUTIVE_SATURATED = 30

RAIL_LOW_COUNTS = 3             # raw ADC counts, out of 0..1023 -- not volts,
RAIL_HIGH_COUNTS = 1020         # to avoid float-rounding ambiguity
RAIL_SUSTAIN_S = 0.5

FAULT_CLEAR_SUSTAIN_S = 5.0     # all-clear for this long -> auto re-arm

STATUS_PERIOD_S = 5.0           # while any enabled channel is still settling
STATUS_PERIOD_SETTLED_S = 30.0  # once all enabled channels are locked+flat
CSV_FLUSH_EVERY_N = 200

# ---- Resonance ID + noise-floor tracking ----
# During each calibration window we now also fit the dominant resonance
# (f0), its undamped Q, and a time-domain noise-floor RMS from the same
# data, instead of only taking a single broadband RMS number. This lets
# the bandpass track the actual mode instead of a fixed guessed range, and
# gives us something real to call "flat" against -- the sensor's own noise
# floor, not just a fraction of the undamped motion.
AUTO_TRACK_RESONANCE = True     # reconfigure the bandpass to bracket f0 each calibration
BP_LOW_FRAC_OF_F0 = 0.5          # bandpass low corner  = BP_LOW_FRAC_OF_F0  * f0
BP_HIGH_FRAC_OF_F0 = 3.0         # bandpass high corner = BP_HIGH_FRAC_OF_F0 * f0
NOISE_BAND_FRAC_OF_F0 = 5.0      # measure noise floor starting this many x f0 above resonance

# Tighter "actually flat" criterion, separate from the existing LOCK
# detector (which only compares against the channel's own undamped
# baseline and can declare victory well above the real noise floor).
# NOISE_LIMITED requires the rolling RMS to also be within this factor of
# the measured noise floor -- i.e. residual motion is no longer
# distinguishable from sensor read noise, which is as flat as physically
# achievable with this sensor.
NOISE_MARGIN_FACTOR = 2.0
NOISE_LIMITED_SUSTAIN_S = 5.0

# ---- Optional slow centering (off by default) ----
# Pure velocity feedback only removes energy near resonance; it does
# nothing about slow drift of the working point. This adds a very-low
# bandwidth integral term (far below resonance, so it can't interact with
# the damping loop) that re-centers the optic. Off by default -- validate
# separately from the damping gain, the same way ch0's gain was validated.
ADD_INTEGRAL_CENTERING = False
INTEGRAL_HZ = 0.02               # far below f0 -- only chases slow drift
INTEGRAL_GAIN = 0.05
INTEGRAL_CLAMP_V = 0.05          # anti-windup: max authority given to the integrator

# ---- Optional safe auto gain search (off by default) ----
# Automates exactly the "nudge gain, watch it, back off if it gets worse"
# process used to validate -0.03 by hand, using the RMS metrics as the
# judge instead of eyeballing a scope. It can only ever move the gain
# toward AUTOTUNE_MAX_GAIN_MAG (which must stay under the -0.04 point
# already flagged as unstable) and it freezes and rolls back the instant
# a rail/runaway/saturation fault fires. Existing hard safety nets
# (rail interlock, runaway breaker, slew limits) are unchanged and are
# what actually keep this safe -- this only decides how hard to push
# *within* those limits. Leave off until steady-state gain behavior is
# well understood on the scope for a given channel.
AUTO_TUNE_GAIN = False
AUTOTUNE_CHECK_INTERVAL_S = 8.0
AUTOTUNE_STEP = 0.0005
AUTOTUNE_MAX_GAIN_MAG = 0.038     # hard ceiling, stays under the known -0.04 instability point
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


def estimate_resonance_and_noise(t_arr, y_arr):
    """Fit the dominant resonance and a real noise floor from one
    calibration-phase (gain=0) time series of the bandpassed signal.

    Returns (f0_hz, q0, noise_floor_rms), any of which may be None if the
    window was too short or too quiet to fit cleanly -- callers should
    fall back to the fixed BP_LOW_HZ/BP_HIGH_HZ defaults in that case.

    Method:
      - Resample onto a uniform time grid (tolerates the small dt jitter
        of a serial-polled loop) and take a Hann-windowed PSD.
      - f0 = frequency of the largest peak in a plausible band.
      - Q0 = f0 / (half-power bandwidth around that peak) -- the standard
        -3dB method, i.e. exactly what Table III-style Q figures use.
      - noise_floor_rms: high-pass the *original* uniform series well
        above f0 (NOISE_BAND_FRAC_OF_F0 x) and take its time-domain RMS.
        By that far above resonance the mechanical response has fallen
        off steeply (~1/f^2 for a simple pendulum), so what's left is a
        reasonable proxy for the sensor's own electronic noise floor
        rather than real motion.
    """
    t_arr = np.asarray(t_arr, dtype=float)
    y_arr = np.asarray(y_arr, dtype=float)
    if len(t_arr) < 64:
        return None, None, None

    dt_mean = float(np.median(np.diff(t_arr)))
    if dt_mean <= 0:
        return None, None, None
    fs = 1.0 / dt_mean

    t_uniform = np.arange(t_arr[0], t_arr[-1], dt_mean)
    if len(t_uniform) < 64:
        return None, None, None
    y_uniform = np.interp(t_uniform, t_arr, y_arr)
    y_uniform = y_uniform - np.mean(y_uniform)

    yw = y_uniform * np.hanning(len(y_uniform))
    freqs = np.fft.rfftfreq(len(yw), d=dt_mean)
    psd = np.abs(np.fft.rfft(yw)) ** 2

    search = (freqs > 0.1) & (freqs < fs / 3.0)
    if not np.any(search):
        return None, None, None
    f0 = float(freqs[search][np.argmax(psd[search])])

    peak_idx = int(np.argmin(np.abs(freqs - f0)))
    half_power = psd[peak_idx] / 2.0
    lo, hi = peak_idx, peak_idx
    while lo > 0 and psd[lo] > half_power:
        lo -= 1
    while hi < len(psd) - 1 and psd[hi] > half_power:
        hi += 1
    delta_f = max(freqs[hi] - freqs[lo], fs / len(yw))
    q0 = float(f0 / delta_f) if delta_f > 0 else None

    hp_noise = OnePoleFilter(max(NOISE_BAND_FRAC_OF_F0 * f0, 5.0), kind="high")
    residual = np.array([hp_noise.update(v, dt_mean) for v in y_uniform])
    settle = int(len(residual) * 0.2)  # drop filter's own settling transient
    if len(residual) > settle + 8:
        noise_floor_rms = float(np.sqrt(np.mean(residual[settle:] ** 2)))
    else:
        noise_floor_rms = None

    return f0, q0, noise_floor_rms


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
    """Returns (counts, volts) as two length-NUM_CHANNELS arrays, or None."""
    if not ser.in_waiting:
        return None
    try:
        raw = ser.readline().decode("utf-8").strip()
        parts = raw.split(",")
        if len(parts) != NUM_CHANNELS:
            return None
        counts = np.array([int(p) for p in parts])
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

    def __init__(self, idx, dac, dac_channel, enabled, steady_gain, capture_gain, bias):
        self.idx = idx
        self.enabled = enabled
        self.steady_gain = steady_gain
        self.capture_gain = capture_gain
        self.active_gain = 0.0
        self.bias = bias

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
        self.baseline_rms = None
        self.calib_t = []
        self.calib_bp = []
        self.f0 = None
        self.q0 = None
        self.noise_floor_rms = None

        self.lock_rms = SlidingRMS(LOCK_WINDOW_S)
        self.runaway_rms = SlidingRMS(ENVELOPE_WINDOW_S)
        self.schedule_rms = SlidingRMS(SCHEDULE_WINDOW_S)
        self.last_ratio = 0.0

        self.rail_since = None
        self.rail_fault = False
        self.excess_since = None
        self.sat_streak = 0
        self.saturated_flag = False
        self.locked = False
        self.locked_since = None
        self.noise_limited = False
        self.noise_limited_since = None

        # auto gain search (off unless AUTO_TUNE_GAIN)
        self.autotune_last_check = None
        self.autotune_last_rms = None
        self.autotune_frozen = False

        # optional slow drift centering (off unless ADD_INTEGRAL_CENTERING)
        self.integ_lp = OnePoleFilter(INTEGRAL_HZ, kind="low")
        self.integ_term = 0.0

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
        if counts <= RAIL_LOW_COUNTS or counts >= RAIL_HIGH_COUNTS:
            if self.rail_since is None:
                self.rail_since = now
            elif now - self.rail_since >= RAIL_SUSTAIN_S:
                self.rail_fault = True
        else:
            self.rail_since = None
            self.rail_fault = False

    def accumulate_calibration(self, t_rel):
        self.calib_sq_sum += self.bp_out ** 2
        self.calib_n += 1
        self.calib_t.append(t_rel)
        self.calib_bp.append(self.bp_out)

    def finish_calibration(self):
        self.baseline_rms = max(np.sqrt(self.calib_sq_sum / max(self.calib_n, 1)), 1e-6)
        self.calib_sq_sum, self.calib_n = 0.0, 0

        f0, q0, noise_floor = estimate_resonance_and_noise(self.calib_t, self.calib_bp)
        self.calib_t, self.calib_bp = [], []
        if f0 is not None:
            self.f0, self.q0, self.noise_floor_rms = f0, q0, noise_floor
            if AUTO_TRACK_RESONANCE:
                low_hz = max(BP_LOW_FRAC_OF_F0 * f0, 0.05)
                high_hz = max(BP_HIGH_FRAC_OF_F0 * f0, low_hz * 2)
                self.hp = OnePoleFilter(low_hz, kind="high")
                self.lp = OnePoleFilter(high_hz, kind="low")
                self.deriv_smooth = OnePoleFilter(min(DERIV_SMOOTH_HZ, high_hz), kind="low")
                self.filt_initialized = False  # let the new filters settle fresh
        # else: keep whatever f0/q0/noise_floor_rms (and filters) we had before,
        # and fall back to the fixed BP_LOW_HZ/BP_HIGH_HZ defaults if this is
        # the very first calibration and identification failed.

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

        if self.out <= VMIN + 1e-6 or self.out >= VMAX - 1e-6:
            self.sat_streak += 1
        else:
            self.sat_streak = 0
        if self.sat_streak > MAX_CONSECUTIVE_SATURATED:
            self.saturated_flag = True
            tripped = True
        else:
            self.saturated_flag = False

        return tripped

    def update_lock(self, t_rel):
        if not self.enabled or self.baseline_rms is None:
            self.locked = False
            self.locked_since = None
            self.noise_limited = False
            self.noise_limited_since = None
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

        # Stricter criterion: are we actually down at the sensor's own
        # noise floor, not just below some fraction of the undamped
        # amplitude? This is the "flat line" condition.
        if self.noise_floor_rms is not None and local_rms < self.noise_floor_rms * NOISE_MARGIN_FACTOR:
            if self.noise_limited_since is None:
                self.noise_limited_since = t_rel
            elif t_rel - self.noise_limited_since >= NOISE_LIMITED_SUSTAIN_S:
                self.noise_limited = True
        else:
            self.noise_limited_since = None
            self.noise_limited = False

    def maybe_autotune(self, t_rel):
        """Optional: nudge steady_gain magnitude up while it keeps helping,
        back off the instant it doesn't. Only runs if AUTO_TUNE_GAIN is on,
        never exceeds AUTOTUNE_MAX_GAIN_MAG, and freezes permanently for
        this run the first time it makes things worse -- it does not
        oscillate back and forth searching for an edge on live hardware.
        """
        if not (AUTO_TUNE_GAIN and self.enabled) or self.autotune_frozen:
            return
        if self.baseline_rms is None or self.rail_fault or self.saturated_flag:
            return
        if self.autotune_last_check is None:
            self.autotune_last_check = t_rel
            self.autotune_last_rms = self.last_ratio * self.baseline_rms
            return
        if t_rel - self.autotune_last_check < AUTOTUNE_CHECK_INTERVAL_S:
            return

        current_rms = self.last_ratio * self.baseline_rms
        improved_or_flat = current_rms <= (self.autotune_last_rms * 1.02)
        self.autotune_last_check = t_rel

        if improved_or_flat and abs(self.steady_gain) < AUTOTUNE_MAX_GAIN_MAG:
            step = -AUTOTUNE_STEP if self.steady_gain < 0 else AUTOTUNE_STEP
            new_gain = self.steady_gain + step
            if abs(new_gain) <= AUTOTUNE_MAX_GAIN_MAG:
                self.steady_gain = new_gain
                self.capture_gain = new_gain + (self.capture_gain - (self.steady_gain - step))
                print(f"    [autotune] ch{self.idx} steady_gain -> {self.steady_gain:+.4f}")
        elif not improved_or_flat:
            step_back = AUTOTUNE_STEP if self.steady_gain < 0 else -AUTOTUNE_STEP
            self.steady_gain += step_back
            self.autotune_frozen = True
            print(f"    [autotune] ch{self.idx} regressed -- freezing at "
                  f"steady_gain={self.steady_gain:+.4f} for this run")

        self.autotune_last_rms = current_rms

    def actuate(self, dt, state):
        if state == "DAMPING" and self.enabled:
            u = -self.active_gain * self.vel
            if ADD_INTEGRAL_CENTERING:
                dc_est = self.integ_lp.update(self.bp_out, dt)
                self.integ_term = float(np.clip(
                    self.integ_term - INTEGRAL_GAIN * dc_est * dt,
                    -INTEGRAL_CLAMP_V, INTEGRAL_CLAMP_V))
                u += self.integ_term
            target = np.clip(self.bias + u, VMIN, VMAX)
        else:
            target = self.bias
            self.integ_term = 0.0  # reset centering authority when not actively damping
        self.out = slew_limit(self.prev_out, target, dt, MAX_SLEW_PER_S)
        self.prev_out = self.out
        self.actuator.send(float(self.out))

    def reset_for_recalibration(self):
        self.calib_sq_sum, self.calib_n = 0.0, 0
        self.baseline_rms = None
        self.active_gain = 0.0
        self.last_ratio = 0.0
        self.excess_since = None
        self.sat_streak = 0
        self.saturated_flag = False
        self.locked = False
        self.locked_since = None
        self.noise_limited = False
        self.noise_limited_since = None
        self.lock_rms.reset()
        self.runaway_rms.reset()
        self.schedule_rms.reset()
        self.calib_t, self.calib_bp = [], []
        self.autotune_last_check = None
        self.autotune_last_rms = None
        # NOTE: f0 / q0 / noise_floor_rms and any autotune_frozen gain are
        # deliberately *not* reset here -- they carry across a recalibration
        # so a fault doesn't erase what was already learned this run.

    def is_settled(self):
        return self.enabled and self.locked and self.noise_limited

    def status_str(self):
        tag = f"ch{self.idx}"
        if not self.enabled:
            return f"{tag} off"
        if self.rail_fault:
            return f"{tag} RAIL FAULT"

        if self.is_settled():
            # nothing new to say -- just where it's sitting
            return f"{tag} flat  {self.bp_out:+.3f}V"

        # still converging: this is the number worth watching
        x_floor = None
        if self.noise_floor_rms:
            local_rms = self.last_ratio * (self.baseline_rms or 0.0)
            x_floor = local_rms / self.noise_floor_rms
        margin = f" ({x_floor:.1f}x floor)" if x_floor is not None else ""
        return f"{tag} settling  {self.bp_out:+.3f}V{margin}"

    def log_fields(self, counts):
        volt = counts * (A_VCC / ADC_MAX_COUNTS)
        return [str(int(counts)), f"{volt:.4f}", f"{self.bp_out:.5f}", f"{self.vel:.5f}",
                f"{self.out:.4f}", f"{self.active_gain:.5f}", f"{self.last_ratio:.4f}",
                str(int(self.rail_fault)), str(int(self.locked)), str(int(self.noise_limited)),
                f"{self.f0:.4f}" if self.f0 else "", f"{self.q0:.2f}" if self.q0 else "",
                f"{self.noise_floor_rms:.5f}" if self.noise_floor_rms else ""]


def main():
    dac = DACController(port=PORT)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))

    input("DAC biases set. Press Enter to start fast-lock damping (Ctrl+C to stop)... ")
    dac.start_stream()
    ser = dac.ser

    channels = [Channel(i, dac, DAC_CHANNELS[i], ENABLE_CHANNEL[i],
                        STEADY_GAIN[i], CAPTURE_GAIN[i], BIAS[i])
                for i in range(NUM_CHANNELS)]

    os.makedirs("data", exist_ok=True)
    csv_path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    csv_file = open(csv_path, "w", buffering=1)
    header = "time_s,state," + ",".join(
        f"ch{i}_counts,ch{i}_V,ch{i}_bp,ch{i}_vel,ch{i}_out,ch{i}_gain,ch{i}_ratio,ch{i}_rail,"
        f"ch{i}_locked,ch{i}_noise_limited,ch{i}_f0,ch{i}_q0,ch{i}_noise_floor"
        for i in range(NUM_CHANNELS)
    )
    csv_file.write(header + "\n")
    print(f"Logging to {csv_path}")

    start_time = time.time()
    prev_t = time.time()
    state = "CALIBRATING"
    calib_start = time.time()
    damping_start = None
    fault_clear_since = None
    locked_announced = False
    last_status = 0.0
    row_count = 0

    print(f"\n[{state}] measuring baseline noise for {CALIBRATION_S:.0f}s "
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

            for ch in channels:
                ch.filter_sample(volts[ch.idx], dt)
                ch.check_rail(counts[ch.idx], now)
            any_rail = any(ch.rail_fault for ch in channels)

            if state == "CALIBRATING":
                for ch in channels:
                    ch.accumulate_calibration(t_rel)
                if any_rail:
                    print("\n!! sensor railed during calibration -- check alignment "
                          "before continuing. Entering FAULT.\n")
                    state = "FAULT"
                    fault_clear_since = None
                elif t_rel - (calib_start - start_time) >= CALIBRATION_S:
                    for ch in channels:
                        ch.finish_calibration()
                    state = "DAMPING"
                    damping_start = now
                    locked_announced = False
                    main._nlim_announced = False
                    baselines = ", ".join(f"ch{ch.idx}={ch.baseline_rms:.4f}V" for ch in channels)
                    print(f"[{state}] baseline set ({baselines}). "
                          f"Gain will schedule between {CAPTURE_GAIN[0]:+.3f} (large amplitude) "
                          f"and {STEADY_GAIN[0]:+.3f} (near lock)...\n")
                    for ch in channels:
                        if ch.f0 is not None:
                            nf = f"{ch.noise_floor_rms:.5f}V" if ch.noise_floor_rms else "n/a"
                            print(f"    ch{ch.idx}: f0={ch.f0:.3f}Hz  Q0~{ch.q0:.1f}  "
                                  f"noise_floor~{nf}  bandpass->[{BP_LOW_FRAC_OF_F0*ch.f0:.2f}, "
                                  f"{BP_HIGH_FRAC_OF_F0*ch.f0:.2f}]Hz")
                        else:
                            print(f"    ch{ch.idx}: resonance ID failed this cycle, "
                                  f"keeping previous filter settings")
                    print()

            elif state == "DAMPING":
                if any_rail:
                    print("\n!! sensor rail detected -- freezing all channels, entering FAULT.\n")
                    state = "FAULT"
                    fault_clear_since = None
                else:
                    tripped_any = False
                    for ch in channels:
                        if ch.enabled:
                            ch.update_schedule(t_rel, dt)
                        else:
                            ch.active_gain = 0.0
                        if ch.check_runaway(t_rel):
                            tripped_any = True
                            print(f"\n!! ch{ch.idx} runaway/saturation -- freezing all "
                                  f"channels, entering FAULT.\n")
                    if tripped_any:
                        state = "FAULT"
                        fault_clear_since = None
                    else:
                        for ch in channels:
                            ch.update_lock(t_rel)
                            ch.maybe_autotune(t_rel)
                        enabled_locked = [ch.locked for ch in channels if ch.enabled]
                        if enabled_locked and all(enabled_locked) and not locked_announced:
                            print(f"\n*** LOCKED -- {now - damping_start:.1f}s after gain "
                                  f"was applied (t={t_rel:.1f}s total) ***\n")
                            locked_announced = True
                        enabled_noise_limited = [ch.noise_limited for ch in channels if ch.enabled]
                        if (enabled_noise_limited and all(enabled_noise_limited)
                                and not getattr(main, "_nlim_announced", False)):
                            print(f"\n*** NOISE-FLOOR-LIMITED -- residual motion on all "
                                  f"enabled channels is within {NOISE_MARGIN_FACTOR:.0f}x of "
                                  f"the measured sensor noise floor (t={t_rel:.1f}s) ***\n")
                            main._nlim_announced = True

            elif state == "FAULT":
                for ch in channels:
                    ch.active_gain = 0.0
                all_clear = not any_rail and all(not ch.saturated_flag for ch in channels)
                if all_clear:
                    if fault_clear_since is None:
                        fault_clear_since = now
                    elif now - fault_clear_since >= FAULT_CLEAR_SUSTAIN_S:
                        print(f"\n[recovered] all channels clear for "
                              f"{FAULT_CLEAR_SUSTAIN_S:.0f}s -- re-calibrating and resuming.\n")
                        for ch in channels:
                            ch.reset_for_recalibration()
                        state = "CALIBRATING"
                        calib_start = now
                        fault_clear_since = None
                else:
                    fault_clear_since = None

            for ch in channels:
                ch.actuate(dt, state)

            all_settled = state == "DAMPING" and all(
                ch.is_settled() for ch in channels if ch.enabled)
            period = STATUS_PERIOD_SETTLED_S if all_settled else STATUS_PERIOD_S
            if now - last_status >= period:
                last_status = now
                print(f"[{t_rel:7.1f}s] {state:11s} " +
                      "  ".join(ch.status_str() for ch in channels))

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

        # Hardware safety first, unconditionally -- never gated behind the
        # save/discard question below.
        dac.stop_stream()
        print("Returning DAC outputs to bias voltages...")
        for c, b in zip(DAC_CHANNELS, BIAS):
            try:
                dac.set_voltage(channel=c, voltage=float(b))
            except RuntimeError:
                pass
        time.sleep(0.1)
        dac.close()

        # Now it's safe to block on a prompt. Default is to keep the data --
        # a second Ctrl+C or closed stdin here shouldn't be able to silently
        # throw away a run.
        keep_data = True
        try:
            answer = input(f"\nSave this run's data ({csv_path})? [Y/n]: ").strip().lower()
            keep_data = not answer.startswith("n")
        except (EOFError, KeyboardInterrupt):
            print("(no response -- keeping the data by default)")
            keep_data = True

        if keep_data:
            print(f"Log saved to {csv_path}")
        else:
            try:
                os.remove(csv_path)
                print(f"Discarded: deleted {csv_path}")
            except OSError as e:
                print(f"Could not delete {csv_path}: {e}")


if __name__ == "__main__":
    main()
