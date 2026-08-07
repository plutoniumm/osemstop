"""
Fast-lock OSEM velocity-feedback (cold damping) controller. Runs until Ctrl+C.

Version "zero", the baseline the ladder (ladder.py) is measured against. ONE
channel enabled, and a saturation trip LATCHES: FAULT is absorbing and needs a
manual restart. DO NOT RUN UNATTENDED.

Only A0 -> ch0 is validated on hardware (2026-07-15 run 3): Kp=-0.030 gives 6x
reduction in oscillation amplitude with no rail clipping; -0.040 is the onset of
instability/rail. Channels 1-3 default off, still filtered and rail-checked.

CALIBRATING (gain 0, per-channel noise floor) -> DAMPING -> FAULT, which freezes
all four outputs: the four OSEMs share one rigid body, so any one rails all.

Sensing is 5.02 V over 1023 ADC counts; actuation a 2.5 V DAC restricted to
VMIN..VMAX = 0.0..0.5 V around BIAS = 0.25 V.
"""

import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from pyDAC import DACController

# ===== SETTINGS =====
PORT = "COM7"
A_VCC = 5.02
ADC_MAX_COUNTS = 1023

# Coil DAC channel per sensor 0..3. Rewired 2026-08-03; full 8-pair map is
# [1,3,5,7,0,2,4,6] (provenance.md). A wrong map is invisible to the simulator
# and to every interlock.
DAC_CHANNELS = [1, 3, 5, 7]

VERSION_TAG, BENCH_STATUS = "zero", "validated"   # ran 2026-08-03 (one channel)

# Enable one at a time, only after confirming unclipped convergence on the scope.
ENABLE_CHANNEL = [True, False, False, False]

BIAS = np.array([0.25, 0.25, 0.25, 0.25])
VMIN, VMAX = 0.0, 0.5
MAX_SLEW_PER_S = 2.0

# Kp per channel. -0.030 on ch0 is the only hardware-validated value (2026-07-15
# run 3); -0.040 is the instability/rail onset, so do not exceed -0.035
# unattended. ch2 is POSITIVE because that OSEM is mounted the other way round:
# not a typo, and a wrong sign PUMPS rather than under-damps. CAPTURE_GAIN is
# transient-only and NOT validated.
STEADY_GAIN  = np.array([-0.030, -0.030, +0.010, -0.030])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035])

# Gain schedule: rolling-RMS/baseline ratio above CAPTURE_HIGH_FRAC -> full
# capture gain, below CAPTURE_LOW_FRAC -> full steady gain, linear between.
CAPTURE_HIGH_FRAC = 0.6
CAPTURE_LOW_FRAC = 0.2
SCHEDULE_WINDOW_S = 1.0

# gain-units/s, on the startup ramp and every schedule transition.
GAIN_SLEW_PER_S = 0.02

# PID on velocity error e = -vel. P is the only term that removes energy and the
# only one validated; I integrates to DISPLACEMENT (a spring), D to ACCELERATION
# (negative mass, noisiest). Both 0.0 here, so the loop is the validated P-only.
KI_GAIN = np.array([0.0, 0.0, 0.0, 0.0])
KD_GAIN = np.array([0.0, 0.0, 0.0, 0.0])
D_SMOOTH_HZ = 2.0      # lowpass on the D term (acceleration estimate)
I_CLAMP_V = 0.15       # anti-windup: |I term| never exceeds this, in DAC volts

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

# Raw ADC counts out of 0..1023, not volts: a railed sensor flatlines the
# bandpass, which an RMS-only check reads as perfect stability.
RAIL_LOW_COUNTS = 3
RAIL_HIGH_COUNTS = 1020
RAIL_SUSTAIN_S = 0.5

FAULT_CLEAR_SUSTAIN_S = 5.0     # all-clear for this long -> auto re-arm

STATUS_PERIOD_S = 5.0
CSV_FLUSH_EVERY_N = 200
# ====================


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
    """Returns (counts, volts) as two length-4 arrays, or None."""
    if not ser.in_waiting:
        return None
    try:
        raw = ser.readline().decode("utf-8").strip()
        parts = raw.split(",")
        # arduino.ino streams A0..A7; only A0..A3 carry OSEMs.
        if len(parts) < 4:
            return None
        counts = np.array([int(p) for p in parts[:4]])
        volts = counts * (A_VCC / ADC_MAX_COUNTS)
        return counts, volts
    except (ValueError, IndexError):
        return None


class Channel:
    """One OSEM/coil pair. Filtering, rail-checking and calibration run even
    when `enabled` is False, so disabled channels still trip the interlock.
    """

    def __init__(self, idx, dac, dac_channel, enabled, steady_gain, capture_gain,
                 ki, kd, bias):
        self.idx = idx
        self.enabled = enabled
        self.steady_gain = steady_gain
        self.capture_gain = capture_gain
        self.active_gain = 0.0
        self.bias = bias

        self.ki = ki
        self.kd = kd
        self.i_term = 0.0
        self.p_term = 0.0
        self.d_term = 0.0
        self.d_smooth = OnePoleFilter(D_SMOOTH_HZ, kind="low")
        self.prev_err = 0.0
        self.err_initialized = False

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
                print(f"RAIL! ch={self.idx}, counts={counts}")
                self.rail_fault = True
        else:
            self.rail_since = None
            self.rail_fault = False

    def accumulate_calibration(self):
        self.calib_sq_sum += self.bp_out ** 2
        self.calib_n += 1

    def finish_calibration(self):
        self.baseline_rms = max(np.sqrt(self.calib_sq_sum / max(self.calib_n, 1)), 1e-6)
        self.calib_sq_sum, self.calib_n = 0.0, 0

    def update_schedule(self, t_rel, dt):
        """Slew active_gain toward the amplitude-scheduled target. Also soft-start."""
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
        if state == "DAMPING" and self.enabled:
            err = -self.vel                       # setpoint is zero velocity
            self.p_term = self.active_gain * err
            if not self.err_initialized:
                self.prev_err = err
                self.err_initialized = True
            d_raw = (err - self.prev_err) / dt if dt > 0 else 0.0
            self.prev_err = err
            self.d_term = self.kd * self.d_smooth.update(d_raw, dt)

            u = self.p_term + self.i_term + self.d_term
            target = np.clip(self.bias + u, VMIN, VMAX)

            # Anti-windup: clamp the accumulator, and stop integrating when
            # pinned and the error pushes further in.
            pinned = not (VMIN < self.bias + u < VMAX)
            if not (pinned and (err > 0) == (u > 0)):
                self.i_term = float(np.clip(self.i_term + self.ki * err * dt,
                                            -I_CLAMP_V, I_CLAMP_V))
        else:
            # Drop every term: no resuming with a stale integral.
            self.p_term = self.d_term = self.i_term = 0.0
            self.err_initialized = False
            target = self.bias
        self.out = slew_limit(self.prev_out, target, dt, MAX_SLEW_PER_S)
        self.prev_out = self.out
        self.actuator.send(float(self.out))

    def reset_for_recalibration(self):
        self.calib_sq_sum, self.calib_n = 0.0, 0
        self.baseline_rms = None
        self.active_gain = 0.0
        self.i_term = self.p_term = self.d_term = 0.0
        self.err_initialized = False
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
    """The fast-lock state machine. main() drives it from serial, sim/server.py
    from a simulated plant, so the simulator tests this code and not a copy.
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
            for ch in channels:
                ch.accumulate_calibration()
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
            # KNOWN BUG, deliberate (versions.md hazard 1, fixed in v3):
            # saturated_flag is only recomputed in check_runaway(), which does
            # not run here, so a saturation trip LATCHES and FAULT is permanent.
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
