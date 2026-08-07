"""
delta: eight OSEMs, 500000 baud wire, control step decoupled from the wire clock.

FIXES lists the seventeen. New here: `sample-guard` (a count outside
0..ADC_MAX_COUNTS is a FRAMING error, not a sample: 10 torn rows in v11's 240 s
log against 0 in every slower one, one worth +45.6 V into the bandpass),
`decimate` (read the wire at full rate, step at CONTROL_HZ on the mean, which
REQUIRES sample-guard because a boxcar spreads a bad sample over a whole step),
`persist-baseline` (the floor cached to JSON behind a fingerprint and an age
limit; cold start 20 s -> 2.0 s) and `baseline-sanity` (a loaded or freshly
measured floor must agree with live signal to within BASELINE_SANITY_RATIO).
`--fresh` / OSEM_FRESH_CALIB=1 refuse the cache by hand.

WATCH:
  * STEADY_GAIN[2] is POSITIVE because ch2 is mounted the other way round. Not a
    typo. A wrong sign PUMPS, it does not under-damp.
  * Kp = -0.040 is the documented instability / rail onset; do not exceed -0.035
    unattended. The simulator provably cannot reproduce that limit.
  * DAC_CHANNELS cannot be checked in software. Verify on the bench, one coil at
    a time, before trusting a run.
  * The rail interlock reads RAW COUNTS at the full wire rate, never the decimated
    mean: a railed sensor flatlines the bandpass, which an RMS-only check reads as
    perfect stability.
  * Needs a reflash (`make arduino`) at pyDAC2's baud. The simulator models FOUR
    OSEMs, so it cannot test this file and harness.py skips it.

Sensing is 5.02 V over 1023 counts; actuation a 2.5 V DAC restricted to 0..0.5 V
around BIAS = 0.25. `enabled` is static config, `healthy` runtime, and
`enabled & healthy` gates the output; absent timers are np.inf.
"""

import json
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime, timezone

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
# DACController.set_voltage() reads up to 50 lines waiting for `OK`, each a lost
# stream sample: 73.0 / 23.5 / 12.5 Hz driving 1 / 4 / 8 coils. FastDAC drops the
# ack, otherwise the same protocol and the same 0..2.5 V firmware clamp.
from pyDAC2 import FastDAC

# ===================== SETTINGS =====================
PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 8

# Sensor 0..7 -> DAC channel, from `ref.py` (provenance.md). NOTHING IN SOFTWARE
# CAN CHECK THIS: sim/server.py stores and reads back through the same map, so a
# wrong map round-trips cleanly and every interlock sees a plausible loop closed
# onto somebody else's coil. Verify on the bench, one coil at a time.
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]

# BENCH 2026-08-06: locked at ratio 0.05-0.15 against 0.35, ~0.012-0.018 V
# residual on a 0.0530 V quiet floor, wire 1018-1043 Hz decimated to 100 Hz, 0 torn
# frames over three runs and a 14-min dither; a kick railing 7 of 8 ADCs gave one
# runaway trip, ch0 peak ratio 3.79, back under threshold in ~15 s.
VERSION_TAG, BENCH_STATUS = "delta", "validated"
# A NAME, not a number. See ladder.py.
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend",
         "bias-trim", "soft-saturation", "baseline-floor",
         "sat-window", "fast-transport",
         "sample-guard", "decimate", "persist-baseline", "baseline-sanity")

# All eight. `enabled` buys a place in the quorum, the lock claim and the trim;
# every channel is filtered, rail-checked and watched regardless. Safe only via
# `baseline-floor`: a channel reading nothing calibrates ~0.002 V, every ratio
# explodes, and the breaker faults the rig (v5.5: ten times in 145 s).
ENABLE_CHANNEL = [True, True, True, True, True, True, True, True]

BIAS = np.full(N, 0.25)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 0.5, 2.0     # nominal; the trim moves the
                                               # __init__, which the trim moves

# Kp. STEADY is bench-validated; CAPTURE is stronger, large-amplitude only, NOT
# independently validated.
#   STEADY_GAIN[2] is POSITIVE: ch2 is mounted the other way round (provenance.md
#   Table 1). Not a typo. A wrong sign PUMPS, it does not under-damp.
#   Kp = -0.040 is the documented instability / rail onset; do not exceed -0.035
#   unattended. The simulator provably cannot reproduce that limit.
# MEASURED 2026-08-06, tune.py fit of data/20260806_182231_tune_raw.csv, 717420
# samples at 880 Hz. ch2 +0.010 -> +0.035, the largest modal residue of the four
# (+13.573 against -7.851 / -8.867 / -5.282 at mode 0).
# a4-a7 ZERO GAIN, sensors only: 0.1-9% of power in 0.4-3 Hz, lock-in SNR 1.1-1.5,
# coil signs unknown (DC diagonals +8 / +4 / +7 counts/V on a 4-count mean
# |response|). Stepped sine, eight coils, both quadratures (CLAUDE.md 2b) fixes it.
STEADY_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                        +0.000, +0.000, +0.000, +0.000])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                         +0.000, +0.000, +0.000, +0.000])

# Ki/Kd on the velocity loop: the integral of velocity is DISPLACEMENT (a spring,
# not damping), its derivative ACCELERATION (negative mass, noisiest term). Same
# sign per channel as Kp, so a guessed sign is wrong in three places at once.
# Ki cut to 15%: MEASURED, the old Ki spent 0.1135 V of peak actuator, 38% of the
# budget, dissipating nothing behind the 0.4 Hz highpass. Peak demand 0.2998 V
# against a 0.250 V half-window, 20% over, which is the clipping; this set lands
# at 0.2001 V, dissipating share 43 -> 88%.
KI_GAIN = np.array([-0.0060, -0.0060, +0.0060, -0.0060,
                    +0.0000, +0.0000, +0.0000, +0.0000])
KD_GAIN = np.array([-0.00045, -0.00045, +0.00045, -0.00045,
                    +0.00000, +0.00000, +0.00000, +0.00000])
D_SMOOTH_HZ, I_CLAMP_V, TRACK_TC_S = 2.0, 0.15, 0.5   # I_CLAMP is a backstop

BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ = 0.4, 3.0, 5.0
CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
SCHEDULE_WINDOW_S, GAIN_SLEW_PER_S = 1.0, 0.02

# ===== decimate: the loop's own clock =====
# The ARRIVAL rate has been 12.5, 18, 23.5, 73 and ~1111 Hz here, set by coil count
# and transport; none of those are control decisions, this one is.
# 100 Hz is bracketed: 27x the fastest mode (2.79 Hz), 33x the authority peak
# (~3 Hz), 9.7 wire samples per step at 1111 Hz, 13.1 deg of phase at 3.75 Hz
# against v11 and 18.4 deg MORE than the four-coil rig the gains were measured at;
# 50 Hz costs 14 deg, 200 Hz costs 6. A slower wire runs a step per arrival.
CONTROL_HZ = 100.0
CONTROL_PERIOD_S = 1.0 / CONTROL_HZ

# ===== calibration: a ceiling and a convergence test, not a fixed duration =====
# CALIBRATION_S is the LONGEST it may take; the ceiling path is v5's estimator.
# CALIB_SUBWINDOW_S = 2.0 s is two cycles of the ~1 Hz resonance, the shortest
# window whose RMS is not dominated by its start phase, and divides the ceiling by
# 10. CALIB_AGREE_TOL = 1.20 sits in a measured gap: the trailing three agree to
# 1.09 then ~1.03 in a quiet lab, but run 1.19-1.66 for the whole 20 s under a
# 6 V/s kick inside the window. Hence _calib_stationary's SECOND test.
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # ceiling; subwindows >= 3
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20

# LOCK_SUSTAIN_S is the next cut, 5 s of a 10 s budget confirming a lock already
# had; deferred because `decimate` also moves the noise the lock detector sees.
LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0
ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE, RUNAWAY_SUSTAIN_S = 2.0, 1.8, 2.0
# The breaker also requires GROWTH, the envelope against itself
# RUNAWAY_TREND_LAG_S ago; 1.02 leaves 2% so envelope noise cannot read as growth.
# RUNAWAY_TREND_LAG_S = 10.0: 2.8x the longest mode beat (modes 0.7155 / 0.9949 /
# 1.6396 Hz beat at 1.082 / 1.551 / 3.578 s, analysis/ringdown.md), 5.6-13x under
# tau 56-130 s, so a real decay falls 7.4-16% across it against 1.5-3.6% at v9's
# 2.0 s, which read the beat instead.
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 10.0, 1.02

# ===== soft-saturation =====
# Clipping removes authority in ONE direction and the coil still pulls the other
# way at full strength, so the fault needs both halves: pinned for SAT_FRACTION of
# a SAT_SUSTAIN_S window AND the envelope not falling, SAT_DECAY_FRAC mirroring
# RUNAWAY_GROWTH_FRAC over the same lag and env_hist. Do NOT add a clip_excess
# condition; see _actuate().
# SECONDS, not samples: 30 consecutive samples meant 0.41 / 1.28 / 2.40 s on 1 / 4
# / 8 coils (73.0 / 23.5 / 12.5 Hz) and one clean sample erased the evidence, so
# this NEVER FIRED in the 240 s three-kick bench run of 2026-08-06. 1.30 s
# preserves four-coil behaviour (30 x 42.6 ms = 1.278 s); 0.80 is the rail
# check's own fraction.
SAT_SUSTAIN_S, SAT_FRACTION = 1.30, 0.80
SAT_DECAY_FRAC = 0.98

# ===== baseline-floor =====
# Smallest baseline a channel may calibrate and still be trusted, as a FRACTION of
# the median across enabled channels. Relative because the floor scales with the
# lab and with alignment (a5 reads the same motion at ~1/12 of a0's gain, two
# independent measurements 2026-08-06); 0.10 because all eight are on ONE rigid
# body, so gains spread in-band RMS by a factor of a few, not 10x. Bench brackets
# (`v55_all8.log`, fractions of the median): a4 0.010 / a7 0.014 / a6 0.036
# demoted, a5 0.579 kept. A dead channel costs the whole rig, so bias to KEEPING.
BASELINE_FLOOR_FRAC = 0.10

# ===== bias trim (from v7) =====
# No OSEM rests at mid-scale: 600 / 631 / 708 / 677 counts against 511.5 at bias
# 0.25 V, so every channel clips its TOP rail first and ch2 has least room. Coil
# bias is a DC force; on hardware 2026-08-04 the trim took total offset
# 565 -> 455 counts at no cost in lock time. NOT a one-shot least-squares solve,
# which wants -1.93..+3.77 V against a 2.5 V DAC. One quantum at a time, kept if
# the TOTAL offset improved, four coils driving two DOF (v6 rank check: 2
# directions above 10%), so revert-on-worse costs one step, not a railed sensor.
BIAS_QUANTUM = 0.25            # coarse: 2 DOF, 4 knobs, so fine steps would
BIAS_MIN, BIAS_MAX = 0.25, 1.25          # just chase each other
BIAS_SWING = 0.25              # +-this around each channel's own bias
MID_COUNTS = 511.5             # (ADC_MAX_COUNTS - 1) / 2
TRIM_PERIOD_S = 15.0           # >= one ringdown at Q~50, f0~1 Hz (~16 s)
TRIM_DEADBAND_COUNTS = 40.0    # inside this, leave it alone
TRIM_MAX_STEPS = 4             # per channel, per run
# A bias step is a force step, ringing the pendulum at ~1 Hz inside
# BP_LOW_HZ..BP_HIGH_HZ, the band both TREND tests read, so it INVALIDATES the
# history: env_hist cleared, excess_since reset, trend halves resuming within
# RUNAWAY_TREND_LAG_S while the level halves keep working. `warm-restart` refuses
# a pre-step baseline.
# SLOPE_SIGN moves a channel's bias to reduce its counts,
#   step = -sign(error) * SLOPE_SIGN * BIAS_QUANTUM
# from the DIAGONAL of the per-coil DC matrix, one coil at a time, 2026-08-04
# (`bench/20260804/dcmatrix.log`), counts/V:
#     a0 -105   a1  -68   a2 +213   a3  -51    solidly measured
#     a4   +8   a5   -7   a6   +4   a7   +7    at that block's noise, mean 4
# ch2 is +1 because it is mounted the other way round. Do NOT use a common-mode
# sweep: four coils together read a3 as +43 counts/V, coil3 -> a3 alone -51.
SLOPE_SIGN = np.array([-1.0, -1.0, +1.0, -1.0,
                       +1.0, -1.0, +1.0, +1.0])

# Rail: raw counts, so there is no float-rounding ambiguity, and a FRACTION of a
# window rather than an unbroken run, so one noise sample dilutes the evidence
# instead of erasing it.
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80

FAULT_CLEAR_SUSTAIN_S = 5.0
REARM_SUSTAIN_S = 2.0        # 5 tau of the 0.4 Hz highpass; timed from rail-clear
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md §3)

# ===== when the fault path may borrow the baseline it already has =====
# An engagement of >= FAST_REFAULT_S resets the reuse budget, the loop having just
# validated that baseline; MAX_BASELINE_REUSE bounds reuses without one and
# BASELINE_MAX_AGE_S is the wall-clock backstop. See _why_reuse.
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0

# ===== persist-baseline =====
# The floor across process restarts. Anchored to THIS FILE's directory, not the
# cwd: `bench.py`, `make run` and a bare `python osem.delta.py` differ.
BASELINE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "data", "baseline.json")
BASELINE_FILE_SCHEMA = 1
# 6x the in-run BASELINE_MAX_AGE_S: 300 s bounds a floor a just-FAULTED loop leans
# on, 1800 s bounds "same lab session", 96x shorter than the two days over which
# the bench floor moved 2.0-2.4x.
BASELINE_FILE_MAX_AGE_S = 1800.0
# Measured, so a tolerance and not equality. 1.5x is above link jitter and below
# every real change: 12.5 -> 18 -> 23.5 -> 1111 Hz are all >= 1.4x apart.
WIRE_RATE_TOL = 1.5
# A MEASUREMENT, not a fingerprint: the loaded floor must agree per channel with
# the first BASELINE_WARMUP_S to within this. Same-session reproducibility is
# 1.21-1.35x, a moved flag 12x (a5 vs a0) to 200x. Biased toward REFUSING.
BASELINE_SANITY_RATIO = 3.0
# Consecutive fresh-baseline refusals before taking the fresh number anyway: a low
# floor only makes the breaker over-cautious, so this just bounds a livelock.
MAX_BASELINE_REFUSALS = 3
# 5 tau of the 0.4 Hz highpass, as REARM_SUSTAIN_S: the filters must be primed. It
# also covers RAIL_SUSTAIN_S = 0.5 s, arming the rail interlock before any coil.
BASELINE_WARMUP_S = 2.0

STATUS_PERIOD_S, CSV_FLUSH_EVERY_N = 5.0, 200

# ===== kick-cue =====
# The operator is at the optic, not the terminal, so the schedule gets spoken.
# `say` is macOS-only and its absence is silent. OFF BY DEFAULT: `make check` runs
# 200-odd scenarios. NON-BLOCKING (Popen): `say` takes ~700 ms, 70 missed steps at
# 100 Hz with the coils held where they were.
#     OSEM_KICK_CUE=25 make run V=delta        # cue every 25 s once damping
KICK_CUE_S = float(os.environ.get("OSEM_KICK_CUE", "0") or 0)
KICK_CUE_PHRASE = "jerk it"
_KICK_CUE_LEAD_S = 3.0          # first cue this long after DAMPING, not instantly


def kick_cue(phrase=KICK_CUE_PHRASE):
    """Speak `phrase` without blocking. Any failure is silent and harmless."""
    try:
        subprocess.Popen(["say", phrase],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, ValueError):
        pass                    # no `say` here

_LOG = (("bp", ".5f"), ("vel", ".5f"), ("out", ".4f"), ("gain", ".5f"),
        ("ratio", ".4f"), ("p", ".5f"), ("i", ".5f"), ("d", ".5f"))
# `ctl` marks the row that completed a control step, `n_avg` how many. See csv_row.
CSV_HEADER = "time_s,state,ctl,n_avg," + ",".join(
    f"ch{i}_{c}" for i in range(N)
    for c in ("counts", "V") + tuple(k for k, _ in _LOG) + ("rail", "locked", "healthy"))
# ====================================================


class OnePole:
    """One-pole low/high pass over N independent lanes. `on` is per lane because a
    lane whose D-term filter was reset must re-prime from its first sample rather
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
    """RMS over a time window, scalar or array. The controller keeps one per lane
    so a single channel's window can be cleared without resizing the others."""

    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = window_s, deque(), 0.0

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        # np.maximum is not padding: the sum is incremental, so a transient ageing
        # out leaves a residue of its own magnitude, measured at -4.5e-14 after a
        # 6 V/s kick. sqrt() of that is nan, every comparison against nan is False,
        # and the breaker would stop tripping and the lock detector stop firing.
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
    """DAC writes for all N coils: 0.5 mV deadband, with the CONTROL STEP as the
    throttle. No second write throttle: one on time.time() would race the loop's
    own 10 ms period and drop a semi-random half of the writes. The budget fits,
    8 coils x 100 Hz x ~15 bytes = 12 kB/s against 50 kB/s at 500000 baud (the
    CLAUDE.md item 2c budget, written against 115200's 11.5 kB/s).

    ValueError is deliberately NOT caught: FastDAC raises it for a channel outside
    0..7 or a voltage outside 0..2.5 V, neither reachable since `self.out` is a
    slew-limited walk inside [0.0, 1.5] V, so it would mean a bug in the clip."""

    def __init__(self, dac, channels):
        self.dac, self.ch = dac, list(channels)
        self.t, self.v = np.zeros(len(self.ch)), np.full(len(self.ch), np.nan)

    def send(self, volts):
        now = time.time()
        due = np.isnan(self.v) | (abs(volts - self.v) >= 0.0005)
        for i in np.nonzero(due)[0]:
            try:
                self.dac.set_voltage(channel=self.ch[i], voltage=float(volts[i]))
                self.t[i], self.v[i] = now, volts[i]
            except RuntimeError:
                pass


def in_range(counts):
    """`sample-guard`: a 10-bit converter cannot produce a count outside 0..1023,
    so a row carrying one is a framing error. A free function because it must mean
    the same thing in `read_sample()`, at the wire, and in `Controller.step()`,
    which sim/server.py and harness.py reach without going through the parser."""
    return bool(np.all((counts >= 0) & (counts <= ADC_MAX_COUNTS)))


def read_sample(ser):
    """(counts, volts) as two length-N arrays, or None. arduino.ino streams eight
    columns, so a four-column firmware never satisfies `len(parts) >= N` and this
    returns None forever, which is the loud failure you want. The board's
    `OK ch=.. v=..` replies are nobody's now and arrive interleaved with data rows;
    the first-character test drops those.

    `sample-guard` is the rest: TEN rows in 241,813 of v11's bench log carry a count
    outside 0..1023 (5659, 7575, 65690, 522676, 730608) against ZERO in each of the
    four slower logs, two fields spliced by a buffer boundary so they parse cleanly.
    counts = 522676 is +45.6 V into a bandpass with tau = 0.398 s, ringing for most
    of a second with velocity peaking at 1401 V/s and P demanding 42 V into a 0.5 V
    rail. Replayed on that log the guard takes ch0 rms 13.109 -> 5.413 V/s and peak
    1441 -> 41 V/s, while decimating WITHOUT it gives 23.4 V/s."""
    if not ser.in_waiting:
        return None
    try:
        raw = ser.readline().decode("utf-8").strip()
        if not raw or raw[0] not in "0123456789-":
            return None                    # OK / ERR / STREAMING / junk
        parts = raw.split(",")
        if len(parts) < N:
            return None
        counts = np.array([int(p) for p in parts[:N]], dtype=float)
        if not in_range(counts):
            read_sample.rejected += 1      # torn row: a framing error, not data
            return None
        return counts, counts * (A_VCC / ADC_MAX_COUNTS)
    except (ValueError, IndexError):
        return None


# Counted here, the only place that sees the wire. Ten in 241,813 rows on v11's
# log; a climbing count means the link is losing framing.
read_sample.rejected = 0


# Channel attribute -> Controller array.
_VIEW = dict(zip("enabled healthy bias steady_gain capture_gain ki kd bp_out vel out "
                 "active_gain last_ratio p_term i_term d_term rail_fault locked "
                 "saturated_flag baseline_rms".split(),
                 "enabled healthy bias steady capture ki kd bp vel out "
                 "gain ratio p i d rail locked sat baseline".split()))


class Channel:
    """One lane of the controller's arrays. State lives in length-N arrays, but the
    logger, the status line and sim/server.py read and write
    `ctl.channels[i].<field>`, including live gain edits from the browser UI."""

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
                 capture=None, ki=None, kd=None, bias=None, baseline_file=None):
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
                  "sat_sum rail_sum").split():
            setattr(self, k, np.zeros(N))
        for k in "primed sat locked rail".split():
            setattr(self, k, np.zeros(N, bool))
        self.healthy = np.ones(N, bool)
        # Per-channel output window; the trim moves each channel's bias alone.
        self.vmin, self.vmax = self.bias - BIAS_SWING, self.bias + BIAS_SWING
        self.clear_t, self.excess_since = np.full(N, np.inf), np.full(N, np.inf)
        self.locked_since = np.full(N, np.inf)
        self.out, self.prev_out = self.bias.copy(), self.bias.copy()
        # `sat_hist` is the saturation counterpart of `rail_hist`: (t, pinned) over
        # the last SAT_SUSTAIN_S, a fraction of a window rather than a run.
        self.rail_hist, self.sat_hist = deque(), deque()
        self.calib, self.events = [], []
        self.channels = [Channel(self, i) for i in range(N)]
        self.state, self.calib_start, self.damping_start = "CALIBRATING", 0.0, None
        self.clear_since = self.lock_time = None
        self.filt_on = self.locked_announced = False
        self.fault_count = self.demote_count = self.reuse_count = 0
        self.damped_for = None       # how long DAMPING lasted before the fault
        # ===== fast-calib bookkeeping =====
        self.sub_rms = []            # one RMS vector per COMPLETED sub-window
        self.sub_start, self.sub_n0 = 0.0, 0
        self.calib_took = None
        self.calib_early = False     # converged, or hit the ceiling
        # Kept so a LATER calibration can be checked against it. See _set_baseline.
        self.trusted_baseline = None
        self.baseline_refused = False
        self.refusals = 0
        # ===== warm-restart bookkeeping =====
        self.baseline_t = None       # when the live baseline was measured
        self.fault_railed = False    # was the fault a SENSOR failure?
        # ===== baseline-floor bookkeeping =====
        # Separate from the rail demotion: a rail clears and re-arms on a timer, a
        # signal-less sensor does not, and only a new baseline overturns it.
        self.floor_bad = np.zeros(N, bool)
        self.floor_count = 0
        # ===== bias-trim bookkeeping: trim_ref is the total offset judged against =====
        self.trim_steps = np.zeros(N, int)
        self.trim_frozen = np.zeros(N, bool)
        self.trim_last = None
        self.trim_pending = None       # (channel, previous_bias) awaiting judgement
        self.trim_ref = None           # total |counts - MID| before that step
        self.counts_mean = np.full(N, MID_COUNTS)
        self.trim_step_t = None
        # ===== decimate bookkeeping =====
        # A running SUM and a count, not a list: touched at the WIRE rate, ~1111 Hz,
        # where appending would allocate a thousand times a second.
        self.acc_c, self.acc_v, self.acc_n = np.zeros(N), np.zeros(N), 0
        self.ctl_t = None            # NOMINAL deadline; None = no step run yet
        self.ctl_prev = None
        self.ctl_dt = CONTROL_PERIOD_S
        self.stepped = False
        self.n_avg = 0
        self.ctl_steps = 0
        self.bad_samples = 0         # sample-guard rejections seen here
        self.wire_n, self.wire_t0 = 0, None      # for the measured wire rate
        # ===== persist-baseline bookkeeping =====
        # A loaded floor is held HERE, not in self.baseline: it is not adopted
        # until BASELINE_WARMUP_S has measured something to check it against.
        self.file_baseline = self.file_floor = None
        self.file_note = ""
        if baseline_file is not None:
            self.file_baseline = np.asarray(baseline_file[0], float)
            self.file_floor = np.asarray(baseline_file[1], bool)
        self.warm_used = False       # did this run engage on a file baseline?
        self.baseline_saveable = False   # main() reads and clears this

    # ===== helpers =====
    def drain_events(self):
        out, self.events = self.events, []
        return out

    def _say(self, msg):
        self.events.append("\n" + msg + "\n")

    def _fault(self, t, msg, railed=False):
        self._say("!! " + msg)
        # Only an engagement that happened counts: damping_start read
        # unconditionally reports the PREVIOUS one after a rail during CALIBRATING.
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
        # A full window must have elapsed, or the first samples trip it on n = 1.
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= RAIL_SUSTAIN_S * 0.9
        self.rail = np.logical_and(
            spanned, self.rail_sum / max(len(self.rail_hist), 1) >= RAIL_FRACTION)

    def _sat_check(self, pinned, t):
        """`sat-window`. `_rail_check` with `pinned` for `railed`, deliberately
        line-for-line the same, down to the 0.9 x window `spanned` guard. Maintained
        in EVERY state: in FAULT the outputs sit at bias and the window drains
        within SAT_SUSTAIN_S, so nothing latches."""
        self.sat_hist.append((t, pinned))
        self.sat_sum = self.sat_sum + pinned
        while self.sat_hist and t - self.sat_hist[0][0] > SAT_SUSTAIN_S:
            self.sat_sum = self.sat_sum - self.sat_hist.popleft()[1]
        spanned = bool(self.sat_hist) and t - self.sat_hist[0][0] >= SAT_SUSTAIN_S * 0.9
        self.sat = np.logical_and(
            spanned, self.sat_sum / max(len(self.sat_hist), 1) >= SAT_FRACTION)

    # ===== calibration =====
    def _close_subwindow(self, t):
        """Fold the samples since the last boundary into one RMS vector. The raw
        samples are KEPT as well: the ceiling path re-splits the whole window the
        way v5 does, and must give v5's answer."""
        a = np.asarray(self.calib[self.sub_n0:])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a ** 2).mean(0)))
        self.sub_n0, self.sub_start = len(self.calib), t

    def _calib_stationary(self):
        """Has the noise floor stopped moving? TWO tests, both needed.

        (1) the trailing CALIB_AGREE_N sub-windows agree with each other.
        (2) their median agrees with the median of every sub-window so far. A
            ringdown is slow (tau ~ 16 s, comparable to the whole window), so three
            sub-windows part-way down it look stationary while sitting far from the
            floor: with (1) alone a 6 V/s kick at t = 2 s reads as settled at
            t = 16.01 s (trailing three agree to 1.187) and stores a baseline skewed
            37.0%, past the 35% the suite asserts. Test (2) sees those windows 1.400
            from the median of the whole and runs on, for 17.6%."""
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
        """`early` picks the estimator: the ceiling branch is v5's, median of
        CALIB_SUBWINDOWS sub-window RMSs over the whole window; the early branch
        takes the median of the trailing windows just shown to agree, not of
        everything, the first sub-window still carrying the bandpass start-up
        transient (measured ~8% high)."""
        if early and len(self.sub_rms) >= CALIB_AGREE_N:
            fresh = np.maximum(
                np.median(np.asarray(self.sub_rms[-CALIB_AGREE_N:]), 0), 1e-6)
        else:
            a = np.asarray(self.calib)
            parts = ([p for p in np.array_split(a, CALIB_SUBWINDOWS) if len(p)]
                     if len(a) else [])
            fresh = (
                np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0), 1e-6)
                if parts else np.full(N, 1e-6))

        # Is this a FLOOR, or a ringdown mistaken for one? BENCH 2026-08-06: after
        # a kick and four re-faults a forced calibration exited EARLY at 10.0 s on
        # "sub-windows agreed" while the optic still rang down, measuring
        # ch0 = 0.6112 V against a quiet 0.0530 V, 11.5x (ch1 13.4x, ch2 8.3x,
        # ch3 15.5x, ch5 10.5x). CALIB_AGREE_TOL tests STATIONARITY, not QUIETNESS:
        # at tau ~ 16 s a ringdown decays ~12% across a 2 s sub-window, inside the
        # 1.20 tolerance. Only the HIGH side is refused.
        ref = self.trusted_baseline
        self.baseline_refused = False
        if ref is not None:
            # NOT `~floor_bad`, which belongs to the PREVIOUS calibration since
            # `_baseline_floor` runs after this: measured 2026-08-06, reading it
            # here refused a whole calibration on ch4 (0.0163V vs 0.0028V, 5.9x)
            # and ch6 (0.0664V vs 0.0031V, 21.3x), whose trusted value is noise.
            live = ref > 0
            med = np.median(ref[live]) if live.any() else 0.0
            m = live & (ref >= med * BASELINE_FLOOR_FRAC)
            bad = m & (fresh > BASELINE_SANITY_RATIO * ref)
            # Refusing keeps a floor that may be too LOW, making the breaker
            # hypersensitive, so refuse/fault/recalibrate is a real livelock. A low
            # floor is the SAFE direction, so the bound is generous.
            if bad.any() and self.refusals >= MAX_BASELINE_REFUSALS:
                self._say(
                    f"!! fresh baseline refused {self.refusals} times running and "
                    "accepted anyway -- the floor really has moved, this is no "
                    "longer a ringdown. The runaway breaker is now scaled off it.")
                bad = np.zeros(N, bool)
            if bad.any():
                self.refusals += 1
                self.baseline_refused = True
                self._say(
                    "!! fresh baseline REFUSED -- "
                    + " / ".join(f"ch{i} measured {fresh[i]:.4f}V against a trusted "
                                 f"{ref[i]:.4f}V ({fresh[i] / ref[i]:.1f}x)"
                                 for i in np.nonzero(bad)[0])
                    + f", over {BASELINE_SANITY_RATIO:.1f}x. That is a ringdown, not "
                      "a floor -- a calibration window can be perfectly STATIONARY "
                      "and still sit far above the floor. Keeping the baseline "
                      "already in hand.")
                # Restore EXPLICITLY from `ref`: entering CALIBRATING clears
                # `self.baseline`, and without this it stayed all zeros with the
                # breaker denominator-less (measured 2026-08-06).
                self.baseline = np.maximum(ref, 1e-6)
                self.calib, self.sub_rms, self.sub_n0 = [], [], 0
                return

        self.baseline = fresh
        self.trusted_baseline = fresh.copy()
        self.refusals = 0
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0

    def _start_calibration(self, t):
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_start = self.sub_start = t

    def _warm_from_file(self, t):
        """`persist-baseline`, the run-time half. Preflight has already accepted the
        FILE on schema, age and a fingerprint of every gain, bias, filter corner,
        rate and scale factor. What that cannot answer is whether this is the same
        OPTIC in the same alignment in the same room, so the loop measures for
        BASELINE_WARMUP_S before closing and the loaded floor must agree with what
        it sees, per channel, to within BASELINE_SANITY_RATIO.

        On refusal nothing is thrown away: the warm-up samples stay in `self.calib`,
        so the next step takes the ordinary `fast-calib` path with a head start.
        Channels the FILE demoted stay demoted and are excluded."""
        if t - self.calib_start < BASELINE_WARMUP_S:
            return
        a = np.asarray(self.calib)
        # Trailing 60% only: the first ~0.8 s is the highpass priming itself
        # (tau = 0.398 s), which reads ~8% high over a 2 s window.
        a = a[int(len(a) * 0.4):]
        seen = np.sqrt((a ** 2).mean(0)) if len(a) >= 2 else np.zeros(N)
        loaded, floor = self.file_baseline, self.file_floor
        self.file_baseline = None                 # decided either way, once
        m = ~floor & (loaded > 0)
        ratio = np.where(m, np.maximum(seen, 1e-9) / np.maximum(loaded, 1e-9), 1.0)
        bad = m & ((ratio > BASELINE_SANITY_RATIO)
                   | (ratio < 1.0 / BASELINE_SANITY_RATIO))
        if not m.any() or bad.any():
            self._say("!! stored baseline REFUSED at the warm-up check -- "
                      + (f"{self._who(bad)} measured "
                         + " / ".join(f"{seen[i]:.4f}V against a stored "
                                      f"{loaded[i]:.4f}V ({ratio[i]:.2f}x)"
                                      for i in np.nonzero(bad)[0])
                         if bad.any() else
                         "the file demoted every channel, so there is nothing "
                         "left to check it against")
                      + f", outside {BASELINE_SANITY_RATIO:.1f}x. Something in "
                      f"the rig or the room has moved. Measuring a fresh floor "
                      f"the long way (up to {CALIBRATION_S:.0f}s).")
            return
        self.baseline = np.maximum(loaded, 1e-6)
        # A warm start gets a cold start's guard: this floor is now the reference.
        self.trusted_baseline = self.baseline.copy()
        self.baseline_t = t
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_took, self.calib_early = t - self.calib_start, True
        self.reuse_count = 0
        self.floor_bad = floor.copy()
        self.healthy[floor], self.gain[floor] = False, 0.0
        self.clear_t[floor] = np.inf
        self.warm_used = True
        self.state, self.damping_start = "DAMPING", t
        self.locked_announced = False
        self._say(f"[DAMPING] engaged on the STORED baseline after {self.calib_took:.1f}s "
                  f"of warm-up (a full calibration is {CALIBRATION_S:.0f}s): "
                  + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                  + ". Measured over the warm-up: "
                  + ", ".join(f"ch{i}={seen[i]:.4f}V" for i in np.nonzero(m)[0])
                  + f" -- worst disagreement {np.abs(np.log(ratio[m])).max():.2f} in log, "
                    f"inside the {BASELINE_SANITY_RATIO:.1f}x window."
                  + (f" {self._who(floor)} stays demoted -- the file says they "
                     f"were not measuring the optic." if floor.any() else "")
                  + f" Gain schedules {CAPTURE_GAIN[0]:+.3f} -> {STEADY_GAIN[0]:+.3f}, "
                    f"ramping from zero.")

    def _baseline_floor(self, t):
        """Demote any channel whose freshly-measured baseline is implausibly small
        NEXT TO THE OTHERS. Runs once, at the end of every calibration.

        The failure is a sensor that is powered, reading, not railed and not
        measuring the optic: a flag outside the partial shadow where a shadow sensor
        is linear. Its bandpassed RMS is its own electronics noise, so it calibrates
        near zero, and every downstream test is a RATIO against that number:

            gain schedule    ratio = env / baseline    -> pinned to CAPTURE_GAIN
            lock detector    env < LOCK_RMS_FACTOR x baseline  -> never satisfied
            runaway breaker  env > RUNAWAY_MULTIPLE x baseline -> FAULTS THE RIG

        The third cost ten faults in 145 s on the bench (v5.5, 2026-08-04).

        The median reference (see BASELINE_FLOOR_FRAC) buys a safety property: the
        largest enabled baseline is >= the median > the threshold, so at least one
        enabled channel always survives and no more than half can ever be demoted.
        With most of the rig signal-less it does NOTHING, the median being itself a
        dead channel, which is intended.

        EVERY channel is tested, not just enabled ones: a config-disabled channel is
        still watched by the runaway breaker, and on the bench it was disabled
        channels that faulted the rig."""
        ref = self.baseline[self.enabled]
        if ref.size == 0:
            return
        med = float(np.median(ref))
        bad = self.baseline < med * BASELINE_FLOOR_FRAC
        self.floor_bad = bad
        if not bad.any():
            return
        # Demoted as a rail demotes: unhealthy, gain zeroed, output parked at bias.
        self.healthy[bad], self.gain[bad], self.clear_t[bad] = False, 0.0, np.inf
        self.floor_count += int(bad.sum())
        self._say("!! " + self._who(bad) + " demoted -- baseline "
                  + " / ".join(f"{self.baseline[i]:.4f}" for i in np.nonzero(bad)[0])
                  + f"V, under {BASELINE_FLOOR_FRAC:.0%} of the {med:.4f}V median "
                  f"across enabled channels. That sensor is not measuring the "
                  f"optic, so a ratio against it is a ratio against nothing -- "
                  f"held at bias and out of the interlocks. "
                  f"{int((self.enabled & self.healthy).sum())}/"
                  f"{int(self.enabled.sum())} still damping.")

    def _health(self, t):
        """Demote a railed channel; re-arm REARM_SUSTAIN_S after its rail clears.
        Only RAIL demotes: a saturated channel measures fine and only its output is
        clipped, so parking it would remove authority at peak amplitude. Filters and
        baseline carry across the gap, the short RMS windows do not."""
        drop = self.healthy & self.rail
        self.healthy[drop], self.gain[drop] = False, 0.0   # so gain ramps from zero
        self.clear_t[self.rail] = np.inf
        # `~floor_bad` is what makes `baseline-floor` stick: the re-arm path is
        # written for a rail, an event that ENDS, and a signal-less sensor never
        # railed, so the guard would otherwise hand it back every 2 s forever.
        idle = ~self.healthy & ~self.rail & ~self.floor_bad
        self.clear_t[idle & np.isinf(self.clear_t)] = t
        back = idle & (t - self.clear_t >= REARM_SUSTAIN_S)
        self.healthy[back], self.clear_t[back] = True, np.inf
        for i in np.nonzero(back)[0]:
            self.rms_run[i].reset()
            self.rms_sch[i].reset()
        self.excess_since[back], self.ratio[back] = np.inf, 0.0
        return drop, back

    # ===== the loop =====
    @property
    def wire_hz(self):
        """The MEASURED arrival rate, never a constant here: 12.5, 18, 23.5, 73 and
        ~1111 Hz by coil count and transport, which is why `decimate` exists.
        Fingerprinted into the baseline file."""
        span = (self.wire_t - self.wire_t0) if self.wire_t0 is not None else 0.0
        return (self.wire_n / span) if span > 0.5 else float("nan")

    def step(self, counts, volts, t, dt):
        """`decimate`. Called once per arriving sample, signature and contract
        unchanged, but only the FULL-RATE half runs every time; the control step
        fires on elapsed time and consumes the MEAN of what arrived since the last.

        Full-rate: the accumulator; `counts_mean`, the trim's rest-position EMA,
        whose time constant is in seconds so more samples is more evidence; and
        `_rail_check`, ON RAW COUNTS, because a rail is a fact about a SAMPLE and
        the mean of eleven of which nine are pinned is not pinned.

        `dt`, the WIRE interval from the caller, is used only by `counts_mean`;
        everything downstream gets `ctl_dt`, measured between control steps."""
        counts, volts = np.asarray(counts, float), np.asarray(volts, float)
        self.stepped = False
        # `sample-guard`, second of two places: sim/server.py and harness.py build
        # their own arrays and never go through read_sample(). See in_range.
        if not in_range(counts):
            self.bad_samples += 1
            return self.state
        if self.wire_t0 is None:
            self.wire_t0 = t
        self.wire_t, self.wire_n = t, self.wire_n + 1
        self.acc_c += counts
        self.acc_v += volts
        self.acc_n += 1
        # Where each OSEM RESTS, which is what the trim steers. A 2 s constant is
        # two cycles of the ~1 Hz resonance; maintained in every state incl. FAULT.
        self.counts_mean += (counts - self.counts_mean) * min(1.0, dt / 2.0)
        self._rail_check(counts, t)

        # Accumulate by TIME, never by a fixed sample count, or a baud change
        # becomes a control-rate change. `ctl_t` is a DEADLINE advancing by whole
        # periods: setting it to `t` rounds up to the wire's grid, measured at
        # 92.6 Hz on a 1111 Hz wire, 86.7 Hz on the simulator's 347 Hz and 83 Hz on
        # 500 Hz. No backlog either: on a stall several deadlines pass at once, so
        # re-sync. That branch also runs when the wire is slower than CONTROL_HZ.
        if self.ctl_t is None:
            self.ctl_t, self.ctl_prev = t - CONTROL_PERIOD_S, t - CONTROL_PERIOD_S
        if t < self.ctl_t + CONTROL_PERIOD_S:
            return self.state
        self.ctl_t += CONTROL_PERIOD_S
        if t - self.ctl_t >= CONTROL_PERIOD_S:
            self.ctl_t = t                     # missed deadline(s): re-sync
        n = self.acc_n
        mean_counts, mean_volts = self.acc_c / n, self.acc_v / n
        self.acc_c, self.acc_v, self.acc_n = np.zeros(N), np.zeros(N), 0
        # The REAL interval between control steps, not the nominal period; the
        # upper clamp keeps a wire stall out of the differentiator.
        self.ctl_dt = min(max(t - self.ctl_prev, 1e-4), 0.05)
        self.ctl_prev, self.stepped, self.n_avg = t, True, n
        self.ctl_steps += 1
        return self._control(mean_counts, mean_volts, t, self.ctl_dt)

    def _control(self, counts, volts, t, dt):
        """One control step, on the mean of the samples since the last. Same
        filters, state machine, interlocks and order; only the rail check moved to
        the full-rate half. `counts` and `volts` are FRACTIONAL here, being means,
        and every consumer scales linearly or compares against a threshold."""
        self.bp = self.lp.update(self.hp.update(volts, dt), dt).copy()
        if self.filt_on:
            dv = (self.bp - self.prev_bp) / dt if dt > 0 else np.zeros(N)
            self.vel = self.dsm.update(dv, dt).copy()
        self.prev_bp, self.filt_on = self.bp, True

        if self.state == "CALIBRATING":
            self.calib.append(self.bp)
            if self.rail.any():
                self._fault(t, f"{self._who(self.rail)} railed during calibration -- "
                                f"check alignment.", railed=True)
            elif self.file_baseline is not None:
                self._warm_from_file(t)
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
                        (f"[DAMPING] baseline KEPT after a refused {self.calib_took:.1f}s "
                         "calibration -- these are the numbers already in hand, not "
                         "freshly measured"
                         if self.baseline_refused else
                         f"[DAMPING] baseline set in {self.calib_took:.1f}s "
                         + ("(sub-windows agreed -- stopped early, ceiling is "
                            f"{CALIBRATION_S:.0f}s)" if early
                            else f"(ran the full {CALIBRATION_S:.0f}s ceiling -- the "
                                 "floor never settled, median of sub-windows used)"))
                        + ": "
                        + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                        + f". Gain schedules {CAPTURE_GAIN[0]:+.3f} -> "
                          f"{STEADY_GAIN[0]:+.3f}...")
                    # AFTER the banner: every baseline first, then the verdict.
                    self._baseline_floor(t)
                    # main() does the file I/O, so `harness.py` cannot overwrite it.
                    self.baseline_saveable = True

        elif self.state == "DAMPING":
            self._damping(t, dt)

        elif self.state == "FAULT":
            self.gain[:] = 0.0
            # `sat` is refreshed in _actuate() in every state, so this is a live
            # reading and not a latch: in FAULT it clears on its own.
            if self.rail.any() or self.sat.any():
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = t
            elif t - self.clear_since >= FAULT_CLEAR_SUSTAIN_S:
                self._recover(t)

        self._actuate(t, dt)
        return self.state

    def _moved(self, t, i, new):
        """Apply a bias change to channel `i` and invalidate everything measured
        against the old operating point: both trend interlocks compare the envelope
        against its own past, and a bias step injects a ~1 Hz transient into the
        band they read."""
        self.bias[i] = new
        self.vmin[i], self.vmax[i] = new - BIAS_SWING, new + BIAS_SWING
        self.trim_step_t = t
        self.env_hist.clear()
        self.excess_since[:] = np.inf
        # A bias step moves this channel's whole output WINDOW, so pinned samples
        # recorded against the old one answer a question no longer asked.
        self.sat_hist.clear()
        self.sat_sum[:] = 0.0

    def _trim(self, t):
        """Nudge one channel's bias a quantum toward mid-scale, then judge it.
        DAMPING only and no faster than TRIM_PERIOD_S, a bias step being a DC force
        step the pendulum must ring down from. Judged on the TOTAL offset because
        four coils drive two DOF, so centring a0 can push a2 out; a step that does
        not improve the total is put back and that channel frozen for the run."""
        if self.trim_last is None:
            self.trim_last = t
            return
        if t - self.trim_last < TRIM_PERIOD_S:
            return
        self.trim_last = t
        total = float(np.abs(self.counts_mean - MID_COUNTS).sum())

        if self.trim_pending is not None:          # judge last period's step
            i, was = self.trim_pending
            self.trim_pending = None
            if total >= self.trim_ref:
                self._moved(t, i, was)
                self.trim_frozen[i] = True
                self._say(f"[trim] ch{i} reverted to {was:.2f}V and frozen -- total "
                          f"offset {self.trim_ref:.0f} -> {total:.0f} counts.")
                return
            self._say(f"[trim] ch{i} kept at {self.bias[i]:.2f}V -- total offset "
                      f"{self.trim_ref:.0f} -> {total:.0f} counts.")

        err = self.counts_mean - MID_COUNTS
        elig = (self.enabled & self.healthy & ~self.trim_frozen
                & (self.trim_steps < TRIM_MAX_STEPS)
                & (np.abs(err) > TRIM_DEADBAND_COUNTS))
        if not elig.any():
            return
        i = int(np.argmax(np.where(elig, np.abs(err), -1.0)))
        step = -np.sign(err[i]) * SLOPE_SIGN[i] * BIAS_QUANTUM
        new = float(np.clip(self.bias[i] + step, BIAS_MIN, BIAS_MAX))
        if abs(new - float(self.bias[i])) < 1e-9:   # already against a clamp
            self.trim_frozen[i] = True
            return
        self.trim_pending, self.trim_ref = (i, float(self.bias[i])), total
        self.trim_steps[i] += 1
        was_counts = self.counts_mean[i]
        self._moved(t, i, new)
        self._say(f"[trim] ch{i} rests at {was_counts:.0f} counts "
                  f"({err[i]:+.0f} off mid-scale) -- bias -> {new:.2f}V")

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

        # Gain schedule: capture -> steady blended on amplitude, slew-limited.
        self.ratio = np.where(live, _rms(self.rms_sch, t, self.bp, live) / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)

        # Gated on `healthy` not `live`: a blind channel measures nothing.
        env = _rms(self.rms_run, t, self.bp, self.healthy)
        # `runaway-trend`. A level test cannot tell a runaway from a ringdown, so it
        # trips on a timer after a kick: hardware 2026-08-04 (v8, 240 s, 3 kicks)
        # gave ten faults with the envelope falling through every one, 5.97 -> 4.61,
        # 3.00 -> 3.14, 2.72 -> 2.07, the only survivor at 1.64 under the 1.8 line.
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
        run_trip = self.healthy & (t - self.excess_since >= RUNAWAY_SUSTAIN_S)
        # `soft-saturation`, second half: a clipped output whose envelope falls is
        # winning, so only a flat or rising one is a fault. Same lag as the trend.
        sat_trip = self.healthy & self.sat & ~(env < past * SAT_DECAY_FRAC)
        if run_trip.any():
            return self._fault(t, f"{self._who(run_trip)} runaway -- freezing all "
                                  f"channels, entering FAULT.")
        if sat_trip.any():
            return self._fault(t, f"{self._who(sat_trip)} pinned against the rail, "
                                  f"still demanding more, and not winning -- freezing "
                                  f"all channels, entering FAULT.")

        # A demoted channel flatlines its bandpass, which an RMS-only test reads as
        # the steadiest axis on the rack, so it may neither lock nor block a lock.
        quiet = live & (_rms(self.rms_lock, t, self.bp, live) < self.baseline * LOCK_RMS_FACTOR)
        self.locked_since = self._hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= LOCK_SUSTAIN_S)
        self._trim(t)
        if n and self.locked[live].all() and not self.locked_announced:
            self.lock_time = t - self.damping_start
            self._say(f"*** LOCKED -- {self.lock_time:.1f}s after gain was applied "
                      f"(t={t:.1f}s total)"
                      + ("" if n == n_conf else f" -- DEGRADED, {n}/{n_conf} channels") + " ***")
            self.locked_announced = True

    def _actuate(self, t, dt):
        # Takes `t`: the saturation interlock is a time window, not a count.
        live = self.enabled & self.healthy & (self.state == "DAMPING")
        err = -self.vel                                # setpoint is zero velocity
        self.p = self.gain * err
        self.prev_vel[~self.primed] = self.vel[~self.primed]
        self.primed[:] = True
        # Derivative on the MEASUREMENT, so it cannot kick if the setpoint moves.
        dv = (self.vel - self.prev_vel) / dt if dt > 0 else np.zeros(N)
        self.prev_vel = self.vel.copy()
        self.d = -self.kd * self.dfilt.update(dv, dt)
        # Back-calculation anti-windup: the integrator unwinds at TRACK_TC_S from
        # last sample's clip rather than freezing at a limit.
        self.i = np.clip(self.i + (self.ki * err - self.clip_excess / TRACK_TC_S) * dt,
                         -I_CLAMP_V, I_CLAMP_V)

        raw = self.bias + self.p + self.i + self.d
        clipped = np.clip(raw, self.vmin, self.vmax)
        self.clip_excess = np.where(live, raw - clipped, 0.0)
        # Bumpless re-entry for every lane not actuating: FAULT, config-disabled and
        # demoted alike, so none resumes with a stale integral or a derivative step.
        dead = ~live
        if dead.any():
            self.p[dead] = self.i[dead] = self.d[dead] = self.prev_vel[dead] = 0.0
            self.primed[dead] = False
            self.dfilt.reset(dead)
        cap = MAX_SLEW_PER_S * dt
        target = np.where(live, clipped, self.bias)
        self.out = self.prev_out + np.clip(target - self.prev_out, -cap, cap)
        self.prev_out = self.out.copy()
        # Maintained HERE, so it refreshes in every state including FAULT, which is
        # what stops the v0/v1/v2 saturation latch.
        pinned = (self.out <= self.vmin + 1e-6) | (self.out >= self.vmax - 1e-6)
        # `sat-window`: a fraction of a window in SECONDS, not a run of
        # consecutive iterations. See _sat_check.
        self._sat_check(pinned, t)
        # `soft-saturation`. A clipped loop is weakened, not broken, and the
        # envelope at the trip below answers whether it is achieving anything. NOT
        # clip_excess: anti-windup drives the integrator down until `raw` stops
        # exceeding the rail, so it decays to zero exactly when saturation is worst
        # (the suite caught that as "0 fault(s)").
        self.act.send(self.out)

    def _why_reuse(self, t):
        """Whether the fault path may re-engage on the baseline in hand, and the
        one-line reason. Four cases must RE-MEASURE, each enforced below: nothing
        measured yet; a RAIL fault, so the floor came through a suspect sensor; a
        baseline older than BASELINE_MAX_AGE_S; MAX_BASELINE_REUSE reuses with no
        FAST_REFAULT_S engagement to revalidate it."""
        if not bool(self.baseline.all()):
            return False, "nothing measured yet"
        if self.fault_railed:
            return False, ("the fault was a RAIL -- a floor measured through a "
                           "suspect sensor is suspect with it")
        # A trim step does NOT invalidate the baseline: it is a BANDPASSED RMS and a
        # bias step moves DC, which the bandpass removes. A quantum is ~25-50 counts
        # of 1023 against thresholds like RUNAWAY_MULTIPLE = 1.8.
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
        # Leaving FAULT: back to the baseline in hand, or a full recalibration.
        if self.damped_for is not None and self.damped_for >= FAST_REFAULT_S:
            self.reuse_count = 0        # the loop itself validated this baseline
        reuse, why = self._why_reuse(t)
        keep, keep_t = self.baseline.copy(), self.baseline_t
        keep_floor = self.floor_bad.copy()
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        for k in "baseline gain ratio sat_sum clip_excess p i d prev_vel".split():
            getattr(self, k)[:] = 0.0
        # `sat_sum` is only half of it: its window must go too, or the next
        # SAT_SUSTAIN_S of drain carries pre-fault pinned samples into the loop.
        self.sat_hist.clear()
        self.primed[:] = self.sat[:] = self.locked[:] = False
        self.locked_since[:] = self.excess_since[:] = self.clear_t[:] = np.inf
        self.healthy[:] = True          # a recovery forgets every demotion...
        self.floor_bad[:] = False
        self.dfilt.reset()
        for b in self.rms_sch + self.rms_run + self.rms_lock:
            b.reset()
        if reuse:
            self.reuse_count += 1
            self.baseline, self.baseline_t = keep, keep_t
            # ...except a floor demotion, which travels WITH its baseline: neither
            # was re-measured, and re-arming would restore the zero denominator.
            self.floor_bad = keep_floor
            self.healthy[keep_floor] = False
            self.state, self.damping_start = "DAMPING", t
            self.locked_announced = False
            self._say(f"[recovered] re-engaging on the baseline already in hand -- "
                      f"{why} ({self.reuse_count}/{MAX_BASELINE_REUSE} before a "
                      f"forced re-calibration). Gain ramps from zero."
                      + (f" {self._who(keep_floor)} stays demoted -- same "
                         f"baseline, same verdict." if keep_floor.any() else ""))
        else:
            # A full recalibration is the only thing that can put one back in.
            self.reuse_count = 0
            self.baseline_t, self.damping_start = None, None
            self._start_calibration(t)
            self.state = "CALIBRATING"
            self._say(f"[recovered] all channels clear for {FAULT_CLEAR_SUSTAIN_S:.0f}s "
                      f"-- re-calibrating ({why}) for up to {CALIBRATION_S:.0f}s.")
        self.fault_railed = False
        self.clear_since = self.lock_time = None

    # ===== reporting =====
    def status_line(self, t):
        def one(i):
            if self.floor_bad[i]:
                # Distinct from DOWN: a rail re-arms itself, NOSIG wants a person.
                return f"ch{i}:NOSIG bp={self.bp[i]:+.3f}V base={self.baseline[i]:.4f}V"
            if not self.enabled[i]:
                return f"ch{i}:off  bp={self.bp[i]:+.3f}V"
            if not self.healthy[i]:
                return f"ch{i}:DOWN bp={self.bp[i]:+.3f}V"
            flags = ("LOCK" if self.locked[i] else "....") + ("!RAIL" if self.rail[i] else "")
            return (f"ch{i}:{flags} g={self.gain[i]:+.4f} "
                    f"bp={self.bp[i]:+.3f}V ratio={self.ratio[i]:.2f}")
        # A quietly changed wire rate has invalidated a threshold here twice.
        # `avg` of 1 means the wire is slower than CONTROL_HZ.
        rate = (f"{self.wire_hz:.0f}/{CONTROL_HZ:.0f}Hz avg{self.n_avg:d}"
                if self.wire_hz == self.wire_hz else f"--/{CONTROL_HZ:.0f}Hz")
        bad = f" bad{self.bad_samples}" if self.bad_samples else ""
        return (f"[{t:7.1f}s] {self.state:11s} {rate}{bad} "
                + "  ".join(one(i) for i in range(N)))

    def csv_row(self, t, counts):
        """`healthy` is the 13th column and `rail` cannot replace it: a demotion
        outlasts the rail that caused it by REARM_SUSTAIN_S. Written for EVERY
        accepted sample, not every control step, the raw stream going to disk
        (CLAUDE.md); `ctl` marks the row that completed a control step and `n_avg`
        how many it averaged, so the decimation replays offline from the counts."""
        row = [f"{t:.4f}", self.state,
               "1" if self.stepped else "0",
               str(self.n_avg if self.stepped else 0)]
        for i in range(N):
            row += [str(int(counts[i])), f"{counts[i] * (A_VCC / ADC_MAX_COUNTS):.4f}"]
            row += [format(getattr(self, k)[i], f) for k, f in _LOG]
            row += [str(int(v[i])) for v in (self.rail, self.locked, self.healthy)]
        return ",".join(row)


# ===== persist-baseline: the file =====
# Module level, out of Controller: `harness.py` and `sim/server.py` step it
# thousands of times per suite and would overwrite the bench's measured floor.
def _fingerprint(bias, enable=None, steady=None, capture=None, ki=None, kd=None):
    """Everything that changes what a baseline MEANS. A mismatch REFUSES the file
    rather than adapting: every downstream test divides by the floor, and a wrong
    denominator both desensitises the runaway breaker and makes LOCKED easier to
    declare. `bias` is passed in because `bias-trim` moves it during a run, so what
    is stored is the bias the floor was MEASURED at."""
    g = lambda v, d: [round(float(x), 9) for x in (d if v is None else v)]
    return dict(
        n_channels=int(N),
        enable=[bool(v) for v in (ENABLE_CHANNEL if enable is None else enable)],
        steady=g(steady, STEADY_GAIN), capture=g(capture, CAPTURE_GAIN),
        ki=g(ki, KI_GAIN), kd=g(kd, KD_GAIN), bias=g(bias, BIAS),
        dac_channels=[int(c) for c in DAC_CHANNELS],
        # Move a corner and the number is an RMS of something else.
        bp_low_hz=BP_LOW_HZ, bp_high_hz=BP_HIGH_HZ,
        deriv_smooth_hz=DERIV_SMOOTH_HZ, d_smooth_hz=D_SMOOTH_HZ,
        # The rate the loop averages at, and the counts->volts scale.
        control_hz=CONTROL_HZ, a_vcc=A_VCC, adc_max_counts=int(ADC_MAX_COUNTS))


def _same(a, b, tol=1e-9):
    if isinstance(a, list) != isinstance(b, list):
        return False
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def save_baseline(ctl, path=BASELINE_PATH):
    """Write the floor just measured with the fingerprint it was measured under,
    through a temp file and os.replace. A floor half-written by a Ctrl+C must never
    be loadable: a truncated JSON array would either fail to parse or, worse, parse
    short and hand the loop a baseline of the wrong length."""
    payload = dict(
        schema=BASELINE_FILE_SCHEMA,
        written_by=VERSION_TAG,
        measured_unix=time.time(),
        measured_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        baseline_v=[float(v) for v in ctl.baseline],
        # Travels WITH the baseline: exactly as fresh as the numbers it came from.
        floor_bad=[bool(v) for v in ctl.floor_bad],
        calib_took_s=(None if ctl.calib_took is None else float(ctl.calib_took)),
        calib_early=bool(ctl.calib_early),
        # MEASURED, not configured, so it is compared with a tolerance on load.
        wire_hz=(float(ctl.wire_hz) if ctl.wire_hz == ctl.wire_hz else None),
        config=_fingerprint(ctl.bias, ctl.enabled, ctl.steady, ctl.capture,
                            ctl.ki, ctl.kd))
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
    os.replace(tmp, path)
    return payload


def load_baseline(path=BASELINE_PATH, fresh=False, now=None):
    """-> ((baseline, floor_bad) or None, [lines to print]).

    Every rejection returns a REASON and main() prints all of them before the first
    sample: the file about to run must be the thing you can read, so
    `persist-baseline` is only acceptable if it is loud."""
    say = [f"stored baseline: {path}"]
    if fresh:
        say.append("  IGNORED -- fresh calibration forced (--fresh / "
                   "OSEM_FRESH_CALIB). Measuring the floor the long way.")
        return None, say
    try:
        with open(path) as fh:
            d = json.load(fh)
    except FileNotFoundError:
        say.append(f"  none yet. Calibrating for up to {CALIBRATION_S:.0f}s and "
                   f"writing one at the end.")
        return None, say
    except (OSError, ValueError) as e:
        say.append(f"  UNREADABLE ({e}) -- ignored, calibrating from scratch.")
        return None, say

    def no(why):
        say.append(f"  REFUSED -- {why}. Calibrating from scratch "
                   f"(up to {CALIBRATION_S:.0f}s).")
        return None, say

    if d.get("schema") != BASELINE_FILE_SCHEMA:
        return no(f"schema {d.get('schema')!r}, this build reads "
                  f"{BASELINE_FILE_SCHEMA}")
    age = (time.time() if now is None else now) - float(d.get("measured_unix", 0.0))
    say.append(f"  written by {d.get('written_by', '?')} at "
               f"{d.get('measured_utc', '?')}, {age:.0f}s ago")
    if not (0 <= age <= BASELINE_FILE_MAX_AGE_S):
        return no(f"{age:.0f}s old against a {BASELINE_FILE_MAX_AGE_S:.0f}s limit"
                  if age >= 0 else
                  f"measured {-age:.0f}s in the FUTURE -- the clock moved")
    have, want = d.get("config", {}), _fingerprint(BIAS)
    for k in sorted(want):
        if k not in have:
            return no(f"the file does not record `{k}`")
        if not _same(have[k], want[k]):
            return no(f"`{k}` differs: file {have[k]!r}, this build {want[k]!r}")
    # Measured, not configured, so a ratio rather than equality: a bigger change
    # means a different transport or a different baud.
    fw = d.get("wire_hz")
    b = d.get("baseline_v") or []
    fl = d.get("floor_bad") or [False] * len(b)
    if len(b) != N or len(fl) != N:
        return no(f"{len(b)} baselines and {len(fl)} floor flags, expected {N}")
    if not all(isinstance(x, (int, float)) and np.isfinite(x) and x > 0 for x in b):
        return no("a baseline is zero, negative or not finite")
    say.append("  " + ", ".join(f"ch{i}={v:.4f}V" + ("(NOSIG)" if fl[i] else "")
                                for i, v in enumerate(b)))
    say.append(f"  measured at {('%.0f Hz' % fw) if fw else 'an unrecorded rate'} "
               f"on the wire, {CONTROL_HZ:.0f} Hz control step, calibration took "
               f"{d.get('calib_took_s') or float('nan'):.1f}s")
    say.append(f"  ACCEPTED provisionally. It is adopted only if the first "
               f"{BASELINE_WARMUP_S:.1f}s of live signal agree with it to within "
               f"{BASELINE_SANITY_RATIO:.1f}x, per channel; otherwise this run "
               f"calibrates normally and says so.")
    return (np.array(b, float), np.array(fl, bool)), say


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    fresh = ("--fresh" in argv
             or os.environ.get("OSEM_FRESH_CALIB", "0") not in ("", "0"))
    loaded, report = load_baseline(BASELINE_PATH, fresh=fresh)
    print()
    for line in report:
        print(line)
    print()

    dac = FastDAC(port=PORT)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))
    # The prompt guards against a coil-driving run started by accident, and is
    # meaningless with no terminal: input() then raises EOFError AFTER the biases
    # are applied, leaving a board at bias with nothing driving it.
    if sys.stdin.isatty():
        input("DAC biases set. Press Enter to start fast-lock damping (Ctrl+C to stop)... ")
    else:
        print("DAC biases set. stdin is not a tty -- starting without the prompt.")
    dac.start_stream()
    ctl = Controller(dac, baseline_file=loaded)

    os.makedirs("data", exist_ok=True)
    path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    log = open(path, "w", buffering=1)
    log.write(CSV_HEADER + "\n")
    print(f"Logging to {path}, every raw sample, control step at {CONTROL_HZ:.0f} Hz "
          f"(`ctl`/`n_avg` mark which samples each step averaged).\n\n"
          + (f"[{ctl.state}] warming up for {BASELINE_WARMUP_S:.1f}s and then "
             f"checking the stored floor against it...\n" if loaded is not None else
             f"[{ctl.state}] measuring baseline noise (outputs held at bias, no "
             f"damping yet). This stops as soon as the floor settles, and after "
             f"{CALIBRATION_S:.0f}s at the latest...\n"))

    start = prev = time.time()
    last_status, rows = 0.0, 0
    next_cue = None                 # armed on the first DAMPING sample, not here
    if KICK_CUE_S > 0:
        print(f"[kick-cue] speaking \"{KICK_CUE_PHRASE}\" every {KICK_CUE_S:.0f}s "
              f"once damping starts. Unset OSEM_KICK_CUE to silence it.\n")
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
            # Only ever True after a REAL calibration, never a reuse or a loaded
            # floor, so a stale number cannot refresh its own timestamp.
            if ctl.baseline_saveable:
                ctl.baseline_saveable = False
                try:
                    save_baseline(ctl)
                    print(f"\n[baseline] written to {BASELINE_PATH} -- the next "
                          f"start-up engages in ~{BASELINE_WARMUP_S:.0f}s instead of "
                          f"{ctl.calib_took:.0f}s, if nothing in the fingerprint or "
                          f"the room has moved.\n")
                except OSError as e:
                    print(f"\n[baseline] could NOT be written ({e}) -- the run is "
                          f"unaffected, the next one just calibrates.\n")
            for msg in ctl.drain_events():
                print(msg)
            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                print(ctl.status_line(t))
            # Cue only while DAMPING: kicking during CALIBRATING poisons the floor
            # being measured, and FAULT freezes the gains. The clock is ABSOLUTE:
            # re-arming per DAMPING entry cued 8 times in 240 s on a run with 12
            # faults against the 5 asked for.
            if KICK_CUE_S > 0:
                if next_cue is None:
                    if ctl.state == "DAMPING":
                        next_cue = t + _KICK_CUE_LEAD_S     # arm once, on first damping
                elif t >= next_cue:
                    next_cue += KICK_CUE_S
                    # Skip a cue that came due while faulted: kicking into a
                    # recovery measures the recovery.
                    if ctl.state == "DAMPING":
                        kick_cue()
                        print(f"[kick-cue] t={t:.0f}s -- KICK NOW")
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
        print(f"{rows} raw samples, {ctl.ctl_steps} control steps "
              f"({rows / max(ctl.ctl_steps, 1):.1f} averaged per step), "
              f"wire {ctl.wire_hz:.0f} Hz.")
        # Loud, because a rising count is a LINK problem and no amount of control
        # tuning addresses it. v11's own 240 s log had ten.
        print(f"sample-guard: {read_sample.rejected} torn row(s) rejected at the "
              f"wire, {ctl.bad_samples} at the controller."
              + (" v11 would have fed every one of those straight into the "
                 "bandpass." if read_sample.rejected or ctl.bad_samples else ""))


if __name__ == "__main__":
    main()
