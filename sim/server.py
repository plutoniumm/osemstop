"""
Simulator server -- runs the REAL controller against a simulated plant
======================================================================
This imports whichever `osem.vN.py` harness.py selects and drives its actual `Controller`,
`Channel`, `OnePoleFilter`, `SlidingRMS`, `slew_limit` and `RateLimitedActuator`.
There is no second implementation of the control law anywhere: what the browser
draws is the output of the same code that talks to the hardware. Edit the
controller and the simulation changes with it.

Two things are faked, and only two:

  * `serial` is stubbed before import, because pyDAC imports it at module level
    and there is no port here. Nothing in the stub is ever called.
  * `DACController` is replaced by `FakeDAC`, which validates its arguments the
    same way the real one does and remembers the last voltage written to each
    channel. That held voltage is what the simulated coil pulls on, so the
    controller's own `RateLimitedActuator` throttle and deadband shape the force
    exactly as they would on the bench.
  * the `time` module inside the controller's namespace, so the actuator
    throttle is spaced in SIMULATED milliseconds rather than wall-clock ones.
    See `_SimTimeModule`. Without it a run stepped faster than real time holds a
    stale DAC value and the damping degrades with host speed.

Controller files are never modified to suit the harness -- known bugs included. They are
reproduced here on purpose, and asserted by harness.py, so that fixing them
in a later version is verifiable rather than hopeful.

The plant is a forced damped oscillator per axis, and it is LINEAR apart from
one measured nonlinearity: the ADC. Sensor volts are converted the way the
hardware converts them -- offset to each channel's measured resting count,
rounded to an integer, and hard-clipped to 0..1023 -- and the controller is then
handed those counts and the volts that come back out of them, exactly as
`read_sample()` does on the bench. Because the OSEMs rest well above mid-scale
(see DC_REST_COUNTS), the top rail is reached long before the bottom one, which
is the asymmetry the 2026-08-03 bench runs show. Nothing else is nonlinear: in
particular this does NOT reproduce the instability at Kp = -0.040, and adding
the converter does not bring it any closer. See "What it cannot tell you" in
README.md.

Disturbances reach it by three distinct physical paths, plus an injection
facility for the things a test has to command on purpose:

  * the coherent drive and the seismic force noise, both forces on the mass;
  * an HVAC tone, also a force on the mass, but at a frequency you choose --
    slide it above the 1 Hz resonance and watch the suspension isolate it;
  * mains pickup, which is NOT a force. It is added to the sensor line after the
    optic, so the optic never moves but the controller cannot tell, and pushes
    against motion that is not there;
  * random shocks: Poisson-timed velocity impulses to the whole rigid body, so
    all four OSEMs see one bump with slightly different projections.

Injected on demand (`kick`, `earthquake`, `set_occlusion` -- and over HTTP at
/api/inject), because the three faults the bench found on 2026-08-03 are
otherwise unreachable from here:

  * a kick: one velocity impulse, what `make test` sends on `x`;
  * an earthquake: a transient aimed at named channels, large enough to rail
    their ADCs and pin their actuators -- the saturation breaker needs 30
    CONSECUTIVE pinned samples, so this is the only way to reach it;
  * an occlusion: one channel held at a rail indefinitely, a blocked OSEM flag.

Not run directly -- `harness.py` in the repo root is the entry point. It chooses
which `osem.vN.py` to load and calls `load()` below before anything else.
"""

import json
import math
import os
import random
import sys
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)            # controllers and pyDAC live one level up

# --- stub `serial` so pyDAC imports without hardware -------------------------
if "serial" not in sys.modules:
    _serial = types.ModuleType("serial")

    class _Serial:                      # never instantiated; import-time only
        def __init__(self, *a, **k):
            raise RuntimeError("the simulator does not open serial ports")

    _serial.Serial = _Serial
    sys.modules["serial"] = _serial

sys.path.insert(0, ROOT)                # so the controller finds pyDAC

import importlib.util

# WAS 500.0. The bench streams at ~348 Hz -- 2.88 ms/sample, measured
# 2026-08-03; the Arduino's serial print dominates its loop. The rate matters
# because ONE controller threshold is counted in samples rather than seconds:
# MAX_CONSECUTIVE_SATURATED = 30 meant 60 ms at 500 Hz and means 86 ms here, so
# a simulated saturation trip used to need a 1.4x faster excursion than the
# hardware does. (RAIL_SUSTAIN_S is seconds in every version -- v0's "250
# consecutive samples" is 0.5 s expressed at the old simulator rate, and 174
# samples on the bench. Its wall-clock meaning does not move with the rate.)
SAMPLE_HZ = 347.2                       # 1 / 2.88 ms -- the WIRE rate

# ...but the CONTROL LOOP does not run at the wire rate whenever it is driving
# coils. `pyDAC.set_voltage()` writes SET and then reads up to 50 lines waiting
# for the `OK` ack, and every one of those lines is a stream sample the
# controller never sees. Measured on the 2026-08-03 logs: median inter-sample
# time across a whole run is 2.80 ms (348 Hz, matching the wire), but across
# consecutive DAMPING rows it is 41 ms -- about 24 Hz, a factor of 14.
#
# Until this was modelled the simulator stepped uniformly at SAMPLE_HZ, i.e. it
# simulated a loop 14x faster than the hardware, which flatters every
# timing-dependent result.
#
# `ack_drain` is how many stream samples ONE DAC write costs. It is CALIBRATED,
# not derived: the real cost is a wall-clock round trip (command out, board
# turnaround, ack back, plus whatever stream lines were queued ahead of it), and
# how many samples that eats depends on buffer occupancy. What is known is the
# aggregate -- 24 Hz during DAMPING against a 348 Hz wire -- so the free
# parameter is set to reproduce it. At ~3.7 writes per loop iteration, 3.5 gives
# 25 Hz measured in-sim, against the bench's 24.
#   3.5  reproduces pyDAC.DACController on the bench   (default)
#   0    pyDAC2.FastDAC, which writes and returns without reading the ack
# A plant parameter rather than a constant, so one session can show both.
ACK_DRAIN = 3.5
DT = 1.0 / SAMPLE_HZ
DECIM = 12
WINDOW_S = 40.0
TRACE_N = int(WINDOW_S * SAMPLE_HZ / DECIM)

# Seismic drive noise is a WHITE FORCE, and how finely we sample it must not
# change how hard it shakes the optic. Drawing `seismic * gauss()` once per step
# and integrating it with `* DT` gives a velocity variance per second
# proportional to DT, so simply retiming the loop from 500 Hz to 348 Hz would
# have raised the plant's own noise floor by sqrt(500/348) = 1.20x -- a quieter
# clock making a louder lab, which is a property of the harness and not of the
# suspension. Scaling each draw by sqrt(DT_REF/DT) fixes the spectral DENSITY
# instead of the per-sample amplitude, so the `seismic` slider means the same
# excitation at any rate and the pre-2026-08-03 numbers stay comparable.
# DT_REF is the rate the slider was calibrated at, and nothing else uses it.
#
# `meas_noise` is deliberately NOT scaled: it is sensor noise referred to one
# ADC conversion, the rail interlock compares one conversion against a count
# threshold, and the noise sweep in harness.py is specified in mV per sample.
DT_REF = 1.0 / 500.0
NOISE_SCALE = math.sqrt(DT_REF / DT)

# What a shock delivers, in sensor volts per second.
#
# KICK_DV is the manual one -- the browser button, `x` in the TUI -- and is
# UNCHANGED at 6.0. It exists to watch a loop absorb a bump.
#
# The AUTO kick is a different job: it is the stimulus for the saturation
# scenario, and it has to hold the actuator against its clip for
# MAX_CONSECUTIVE_SATURATED CONSECUTIVE samples. That threshold is counted in
# samples, so at the bench's 348 Hz it means 86 ms where at the old simulated
# 500 Hz it meant 60 ms, and a bare 6.0 V/s impulse under Kp = -0.600 no longer
# reaches it: the gain damps the impulse away inside one half cycle, and the
# actuator's own 2.0 V/s slew limit spends part of that just travelling. WAS a
# 6.0 V/s impulse; it is now a SUSTAINED transient of the same order, which is
# what the bench event actually was -- v2 pinned for 31 samples while being
# driven, not while ringing down. See Sim.earthquake.
KICK_DV = 6.0
AUTO_KICK_DV = 9.0
AUTO_KICK_S = 2.0

X_REST = 2.5                # mechanical zero of the optic, in sensor volts
F0 = 1.0                    # nominal pendulum frequency; MODE_F0 is what runs
W0 = 2.0 * math.pi * F0
SENSOR_MIN_V, SENSOR_MAX_V = 0.0, 5.02

# --- the ADC, as the bench actually presents it -----------------------------
# The OSEMs do NOT rest at mid-scale. Resting counts, from the CALIBRATING
# stretches of the nine bench logs in data/ (gain zero, outputs held at bias),
# averaged over the runs:
#
#     ch0 ~ 615    ch1 ~ 641    ch2 ~ 717    ch3 ~ 679       out of 0..1023
#
# against a mid-scale of 511.5, so every channel sits 100..205 counts HIGH and
# the headroom is badly asymmetric -- ch2 has 306 counts of room upward and 717
# downward. That asymmetry is why the bench clips the TOP rail and effectively
# never the bottom: v1 railed 246 samples (2.19% of the run) and v2 10.40%, all
# at the high end, while neither ever approached 0. Model the offset and that
# behaviour falls out; leave the sensors centred, as this file used to, and no
# amount of amplitude reproduces it.
#
# Provenance note, because these numbers travel: 622 / 632 / 598 has been
# quoted as "ch0 / ch1 / ch2". Those three values are real and they are in
# data/, but they are ch0's mean in three DIFFERENT runs (155449, 155916,
# 160302) -- not three channels of one run. Per channel the effect is the same
# sign and the same order on all four, which is what is used here.
DC_REST_COUNTS = [615.0, 641.0, 717.0, 679.0]

# Populated by load(); every controller version defines these, but not with the
# same values, so nothing here may be computed at import time.
osem = None
SIM = None
CONTROLLER_FILE = None
VERSION = None
COUNTS_PER_VOLT = None
VOLTS_PER_COUNT = None
DC_REST_V = None            # DC_REST_COUNTS in volts, once A_VCC is known

# --- rigid-body geometry ----------------------------------------------------
# Four OSEMs are not four oscillators. They are four sensors watching ONE mass,
# so their readings are projections of the same few degrees of freedom and are
# strongly correlated by construction.
#
# Quadrant layout from provenance.md p1: A1 top-left, A3 top-right, A0 bottom-left,
# A2 bottom-right. Each OSEM measures longitudinal displacement at its own
# corner, so with x = left/right and y = up/down, in units of the corner radius:
GEOM = [(-1.0, -1.0),      # A0  bottom-left
        (-1.0, +1.0),      # A1  top-left
        (+1.0, -1.0),      # A2  bottom-right
        (+1.0, +1.0)]      # A3  top-right

# z_i = LONG + PITCH*y_i + YAW*x_i   (small angles, pitch/yaw as displacement
# at unit radius). Three rigid-body freedoms, four sensors -- so the sensing
# matrix has a fourth row that NO rigid-body motion can produce:
#
#   LONG      = (z0 + z1 + z2 + z3) / 4
#   PITCH     = (z1 + z3 - z0 - z2) / 4      top minus bottom
#   YAW       = (z2 + z3 - z0 - z1) / 4      right minus left
#   BUTTERFLY = (z0 + z3 - z1 - z2) / 4      identically zero for a rigid body
#
# BUTTERFLY is the null space and is worth its weight: any reading in it is a
# sensor disagreeing with the other three, which is a free consistency check
# that does not depend on the rail interlock or on any threshold in counts.
SENSE = [[+0.25, +0.25, +0.25, +0.25],     # LONG
         [-0.25, +0.25, -0.25, +0.25],     # PITCH
         [-0.25, -0.25, +0.25, +0.25],     # YAW
         [+0.25, -0.25, -0.25, +0.25]]     # BUTTERFLY (unactuatable)
MODE_NAMES = ["LONG", "PITCH", "YAW"]

# One suspended mass has one pendulum frequency in longitudinal and separate
# pitch/yaw resonances set by its inertia. They sit close together -- provenance.md
# Fig. 2's ASD shows a single unresolved ~1 Hz feature, not three peaks -- and it
# is that near-degeneracy, not four detuned oscillators, that makes the four
# traces look alike but drift tens of degrees apart in phase.
#
# UNCHANGED, and checked against the bench 2026-08-03 rather than assumed: A0..A3
# showed 2.0-4.2 mean-crossings per second, i.e. 1.0-2.1 Hz with the dominant
# motion at ~1 Hz. All three modes sit inside the controller's BP_LOW_HZ = 0.4 to
# BP_HIGH_HZ = 3.0 bandpass, which is the thing that has to hold: a resonance
# outside that band would be filtered out of the very signal the loop closes on,
# and the sim would show a loop that cannot work on hardware working fine.
MODE_F0 = [1.000, 0.940, 1.070]     # LONG, PITCH, YAW, in Hz
MODE_DRIVE = [1.00, 0.55, 0.40]     # how strongly the ground drive couples in
MODE_AUTHORITY = [1.00, 0.90, 0.90]  # coil force -> modal acceleration

# Lumped loop gain per channel -- sign AND magnitude, relative to ch0. This is
# an INFERENCE from provenance.md, not a measurement; the report characterises no
# actuator. It is derived because assuming all four identical contradicts the
# bench outright.
#
# Sign: ch2 runs at STEADY_GAIN[2] = +0.010, positive where the other three are
# negative (Fig. 2 panel 5), and Table 1 has it damping FASTEST of the four. A
# positive gain on a normally-wired channel pumps the resonance; the only
# reading that fits is that ch2's coil or OSEM is mounted the other way round.
#
# Magnitude: added damping rate = total minus intrinsic, gamma_int = w0/2Q.
# From Table 1, with the simulator's Q = 50 (the report measures no Q, so this
# part inherits that assumption):
#     A0  0.2197 - 0.0628 = 0.1569   at |K| = 0.030
#     A1  0.1821 - 0.0628 = 0.1193   at |K| = 0.030  ->  0.76x of A0
#     A2  0.2796 - 0.0628 = 0.2168   at |K| = 0.010  ->  4.15x of A0
#     A3  0.1982 - 0.0628 = 0.1354   at |K| = 0.030  ->  0.86x of A0
# So ch2 gets ~4x the authority per volt, which is exactly why a gain three
# times smaller still damps it fastest. Fit quality behind these is R^2 =
# 0.65..0.92, so treat one significant figure as the honest precision.
COIL_GAIN = [1.00, 0.76, -4.15, 0.86]

# Mains pickup is one ground loop seen by four preamps, so it is essentially in
# phase across the channels and differs only in how much each one picks up.
HUM_SCALE = [1.00, 0.86, 1.12, 0.94]

STATE_CODE = {"CALIBRATING": 0, "DAMPING": 1, "FAULT": 2, "IDLE": 3}

# Every reset() re-seeds from here, so a scenario is repeatable and two versions
# see the same lab. It is a NAMED constant, not a literal buried in reset(),
# because several of the numbers quoted in versions.md are single-realisation
# statistics -- the calibration-skew sweep especially -- and anything that
# changes how many draws happen per second (the sample rate, for one) re-rolls
# them. Sweep this to find out whether a quoted number is a result or a seed.
SEED = 0x2F6E2B1


class FakeDAC:
    """Stands in for DACController. Same validation, no I/O, no printing."""

    def __init__(self):
        self.held = {c: float(osem.BIAS[i]) for i, c in enumerate(osem.DAC_CHANNELS)}
        self.writes = 0          # drives the ack-drain model; see ACK_DRAIN

    def set_voltage(self, channel, voltage):
        if not (0 <= channel <= 7):
            raise ValueError("Channel must be 0-7")
        if not (0.0 <= voltage <= 2.5):
            raise ValueError("Voltage must be 0.0-2.5 V")
        self.held[channel] = float(voltage)
        self.writes += 1
        return "OK"


class _SimTimeModule:
    """Stands in for the `time` module inside the controller's namespace.

    `RateLimitedActuator.send()` calls `time.time()` directly to throttle DAC
    writes. On the bench that is exactly right -- writes should be spaced in real
    milliseconds. In here it is wrong: the harness steps far faster than real
    time, so hundreds of simulated samples elapse inside one 10 ms wall-clock
    window and the actuator holds a stale voltage. The damping then degrades in
    proportion to how fast the host happens to run, which is a property of the
    harness and not of the controller.

    Rather than edit the controller to take an injectable clock, we swap the
    module object it resolves `time` through. The file on disk stays byte-for-
    byte what runs on the hardware. Everything except `time()` is delegated to
    the real module, and `main()` is never called from here, so nothing else is
    affected.
    """

    def __init__(self, sim):
        self._sim = sim

    def time(self):
        return self._sim.t

    def __getattr__(self, name):
        return getattr(time, name)


def gauss():
    """Unit-variance, roughly Gaussian (Irwin-Hall). Real noise has tails; a
    single uniform draw is hard-bounded and would give thresholds like the rail
    check an artificially crisp pass/fail edge."""
    return (random.random() + random.random() + random.random() - 1.5) * 2.0


class Sim:
    def __init__(self):
        self.lock = threading.RLock()
        self.t = 0.0
        self._drain = 0          # stream samples still being eaten by an ack
        self._dt_lost = 0.0      # their duration, handed to the controller
        self._loop_win = []      # (t, seen_by_controller) over the last 1 s
        self.loop_hz = SAMPLE_HZ
        osem.time = _SimTimeModule(self)     # see the class docstring
        self.plant = dict(ack_drain=ACK_DRAIN,
                          f_drive=1.0, drive_amp=0.8, seismic=0.8, Q=50.0,
                          k_act=20.0, meas_noise=20.0,
                          hum_hz=60.0, hum_mv=0.0,        # electrical, on the sensor
                          hvac_hz=2.5, hvac_amp=0.0,      # mechanical, on the mass
                          shock_rate=0.0, shock_amp=3.0)  # per minute, V/s
        self.occlude = [False] * 4
        self.occlude_high = [False] * 4   # which rail a blinded channel sits on
        # Per-channel counts-per-metre, relative to nominal. NOT a fudge factor:
        # an OSEM is a shadow sensor and is only linear while its flag sits in
        # the partial shadow, so alignment sets how many counts one metre of
        # optic motion produces -- and the bench 2026-08-06 measured that spread
        # directly across the eight OSEMs. a5 reads the 1.046 Hz mode at ~1/12 of
        # a0's counts-per-metre (DC matrix ~13x, passive mode amplitudes ~10.5x,
        # two independent estimates), while a4/a6/a7 sit clean OUTSIDE the shadow
        # and modulate with nothing at all: 0.1-9% of their power in the 0.4-3 Hz
        # loop band, against 77-99% for a0-a3.
        #
        # 1.0 is a normal OSEM. 0.0 is the a4/a6/a7 case -- powered, reading, not
        # sensing the optic, so what reaches the ADC is that channel's own noise
        # about its resting count. This is the failure `auto-disable` cannot see,
        # because such a channel never RAILS: it sits mid-scale and flat.
        self.sens_gain = [1.0] * 4
        self.quake = None                 # sustained injected shake, or None
        self.enable = [bool(v) for v in osem.ENABLE_CHANNEL]
        self.steady = [float(v) for v in osem.STEADY_GAIN]
        self.capture = [float(v) for v in osem.CAPTURE_GAIN]
        self.ki = [float(v) for v in osem.KI_GAIN]
        self.kd = [float(v) for v in osem.KD_GAIN]
        self.speed = 1.0
        self.running = False
        self.auto_kick = False
        self.reset()

    # ---------------- lifecycle ----------------
    def reset(self):
        with self.lock:
            random.seed(SEED)
            # A shock's DIRECTION is drawn from its own stream, not from the one
            # the lab noise runs on. Two reasons, both about being able to
            # believe a measurement made with one:
            #   * harness.py's calibration-skew check compares a shocked run
            #     against an unshocked one and has to note that they are "not an
            #     exact counterfactual", because kick() used to consume two
            #     draws from the shared stream and shift every subsequent noise
            #     sample. On a private stream the two runs are identical
            #     everywhere except the shock, so the difference IS the shock.
            #   * the shared stream's position at a given TIME depends on the
            #     sample rate, so retiming the loop silently re-rolled which way
            #     every scripted shock pointed. Measured: v0's worst-case
            #     baseline skew over the swept shock times moved from 45.3% to
            #     29.3% on that alone, with nothing physical changed.
            self.rng_shock = random.Random(SEED)
            self.dac = FakeDAC()
            self.ctl = osem.Controller(self.dac, enable=self.enable,
                                       steady=self.steady, capture=self.capture,
                                       ki=self.ki, kd=self.kd)
            self.diag = [osem.SlidingRMS(osem.ENVELOPE_WINDOW_S) for _ in range(4)]
            self.diag_ratio = [0.0] * 4
            self.t = 0.0
            self._drain = 0      # stream samples still being eaten by an ack
            self._dt_lost = 0.0  # their duration, handed to the controller
            self._loop_win = []  # (t, seen) over the last 1 s -> loop_hz
            self.loop_hz = SAMPLE_HZ
            self.log = []
            self.trace = []
            self.cursor = 0
            self.decim = 0
            self.blk_min = [float("inf")] * 4
            self.blk_max = [float("-inf")] * 4
            self.blk_shock = 0.0
            self.shocks = 0
            self.last_shock = None
            self.events = []
            self.kick_pending = self.auto_kick
            self.quake = None
            # ADC rail bookkeeping, so "what fraction of this run clipped?" is
            # answerable the same way it is off a bench CSV (v1 2.19%, v2 10.40%).
            self.clip_n = [0] * 4
            self.samp_n = 0

            # Start each MODE on the steady-state orbit of its own forced
            # oscillator, so calibration measures a settled amplitude rather
            # than a startup transient. Holds off resonance too, which matters
            # because the drive frequency is adjustable.
            zeta = 1.0 / (2.0 * self.plant["Q"])
            w = 2.0 * math.pi * self.plant["f_drive"]
            self.q, self.qd = [], []          # modal displacement and velocity
            for m in range(3):
                w0 = 2.0 * math.pi * MODE_F0[m]
                a0 = self.plant["drive_amp"] * MODE_DRIVE[m]
                A = a0 / math.sqrt((w0 * w0 - w * w) ** 2 + (2 * zeta * w0 * w) ** 2)
                phi = math.atan2(2 * zeta * w0 * w, w0 * w0 - w * w)
                self.q.append(-A * math.sin(phi))
                self.qd.append(A * w * math.cos(phi))
            self.x = self.project()           # the four sensor readings
            self.modes = self.modal(self.x)   # LONG, PITCH, YAW, BUTTERFLY

    def project(self):
        """Rigid-body modes -> what each OSEM sees. This is the whole reason the
        four channels are correlated: they are four views of three numbers."""
        return [X_REST + self.q[0] + self.q[1] * GEOM[i][1] + self.q[2] * GEOM[i][0]
                for i in range(4)]

    def modal(self, z):
        """The inverse: four sensor readings -> LONG, PITCH, YAW, BUTTERFLY."""
        d = [zi - X_REST for zi in z]
        return [sum(SENSE[m][i] * d[i] for i in range(4)) for m in range(4)]

    def apply_gains(self):
        """Push live edits onto the controller's own Channel objects."""
        with self.lock:
            for i, ch in enumerate(self.ctl.channels):
                ch.enabled = self.enable[i]
                ch.steady_gain = self.steady[i]
                ch.capture_gain = self.capture[i]
                ch.ki = self.ki[i]
                ch.kd = self.kd[i]

    def kick(self, dv=KICK_DV):
        """A manual shock. Goes through the same bookkeeping as a random one so
        it is marked on the chart and counted -- otherwise pressing the button
        looks like it did nothing."""
        with self.lock:
            # A real bump is off-centre, so it puts energy into pitch and yaw as
            # well as piston -- which is exactly the cross-coupling a per-channel
            # loop has no way to see and a modal loop does. Drawn from the shock
            # stream, so where it points does not depend on how many noise
            # samples happen to have been drawn before it -- see reset().
            self.qd[0] += dv
            self.qd[1] += dv * 0.35 * (2.0 * self.rng_shock.random() - 1.0)
            self.qd[2] += dv * 0.35 * (2.0 * self.rng_shock.random() - 1.0)
            self.shocks += 1
            self.last_shock = (round(self.t, 2), round(dv, 2))
            self.blk_shock = max(self.blk_shock, abs(dv))

    # ---------------- injected disturbances ----------------
    # The bench found three distinct faults on 2026-08-03 and none of them were
    # reachable from here, so none of them had ever been reproduced off the
    # hardware. What each one needs:
    #
    #   saturation  30 CONSECUTIVE samples with the actuator pinned at VMIN or
    #               VMAX (86 ms at the bench rate). Needs a velocity large
    #               enough that |Kp*v| exceeds the 0.25 V authority and STAYS
    #               there -- a big aimed transient.
    #   runaway     a 2 s RMS above RUNAWAY_MULTIPLE x baseline for
    #               RUNAWAY_SUSTAIN_S, WITHOUT pinning the actuator. A wrong
    #               -signed gain does this; the suite already drives it.
    #   rail        ADC counts outside RAIL_LOW/HIGH long enough. A blocked
    #               flag does it indefinitely; a big enough transient does it
    #               in bursts, which is what the bench saw and what v0/v1/v2's
    #               consecutive-sample check misses.

    def aim(self, channels=None):
        """Modal direction that shows up hardest on the named sensors.

        z_i = LONG + PITCH*y_i + YAW*x_i, so pushing along (1, y_i, x_i) is the
        off-centre shove that lands on sensor i -- which is what a real one is;
        nothing hits a suspended mass exactly on its centre of percussion. The
        vector is normalised so one unit of it moves the worst-hit NAMED sensor
        by one unit. Naming all four, or none, collapses to pure LONG.
        """
        if not channels:
            return [1.0, 0.0, 0.0]
        v = [0.0, 0.0, 0.0]
        for i in channels:
            v[0] += 1.0
            v[1] += GEOM[i][1]
            v[2] += GEOM[i][0]
        gain = max(abs(v[0] + v[1] * GEOM[i][1] + v[2] * GEOM[i][0])
                   for i in channels)
        return [c / gain for c in v] if gain > 1e-9 else [1.0, 0.0, 0.0]

    def earthquake(self, channels=None, dv=25.0, duration=0.0):
        """A large transient, optionally aimed and optionally sustained.

        `dv` is the velocity impulse delivered to the worst-hit named channel,
        in sensor volts per second. At the ~1 Hz resonance that is dv/w0 volts
        of displacement, so dv = 25 is about 4 V -- twice the ~2 V of upward
        headroom the measured DC offsets leave, i.e. flat-topped on the way up
        and clean on the way down, the same asymmetry the bench logs show.

        `duration > 0` also shakes the suspension on resonance in the same
        direction for that long, at the force amplitude that HOLDS the impulse's
        amplitude instead of letting it ring down. That is the difference
        between sweeping past a rail and sitting on one: v0/v1/v2's rail check
        needs an unbroken run of railed samples, so only a sustained shake
        reaches it.

        Goes through the same bookkeeping as a kick, so it is marked on the
        chart and counted.
        """
        with self.lock:
            u = self.aim(channels)
            for m in range(3):
                self.qd[m] += dv * u[m]
            if duration > 0:
                # On resonance the steady-state amplitude is F*Q/w0^2, so the
                # force that sustains an amplitude of dv/w0 is dv*w0/Q.
                w0 = 2.0 * math.pi * MODE_F0[0]
                self.quake = dict(until=self.t + float(duration),
                                  hz=MODE_F0[0], proj=u,
                                  amp=abs(dv) * w0 / max(self.plant["Q"], 1e-6))
            self.shocks += 1
            self.last_shock = (round(self.t, 2), round(dv, 2))
            self.blk_shock = max(self.blk_shock, abs(dv))

    def set_occlusion(self, channel, on=True, side="low"):
        """Blind one OSEM: a flag blocking the beam pins the photodiode at the
        bottom of its range (`side="low"`, the failure the interlock was written
        for), a flag swung clear of it pins it at the top. Either way the sensor
        stops reporting where the optic is, and it stays that way until cleared
        -- unlike an earthquake, which passes."""
        with self.lock:
            self.occlude[int(channel)] = bool(on)
            self.occlude_high[int(channel)] = str(side).lower() == "high"

    def inject(self, kind, **kw):
        """One entry point for the browser and for tests. Returns a short
        description of what it did, or raises ValueError on an unknown kind."""
        kind = str(kind).lower()
        if kind == "kick":
            self.kick(float(kw.get("dv", 6.0)))
            return f"kick {float(kw.get('dv', 6.0)):+.1f} V/s"
        if kind in ("quake", "earthquake"):
            ch = kw.get("channels")
            ch = None if ch is None else [int(c) for c in ch]
            dv = float(kw.get("dv", 25.0))
            dur = float(kw.get("duration", 0.0))
            self.earthquake(channels=ch, dv=dv, duration=dur)
            return (f"earthquake {dv:+.1f} V/s on "
                    f"{'all' if not ch else ','.join(map(str, ch))}"
                    + (f" for {dur:.1f}s" if dur > 0 else ""))
        if kind in ("occlude", "occlusion"):
            for c in (kw.get("channels") or []):
                self.set_occlusion(int(c), bool(kw.get("on", True)),
                                   str(kw.get("side", "low")))
            return f"occlude {kw.get('channels')} {kw.get('side', 'low')}"
        if kind in ("sensor", "sens_gain"):
            # Mis-align one OSEM: scale its counts-per-metre. 0.0 is the a4/a6/a7
            # case measured 2026-08-06 -- reading, not sensing, and never railing.
            for c in (kw.get("channels") or []):
                self.sens_gain[int(c)] = float(kw.get("gain", 1.0))
            return f"sens_gain {kw.get('channels')} -> {float(kw.get('gain', 1.0)):.3f}"
        if kind == "clear":
            with self.lock:
                self.occlude = [False] * 4
                self.occlude_high = [False] * 4
                self.sens_gain = [1.0] * 4
                self.quake = None
            return "cleared"
        raise ValueError(f"unknown injection: {kind}")

    # ---------------- one control cycle ----------------
    def _trim_loop_win(self, t):
        w = self._loop_win
        while w and t - w[0][0] > 1.0:
            w.pop(0)
        span = (t - w[0][0]) if w else 0.0
        # Needs a real window before the number means anything; until then report
        # the wire rate rather than a division by ~0.
        self.loop_hz = (sum(1 for _, seen in w if seen) / span
                        if span > 0.2 else SAMPLE_HZ)

    def step(self):
        p = self.plant
        self.t += DT
        t = self.t

        if (self.kick_pending and self.ctl.state == "DAMPING"
                and self.ctl.damping_start is not None
                and t - self.ctl.damping_start > 4.0):
            self.earthquake(dv=AUTO_KICK_DV, duration=AUTO_KICK_S)
            self.kick_pending = False

        # Random shocks: a door, a footfall, a crane on the floor above. Poisson
        # arrivals, so the gaps are exponential and occasionally two land close
        # together -- an evenly spaced impulse train would never test that.
        # One bump moves the whole rigid body, so all four axes get it at once,
        # mostly common-mode with some differential from where it landed.
        if p["shock_rate"] > 0 and random.random() < p["shock_rate"] / 60.0 * DT:
            mag = p["shock_amp"] * (0.35 + 1.65 * random.random())
            if random.random() < 0.5:
                mag = -mag
            self.kick(mag)

        zeta = 1.0 / (2.0 * p["Q"])
        drive = p["drive_amp"] * math.sin(2.0 * math.pi * p["f_drive"] * t)
        # HVAC: a steady tone from plant machinery, coupled in through the floor.
        # It is a force like any other, so the suspension's own transfer function
        # decides what survives -- near 1 Hz it is amplified by Q, well above it
        # the mass cannot follow and it rolls off as (f0/f)^2.
        hvac = p["hvac_amp"] * math.sin(2.0 * math.pi * p["hvac_hz"] * t)
        # An injected earthquake, if one is still running: a force on the mass
        # like the drive, but aimed along the modal direction that lands on the
        # named channels. It expires on its own.
        quake = [0.0, 0.0, 0.0]
        if self.quake is not None:
            if t >= self.quake["until"]:
                self.quake = None
            else:
                s = self.quake["amp"] * math.sin(2.0 * math.pi * self.quake["hz"] * t)
                quake = [s * u for u in self.quake["proj"]]
        noise_v = p["meas_noise"] / 1000.0
        # Mains pickup enters AFTER the optic, on the sensor line. The optic is
        # not moving at 60 Hz; only the measurement of it is.
        hum_v = p["hum_mv"] / 1000.0
        hum_phase = 2.0 * math.pi * p["hum_hz"] * t

        # ---- each coil pushes at ITS OWN CORNER of one rigid body ----
        # A force at a corner is a longitudinal force AND a torque, so every coil
        # necessarily stirs all three modes. That cross-coupling is not a defect
        # in the model, it is the geometry, and it is what four independent
        # per-channel loops cannot see: each one is fighting a plant the other
        # three are also driving.
        f_mode = [0.0, 0.0, 0.0]
        for i in range(4):
            # Coil authority is negative: raising the DAC pushes in -x, which is
            # what makes a NEGATIVE Kp damping. COIL_GAIN flips that for ch2, so
            # its +0.010 damps there and would pump anywhere else.
            held = self.dac.held[osem.DAC_CHANNELS[i]]
            f_i = -COIL_GAIN[i] * p["k_act"] * (held - float(osem.BIAS[i]))
            f_mode[0] += f_i                      # net longitudinal force
            f_mode[1] += f_i * GEOM[i][1]         # torque about the horizontal
            f_mode[2] += f_i * GEOM[i][0]         # torque about the vertical

        # ---- advance the three rigid-body modes ----
        seismic_common = p["seismic"] * gauss()
        for m in range(3):
            w0 = 2.0 * math.pi * MODE_F0[m]
            a = (-w0 * w0 * self.q[m] - 2.0 * zeta * w0 * self.qd[m]
                 + (drive + hvac) * MODE_DRIVE[m] + quake[m]
                 + p["seismic"] * gauss() * NOISE_SCALE * (1.0 if m == 0 else 0.6)
                 + MODE_AUTHORITY[m] * f_mode[m] / (1.0 if m == 0 else 2.0))
            self.qd[m] += a * DT
            self.q[m] += self.qd[m] * DT
        self.x = self.project()
        self.modes = self.modal(self.x)

        counts = [0] * 4
        volts = [0.0] * 4
        self.samp_n += 1
        full_scale = int(osem.ADC_MAX_COUNTS)
        for i in range(4):
            # An occluded OSEM is not reporting the optic at all: a flag across
            # the beam pins the photodiode at the bottom of its range, a flag
            # clear of it at the top, wherever the mass actually is. That is the
            # failure the rail interlock exists to catch, and the one an
            # RMS-only test cannot see -- a blind channel's bandpassed signal
            # flatlines and reads as perfect stability.
            if self.occlude[i]:
                measured = SENSOR_MAX_V if self.occlude_high[i] else SENSOR_MIN_V
            else:
                # Sensor volts about THIS channel's measured resting point, not
                # about mid-scale. See DC_REST_COUNTS: this is what makes the
                # top rail reachable and the bottom one effectively not.
                #
                # `sens_gain` scales the MOTION only, never the resting point --
                # a mis-aligned flag changes counts-per-metre, not where the
                # photodiode sits. At 0.0 the channel still reports a perfectly
                # healthy-looking DC level and a perfectly healthy-looking noise
                # floor; only the optic is missing from it.
                measured = DC_REST_V[i] + self.sens_gain[i] * (self.x[i] - X_REST)
            sig = measured + noise_v * gauss()
            if hum_v:
                sig += hum_v * HUM_SCALE[i] * math.sin(hum_phase)

            # The converter, modelled the way the hardware behaves: integer
            # counts, hard-clipped to 0..1023. The controller then scales those
            # counts back to volts itself, so the loop sees exactly what
            # read_sample() hands it on the bench -- including the ~4.9 mV
            # quantisation step and, at large amplitude, a flat top. This is the
            # ONLY nonlinearity in the plant. It is not a model of the -0.040
            # instability and does not produce one.
            c = int(round(sig * COUNTS_PER_VOLT))
            if c <= 0 or c >= full_scale:
                c = 0 if c <= 0 else full_scale
                self.clip_n[i] += 1
            counts[i] = c
            volts[i] = c * VOLTS_PER_COUNT

            rv = volts[i] - DC_REST_V[i]
            if rv < self.blk_min[i]:
                self.blk_min[i] = rv
            if rv > self.blk_max[i]:
                self.blk_max[i] = rv

        # ---- the real controller, one iteration ---------------------------
        # ...but only if this sample actually reaches it. While pyDAC is waiting
        # for an ack it is consuming stream lines, and the controller sees none
        # of them. The plant has already advanced above, exactly as the real
        # optic keeps moving while the host is blocked in readline().
        if self._drain > 0:
            self._drain -= 1
            self._dt_lost += DT
            self._loop_win.append((t, False))
            self._trim_loop_win(t)
            return self.ctl.state

        dt_eff = DT + self._dt_lost           # the gap the controller really saw
        self._dt_lost = 0.0
        w0 = self.dac.writes
        state = self.ctl.step(counts, volts, t, dt_eff)
        self._drain = int(round((self.dac.writes - w0) * self.plant["ack_drain"]))
        self._loop_win.append((t, True))
        self._trim_loop_win(t)

        evs = self.ctl.drain_events()
        if evs:
            self.events.extend(e.strip() for e in evs if e.strip())
            del self.events[:-40]

        # simulator-only diagnostic: amplitude ratio on EVERY channel, always.
        # The controller's own scheduler RMS updates only on enabled channels
        # while DAMPING, so it cannot be used to compare damped against undamped.
        for i, ch in enumerate(self.ctl.channels):
            r = self.diag[i].update(t, ch.bp_out)
            self.diag_ratio[i] = (r / ch.baseline_rms) if ch.baseline_rms else 0.0

        self.decim += 1
        if self.decim >= DECIM:
            self.decim = 0
            chans = self.ctl.channels
            row = [round(t, 4), STATE_CODE[state]]
            for i in range(4):
                # signal is plotted about each channel's OWN resting point, so
                # the traces stay centred now that they no longer share one.
                row += [round(chans[i].bp_out, 5), round(volts[i] - DC_REST_V[i], 5),
                        round(self.blk_min[i], 5), round(self.blk_max[i], 5),
                        round(self.diag_ratio[i], 4), round(float(chans[i].out), 5)]
                self.blk_min[i] = float("inf")
                self.blk_max[i] = float("-inf")
            row.append(round(self.blk_shock, 3))   # 0 unless a shock landed here
            self.blk_shock = 0.0
            self.trace.append(row)
            self.cursor += 1
            if len(self.trace) > TRACE_N:
                del self.trace[:len(self.trace) - TRACE_N]

            if len(self.log) < 200000:
                lr = ["%.4f" % t, state]
                for i, ch in enumerate(chans):
                    lr += ["%.5f" % volts[i], "%.5f" % ch.bp_out, "%.5f" % ch.vel,
                           "%.4f" % ch.out, "%.5f" % ch.active_gain,
                           "%.4f" % ch.last_ratio, "%.5f" % ch.p_term,
                           "%.5f" % ch.i_term, "%.5f" % ch.d_term,
                           str(int(ch.rail_fault)), str(int(ch.locked))]
                self.log.append(",".join(lr))

    # ---------------- snapshot for the client ----------------
    def snapshot(self, since):
        with self.lock:
            first = self.cursor - len(self.trace)
            start = max(0, since - first)
            rows = self.trace[start:] if since >= first else list(self.trace)
            chans = self.ctl.channels
            status = {
                "version": VERSION,
                "t": round(self.t, 2),
                "state": self.ctl.state if self.running or self.t > 0 else "IDLE",
                "running": self.running,
                "lockTime": (round(self.ctl.lock_time, 1)
                             if self.ctl.lock_time is not None else None),
                "faults": self.ctl.fault_count,
                "wireHz": round(SAMPLE_HZ, 1),
                "loopHz": round(self.loop_hz, 1),
                "ackDrain": round(float(self.plant["ack_drain"]), 1),
                "shocks": self.shocks,
                "modes": [round(v, 4) for v in self.modes],
                "modeNames": MODE_NAMES + ["BUTTERFLY"],
                "lastShock": self.last_shock,
                "quake": (round(self.quake["until"] - self.t, 1)
                          if self.quake else None),
                "events": self.events[-6:],
                "ch": [{
                    "enabled": bool(c.enabled), "locked": bool(c.locked),
                    "rail": bool(c.rail_fault), "occluded": bool(self.occlude[i]),
                    # percentage of this run's samples that hit an ADC end stop.
                    # Directly comparable to the bench: v1 2.19%, v2 10.40%.
                    "clip": round(100.0 * self.clip_n[i] / max(self.samp_n, 1), 2),
                    "gain": round(float(c.active_gain), 5),
                    "p": round(float(c.p_term), 5), "i": round(float(c.i_term), 5),
                    "d": round(float(c.d_term), 5), "out": round(float(c.out), 5),
                    "signal": round(float(self.x[i]), 4),
                    "ratio": round(self.diag_ratio[i], 3),
                    "baseline": (round(float(c.baseline_rms), 5)
                                 if c.baseline_rms else None),
                } for i, c in enumerate(chans)],
            }
            return {"cursor": self.cursor, "rows": rows, "status": status}


def load(controller_file):
    """Load one `osem.vN.py` and build a fresh Sim around it.

    Every version is a separate file with its own constants, so nothing that
    depends on the controller can be computed at import time -- it all happens
    here. Calling load() again swaps versions in place, which is what the
    harness does when it runs the suite across the ladder.
    """
    global osem, SIM, CONTROLLER_FILE, VERSION
    global COUNTS_PER_VOLT, VOLTS_PER_COUNT, DC_REST_V
    CONTROLLER_FILE = os.path.abspath(controller_file)
    VERSION = os.path.basename(CONTROLLER_FILE)[:-3]        # "osem.v0"
    modname = VERSION.replace(".", "_")                     # dots are not legal
    spec = importlib.util.spec_from_file_location(modname, CONTROLLER_FILE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[modname] = mod
    spec.loader.exec_module(mod)        # safe: main() is behind __main__
    osem = mod
    COUNTS_PER_VOLT = osem.ADC_MAX_COUNTS / osem.A_VCC
    VOLTS_PER_COUNT = osem.A_VCC / osem.ADC_MAX_COUNTS
    # Every version declares its own A_VCC and ADC_MAX_COUNTS, so the resting
    # points can only be turned into volts once one of them is loaded.
    DC_REST_V = [c / COUNTS_PER_VOLT for c in DC_REST_COUNTS]
    SIM = Sim()
    return osem


def loop():
    """Real-time pacing. Steps are batched: sleeping per-sample at 500 Hz costs
    more than the control cycle itself."""
    last = time.perf_counter()
    while True:
        now = time.perf_counter()
        with SIM.lock:
            running, speed = SIM.running, SIM.speed
        if not running:
            last = now
            time.sleep(0.01)
            continue
        n = int((now - last) * speed / DT)
        if n <= 0:
            time.sleep(0.001)
            continue
        n = min(n, 6000)
        last += n * DT / speed
        with SIM.lock:
            for _ in range(n):
                SIM.step()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html", "/ui.html"):
            try:
                with open(os.path.join(HERE, "ui.html")) as f:
                    page = f.read()
            except OSError:
                return self._send(404, "sim/ui.html not found", "text/plain")
            return self._send(200, page, "text/html; charset=utf-8")

        if path == "/api/state":
            since = 0
            if "?" in self.path:
                for kv in self.path.split("?", 1)[1].split("&"):
                    if kv.startswith("since="):
                        try:
                            since = int(kv[6:])
                        except ValueError:
                            since = 0
            return self._send(200, json.dumps(SIM.snapshot(since)), "application/json")

        if path == "/api/csv":
            with SIM.lock:
                header = "time_s,state," + ",".join(
                    f"ch{i}_signal,ch{i}_bp,ch{i}_vel,ch{i}_out,ch{i}_gain,ch{i}_ratio,"
                    f"ch{i}_p,ch{i}_i,ch{i}_d,ch{i}_rail,ch{i}_locked" for i in range(4))
                body = header + "\n" + "\n".join(SIM.log) + "\n"
            return self._send(200, body, "text/csv")

        self._send(404, "not found", "text/plain")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            msg = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return self._send(400, '{"error":"bad json"}', "application/json")
        path = self.path.split("?")[0]

        if path == "/api/config":
            with SIM.lock:
                for key in ("enable", "occlude"):
                    if key in msg:
                        setattr(SIM, key, [bool(v) for v in msg[key]])
                if "occludeSide" in msg:
                    SIM.occlude_high = [str(v).lower() == "high"
                                        for v in msg["occludeSide"]]
                for key in ("steady", "capture", "ki", "kd"):
                    if key in msg:
                        setattr(SIM, key, [float(v) for v in msg[key]])
                if "plant" in msg:
                    SIM.plant.update({k: float(v) for k, v in msg["plant"].items()})
                # Promoted out of `plant` because it is the one setting that
                # changes what the simulator IS rather than what the lab is
                # doing: 14 models pyDAC's ack round-trip, 0 models pyDAC2.
                if "ackDrain" in msg:
                    SIM.plant["ack_drain"] = max(0.0, float(msg["ackDrain"]))
                if "speed" in msg:
                    SIM.speed = max(0.25, min(float(msg["speed"]), 12.0))
                if "autoKick" in msg:
                    SIM.auto_kick = bool(msg["autoKick"])
            SIM.apply_gains()
            return self._send(200, '{"ok":true}', "application/json")

        if path == "/api/run":
            act = msg.get("action")
            with SIM.lock:
                if act == "start":
                    SIM.running = True
                elif act == "pause":
                    SIM.running = False
                elif act == "reset":
                    SIM.reset()
                elif act == "kick":
                    SIM.kick()
                elif act in ("quake", "earthquake"):
                    # same facility as /api/inject, reachable from the run row
                    SIM.earthquake(channels=msg.get("channels"),
                                   dv=float(msg.get("dv", 25.0)),
                                   duration=float(msg.get("duration", 0.0)))
            return self._send(200, '{"ok":true}', "application/json")

        if path == "/api/inject":
            # One endpoint for every deliberate disturbance: see Sim.inject.
            #   {"kind":"kick","dv":6}
            #   {"kind":"quake","channels":[0],"dv":25,"duration":3}
            #   {"kind":"occlude","channels":[1],"side":"low","on":true}
            #   {"kind":"clear"}
            kind = msg.pop("kind", None)
            try:
                what = SIM.inject(kind, **msg)
            except (ValueError, TypeError, IndexError) as exc:
                return self._send(400, json.dumps({"error": str(exc)}),
                                  "application/json")
            return self._send(200, json.dumps({"ok": True, "did": what}),
                              "application/json")

        self._send(404, '{"error":"not found"}', "application/json")


DEFAULT_PORT = 8770


def serve(port):
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    except OSError as exc:
        print(f"Could not bind port {port}: {exc}")
        print(f"Something else is already listening. Try: --port {port + 1}")
        return 1
    threading.Thread(target=loop, daemon=True).start()
    print(f"Controller: {os.path.relpath(CONTROLLER_FILE, ROOT)}")
    print(f"Serving http://localhost:{port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


if __name__ == "__main__":
    sys.exit("Run harness.py in the repo root instead -- it picks the version.")
