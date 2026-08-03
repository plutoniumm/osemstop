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

The plant is a forced damped oscillator per axis. There is no converter in the
loop: the signal stays continuous volts end to end. The controller's rail test
is written against ADC counts, so the harness hands it counts as an unrounded
float -- counts are a linear map of the signal, so the trip points are identical
without quantizing anything.

Four disturbances reach it, by three distinct physical paths:

  * the coherent drive and the seismic force noise, both forces on the mass;
  * an HVAC tone, also a force on the mass, but at a frequency you choose --
    slide it above the 1 Hz resonance and watch the suspension isolate it;
  * mains pickup, which is NOT a force. It is added to the sensor line after the
    optic, so the optic never moves but the controller cannot tell, and pushes
    against motion that is not there;
  * random shocks: Poisson-timed velocity impulses to the whole rigid body, so
    all four OSEMs see one bump with slightly different projections.

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

SAMPLE_HZ = 500.0
DT = 1.0 / SAMPLE_HZ
DECIM = 12
WINDOW_S = 40.0
TRACE_N = int(WINDOW_S * SAMPLE_HZ / DECIM)

X_REST = 2.5
F0 = 1.0
W0 = 2.0 * math.pi * F0
SENSOR_MIN_V, SENSOR_MAX_V = 0.0, 5.02

# Populated by load(); every controller version defines these, but not with the
# same values, so nothing here may be computed at import time.
osem = None
SIM = None
CONTROLLER_FILE = None
VERSION = None
COUNTS_PER_VOLT = None

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


class FakeDAC:
    """Stands in for DACController. Same validation, no I/O, no printing."""

    def __init__(self):
        self.held = {c: float(osem.BIAS[i]) for i, c in enumerate(osem.DAC_CHANNELS)}

    def set_voltage(self, channel, voltage):
        if not (0 <= channel <= 7):
            raise ValueError("Channel must be 0-7")
        if not (0.0 <= voltage <= 2.5):
            raise ValueError("Voltage must be 0.0-2.5 V")
        self.held[channel] = float(voltage)
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
        osem.time = _SimTimeModule(self)     # see the class docstring
        self.plant = dict(f_drive=1.0, drive_amp=0.8, seismic=0.8, Q=50.0,
                          k_act=20.0, meas_noise=20.0,
                          hum_hz=60.0, hum_mv=0.0,        # electrical, on the sensor
                          hvac_hz=2.5, hvac_amp=0.0,      # mechanical, on the mass
                          shock_rate=0.0, shock_amp=3.0)  # per minute, V/s
        self.occlude = [False] * 4
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
            random.seed(0x2F6E2B1)
            self.dac = FakeDAC()
            self.ctl = osem.Controller(self.dac, enable=self.enable,
                                       steady=self.steady, capture=self.capture,
                                       ki=self.ki, kd=self.kd)
            self.diag = [osem.SlidingRMS(osem.ENVELOPE_WINDOW_S) for _ in range(4)]
            self.diag_ratio = [0.0] * 4
            self.t = 0.0
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

    def kick(self, dv=6.0):
        """A manual shock. Goes through the same bookkeeping as a random one so
        it is marked on the chart and counted -- otherwise pressing the button
        looks like it did nothing."""
        with self.lock:
            # A real bump is off-centre, so it puts energy into pitch and yaw as
            # well as piston -- which is exactly the cross-coupling a per-channel
            # loop has no way to see and a modal loop does.
            self.qd[0] += dv
            self.qd[1] += dv * 0.35 * (2.0 * random.random() - 1.0)
            self.qd[2] += dv * 0.35 * (2.0 * random.random() - 1.0)
            self.shocks += 1
            self.last_shock = (round(self.t, 2), round(dv, 2))
            self.blk_shock = max(self.blk_shock, abs(dv))

    # ---------------- one control cycle ----------------
    def step(self):
        p = self.plant
        self.t += DT
        t = self.t

        if (self.kick_pending and self.ctl.state == "DAMPING"
                and self.ctl.damping_start is not None
                and t - self.ctl.damping_start > 4.0):
            self.kick(6.0)
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
                 + (drive + hvac) * MODE_DRIVE[m]
                 + p["seismic"] * gauss() * (1.0 if m == 0 else 0.6)
                 + MODE_AUTHORITY[m] * f_mode[m] / (1.0 if m == 0 else 2.0))
            self.qd[m] += a * DT
            self.q[m] += self.qd[m] * DT
        self.x = self.project()
        self.modes = self.modal(self.x)

        counts = [0.0] * 4
        volts = [0.0] * 4
        for i in range(4):
            # An occluded OSEM sees no light: its output sits at the bottom of
            # the sensor range wherever the optic actually is. That is the
            # failure the rail interlock exists to catch, and the one an
            # RMS-only test cannot see -- a blind channel's bandpassed signal
            # flatlines and reads as perfect stability.
            measured = 0.0 if self.occlude[i] else self.x[i]
            sig = measured + noise_v * gauss()
            if hum_v:
                sig += hum_v * HUM_SCALE[i] * math.sin(hum_phase)
            sig = min(max(sig, SENSOR_MIN_V), SENSOR_MAX_V)
            volts[i] = sig
            counts[i] = sig * COUNTS_PER_VOLT     # unrounded: no quantizer here

            rv = sig - X_REST
            if rv < self.blk_min[i]:
                self.blk_min[i] = rv
            if rv > self.blk_max[i]:
                self.blk_max[i] = rv

        # ---- the real controller, one iteration ----
        state = self.ctl.step(counts, volts, t, DT)

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
                row += [round(chans[i].bp_out, 5), round(volts[i] - X_REST, 5),
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
                "shocks": self.shocks,
                "modes": [round(v, 4) for v in self.modes],
                "modeNames": MODE_NAMES + ["BUTTERFLY"],
                "lastShock": self.last_shock,
                "events": self.events[-6:],
                "ch": [{
                    "enabled": bool(c.enabled), "locked": bool(c.locked),
                    "rail": bool(c.rail_fault), "occluded": bool(self.occlude[i]),
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
    global osem, SIM, CONTROLLER_FILE, VERSION, COUNTS_PER_VOLT
    CONTROLLER_FILE = os.path.abspath(controller_file)
    VERSION = os.path.basename(CONTROLLER_FILE)[:-3]        # "osem.v0"
    modname = VERSION.replace(".", "_")                     # dots are not legal
    spec = importlib.util.spec_from_file_location(modname, CONTROLLER_FILE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[modname] = mod
    spec.loader.exec_module(mod)        # safe: main() is behind __main__
    osem = mod
    COUNTS_PER_VOLT = osem.ADC_MAX_COUNTS / osem.A_VCC
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
                for key in ("steady", "capture", "ki", "kd"):
                    if key in msg:
                        setattr(SIM, key, [float(v) for v in msg[key]])
                if "plant" in msg:
                    SIM.plant.update({k: float(v) for k, v in msg["plant"].items()})
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
            return self._send(200, '{"ok":true}', "application/json")

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
