"""transport -- where samples come from and where volts go.

One protocol, three implementations, so the Controller cannot tell them apart:

    read()        -> (t, counts) | None      None = nothing yet; EOFError = finished
    write(volts)                             one value per SENSOR index
    park(volts)                              every coil to bias, best effort
    close()

    Serial   the board (firmware 2)          Sim      a synthetic plate
    Replay   a recorded CSV, open loop
"""

import time

import numpy as np

import stdlib as sl


class Serial:
    """The Arduino, on firmware 2: checksummed binary frames BOTH ways.

    Samples arrive as the sum of `oversample` ADC scans per frame, so a count is
    a float with ~2 more bits than the 10-bit ADC at 16x. Commands leave as one
    20-byte frame for all eight coils, re-sent every control step: a frame the
    board rejects is corrected 10 ms later, and the board reports how many it
    accepted and rejected in every sample frame, so delivery is MEASURED.

    Why not ASCII, measured 2026-10-05: 7 rows in 469 324 lost a digit on the
    wire (717 -> 7, 595 -> 57) and passed the 0..1023 range check; the sample
    rate fell 352 -> 225 Hz the moment commands arrived; and a `SET` line torn by
    a receive overrun is still parsed and applied.

    Probes the port, the baud AND the sketch. The tree is not evidence about the
    board: firmware 1 answers `ERR` to everything here and this refuses to run.
    """

    FRAME, SYNC, CMD = 23, b"\xa5\xc3", b"\xa5\x3c"

    def __init__(self, rig, cfg, port=None, oversample=16, say=print):
        import pyDAC2
        port = port or self.find_port()
        pyDAC2.resolve_baud(port, log=say)
        self.dac = pyDAC2.FastDAC(port)                  # ASCII handshake, stream off
        self.ser, self.say = self.dac.ser, say
        info = self._ask(b"INFO")
        if b"fw=2" not in info:
            self.dac.close()
            raise SystemExit("board answered %r to INFO -- not firmware 2. Flash it: "
                             "`run.py flash`." % info[:60])
        self.os = int(oversample)
        for cmd, want in ((b"OS %d" % self.os, b"os=%d" % self.os),
                          (b"MODE BIN2", b"mode=bin2")):
            got = self._ask(cmd)
            if want not in got:
                raise SystemExit("board answered %r to %r" % (got[:60], cmd))
        say("  firmware 2: binary both ways, %dx oversampled" % self.os)
        self.map, self.n, self.max = list(rig.dac_map), rig.n, cfg.adc_max
        self.buf, self.seq, self.t0 = bytearray(), None, None
        self.frames = self.lost = self.bad = self.sent = 0
        self.cmd_ok = self.cmd_bad = 0
        self._rx = None                                  # board's (ok, bad) counters

    def _ask(self, cmd):
        self.ser.read(self.ser.in_waiting or 0)
        self.ser.write(cmd + b"\n")
        time.sleep(0.25)
        return self.ser.read(self.ser.in_waiting or 1)

    @staticmethod
    def find_port():
        from serial.tools import list_ports
        found = [p.device for p in list_ports.comports()
                 if "usbmodem" in p.device or "usbserial" in p.device]
        if len(found) != 1:
            raise SystemExit("need exactly one USB serial board, found %s -- pass "
                             "--port" % (found or "none"))
        return found[0]

    def start(self, bias):
        self.park(bias)
        self.ser.read(self.ser.in_waiting or 0)
        self.ser.write(b"STREAM\n")
        self.t0 = time.perf_counter()

    def read(self):
        k = self.ser.in_waiting
        if k:
            self.buf += self.ser.read(k)
        b = self.buf
        while True:
            i = b.find(self.SYNC)
            if i < 0:
                del b[:max(len(b) - 1, 0)]               # sync may straddle two reads
                return None
            if len(b) - i < self.FRAME:
                del b[:i]
                return None
            fr = b[i:i + self.FRAME]
            vals = np.frombuffer(bytes(fr[3:19]), "<u2") / float(fr[19] or 1)
            if (sum(fr[2:22]) & 0xFF != fr[22] or fr[19] != self.os
                    or vals.max() > self.max):
                self.bad += 1
                del b[:i + 1]                            # false sync: rescan
                continue
            del b[:i + self.FRAME]
            if self.seq is not None:
                self.lost += (fr[2] - self.seq - 1) & 0xFF
            self.seq = fr[2]
            if self._rx is not None:
                self.cmd_ok += (fr[20] - self._rx[0]) & 0xFF
                self.cmd_bad += (fr[21] - self._rx[1]) & 0xFF
            self._rx = (fr[20], fr[21])
            self.frames += 1
            return time.perf_counter() - self.t0, vals[:self.n]

    def write(self, volts):
        """All coils in one checksummed frame. `volts` is per SENSOR index."""
        units = np.full(8, 0xFFFF, dtype="<u2")
        for i, v in enumerate(volts):
            if not 0.0 <= v <= 2.5:
                raise ValueError("coil %d asked for %.4f V, outside 0..2.5" % (i, v))
            units[self.map[i]] = int(round(float(v) * 1e4))
        p = units.tobytes()
        x = 0
        for c in p:
            x ^= c
        self.ser.write(self.CMD + p + bytes((sum(p) & 0xFF, x)))
        self.sent += 1

    def park(self, volts):
        for _ in range(3):                               # any one landing is enough
            try:
                self.write(volts)
                time.sleep(0.02)
            except (OSError, ValueError) as e:
                self.say("!! park: %r" % e)

    def report(self):
        return ("link: %d frames, %d lost, %d bad; %d command frames sent, board "
                "accepted %d and rejected %d"
                % (self.frames, self.lost, self.bad, self.sent, self.cmd_ok,
                   self.cmd_bad))

    def close(self):
        self.say(self.report())
        for step in (lambda: self.ser.write(b"STOP\nMODE ASCII\nOS 1\n"),
                     lambda: time.sleep(0.1), self.ser.close):
            try:
                step()
            except OSError as e:
                self.say("!! close: %r" % e)


class Sim:
    """A plate that obeys a Rig: lightly damped modes read through Phi and pushed
    through A, ambient-driven, quantised by a 10-bit ADC.

    It is the harness's plant and NOTHING MORE. It is linear, its A is the
    rig's own A (times `a_scale`), and it has no supply, no shared lines and no
    latency -- so a pass here means the code does what the model says, not that
    the model is the rig. Every pumping run in 1.0 passed its simulator first.
    """

    OFFSET = np.array([560., 630., 700., 680., 670., 535., 590., 587.])

    def __init__(self, rig, seconds, wire_hz=400.0, seed=0, gamma=0.0072,
                 mode_rms=20.0, noise=1.0, a_scale=1.0, kicks=(), stuck=None):
        self.rig, self.dt, self.end = rig, 1.0 / wire_hz, seconds
        self.rng = np.random.default_rng(seed)
        self.w = 2.0 * np.pi * rig.f_hz
        self.gamma, self.noise = gamma, noise
        self.a = rig.a_dc * a_scale                       # the TRUE plant's A
        self.strong = np.linalg.norm(rig.phi, axis=1) > 0.5
        self.drive = 2.0 * self.w * mode_rms * np.sqrt(gamma)   # ambient forcing
        self.q = mode_rms * self.rng.normal(size=rig.nm)        # start stationary
        self.v = mode_rms * self.w * self.rng.normal(size=rig.nm)
        self.kicks = sorted(kicks)                        # [(t, x mode_rms)]
        self.stuck = stuck or {}                          # {channel: (t, counts)}
        self.u = rig.bias.copy()
        self.t, self.parked, self.writes = 0.0, False, 0

    def read(self):
        if self.t >= self.end:
            raise EOFError
        self.t += self.dt
        du = self.u - self.rig.bias
        eq = self.a @ du                                  # static deflection
        c, s = np.cos(self.w * self.dt), np.sin(self.w * self.dt)
        x, v = self.q - eq, self.v
        self.q = eq + (c * x + s / self.w * v)
        self.v = (-self.w * s * x + c * v) * np.exp(-2.0 * self.gamma * self.dt)
        self.v += self.drive * np.sqrt(self.dt) * self.rng.normal(size=len(self.w))
        while self.kicks and self.kicks[0][0] <= self.t:
            self.v += self.kicks.pop(0)[1] * self.w * 20.0
        # Corner sensors read the modes; the rest read their own static coupling.
        y = self.OFFSET + self.rig.phi @ self.q
        y = y + np.where(self.strong, 0.0, self.rig.dc @ du)
        y = y + self.noise * self.rng.normal(size=len(y))
        for i, (t0, val) in self.stuck.items():
            if self.t >= t0:
                y[i] = val
        return self.t, np.clip(np.round(y), 0, 1023)

    def write(self, volts):
        self.u, self.writes = np.asarray(volts, float).copy(), self.writes + 1

    def park(self, volts):
        self.write(volts)
        self.parked = True

    def close(self):
        pass


class Replay:
    """A recorded CSV played back open loop: the estimator on REAL data.

    Reads 1.0's status records (`time_s`, `a0..`), 1.0's fast_lock logs
    (`time_s`, `chN_counts`) and 2.0's own logs (`t`, `a0..`).
    """

    def __init__(self, path, n):
        with open(path) as fh:
            head = fh.readline().strip().split(",")
        tcol = head.index("t" if "t" in head else "time_s")
        names = (["a%d" % i for i in range(n)] if "a0" in head
                 else ["ch%d_counts" % i for i in range(n)])
        data = np.loadtxt(path, delimiter=",", skiprows=1,
                          usecols=[tcol] + [head.index(k) for k in names])
        self.t, self.c, self.k = data[:, 0] - data[0, 0], data[:, 1:], 0

    def read(self):
        if self.k >= len(self.t):
            raise EOFError
        self.k += 1
        return self.t[self.k - 1], self.c[self.k - 1]

    def write(self, volts):
        pass

    park = write

    def close(self):
        pass
