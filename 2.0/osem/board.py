import glob
import time
import numpy as np


class Board:
    FRAME, SYNC, CMD = (23, b"\xa5\xc3", b"\xa5<")

    def __init__(self, rig, cfg, port=None, oversample=10, corners=2, say=print):
        import serial

        self.say, self.os, self.cr = say, int(oversample), int(corners)
        self.map, self.n, self.max = (list(rig.dac_map), rig.n, cfg.adc_max)
        self.ser = serial.Serial(port or self.find_port(), 115200, timeout=2)
        if b"READY" not in self._wait(b"READY", 6.0):
            raise SystemExit(
                "no READY from the board at 115200 baud. Wrong port, or flash it: `run.py flash`."
            )
        self._ask(b"STOP")
        info = self._ask(b"INFO")
        if b"fw=2" not in info:
            raise SystemExit(
                "board answered %r to INFO, not firmware 2. Flash it: `run.py flash`." % info[:60]
            )
        for cmd, want in (
            (b"ACK 0", b"ack=0"),
            (b"CR %d" % self.cr, b"cr=%d" % self.cr),
            (b"OS %d" % self.os, b"os=%d" % self.os),
            (b"MODE BIN2", b"mode=bin2"),
        ):
            got = self._ask(cmd)
            if want not in got:
                raise SystemExit("board answered %r to %r" % (got[:60], cmd))
        say(
            "[board] ready: binary both ways, %d scans per frame, corners read %dx"
            % (self.os, self.cr)
        )
        self.div = self.os * np.array([self.cr] * 4 + [1] * (8 - 4), float)
        self.buf, self.seq, self.t0, self._rx = (bytearray(), None, None, None)
        self.frames = self.lost = self.bad = self.sent = self.cmd_ok = self.cmd_bad = 0

    @staticmethod
    def find_port():
        found = glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/cu.usbmodem*")
        if len(found) != 1:
            raise SystemExit(
                "need exactly one USB serial board, found %s. Pass --port." % (found or "none")
            )
        return found[0]

    def _wait(self, token, seconds):
        end, got = (time.time() + seconds, b"")
        while time.time() < end and token not in got:
            got += self.ser.read(self.ser.in_waiting or 1)
        return got

    def _ask(self, cmd):
        self.ser.read(self.ser.in_waiting or 0)
        self.ser.write(cmd + b"\n")
        time.sleep(0.25)
        return self.ser.read(self.ser.in_waiting or 1)

    def start(self, volts):
        self.park(volts)
        self.ser.read(self.ser.in_waiting or 0)
        self.ser.write(b"STREAM\n")
        self.t0 = time.perf_counter()

    def read(self):
        if self.ser.in_waiting:
            self.buf += self.ser.read(self.ser.in_waiting)
        b = self.buf
        while True:
            i = b.find(self.SYNC)
            if i < 0:
                del b[: max(len(b) - 1, 0)]
                return None
            if len(b) - i < self.FRAME:
                del b[:i]
                return None
            fr = b[i : i + self.FRAME]
            vals = np.frombuffer(bytes(fr[3:19]), "<u2") / self.div
            if sum(fr[2:22]) & 255 != fr[22] or fr[19] != self.os or vals.max() > self.max:
                self.bad += 1
                del b[: i + 1]
                continue
            del b[: i + self.FRAME]
            if self.seq is not None:
                self.lost += fr[2] - self.seq - 1 & 255
            self.seq = fr[2]
            if self._rx is not None:
                self.cmd_ok += fr[20] - self._rx[0] & 255
                self.cmd_bad += fr[21] - self._rx[1] & 255
            self._rx = (fr[20], fr[21])
            self.frames += 1
            return (time.perf_counter() - self.t0, vals[: self.n])

    def write(self, volts):
        units = np.full(8, 65535, dtype="<u2")
        for i, v in enumerate(volts):
            if not 0.0 <= v <= 2.5:
                raise ValueError("coil %d asked for %.4f V, outside 0..2.5" % (i, v))
            units[self.map[i]] = int(round(float(v) * 10000.0))
        p = units.tobytes()
        x = 0
        for c in p:
            x ^= c
        self.ser.write(self.CMD + p + bytes((sum(p) & 255, x)))
        self.sent += 1

    def park(self, volts):
        for _ in range(3):
            try:
                self.write(volts)
                time.sleep(0.02)
            except (OSError, ValueError) as e:
                self.say("!! park: %r" % e)

    def close(self):
        self.say(
            "[link] %d frames, %d lost, %d bad; %d command frames sent, board accepted %d and rejected %d"
            % (self.frames, self.lost, self.bad, self.sent, self.cmd_ok, self.cmd_bad)
        )
        for step in (
            lambda: self.ser.write(b"STOP\nMODE ASCII\nOS 1\n"),
            lambda: time.sleep(0.1),
            self.ser.close,
        ):
            try:
                step()
            except OSError as e:
                self.say("!! close: %r" % e)


class Sim:
    OFFSET = np.array([560.0, 630.0, 700.0, 680.0, 670.0, 535.0, 590.0, 587.0])

    def __init__(
        self,
        rig,
        seconds,
        wire_hz=400.0,
        seed=0,
        gamma=0.0072,
        mode_rms=20.0,
        noise=1.0,
        a_scale=1.0,
        kicks=(),
        stuck=None,
    ):
        self.rig, self.dt, self.end = (rig, 1.0 / wire_hz, seconds)
        self.rng = np.random.default_rng(seed)
        self.w = 2.0 * np.pi * rig.f_hz
        self.gamma, self.noise = (gamma, noise)
        self.a = rig.a_dc * a_scale
        self.strong = np.linalg.norm(rig.phi, axis=1) > 0.5
        self.drive = 2.0 * self.w * mode_rms * np.sqrt(gamma)
        self.q = mode_rms * self.rng.normal(size=rig.nm)
        self.v = mode_rms * self.w * self.rng.normal(size=rig.nm)
        self.kicks = sorted(kicks)
        self.stuck = stuck or {}
        self.u = rig.bias.copy()
        self.t, self.parked = (0.0, False)

    def read(self):
        if self.t >= self.end:
            raise EOFError
        self.t += self.dt
        du = self.u - self.rig.bias
        eq = self.a @ du
        c, s = (np.cos(self.w * self.dt), np.sin(self.w * self.dt))
        x, v = (self.q - eq, self.v)
        self.q = eq + (c * x + s / self.w * v)
        self.v = (-self.w * s * x + c * v) * np.exp(-2.0 * self.gamma * self.dt)
        self.v += self.drive * np.sqrt(self.dt) * self.rng.normal(size=len(self.w))
        while self.kicks and self.kicks[0][0] <= self.t:
            self.v += self.kicks.pop(0)[1] * self.w * 20.0
        y = self.OFFSET + self.rig.phi @ self.q
        y = y + np.where(self.strong, 0.0, self.rig.dc @ du)
        y = y + self.noise * self.rng.normal(size=len(y))
        for i, (t0, val) in self.stuck.items():
            if self.t >= t0:
                y[i] = val
        return (self.t, np.clip(np.round(y), 0, 1023))

    def write(self, volts):
        self.u = np.asarray(volts, float).copy()

    def park(self, volts):
        self.write(volts)
        self.parked = True

    def close(self):
        pass


class Replay:

    def __init__(self, path, n=8):
        with open(path) as fh:
            head = fh.readline().strip().split(",")
        tcol = head.index("t" if "t" in head else "time_s")
        names = (
            ["a%d" % i for i in range(n)] if "a0" in head else ["ch%d_counts" % i for i in range(n)]
        )
        data = np.loadtxt(
            path, delimiter=",", skiprows=1, usecols=[tcol] + [head.index(k) for k in names]
        )
        self.t, self.c, self.k = (data[:, 0] - data[0, 0], data[:, 1:], 0)

    def read(self):
        if self.k >= len(self.t):
            raise EOFError
        self.k += 1
        return (self.t[self.k - 1], self.c[self.k - 1])

    def write(self, volts):
        pass

    park = write

    def close(self):
        pass
