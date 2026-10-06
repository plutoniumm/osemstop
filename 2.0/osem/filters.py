from collections import deque
import numpy as np


class Decimator:

    def __init__(self, control_hz, n, mains_null=True):
        self.hz = float(control_hz)
        self.period = 1.0 / self.hz
        self.n = int(n)
        self.mains_null = bool(mains_null)
        self.acc_c, self.acc_v, self.acc_n = (np.zeros(n), np.zeros(n), 0)
        self.ctl_t = self.ctl_prev = None
        self.prev_c = self.prev_v = None
        self.steps, self.n_avg = (0, 0)
        self.wire_n, self.wire_t0, self.wire_t = (0, None, None)

    def feed(self, counts, volts, t):
        if self.wire_t0 is None:
            self.wire_t0 = t
        self.wire_t, self.wire_n = (t, self.wire_n + 1)
        self.acc_c = self.acc_c + counts
        self.acc_v = self.acc_v + volts
        self.acc_n += 1
        if self.ctl_t is None:
            self.ctl_t = self.ctl_prev = t - self.period
        if t < self.ctl_t + self.period:
            return None
        self.ctl_t += self.period
        if t - self.ctl_t >= self.period:
            self.ctl_t = t
        n = self.acc_n
        mc, mv = (self.acc_c / n, self.acc_v / n)
        self.acc_c, self.acc_v, self.acc_n = (np.zeros(self.n), np.zeros(self.n), 0)
        if self.mains_null:
            pc, pv = (mc, mv) if self.prev_v is None else (self.prev_c, self.prev_v)
            self.prev_c, self.prev_v = (mc, mv)
            mc, mv = (0.5 * (mc + pc), 0.5 * (mv + pv))
        dt = min(max(t - self.ctl_prev, 0.0001), 0.05)
        self.ctl_prev, self.n_avg = (t, n)
        self.steps += 1
        return (mc, mv, dt, n)

    @property
    def wire_hz(self):
        span = self.wire_t - self.wire_t0 if self.wire_t0 is not None else 0.0
        return self.wire_n / span if span > 0.5 else float("nan")


class OnePole:

    def __init__(self, hz, kind="low", n=8):
        self.tau, self.low = (1.0 / (2.0 * np.pi * hz), kind == "low")
        self.y, self.xp, self.on = (np.zeros(n), np.zeros(n), np.zeros(n, bool))

    def update(self, x, dt):
        new = ~self.on
        y0 = np.where(new, x if self.low else 0.0, self.y)
        a = dt / (self.tau + dt) if self.low else self.tau / (self.tau + dt)
        nxt = y0 + a * (x - y0) if self.low else a * (y0 + x - np.where(new, x, self.xp))
        self.y, self.xp = (np.where(new, y0, nxt), x.copy())
        self.on[:] = True
        return self.y

    def reset(self, m=slice(None)):
        self.y[m], self.xp[m], self.on[m] = (0.0, 0.0, False)


class SlidingRMS:

    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = (window_s, deque(), 0.0)

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        return np.sqrt(np.maximum(self.sq, 0.0) / len(self.buf)) if self.buf else self.sq * 0.0

    def reset(self):
        self.buf.clear()
        self.sq = 0.0


class RMSBank:

    def __init__(self, window_s, n):
        self.b = [SlidingRMS(window_s) for _ in range(n)]
        self.n = int(n)

    def update(self, t, x, mask):
        return np.array([self.b[i].update(t, x[i]) if mask[i] else 0.0 for i in range(self.n)])

    def reset(self, mask=None):
        for i in range(self.n):
            if mask is None or mask[i]:
                self.b[i].reset()


class InBand:

    def __init__(self, f, n):
        self.w = 2.0 * np.pi * np.asarray(f, float)
        self.n = int(n)
        self.reset()

    def reset(self):
        nm = len(self.w)
        self.k = 0
        self.sy = np.zeros(self.n)
        self.yc = np.zeros((self.n, nm))
        self.ys = np.zeros((self.n, nm))
        self.c = np.zeros(nm)
        self.s = np.zeros(nm)
        self.cc = np.zeros(nm)
        self.ss = np.zeros(nm)

    def add(self, t, y):
        c, s = (np.cos(self.w * t), np.sin(self.w * t))
        y = np.asarray(y, float)
        self.k += 1
        self.sy += y
        self.yc += y[:, None] * c[None, :]
        self.ys += y[:, None] * s[None, :]
        self.c += c
        self.s += s
        self.cc += c * c
        self.ss += s * s

    def rms(self):
        if self.k < 8:
            return np.zeros(self.n)
        k = float(self.k)
        dc = self.cc - self.c**2 / k
        ds = self.ss - self.s**2 / k
        a = (self.yc - self.sy[:, None] * self.c[None, :] / k) / np.maximum(dc, 1e-12)
        b = (self.ys - self.sy[:, None] * self.s[None, :] / k) / np.maximum(ds, 1e-12)
        good = (dc > 1e-09) & (ds > 1e-09)
        amp2 = np.where(good[None, :], a**2 + b**2, 0.0)
        return np.sqrt(0.5 * amp2.sum(axis=1))
