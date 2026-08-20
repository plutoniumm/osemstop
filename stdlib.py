"""stdlib -- the shared machinery every osem.* controller used to carry its own copy of.

Contract (stable; other agents read it):

    transport   open_dac  park  Actuator  SampleGuard
    rate        Decimator
    filters     OnePole  SlidingRMS  RMSBank  InBand
    estimators  KalmanVelocity  ModalKalman  Chi2Log
    modal       Modal          (geometric or measured Phi, A, validate, sense,
                                project, allocate, box_args)
    allocation  cap_scale      one scalar on the whole modal vector: conservative,
                               proven, and what every rung ships today
                cap_box        the same total cap solved per coil as a box QP.
                               NOT wired to anything -- a caller must ask for it
                               and pass `Modal.box_args`'s metric
    interlocks  Health  Breaker
    law         PID
    floor       Baseline  fingerprint  same  baseline_payload
                save_baseline  load_baseline
    io          Recorder  Console
    skeleton    Loop  share

Nothing here reads a controller's constants: every threshold is a constructor
argument, so a version can change one without touching this file.
`share(cls, owner, "a b c")` is how a controller exposes a component's arrays
under its own names -- the per-channel view and the test harness both index them
directly. `--selftest` covers the pure-math pieces.
"""

import json
import os
from collections import deque
from datetime import datetime, timezone

import numpy as np


# ===========================================================================
# transport
# ===========================================================================
def open_dac(port, log=print, probe=True, **kw):
    """FastDAC on `port`, baud resolved against the board first."""
    import pyDAC2
    if probe:
        pyDAC2.resolve_baud(port, log=log)      # sets pyDAC2.BAUD; owned there
    return pyDAC2.FastDAC(port=port, **kw)


def park(dac, channels, volts):
    """Every coil to its bias. Best effort: a closing port must not raise."""
    for c, v in zip(channels, volts):
        try:
            dac.set_voltage(channel=int(c), voltage=float(v))
        except (RuntimeError, ValueError, OSError):
            pass


class Actuator:
    """Writes only channels that moved by more than `deadband`."""

    def __init__(self, dac, channels, deadband=0.0005):
        self.dac, self.ch, self.deadband = dac, list(channels), float(deadband)
        self.v = np.full(len(self.ch), np.nan)

    def send(self, volts):
        due = np.isnan(self.v) | (np.abs(volts - self.v) >= self.deadband)
        for i in np.nonzero(due)[0]:
            try:
                self.dac.set_voltage(channel=self.ch[i], voltage=float(volts[i]))
                self.v[i] = volts[i]
            except RuntimeError:
                pass


class SampleGuard:
    """One row off the wire, range-checked. ASCII or binary.

    THE RANGE CHECK IS THE POINT: the wire itself does not check. v11's 240 s log
    carried ten rows with counts outside 0..1023 (5659, 65690, 522676), and one is
    worth ~1500 V/s of velocity into a loop whose whole scale is under 1 V/s.

    THE BINARY PATH WAS MISSING UNTIL 2026-08-20, AND THAT WAS A REAL BUG.
    `FastDAC` has framed binary since 2026-08-06, but this class took a raw
    `serial.Serial` and called `readline().decode("utf-8")`, so a binary transport
    left the controller decoding frames as text: `read` returned None forever, the
    CSV got a header and no rows, and the run printed nothing -- the fourth
    header-only recording in this repo's history. The fix is DELEGATION, not a
    second decoder: `read` takes the DAC and hands binary frames to
    `FastDAC.read_sample`, so the transport cannot disagree with the reader again.
    """

    def __init__(self, n, vcc, max_counts):
        self.n, self.vcc, self.max = int(n), float(vcc), int(max_counts)
        self.scale = self.vcc / self.max
        self.rejected = 0

    def in_range(self, counts):
        return bool(np.all((counts >= 0) & (counts <= self.max)))

    def read(self, dac):
        """(counts, volts) or None. None is normal: acks, partial rows, junk.

        Accepts a FastDAC, or a bare `serial.Serial` for the ASCII callers that
        predate this -- `getattr(dac, "binary", False)` is the whole test, and a
        Serial has no such attribute.
        """
        if getattr(dac, "binary", False):
            row = dac.read_sample(self.n)
            if row is None:
                return None
            counts = np.asarray(row, dtype=float)
            # The frame checksum already rejects a torn frame; range-check anyway,
            # because this guarantee must not depend on the transport in use.
            if not self.in_range(counts):
                self.rejected += 1
                return None
            return counts, counts * self.scale

        ser = getattr(dac, "ser", dac)
        if not ser.in_waiting:
            return None
        try:
            raw = ser.readline().decode("utf-8").strip()
            if not raw or raw[0] not in "0123456789-":
                return None                      # OK / ERR / STREAMING / junk
            parts = raw.split(",")
            if len(parts) < self.n:
                return None
            counts = np.array([int(p) for p in parts[:self.n]], dtype=float)
            if not self.in_range(counts):
                self.rejected += 1               # torn row: framing, not data
                return None
            return counts, counts * self.scale
        except (ValueError, IndexError):
            return None


# ===========================================================================
# rate: wire -> control clock
# ===========================================================================
class Decimator:
    """Averages every sample that arrived since the last control step.

    The control rate has never been a control decision in this repo -- it was
    the arrival rate, and that has been 12.5 to 1113 Hz depending on transport.
    Averaging rather than sub-sampling is the sqrt(n) the change exists for.
    `mains_null` is a two-step boxcar: at 100 Hz control it is a null at 50 Hz.
    """

    def __init__(self, control_hz, n, mains_null=True):
        self.hz = float(control_hz)
        self.period = 1.0 / self.hz
        self.n = int(n)
        self.mains_null = bool(mains_null)
        self.acc_c, self.acc_v, self.acc_n = np.zeros(n), np.zeros(n), 0
        self.ctl_t = self.ctl_prev = None
        self.prev_c = self.prev_v = None
        self.steps, self.n_avg = 0, 0
        self.wire_n, self.wire_t0, self.wire_t = 0, None, None

    def feed(self, counts, volts, t):
        """None, or (mean_counts, mean_volts, dt, n_avg) when a step is due."""
        if self.wire_t0 is None:
            self.wire_t0 = t
        self.wire_t, self.wire_n = t, self.wire_n + 1
        self.acc_c = self.acc_c + counts
        self.acc_v = self.acc_v + volts
        self.acc_n += 1
        if self.ctl_t is None:
            self.ctl_t = self.ctl_prev = t - self.period
        if t < self.ctl_t + self.period:
            return None
        self.ctl_t += self.period
        if t - self.ctl_t >= self.period:
            self.ctl_t = t                       # missed deadline(s): re-sync
        n = self.acc_n
        mc, mv = self.acc_c / n, self.acc_v / n
        self.acc_c, self.acc_v, self.acc_n = np.zeros(self.n), np.zeros(self.n), 0
        if self.mains_null:
            pc, pv = (mc, mv) if self.prev_v is None else (self.prev_c, self.prev_v)
            self.prev_c, self.prev_v = mc, mv
            mc, mv = 0.5 * (mc + pc), 0.5 * (mv + pv)
        dt = min(max(t - self.ctl_prev, 1e-4), 0.05)
        self.ctl_prev, self.n_avg = t, n
        self.steps += 1
        return mc, mv, dt, n

    @property
    def wire_hz(self):
        span = (self.wire_t - self.wire_t0) if self.wire_t0 is not None else 0.0
        return (self.wire_n / span) if span > 0.5 else float("nan")


# ===========================================================================
# filters
# ===========================================================================
class OnePole:
    """First-order low- or high-pass, per channel, dt taken per call."""

    def __init__(self, hz, kind="low", n=8):
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
    """RMS over a wall-clock window, running sum, scalar or vector."""

    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = window_s, deque(), 0.0

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        return (np.sqrt(np.maximum(self.sq, 0.0) / len(self.buf)) if self.buf
                else self.sq * 0.0)

    def reset(self):
        self.buf.clear()
        self.sq = 0.0


class RMSBank:
    """One SlidingRMS per channel; only masked channels are fed."""

    def __init__(self, window_s, n):
        self.b = [SlidingRMS(window_s) for _ in range(n)]
        self.n = int(n)

    def update(self, t, x, mask):
        return np.array([self.b[i].update(t, x[i]) if mask[i] else 0.0
                         for i in range(self.n)])

    def reset(self, mask=None):
        for i in range(self.n):
            if mask is None or mask[i]:
                self.b[i].reset()


class InBand:
    """Per-channel in-band amplitude: lock-in at the measured mode frequencies.

    Why not the estimator's own displacement: measured leakage of a 1 V tone
    into `KalmanVelocity.displacement()` is 0.16-0.44 V at 6.19 Hz and
    0.38-0.85 V at 3 Hz, and it is 30x smaller on the quiet channels than the
    loud ones -- so a median of that statistic is set by whoever has the most
    out-of-band interference. A DFT bin at each f_m is brick-wall by
    comparison, and the frequencies are known to 0.001 Hz (versions.md,
    2026-08-17). The mean is removed in covariance form so DC cannot leak.
    """

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
        c, s = np.cos(self.w * t), np.sin(self.w * t)
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
        """sqrt(sum_m amp_m^2 / 2) per channel; zeros before enough samples."""
        if self.k < 8:
            return np.zeros(self.n)
        k = float(self.k)
        # Regression of y on cos/sin with the mean projected out of both.
        dc = self.cc - self.c ** 2 / k
        ds = self.ss - self.s ** 2 / k
        a = (self.yc - self.sy[:, None] * self.c[None, :] / k) / np.maximum(dc, 1e-12)
        b = (self.ys - self.sy[:, None] * self.s[None, :] / k) / np.maximum(ds, 1e-12)
        good = (dc > 1e-9) & (ds > 1e-9)
        amp2 = np.where(good[None, :], a ** 2 + b ** 2, 0.0)
        return np.sqrt(0.5 * amp2.sum(axis=1))


# ===========================================================================
# estimators
# ===========================================================================
def _osc_blocks(Phi, w, dt):
    """Exact-ZOH 2x2 block per UNDAMPED mode, written into `Phi` in place.

    The one piece of math `steady_state_k`, `KalmanVelocity` and `ModalKalman`
    must agree on exactly; the selftest checks 1e5 steps conserve oscillator
    energy, where Euler at the same dt drifts.
    """
    for m, wm in enumerate(w):
        th = wm * dt
        c, s = np.cos(th), np.sin(th)
        Phi[2 * m:2 * m + 2, 2 * m:2 * m + 2] = [[c, s / wm], [-wm * s, c]]
    return Phi


def steady_state_k(R, Q, q_dc, f, dt, iters=5000, tol=1e-14):
    """DIAGNOSTIC ONLY: steady-state Kalman gain for KalmanVelocity's model.

    NOTHING IN THE CONTROL PATH CALLS THIS. The rungs ship `KALMAN_K` as literals;
    this exists so a session can check them against the (R, Q, f, dt) they are
    supposed to belong to -- a steady-state gain is correct for exactly one such
    tuple, `F_MODE_HZ` is re-measured every session, and nothing in the loop
    notices when the gain stops being correct: the filter keeps returning a
    number, it is simply no longer velocity.

    MEASURED 2026-08-20, AND OPEN: the derived gain disagrees with the shipped
    literals by up to 51x, yet over 41 313 control steps of
    data/20260820_173738_fast_lock.csv the two velocities correlate with a
    zero-phase reference at 0.597/0.710/0.747/0.749 shipped against
    0.519/0.723/0.706/0.748 derived. Do not "fix" either against the other.

    The model, per mode: an UNDAMPED oscillator with continuous white-noise
    acceleration of density q, discretised exactly, Q_d = [[dt^3/3, dt^2/2],
    [dt^2/2, dt]] * q. The DC state is a random walk of variance q_dc per step;
    the measurement is displacement plus DC. Iterates the Riccati recursion so it
    needs no scipy; `iters` is a bound and the loop exits on `tol`.
    """
    R = np.atleast_1d(np.asarray(R, float))
    Q = np.atleast_2d(np.asarray(Q, float))
    q_dc = np.atleast_1d(np.asarray(q_dc, float))
    f = np.asarray(f, float)
    nm, n = len(f), len(R)
    nx = 2 * nm + 1

    Phi = _osc_blocks(np.zeros((nx, nx)), 2.0 * np.pi * f, dt)
    Phi[-1, -1] = 1.0
    H = np.zeros((1, nx))
    H[0, 0:2 * nm:2] = 1.0
    H[0, -1] = 1.0

    K = np.zeros((n, nx))
    for i in range(n):
        Qd = np.zeros((nx, nx))
        for m in range(nm):
            q = Q[i, m]
            Qd[2 * m, 2 * m] = q * dt ** 3 / 3.0
            Qd[2 * m, 2 * m + 1] = Qd[2 * m + 1, 2 * m] = q * dt ** 2 / 2.0
            Qd[2 * m + 1, 2 * m + 1] = q * dt
        Qd[-1, -1] = q_dc[i]
        P, k = np.eye(nx), np.zeros((nx, 1))
        for _ in range(iters):
            P = Phi @ P @ Phi.T + Qd
            k_new = P @ H.T / float((H @ P @ H.T).ravel()[0] + R[i])
            P = P - k_new @ H @ P
            P = 0.5 * (P + P.T)
            if np.max(np.abs(k_new - k)) < tol:
                k = k_new
                break
            k = k_new
        K[i] = k.ravel()
    return K


class KalmanVelocity:
    """Per-sensor velocity/displacement: one undamped oscillator per mode + a DC state.

    Exact-ZOH transition, fixed measured gains K. Undamped is measured, not
    assumed: Q > 433 at 1 sigma, tau > 138 s (analysis/ringdown.md) against a
    2.9-4.7 s closed loop.
    """

    def __init__(self, K, n, f, dt):
        self.n, self.dt, self.f = int(n), float(dt), np.asarray(f, float)
        self.w = 2.0 * np.pi * self.f
        self.nm = len(self.f)
        self.nx = 2 * self.nm + 1
        self.K = np.asarray(K, float).reshape(self.n, self.nx)
        self.H = np.zeros(self.nx)
        self.H[0:2 * self.nm:2] = 1.0
        self.H[-1] = 1.0
        self.Cv = np.zeros(self.nx)
        self.Cv[1:2 * self.nm:2] = 1.0
        self.Phi = self._transition(self.dt)
        self.x = np.zeros((self.n, self.nx))
        self.primed = np.zeros(self.n, bool)

    def _transition(self, dt):
        Phi = _osc_blocks(np.zeros((self.nx, self.nx)), self.w, dt)
        Phi[-1, -1] = 1.0
        return Phi

    def update(self, y, dt=None, valid=None):
        Phi = (self.Phi if dt is None or abs(dt - self.dt) < 1e-9
               else self._transition(dt))
        y = np.asarray(y, float)
        cold = ~self.primed
        if cold.any():
            self.x[cold] = 0.0
            self.x[cold, -1] = y[cold]           # the offset, not the motion
            self.primed[:] = True
        xp = self.x @ Phi.T
        e = y - xp @ self.H
        if valid is not None:
            e = np.where(valid, e, 0.0)
        self.x = xp + self.K * e[:, None]
        return self.x @ self.Cv

    def reset(self, mask=slice(None)):
        self.x[mask] = 0.0
        self.primed[mask] = False

    def displacement(self):
        return self.x[:, 0:2 * self.nm:2].sum(axis=1)

    def modal_velocity(self):
        """n x nm: sensor i's velocity in mode m's band."""
        return self.x[:, 1:2 * self.nm:2]


class ModalKalman:
    """One filter whose state IS the modal coordinates.

        x = [q_m, v_m] per mode + one DC state per sensor
        y_i = sum_m Phi[i,m] q_m + d_i + n_i,   n_i ~ N(0, R_i)

    Joseph-form covariance, and chi2 = e^T S^-1 e per dof from the innovation
    covariance. chi2 is LOGGED and NOT acted on: its distribution on this rig
    has never been measured, so a threshold would be a guess
    (mimo_closed.md 4.5). A static per-sensor offset is invisible to it BY
    DESIGN -- that is what the DC states absorb.
    """

    def __init__(self, phi, rowok, r, q_sensor, q_dc, f, dt, n, t_amp_s):
        self.f = np.asarray(f, float)
        self.w = 2.0 * np.pi * self.f
        self.nm = len(self.f)
        self.n = int(n)
        self.nx = 2 * self.nm + self.n
        self.dt = float(dt)
        self.r = np.asarray(r, float)
        self.q_dc = np.asarray(q_dc, float)
        self.rowok = np.asarray(rowok, bool).reshape(self.n, self.nm)
        self.phi = np.where(self.rowok, np.asarray(phi, float), 0.0)
        q_sensor = np.asarray(q_sensor, float)

        # One modal process variance per mode: the sum over the sensors whose
        # row is determined there. A mode with no determined row falls back to
        # the loudest sensor rather than to zero, which would freeze it.
        self.q_mode = np.array([float(q_sensor[self.rowok[:, m], m].sum())
                                for m in range(self.nm)])
        for m in range(self.nm):
            if not (self.q_mode[m] > 0):
                self.q_mode[m] = float(q_sensor[:, m].max())
        self.var_q = self.q_mode * float(t_amp_s) / (2.0 * self.w ** 2)

        self.H0 = np.zeros((self.n, self.nx))
        self.H0[:, 0:2 * self.nm:2] = self.phi
        self.H0[np.arange(self.n), 2 * self.nm + np.arange(self.n)] = 1.0

        self.x = np.zeros(self.nx)
        self.P = np.zeros((self.nx, self.nx))
        self.dc_primed = np.zeros(self.n, bool)
        self.seen = np.zeros(self.nm, int)
        self.chi2, self.dof = float("nan"), 0
        self.n_singular = 0
        self._trans_dt = None
        self.Phi = self.Qd = None
        self._prior()

    def _prior(self):
        self.P[:] = 0.0
        for m in range(self.nm):
            self.P[2 * m, 2 * m] = self.var_q[m]
            self.P[2 * m + 1, 2 * m + 1] = self.var_q[m] * self.w[m] ** 2
        for i in range(self.n):
            self.P[2 * self.nm + i, 2 * self.nm + i] = self.r[i]

    def reset(self):
        self.x[:] = 0.0
        self.dc_primed[:] = False
        self.seen[:] = 0
        self.chi2, self.dof = float("nan"), 0
        self._prior()

    def _transition(self, dt):
        if self._trans_dt is not None and abs(dt - self._trans_dt) < 1e-9:
            return
        nm, nx = self.nm, self.nx
        Phi = _osc_blocks(np.eye(nx), self.w, dt)
        Qd = np.zeros((nx, nx))
        for m, w in enumerate(self.w):
            th = w * dt
            s, s2 = np.sin(th), np.sin(2.0 * th)
            Qd[2 * m:2 * m + 2, 2 * m:2 * m + 2] = self.q_mode[m] * np.array(
                [[(dt / 2.0 - s2 / (4.0 * w)) / w ** 2, s ** 2 / (2.0 * w ** 2)],
                 [s ** 2 / (2.0 * w ** 2), dt / 2.0 + s2 / (4.0 * w)]])
        Qd[np.arange(2 * nm, nx), np.arange(2 * nm, nx)] = self.q_dc * dt
        self.Phi, self.Qd, self._trans_dt = Phi, Qd, dt

    def update(self, y, live, dt=None):
        dt = self.dt if dt is None else dt
        self._transition(dt)
        y = np.asarray(y, float)
        live = np.asarray(live, bool)
        nm = self.nm

        # A sensor rejoining brings an unknown offset, so re-prime its DC state
        # from the current reading rather than trusting a stale one.
        self.dc_primed &= live
        for i in np.nonzero(live & ~self.dc_primed)[0]:
            k = 2 * nm + i
            self.x[k] = y[i] - float(self.phi[i] @ self.x[0:2 * nm:2])
            self.P[k, :] = 0.0
            self.P[:, k] = 0.0
            self.P[k, k] = self.r[i]
        self.dc_primed |= live
        self.seen = np.array([int((live & self.rowok[:, m]).sum())
                              for m in range(nm)])

        xp = self.Phi @ self.x
        Pp = self.Phi @ self.P @ self.Phi.T + self.Qd
        idx = np.nonzero(live)[0]
        if idx.size == 0:
            self.x, self.P = xp, Pp
            self.chi2, self.dof = float("nan"), 0
            return self.qdot()

        H = self.H0[idx]
        rl = self.r[idx]
        e = y[idx] - H @ xp
        PHt = Pp @ H.T
        S = H @ PHt + np.diag(rl)
        try:
            Se = np.linalg.solve(S, np.column_stack([e, PHt.T]))
        except np.linalg.LinAlgError:
            self.n_singular += 1
            self.x, self.P = xp, Pp
            self.chi2, self.dof = float("nan"), 0
            return self.qdot()
        Sinv_e, K = Se[:, 0], Se[:, 1:].T
        self.chi2, self.dof = float(e @ Sinv_e), int(idx.size)
        self.x = xp + K @ e
        IKH = np.eye(self.nx) - K @ H
        self.P = IKH @ Pp @ IKH.T + (K * rl) @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return self.qdot()

    def qdot(self):
        return self.x[1:2 * self.nm:2].copy()

    def q(self):
        return self.x[0:2 * self.nm:2].copy()

    def chi2_per_dof(self):
        return self.chi2 / self.dof if self.dof else float("nan")

    def report(self):
        return ["[mkalman] %d states: %d modes x 2 + %d sensor DC"
                % (self.nx, self.nm, self.n),
                "[mkalman] modes %s Hz" % np.round(self.f, 5),
                "[mkalman] rows determined per mode: %s"
                % [int(self.rowok[:, m].sum()) for m in range(self.nm)],
                "[mkalman] q_modal (V^2/s^3) %s"
                % np.array2string(self.q_mode, precision=4),
                "[mkalman] prior rms per mode (V) %s"
                % np.array2string(np.sqrt(self.var_q), precision=4),
                "[mkalman] sqrt(R) per sensor (V) %s"
                % np.array2string(np.sqrt(self.r), precision=4),
                "[mkalman] chi2 per dof is LOGGED, NOT ACTED ON -- no threshold "
                "exists for this rig; the first bench run measures one."]


class Chi2Log:
    """Distribution accumulator: n, mean, max, and the fraction over each level."""

    def __init__(self, levels):
        self.levels = tuple(levels)
        self.n, self.total, self.peak = 0, 0.0, 0.0
        self.over = [0] * len(self.levels)

    def add(self, x):
        x = float(x)
        if not np.isfinite(x):
            return                               # no live row that step
        self.n += 1
        self.total += x
        self.peak = max(self.peak, x)
        for k, lv in enumerate(self.levels):
            if x > lv:
                self.over[k] += 1

    def line(self, label):
        if not self.n:
            return "  %-12s no samples" % label
        return ("  %-12s n=%-7d mean %7.3f  max %9.3f   " % (
                    label, self.n, self.total / self.n, self.peak)
                + "  ".join(">%g %5.1f%%" % (lv, 100.0 * c / self.n)
                            for lv, c in zip(self.levels, self.over)))


# ===========================================================================
# modal: load and validate Phi and A, sense, project, allocate
# ===========================================================================
class Modal:
    """Phi and A, from geometry or from a JSON file, with the refusals, plus the law.

    Phi comes either from the file or from `phi=` -- a GEOMETRIC Phi, exact and
    signed. Prefer the second. Phi and A are each determined only up to a shared
    per-mode sign, so a measured pair has a gauge to get wrong, and getting it
    wrong PUMPS: measured on hardware 2026-08-17, median channel ratio 1.5
    diagonal -> 2.3 modal. Nothing offline catches it -- the allocator inverts a
    wrong-signed A exactly, so the force is right in the model and backwards in
    the plant. A geometric Phi has no free sign, so A's sign follows from the
    driven measurement. A measured Phi is then a CHECK: per-mode |cos| against
    the geometry, warned under `phi_cos_warn`.

    Refuses on: schema, shape, frequency mismatch, age, malformed blocks, a
    missing A, a provisional A, a Phi/A gauge mismatch, and a Phi/A BASIS
    mismatch -- an A measured in one basis paired with a Phi in another is the
    pumping case above.

    Does NOT refuse on colocation. `A[m,j] = lambda_j Phi[j,m]` needs each
    coil's force along its own sensor's axis at the same head, and the rig is
    not built that way: four coils point the same way, two are diametrically
    opposed, three planes. Evidence it does not hold: two independent coil
    passes gave lambda signs (+,+,+,+) and (+,-,-,+), and a colocation-
    constrained rank-1 fit had sigma2/sigma1 = 0.67 where colocation gives ~0.
    The check also rejected a good A. It is computed and logged loudly, never a
    gate.
    """

    def __init__(self, path, n, f, schema, min_sensors=2, min_coils=3,
                 cond_max=12.0, max_age_s=7 * 24 * 3600.0, allow_provisional=False,
                 balance=(12, 0.5, 1.05), demand_cap_v=0.20, imag_warn=0.35,
                 now=None, phi=None, basis=None, phi_cos_warn=0.90, a=None,
                 a_coils=(), a_provisional=True, a_provenance=""):
        self.path, self.n = path, int(n)
        self.f = np.asarray(f, float)
        self.nm = len(self.f)
        self.schema = schema
        self.min_sensors, self.min_coils = int(min_sensors), int(min_coils)
        self.cond_max, self.max_age_s = float(cond_max), float(max_age_s)
        self.allow_provisional = bool(allow_provisional)
        self.balance = balance
        self.demand_cap_v = float(demand_cap_v)
        self.imag_warn = float(imag_warn)
        self.ok, self.why, self.source, self.created = False, "not loaded", "", ""
        self.phi = np.zeros((self.n, self.nm))
        self.sigma = np.full((self.n, self.nm), np.inf)
        self.rowok = np.zeros((self.n, self.nm), bool)
        self.a, self.a_coils = None, []
        self.provisional_a = False
        self.lam = {}
        self.imag_frac = [float("nan")] * self.nm
        self.notes = []
        # Phi supplied directly: no file, no jackknife, no gauge stamp.
        self.phi_in = (None if phi is None
                       else np.asarray(phi, float).reshape(self.n, self.nm))
        self.basis = str(basis or ("geometric" if phi is not None else "measured"))
        self.phi_cos_warn = float(phi_cos_warn)
        self.phi_cos = np.full(self.nm, np.nan)
        self.phi_cos_bad = False
        self.a_in = (None if a is None
                     else np.asarray(a, float).reshape(self.nm, self.n))
        self.a_in_coils = [int(c) for c in a_coils]
        self.a_in_provisional = bool(a_provisional)
        self.a_in_provenance = str(a_provenance)
        self.a_from = ""
        self._load(now)

    def _refuse(self, why):
        self.ok, self.why = False, why
        return False

    def _load(self, now=None):
        d, absent = None, None
        if self.path is None:
            absent = ("no modal path given -- the Controller default, so the "
                      "suite is reproducible")
        elif not os.path.exists(self.path):
            absent = ("no %s -- run `status.py phi --save-modal`"
                      % os.path.basename(self.path))
        if absent is not None and self.phi_in is None:
            return self._refuse(absent)     # a measured Phi is the only Phi there is
        if absent is not None:
            self.notes.append("%s. Phi is GEOMETRIC and needs none, so nothing in "
                              "this run cross-checks the geometry." % absent)
        else:
            try:
                with open(self.path) as fh:
                    d = json.load(fh)
            except (OSError, ValueError) as e:
                return self._refuse("%s is unreadable (%s)"
                                    % (os.path.basename(self.path), e))
            if d.get("schema") != self.schema:
                return self._refuse("schema is %r, this file wants %r"
                                    % (d.get("schema"), self.schema))
            if (int(d.get("n_sensors", -1)) != self.n
                    or int(d.get("n_modes", -1)) != self.nm):
                return self._refuse("measured for %s sensors x %s modes, this file "
                                    "is %d x %d" % (d.get("n_sensors"),
                                                    d.get("n_modes"), self.n, self.nm))
            fs = [m.get("f_hz") for m in d.get("modes", [])]
            if len(fs) != self.nm or any(
                    abs(float(a) - float(b)) > 0.005 for a, b in zip(fs, self.f)):
                return self._refuse("measured at %s Hz, this file runs at %s Hz -- "
                                    "re-measure, or the shapes do not apply"
                                    % ([round(float(x), 4) for x in fs],
                                       [round(float(x), 4) for x in self.f]))
            self.created, self.source = d.get("created", ""), d.get("source", "")
            age = self._age(self.created, now)
            if age is not None and age > self.max_age_s:
                return self._refuse("measured %.1f days ago (limit %.1f) -- these "
                                    "numbers drift, re-run status.py"
                                    % (age / 86400.0, self.max_age_s / 86400.0))

        p = (d or {}).get("phi") or {}
        if self.phi_in is not None:
            self._geometric_phi(p)
        elif not self._measured_phi(p):
            return False
        if not self._take_a(d):
            return False

        # Unit-norm each Phi column and each A row: the law's scale lives in the
        # gains, not in whatever units the measurement came out in.
        for m in range(self.nm):
            s = np.linalg.norm(self.phi[self.rowok[:, m], m])
            if s > 0:
                self.phi[:, m] /= s
                self.sigma[:, m] /= s
            s = np.linalg.norm(self.a[m, self.a_coils]) if self.a_coils else 0.0
            if s > 0:
                self.a[m] /= s
        a = (d or {}).get("a") or {}
        if (self.phi_in is None and self.a_from == "file"
                and (p.get("gauge") or "") != (a.get("gauge") or "")):
            return self._refuse("phi and A come from different measurements "
                                "(%r vs %r). Their per-mode signs are then "
                                "independent, and a wrong sign pumps. Re-run "
                                "`status.py coils --save-modal`, which writes "
                                "both from one factorisation."
                                % (p.get("gauge"), a.get("gauge")))

        self._colocation()
        self._build_tables()
        self.ok, self.why = True, "loaded"
        return True

    def _measured_phi(self, p):
        try:
            self.phi = np.asarray(p["value"], float).reshape(self.n, self.nm)
            self.sigma = np.abs(np.asarray(p["sigma"], float)).reshape(self.n, self.nm)
            self.rowok = np.asarray(p["ok"], bool).reshape(self.n, self.nm)
        except (KeyError, ValueError, TypeError) as e:
            return self._refuse("phi block is malformed (%s)" % e)
        self.imag_frac = [float(x) for x in
                          p.get("imag_frac", [float("nan")] * self.nm)]
        for mi, fr in enumerate(self.imag_frac):
            if fr == fr and fr > self.imag_warn:
                self.notes.append("mode %d shape is %.0f%% imaginary -- no real "
                                  "mode shape explains that; provisional"
                                  % (mi, 100.0 * fr))
        return True

    def _geometric_phi(self, p):
        """Phi from geometry; a measured Phi in the file only CHECKS it."""
        self.phi = self.phi_in.copy()
        self.rowok = self.phi != 0.0
        # Geometry carries no per-entry uncertainty, so `sense`'s weighted LS is
        # unweighted here. The Kalman still weights by the MEASURED per-sensor R.
        self.sigma = np.where(self.rowok, 1.0, np.inf)
        try:
            meas = np.asarray(p["value"], float).reshape(self.n, self.nm)
        except (KeyError, ValueError, TypeError):
            return
        for m in range(self.nm):
            use = self.rowok[:, m] & np.isfinite(meas[:, m])
            g, v = self.phi[use, m], meas[use, m]
            den = np.linalg.norm(g) * np.linalg.norm(v)
            self.phi_cos[m] = abs(float(g @ v)) / den if den > 0 else float("nan")
        bad = [m for m in range(self.nm) if not self.phi_cos[m] >= self.phi_cos_warn]
        self.phi_cos_bad = bool(bad)
        line = " ".join("%.3f" % c for c in self.phi_cos)
        if bad:
            self.notes.append(
                "!! measured Phi against GEOMETRY, |cos| per mode %s -- mode(s) %s "
                "under %.2f. Either the geometry or the corner assignment is "
                "wrong, or the measurement is. It reads 0.971 / 0.978 / 0.963 when "
                "both are right (data/modal.json, 2026-08-17)."
                % (line, bad, self.phi_cos_warn))
        else:
            self.notes.append("measured Phi against GEOMETRY, |cos| per mode %s, "
                              "all over %.2f -- a confirmation, not an input"
                              % (line, self.phi_cos_warn))

    def _refuse_provisional(self, what, provenance):
        return self._refuse("%s (%s) and was never validated closed-loop. Set "
                            "OSEM_MODAL_PROVISIONAL=1 to run on it anyway, which "
                            "is a test configuration, not a bench one."
                            % (what, provenance or "?"))

    def _take_a(self, d):
        """A from the file if there is one, else the caller's fallback."""
        a = (d or {}).get("a")
        if a is None:
            if self.a_in is None:
                return self._refuse("phi is present but A is not -- run "
                                    "`status.py coils --save-modal`. Sensing is "
                                    "solved; allocation is not.")
            self.a, self.a_coils, self.a_from = (self.a_in.copy(),
                                                 list(self.a_in_coils), "supplied")
            self.provisional_a = self.a_in_provisional
            if self.a_in_provisional and not self.allow_provisional:
                return self._refuse_provisional(
                    "the only A here is the supplied fallback",
                    self.a_in_provenance)
            return True
        try:
            self.a = np.asarray(a["value"], float).reshape(self.nm, self.n)
            self.a_coils = [int(c) for c in a.get("coils", [])]
        except (KeyError, ValueError, TypeError) as e:
            return self._refuse("a block is malformed (%s)" % e)
        self.a_from = "file"
        self.provisional_a = bool(a.get("provisional"))
        fb = str(a.get("basis") or "")
        if (self.basis != "measured" or fb) and fb != self.basis:
            return self._refuse("A in %s is in the %r basis and Phi is %r. Their "
                                "per-mode signs are then independent and a wrong "
                                "sign PUMPS -- 1.5 -> 2.3 median ratio on hardware "
                                "2026-08-17. Re-derive A against this Phi and stamp "
                                "the a block basis=%r; with no file at all the loop "
                                "falls back to the A the caller supplied."
                                % (os.path.basename(self.path or "?"),
                                   fb or "measured", self.basis, self.basis))
        if a.get("provisional") and not self.allow_provisional:
            return self._refuse_provisional("A in the file is PROVISIONAL",
                                            a.get("provenance"))
        return True

    def _colocation(self):
        """A/Phi per mode, per coil. Logged, never a gate. See the class docstring."""
        for j in self.a_coils:
            lam = [self.a[m, j] / self.phi[j, m] for m in range(self.nm)
                   if self.rowok[j, m] and abs(self.phi[j, m]) > 1e-12]
            self.lam[j] = lam
            if len(lam) < 2:
                self.notes.append("coil %d: %d determined mode(s), colocation "
                                  "ratio not testable" % (j, len(lam)))
            elif all(x > 0 for x in lam) or all(x < 0 for x in lam):
                self.notes.append("coil %d colocation ratio CONSISTENT (%s)"
                                  % (j, ", ".join("%+.3f" % x for x in lam)))
            else:
                self.notes.append(
                    "coil %d A/Phi SIGN DISAGREES across modes (%s). On a "
                    "colocated head that would mean Phi and A disagree; this rig "
                    "is not colocated (four coils one way, two opposed, three "
                    "planes) so it is NOT a gate. A's signs are settled by the DC "
                    "and driven passes, 12 of 12 across three determinations that "
                    "share no estimator." % (j, ", ".join("%+.3f" % x for x in lam)))

    @staticmethod
    def _age(created, now=None):
        if not created:
            return None
        try:
            t = datetime.fromisoformat(created)
        except ValueError:
            return None
        ref = datetime.now() if now is None else now
        if t.tzinfo is not None:
            t = t.replace(tzinfo=None)
        return max((ref - t).total_seconds(), 0.0)

    def _build_tables(self):
        """One allocator per coil subset: 2^n masks, precomputed once."""
        iters, step, tol = self.balance
        self.pinv = [None] * (1 << self.n)
        self.cond = [float("inf")] * (1 << self.n)
        self.gate = [False] * (1 << self.n)
        self.rank = [0] * (1 << self.n)
        self.balanced = [False] * (1 << self.n)
        # The row-balancing scale D per mask, the tie-break metric `cap_box`
        # needs to reproduce this allocator exactly when the box is slack.
        self.bal = [None] * (1 << self.n)
        drivable = np.zeros(self.n, bool)
        drivable[self.a_coils] = True
        for mask in range(1 << self.n):
            cols = [j for j in range(self.n) if (mask >> j) & 1 and drivable[j]]
            if len(cols) < self.min_coils:
                continue
            Ac = self.a[:, cols]
            U, sv, Vh = np.linalg.svd(Ac, full_matrices=False)
            if sv[0] <= 1e-9:
                continue
            self.cond[mask] = float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else float("inf")
            keep = int(np.sum(sv >= sv[0] / self.cond_max))
            keep = min(keep, int(np.linalg.matrix_rank(Ac, tol=1e-6)))
            if keep < 1:
                continue
            Uk, svk, Vhk = U[:, :keep], sv[:keep], Vh[:keep, :]
            P = (Vhk.conj().T * (1.0 / svk)) @ Uk.conj().T
            Ak = (Uk @ Uk.conj().T) @ Ac          # A restricted to kept directions

            # Row-norm balancing. Equalising ||P[j,:]|| equalises the worst-case
            # per-coil demand over all f; substituting u = D v makes it plain
            # min-norm, so A_C P = I still holds for any positive weights. That
            # identity is VERIFIED numerically below -- an earlier version
            # silently dropped a singular direction -- and the unweighted
            # inverse is the fallback.
            w, Pw, Dw = np.ones(len(cols)), None, np.ones(len(cols))
            for _ in range(int(iters)):
                D = 1.0 / w
                try:
                    Pw = np.linalg.pinv(Ak * D[None, :]) * D[:, None]
                except np.linalg.LinAlgError:
                    Pw = None
                    break
                Dw = D            # the D THIS Pw was built from: `w` moves below
                rn = np.linalg.norm(Pw, axis=1)
                if not np.all(np.isfinite(rn)) or rn.max() <= 0:
                    Pw = None
                    break
                if rn.max() / max(rn.min(), 1e-30) < tol:
                    break
                w = np.clip(w * (rn / rn.mean()) ** step, 1e-3, 1e3)
                w = w / w.mean()
            if Pw is not None and np.allclose(Ac @ Pw, Uk @ Uk.conj().T,
                                              rtol=1e-7, atol=1e-10):
                P, self.balanced[mask] = Pw, True
                self.bal[mask] = np.real(Dw)
            if self.bal[mask] is None:
                self.bal[mask] = np.ones(len(cols))
            self.pinv[mask] = (cols, np.real(P), np.real(Uk @ Uk.conj().T))
            self.rank[mask] = keep
            self.gate[mask] = keep >= 1

    def sense(self, vmodal, sense_ok):
        """Per-mode scalar LS on the determined rows -> (q, out-of-mode residual, n)."""
        q = np.zeros(self.nm)
        r = np.zeros(self.nm)
        nseen = np.zeros(self.nm, int)
        for m in range(self.nm):
            use = sense_ok & self.rowok[:, m]
            nseen[m] = k = int(use.sum())
            if k < self.min_sensors:
                continue
            phi, v = self.phi[use, m], vmodal[use, m]
            w = 1.0 / np.maximum(self.sigma[use, m], 1e-12) ** 2
            den = float(np.sum(w * phi * phi))
            if den <= 0:
                continue
            q[m] = float(np.sum(w * phi * v)) / den
            nv = float(np.linalg.norm(v))
            r[m] = float(np.linalg.norm(v - phi * q[m]) / nv) if nv > 0 else 0.0
        return q, r, nseen

    @staticmethod
    def mask_of(drive_ok):
        mask = 0
        for j, b in enumerate(drive_ok):
            if b:
                mask |= 1 << j
        return mask

    def project(self, qdot, drive_ok):
        """Project the VELOCITY onto the reachable modes, before the gain.

        -Proj K Proj qdot is symmetric PSD and so dissipative; -Proj K qdot is
        not, and can pump. Never project the force after the gain.
        """
        mask = self.mask_of(drive_ok)
        e = self.pinv[mask]
        if not self.gate[mask] or e is None:
            return np.zeros(self.nm), 0
        return e[2] @ np.asarray(qdot, float), self.rank[mask]

    def allocate(self, f, drive_ok):
        """(u, mask, cols_mask, ok). One uniform scale if the peak exceeds the cap."""
        mask = self.mask_of(drive_ok)
        cm = np.zeros(self.n, bool)
        if not self.gate[mask] or self.pinv[mask] is None:
            return np.zeros(self.n), mask, cm, False
        cols, P, Proj = self.pinv[mask]
        u = np.zeros(self.n)
        u[cols] = P @ (Proj @ f)
        cm[cols] = True
        pk = float(np.max(np.abs(u))) if u.size else 0.0
        if pk > self.demand_cap_v:
            u *= self.demand_cap_v / pk           # uniform: direction preserved
        return u, mask, cm, True

    def box_args(self, mask, kp):
        """(W, d) for `cap_box` on this coil mask, or (None, None) -- KEEP `cap_scale`.

        W is the metric `cap_box`'s dissipation proof needs and nothing else:
        W = pinv(Proj K Proj) with K = diag(kp) the per-mode gain ACTUALLY
        applied this step. On a full-rank mask Proj = I, so W = diag(1/kp)
        exactly.

        Returns (None, None) -- meaning the caller must keep `cap_scale`, which
        needs no metric -- in the two cases where the proof does not close:

        * a RANK-DEFICIENT mask (`rank[mask] < nm`, which is 2 of the 3 modes
          gone or worse). There the realised force can leave range(Proj) and the
          projection argument only bounds `qdot' Proj p`, not `qdot' p`. Today's
          allocator never realises an out-of-range force at all -- `A_C P` is
          verified equal to Proj in `_build_tables` -- so scaling it by one
          scalar cannot either, and `cap_scale` stays proven where this is not.
        * a NON-POSITIVE per-mode gain, which is every mode a caller zeroed
          (osem.eta.py zeroes a mode with too few determined sensors) and the
          whole vector during the gain ramp's first step. K is then singular and
          K^-1 does not exist.

        `d` is the row-balancing scale `_build_tables` used, so that a slack box
        returns today's allocation to floating point instead of a different
        point of the same objective.
        """
        e = self.pinv[mask]
        if e is None or not self.gate[mask] or self.rank[mask] < self.nm:
            return None, None
        k = np.asarray(kp, float).ravel()
        if k.size != self.nm or not np.all(np.isfinite(k)) or np.any(k <= 0.0):
            return None, None
        cols = e[0]
        d = np.ones(self.n)
        b = self.bal[mask]
        if b is not None:
            d[cols] = b
        return 1.0 / k, d

    def report(self, kp=None, scale=1.0):
        out = ["[modal] %s" % ("LOADED" if self.ok else "REFUSED -- running DIAGONAL"),
               "[modal] %s" % self.why]
        if self.source:
            out.append("[modal] source: %s" % self.source)
        if self.created:
            out.append("[modal] measured: %s" % self.created)
        out.append("[modal] Phi basis: %s -- %s"
                   % (self.basis,
                      "exact and signed, so there is no per-mode gauge to get wrong"
                      if self.phi_in is not None else
                      "determined only up to a per-mode sign, which A must share"))
        if np.isfinite(self.phi_cos).any():
            out.append("[modal] measured Phi vs geometry |cos| %s (warn under %.2f)"
                       % (" ".join("%.3f" % c for c in self.phi_cos),
                          self.phi_cos_warn))
        if self.a is not None:
            out.append("[modal] A from %s%s" % (self.a_from,
                       " -- PROVISIONAL" if self.provisional_a else ""))
        for n in self.notes:
            out.append("[modal] NOTE: %s" % n)
        if not self.ok:
            return out
        out.append("[modal] Phi rows determined at every mode: %s"
                   % [i for i in range(self.n) if self.rowok[i].all()])
        out.append("[modal] Phi (unit-norm columns):")
        for i in range(self.n):
            out.append("[modal]   a%d  %s   %s"
                       % (i, " ".join("%+8.4f" % x for x in self.phi[i]),
                          "".join("y" if x else "." for x in self.rowok[i])))
        out.append("[modal] A rows (unit-norm), coils %s:" % self.a_coils)
        for m in range(self.nm):
            out.append("[modal]   mode %d  %s"
                       % (m, " ".join("%+8.4f" % x for x in self.a[m])))
        full = int(sum(1 << j for j in self.a_coils))
        out.append("[modal] full coil set %s: cond %.2f, gate %s, row-balanced %s"
                   % (self.a_coils, self.cond[full],
                      "PASS" if self.gate[full] else "FAIL",
                      "yes" if self.balanced[full] else "no (unweighted inverse)"))
        for j in self.a_coils:
            m2 = full & ~(1 << j)
            out.append("[modal]   drop coil %d -> cond %.2f, %s"
                       % (j, self.cond[m2], "MIMO" if self.gate[m2] else "diagonal"))
        out.append("[modal] %d of %d coil masks run MIMO; the rest fall back to the "
                   "diagonal law, a degradation and not a fault."
                   % (sum(1 for g in self.gate if g), 1 << self.n))
        if kp is not None:
            out.append("[modal] Kp per mode %s x scale %.2f = %s"
                       % (np.round(kp, 4), scale, np.round(np.asarray(kp) * scale, 4)))
        return out


# ===========================================================================
# interlocks
# ===========================================================================
class Health:
    """Rail detect, sustained-rail interlock, dead-pin detect, floor demotion, re-arm.

    Owns: rail, rail_std, sat, healthy, clear_t, floor_bad.
    """

    def __init__(self, n, rail_lo, rail_hi, rail_sustain_s, rail_frac,
                 sat_sustain_s, sat_frac, dead_std, rearm_s, floor_frac):
        self.n = int(n)
        self.rail_lo, self.rail_hi = rail_lo, rail_hi
        self.rail_sustain_s, self.rail_frac = float(rail_sustain_s), float(rail_frac)
        self.sat_sustain_s, self.sat_frac = float(sat_sustain_s), float(sat_frac)
        self.dead_std, self.rearm_s = float(dead_std), float(rearm_s)
        self.floor_frac = float(floor_frac)
        self.rail = np.zeros(self.n, bool)
        self.sat = np.zeros(self.n, bool)
        self.healthy = np.ones(self.n, bool)
        self.floor_bad = np.zeros(self.n, bool)
        self.clear_t = np.full(self.n, np.inf)
        self.rail_std = np.zeros(self.n)
        self.rail_hist, self.sat_hist = deque(), deque()
        self.rail_sum = np.zeros(self.n)
        self.sat_sum = np.zeros(self.n)
        self.c_sum, self.c2_sum = np.zeros(self.n), np.zeros(self.n)

    @staticmethod
    def hold(cond, since, t):
        """Wall-clock latch: first t at which `cond` became true, else inf."""
        return np.where(cond, np.where(np.isinf(since), t, since), np.inf)

    def rails(self, counts, t):
        """At the WIRE rate. Also accumulates the raw-count std over the same window."""
        railed = (counts <= self.rail_lo) | (counts >= self.rail_hi)
        self.rail_hist.append((t, railed, counts.copy()))
        self.rail_sum = self.rail_sum + railed
        self.c_sum = self.c_sum + counts
        self.c2_sum = self.c2_sum + counts * counts
        while self.rail_hist and t - self.rail_hist[0][0] > self.rail_sustain_s:
            _, r0, c0 = self.rail_hist.popleft()
            self.rail_sum = self.rail_sum - r0
            self.c_sum = self.c_sum - c0
            self.c2_sum = self.c2_sum - c0 * c0
        spanned = (bool(self.rail_hist)
                   and t - self.rail_hist[0][0] >= self.rail_sustain_s * 0.9)
        nw = max(len(self.rail_hist), 1)
        self.rail = np.logical_and(spanned, self.rail_sum / nw >= self.rail_frac)
        self.rail_std = np.sqrt(np.maximum(
            self.c2_sum / nw - (self.c_sum / nw) ** 2, 0.0))

    def sats(self, pinned, t):
        self.sat_hist.append((t, pinned))
        self.sat_sum = self.sat_sum + pinned
        while self.sat_hist and t - self.sat_hist[0][0] > self.sat_sustain_s:
            self.sat_sum = self.sat_sum - self.sat_hist.popleft()[1]
        spanned = (bool(self.sat_hist)
                   and t - self.sat_hist[0][0] >= self.sat_sustain_s * 0.9)
        self.sat = np.logical_and(
            spanned, self.sat_sum / max(len(self.sat_hist), 1) >= self.sat_frac)

    def dead_pin(self, enabled):
        """Railed AND essentially motionless -> an unwired pin, not a plant excursion.

        STD of raw counts over the rail window, threshold `dead_std`. Span
        thresholds provably overlap (dead a7 reaches 4.625, sim occlusion sits at
        2.71); std does not -- dead a6/a7 max 0.853, sim occlusion min 1.998,
        live channels never below 4.80. Returns (mask, witness_ok): with no
        witness left nothing is demoted and the rail fault stands.
        """
        out = self.rail & (self.rail_std < self.dead_std) & enabled
        if out.any() and not (enabled & ~out).any():
            return np.zeros(self.n, bool), False
        return out, True

    def rearm(self, t, kf_reset=None):
        """Demote a railed channel; hand one back rearm_s after its rail clears.

        Only RAIL demotes: a saturated channel measures fine and only its output
        is clipped, so parking it removes authority at peak amplitude.
        `~floor_bad` is what makes the floor demotion stick -- a signal-less
        sensor never railed, so this path would hand it back every 2 s forever.
        """
        drop = self.healthy & self.rail
        self.healthy[drop] = False
        self.clear_t[self.rail] = np.inf
        idle = ~self.healthy & ~self.rail & ~self.floor_bad
        starting = idle & np.isinf(self.clear_t)
        self.clear_t[starting] = t
        if kf_reset is not None and starting.any():
            kf_reset(starting)
        back = idle & (t - self.clear_t >= self.rearm_s)
        self.healthy[back], self.clear_t[back] = True, np.inf
        return drop, starting, back

    def floor(self, x, enabled, exempt=None, motion=None):
        """Demote any channel under floor_frac of the median, witness rule applied.

        (bad, median, witnessed). `x` must be BAND-LIMITED amplitude -- on a
        total-amplitude statistic the median is set by whoever has the loudest
        out-of-band interference, which is how a4 (good in-band SNR, 0.0030 V
        total against a 0.0953 V median) was demoted on 2026-08-17.

        `exempt` NAMES CHANNELS THE BAND DOES NOT DESCRIBE. The band is the three
        modes of the damped DOF, so a sensor whose axis is ORTHOGONAL to them is
        quiet in it BY CONSTRUCTION, not because it is blind: measured 2026-08-18,
        a4/a6/a7 read 0.0027 / 0.0026 / 0.0024 V against a 0.0834 V median, were
        demoted at engage in both runs, and so carried gain exactly 0.0000 for
        100 % of every DAMPING sample -- the hybrid law never ran once -- while
        their bias-corrected multiple coherence with the optic over that same band
        is 0.46 / 0.26 / 0.50 (CLAUDE.md 6). That is the fourth instance of the
        error family holding "a4-a7 are not wired", "the flags are misaligned" and
        "a weak driven response bounds the COILS, not the sensors".

        An exempt channel is judged on `motion` -- its own BROADBAND activity --
        under the SAME relative rule, `floor_frac` of the median broadband across
        the channels that do carry the band: same criterion, on a statistic the
        channel's axis can actually produce. It keeps them because a4/a6/a7 run
        43.4 / 61.3 / 32.6 counts rms against a0-a3's 57.9-96.0, i.e. 41-77 % of
        the median where their IN-BAND amplitude is 3 % of it.

        AN ABSOLUTE LINE IS NOT ENOUGH, and the simulator proves it: a blind
        sensor there is zero sensor gain, which still returns ADC dither, so "std
        over `dead_std`" passes a sensor that measures nothing. The relative test
        demotes it. `dead_std` stays as a floor under the relative one, so a
        hard-grounded pin goes either way: a5 has variance EXACTLY zero over
        236387 samples.

        With no `motion` given nothing is exempted -- the caller has no evidence,
        so it gets the old behaviour. The median is taken over NON-exempt
        channels: letting an orthogonal axis into it drags the line down and hides
        a genuinely blind in-band sensor.
        """
        ex = (np.zeros(self.n, bool) if exempt is None or motion is None
              else np.asarray(exempt, bool))
        ref = x[enabled & ~ex]
        if ref.size == 0 or not np.any(ref > 0):
            return np.zeros(self.n, bool), 0.0, True
        med = float(np.median(ref))
        bad = x < med * self.floor_frac          # every channel, enabled or not
        if ex.any():
            mot = np.asarray(motion, float)
            mref = mot[enabled & ~ex]
            mmed = float(np.median(mref)) if mref.size else 0.0
            bad = np.where(ex, (mot < mmed * self.floor_frac)
                           | (mot < self.dead_std), bad)
        if (bad & enabled).any() and not (enabled & ~bad).any():
            return np.zeros(self.n, bool), med, False
        return bad, med, True


class Breaker:
    """Runaway trend and the saturation trip. Tests for GROWTH, not level.

    Level alone trips on a kick that is already decaying; `growing` is what
    separates a disturbance the loop is winning against from one it is not.

    `growing` ALONE IS NOT ENOUGH, AND THAT ENDED TWO RUNS. `past` is a maximum
    over history entries at least `window_s` old, so a flat or falling envelope
    still reads as growing until `past` catches up. Reconstructed from the
    records, 2026-08-18: at the modal trip ch0's envelope fell 0.5767 -> 0.5707 V
    while `past` climbed 0.3943 -> 0.4599 V and `growing` stayed true for 3.99 s
    against a 4.0 s sustain. Both runs that night died on the first hand kick,
    under both laws, with zero margin (longest continuous `high & growing`
    4.00 s). So the trip ALSO requires the envelope not to have RECEDED from its
    own recent peak -- the peak INCLUDING the last `window_s`, which is the part
    `past` cannot see. `decay_frac` is the recession test the saturation trip
    already uses, so this adds no threshold.

    A RECESSION VETOES THE TRIP. IT DOES NOT CLEAR THE LATCH, AND THAT IS THE
    WHOLE OF THIS CHANGE -- 2026-08-20. Until then `~receded` sat inside the
    `Health.hold` conjunction, so one rippled sample restarted the sustain timer
    and the breaker went BLIND ACROSS ITS WHOLE PURPOSE: the simulator's pumped
    resonance reached ratio 3.43 with `env > 1.8 x ref` on 84.7 % of samples and
    never tripped, and over 13 recorded bench runs the shipped 0.98 line fires on
    19-61 % of the samples where the latch is trying to accumulate -- between the
    MEDIAN (0.97-1.00) and the 10th percentile (0.82-0.98) of ordinary drawdown.

    NO THRESHOLD AND NO SUSTAIN CAN SEPARATE THE TWO POPULATIONS BY THEMSELVES,
    which is why the fix is structural. From this class's own internals replayed
    over data/2026081[78]_*_fast_lock.csv, channels a0-a3 in DAMPING:

      * DEPTH does not separate. Deepest `env/peak` on latch-candidate samples is
        0.778 over the hand-kick corpus and 0.760 over a bench-sourced runaway
        (the 475 s quiet record 20260818_032243 multiplied by exp(g t)), so every
        `decay_frac` from 0.80 to 0.98 vetoes both or neither.
      * DURATION does not separate. Longest continuous `high & growing`: hand kick
        5.28 s (20260818_024302 ch3, t=150.7-156.0, envelope 0.245 -> 0.630 V,
        ratio 2.0 -> 5.2) against a bench-sourced runaway's 3.71-6.01 s at
        0.05 /s and 4.21-7.16 s at 0.0739 /s -- and at `decay_frac = 0.98` the
        order INVERTS, kick 2.41 s against runaway 1.38-2.83 s.
      * WHY. The three-mode envelope beats at 0.99193 - 0.72194 = 0.26999 Hz, a
        3.704 s period, and a 2 s RMS window is shorter than that, so the beat
        passes into the envelope. The instabilities this loop's own gain can
        produce are 0.05-0.0739 /s -- SLOWER than the beat -- while a hand kick
        rings up at 0.27 /s, 3.7x faster. So it is the KICK whose envelope is
        monotone and the RUNAWAY whose is not, and any test built on "keeps making
        new peaks" ranks them backwards.

    Vetoing separates them because it asks a different question at a different
    time: the latch asks "has this been high and growing continuously for
    `sustain_s`", the recession asks, ONLY once that is already true, "and is it
    at a new peak right now". A kick past its own maximum can never answer yes;
    a runaway makes a new peak every beat. Measured on 52 channel-instances --
    13 bench runs x a0-a3, covering nine hand-kick runs, three quiet closed-loop
    runs and the pumped run 20260817_191710 -- ZERO trips at `sustain_s` 5.0 and
    one at 4.0 (20260818_024302 ch2 at t=39.1 s, the first hand kick of the valid
    kick set). On the same bench-sourced runaway it trips on 2 of 4 channels at
    0.05 /s, 3 of 4 at 0.0739 /s and 4 of 4 at 0.15 /s, where the shipped rule
    tripped on NONE. That is why the rungs moved `RUNAWAY_SUSTAIN_S` 4.0 -> 5.0.

    KNOWN LIMIT, and not fixed by any of this: the one recorded bench pumping
    episode (20260817_191710, a wrong per-mode sign of A, median channel ratio
    2.21-2.46 held for 148 s) is a SUSTAINED elevated state, not a ramp. A growth
    test cannot see it by construction, and this one does not.
    """

    def __init__(self, n, window_s, multiple, sustain_s, lag_s, growth_frac,
                 decay_frac, quorum):
        self.n = int(n)
        self.window_s, self.multiple = float(window_s), float(multiple)
        self.sustain_s, self.lag_s = float(sustain_s), float(lag_s)
        self.growth_frac, self.decay_frac = float(growth_frac), float(decay_frac)
        self.quorum = int(quorum)
        self.bank = RMSBank(self.window_s, self.n)
        self.hist = deque()
        self.valid_t = 0.0
        self.excess_since = np.full(self.n, np.inf)
        self.env = np.zeros(self.n)
        self.peak = np.zeros(self.n)
        self.receded = np.zeros(self.n, bool)

    def arm(self, t):
        """After a re-engagement the trend is blind until its window refills."""
        self.hist.clear()
        self.valid_t = t + self.window_s
        self.excess_since[:] = np.inf

    def reset(self, mask=None):
        self.bank.reset(mask)

    def sample(self, t, x, healthy, drives, baseline, sat, judge=None):
        """`judge` names who may TRIP on amplitude; `drives` is who is watched.

        They differ. A channel whose in-band motion is only fractionally the optic
        has an amplitude that is not evidence ABOUT the optic, and letting it
        freeze the whole rig is the veto CLAUDE.md 7 spent three versions
        removing. The saturation trip is not gated the same way: a coil pinned
        against its rail and still demanding more is unambiguous whatever the
        sensor's coherence. Defaults to `drives`, i.e. the old behaviour.
        """
        judge = drives if judge is None else judge
        env = self.bank.update(t, x, healthy)
        self.env = env
        if t >= self.valid_t:
            self.hist.append((t, env.copy()))
        while self.hist and t - self.hist[0][0] > self.lag_s * 2:
            self.hist.popleft()
        older = [e for ts, e in self.hist if t - ts >= self.window_s]
        past = np.maximum.reduce(older) if older else env
        # The peak INCLUDING the last window_s -- the part `past` is blind to, and
        # the only thing that tells a kick's plateau from a runaway's climb.
        peak = np.maximum.reduce([e for _, e in self.hist]) if self.hist else env
        growing = env > past * self.growth_frac
        high = env > baseline * self.multiple
        receded = env < peak * self.decay_frac
        self.peak, self.receded = peak, receded
        # THE LATCH DOES NOT SEE `receded`. A recession is evidence against
        # tripping RIGHT NOW, not evidence that the last `sustain_s` never
        # happened -- see the class docstring for the 52 channel-instances that
        # says so. What still clears the latch is `high` or `growing` going false,
        # which is what a kick does as soon as it stops rising.
        self.excess_since = Health.hold(healthy & judge & high & growing,
                                        self.excess_since, t)
        # ...and the recession VETOES: the trip needs the envelope to be within
        # `decay_frac` of its own recent peak at the moment it fires. A kick past
        # its own maximum cannot get back there; a runaway does it every beat.
        run_trip = (healthy & judge & ~receded
                    & (t - self.excess_since >= self.sustain_s))
        sat_trip = healthy & sat & ~(env < past * self.decay_frac)
        return dict(env=env, growing=growing, high=high, peak=peak,
                    receded=receded, run_trip=run_trip, sat_trip=sat_trip)


class QuietLevel:
    """The level the RUNNING loop holds, as against the level the plant sits at
    with the gain OFF.

    `Baseline` measures during CALIBRATING, a ZERO-GAIN window by construction, so
    every threshold scaled off it is a threshold against the UNDAMPED plate --
    the right reference for "is the loop making this worse", and not a description
    of the closed loop: measured 2026-08-18 against ONE calibration window, the
    modal law holds per-channel `ratio` 0.117 and the diagonal law
    1.449 / 1.832 / 1.769, a factor of about 13 between two laws sharing that
    window (data/20260818_002207_jerk_eta.log).

    ROBUST TO BEING KICKED BY CONSTRUCTION: the answer is a low percentile of the
    recent envelope, so a disturbance occupying up to (100 - `percentile`) % of the
    window leaves it where it was. A mean would follow the kick, and the kick is
    exactly when the number is read.

    BLIND UNTIL THE WINDOW FILLS. `level()` returns None until `min_fill` of it is
    in hand and every caller falls back to the calibration baseline, which is what
    shipped before this class existed: booting into an untested threshold is worse
    than booting into a known one.

    FED ONLY WHILE DAMPING, AND PRUNED ONLY WHEN FED, so the window holds the last
    `window_s` of CLOSED-LOOP time. A FAULT freezes it rather than ageing it out --
    the actuators are off there, so the sensors see the open-loop plant, which says
    nothing about where this loop lives.
    """

    def __init__(self, n, window_s, percentile, keep_hz, min_fill):
        self.n = int(n)
        self.window_s, self.pct = float(window_s), float(percentile)
        self.keep_dt = 1.0 / float(keep_hz)
        # Per channel, not overall: a channel demoted for half the window has half
        # the votes and must not be answered from the other channels' data.
        self.want = max(2, int(float(min_fill) * self.window_s * float(keep_hz)))
        self.hist = deque()
        self.last_t, self._cache, self._dirty = None, None, True

    def reset(self):
        """A new engagement describes a new loop: gains, law and coils may differ."""
        self.hist.clear()
        self.last_t, self._cache, self._dirty = None, None, True

    def add(self, t, env, mask):
        """One envelope sample, retained at `keep_hz`.

        The envelope is already a sliding RMS over `Breaker.window_s`, so samples
        one control step apart are not independent and keeping all of them buys
        nothing but arithmetic.
        """
        if self.last_t is not None and t - self.last_t < self.keep_dt:
            return
        self.last_t = t
        self.hist.append((t, np.asarray(env, float).copy(),
                          np.asarray(mask, bool).copy()))
        while self.hist and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        self._dirty = True

    def ready(self):
        return len(self.hist) >= self.want

    def level(self):
        """Per-channel low percentile over the window, or None while blind.

        A channel with fewer than `want` of its own samples reads 0.0, which is a
        no-op under the `maximum` its callers take it in: the calibration baseline
        answers for it until it has its own evidence.
        """
        if not self.ready():
            return None
        if self._dirty or self._cache is None:
            x = np.array([e for _, e, _ in self.hist])
            m = np.array([v for _, _, v in self.hist])
            out = np.zeros(self.n)
            for j in range(self.n):
                col = x[m[:, j], j]
                if col.size >= self.want:
                    out[j] = float(np.percentile(col, self.pct))
            self._cache, self._dirty = out, False
        return self._cache


def cap_scale(m, fixed, cap, mask):
    """The largest s in [0, 1] with |s*m + fixed| <= cap on every masked channel.

    ONE SCALAR ON THE WHOLE VECTOR. Scaling a modal allocation by a scalar
    preserves its direction, so the realised force stays parallel to the commanded
    one and the law stays dissipative; a per-channel clip does not.

    Scaling by whatever fraction of `cap` the ALLOCATION alone occupies does not
    bring the TOTAL under `cap` once the terms added after it are themselves large.
    That is how coil 3 reached 0.4229 V rms-peak against a 0.250 V half-window --
    169 % -- while the modal allocation stayed inside its own 0.20 V cap for the
    whole run (CLAUDE.md 2). This solves for the scale instead of estimating it.

    Returns 0.0 where `fixed` alone already exceeds `cap`: no scale can fix that,
    and `PID.drive`'s clip is what handles it. Returns 1.0 if nothing is masked.
    """
    mask = np.asarray(mask, bool)
    if not mask.any():
        return 1.0
    mv = np.asarray(m, float)[mask]
    fv = np.asarray(fixed, float)[mask]
    room = float(cap) - np.sign(mv) * fv        # distance to the rail s moves toward
    s = np.where(np.abs(mv) > 0.0, room / np.where(mv != 0.0, np.abs(mv), 1.0), 1.0)
    return float(np.clip(s, 0.0, 1.0).min())


# ---------------------------------------------------------------------------
# the box-constrained allocator -- SELECTABLE, and OFF unless a caller asks
# ---------------------------------------------------------------------------
# A memo, not a threshold: how many distinct (A_C, W, d) tables to keep. The
# table depends only on the coil mask, the metric and the tie-break scale, all
# of which are constant across a run in the shipping configuration, so one
# entry per live coil mask is the steady state.
_BOX_CACHE_MAX = 64
_BOX_CACHE = {}


def _box_metric(w, nm):
    """(W, W^(1/2)) from `w`: None -> I, a vector -> diag, a matrix -> symmetrised."""
    if w is None:
        eye = np.eye(nm)
        return eye, eye
    W = np.asarray(w, float)
    if W.ndim == 1:
        W = np.diag(W)
    lam, V = np.linalg.eigh(0.5 * (W + W.T))
    lam = np.clip(lam, 0.0, None)
    return (V * lam) @ V.T, (V * np.sqrt(lam)) @ V.T


def _box_table(A, Wh, ds):
    """One reduced solver per FREE SUBSET of the coils: 2^nc of them, built once.

    For a free set S the reduced problem is min ||A_S u_S - r||^2_W with the
    minimum-||u/d|| tie-break, whose solution is a fixed matrix times r. The
    pinned coils only move `r`, so the 3^nc active-set patterns need 2^nc
    matrices, not 3^nc.
    """
    nm, nc = A.shape
    AW = Wh @ A
    tab = []
    for s in range(1 << nc):
        S = np.array([j for j in range(nc) if (s >> j) & 1], int)
        P = np.array([j for j in range(nc) if not (s >> j) & 1], int)
        if S.size:
            B = AW[:, S] * ds[S][None, :]
            M = ds[S][:, None] * (np.linalg.pinv(B) @ Wh)      # (|S|, nm)
        else:
            M = np.zeros((0, nm))
        corners = np.array([[bool((k >> i) & 1) for i in range(P.size)]
                            for k in range(1 << P.size)],
                           bool).reshape(1 << P.size, P.size)
        tab.append((S, P, M, corners, A[:, P].T.copy()))
    return tab


def cap_box(m, fixed, cap, mask, a, w=None, d=None, kkt_tol=1e-9):
    """Box-constrained control allocation: the same TOTAL cap as `cap_scale`, per coil.

    Solves, exactly, per control step

        minimise  || A_C u - g ||^2_W    subject to   lo_j <= u_j <= hi_j

    where `g = A_C m` is the modal force today's uncapped allocation realises,
    and the box is the same TOTAL window `cap_scale` enforces:

        hi_j = max(0, cap - fixed_j)     lo_j = min(0, -cap - fixed_j)

    Returns `(u, pinned)`: the allocation over `mask` (zero elsewhere) and which
    coils came back sitting on a bound.

    WHY, AND THE NUMBER. `cap_scale` shrinks the whole vector by ONE scalar, so
    one coil near its rail throttles all four. The asymmetry is measured: static
    rigid-body force per coil is 9.92 / 29.80 / 34.28 / 28.93 counts/V, i.e.
    coil 0 is about 3x weaker than 1/2/3, and coil 0 is the one that saturated
    (CLAUDE.md, "The geometry, and what it settled"). This spends each coil's
    own remaining room instead.

    WHY IT IS STILL DISSIPATIVE -- the whole reason the change is safe. The
    reachable set C = {A_C u : u in box} is convex and contains 0 (the max/min
    above), so the W-projection p = Proj_C(g) satisfies
    (g - p)' W (c - p) <= 0 for every c in C; take c = 0 and it gives
    g' W p >= p' W p >= 0. The commanded force is g = -K~ qdot with
    K~ = Proj K Proj the projected per-mode gain (`Modal.project`: -Proj K Proj
    qdot is symmetric PSD, -Proj K qdot is not), so with W = pinv(K~),

        g' W p = -qdot' K~ pinv(K~) p = -qdot' p          (p is in range(K~))

    hence   qdot' p <= -p' W p < 0 for p != 0: power always leaves the plant,
    whatever the constraint does. THE METRIC IS LOAD-BEARING -- with W = I and a
    non-scalar K the middle line does not go through and nothing here is proven.
    `Modal.box_args` builds the right W; where it cannot it returns None, and the
    caller must keep `cap_scale`. On a full-rank mask Proj = I and W = diag(1/kp)
    exactly, so with `MODAL_KP` flat at [0.035, 0.035, 0.035] as it ships W is a
    multiple of the identity and cannot change the answer. It matters the moment a
    per-mode gain is not flat, which is CLAUDE.md 16's whole point.

    HOW IT IS SOLVED: exactly, in bounded time, no iteration and no solver. Every
    coil is free, pinned low or pinned high, so 3^nc patterns cover every KKT
    point -- 81 at nc = 4. Each is a precomputed reduced least-squares, rejected
    if a free coil lands outside its box or a pinned coil's gradient has the wrong
    sign, and the lowest-objective feasible candidate wins. All-pinned-low is
    always feasible, so a candidate always exists: no convergence risk at 100 Hz.

    WHAT IT IS WORTH, MEASURED OFFLINE ON THE RECORDED RUNS. Every modal DAMPING
    step of the 13 eta runs data/20260818_02*_fast_lock.csv and
    data/20260818_03*_fast_lock.csv -- 315 024 steps -- re-solved from the logged
    state, both allocators on the same inputs. THE BOX BINDS RARELY: 123 steps,
    0.04 %, and on exactly the same steps `cap_scale` binds. Where it binds it
    realises more force (||A_C u|| ratio median 1.067, p90 1.205, max 1.511, and
    >= 1.000 on all 123) and both laws dissipated on all 314 996 steps where the
    metric exists. On the 00:22 run CLAUDE.md 2 measured coil 3's 0.4229 V on,
    9914 modal steps, it binds on 0.40 % at ratio median 1.048, max 1.086. Wall
    time warm, 4 coils: median 0.30 ms, p99 0.38 ms against a 10 ms control step;
    the first call builds the table and cost 22 ms. OPEN LOOP -- this is what the
    two allocators would have COMMANDED on the same inputs, not what the plant
    would have done. NOTHING HERE SAYS IT DAMPS BETTER; only the bench can.

    TIES. The objective is strictly convex in the realised force p = A_C u, but
    A_C is 3x4 here so u is not unique. Ties break by minimum ||u/d||; with `d`
    from `Modal.box_args` that is the row-balancing metric `_build_tables` already
    uses, so a slack box returns today's `A_C^+ f` to floating point rather than a
    different point of the same objective. `d = None` is plain minimum norm.

    DEGENERATE CASES, handled as `cap_scale` handles them. `fixed` alone past the
    cap does not empty the box: it collapses to the interval between 0 and the
    value that brings the coil back inside, so the allocator can pull that coil
    toward the window and never push it further out. The invariant is

        |u_j + fixed_j| <= max(cap, |fixed_j|)

    -- never over the cap when `fixed` was inside it, never worse than doing
    nothing when it was already outside, which is the case `cap_scale` answers
    with a scale of 0.0 and leaves to `PID.drive`'s clip. An empty mask, a zero
    `f`, a rank-deficient A_C and fewer than four drivable coils all fall out of
    the same enumeration with no special case.
    """
    mask = np.asarray(mask, bool)
    u = np.zeros(mask.size)
    pinned = np.zeros(mask.size, bool)
    cols = np.flatnonzero(mask)
    if cols.size == 0:
        return u, pinned
    A = np.asarray(a, float)[:, cols]
    nm, nc = A.shape
    g = A @ np.asarray(m, float)[cols]
    fx = np.asarray(fixed, float)[cols]
    cap = float(cap)
    hi = np.maximum(0.0, cap - fx)
    lo = np.minimum(0.0, -cap - fx)
    ds = np.ones(nc) if d is None else np.abs(np.asarray(d, float)[cols])
    ds = np.where(ds > 0.0, ds, 1.0)
    Wm, Wh = _box_metric(w, nm)

    key = (A.tobytes(), Wh.tobytes(), ds.tobytes(), nm, nc)
    tab = _BOX_CACHE.get(key)
    if tab is None:
        if len(_BOX_CACHE) >= _BOX_CACHE_MAX:
            _BOX_CACHE.clear()
        tab = _BOX_CACHE[key] = _box_table(A, Wh, ds)

    U = np.zeros((3 ** nc, nc))
    code = np.zeros((3 ** nc, nc), np.int8)     # 0 free, 1 pinned lo, 2 pinned hi
    k = 0
    for S, P, M, corners, Apt in tab:
        nk = corners.shape[0]
        blk = np.zeros((nk, nc))
        if P.size:
            blk[:, P] = np.where(corners, hi[P][None, :], lo[P][None, :])
            r = g[None, :] - blk[:, P] @ Apt
            code[k:k + nk, P] = np.where(corners, 2, 1)
        else:
            r = np.repeat(g[None, :], nk, axis=0)
        if S.size:
            blk[:, S] = r @ M.T
        U[k:k + nk] = blk
        k += nk

    U, code = U[:k], code[:k]
    resid = U @ A.T - g[None, :]
    RW = resid @ Wm
    obj = np.einsum("ij,ij->i", RW, resid)
    grad = 2.0 * (RW @ A)                       # d/du of the objective
    span = max(1.0, float(np.max(np.abs(hi - lo))))
    tolb = 1e-9 * span
    feas = np.all((U >= lo[None, :] - tolb) & (U <= hi[None, :] + tolb), axis=1)
    gt = kkt_tol * max(1.0, float(np.max(np.abs(grad))) if grad.size else 1.0)
    kkt = np.all(np.where(code == 2, grad <= gt,
                          np.where(code == 1, grad >= -gt, True)), axis=1)
    pool = feas & kkt
    if not pool.any():
        pool = feas                             # always non-empty: all-pinned-low
    # Minimise the objective, THEN break the tie on ||u/d||. Two stages, not one
    # perturbed objective: A_C is 3x4 here so ties are the normal case, not an
    # edge, and a `f = 0` step must come back with u = 0 rather than with
    # whichever corner the enumeration happened to visit first.
    obj_pool = np.where(pool, obj, np.inf)
    best = float(np.min(obj_pool))
    tie = pool & (obj <= best + 1e-12 * max(1.0, float(g @ Wm @ g)))
    nrm = np.einsum("ij,ij->i", U / ds[None, :], U / ds[None, :])
    sol = np.clip(U[int(np.argmin(np.where(tie, nrm, np.inf)))], lo, hi)
    u[cols] = sol
    pinned[cols] = (sol <= lo + tolb) | (sol >= hi - tolb)
    return u, pinned


# ===========================================================================
# the diagonal law
# ===========================================================================
class PID:
    """Per-channel P/I/D on velocity, with anti-windup, clip and slew.

    Back-calculation anti-windup: the integrator unwinds at track_tc from LAST
    sample's clip rather than freezing at a limit. Stepped only where ki != 0 --
    with ki = 0 the back-calculation is a one-way ratchet and one clipped sample
    would leave a standing offset of up to i_clamp against a 0.250 V half-window.
    """

    def __init__(self, n, ki, kd, d_hz, i_clamp, track_tc_s, slew_per_s):
        self.n = int(n)
        self.ki, self.kd = np.asarray(ki, float), np.asarray(kd, float)
        self.i_clamp, self.track_tc = float(i_clamp), float(track_tc_s)
        self.slew = float(slew_per_s)
        self.dfilt = OnePole(d_hz, "low", self.n)
        for k in "p i d prev_vel clip_excess demand".split():
            setattr(self, k, np.zeros(self.n))
        self.primed = np.zeros(self.n, bool)
        self.out = self.prev_out = np.zeros(self.n)

    def bias(self, bias):
        """Bumpless start: hold the last commanded output at bias."""
        self.out = np.array(bias, float)
        self.prev_out = self.out.copy()

    def terms(self, vel, gain, dt):
        """err = -vel (the setpoint is zero velocity). Returns (p, i, d)."""
        err = -np.asarray(vel, float)
        self.p = gain * err
        self.prev_vel[~self.primed] = vel[~self.primed]
        self.primed[:] = True
        dv = (vel - self.prev_vel) / dt if dt > 0 else np.zeros(self.n)
        self.prev_vel = np.array(vel, float)
        self.d = -self.kd * self.dfilt.update(dv, dt)      # D on the measurement
        self.i = np.where(
            self.ki != 0.0,
            np.clip(self.i + (self.ki * err - self.clip_excess / self.track_tc) * dt,
                    -self.i_clamp, self.i_clamp),
            0.0)
        return self.p, self.i, self.d

    def drive(self, raw, bias, vmin, vmax, live, dt):
        """Clip, record the excess for the anti-windup, slew-limit, return the output."""
        clipped = np.clip(raw, vmin, vmax)
        self.clip_excess = np.where(live, raw - clipped, 0.0)
        self.demand = np.abs(raw - bias)
        dead = ~live
        if dead.any():
            self.p[dead] = self.i[dead] = self.d[dead] = self.prev_vel[dead] = 0.0
            self.primed[dead] = False
            self.dfilt.reset(dead)
        cap = self.slew * dt
        target = np.where(live, clipped, bias)
        self.out = self.prev_out + np.clip(target - self.prev_out, -cap, cap)
        self.prev_out = self.out.copy()
        return self.out

    def reset(self, mask=slice(None)):
        for k in "p i d prev_vel clip_excess".split():
            getattr(self, k)[mask] = 0.0
        self.primed[mask] = False
        self.dfilt.reset(mask)


# ===========================================================================
# the floor: calibrate, persist, fingerprint, sanity-check
# ===========================================================================
class Baseline:
    """Measures the floor in sub-windows and stops as soon as they agree.

    A calibration window can be perfectly STATIONARY and still sit far above the
    floor, so a fresh measurement more than `sanity_ratio` above a trusted one is
    a ringdown and is refused -- up to `max_refusals` times, after which the
    floor really has moved.
    """

    def __init__(self, n, subwindow_s, n_subwindows, min_subwindows, agree_n,
                 agree_tol, sanity_ratio, max_refusals, floor_frac):
        self.n = int(n)
        self.subwindow_s = float(subwindow_s)
        self.n_subwindows = int(n_subwindows)
        self.min_subwindows = int(min_subwindows)
        self.agree_n, self.agree_tol = int(agree_n), float(agree_tol)
        self.sanity_ratio = float(sanity_ratio)
        self.max_refusals = int(max_refusals)
        self.floor_frac = float(floor_frac)
        self.trusted = None
        self.refusals = 0
        self.refused = False
        self.acc, self.sub_rms, self.sub_n0, self.sub_start = [], [], 0, 0.0

    def start(self, t):
        self.acc, self.sub_rms, self.sub_n0, self.sub_start = [], [], 0, t

    def add(self, x):
        self.acc.append(x)

    def close_subwindow(self, t):
        a = np.asarray(self.acc[self.sub_n0:])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a ** 2).mean(0)))
        self.sub_n0, self.sub_start = len(self.acc), t

    def stationary(self):
        """The last agree_n sub-windows agree, and agree with the run as a whole."""
        w = self.sub_rms
        if len(w) < max(self.min_subwindows, self.agree_n):
            return False
        a = np.asarray(w)
        tail = a[-self.agree_n:]
        if not bool((tail.max(0) / np.maximum(tail.min(0), 1e-9)
                     <= self.agree_tol).all()):
            return False
        m_tail, m_all = np.median(tail, 0), np.median(a, 0)
        spread = (np.maximum(m_tail, m_all)
                  / np.maximum(np.minimum(m_tail, m_all), 1e-9))
        return bool((spread <= self.agree_tol).all())

    def measure(self, early):
        """The fresh floor: median of the settled sub-windows, or of all of them."""
        if early and len(self.sub_rms) >= self.agree_n:
            return np.maximum(
                np.median(np.asarray(self.sub_rms[-self.agree_n:]), 0), 1e-6)
        a = np.asarray(self.acc)
        parts = ([p for p in np.array_split(a, self.n_subwindows) if len(p)]
                 if len(a) else [])
        return (np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0),
                           1e-6)
                if parts else np.full(self.n, 1e-6))

    def settle(self, early):
        """(baseline, note_or_None). Refuses a fresh floor far above a trusted one."""
        fresh = self.measure(early)
        ref, self.refused = self.trusted, False
        note = None
        if ref is not None:
            live = ref > 0
            med = np.median(ref[live]) if live.any() else 0.0
            m = live & (ref >= med * self.floor_frac)
            bad = m & (fresh > self.sanity_ratio * ref)
            if bad.any() and self.refusals >= self.max_refusals:
                note = ("!! fresh baseline refused %d times running and accepted "
                        "anyway -- the floor really has moved, this is no longer a "
                        "ringdown. The runaway breaker is now scaled off it."
                        % self.refusals)
                bad = np.zeros(self.n, bool)
            elif bad.any():
                self.refusals += 1
                self.refused = True
                note = ("!! fresh baseline REFUSED -- "
                        + " / ".join("ch%d measured %.4fV against a trusted %.4fV "
                                     "(%.1fx)" % (i, fresh[i], ref[i],
                                                  fresh[i] / ref[i])
                                     for i in np.nonzero(bad)[0])
                        + ", over %.1fx. That is a ringdown, not a floor. Keeping "
                          "the baseline already in hand." % self.sanity_ratio)
                self.start(0.0)
                return np.maximum(ref, 1e-6), note
        self.trusted, self.refusals = fresh.copy(), 0
        self.start(0.0)
        return fresh, note


def fingerprint(**fields):
    """The configuration a stored floor was measured under. Any change refuses it."""
    out = {}
    for k, v in fields.items():
        if isinstance(v, bool):
            out[k] = bool(v)
        elif isinstance(v, (list, tuple, np.ndarray)):
            out[k] = [bool(x) if isinstance(x, (bool, np.bool_))
                      else round(float(x), 9) for x in v]
        elif isinstance(v, (int, np.integer)):
            out[k] = int(v)
        elif isinstance(v, float):
            out[k] = float(v)
        else:
            out[k] = v
    return out


def same(a, b, tol=1e-9):
    if isinstance(a, list) != isinstance(b, list):
        return False
    if isinstance(a, list):
        return len(a) == len(b) and all(same(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def save_baseline(path, payload):
    """Atomic write, so a crash cannot leave half a file for the next start-up."""
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
    os.replace(tmp, path)
    return payload


def load_baseline(path, schema, want, n, max_age_s, now, fresh=False,
                  calibration_s=20.0, control_hz=100.0, warmup_s=2.0,
                  sanity_ratio=3.0):
    """((baseline, floor_bad), report) or (None, report). Every refusal says why."""
    say = ["stored baseline: %s" % path]
    if fresh:
        say.append("  IGNORED -- fresh calibration forced (--fresh / "
                   "OSEM_FRESH_CALIB). Measuring the floor the long way.")
        return None, say
    try:
        with open(path) as fh:
            d = json.load(fh)
    except FileNotFoundError:
        say.append("  none yet. Calibrating for up to %.0fs and writing one at "
                   "the end." % calibration_s)
        return None, say
    except (OSError, ValueError) as e:
        say.append("  UNREADABLE (%s) -- ignored, calibrating from scratch." % e)
        return None, say

    def no(why):
        say.append("  REFUSED -- %s. Calibrating from scratch (up to %.0fs)."
                   % (why, calibration_s))
        return None, say

    if d.get("schema") != schema:
        return no("schema %r, this build reads %r" % (d.get("schema"), schema))
    age = now - float(d.get("measured_unix", 0.0))
    say.append("  written by %s at %s, %.0fs ago"
               % (d.get("written_by", "?"), d.get("measured_utc", "?"), age))
    if not (0 <= age <= max_age_s):
        return no("%.0fs old against a %.0fs limit" % (age, max_age_s) if age >= 0
                  else "measured %.0fs in the FUTURE -- the clock moved" % -age)
    have = d.get("config", {})
    extra = sorted(set(have) - set(want))
    if extra:
        return no("the file records %s, which this build does not know -- it was "
                  "written against a different velocity path, so its floor is an "
                  "RMS of a different signal"
                  % ", ".join("`%s`" % k for k in extra))
    for k in sorted(want):
        if k not in have:
            return no("the file does not record `%s`" % k)
        if not same(have[k], want[k]):
            return no("`%s` differs: file %r, this build %r" % (k, have[k], want[k]))
    fw = d.get("wire_hz")
    b = d.get("baseline_v") or []
    fl = d.get("floor_bad") or [False] * len(b)
    if len(b) != n or len(fl) != n:
        return no("%d baselines and %d floor flags, expected %d" % (len(b), len(fl), n))
    if not all(isinstance(x, (int, float)) and np.isfinite(x) and x > 0 for x in b):
        return no("a baseline is zero, negative or not finite")
    say.append("  " + ", ".join("ch%d=%.4fV%s" % (i, v, "(NOSIG)" if fl[i] else "")
                                for i, v in enumerate(b)))
    say.append("  measured at %s on the wire, %.0f Hz control step, calibration "
               "took %.1fs" % (("%.0f Hz" % fw) if fw else "an unrecorded rate",
                               control_hz, d.get("calib_took_s") or float("nan")))
    say.append("  ACCEPTED provisionally. Adopted only if the first %.1fs of live "
               "signal agree with it to within %.1fx, per channel."
               % (warmup_s, sanity_ratio))
    return (np.array(b, float), np.array(fl, bool)), say


def baseline_payload(schema, tag, baseline, floor_bad, calib_took_s, calib_early,
                     wire_hz, config, now):
    return dict(schema=schema, written_by=tag, measured_unix=now,
                measured_utc=datetime.fromtimestamp(now, timezone.utc)
                .isoformat(timespec="seconds"),
                baseline_v=[float(v) for v in baseline],
                floor_bad=[bool(v) for v in floor_bad],
                calib_took_s=(None if calib_took_s is None else float(calib_took_s)),
                calib_early=bool(calib_early),
                wire_hz=(float(wire_hz) if wire_hz == wire_hz else None),
                config=config)


# ===========================================================================
# io
# ===========================================================================
class Recorder:
    """Line-buffered CSV; the header is assembled from the columns present.

    Standing practice: every raw sample goes to disk as it arrives, closed in a
    `finally`. Two real defects were found offline in files already written.
    """

    def __init__(self, path, cols, flush_every=200):
        self.path, self.cols = path, list(cols)      # [(name, fmt)]
        self.flush_every, self.rows = int(flush_every), 0
        self.fh = open(path, "w", buffering=1)
        self.fh.write(self.header + "\n")

    @property
    def header(self):
        return ",".join(name for name, _ in self.cols)

    def write(self, values):
        out = []
        for (_, fmt), v in zip(self.cols, values):
            out.append(v if fmt is None else format(v, fmt))
        self.fh.write(",".join(out) + "\n")
        self.rows += 1
        if self.rows % self.flush_every == 0:
            self.fh.flush()

    def close(self):
        try:
            self.fh.close()
        except OSError:
            pass


class Console:
    """The one-line status print, on a wall-clock period."""

    def __init__(self, period_s):
        self.period = float(period_s)
        self.last = -1e18

    def due(self, now):
        if now - self.last < self.period:
            return False
        self.last = now
        return True

    @staticmethod
    def line(t, state, rate, extra, per_channel):
        return ("[%7.1fs] %-11s %s %s " % (t, state, rate, extra)
                + "  ".join(per_channel))


# ===========================================================================
# the state machine skeleton
# ===========================================================================
class Loop:
    """CALIBRATING -> DAMPING -> FAULT, with the FAULT hold in one place.

    LOCKED is an annotation on DAMPING, not a fourth state: the loop keeps
    damping after it locks, and the deliverable is the TIME it announced.
    """

    def __init__(self):
        self.state = "CALIBRATING"
        self.events = []
        self.fault_count = 0
        self.clear_since = None
        self.lock_time = None
        self.locked_announced = False

    def say(self, msg):
        self.events.append("\n" + msg + "\n")

    def drain_events(self):
        out, self.events = self.events, []
        return out

    def clear_gate(self, t, blocked, sustain_s):
        """True once `blocked` has been continuously false for sustain_s."""
        if blocked:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = t
        elif t - self.clear_since >= sustain_s:
            return True
        return False


def share(cls, owner, names):
    """Expose `owner`'s arrays on `cls` under their own names, get and set."""
    for k in names.split():
        def get(self, o=owner, k=k):
            return getattr(getattr(self, o), k)

        def put(self, v, o=owner, k=k):
            setattr(getattr(self, o), k, v)
        setattr(cls, k, property(get, put))


# ===========================================================================
# selftest: the pure-math pieces only
# ===========================================================================
def _selftest():
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("   %-56s %s%s" % (name, "PASS" if cond else "FAIL",
                                 ("   " + detail) if detail else ""))

    print("\n  === stdlib.py selftest ===\n")
    rng = np.random.default_rng(20260817)
    n, nm = 8, 3
    f = np.array([0.72294, 0.99193, 1.65657])
    dt = 0.01
    r = np.full(n, 1e-4)
    q = np.full((n, nm), 1e-3)

    # ---- exact ZOH ----------------------------------------------------------
    mk = ModalKalman(np.ones((n, nm)), np.ones((n, nm), bool), r, q,
                     r / 20.0, f, dt, n, 50.0)
    mk._transition(dt)
    worst = 0.0
    for mi in range(nm):
        B = mk.Phi[2 * mi:2 * mi + 2, 2 * mi:2 * mi + 2]
        st = np.array([1.0, 0.0])
        e0 = mk.w[mi] ** 2 * st[0] ** 2 + st[1] ** 2
        for _ in range(100000):
            st = B @ st
        worst = max(worst, abs((mk.w[mi] ** 2 * st[0] ** 2 + st[1] ** 2) / e0 - 1.0))
    check("exact-ZOH conserves oscillator energy, 1e5 steps", worst < 1e-9,
          "worst drift %.2e" % worst)
    eu = np.array([[1.0, dt], [-mk.w[0] ** 2 * dt, 1.0]])
    st = np.array([1.0, 0.0])
    for _ in range(100000):
        st = eu @ st
    drift = abs((mk.w[0] ** 2 * st[0] ** 2 + st[1] ** 2) / mk.w[0] ** 2 - 1.0)
    check("...and Euler at the same dt does not", drift > 1e-3, "%.3e" % drift)

    kv = KalmanVelocity(np.zeros((n, 2 * nm + 1)), n, f, dt)
    kv.Phi = kv._transition(dt)
    st = np.zeros(2 * nm + 1)
    st[0] = 1.0
    e0 = (kv.w[0] * st[0]) ** 2 + st[1] ** 2
    for _ in range(20000):
        st = kv.Phi @ st
    check("KalmanVelocity uses the same exact-ZOH block",
          abs(((kv.w[0] * st[0]) ** 2 + st[1] ** 2) / e0 - 1.0) < 1e-9)

    # ---- chi2 ---------------------------------------------------------------
    phi = np.sign(rng.normal(size=(n, nm))) * (0.5 + rng.random((n, nm)))
    mk = ModalKalman(phi, np.ones((n, nm), bool), r, q, r / 20.0, f, dt, n, 50.0)
    mk._transition(dt)
    B = mk.Phi[:2 * nm, :2 * nm]
    L = np.linalg.cholesky(mk.Qd[:2 * nm, :2 * nm])
    live = np.ones(n, bool)

    def run(corrupt=None, steps=6000):
        mk.reset()
        mk._transition(dt)
        xt = np.zeros(2 * nm)
        xt[0:2 * nm:2] = np.sqrt(mk.var_q)
        dc = np.zeros(n)
        sr = np.sqrt(mk.r)
        rg = np.random.default_rng(7)
        acc = []
        for k in range(steps):
            xt = B @ xt + L @ rg.normal(size=2 * nm)
            dc = dc + np.sqrt(mk.q_dc * dt) * rg.normal(size=n)
            y = phi @ xt[0:2 * nm:2] + dc + sr * rg.normal(size=n)
            if corrupt is not None:
                i, kind, amt = corrupt
                if kind == "offset":
                    y[i] += amt
                elif kind == "gain":
                    y[i] *= amt
            mk.update(y, live, dt)
            if k > steps // 2:
                acc.append(mk.chi2_per_dof())
        acc = np.asarray(acc)
        return float(np.nanmean(acc))

    base = run()
    check("chi2/dof is order 1 when the model fits", 0.7 < base < 1.4,
          "mean %.4f over %d dof" % (base, n))
    off = run(corrupt=(3, "offset", 0.5))
    check("a STATIC offset is invisible -- the DC state absorbs it, by design",
          abs(off - base) < 0.15, "%.4f against %.4f" % (off, base))
    gn = run(corrupt=(3, "gain", 1.5))
    check("a sensor whose GAIN is wrong by 1.5x is not", gn > 2.0 * base,
          "%.3f against %.3f" % (gn, base))

    # ---- allocator ----------------------------------------------------------
    import tempfile
    # A well-conditioned 3x4 over coils 0-3, so full row rank is reachable and
    # the balancing has something to equalise. Rows orthonormal -> cond 1.
    A = np.zeros((nm, n))
    A[:, :4] = np.linalg.qr(rng.normal(size=(4, 4)))[0][:, :nm].T
    okrow = np.zeros((n, nm), bool)
    okrow[:6] = True
    doc = dict(schema="osem-modal-1",
               created=datetime.now().isoformat(timespec="seconds"),
               source="stdlib-selftest", n_sensors=n, n_modes=nm,
               modes=[dict(name=c, f_hz=float(x)) for c, x in zip("ABC", f)],
               phi=dict(value=phi.tolist(), sigma=np.full((n, nm), 0.01).tolist(),
                        ok=okrow.tolist(), imag_frac=[0.0] * nm, gauge="s"),
               a=dict(value=A.tolist(), coils=[0, 1, 2, 3], gauge="s",
                      provisional=False, provenance="s"))
    p = os.path.join(tempfile.gettempdir(), "stdlib_selftest_modal.json")
    with open(p, "w") as fh:
        json.dump(doc, fh)
    M = Modal(p, n, f, "osem-modal-1")
    check("a consistent Phi/A pair loads", M.ok, M.why)
    drive = np.zeros(n, bool)
    drive[[0, 1, 2, 3]] = True
    fdem = -np.array([0.02, 0.01, 0.015])
    u, _mask, cm, gok = M.allocate(fdem, drive)
    got = M.a[:, [0, 1, 2, 3]] @ u[[0, 1, 2, 3]]
    check("allocation is exact at full row rank (A P = I)",
          gok and np.allclose(got, fdem, rtol=1e-8, atol=1e-12),
          "%s vs %s" % (np.round(got, 6), np.round(fdem, 6)))
    rn = np.linalg.norm(M.pinv[0b1111][1], axis=1)
    check("the balanced allocator still satisfies A P = I, and equalises row norms",
          M.balanced[0b1111] and M.rank[0b1111] == nm
          and rn.max() / rn.min() < 1.05,
          "rank %d, row-norm spread %.4f" % (M.rank[0b1111], rn.max() / rn.min()))
    check("allocate reports which coils it commanded",
          list(np.nonzero(cm)[0]) == [0, 1, 2, 3], str(np.nonzero(cm)[0]))
    qd = np.array([0.3, -0.12, 0.07])
    proj, rk = M.project(qd, drive)
    power = float((M.a[:, [0, 1, 2, 3]]
                   @ M.allocate(-0.035 * proj, drive)[0][[0, 1, 2, 3]]) @ qd)
    check("the realised modal force dissipates (f . qdot < 0)", power < 0,
          "%+.6f, rank %d" % (power, rk))
    check("colocation is computed and logged, never a gate",
          M.ok and any("colocation" in s for s in M.notes),
          "%d note(s)" % len(M.notes))

    # ---- a geometric Phi, and the measured one as a CHECK on it -------------
    # Four coplanar corner sensors on one axis: three rigid-body columns plus a
    # warp direction no rigid body produces. Test data here, not a rig constant.
    G = np.zeros((n, nm))
    G[:4, 0], G[:4, 1], G[:4, 2] = ([+1, +1, -1, -1], [+1, +1, +1, +1],
                                    [+1, -1, +1, -1])
    warp = np.zeros(n)
    warp[:4] = [+1, -1, -1, +1]
    Mg = Modal(None, n, f, "osem-modal-1", phi=G, a=A, a_coils=[0, 1, 2, 3],
               a_provisional=False)
    check("a geometric Phi loads with no file, no gauge and no jackknife",
          Mg.ok and Mg.basis == "geometric", Mg.why)
    check("...with rows determined exactly where it is non-zero",
          bool(Mg.rowok[:4].all()) and not bool(Mg.rowok[4:].any()))
    check("...and a supplied A stays refusable: provisional needs allowing",
          not Modal(None, n, f, "osem-modal-1", phi=G, a=A, a_coils=[0, 1, 2, 3],
                    a_provisional=True).ok)

    def geo_file(phi_value, basis="geometric"):
        e = json.loads(json.dumps(doc))
        e["phi"]["value"] = np.asarray(phi_value, float).tolist()
        e["a"]["basis"] = basis
        pp = os.path.join(tempfile.gettempdir(), "stdlib_selftest_geo.json")
        with open(pp, "w") as fh:
            json.dump(e, fh)
        return pp

    near = G + 0.08 * rng.normal(size=(n, nm)) * (G != 0)
    Mn = Modal(geo_file(near), n, f, "osem-modal-1", phi=G, a_provisional=False)
    check("a measured Phi that agrees is a confirmation, not an input",
          Mn.ok and not Mn.phi_cos_bad and float(Mn.phi_cos.min()) > 0.90,
          "|cos| %s" % np.array2string(Mn.phi_cos, precision=3))
    wrong = near.copy()
    wrong[:4, 2] = warp[:4]                       # mode C handed the warp direction
    Mw = Modal(geo_file(wrong), n, f, "osem-modal-1", phi=G, a_provisional=False)
    check("a measured Phi that disagrees warns loudly and still is not a gate",
          Mw.ok and Mw.phi_cos_bad and Mw.phi_cos[2] < 0.90
          and any(s.startswith("!!") for s in Mw.notes),
          "|cos| %s" % np.array2string(Mw.phi_cos, precision=3))
    Mga = Modal(geo_file(near), n, f, "osem-modal-1")
    check("A and Phi are never paired across bases: geometric A, measured Phi",
          not Mga.ok, Mga.why[:56])
    Mma = Modal(geo_file(near, basis=""), n, f, "osem-modal-1", phi=G)
    check("...nor measured A with a geometric Phi", not Mma.ok, Mma.why[:56])

    # ---- decimation ---------------------------------------------------------
    d = Decimator(100.0, n, mains_null=False)
    got, wire_hz = 0, 1000.0
    for k in range(int(2.0 * wire_hz)):
        if d.feed(np.full(n, 100.0 + k), np.full(n, 1.0), k / wire_hz) is not None:
            got += 1
    check("decimation runs at CONTROL_HZ off any wire rate",
          abs(got - 200) <= 1 and d.n_avg == 10,
          "%d steps in 2.0s, %d samples averaged per step" % (got, d.n_avg))
    d2 = Decimator(100.0, n, mains_null=True)
    out = None
    for k in range(300):
        got2 = d2.feed(np.full(n, 10.0 * (k % 2)), np.full(n, 10.0 * (k % 2)),
                       k / 200.0)
        if got2 is not None:
            out = got2
    check("mains-null averages two consecutive control steps",
          out is not None and abs(float(out[1][0]) - 5.0) < 1e-9,
          "alternating 0/10 -> %.3f" % float(out[1][0]))

    # ---- sample guard -------------------------------------------------------
    g = SampleGuard(n, 5.02, 1023)
    check("sample guard accepts 0..1023", g.in_range(np.array([0.0, 1023.0])))
    check("...and rejects anything outside it",
          not g.in_range(np.array([522676.0])) and not g.in_range(np.array([-1.0])))

    class _Ser:
        def __init__(self, lines):
            self.lines = list(lines)

        @property
        def in_waiting(self):
            return len(self.lines)

        def readline(self):
            return self.lines.pop(0)

    s = _Ser([b"1,2,3,4,5,6,7,8\n", b"OK ch=0\n", b"1,2,3,4,5,6,7,522676\n",
              b"5,5,5,5,5,5,5,5\n"])
    seen = [g.read(s) for _ in range(4)]
    check("a torn row is rejected and counted, an ack is not",
          g.rejected == 1 and seen[0] is not None and seen[1] is None
          and seen[2] is None and seen[3] is not None,
          "%d rejected" % g.rejected)

    # THE BINARY PATH. Its absence was a silent bug on 2026-08-20 -- see the
    # class docstring -- so it is covered here, not merely written.
    class _BinDac:
        """Quacks like FastDAC: `binary` True and a `read_sample(n)`."""

        def __init__(self, rows):
            self.binary = True
            self.rows = list(rows)

        def read_sample(self, ncols):
            return self.rows.pop(0) if self.rows else None

    gb = SampleGuard(n, 5.02, 1023)
    bd = _BinDac([[1, 2, 3, 4, 5, 6, 7, 8], None,
                  [1, 2, 3, 4, 5, 6, 7, 522676], [5] * 8])
    bseen = [gb.read(bd) for _ in range(4)]
    check("the sample guard reads the BINARY transport, not only ASCII",
          bseen[0] is not None and bseen[3] is not None
          and float(bseen[0][0][0]) == 1.0,
          "a binary-mode DAC decoded as text returns None forever, which looks "
          "exactly like a dead rig -- header-only CSV, no printed output")
    check("...and still range-checks it: the guarantee is transport independent",
          bseen[1] is None and bseen[2] is None and gb.rejected == 1,
          "%d rejected out of one torn frame and one empty read" % gb.rejected)
    check("...and a bare Serial still works, for the ASCII callers",
          SampleGuard(n, 5.02, 1023).read(
              _Ser([b"9,9,9,9,9,9,9,9\n"])) is not None,
          "getattr(dac, 'binary', False) is the whole test, and Serial has no "
          "such attribute")

    # ---- in-band floor ------------------------------------------------------
    ib = InBand(f, n)
    tt = np.arange(0, 20.0, 0.01)
    for k, t in enumerate(tt):
        y = np.zeros(n)
        y[0] = 3.3 + 1.0 * np.sin(2 * np.pi * f[1] * t)          # in band
        y[1] = 3.3 + 1.0 * np.sin(2 * np.pi * 6.19 * t)          # out of band
        y[2] = 3.3                                                # DC only
        ib.add(t, y)
    amp = ib.rms()
    check("in-band lock-in keeps an in-band tone", abs(amp[0] - 0.7071) < 0.01,
          "%.4f V rms of a 1 V tone" % amp[0])
    check("...rejects 6.19 Hz by >30x", amp[1] < amp[0] / 30.0,
          "%.5f V against %.4f V" % (amp[1], amp[0]))
    check("...and does not see a DC offset at all", amp[2] < 1e-6, "%.2e" % amp[2])

    # ---- health / breaker ---------------------------------------------------
    # stdlib holds no constants, so every threshold is spelled out here once.
    def health(nn=n):
        return Health(nn, 12, 1011, 0.5, 0.8, 1.3, 0.8, 1.3, 2.0, 0.10)

    def breaker(nn=n, sustain=4.0):
        return Breaker(nn, 2.0, 1.8, sustain, 10.0, 1.02, 0.98, 1)

    h = health()
    rgen = np.random.default_rng(3)
    for k in range(1200):
        c = np.full(n, 600.0) + 5.0 * rgen.normal(size=n)
        c[6] = 4.3 + 0.5 * rgen.normal()          # bottom-railed, motionless
        c[7] = 3.0 if (k % 40) < 34 else 400.0    # railed but MOVING
        h.rails(c, k / 400.0)
    dead, witness = h.dead_pin(np.ones(n, bool))
    check("dead-pin: std separates an unwired pin from a railed-but-moving one",
          dead[6] and not dead[7] and witness,
          "ch6 std %.3f, ch7 std %.1f, line %.2f"
          % (h.rail_std[6], h.rail_std[7], h.dead_std))
    h2 = health()
    for k in range(1200):
        h2.rails(np.zeros(n), k / 400.0)
    dead2, witness2 = h2.dead_pin(np.ones(n, bool))
    check("...and demotes nothing when there is no witness left",
          not dead2.any() and not witness2)
    x = np.array([1.0, 1.0, 1.0, 1.0, 0.02, 1.0, 1.0, 1.0])
    bad, med, wit = h.floor(x, np.ones(n, bool))
    check("floor demotion is relative to the median, with a witness rule",
          bad[4] and bad.sum() == 1 and wit, "median %.3f" % med)
    bad2, _, wit2 = h.floor(np.full(n, 1.0), np.zeros(n, bool))
    check("...and refuses to run with nothing enabled", not bad2.any() and wit2)

    b = breaker()
    b.arm(0.0)
    base_v = np.full(n, 0.01)
    tripped = None
    for k in range(4000):
        t = 2.0 + k * 0.01
        amp = 0.01 * np.exp(0.15 * (t - 2.0))
        got = b.sample(t, np.full(n, amp), np.ones(n, bool), np.ones(n, bool),
                       base_v, np.zeros(n, bool))
        if got["run_trip"].any() and tripped is None:
            tripped = t
    check("the breaker trips on GROWTH sustained past the line",
          tripped is not None, "at t=%.1fs" % (tripped or -1))
    b2 = breaker()
    b2.arm(0.0)
    hot = False
    for k in range(4000):
        t = 2.0 + k * 0.01
        got = b2.sample(t, np.full(n, 0.05), np.ones(n, bool), np.ones(n, bool),
                        base_v, np.zeros(n, bool))
        hot = hot or bool(got["run_trip"].any())
    check("...and not on a level that is 5x the floor but flat", not hot)

    # ---- PID ----------------------------------------------------------------
    pid = PID(n, np.zeros(n), np.zeros(n), 2.0, 0.15, 0.5, 2.0)
    pid.bias(np.full(n, 0.25))
    live = np.ones(n, bool)
    pid.terms(np.full(n, 1.0), np.full(n, -0.035), 0.01)
    p_up = pid.p.copy()
    pid.terms(np.full(n, -1.0), np.full(n, -0.035), 0.01)
    check("P is gain * (-velocity): the coil polarity lives in the gain sign",
          abs(p_up[0] - 0.035) < 1e-12 and abs(pid.p[0] + 0.035) < 1e-12,
          "vel +1 -> %+.4f, vel -1 -> %+.4f" % (p_up[0], pid.p[0]))
    out = pid.drive(np.full(n, 5.0), np.full(n, 0.25), np.zeros(n),
                    np.full(n, 0.5), live, 0.01)
    check("the slew limit bounds one step", abs(out[0] - 0.25) <= 2.0 * 0.01 + 1e-12,
          "%.4f V after one 10 ms step" % out[0])
    check("...and the clip excess is recorded for the anti-windup",
          abs(pid.clip_excess[0] - 4.5) < 1e-9, "%.3f V" % pid.clip_excess[0])
    pidi = PID(n, np.full(n, 0.01), np.zeros(n), 2.0, 0.15, 0.5, 2.0)
    pidi.bias(np.full(n, 0.25))
    for _ in range(500):
        pidi.terms(np.full(n, 1.0), np.full(n, -0.035), 0.01)
        pidi.drive(np.full(n, 5.0), np.full(n, 0.25), np.zeros(n),
                   np.full(n, 0.5), live, 0.01)
    check("anti-windup holds the integrator inside its clamp",
          abs(pidi.i[0]) <= 0.15 + 1e-9, "%.4f V" % pidi.i[0])

    # ---- loop / recorder ----------------------------------------------------
    lp = Loop()
    got = [lp.clear_gate(t, t < 3.0, 5.0) for t in np.arange(0, 12.0, 0.5)]
    check("the fault hold clears sustain_s after the block goes away",
          not any(got[:16]) and got[-1], "cleared at t=%.1fs"
          % (0.5 * got.index(True) if True in got else -1))
    lp2 = Loop()
    check("...and never while it is still blocked",
          not any(lp2.clear_gate(t, True, 5.0) for t in np.arange(0, 30.0, 0.5)))

    rp = os.path.join(tempfile.gettempdir(), "stdlib_selftest.csv")
    rec = Recorder(rp, [("time_s", ".3f"), ("state", None), ("ch0_v", ".4f")], 1)
    rec.write([1.5, "DAMPING", 0.25])
    rec.close()
    with open(rp) as fh:
        lines = fh.read().strip().split("\n")
    check("the recorder header is the columns it was given",
          lines[0] == "time_s,state,ch0_v" and lines[1] == "1.500,DAMPING,0.2500",
          lines[1])

    # ---- the running quiet level --------------------------------------------
    print()
    ql = QuietLevel(4, 30.0, 20.0, 10.0, 0.80)
    check("a quiet level is BLIND until its window fills, and says so with None",
          ql.level() is None and not ql.ready(), "0 samples")
    tq, qm = 0.0, np.ones(4, bool)
    while tq < 20.0:                     # 20 s of a 30 s window: still blind
        ql.add(tq, np.full(4, 0.10), qm)
        tq += 0.01
    check("...still blind at 20s of a 30s window, so the caller keeps the baseline",
          ql.level() is None, "%d samples, wants %d" % (len(ql.hist), ql.want))
    while tq < 34.0:
        ql.add(tq, np.full(4, 0.10), qm)
        tq += 0.01
    check("...and arms once it is full, at the level the loop actually held",
          ql.level() is not None and abs(ql.level()[0] - 0.10) < 1e-9,
          "%.4f V" % ql.level()[0])
    # A kick for the last 10 s of the window: a third of it. A mean would move.
    for _ in range(1000):
        ql.add(tq, np.full(4, 4.0), qm)
        tq += 0.01
    lv, mean = ql.level()[0], (0.10 * 2 + 4.0 * 1) / 3.0
    check("A KICK OVER A THIRD OF THE WINDOW DOES NOT MOVE IT -- the whole point",
          abs(lv - 0.10) < 1e-9 and mean > 10 * lv,
          "%.4f V held; the mean of the same window is %.4f V" % (lv, mean))
    ql.reset()
    check("a re-engagement resets it: a new loop has a new floor",
          ql.level() is None and not ql.ready())
    ql2 = QuietLevel(4, 30.0, 20.0, 10.0, 0.80)
    for k in range(4000):
        ql2.add(k * 0.01, np.full(4, 0.2), np.array([1, 1, 0, 0], bool))
    check("a channel with no samples of its own reads 0.0, never another channel's",
          ql2.level()[0] == 0.2 and ql2.level()[2] == 0.0,
          "%s" % np.round(ql2.level(), 3))

    # ---- the breaker: a kick that plateaus is not a runaway -----------------
    print()
    # `growing` lags the envelope by `past`'s exclusion window: on
    # data/20260818_002229_fast_lock.csv it held true for 3.99 s against a 4.0 s
    # sustain and ended the run on the first hand kick. sustain 5.0 is what
    # osem.eta.py ships as of 2026-08-20 (4.0 while the recession CLEARED the
    # latch), named explicitly here because stdlib holds no constants.
    def envelope_run(shape, secs=40.0, hz=100.0, base=0.05, watch=None,
                     sustain=5.0):
        """Drive the breaker with a sinusoid whose amplitude follows `shape`."""
        b = breaker(4, sustain)
        b.arm(0.0)
        on, trip, seen = np.ones(4, bool), None, []
        for k in range(int(secs * hz)):
            tt = k / hz
            x = np.full(4, shape(tt) * np.sqrt(2.0) * np.sin(2 * np.pi * 1.0 * tt))
            g = b.sample(tt, x, on, on, np.full(4, base), np.zeros(4, bool))
            if g["run_trip"].any() and trip is None:
                trip = tt
            if watch is not None and watch[0] <= tt <= watch[1]:
                seen.append((tt, bool(g["growing"][0]), bool(g["receded"][0]),
                             float(g["env"][0])))
        return trip, seen

    # A KICKED PLATE, as this one actually is: a ring-up over ONE period of the
    # dominant mode (an impulse rings a resonator up in about a period; a longer
    # linear ramp is a sustained push), a decay at the measured re-quiet rate, and
    # the beat three modes necessarily produce -- 0.99193 - 0.72194 = 0.26999 Hz,
    # on which the recorded envelope dips about 10 % (ch0 ran 0.4166 -> 0.4342 V
    # under a 0.4599 V peak, data/20260818_002229_fast_lock.csv).
    def kicked(rate, ripple=0.10, t0=10.0, rise=1.0 / f[1]):
        def f(tt):
            if tt < t0:
                return 0.02
            a = 0.60 * (min(1.0, (tt - t0) / rise) if tt < t0 + rise
                        else np.exp(-rate * (tt - t0 - rise)))
            return a * (1.0 + ripple * np.sin(2 * np.pi * 0.26999 * (tt - t0)))
        return f

    _, seen = envelope_run(kicked(0.1393), watch=(10.0, 20.0))
    pk_t, pk = max(((tt, e) for tt, _, _, e in seen), key=lambda z: z[1])
    g_after = max((tt - pk_t for tt, g, _, _ in seen if g and tt > pk_t), default=0.0)
    r_after = min((tt - pk_t for tt, _, r, _ in seen if r and tt > pk_t), default=99.0)
    check("THE DEFECT: `growing` still reads TRUE on an envelope that is FALLING",
          g_after > 1.0,
          "envelope peaked %.4fV at t=%.2fs; `growing` held %.2fs past that -- "
          "`past` is a max over entries >= window_s old, so it lags by that much"
          % (pk, pk_t, g_after))
    check("THE FIX: `receded` turns over WITH the envelope, not window_s later",
          r_after < 0.5 and r_after < g_after,
          "receded %.2fs after the peak against `growing`'s %.2fs; 4.00s of that "
          "lag is what ended both runs on 2026-08-18" % (r_after, g_after))
    for rate, law in ((0.1393, "modal"), (0.0278, "diagonal")):
        tk, _ = envelope_run(kicked(rate), secs=70.0)
        check("A KICK THAT RISES AND THEN DECAYS CANNOT END A RUN (%s)" % law,
              tk is None, "decay %.4f /s, the measured re-quiet rate: trip %s"
              % (rate, "never" if tk is None else "%.1fs" % tk))
    # And it is not the ripple that saves it. The recession only VETOES now, so a
    # kick survives because it stops GROWING, not because the envelope wobbles.
    tk0, _ = envelope_run(kicked(0.0278, ripple=0.0), secs=70.0)
    check("...and one with no beat ripple at all, which a real plate does not make",
          tk0 is None, "no ripple, 0.0278 /s decay: trip %s"
          % ("never" if tk0 is None else "%.1fs" % tk0))
    # THE LIMIT: a ring-up that climbs monotonically for longer than the sustain
    # is indistinguishable from a runaway, and 2.0 s of linear rise plus the 2 s
    # RMS window is 4.0 s of monotone growth, which with the 2 s `past` exclusion
    # clears a 5.0 s sustain. It is a sustained push, not a kick.
    tk2, _ = envelope_run(kicked(0.0278, rise=2.0), secs=70.0)
    check("KNOWN LIMIT: a ring-up longer than one period is treated as a push",
          tk2 is not None,
          "2.0s linear rise (2x the 1.008s period) plus the 2s RMS window is 4.0s "
          "of monotone growth: trips at %.1fs, whatever the ripple"
          % (tk2 if tk2 is not None else float("nan")))
    grow = lambda tt: 0.02 * np.exp(0.0739 * max(tt - 10.0, 0.0))
    t_run, _ = envelope_run(grow, secs=70.0)
    check("...and a real runaway still trips, at the slowest rate this gain allows",
          t_run is not None,
          "0.0739 /s, the worst-mode added decay: trip at %s"
          % ("never" if t_run is None else "%.1fs" % t_run))
    # THE REGRESSION TEST FOR 2026-08-20, the whole defect in one fixture. A real
    # plate's envelope wobbles about its own running maximum as a matter of course
    # -- 13 bench runs, `env < 0.98 x peak` on 19-61 % of latch-accumulating
    # samples -- and while `~receded` sat inside the latch conjunction every one of
    # those restarted the sustain. The SAME 0.0739 /s ramp with a 0.6 s notch to
    # 0.70 amplitude three seconds before the clean trip takes the envelope 8 %
    # under its own peak (`receded` fires) while leaving it 6.6 % over
    # `past x 1.02` (`growing` does not): the shape of a beat trough.
    def notched(t0, secs=0.6, depth=0.70):
        return lambda tt: grow(tt) * (depth if t0 <= tt < t0 + secs else 1.0)

    t_notch, seen_n = envelope_run(notched(t_run - 3.0), secs=70.0,
                                   watch=(t_run - 3.0, t_run - 1.0))
    saw_recession = any(r for _, _, r, _ in seen_n)
    check("A RECESSION VETOES THE TRIP, IT DOES NOT RESTART THE SUSTAIN",
          saw_recession and t_notch is not None and abs(t_notch - t_run) < 0.5,
          "clean ramp trips at %.2fs, the same ramp notched at %.2fs trips at %s "
          "-- the notch DID recede (`receded` seen), and under the old conjunction "
          "it would have cost a fresh 5.0s sustain"
          % (t_run, t_run - 3.0,
             "never" if t_notch is None else "%.2fs" % t_notch))
    # `judge` separates "watched" from "may freeze the whole rig".
    def judged_run(judge_mask, secs=40.0, hz=100.0):
        b = breaker(4)
        b.arm(0.0)
        on, trip = np.ones(4, bool), None
        for k in range(int(secs * hz)):
            tt = k / hz
            a = 0.02 * np.exp(0.3 * max(tt - 10.0, 0.0))
            x = np.full(4, a * np.sqrt(2.0) * np.sin(2 * np.pi * 1.0 * tt))
            g = b.sample(tt, x, on, on, np.full(4, 0.05), np.zeros(4, bool),
                         judge=judge_mask)
            if g["run_trip"].any() and trip is None:
                trip = tt
        return trip

    # The capability exists and is tested; osem.eta.py deliberately does NOT use
    # it -- narrowing the evidence set lost the pumped-resonance detection.
    t_judged = judged_run(np.ones(4, bool))
    check("`judge` can separate who is watched from who may trip the whole rig",
          judged_run(np.zeros(4, bool)) is None and t_judged is not None,
          "unjudged never trips; judged trips at %.1fs" % t_judged)
    check("...and it defaults to `drives`, so callers that omit it are unchanged",
          judged_run(None) == t_judged, "both trip at %s" % t_judged)

    ramp = lambda tt: 0.02 if tt < 10.0 else 0.02 + 0.58 * min(1.0, (tt - 10.0) / 14.0)
    t_ramp, _ = envelope_run(ramp, secs=40.0)
    check("...as does an envelope that climbs for longer than the sustain",
          t_ramp is not None, "a 14s monotone rise is a sustained push, not a "
          "kick: trip %s" % ("never" if t_ramp is None else "%.1fs" % t_ramp))

    # ---- the total cap, not the allocation's own cap ------------------------
    print()
    cap, msk = 0.225, np.array([1, 1, 1, 1, 0, 0, 0, 0], bool)
    mu = np.array([0.05, -0.19, 0.12, 0.02, 0, 0, 0, 0], float)
    fx = np.array([0.01, -0.02, 0.22, 0.00, 0, 0, 0, 0], float)   # a big D term
    s = cap_scale(mu, fx, cap, msk)
    tot = np.abs(s * mu + fx)[msk]
    check("cap_scale puts the TOTAL inside the cap, not just the allocation",
          tot.max() <= cap + 1e-12,
          "peak total %.4f V against a %.3f V cap, scale %.4f" % (tot.max(), cap, s))
    old = cap / (np.abs(mu + fx)[msk].max() / 1.0)      # the previous one-shot form
    check("...which the previous one-shot scale did NOT: it assumed p was the demand",
          np.abs(old * mu + fx)[msk].max() > cap,
          "one-shot leaves %.4f V against %.3f V"
          % (np.abs(old * mu + fx)[msk].max(), cap))
    check("...and one scalar keeps the modal vector PARALLEL, so it stays dissipative",
          np.allclose(np.cross((s * mu)[:3], mu[:3]), 0.0, atol=1e-15),
          "scale %.4f applied to every element" % s)
    check("a fixed term already over the cap scales the allocation to zero",
          cap_scale(np.array([0.1]), np.array([0.4]), 0.225, np.array([True]))
          == 0.0, "no scale can fix it; PID.drive's clip does")

    # ---- the box allocator: the same total cap, solved per coil ------------
    print()
    rngb = np.random.default_rng(20260820)
    KP = np.array([0.035, 0.035, 0.035])        # MODAL_KP as it ships, flat
    worst_box = worst_pow = worst_proj = worst_obj = -np.inf
    ndraw = nbind = nclip_pump = nrankdef = 0
    ratio = []
    for it in range(3000):
        nc = int(rngb.integers(3, 5))            # 3 or 4 drivable coils
        cb = list(rngb.permutation(4)[:nc])
        Ab = np.zeros((nm, n))
        Ab[:, cb] = rngb.normal(size=(nm, nc))
        if it % 7 == 0:                          # rank-deficient on purpose
            Ab[:, cb[1]] = Ab[:, cb[0]] * rngb.normal()
        if it % 23 == 0:                         # a coil set that moves nothing
            Ab[:, cb] = 0.0
        msk = np.zeros(n, bool)
        msk[cb] = True
        Ac = Ab[:, cb]
        Uc, svc, _ = np.linalg.svd(Ac, full_matrices=False)
        keep = int(np.sum(svc > max(float(svc[0]), 1e-300) * 1e-9))
        Pj = Uc[:, :keep] @ Uc[:, :keep].T
        kp = KP if it % 3 else KP * np.array([1.0, 3.0, 0.4])   # non-flat too
        qd = rngb.normal(size=nm) * (0.0 if it % 13 == 0 else 1.0)   # f = 0 draws
        fd = -kp * (Pj @ qd)
        gv = Pj @ fd
        mb = np.zeros(n)
        mb[cb] = np.linalg.pinv(Ac) @ gv
        capb = float(10.0 ** rngb.uniform(-3.0, -0.3))
        fixb = np.zeros(n)
        # every fifth draw puts `fixed` ALONE past the cap -- the case cap_scale
        # answers with a scale of 0.0 (CLAUDE.md 2: coil 3 reached 0.4229V
        # against a 0.250V half-window while the allocation stayed inside 0.20V)
        fixb[cb] = rngb.normal(size=nc) * capb * (3.0 if it % 5 == 0 else 0.3)
        W = np.linalg.pinv(Pj @ np.diag(kp) @ Pj)
        ub, _pin = cap_box(mb, fixb, capb, msk, Ab, w=W)
        ndraw += 1
        nrankdef += keep < nm
        # 1. the box, stated as the invariant of the docstring
        tot = np.abs(ub + fixb)[msk]
        worst_box = max(worst_box, float(np.max(tot - np.maximum(capb,
                                                                np.abs(fixb)[msk]))))
        worst_box = max(worst_box, float(np.max(np.abs(ub[~msk]))))
        # 2. dissipativity under saturation
        pv = Ac @ ub[cb]
        if keep == nm:
            worst_pow = max(worst_pow, float(qd @ pv) if np.any(pv) else -np.inf)
        worst_proj = max(worst_proj, float(qd @ (Pj @ pv)) if np.any(pv) else -np.inf)
        # a naive per-channel clip has neither property
        lob = np.minimum(0.0, -capb - fixb[cb])
        hib = np.maximum(0.0, capb - fixb[cb])
        if keep == nm and float(qd @ (Ac @ np.clip(mb[cb], lob, hib))) > 1e-15:
            nclip_pump += 1
        # 3. never worse than cap_scale on the objective it is asked to minimise
        us = cap_scale(mb, fixb, capb, msk) * mb
        rb, rs = Ac @ ub[cb] - gv, Ac @ us[cb] - gv
        scl = max(1.0, float(gv @ W @ gv))
        worst_obj = max(worst_obj, float(rb @ W @ rb - rs @ W @ rs) / scl)
        if float(np.max(np.abs(ub - mb))) > 1e-12 * max(1.0, float(np.max(np.abs(mb)))):
            nbind += 1
            ns = float(np.linalg.norm(Ac @ us[cb]))
            if ns > 0:
                ratio.append(float(np.linalg.norm(pv)) / ns)

    check("cap_box NEVER leaves the box, over %d randomised draws" % ndraw,
          worst_box <= 1e-12,
          "worst |u+fixed| - max(cap,|fixed|) = %+.2e; %d draws rank-deficient, "
          "%d with fixed alone past the cap" % (worst_box, nrankdef, ndraw // 5))
    check("...and it DISSIPATES under saturation: qdot . (A_C u) < 0",
          worst_pow < 0.0,
          "worst power %+.3e over the full-rank draws, where W = pinv(Proj K "
          "Proj) is the metric the proof needs" % worst_pow)
    check("...on a rank-deficient mask only the PROJECTED power is bounded",
          worst_proj <= 1e-12,
          "worst qdot . Proj p = %+.3e -- which is why `box_args` returns None "
          "there and the caller must keep cap_scale" % worst_proj)
    check("...where a naive per-channel clip PUMPS instead",
          nclip_pump > 0, "clip(u) gave qdot . p > 0 on %d of %d full-rank draws"
          % (nclip_pump, ndraw - nrankdef))
    check("...and it never does worse than cap_scale on the objective",
          worst_obj <= 1e-9,
          "worst normalised excess %+.2e over %d draws, %d of them binding"
          % (worst_obj, ndraw, nbind))
    check("...and where the box binds it realises MORE force than cap_scale",
          len(ratio) > 0 and float(np.median(ratio)) > 1.0,
          "||A_C u|| ratio median %.3f, p10 %.3f, p90 %.3f over %d binding draws"
          % (float(np.median(ratio)), float(np.percentile(ratio, 10)),
             float(np.percentile(ratio, 90)), len(ratio)))

    # An UNBOUND box is today's allocator exactly -- not a nearby point of the
    # same objective. That is what the ||u/d|| tie-break is for, and `d` has to
    # be the row-balancing scale `_build_tables` used or it lands elsewhere.
    drv = np.zeros(n, bool)
    drv[[0, 1, 2, 3]] = True
    u0, mk0, cm0, _ = M.allocate(-KP * np.array([0.60, -0.30, 0.45]), drv)
    Wf, df = M.box_args(mk0, KP)
    ubig, _ = cap_box(u0, np.zeros(n), 1e6, cm0, M.a, w=Wf, d=df)
    check("unconstrained, cap_box reproduces the ROW-BALANCED A_C^+ f exactly",
          np.allclose(ubig, u0, rtol=0, atol=1e-12) and M.balanced[mk0],
          "max |cap_box - allocate| = %.2e over the balanced inverse the rungs "
          "actually run" % float(np.max(np.abs(ubig - u0))))
    check("...and `box_args` hands back W = diag(1/kp) on a full-rank mask",
          np.allclose(Wf, 1.0 / KP) and df.shape == (n,),
          "kp %s -> W diag %s" % (np.round(KP, 4), np.round(Wf, 2)))
    check("...and REFUSES the two cases the dissipation proof does not cover",
          M.box_args(mk0, np.array([0.035, 0.0, 0.035])) == (None, None)
          and M.box_args(0b0011, KP) == (None, None),
          "a zero per-mode gain (K singular), and a 2-coil mask that has no "
          "allocator table at all (min_coils %d)" % M.min_coils)

    # The measured overrun, as a worked case. Coil 0 is 3x weaker than 1/2/3
    # (9.92 against 29.80 / 34.28 / 28.93 counts/V, CLAUDE.md "The geometry"),
    # so under one scalar it is the coil that decides the whole vector.
    capd, mskd = 0.225, np.array([1, 1, 1, 1, 0, 0, 0, 0], bool)
    Ad = np.zeros((nm, n))
    Ad[:, :4] = np.array([[-0.0764, +0.2480, +0.8196, +0.5108],
                          [-0.5489, +0.4114, -0.2699, -0.6757],
                          [-0.2884, -0.5612, -0.6515, +0.4212]])   # 02:42 run's A
    md = np.zeros(n)
    md[:4] = np.array([0.150, -0.040, 0.030, 0.020])
    fxd = np.zeros(n)
    fxd[:4] = np.array([0.100, -0.010, 0.005, 0.000])   # a big D term on coil 0
    sd = cap_scale(md, fxd, capd, mskd)
    ud, _ = cap_box(md, fxd, capd, mskd, Ad, w=1.0 / KP)
    gd = Ad[:, :4] @ md[:4]
    check("ONE coil near its rail no longer throttles the other three",
          np.abs(ud + fxd)[mskd].max() <= capd + 1e-12
          and np.linalg.norm(Ad[:, :4] @ ud[:4] - gd)
          < np.linalg.norm(Ad[:, :4] @ (sd * md)[:4] - gd),
          "cap_scale scales all four by %.3f; cap_box pins coil 0 at %.4fV and "
          "keeps %.0f%% of the commanded force against its %.0f%%"
          % (sd, ud[0], 100 * np.linalg.norm(Ad[:, :4] @ ud[:4])
             / np.linalg.norm(gd),
             100 * np.linalg.norm(Ad[:, :4] @ (sd * md)[:4]) / np.linalg.norm(gd)))

    # ---- the in-band floor must not grade an orthogonal axis blind ----------
    print()
    hf = health()
    enb = np.array([1, 1, 1, 1, 1, 0, 1, 1], bool)
    exm = np.array([0, 0, 0, 0, 1, 1, 1, 1], bool)
    # a0-a3 carry the band; a4/a6/a7 read orthogonal axes and MOVE; a5 is a pin.
    inband = np.array([.083, .090, .078, .085, .0027, .0000, .0026, .0024])
    motion = np.array([70.7, 48.4, 75.0, 90.2, 43.4, 0.00, 61.3, 32.6])
    bad, med, wit = hf.floor(inband, enb, exempt=exm, motion=motion)
    check("A LOW IN-BAND CHANNEL THAT MOVES IS NOT DEMOTED -- its axis is orthogonal",
          not bad[4] and not bad[6] and not bad[7] and wit,
          "a4/a6/a7 in-band %.4f/%.4f/%.4fV of a %.4fV median, but std "
          "%.1f/%.1f/%.1f counts" % (inband[4], inband[6], inband[7], med,
                                     motion[4], motion[6], motion[7]))
    check("...while a genuinely dead pin still IS: variance is the real test",
          bool(bad[5]), "a5 std %.2f counts, under the %.2f-count line"
          % (motion[5], hf.dead_std))
    # A blind sensor still returns ADC dither, so an ABSOLUTE std line passes it.
    # This is how the simulator models one, and it is why the exempt test is
    # relative to what the live channels move.
    dither = motion.copy()
    dither[4] = dither[6] = dither[7] = 2.5      # over dead_std, still dither
    badd, _, _ = hf.floor(inband, enb, exempt=exm, motion=dither)
    check("...and a BLIND channel that only returns dither is demoted too",
          bool(badd[4] and badd[6] and badd[7]),
          "2.50 counts against a %.1f-count median across the channels that carry "
          "the band -- over the %.2f-count dead-pin line, under %.0f%% of the median"
          % (float(np.median(motion[:4])), hf.dead_std, 100 * hf.floor_frac))
    blind = inband.copy()
    blind[2] = 0.0005
    bad2, _, _ = hf.floor(blind, enb, exempt=exm, motion=motion)
    check("...and a channel that IS in the band and blind is still caught",
          bool(bad2[2]), "a2 in-band %.4fV" % blind[2])
    bad3, med3, _ = hf.floor(inband, enb)
    check("with no motion evidence nothing is exempted -- the old behaviour exactly",
          bool(bad3[4] and bad3[6] and bad3[7]),
          "median %.4fV over every enabled channel" % med3)

    print("\n  %s\n" % ("ALL PASS -- the pure-math pieces hold. Nothing here says "
                        "the loop damps the optic; only the bench can."
                        if ok else "FAILURES ABOVE."))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv[1:]:
        sys.exit(_selftest())
    print(__doc__)
