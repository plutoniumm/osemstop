from collections import deque
import numpy as np
from scipy.linalg import solve_discrete_are


def osc(w, dt, q=None):
    n = 2 * len(w)
    F, Q = (np.zeros((n, n)), np.zeros((n, n)))
    for m, wm in enumerate(w):
        th = wm * dt
        c, s, s2 = (np.cos(th), np.sin(th), np.sin(2.0 * th))
        F[2 * m : 2 * m + 2, 2 * m : 2 * m + 2] = [[c, s / wm], [-wm * s, c]]
        if q is not None:
            Q[2 * m : 2 * m + 2, 2 * m : 2 * m + 2] = q[m] * np.array(
                [
                    [(dt / 2.0 - s2 / (4.0 * wm)) / wm**2, s**2 / (2.0 * wm**2)],
                    [s**2 / (2.0 * wm**2), dt / 2.0 + s2 / (4.0 * wm)],
                ]
            )
    return F if q is None else (F, Q)


def steady_gain(r, q, q_dc, f, dt):
    w, nm = (2.0 * np.pi * np.asarray(f, float), len(f))
    nx = 2 * nm + 1
    H = np.zeros(nx)
    H[0 : 2 * nm : 2] = H[-1] = 1.0
    K = np.zeros((len(r), nx))
    for i in range(len(r)):
        F, Q = (np.eye(nx), np.zeros((nx, nx)))
        F[:-1, :-1], Q[:-1, :-1] = osc(w, dt, q[i])
        Q[-1, -1] = q_dc[i] * dt
        on = np.r_[np.repeat(q[i] > 0, 2), True]
        Fi, Hi = (F[np.ix_(on, on)], H[on])
        P = solve_discrete_are(Fi.T, Hi[:, None], Q[np.ix_(on, on)], r[i])
        K[i, on] = P @ Hi / (Hi @ P @ Hi + r[i])
    return K


class Clock:

    def __init__(self, hz):
        self.period = 1.0 / float(hz)
        self.ts, self.err, self.prev = (deque(maxlen=max(int(hz), 33)), 0.0, None)

    def tick(self, t):
        self.ts.append(t)
        if len(self.ts) > 32:
            est = (t - self.ts[0]) / (len(self.ts) - 1)
            if abs(est - self.period) > 0.002 * self.period:
                self.period = min(max(est, 0.0001), 0.05)
        dt = self.period
        self.err += t - (t - dt if self.prev is None else self.prev) - dt
        if abs(self.err) > 0.001:
            dt = min(max(self.period + self.err, 0.0001), 0.05)
            self.err = min(self.period + self.err - dt, 0.0)
        self.prev = t
        return dt


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
    # one window per channel: a masked channel skips its append, which sets the breaker's timing

    def __init__(self, window_s, n):
        self.b = [SlidingRMS(window_s) for _ in range(n)]

    def update(self, t, x, mask):
        return np.array([b.update(t, x[i]) if mask[i] else 0.0 for i, b in enumerate(self.b)])

    def reset(self, mask=None):
        for i, b in enumerate(self.b):
            if mask is None or mask[i]:
                b.reset()


class KalmanVelocity:

    def __init__(self, K, n, f, dt):
        self.n, self.dt, self.w = (int(n), float(dt), 2.0 * np.pi * np.asarray(f, float))
        self.nm = len(self.w)
        self.nx = 2 * self.nm + 1
        self.K = np.asarray(K, float).reshape(self.n, self.nx)
        self.H, self.Cv = (np.zeros(self.nx), np.zeros(self.nx))
        self.H[0 : 2 * self.nm : 2] = self.H[-1] = 1.0
        self.Cv[1 : 2 * self.nm : 2] = 1.0
        self.Phi = self._transition(self.dt)
        self.x = np.zeros((self.n, self.nx))
        self.primed = np.zeros(self.n, bool)

    def _transition(self, dt):
        Phi = np.zeros((self.nx, self.nx))
        Phi[:-1, :-1], Phi[-1, -1] = (osc(self.w, dt), 1.0)
        return Phi

    def update(self, y, dt):
        Phi = self.Phi if abs(dt - self.dt) < 1e-09 else self._transition(dt)
        y = np.asarray(y, float)
        cold = ~self.primed
        if cold.any():
            self.x[cold] = 0.0
            self.x[cold, -1] = y[cold]
            self.primed[:] = True
        xp = self.x @ Phi.T
        self.x = xp + self.K * (y - xp @ self.H)[:, None]
        return self.x @ self.Cv

    def reset(self, mask=slice(None)):
        self.x[mask] = 0.0
        self.primed[mask] = False

    def displacement(self):
        return self.x[:, 0 : 2 * self.nm : 2].sum(axis=1)


class ModalKalman:

    def __init__(self, rig, cfg, g):
        self.w, self.nm, self.n = (2.0 * np.pi * rig.f_hz, rig.nm, rig.n)
        self.nx = 2 * self.nm + self.n
        self.r, self.q_dc, self.g = (rig.kalman_r, rig.kalman_r / cfg.t_dc_s, g)
        self.rowok = rig.phi != 0
        self.phi = np.where(self.rowok, rig.phi_unit, 0.0)
        q = rig.kalman_q
        self.q_mode = np.array([float(q[self.rowok[:, m], m].sum()) for m in range(self.nm)])
        for m in range(self.nm):
            if not self.q_mode[m] > 0:
                self.q_mode[m] = float(q[:, m].max())
        self.var_q = self.q_mode * float(cfg.t_amp_s) / (2.0 * self.w**2)
        self.H0 = np.zeros((self.n, self.nx))
        self.H0[:, 0 : 2 * self.nm : 2] = self.phi
        self.H0[np.arange(self.n), 2 * self.nm + np.arange(self.n)] = 1.0
        self.x = np.zeros(self.nx)
        self.P = np.zeros((self.nx, self.nx))
        self._dt = None
        self.reset()

    def reset(self):
        self.x[:] = 0.0
        self.dc_primed = np.zeros(self.n, bool)
        self.seen = np.zeros(self.nm, int)
        self.chi2 = float("nan")
        self.P[:] = 0.0
        for m in range(self.nm):
            self.P[2 * m, 2 * m] = self.var_q[m]
            self.P[2 * m + 1, 2 * m + 1] = self.var_q[m] * self.w[m] ** 2
        for i in range(self.n):
            self.P[2 * self.nm + i, 2 * self.nm + i] = self.r[i]

    def _transition(self, dt):
        if self._dt is not None and abs(dt - self._dt) < 1e-09:
            return
        k = 2 * self.nm
        self.Phi, self.Qd, self.B = (np.eye(self.nx), np.zeros((self.nx, self.nx)), None)
        self.Phi[:k, :k], self.Qd[:k, :k] = osc(self.w, dt, self.q_mode)
        self.Qd[np.arange(k, self.nx), np.arange(k, self.nx)] = self.q_dc * dt
        self.B = np.zeros((self.nx, self.g.shape[1]))
        self.B[0:k:2] = (1.0 - np.cos(self.w * dt))[:, None] * self.g
        self.B[1:k:2] = (self.w * np.sin(self.w * dt))[:, None] * self.g
        self._dt = dt

    def update(self, y, live, dt, u):
        self._transition(dt)
        y, live, nm = (np.asarray(y, float), np.asarray(live, bool), self.nm)
        self.dc_primed &= live
        for i in np.nonzero(live & ~self.dc_primed)[0]:
            k = 2 * nm + i
            self.x[k] = y[i] - float(self.phi[i] @ self.x[0 : 2 * nm : 2])
            self.P[k, :] = 0.0
            self.P[:, k] = 0.0
            self.P[k, k] = self.r[i]
        self.dc_primed |= live
        self.seen = np.array([int((live & self.rowok[:, m]).sum()) for m in range(nm)])
        self.x = self.Phi @ self.x + self.B @ u
        self.P = Pp = self.Phi @ self.P @ self.Phi.T + self.Qd
        self.chi2 = float("nan")
        idx = np.nonzero(live)[0]
        if idx.size == 0:
            return self.qdot()
        H, rl = (self.H0[idx], self.r[idx])
        e = y[idx] - H @ self.x
        PHt = Pp @ H.T
        try:
            Se = np.linalg.solve(H @ PHt + np.diag(rl), np.column_stack([e, PHt.T]))
        except np.linalg.LinAlgError:
            return self.qdot()
        Sinv_e, K = (Se[:, 0], Se[:, 1:].T)
        self.chi2 = float(e @ Sinv_e) / idx.size
        self.x = self.x + K @ e
        IKH = np.eye(self.nx) - K @ H
        self.P = IKH @ Pp @ IKH.T + K * rl @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return self.qdot()

    def qdot(self):
        return self.x[1 : 2 * self.nm : 2].copy()


class Estimator:

    def __init__(self, rig, cfg):
        dt = 1.0 / cfg.control_hz
        g = np.zeros((rig.nm, rig.n))
        scale = np.linalg.norm(rig.phi, axis=0) * cfg.volts_per_count
        g[:, rig.a_coils] = scale[:, None] * rig.a_dc[:, rig.a_coils]
        self.kf = KalmanVelocity(rig.kalman_k, rig.n, rig.f_hz, dt)
        self.mkf = ModalKalman(rig, cfg, g)
        self.vel = self.bp = np.zeros(rig.n)
        self.qdot = np.zeros(rig.nm)
        self.seen = np.zeros(rig.nm, int)
        self.chi2 = float("nan")

    def update(self, volts, dt, usable, u):
        self.vel = self.kf.update(volts, dt)
        self.bp = self.kf.displacement()
        self.qdot = self.mkf.update(volts, usable, dt, u)
        self.chi2, self.seen = (self.mkf.chi2, self.mkf.seen)

    def reset(self):
        self.kf.reset()
        self.mkf.reset()
