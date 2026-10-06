import numpy as np


def _osc_blocks(Phi, w, dt):
    for m, wm in enumerate(w):
        th = wm * dt
        c, s = (np.cos(th), np.sin(th))
        Phi[2 * m : 2 * m + 2, 2 * m : 2 * m + 2] = [[c, s / wm], [-wm * s, c]]
    return Phi


class KalmanVelocity:

    def __init__(self, K, n, f, dt):
        self.n, self.dt, self.f = (int(n), float(dt), np.asarray(f, float))
        self.w = 2.0 * np.pi * self.f
        self.nm = len(self.f)
        self.nx = 2 * self.nm + 1
        self.K = np.asarray(K, float).reshape(self.n, self.nx)
        self.H = np.zeros(self.nx)
        self.H[0 : 2 * self.nm : 2] = 1.0
        self.H[-1] = 1.0
        self.Cv = np.zeros(self.nx)
        self.Cv[1 : 2 * self.nm : 2] = 1.0
        self.Phi = self._transition(self.dt)
        self.x = np.zeros((self.n, self.nx))
        self.primed = np.zeros(self.n, bool)

    def _transition(self, dt):
        Phi = _osc_blocks(np.zeros((self.nx, self.nx)), self.w, dt)
        Phi[-1, -1] = 1.0
        return Phi

    def update(self, y, dt=None, valid=None):
        Phi = self.Phi if dt is None or abs(dt - self.dt) < 1e-09 else self._transition(dt)
        y = np.asarray(y, float)
        cold = ~self.primed
        if cold.any():
            self.x[cold] = 0.0
            self.x[cold, -1] = y[cold]
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
        return self.x[:, 0 : 2 * self.nm : 2].sum(axis=1)

    def modal_velocity(self):
        return self.x[:, 1 : 2 * self.nm : 2]


class ModalKalman:

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
        self.q_mode = np.array([float(q_sensor[self.rowok[:, m], m].sum()) for m in range(self.nm)])
        for m in range(self.nm):
            if not self.q_mode[m] > 0:
                self.q_mode[m] = float(q_sensor[:, m].max())
        self.var_q = self.q_mode * float(t_amp_s) / (2.0 * self.w**2)
        self.H0 = np.zeros((self.n, self.nx))
        self.H0[:, 0 : 2 * self.nm : 2] = self.phi
        self.H0[np.arange(self.n), 2 * self.nm + np.arange(self.n)] = 1.0
        self.x = np.zeros(self.nx)
        self.P = np.zeros((self.nx, self.nx))
        self.dc_primed = np.zeros(self.n, bool)
        self.seen = np.zeros(self.nm, int)
        self.chi2, self.dof = (float("nan"), 0)
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
        self.chi2, self.dof = (float("nan"), 0)
        self._prior()

    def _transition(self, dt):
        if self._trans_dt is not None and abs(dt - self._trans_dt) < 1e-09:
            return
        nm, nx = (self.nm, self.nx)
        Phi = _osc_blocks(np.eye(nx), self.w, dt)
        Qd = np.zeros((nx, nx))
        for m, w in enumerate(self.w):
            th = w * dt
            s, s2 = (np.sin(th), np.sin(2.0 * th))
            Qd[2 * m : 2 * m + 2, 2 * m : 2 * m + 2] = self.q_mode[m] * np.array(
                [
                    [(dt / 2.0 - s2 / (4.0 * w)) / w**2, s**2 / (2.0 * w**2)],
                    [s**2 / (2.0 * w**2), dt / 2.0 + s2 / (4.0 * w)],
                ]
            )
        Qd[np.arange(2 * nm, nx), np.arange(2 * nm, nx)] = self.q_dc * dt
        self.Phi, self.Qd, self._trans_dt = (Phi, Qd, dt)

    def update(self, y, live, dt=None):
        dt = self.dt if dt is None else dt
        self._transition(dt)
        y = np.asarray(y, float)
        live = np.asarray(live, bool)
        nm = self.nm
        self.dc_primed &= live
        for i in np.nonzero(live & ~self.dc_primed)[0]:
            k = 2 * nm + i
            self.x[k] = y[i] - float(self.phi[i] @ self.x[0 : 2 * nm : 2])
            self.P[k, :] = 0.0
            self.P[:, k] = 0.0
            self.P[k, k] = self.r[i]
        self.dc_primed |= live
        self.seen = np.array([int((live & self.rowok[:, m]).sum()) for m in range(nm)])
        xp = self.Phi @ self.x
        Pp = self.Phi @ self.P @ self.Phi.T + self.Qd
        idx = np.nonzero(live)[0]
        if idx.size == 0:
            self.x, self.P = (xp, Pp)
            self.chi2, self.dof = (float("nan"), 0)
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
            self.x, self.P = (xp, Pp)
            self.chi2, self.dof = (float("nan"), 0)
            return self.qdot()
        Sinv_e, K = (Se[:, 0], Se[:, 1:].T)
        self.chi2, self.dof = (float(e @ Sinv_e), int(idx.size))
        self.x = xp + K @ e
        IKH = np.eye(self.nx) - K @ H
        self.P = IKH @ Pp @ IKH.T + K * rl @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return self.qdot()

    def qdot(self):
        return self.x[1 : 2 * self.nm : 2].copy()

    def q(self):
        return self.x[0 : 2 * self.nm : 2].copy()

    def chi2_per_dof(self):
        return self.chi2 / self.dof if self.dof else float("nan")


class Chi2Log:

    def __init__(self, levels):
        self.levels = tuple(levels)
        self.n, self.total, self.peak = (0, 0.0, 0.0)
        self.over = [0] * len(self.levels)

    def add(self, x):
        x = float(x)
        if not np.isfinite(x):
            return
        self.n += 1
        self.total += x
        self.peak = max(self.peak, x)
        for k, lv in enumerate(self.levels):
            if x > lv:
                self.over[k] += 1

    def line(self, label):
        if not self.n:
            return "  %-12s no samples" % label
        return "  %-12s n=%-7d mean %7.3f  max %9.3f   " % (
            label,
            self.n,
            self.total / self.n,
            self.peak,
        ) + "  ".join(
            (">%g %5.1f%%" % (lv, 100.0 * c / self.n) for lv, c in zip(self.levels, self.over))
        )


class Estimator:

    def __init__(self, rig, cfg, modes=True):
        dt = 1.0 / cfg.control_hz
        self.kf = KalmanVelocity(rig.kalman_k, rig.n, rig.f_hz, dt)
        self.mkf = (
            ModalKalman(
                rig.phi_unit,
                rig.phi != 0,
                rig.kalman_r,
                rig.kalman_q,
                rig.kalman_r / cfg.t_dc_s,
                rig.f_hz,
                dt,
                rig.n,
                cfg.t_amp_s,
            )
            if modes
            else None
        )
        self.vel = self.bp = np.zeros(rig.n)
        self.qdot = np.zeros(rig.nm)
        self.seen = np.zeros(rig.nm, int)
        self.chi2 = float("nan")

    def update(self, volts, dt, usable):
        self.vel = self.kf.update(volts, dt)
        self.bp = self.kf.displacement()
        if self.mkf is not None:
            self.qdot = self.mkf.update(volts, usable, dt)
            self.chi2, self.seen = (self.mkf.chi2_per_dof(), self.mkf.seen)

    def reset(self):
        self.kf.reset()
        if self.mkf is not None:
            self.mkf.reset()
