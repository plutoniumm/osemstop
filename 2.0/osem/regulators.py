import numpy as np
from .filters import OnePole


class Regulator:
    letter = "?"
    needs_modes = False

    def __init__(self, rig, cfg):
        self.n = rig.n
        self.cap = cfg.budget_v
        self.slew = cfg.gain_slew_per_s
        self.coils = np.zeros(rig.n, bool)
        self.claimed = np.zeros(rig.n, bool)

    def ramp(self, dt, sense_ok):
        pass

    def zero(self, mask=slice(None)):
        pass

    def reset(self):
        pass

    def release(self, dead):
        pass

    def command(self, est, free, sense_ok, dt, on):
        raise NotImplementedError


class Modal(Regulator):
    letter = "M"
    needs_modes = True
    balance = (12, 0.5, 1.05)

    def __init__(self, rig, cfg):
        super().__init__(rig, cfg)
        self.a = rig.a_unit
        self.coils[rig.a_coils] = True
        self.target = np.full(rig.nm, cfg.modal_kp * cfg.modal_scale)
        self.gain = np.zeros(rig.nm)
        self.cond_max = cfg.modal_cond_max
        self.min_coils, self.min_sensors = (cfg.modal_min_coils, cfg.modal_min_sensors)
        self._solutions = {}

    def ramp(self, dt, sense_ok):
        step = self.slew * dt
        self.gain = self.gain + np.clip(self.target - self.gain, -step, step)

    def zero(self, mask=slice(None)):
        if isinstance(mask, slice):
            self.gain[:] = 0.0

    def _solution(self, cols):
        if cols not in self._solutions:
            self._solutions[cols] = self._solve(list(cols))
        return self._solutions[cols]

    def _solve(self, cols):
        if len(cols) < self.min_coils:
            return None
        Ac = self.a[:, cols]
        U, sv, Vh = np.linalg.svd(Ac, full_matrices=False)
        if sv[0] <= 1e-09:
            return None
        keep = int(np.sum(sv >= sv[0] / self.cond_max))
        keep = min(keep, int(np.linalg.matrix_rank(Ac, tol=1e-06)))
        if keep < 1:
            return None
        Uk, svk, Vhk = (U[:, :keep], sv[:keep], Vh[:keep, :])
        proj = Uk @ Uk.T
        P = Vhk.T * (1.0 / svk) @ Uk.T
        iters, step, tol = self.balance
        Ak, w, Pw = (proj @ Ac, np.ones(len(cols)), None)
        for _ in range(iters):
            D = 1.0 / w
            try:
                Pw = np.linalg.pinv(Ak * D[None, :]) * D[:, None]
            except np.linalg.LinAlgError:
                Pw = None
                break
            rn = np.linalg.norm(Pw, axis=1)
            if not np.all(np.isfinite(rn)) or rn.max() <= 0:
                Pw = None
                break
            if rn.max() / max(rn.min(), 1e-30) < tol:
                break
            w = np.clip(w * (rn / rn.mean()) ** step, 0.001, 1000.0)
            w = w / w.mean()
        if Pw is not None and np.allclose(Ac @ Pw, proj, rtol=1e-07, atol=1e-10):
            P = Pw
        return (P, proj)

    def solve(self, qdot, gain, coil_ok, seen):
        u, used = (np.zeros(self.n), np.zeros(self.n, bool))
        cols = tuple((j for j in range(self.n) if coil_ok[j] and self.coils[j]))
        enough = seen >= self.min_sensors
        sol = self._solution(cols)
        if sol is None or not enough.any():
            return (u, used)
        P, proj = sol
        f = np.where(enough, -gain * (proj @ qdot), 0.0)
        u[list(cols)] = P @ (proj @ f)
        peak = float(np.abs(u).max())
        if peak > self.cap:
            u *= self.cap / peak
        used[list(cols)] = True
        return (u, used)

    def command(self, est, free, sense_ok, dt, on):
        u, self.claimed = (np.zeros(self.n), np.zeros(self.n, bool))
        if on:
            u, self.claimed = self.solve(est.qdot, self.gain, free, est.seen)
        return u


class Pid(Regulator):
    letter = "P"

    def __init__(self, rig, cfg):
        super().__init__(rig, cfg)
        # power = -slope * gain * vel^2: the gain carries the slope's sign
        self.target = cfg.kp * rig.slope_sign * rig.pid_weight
        self.target[list(cfg.disable)] = 0.0
        S = rig.a_dc @ np.diag(self.target) @ rig.phi
        self.eig = np.linalg.eigvalsh(0.5 * (S + S.T))
        # per-channel feedback pumps a mode unless sym(A g Phi) > 0
        self.safe = bool((self.eig > 0).all())
        if not self.safe:
            self.target[rig.a_coils] = 0.0
        self.coils = self.target != 0.0
        self.kd = cfg.kd * self.target / cfg.kp if cfg.kp else np.zeros(rig.n)
        self.gain = np.zeros(rig.n)
        self.dfilt = OnePole(cfg.d_hz, "low", rig.n)
        self.prev_vel = np.zeros(rig.n)
        self.primed = np.zeros(rig.n, bool)

    def ramp(self, dt, sense_ok):
        step = self.slew * dt
        self.gain = np.where(
            sense_ok, self.gain + np.clip(self.target - self.gain, -step, step), 0.0
        )

    def zero(self, mask=slice(None)):
        self.gain[mask] = 0.0

    def reset(self):
        self.release(slice(None))

    def release(self, dead):
        self.prev_vel[dead] = 0.0
        self.primed[dead] = False
        self.dfilt.reset(dead)

    def command(self, est, free, sense_ok, dt, on):
        vel = est.vel
        self.prev_vel[~self.primed] = vel[~self.primed]
        self.primed[:] = True
        dv = (vel - self.prev_vel) / dt if dt > 0 else np.zeros(self.n)
        self.prev_vel = np.array(vel, float)
        d = -self.kd * self.dfilt.update(dv, dt)
        self.claimed = free & sense_ok & self.coils
        return np.clip(-self.gain * vel + d, -self.cap, self.cap)
