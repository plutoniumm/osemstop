from collections import deque
import numpy as np
from rich.text import Text
from .filters import Clock, Estimator, RMSBank
from .rig import GEOMETRY, Config


def hold(cond, since, t):
    return np.where(cond, np.where(np.isinf(since), t, since), np.inf)


def box_solve(H, b, lo, hi, room, start=None, rounds=6):
    n = len(b)
    u, free, tied = (np.zeros(n), np.ones(n, bool), False) if start is None else start
    u, free, lam = (u.copy(), free.copy(), 0.0)
    for _ in range(rounds):
        k = np.nonzero(free)[0]
        if len(k):
            u[k] = 0.0
            rhs = (b - H @ u)[k]
            if tied:
                K = np.ones((len(k) + 1, len(k) + 1))
                K[:-1, :-1], K[-1, -1] = (H[k][:, k], 0.0)
                x = np.linalg.solve(K, np.append(rhs, room - u.sum()))
                x, lam = (x[:-1], x[-1])
            else:
                x, lam = (np.linalg.solve(H[k][:, k], rhs), 0.0)
            bad = (x < lo[k]) | (x > hi[k])
            u[k] = np.clip(x, lo[k], hi[k])
            if bad.any():
                free[k[bad]] = False
                continue
        push = H @ u - b
        if not tied and u.sum() > room:
            tied = True
            if not len(k):
                # every coil pinned and the sum is over: free the one that helps least
                free[np.argmax(np.where(u > 0, push, -np.inf))] = True
            continue
        if tied and lam < 0:
            tied = False
            continue
        push = push + lam
        wrong = ~free & (((u >= hi) & (push > 0)) | ((u <= lo) & (push < 0)))
        if not wrong.any():
            break
        free |= wrong
    state = (u, free, tied)
    if u.sum() > room:
        u = u * (room / u.sum())
    return (u, state)


class Modal:
    RIDGE, GAIN_FLOOR = (0.01, 0.1)

    def __init__(self, rig, cfg):
        self.cfg, self.n, self.a = (cfg, rig.n, rig.a_unit)
        self.coils = np.zeros(rig.n, bool)
        self.coils[rig.a_coils] = True
        self.target = np.full(rig.nm, cfg.modal_kp * cfg.modal_scale)
        self.target *= [dict(cfg.mode_gain).get(d, 1.0) for d in rig.dof]
        self.gain = np.zeros(rig.nm)
        self.min_sensors = np.minimum(cfg.modal_min_sensors, (rig.phi != 0).sum(axis=0))
        self.lo = np.maximum(-cfg.budget_v, cfg.vmin - rig.bias)
        self.hi = np.minimum(cfg.budget_v, cfg.vmax - rig.bias)
        self.room = max(cfg.sum_v_max - float(rig.bias.sum()), 0.0)
        self._solutions, self._warm = ({}, (None, None))

    def ramp(self, dt):
        step = self.cfg.gain_slew_per_s * dt
        self.gain = self.gain + np.clip(self.target - self.gain, -step, step)

    def _solve(self, cols):
        if len(cols) < self.cfg.modal_min_coils:
            return None
        Ac = self.a[:, cols]
        top = np.linalg.svd(Ac, compute_uv=False)[0]
        if top <= 1e-09:
            return None
        return (Ac, self.RIDGE * (top / self.cfg.modal_cond_max) ** 2 * np.eye(len(cols)))

    def solve(self, qdot, gain, coil_ok, seen):
        u, used = (np.zeros(self.n), np.zeros(self.n, bool))
        cols = tuple((j for j in range(self.n) if coil_ok[j] and self.coils[j]))
        enough = seen >= self.min_sensors
        if cols not in self._solutions:
            self._solutions[cols] = self._solve(list(cols))
        sol = self._solutions[cols]
        if sol is None or not enough.any():
            return (u, used)
        k = list(cols)
        used[k] = True
        g = np.where(enough, gain, 0.0)
        if g.max() <= 0:
            return (u, used)
        Ac, ridge = sol
        # weighting each mode's shortfall by 1/gain keeps qdot.(A u) <= 0 at the optimum
        AtW = Ac.T / np.maximum(g / g.max(), self.GAIN_FLOOR)
        start = self._warm[1] if self._warm[0] == cols else None
        x, state = box_solve(
            AtW @ Ac + ridge, AtW @ (-g * qdot), self.lo[k], self.hi[k], self.room, start
        )
        self._warm = (cols, state)
        if (qdot * (Ac @ x))[g > 0].sum() > 0:
            return (u, used)
        u[k] = x
        return (u, used)


class Health:

    def __init__(self, n, cfg):
        self.n, self.cfg = (n, cfg)
        self.rail = np.zeros(n, bool)
        self.sat = np.zeros(n, bool)
        self.healthy = np.ones(n, bool)
        self.floor_bad = np.zeros(n, bool)
        self.clear_t = np.full(n, np.inf)
        self.rail_std = np.zeros(n)
        self.rail_hist, self.sat_hist = (deque(), deque())
        self.rail_sum, self.sat_sum = (np.zeros(n), np.zeros(n))
        self.c_sum, self.c2_sum = (np.zeros(n), np.zeros(n))

    def rails(self, counts, t):
        cfg = self.cfg
        railed = (counts <= cfg.rail_lo) | (counts >= cfg.rail_hi)
        self.rail_hist.append((t, railed, counts.copy()))
        self.rail_sum = self.rail_sum + railed
        self.c_sum = self.c_sum + counts
        self.c2_sum = self.c2_sum + counts * counts
        while self.rail_hist and t - self.rail_hist[0][0] > cfg.rail_sustain_s:
            _, r0, c0 = self.rail_hist.popleft()
            self.rail_sum = self.rail_sum - r0
            self.c_sum = self.c_sum - c0
            self.c2_sum = self.c2_sum - c0 * c0
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= cfg.rail_sustain_s * 0.9
        nw = max(len(self.rail_hist), 1)
        self.rail = np.logical_and(spanned, self.rail_sum / nw >= cfg.rail_fraction)
        self.rail_std = np.sqrt(np.maximum(self.c2_sum / nw - (self.c_sum / nw) ** 2, 0.0))

    def sats(self, pinned, t):
        cfg = self.cfg
        self.sat_hist.append((t, pinned))
        self.sat_sum = self.sat_sum + pinned
        while self.sat_hist and t - self.sat_hist[0][0] > cfg.sat_sustain_s:
            self.sat_sum = self.sat_sum - self.sat_hist.popleft()[1]
        spanned = bool(self.sat_hist) and t - self.sat_hist[0][0] >= cfg.sat_sustain_s * 0.9
        self.sat = np.logical_and(
            spanned, self.sat_sum / max(len(self.sat_hist), 1) >= cfg.sat_fraction
        )

    def dead_pin(self):
        out = self.rail & (self.rail_std < self.cfg.dead_pin_std)
        return out & ~out.all()

    def rearm(self, t, kf_reset):
        drop = self.healthy & self.rail
        self.healthy[drop] = False
        self.clear_t[self.rail] = np.inf
        idle = ~self.healthy & ~self.rail & ~self.floor_bad
        starting = idle & np.isinf(self.clear_t)
        self.clear_t[starting] = t
        if starting.any():
            kf_reset(starting)
        back = idle & (t - self.clear_t >= self.cfg.rearm_s)
        self.healthy[back], self.clear_t[back] = (True, np.inf)
        return (drop, back)


class Breaker:

    def __init__(self, n, cfg):
        self.n, self.cfg = (n, cfg)
        self.bank = RMSBank(cfg.envelope_window_s, n)
        self.ht, self.he, self.h0, self.h1 = (np.zeros(64), np.zeros((n, 64)), 0, 0)
        self.valid_t = 0.0
        self.excess_since = np.full(n, np.inf)

    def arm(self, t):
        self.h0 = self.h1 = 0
        self.valid_t = t + self.cfg.envelope_window_s
        self.excess_since[:] = np.inf

    def reset(self, mask=None):
        self.bank.reset(mask)

    def sample(self, t, x, healthy, judge, baseline, sat):
        cfg = self.cfg
        env = self.bank.update(t, x, healthy)
        if t >= self.valid_t:
            if self.h1 == len(self.ht):
                k = self.h1 - self.h0
                ht, he = (np.zeros(max(4 * k, 64)), np.zeros((self.n, max(4 * k, 64))))
                ht[:k], he[:, :k] = (self.ht[self.h0 : self.h1], self.he[:, self.h0 : self.h1])
                self.ht, self.he, self.h0, self.h1 = (ht, he, 0, k)
            self.ht[self.h1], self.he[:, self.h1] = (t, env)
            self.h1 += 1
        while self.h0 < self.h1 and t - self.ht[self.h0] > cfg.runaway_lag_s * 2:
            self.h0 += 1
        he = self.he[:, self.h0 : self.h1]
        old = int(np.count_nonzero(t - self.ht[self.h0 : self.h1] >= cfg.envelope_window_s))
        past = he[:, :old].max(axis=1) if old else env
        peak = he.max(axis=1) if self.h1 > self.h0 else env
        growing = env > past * cfg.runaway_growth
        high = env > baseline * cfg.runaway_multiple
        receded = env < peak * cfg.sat_decay
        self.excess_since = hold(healthy & judge & high & growing, self.excess_since, t)
        run_trip = healthy & judge & ~receded & (t - self.excess_since >= cfg.runaway_sustain_s)
        sat_trip = healthy & sat & ~(env < past * cfg.sat_decay)
        return (env, run_trip, sat_trip)


class QuietLevel:

    def __init__(self, n, cfg):
        self.n, self.window_s, self.pct = (n, cfg.quiet_window_s, cfg.quiet_pct)
        self.keep_dt = 1.0 / cfg.quiet_keep_hz
        self.want = max(2, int(cfg.quiet_min_fill * cfg.quiet_window_s * cfg.quiet_keep_hz))
        self.hist = deque()
        self.reset()

    def reset(self):
        self.hist.clear()
        self.last_t, self._cache = (None, None)

    def add(self, t, env, mask):
        if self.last_t is not None and t - self.last_t < self.keep_dt:
            return
        self.last_t, self._cache = (t, None)
        self.hist.append((t, np.asarray(env, float).copy(), np.asarray(mask, bool).copy()))
        while self.hist and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()

    def level(self):
        if len(self.hist) < self.want:
            return None
        if self._cache is None:
            x = np.array([e for _, e, _ in self.hist])
            m = np.array([v for _, _, v in self.hist])
            out = np.zeros(self.n)
            for j in range(self.n):
                col = x[m[:, j], j]
                if col.size >= self.want:
                    out[j] = float(np.percentile(col, self.pct))
            self._cache = out
        return self._cache


class Controller:

    def __init__(self, rig, cfg=Config()):
        self.rig, self.cfg, self.n = (rig, cfg, rig.n)
        n = rig.n
        self.modal = Modal(rig, cfg)
        self.est = Estimator(rig, cfg)
        self.bias = rig.bias.copy()
        self.vmin = np.maximum(self.bias - cfg.bias_swing, cfg.vmin)
        self.vmax = np.minimum(self.bias + cfg.bias_swing, cfg.vmax)
        self.out = self.bias.copy()
        core = np.isin(rig.dof, list(GEOMETRY))
        strength = np.linalg.norm(rig.phi[:, core], axis=1)
        self.phi_rows = np.any(rig.phi != 0, axis=1)
        self.driven = self.modal.coils
        voters = strength >= 0.5 * strength.max()
        self.extra = np.any(rig.phi[:, ~core] != 0, axis=1) & ~voters
        self.vote = self.driven & voters if (self.driven & voters).any() else self.driven
        # a mode only these sensors see must be able to trip the breaker
        self.judge = self.vote | self.extra
        self.clock = Clock(cfg.control_hz)
        self.health = Health(n, cfg)
        self.brk = Breaker(n, cfg)
        self.rms_ratio = RMSBank(cfg.ratio_window_s, n)
        self.rms_lock = RMSBank(cfg.lock_window_s, n)
        self.quiet = QuietLevel(n, cfg)
        self.state, self.events = ("WARMUP", [])
        # a sensor is quiet when its band-passed rms is under this many times its own noise floor
        self.baseline = cfg.quiet_sigma * np.sqrt(rig.kalman_r)
        self.ratio = np.zeros(n)
        self.locked = np.zeros(n, bool)
        self.locked_announced, self.lock_time = (False, None)
        self.sense_ok = np.ones(n, bool)
        self.claimed = np.zeros(n, bool)
        self.warm_start, self.warm, self.damping_start = (0.0, [], None)
        self.clear_since = None
        # until it has locked once, a single failed engagement latches
        self.fault_count, self.failed = (0, cfg.max_failed_engagements - 1)
        self.stepped = False
        self.peak_demand, self.peak_sum = (0.0, 0.0)
        self._said = set()
        self._mot = np.zeros((3, n))
        self.motion, self.free = (np.zeros(n), np.zeros(n))

    @property
    def latched(self):
        return self.failed >= self.cfg.max_failed_engagements

    def say(self, msg):
        self.events.append(msg)

    def drain_events(self):
        out, self.events = (self.events, [])
        return out

    def _once(self, key, msg):
        if key not in self._said:
            self._said.add(key)
            self.say(msg)

    @staticmethod
    def _who(mask):
        return ", ".join(("a%d" % i for i in np.nonzero(mask)[0]))

    def _amp_ref(self):
        q = self.quiet.level()
        return self.baseline if q is None else np.maximum(self.baseline, q)

    def step(self, counts, t):
        counts = np.asarray(counts, float)
        self.stepped = False
        if not np.all((counts >= 0) & (counts <= self.cfg.adc_max)):
            return None
        self._mot += (np.ones(self.n), counts, counts**2)
        self.health.rails(counts, t)
        dt = self.clock.tick(t)
        volts = counts * self.cfg.volts_per_count
        self.stepped = True
        h = self.health
        self.sense_ok = h.healthy & ~h.floor_bad
        self.est.update(volts, dt, self.sense_ok & ~h.rail & self.phi_rows, self.out - self.bias)
        self.ratio = self.rms_ratio.update(t, self.est.bp, np.ones(self.n, bool)) / self.baseline
        if self.state == "WARMUP":
            self._warmup(t, counts)
        elif self.state == "DAMPING":
            self._damping(t, dt)
        else:
            self._faulted(t)
        return self._actuate(t, dt)

    def _fault(self, t, msg):
        self.say("!! " + msg)
        self.failed += self.state == "DAMPING"
        self.state, self.clear_since = ("FAULT", None)
        self.fault_count += 1

    def _warmup(self, t, counts):
        h = self.health
        self.warm.append(counts)
        railed = h.rail & ~h.dead_pin()
        if railed.any():
            return self._fault(t, "%s railed at start -- check alignment." % self._who(railed))
        if t - self.warm_start < self.cfg.warmup_s:
            return
        self.free = np.std(self.warm, axis=0)
        self._engage(t)
        self.say("gain on at %.1f s" % t)
        bad = (self.free < self.cfg.dead_pin_std) & ~self.extra
        if not bad.any() or bad.all():
            return
        h.floor_bad = bad
        h.healthy[bad], h.clear_t[bad] = (False, np.inf)
        self.say(
            "%s barely move, so lock is judged on the other sensors; their coils still drive."
            % self._who(bad)
        )

    def _engage(self, t):
        self.state, self.damping_start = ("DAMPING", t)
        self.brk.arm(t)
        self.quiet.reset()
        self.locked_announced = False

    def _damping(self, t, dt):
        cfg, h, bp = (self.cfg, self.health, self.est.bp)
        drop, back = h.rearm(t, self.est.kf.reset)
        if back.any():
            self.brk.reset(back)
            self.rms_ratio.reset(back)
            self.brk.excess_since[back] = np.inf
        live = h.healthy
        for i in np.nonzero(drop)[0]:
            self.say("!! ch%d railed -- sensor out, its coil held at bias." % i)
        for i in np.nonzero(back)[0]:
            self.say("[ch%d back] rail clear %.0fs." % (i, cfg.rearm_s))
        if int(live.sum()) < cfg.min_healthy:
            return self._fault(
                t,
                "quorum lost -- %d healthy, need %d." % (live.sum(), cfg.min_healthy),
            )
        self.modal.ramp(dt)
        env, run_trip, sat_trip = self.brk.sample(t, bp, live, self.judge, self._amp_ref(), h.sat)
        self.quiet.add(t, env, live)
        if run_trip.any():
            return self._fault(t, "%s runaway -- all coils to bias." % self._who(run_trip))
        if sat_trip.any():
            return self._fault(
                t,
                "%s pinned against the window, still demanding more, and not winning."
                % self._who(sat_trip),
            )
        # no lock before the window is full and the breaker has had its first look at growth
        seen = t - self.damping_start >= max(cfg.lock_window_s, 2.0 * cfg.envelope_window_s)
        self.locked = live & seen & (self.rms_lock.update(t, bp, live) < self.baseline)
        claim = live & self.vote
        calm = np.isinf(self.brk.excess_since[self.judge & live]).all()
        if claim.any() and self.locked[claim].all() and calm and (not self.locked_announced):
            self.locked_announced = True
            self.failed = 0
            self.lock_time = t - self.damping_start
            self.say("LOCKED %.1f s after gain" % self.lock_time)

    def _faulted(self, t):
        cfg, h = (self.cfg, self.health)
        self.modal.gain[:] = 0.0
        if self.latched:
            return self._once(
                "latch",
                "!! LATCHED OFF -- gain ended in a fault without the plate going quiet first. Coils stay at bias until restarted. Run `python debug.py`, then `run.py calib`, before trying again.",
            )
        watch = ~h.floor_bad
        if (h.rail & watch & ~h.dead_pin()).any() or (h.sat & watch).any():
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = t
        elif t - self.clear_since >= cfg.fault_clear_s:
            self._recover(t)

    def _recover(self, t):
        h = self.health
        h.sat_hist.clear()
        h.sat_sum[:], h.sat[:], h.healthy[:], h.floor_bad[:] = (0.0, False, True, False)
        h.clear_t[:] = self.brk.excess_since[:] = np.inf
        self.locked[:] = False
        self.modal.gain[:] = 0.0
        for part in (self.est, self.brk, self.rms_ratio, self.rms_lock):
            part.reset()
        self.state, self.warm_start, self.warm = ("WARMUP", t, [])
        self.clear_since = self.lock_time = None
        self.say("[recovered] clear for %.0f s -- starting again." % self.cfg.fault_clear_s)

    def _actuate(self, t, dt):
        cfg, h = (self.cfg, self.health)
        on = self.state == "DAMPING"
        # a quiet sensor says nothing about its coil; a railed one does
        coil_ok = (h.healthy | h.floor_bad) & on
        cmd, self.claimed = self.modal.solve(self.est.qdot, self.modal.gain, coil_ok, self.est.seen)
        if on and (not self.claimed.any()) and self.est.seen.any():
            self._once("nomodal", "[modal] no reachable mode with these coils and sensors.")
        # one factor on every coil keeps the force direction
        over = float((self.bias + cmd).sum()) - cfg.sum_v_max
        if over > 0 and cmd.sum() > 0:
            cmd = cmd * max(1.0 - over / float(cmd.sum()), 0.0)
        self.peak_demand = max(self.peak_demand, float(np.abs(cmd).max()))
        self.peak_sum = max(self.peak_sum, float((self.bias + cmd).sum()))
        target = np.where(coil_ok, np.clip(self.bias + cmd, self.vmin, self.vmax), self.bias)
        step = cfg.slew_per_s * dt
        self.out = self.out + np.clip(target - self.out, -step, step)
        # slewing can leave the rising coils ahead of the falling ones: hold the sum after it too
        up, over = (float((self.out - self.bias).sum()), float(self.out.sum()) - cfg.sum_v_max)
        if over > 0 and up > 0:
            self.out = self.bias + (self.out - self.bias) * max(1.0 - over / up, 0.0)
        h.sats((self.out <= self.vmin + 1e-06) | (self.out >= self.vmax - 1e-06), t)
        return self.out

    def banner(self):
        r = self.rig
        return "rig     calibrated %.1f days ago; modes %s Hz; %d coils in the loop" % (
            r.age_days(),
            " / ".join(("%.3f" % f for f in r.f_hz)),
            self.driven.sum(),
        )

    def status_header(self):
        cols = "".join(("%7s" % ("a%d" % i) for i in range(self.n)))
        return Text(
            "\nhow much each sensor moves, in counts rms (1 count = 5 mV); smaller is better\n"
            "   time  state    " + cols + "   quieter",
            "dim",
        )

    def status_line(self, t):
        k, sx, sxx = self._mot
        if k[0] > 1:
            self.motion = np.sqrt(np.maximum(sxx / k - (sx / k) ** 2, 0.0))
        self._mot[:] = 0.0
        v, h = (self.vote, self.health)
        gain = self.free[v].mean() / max(self.motion[v].mean(), 1e-9) if self.free[v].any() else 0
        locked = self.state == "DAMPING" and self.locked_announced and self.locked[v].all()
        word, col = {
            "WARMUP": ("starting", "yellow"),
            "DAMPING": ("locked", "green") if locked else ("damping", "yellow"),
            "FAULT": ("FAULT", "red"),
        }[self.state]
        line = Text.assemble("%5.0f s  " % t, ("%-9s" % word, col))
        for i in range(self.n):
            style = (
                "dim"
                if h.floor_bad[i]
                else "red" if h.rail[i] or not h.healthy[i] else "green" if self.locked[i] else ""
            )
            line.append("%7.1f" % self.motion[i], style)
        return line.append("   %.0fx" % gain if self.state == "DAMPING" and gain else "")

    def csv_header(self):
        cols = ["t", "state", "ctl", "n_avg", "chi2"] + ["qdot%d" % m for m in range(self.rig.nm)]
        for i in range(self.n):
            cols += ["%s%d" % (k, i) for k in ("a", "bp", "vel", "out", "ratio", "law")]
        return cols

    def csv_row(self, t, counts):
        e = self.est
        row = ["%.6f" % t, self.state, "1" if self.stepped else "0", "1", "%.4f" % e.chi2]
        row += ["%.6f" % x for x in e.qdot]
        for i in range(self.n):
            row += [
                "%.4f" % counts[i],
                "%.6f" % e.bp[i],
                "%.6f" % e.vel[i],
                "%.5f" % self.out[i],
                "%.4f" % self.ratio[i],
                "M" if self.claimed[i] else "-",
            ]
        return row
