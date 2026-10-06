from collections import deque
import numpy as np
from .filters import RMSBank


class Health:

    def __init__(
        self,
        n,
        rail_lo,
        rail_hi,
        rail_sustain_s,
        rail_frac,
        sat_sustain_s,
        sat_frac,
        dead_std,
        rearm_s,
        floor_frac,
    ):
        self.n = int(n)
        self.rail_lo, self.rail_hi = (rail_lo, rail_hi)
        self.rail_sustain_s, self.rail_frac = (float(rail_sustain_s), float(rail_frac))
        self.sat_sustain_s, self.sat_frac = (float(sat_sustain_s), float(sat_frac))
        self.dead_std, self.rearm_s = (float(dead_std), float(rearm_s))
        self.floor_frac = float(floor_frac)
        self.rail = np.zeros(self.n, bool)
        self.sat = np.zeros(self.n, bool)
        self.healthy = np.ones(self.n, bool)
        self.floor_bad = np.zeros(self.n, bool)
        self.clear_t = np.full(self.n, np.inf)
        self.rail_std = np.zeros(self.n)
        self.rail_hist, self.sat_hist = (deque(), deque())
        self.rail_sum = np.zeros(self.n)
        self.sat_sum = np.zeros(self.n)
        self.c_sum, self.c2_sum = (np.zeros(self.n), np.zeros(self.n))

    @staticmethod
    def hold(cond, since, t):
        return np.where(cond, np.where(np.isinf(since), t, since), np.inf)

    def rails(self, counts, t):
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
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= self.rail_sustain_s * 0.9
        nw = max(len(self.rail_hist), 1)
        self.rail = np.logical_and(spanned, self.rail_sum / nw >= self.rail_frac)
        self.rail_std = np.sqrt(np.maximum(self.c2_sum / nw - (self.c_sum / nw) ** 2, 0.0))

    def sats(self, pinned, t):
        self.sat_hist.append((t, pinned))
        self.sat_sum = self.sat_sum + pinned
        while self.sat_hist and t - self.sat_hist[0][0] > self.sat_sustain_s:
            self.sat_sum = self.sat_sum - self.sat_hist.popleft()[1]
        spanned = bool(self.sat_hist) and t - self.sat_hist[0][0] >= self.sat_sustain_s * 0.9
        self.sat = np.logical_and(
            spanned, self.sat_sum / max(len(self.sat_hist), 1) >= self.sat_frac
        )

    def dead_pin(self, enabled):
        out = self.rail & (self.rail_std < self.dead_std) & enabled
        if out.any() and (not (enabled & ~out).any()):
            return (np.zeros(self.n, bool), False)
        return (out, True)

    def rearm(self, t, kf_reset=None):
        drop = self.healthy & self.rail
        self.healthy[drop] = False
        self.clear_t[self.rail] = np.inf
        idle = ~self.healthy & ~self.rail & ~self.floor_bad
        starting = idle & np.isinf(self.clear_t)
        self.clear_t[starting] = t
        if kf_reset is not None and starting.any():
            kf_reset(starting)
        back = idle & (t - self.clear_t >= self.rearm_s)
        self.healthy[back], self.clear_t[back] = (True, np.inf)
        return (drop, starting, back)

    def floor(self, x, enabled, exempt=None, motion=None):
        ex = (
            np.zeros(self.n, bool) if exempt is None or motion is None else np.asarray(exempt, bool)
        )
        ref = x[enabled & ~ex]
        if ref.size == 0 or not np.any(ref > 0):
            return (np.zeros(self.n, bool), 0.0, True)
        med = float(np.median(ref))
        bad = x < med * self.floor_frac
        if ex.any():
            mot = np.asarray(motion, float)
            mref = mot[enabled & ~ex]
            mmed = float(np.median(mref)) if mref.size else 0.0
            bad = np.where(ex, (mot < mmed * self.floor_frac) | (mot < self.dead_std), bad)
        if (bad & enabled).any() and (not (enabled & ~bad).any()):
            return (np.zeros(self.n, bool), med, False)
        return (bad, med, True)


class Breaker:

    def __init__(self, n, window_s, multiple, sustain_s, lag_s, growth_frac, decay_frac, quorum):
        self.n = int(n)
        self.window_s, self.multiple = (float(window_s), float(multiple))
        self.sustain_s, self.lag_s = (float(sustain_s), float(lag_s))
        self.growth_frac, self.decay_frac = (float(growth_frac), float(decay_frac))
        self.quorum = int(quorum)
        self.bank = RMSBank(self.window_s, self.n)
        self.hist = deque()
        self.valid_t = 0.0
        self.excess_since = np.full(self.n, np.inf)
        self.env = np.zeros(self.n)
        self.peak = np.zeros(self.n)
        self.receded = np.zeros(self.n, bool)

    def arm(self, t):
        self.hist.clear()
        self.valid_t = t + self.window_s
        self.excess_since[:] = np.inf

    def reset(self, mask=None):
        self.bank.reset(mask)

    def sample(self, t, x, healthy, drives, baseline, sat, judge=None):
        judge = drives if judge is None else judge
        env = self.bank.update(t, x, healthy)
        self.env = env
        if t >= self.valid_t:
            self.hist.append((t, env.copy()))
        while self.hist and t - self.hist[0][0] > self.lag_s * 2:
            self.hist.popleft()
        older = [e for ts, e in self.hist if t - ts >= self.window_s]
        past = np.maximum.reduce(older) if older else env
        peak = np.maximum.reduce([e for _, e in self.hist]) if self.hist else env
        growing = env > past * self.growth_frac
        high = env > baseline * self.multiple
        receded = env < peak * self.decay_frac
        self.peak, self.receded = (peak, receded)
        self.excess_since = Health.hold(healthy & judge & high & growing, self.excess_since, t)
        run_trip = healthy & judge & ~receded & (t - self.excess_since >= self.sustain_s)
        sat_trip = healthy & sat & ~(env < past * self.decay_frac)
        return dict(
            env=env,
            growing=growing,
            high=high,
            peak=peak,
            receded=receded,
            run_trip=run_trip,
            sat_trip=sat_trip,
        )


class QuietLevel:

    def __init__(self, n, window_s, percentile, keep_hz, min_fill):
        self.n = int(n)
        self.window_s, self.pct = (float(window_s), float(percentile))
        self.keep_dt = 1.0 / float(keep_hz)
        self.want = max(2, int(float(min_fill) * self.window_s * float(keep_hz)))
        self.hist = deque()
        self.last_t, self._cache, self._dirty = (None, None, True)

    def reset(self):
        self.hist.clear()
        self.last_t, self._cache, self._dirty = (None, None, True)

    def add(self, t, env, mask):
        if self.last_t is not None and t - self.last_t < self.keep_dt:
            return
        self.last_t = t
        self.hist.append((t, np.asarray(env, float).copy(), np.asarray(mask, bool).copy()))
        while self.hist and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        self._dirty = True

    def ready(self):
        return len(self.hist) >= self.want

    def level(self):
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
            self._cache, self._dirty = (out, False)
        return self._cache


class Baseline:

    def __init__(
        self,
        n,
        subwindow_s,
        n_subwindows,
        min_subwindows,
        agree_n,
        agree_tol,
        sanity_ratio,
        max_refusals,
        floor_frac,
    ):
        self.n = int(n)
        self.subwindow_s = float(subwindow_s)
        self.n_subwindows = int(n_subwindows)
        self.min_subwindows = int(min_subwindows)
        self.agree_n, self.agree_tol = (int(agree_n), float(agree_tol))
        self.sanity_ratio = float(sanity_ratio)
        self.max_refusals = int(max_refusals)
        self.floor_frac = float(floor_frac)
        self.trusted = None
        self.refusals = 0
        self.refused = False
        self.acc, self.sub_rms, self.sub_n0, self.sub_start = ([], [], 0, 0.0)

    def start(self, t):
        self.acc, self.sub_rms, self.sub_n0, self.sub_start = ([], [], 0, t)

    def add(self, x):
        self.acc.append(x)

    def close_subwindow(self, t):
        a = np.asarray(self.acc[self.sub_n0 :])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a**2).mean(0)))
        self.sub_n0, self.sub_start = (len(self.acc), t)

    def stationary(self):
        w = self.sub_rms
        if len(w) < max(self.min_subwindows, self.agree_n):
            return False
        a = np.asarray(w)
        tail = a[-self.agree_n :]
        if not bool((tail.max(0) / np.maximum(tail.min(0), 1e-09) <= self.agree_tol).all()):
            return False
        m_tail, m_all = (np.median(tail, 0), np.median(a, 0))
        spread = np.maximum(m_tail, m_all) / np.maximum(np.minimum(m_tail, m_all), 1e-09)
        return bool((spread <= self.agree_tol).all())

    def measure(self, early):
        if early and len(self.sub_rms) >= self.agree_n:
            return np.maximum(np.median(np.asarray(self.sub_rms[-self.agree_n :]), 0), 1e-06)
        a = np.asarray(self.acc)
        parts = [p for p in np.array_split(a, self.n_subwindows) if len(p)] if len(a) else []
        return (
            np.maximum(np.median([np.sqrt((p**2).mean(0)) for p in parts], 0), 1e-06)
            if parts
            else np.full(self.n, 1e-06)
        )

    def settle(self, early):
        fresh = self.measure(early)
        ref, self.refused = (self.trusted, False)
        note = None
        if ref is not None:
            live = ref > 0
            med = np.median(ref[live]) if live.any() else 0.0
            m = live & (ref >= med * self.floor_frac)
            bad = m & (fresh > self.sanity_ratio * ref)
            if bad.any() and self.refusals >= self.max_refusals:
                note = (
                    "!! fresh baseline refused %d times running and accepted anyway -- the floor really has moved, this is no longer a ringdown. The runaway breaker is now scaled off it."
                    % self.refusals
                )
                bad = np.zeros(self.n, bool)
            elif bad.any():
                self.refusals += 1
                self.refused = True
                note = (
                    "!! fresh baseline REFUSED -- "
                    + " / ".join(
                        (
                            "ch%d measured %.4fV against a trusted %.4fV (%.1fx)"
                            % (i, fresh[i], ref[i], fresh[i] / ref[i])
                            for i in np.nonzero(bad)[0]
                        )
                    )
                    + ", over %.1fx. That is a ringdown, not a floor. Keeping the baseline already in hand."
                    % self.sanity_ratio
                )
                self.start(0.0)
                return (np.maximum(ref, 1e-06), note)
        self.trusted, self.refusals = (fresh.copy(), 0)
        self.start(0.0)
        return (fresh, note)
