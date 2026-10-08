import json
import os
import re
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime
import numpy as np
from scipy.signal import welch
from .filters import steady_gain

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(HERE, "rig.json")
DATA = os.path.join(os.path.dirname(HERE), "data")
GEOMETRY = {"Z": [1, 1, 1, 1], "T1": [1, 1, -1, -1], "T2": [1, -1, 1, -1]}
WARP = [1, -1, -1, 1]
_ARRAYS = "f_hz slope slope_sigma phi a_dc dc kalman_r kalman_q kalman_k bias".split()

SEGMENTS, SIGMAS, HALF_HZ = (10, 4.0, 0.05)


@dataclass(frozen=True)
class Config:
    vcc: float = 5.02
    adc_max: int = 1023
    control_hz: float = 225.0
    bias_swing: float = 0.3
    vmin: float = 0.0
    vmax: float = 2.5
    slew_per_s: float = 20.0
    budget_frac: float = 0.9
    # the supply folds back near this many volts summed over all coils
    sum_v_limit: float = 4.4
    sum_v_max: float = 3.6
    modal_kp: float = 0.035
    modal_scale: float = 4.0
    mode_gain: tuple = (("bounce", 0.3),)
    gain_slew_per_s: float = 0.1
    modal_cond_max: float = 12.0
    modal_min_sensors: int = 2
    modal_min_coils: int = 3
    t_amp_s: float = 50.0
    t_dc_s: float = 20.0
    warmup_s: float = 1.5
    quiet_sigma: float = 5.0
    lock_window_s: float = 2.0
    ratio_window_s: float = 1.0
    envelope_window_s: float = 2.0
    runaway_multiple: float = 1.8
    runaway_sustain_s: float = 5.0
    runaway_lag_s: float = 10.0
    runaway_growth: float = 1.02
    sat_sustain_s: float = 1.3
    sat_fraction: float = 0.8
    sat_decay: float = 0.98
    rail_lo: int = 12
    rail_hi: int = 1011
    rail_sustain_s: float = 0.5
    rail_fraction: float = 0.8
    dead_pin_std: float = 1.3
    rearm_s: float = 2.0
    min_healthy: int = 1
    fault_clear_s: float = 5.0
    max_failed_engagements: int = 2
    quiet_window_s: float = 30.0
    quiet_pct: float = 20.0
    quiet_keep_hz: float = 10.0
    quiet_min_fill: float = 0.8
    status_period_s: float = 5.0
    rig_max_age_days: float = 7.0

    @property
    def budget_v(self):
        return self.budget_frac * self.bias_swing

    @property
    def volts_per_count(self):
        return self.vcc / self.adc_max


@dataclass(frozen=True)
class Rig:
    measured: str
    source: dict
    dac_map: list
    f_hz: np.ndarray
    dof: list
    slope: np.ndarray
    slope_sigma: np.ndarray
    phi: np.ndarray
    a_dc: np.ndarray
    a_coils: list
    dc: np.ndarray
    kalman_r: np.ndarray
    kalman_q: np.ndarray
    kalman_k: np.ndarray
    bias: np.ndarray
    notes: list = field(default_factory=list)

    def __post_init__(self):
        for k in _ARRAYS:
            object.__setattr__(self, k, np.asarray(getattr(self, k), float))

    @property
    def n(self):
        return len(self.slope)

    @property
    def nm(self):
        return len(self.f_hz)

    @property
    def a(self):
        return self.a_dc * (2.0 * np.pi * self.f_hz[:, None]) ** 2

    @property
    def phi_unit(self):
        norm = np.linalg.norm(self.phi, axis=0)
        return self.phi / np.where(norm > 0, norm, 1.0)

    @property
    def a_unit(self):
        a = self.a.copy()
        norm = np.linalg.norm(a[:, self.a_coils], axis=1) if self.a_coils else np.zeros(self.nm)
        return a / np.where(norm > 0, norm, 1.0)[:, None]

    @property
    def slope_sign(self):
        ok = np.abs(self.slope) >= 3.0 * self.slope_sigma
        return np.where(ok, np.sign(self.slope), 0.0)

    def age_days(self, now=None):
        t = datetime.fromisoformat(self.measured)
        return ((now or datetime.now()) - t).total_seconds() / 86400.0

    def save(self, path=PATH):
        d = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in asdict(self).items()}
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(d, fh, indent=1)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path=PATH):
        with open(path) as fh:
            return cls(**json.load(fh))


def _stamp(path):
    m = re.match("(\\d{8})_(\\d{6})", os.path.basename(path))
    t = (
        datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
        if m
        else datetime.fromtimestamp(os.path.getmtime(path))
    )
    return t.isoformat(timespec="seconds")


def _despike(c, jump=150.0):
    c = c.copy()
    mid = 0.5 * (c[:-2] + c[2:])
    bad = (np.abs(c[1:-1] - mid) > jump) & (np.abs(c[:-2] - c[2:]) < jump)
    c[1:-1][bad] = mid[bad]
    return (c, int(bad.sum()))


def _load(path, n):
    raw = np.loadtxt(path, delimiter=",", skiprows=1, usecols=[0, 3, 6] + list(range(7, 7 + n)))
    return (raw[:, 0], raw[:, 1], raw[:, 2], *_despike(raw[:, 3:]))


def _uniform(t, y):
    fs = (len(t) - 1) / (t[-1] - t[0])
    tu = t[0] + np.arange(len(t)) / fs
    return (fs, np.column_stack([np.interp(tu, t, y[:, i]) for i in range(y.shape[1])]))


def _spectrum(fs, y, pad=1):
    y = y - y.mean(axis=0)
    w = np.hanning(len(y))[:, None]
    return (np.fft.rfftfreq(len(y) * pad, 1.0 / fs), np.fft.rfft(y * w, n=len(y) * pad, axis=0))


def _segments(fs, y):
    parts = np.array_split(y, SEGMENTS)
    short = min((len(p) for p in parts))
    return ([_spectrum(fs, p[:short]) for p in parts], 2.0 * SEGMENTS * fs / len(y))


def _shape(spec, bw, f0, ref, pre=lambda Z: Z):
    c = []
    for fk, Yk in spec:
        Yb = pre(Yk[np.abs(fk - f0) <= bw])
        q = ref(Yb)
        c.append(np.real(Yb.T @ q.conj()) / float((np.abs(q) ** 2).sum()))
    return (np.mean(c, 0), np.std(c, 0, ddof=1) / np.sqrt(SEGMENTS))


def _peak(f, p, f0, half):
    band = np.nonzero(np.abs(f - f0) <= half)[0]
    k = band[np.argmax(p[band])]
    edge = k in (band[0], band[-1])
    a, b, c = np.log(p[k - 1 : k + 2])
    return (f[k] + 0.5 * (a - c) / (a - 2 * b + c) * (f[1] - f[0]), edge)


def dc_matrix(path, n, f_hz):
    t, coil, u, c, _ = _load(path, n)
    D, sD = (np.full((n, n), np.nan), np.full((n, n), np.nan))

    def blocks(x, tt):
        return np.array([b.mean(0) for b in np.array_split(x, max(int(tt[-1] - tt[0]), 2))])

    def level(x, tt):
        cols = [np.ones(len(tt))]
        for fm in f_hz:
            cols += [np.cos(2 * np.pi * fm * tt), np.sin(2 * np.pi * fm * tt)]
        G = np.column_stack(cols)
        beta = np.linalg.lstsq(G, x, rcond=None)[0]
        return (beta[0], x - G @ beta + beta[0])

    for j in range(n):
        me = coil == j
        if not me.any():
            continue
        lv, cnt = np.unique(np.round(u[me], 3), return_counts=True)
        v_lo, v_hi = np.sort(lv[np.argsort(cnt)[-2:]])
        hi, lo = (me & (np.abs(u - v_hi) < 0.001), me & (np.abs(u - v_lo) < 0.001))
        dv = v_hi - v_lo
        (mh, rh), (ml, rl) = (level(c[hi], t[hi]), level(c[lo], t[lo]))
        bh, bl = (blocks(rh, t[hi]), blocks(rl, t[lo]))
        D[:, j] = (mh - ml) / dv
        sD[:, j] = np.sqrt(bh.var(0, ddof=1) / len(bh) + bl.var(0, ddof=1) / len(bl)) / dv
    if np.isnan(D).any():
        raise ValueError(
            "DC pass covers coils %s, need all %d" % (sorted(set(coil[coil >= 0].astype(int))), n)
        )
    return (D, sD)


def noise_model(fs, y, f_hz, cfg, rows):
    v = y[int(20.0 * fs) :] * cfg.volts_per_count
    nper = 1 << int(np.log2(len(v) / 3))
    f, P = welch(v, fs, np.hanning(nper), noverlap=nper // 2, axis=0)
    P[-1] *= 2.0  # scipy halves the Nyquist bin; R was fitted with it counted like the rest
    df, w = (f[1] - f[0], 2.0 * np.pi * np.asarray(f_hz))
    modes = np.any(np.abs(f[:, None] - np.asarray(f_hz)[None, :]) <= 0.15, axis=1)
    keep, band = (~modes & (f >= 0.2), f >= 0.2)
    R = np.array(
        [np.trapezoid(np.interp(f, f[keep], P[keep, i])[band], f[band]) for i in range(v.shape[1])]
    )
    sig2 = np.array(
        [[P[np.abs(f - fm) <= 0.06, i].sum() * df for fm in f_hz] for i in range(v.shape[1])]
    )
    return (R, np.where(rows, 2.0 * (w**2)[None, :] * sig2 / cfg.t_amp_s, 0.0))


def extra_modes(fs, y, f_hz, spec, bw, say):
    lo, hi, snr_min, rank_min = (0.3, 15.0, 100.0, 0.95)
    n = y.shape[1]
    f, Y = _spectrum(fs, y)
    df = f[1] - f[0]
    b = (f > 0.5 * min(f_hz)) & (f < 1.5 * max(f_hz))
    c = np.linalg.lstsq(
        np.r_[Y[b, :4].real, Y[b, :4].imag], np.r_[Y[b, 4:].real, Y[b, 4:].imag], rcond=None
    )[0]

    def rest(Z):
        return np.hstack([Z[:, :4], Z[:, 4:] - Z[:, :4] @ c])

    R = rest(Y)
    P = np.abs(R[:, 4:]) ** 2
    w = int(0.5 / df)

    def snr(k):
        return P[k] / np.median(P[max(k - w, 0) : k + w + 1], axis=0)

    ks = np.nonzero((f >= lo) & (f <= hi))[0]
    s = np.array([snr(k) for k in ks])
    mains = None
    km = np.nonzero((f >= 45.0) & (f <= 65.0))[0]
    if len(km):
        km = km[np.argmax(P[km].sum(1))]
        if snr(km).max() >= snr_min:
            mains = np.sqrt(P[km] / P[km].sum())
    out, done = ([], [])
    for j in np.argsort(s.max(1))[::-1]:
        k, i = (ks[j], int(np.argmax(s[j])))
        if s[j, i] < snr_min:
            break
        if any((abs(f[k] - g) < 0.1 for g in done)):
            continue
        done.append(f[k])
        f0 = _peak(f, P[:, i], f[k], 2 * df)[0]
        lo_k, hi_k = (k, k)
        while P[lo_k, i] > 0.5 * P[k, i]:
            lo_k -= 1
        while P[hi_k, i] > 0.5 * P[k, i]:
            hi_k += 1
        width = (hi_k - lo_k - 1) * df
        Rb = R[k - 3 : k + 4, 4:]
        ev = np.linalg.eigvalsh(Rb.T @ Rb.conj())
        rank1 = ev[-1] / ev.sum()
        h = np.round(f0 / np.asarray(f_hz))
        cm = 0.0 if mains is None else float(mains @ np.sqrt(P[k] / P[k].sum()))
        why = (
            "harmonic of a corner mode"
            if np.any((h >= 2) & (np.abs(f0 - h * np.asarray(f_hz)) < 2 * df))
            else (
                "mains pickup shape (cos %.2f)" % cm
                if cm > 0.9
                else (
                    "line %.3f Hz wide" % width
                    if width > max(3 * df, f0 / 200.0)
                    else "rank-1 under %.2f" % rank_min if rank1 < rank_min else ""
                )
            )
        )
        mean, sem = _shape(spec, bw, f0, lambda Yb: Yb[:, 4 + i], rest)
        col = np.where(np.abs(mean) >= SIGMAS * np.maximum(sem, 1e-12), mean, 0.0)
        col[4 + i] = 1.0
        x, v = (col[4:6], col[6:8])
        name = (
            "side"
            if np.abs(x).max() > np.abs(v).max()
            else "bounce" if v[0] * v[1] > 0 else "roll" if v[0] * v[1] < 0 else "extra"
        )
        say(
            "  line %8.4f Hz on a%d: SNR %.0f, rank-1 %.3f, width %.4f Hz -> %s"
            % (f0, 4 + i, s[j, i], rank1, width, "REJECTED, " + why if why else name)
        )
        if not why:
            say(
                "    shape %s"
                % "  ".join(
                    (
                        "a%d %+.3f+-%.3f%s" % (q, mean[q], sem[q], "" if col[q] else " (0)")
                        for q in range(n)
                    )
                )
            )
            out.append((f0, name, col))
    return out


def identify(census, dc_pass, prior, cfg=Config(), say=print):
    n, nm = (prior.n, len(GEOMETRY))
    t, _, _, clean, nbad = _load(census, n)
    fs, y = _uniform(t, clean)
    say(
        "census %s: %d samples, %.0f s, %.1f Hz, %d torn sample(s) repaired"
        % (os.path.basename(census), len(y), len(y) / fs, fs, nbad)
    )
    notes = []
    f, Y = _spectrum(fs, y, pad=8)
    P = np.abs(Y) ** 2
    f_hz = np.zeros(nm)
    for m in range(nm):
        got = [_peak(f, P[:, i], prior.f_hz[m], HALF_HZ) for i in range(4)]
        f_hz[m] = np.median([g[0] for g in got])
        spread = np.ptp([g[0] for g in got])
        if any((g[1] for g in got)):
            notes.append(
                "mode %d peak sits on the edge of its +-%.2f Hz search window -- widen it, this frequency is not measured"
                % (m, HALF_HZ)
            )
        say(
            "  mode %d  %.5f Hz  (was %.5f, corner spread %.5f)"
            % (m, f_hz[m], prior.f_hz[m], spread)
        )
    G = {k: np.array(v, float) / 4.0 for k, v in GEOMETRY.items()}
    G["WARP"] = np.array(WARP, float) / 4.0
    dof = []
    for m in range(nm):
        band = np.abs(f - f_hz[m]) <= 0.02
        pw = {k: float((np.abs(Y[band, :4] @ g) ** 2).sum()) for k, g in G.items()}
        tot = sum(pw.values())
        best = max(GEOMETRY, key=pw.get)
        dof.append(best)
        say(
            "  mode %d -> %-2s  %s"
            % (m, best, "  ".join(("%s %4.1f%%" % (k, 100 * v / tot) for k, v in pw.items())))
        )
    if sorted(dof) != sorted(GEOMETRY):
        raise ValueError(
            "modes map to %s, not one each of Z/T1/T2 -- the census does not separate them; take a longer, quieter one"
            % dof
        )
    if dof != prior.dof[:nm]:
        notes.append("mode labels CHANGED: %s -> %s" % (prior.dof[:nm], dof))
    phi = np.zeros((n, nm))
    spec, bw = _segments(fs, y)
    for m, k in enumerate(dof):
        g = np.array(GEOMETRY[k], float)
        shapes = []
        for fk, Yk in spec:
            Yb = Yk[np.abs(fk - f_hz[m]) <= bw][:, :4]
            ev, V = np.linalg.eigh(Yb.T @ Yb.conj())
            v = V[:, -1] * np.exp(-0.5j * np.angle((V[:, -1] ** 2).sum()))
            v = v.real / np.linalg.norm(v.real) * 2.0
            shapes.append(v * np.sign(v @ g))
            rank1 = ev[-1] / ev.sum()
        shapes = np.array(shapes)
        phi[:4, m] = shapes.mean(0)
        sem = shapes.std(0, ddof=1) / np.sqrt(SEGMENTS)
        say(
            "  Phi mode %d (%s): corners %s  +-%s   |cos| with +-1 pattern %.3f, rank-1 %.3f"
            % (
                m,
                k,
                " ".join(("%+.3f" % x for x in phi[:4, m])),
                "/".join(("%.3f" % x for x in sem)),
                abs(phi[:4, m] @ g) / (2 * np.linalg.norm(phi[:4, m])),
                rank1,
            )
        )
    pinv4 = np.linalg.pinv(phi[:4])
    say("  corner Phi: cond %.2f" % np.linalg.cond(phi[:4]))
    for m in range(nm):
        mean, sem = _shape(spec, bw, f_hz[m], lambda Yb: Yb[:, :4] @ pinv4[m])
        keep = np.abs(mean) >= SIGMAS * sem
        phi[4:, m] = np.where(keep[4:], mean[4:], 0.0)
        say(
            "  Phi mode %d: %s"
            % (
                m,
                "  ".join(
                    (
                        "a%d %+.4f+-%.4f%s" % (i, mean[i], sem[i], "" if keep[i] else " (0)")
                        for i in range(4, n)
                    )
                ),
            )
        )
    D, sD = dc_matrix(dc_pass, n, f_hz)
    slope, sigma = (np.diag(D).copy(), np.diag(sD).copy())
    say("dc %s:" % os.path.basename(dc_pass))
    say(
        "  own-sensor slope counts/V  "
        + "  ".join(("a%d %+.2f+-%.2f" % (i, slope[i], sigma[i]) for i in range(n)))
    )
    flipped = [
        i
        for i in range(n)
        if prior.slope_sign[i]
        and np.sign(slope[i]) != prior.slope_sign[i]
        and (abs(slope[i]) >= 3 * sigma[i])
    ]
    if flipped:
        notes.append(
            "slope SIGN CHANGED on %s since the prior rig" % ", ".join(("a%d" % i for i in flipped))
        )
    a_dc = pinv4 @ D[:4]
    s_a = np.sqrt(pinv4**2 @ sD[:4] ** 2)
    extra = extra_modes(fs, y, f_hz, spec, bw, say)
    # strongest line of each kind the coils are known to reach
    extra = [
        next(e for e in extra if e[1] == k)
        for k in ("side", "bounce")
        if k in {e[1] for e in extra}
    ]
    if extra:
        phix = np.column_stack([e[2] for e in extra])
        rows = np.any(phix != 0, axis=1)
        pin = np.linalg.pinv(phix[rows])
        # same Phi column on both sides: a sign flip of the shape flips this row with it
        a_dc = np.vstack([a_dc, pin @ (D[rows] - phi[rows] @ a_dc)])
        s_a = np.vstack([s_a, np.sqrt(pin**2 @ sD[rows] ** 2)])
        phi, f_hz = (np.hstack([phi, phix]), np.r_[f_hz, [e[0] for e in extra]])
        dof = dof + [e[1] for e in extra]
        nm = len(f_hz)
    nsig = np.linalg.norm(a_dc, axis=0) / np.linalg.norm(s_a, axis=0)
    a_coils = [j for j in range(n) if nsig[j] >= 3.0]
    say("  A (counts/V), resolved coils %s:" % a_coils)
    for m in range(nm):
        say("    mode %d  %s" % (m, " ".join(("%+8.2f" % x for x in a_dc[m]))))
    say(
        "    n-sigma %s   (a column under 3 is left out of the modal law)"
        % " ".join(("%8.1f" % x for x in nsig))
    )
    weak = [j for j in range(n) if j not in a_coils]
    if weak:
        notes.append(
            "coils %s are unresolved in A (%s sigma) -- left out of the modal law."
            % (weak, "/".join(("%.1f" % nsig[j] for j in weak)))
        )
    R, Q = noise_model(fs, y, f_hz, cfg, (phi != 0) | np.isin(dof, list(GEOMETRY))[None, :])
    K = steady_gain(R, Q, R / cfg.t_dc_s, f_hz, 1.0 / fs)
    say("  sqrt(R), counts  " + " ".join(("%.2f" % x for x in np.sqrt(R) / cfg.volts_per_count)))
    for line in notes:
        say("  !! " + line)
    return replace(
        prior,
        measured=min(_stamp(census), _stamp(dc_pass)),
        f_hz=f_hz,
        dof=dof,
        slope=slope,
        slope_sigma=sigma,
        phi=phi,
        a_dc=a_dc,
        a_coils=a_coils,
        dc=D,
        kalman_r=R,
        kalman_q=Q,
        kalman_k=K,
        notes=notes,
        source=dict(
            dac_map=prior.source.get("dac_map", ""),
            bias=prior.source.get("bias", ""),
            f_hz=census,
            dof=census,
            phi=census + " (measured corner shapes)",
            slope=dc_pass,
            dc=dc_pass + " (ring fitted out)",
            a_dc=dc_pass,
            kalman_r=census,
            kalman_q=census,
            kalman_k="steady_gain(R, Q, f)",
        ),
    )
