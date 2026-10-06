import json
import os
import re
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(HERE, "rig.json")
GEOMETRY = {"Z": [1, 1, 1, 1], "T1": [1, 1, -1, -1], "T2": [1, -1, 1, -1]}
WARP = [1, -1, -1, 1]
_ARRAYS = "f_hz slope slope_sigma phi a_dc dc kalman_r kalman_q kalman_k pid_weight bias".split()


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
    pid_weight: np.ndarray
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


def _uniform(t, y):
    fs = (len(t) - 1) / (t[-1] - t[0])
    tu = t[0] + np.arange(len(t)) / fs
    return (fs, np.column_stack([np.interp(tu, t, y[:, i]) for i in range(y.shape[1])]))


def _spectrum(fs, y, pad=1):
    y = y - y.mean(axis=0)
    w = np.hanning(len(y))[:, None]
    return (np.fft.rfftfreq(len(y) * pad, 1.0 / fs), np.fft.rfft(y * w, n=len(y) * pad, axis=0))


def _peak(f, p, f0, half):
    band = np.nonzero(np.abs(f - f0) <= half)[0]
    k = band[np.argmax(p[band])]
    edge = k in (band[0], band[-1])
    a, b, c = np.log(p[k - 1 : k + 2])
    return (f[k] + 0.5 * (a - c) / (a - 2 * b + c) * (f[1] - f[0]), edge)


def dc_matrix(path, n=8, f_hz=None):
    raw = np.loadtxt(path, delimiter=",", skiprows=1, usecols=[0, 3, 6] + list(range(7, 7 + n)))
    t, coil, u, c = (raw[:, 0], raw[:, 1], raw[:, 2], _despike(raw[:, 3:])[0])
    D, sD = (np.full((n, n), np.nan), np.full((n, n), np.nan))

    def blocks(x, tt):
        return np.array([b.mean(0) for b in np.array_split(x, max(int(tt[-1] - tt[0]), 2))])

    def level(x, tt):
        cols = [np.ones(len(tt))]
        for fm in [] if f_hz is None else f_hz:
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


def steady_gain(r, q, q_dc, f, dt, iters=60000, tol=1e-12):
    r, q, q_dc = (np.asarray(r, float), np.asarray(q, float), np.asarray(q_dc, float))
    w, nm = (2.0 * np.pi * np.asarray(f, float), len(f))
    nx = 2 * nm + 1
    F, H = (np.eye(nx), np.zeros(nx))
    H[0 : 2 * nm : 2], H[-1] = (1.0, 1.0)
    for m, wm in enumerate(w):
        c, sn = (np.cos(wm * dt), np.sin(wm * dt))
        F[2 * m : 2 * m + 2, 2 * m : 2 * m + 2] = [[c, sn / wm], [-wm * sn, c]]
    K = np.zeros((len(r), nx))
    for i in range(len(r)):
        Q = np.zeros((nx, nx))
        for m, wm in enumerate(w):
            sn, s2 = (np.sin(wm * dt), np.sin(2.0 * wm * dt))
            Q[2 * m : 2 * m + 2, 2 * m : 2 * m + 2] = q[i, m] * np.array(
                [
                    [(dt / 2 - s2 / (4 * wm)) / wm**2, sn * sn / (2 * wm * wm)],
                    [sn * sn / (2 * wm * wm), dt / 2 + s2 / (4 * wm)],
                ]
            )
        Q[-1, -1] = q_dc[i] * dt
        P, prev = (np.eye(nx), None)
        for _ in range(iters):
            Pm = F @ P @ F.T + Q
            k = Pm @ H / (H @ Pm @ H + r[i])
            P = (np.eye(nx) - np.outer(k, H)) @ Pm
            P = 0.5 * (P + P.T)
            if prev is not None and np.abs(k - prev).max() < tol * max(1.0, np.abs(k).max()):
                break
            prev = k
        K[i] = k
    return K


def noise_model(
    fs,
    y,
    f_hz,
    volts_per_count,
    control_hz=100.0,
    t_amp_s=50.0,
    skip_s=20.0,
    half=0.15,
    mode_half=0.06,
):
    v = y[int(skip_s * fs) :] * volts_per_count
    k = (np.arange(len(v)) * control_hz / fs).astype(int)
    v = np.array([np.bincount(k, v[:, i]) / np.bincount(k) for i in range(v.shape[1])]).T
    v = 0.5 * (v[1:] + v[:-1])
    nper = 1 << int(np.log2(len(v) / 3))
    win = np.hanning(nper)
    f = np.fft.rfftfreq(nper, 1.0 / control_hz)
    P = np.mean(
        [
            np.abs(np.fft.rfft((v[a : a + nper] - v[a : a + nper].mean(0)) * win[:, None], axis=0))
            ** 2
            for a in range(0, len(v) - nper + 1, nper // 2)
        ],
        axis=0,
    )
    P *= 2.0 / (control_hz * (win**2).sum())
    df, w = (f[1] - f[0], 2.0 * np.pi * np.asarray(f_hz))
    modes = np.any(np.abs(f[:, None] - np.asarray(f_hz)[None, :]) <= half, axis=1)
    keep, band = (~modes & (f >= 0.2), f >= 0.2)
    R = np.array(
        [np.trapezoid(np.interp(f, f[keep], P[keep, i])[band], f[band]) for i in range(v.shape[1])]
    )
    sig2 = np.array(
        [[P[np.abs(f - fm) <= mode_half, i].sum() * df for fm in f_hz] for i in range(v.shape[1])]
    )
    return (R, 2.0 * (w**2)[None, :] * sig2 / t_amp_s)


def coherent_fraction(fs, y, lo=0.6, hi=1.8, segments=24):
    seg = np.array_split(y, segments)
    n = min((len(s) for s in seg))
    S = 0
    for sg in seg:
        f, Y = _spectrum(fs, sg[:n])
        Yb = Y[(f >= lo) & (f <= hi)]
        S = S + Yb.T @ Yb.conj()
    out = np.ones(y.shape[1])
    for i in range(4, y.shape[1]):
        g = np.real(S[i, :4] @ np.linalg.solve(S[:4, :4], S[:4, i])) / S[i, i].real
        out[i] = max((g - 4.0 / segments) / (1.0 - 4.0 / segments), 0.0)
    return out


def identify(
    census,
    dc_pass,
    prior,
    segments=10,
    sigmas=4.0,
    half_hz=0.05,
    volts_per_count=5.02 / 1023,
    control_hz=100.0,
    t_amp_s=50.0,
    t_dc_s=20.0,
    say=print,
):
    n, nm = (prior.n, prior.nm)
    raw = np.loadtxt(census, delimiter=",", skiprows=1, usecols=[0] + list(range(7, 7 + n)))
    clean, nbad = _despike(raw[:, 1:])
    fs, y = _uniform(raw[:, 0], clean)
    say(
        "census %s: %d samples, %.0f s, %.1f Hz, %d torn sample(s) repaired"
        % (os.path.basename(census), len(y), len(y) / fs, fs, nbad)
    )
    notes = []
    f, Y = _spectrum(fs, y, pad=8)
    P = np.abs(Y) ** 2
    f_hz = np.zeros(nm)
    for m in range(nm):
        got = [_peak(f, P[:, i], prior.f_hz[m], half_hz) for i in range(4)]
        f_hz[m] = np.median([g[0] for g in got])
        spread = np.ptp([g[0] for g in got])
        if any((g[1] for g in got)):
            notes.append(
                "mode %d peak sits on the edge of its +-%.2f Hz search window -- widen it, this frequency is not measured"
                % (m, half_hz)
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
    if dof != prior.dof:
        notes.append("mode labels CHANGED: %s -> %s" % (prior.dof, dof))
    phi = np.zeros((n, nm))
    seg = [
        rs[: min((len(x) for x in np.array_split(y, segments)))]
        for rs in np.array_split(y, segments)
    ]
    spec = [_spectrum(fs, ys) for ys in seg]
    bw = 2.0 * segments * fs / len(y)
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
        sem = shapes.std(0, ddof=1) / np.sqrt(segments)
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
        c = np.zeros((segments, n))
        for sgi, (fk, Yk) in enumerate(spec):
            Yb = Yk[np.abs(fk - f_hz[m]) <= bw]
            q = Yb[:, :4] @ pinv4[m]
            c[sgi] = np.real(Yb.T @ q.conj()) / float((np.abs(q) ** 2).sum())
        mean, sem = (c.mean(0), c.std(0, ddof=1) / np.sqrt(segments))
        keep = np.abs(mean) >= sigmas * sem
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
            "coils %s are unresolved in A (%s sigma) -- left out of the modal law. Each gets PID only if its own-sensor slope is resolved."
            % (weak, "/".join(("%.1f" % nsig[j] for j in weak)))
        )
    R, Q = noise_model(fs, y, f_hz, volts_per_count, control_hz, t_amp_s)
    K = steady_gain(R, Q, R / t_dc_s, f_hz, 1.0 / control_hz)
    say("  sqrt(R), counts  " + " ".join(("%.2f" % x for x in np.sqrt(R) / volts_per_count)))
    ok = np.abs(slope) >= 3.0 * sigma
    weight = coherent_fraction(fs, y)
    weight[:4] = np.abs(slope[:4])[ok[:4]].min() / np.abs(slope[:4])
    weight = np.where(ok, np.minimum(weight, 1.0), 0.0)
    say("  pid weight       " + " ".join(("%.2f" % x for x in weight)))
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
        pid_weight=weight,
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
            pid_weight="corners: equal |slope| x gain; others: coherent fraction, " + census,
        ),
    )
