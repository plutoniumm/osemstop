"""rig -- everything MEASURED about the plant, in one file the controller is handed.

Nothing in 2.0 hardcodes a frequency, a slope, a mode shape or a coil map. They
live in rig.json, they carry the record they came from, and `identify` rebuilds
them from the two records 1.0's tools already write:

    a quiet census   1.0/status.py sensors   -> mode frequencies, which geometric
                                                DOF each mode is, Phi rows a4-a7
    a DC step pass   1.0/slopesign.py        -> D[sensor, coil], slopes, A

Run it after ANYBODY touches the hardware: a3's slope changed sign across one
re-seating and the two tilts swapped labels, and nothing in software sees either.
"""

import json
import os
import re
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(HERE, "rig.json")

# Corner sensors a0-a3, one plane, sensing along the plate normal. a0 is diagonal
# to a3 (the pairing that minimises warp, re-confirmed 2026-08-20).
GEOMETRY = {"Z": [1, 1, 1, 1], "T1": [1, 1, -1, -1], "T2": [1, -1, 1, -1]}
WARP = [1, -1, -1, 1]

_ARRAYS = ("f_hz slope slope_sigma phi a_dc dc kalman_r kalman_q kalman_k "
           "pid_weight bias").split()


@dataclass(frozen=True)
class Rig:
    measured: str                      # ISO time of the OLDEST record in here
    source: dict                       # field -> where the number came from
    dac_map: list                      # sensor index -> DAC channel
    f_hz: np.ndarray                   # (nm,) mode frequencies
    dof: list                          # (nm,) which geometric DOF each mode is
    slope: np.ndarray                  # (n,) counts/V, coil j on its OWN sensor
    slope_sigma: np.ndarray            # (n,) standard error of that
    phi: np.ndarray                    # (n, nm) mode shapes; 0 = not determined
    a_dc: np.ndarray                   # (nm, n) static modal response, counts/V
    a_coils: list                      # coils whose A column is resolved
    dc: np.ndarray                     # (n, n) D[sensor, coil], counts/V
    kalman_r: np.ndarray               # (n,) measurement variance, V^2
    kalman_q: np.ndarray               # (n, nm) process variance, V^2/s^3
    kalman_k: np.ndarray               # (n, 2nm+1) per-channel steady-state gain
    pid_weight: np.ndarray             # (n,) 0..1 scale on the per-channel gain
    bias: np.ndarray                   # (n,) V
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
        """A as a FORCE matrix: a static response is the force over omega^2."""
        return self.a_dc * (2.0 * np.pi * self.f_hz[:, None]) ** 2

    @property
    def slope_sign(self):
        """+-1 where the own-sensor slope is resolved at 3 sigma, else 0.

        Zero is a refusal, not a default: a wrong sign does not under-damp, it
        pumps, so an unresolved channel gets no per-channel gain at all.
        """
        ok = np.abs(self.slope) >= 3.0 * self.slope_sigma
        return np.where(ok, np.sign(self.slope), 0.0)

    def age_days(self, now=None):
        t = datetime.fromisoformat(self.measured)
        return ((now or datetime.now()) - t).total_seconds() / 86400.0

    def save(self, path=PATH):
        d = {k: (v.tolist() if isinstance(v, np.ndarray) else v)
             for k, v in asdict(self).items()}
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(d, fh, indent=1)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path=PATH):
        with open(path) as fh:
            return cls(**json.load(fh))


# ---------------------------------------------------------------------------
# identify: two records -> a Rig
# ---------------------------------------------------------------------------
def _stamp(path):
    m = re.match(r"(\d{8})_(\d{6})", os.path.basename(path))
    t = (datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S") if m
         else datetime.fromtimestamp(os.path.getmtime(path)))
    return t.isoformat(timespec="seconds")


def _despike(c, jump=150.0):
    """Replace single samples that leap from both neighbours by their mean.

    1.0's ASCII records lose a digit on the wire about once in 67 000 rows
    (717 -> 7, 595 -> 57; 2026-10-05) and that passes any 0..1023 range check.
    Firmware 2's frames are checksummed and never need this.
    """
    c = c.copy()
    mid = 0.5 * (c[:-2] + c[2:])
    bad = (np.abs(c[1:-1] - mid) > jump) & (np.abs(c[:-2] - c[2:]) < jump)
    c[1:-1][bad] = mid[bad]
    return c, int(bad.sum())


def _uniform(t, y):
    """Resample onto a uniform clock: the wire's timestamps are not one."""
    fs = (len(t) - 1) / (t[-1] - t[0])
    tu = t[0] + np.arange(len(t)) / fs
    return fs, np.column_stack([np.interp(tu, t, y[:, i]) for i in range(y.shape[1])])


def _spectrum(fs, y, pad=1):
    y = y - y.mean(axis=0)
    w = np.hanning(len(y))[:, None]
    return (np.fft.rfftfreq(len(y) * pad, 1.0 / fs),
            np.fft.rfft(y * w, n=len(y) * pad, axis=0))


def _peak(f, p, f0, half):
    """Interpolated peak of power `p` within f0 +- half, and whether it hit an edge.

    A peak at the edge of its search window is not a peak: that cost a factor of
    2 on 2026-08-17, so the edge is reported rather than quietly returned.
    """
    band = np.nonzero(np.abs(f - f0) <= half)[0]
    k = band[np.argmax(p[band])]
    edge = k in (band[0], band[-1])
    a, b, c = np.log(p[k - 1:k + 2])
    return f[k] + 0.5 * (a - c) / (a - 2 * b + c) * (f[1] - f[0]), edge


def dc_matrix(path, n=8, f_hz=None):
    """(D, sigma), both [sensor, coil] in counts/V, from a slopesign-style record:
    each coil held at two levels in turn, D the level shift per volt between them.

    With `f_hz`, each dwell is fitted as a constant PLUS a sinusoid at every mode
    frequency and the constant is the level. The step itself rings the plate
    (tau 138 s, so it never dies inside a dwell), and a plain mean over 12 s
    still carries that ring at ~A/(pi f T) -- which was the whole error bar.
    Without `f_hz` the level is the plain mean, as 1.0's slopesign.py takes it.

    Sigma is the standard error of the level from ~1 s block means of what is
    left after the fit: samples are not independent in the mode band.
    """
    raw = np.loadtxt(path, delimiter=",", skiprows=1,
                     usecols=[0, 3, 6] + list(range(7, 7 + n)))
    t, coil, u, c = raw[:, 0], raw[:, 1], raw[:, 2], _despike(raw[:, 3:])[0]
    D, sD = np.full((n, n), np.nan), np.full((n, n), np.nan)

    def blocks(x, tt):
        return np.array([b.mean(0) for b in
                         np.array_split(x, max(int(tt[-1] - tt[0]), 2))])

    def level(x, tt):
        """(level, residual) of one dwell."""
        cols = [np.ones(len(tt))]
        for fm in ([] if f_hz is None else f_hz):
            cols += [np.cos(2 * np.pi * fm * tt), np.sin(2 * np.pi * fm * tt)]
        G = np.column_stack(cols)
        beta = np.linalg.lstsq(G, x, rcond=None)[0]
        return beta[0], x - G @ beta + beta[0]

    for j in range(n):
        me = coil == j
        if not me.any():
            continue
        # The two levels it DWELT at, i.e. the two with the most samples. Not
        # max/min: the settle back at bias carries the coil's tag too, and once
        # the bias sits above the step that short settle would be read as "high".
        lv, cnt = np.unique(np.round(u[me], 3), return_counts=True)
        v_lo, v_hi = np.sort(lv[np.argsort(cnt)[-2:]])
        hi, lo = me & (np.abs(u - v_hi) < 1e-3), me & (np.abs(u - v_lo) < 1e-3)
        dv = v_hi - v_lo
        (mh, rh), (ml, rl) = level(c[hi], t[hi]), level(c[lo], t[lo])
        bh, bl = blocks(rh, t[hi]), blocks(rl, t[lo])
        D[:, j] = (mh - ml) / dv
        sD[:, j] = np.sqrt(bh.var(0, ddof=1) / len(bh)
                           + bl.var(0, ddof=1) / len(bl)) / dv
    if np.isnan(D).any():
        raise ValueError("DC pass covers coils %s, need all %d"
                         % (sorted(set(coil[coil >= 0].astype(int))), n))
    return D, sD


def steady_gain(r, q, q_dc, f, dt, iters=60000, tol=1e-12):
    """(n, 2nm+1) steady-state Kalman gain, one row per sensor, for the
    per-channel filter: an undamped oscillator per mode plus a DC random walk.

    The process noise is EXACTLY stdlib.ModalKalman's: white force on each
    oscillator, discretised in closed form, and q_dc * dt on the DC state. That
    is also 1.0's analysis/kalman.py, which is where eta's KALMAN_K literals
    came from -- this reproduces them (see CLAUDE.md, 2026-10-05).
    stdlib.steady_state_k does NOT: it puts q_dc on the DC state without the dt.
    """
    r, q, q_dc = np.asarray(r, float), np.asarray(q, float), np.asarray(q_dc, float)
    w, nm = 2.0 * np.pi * np.asarray(f, float), len(f)
    nx = 2 * nm + 1
    F, H = np.eye(nx), np.zeros(nx)
    H[0:2 * nm:2], H[-1] = 1.0, 1.0
    for m, wm in enumerate(w):
        c, sn = np.cos(wm * dt), np.sin(wm * dt)
        F[2 * m:2 * m + 2, 2 * m:2 * m + 2] = [[c, sn / wm], [-wm * sn, c]]
    K = np.zeros((len(r), nx))
    for i in range(len(r)):
        Q = np.zeros((nx, nx))
        for m, wm in enumerate(w):
            sn, s2 = np.sin(wm * dt), np.sin(2.0 * wm * dt)
            Q[2 * m:2 * m + 2, 2 * m:2 * m + 2] = q[i, m] * np.array(
                [[(dt / 2 - s2 / (4 * wm)) / wm ** 2, sn * sn / (2 * wm * wm)],
                 [sn * sn / (2 * wm * wm), dt / 2 + s2 / (4 * wm)]])
        Q[-1, -1] = q_dc[i] * dt
        P, prev = np.eye(nx), None
        for _ in range(iters):
            Pm = F @ P @ F.T + Q
            k = (Pm @ H) / (H @ Pm @ H + r[i])
            P = (np.eye(nx) - np.outer(k, H)) @ Pm
            P = 0.5 * (P + P.T)
            if prev is not None and np.abs(k - prev).max() < tol * max(1.0, np.abs(k).max()):
                break
            prev = k
        K[i] = k
    return K


def noise_model(fs, y, f_hz, volts_per_count, control_hz=100.0, t_amp_s=50.0,
                skip_s=20.0, half=0.15, mode_half=0.06):
    """(R, Q) for the Kalman filters, from a passive record, in volts.

    Measured on what the filter actually sees: the per-control-step mean with
    the two-step mains null. R is everything the model does not carry -- the
    spectrum over 0.2 Hz..Nyquist with the mode bands bridged. Q follows from
    each mode's measured variance in each sensor: q = 2 w^2 sigma^2 / t_amp,
    the white force that grows that variance in t_amp seconds.
    """
    v = y[int(skip_s * fs):] * volts_per_count
    k = (np.arange(len(v)) * control_hz / fs).astype(int)
    v = np.array([np.bincount(k, v[:, i]) / np.bincount(k) for i in range(v.shape[1])]).T
    v = 0.5 * (v[1:] + v[:-1])                           # mains null
    nper = 1 << int(np.log2(len(v) / 3))                 # >= 5 half-overlapped segments
    win = np.hanning(nper)
    f = np.fft.rfftfreq(nper, 1.0 / control_hz)
    P = np.mean([np.abs(np.fft.rfft((v[a:a + nper] - v[a:a + nper].mean(0))
                                    * win[:, None], axis=0)) ** 2
                 for a in range(0, len(v) - nper + 1, nper // 2)], axis=0)
    P *= 2.0 / (control_hz * (win ** 2).sum())
    df, w = f[1] - f[0], 2.0 * np.pi * np.asarray(f_hz)
    modes = np.any(np.abs(f[:, None] - np.asarray(f_hz)[None, :]) <= half, axis=1)
    keep, band = ~modes & (f >= 0.2), f >= 0.2
    R = np.array([np.trapezoid(np.interp(f, f[keep], P[keep, i])[band], f[band])
                  for i in range(v.shape[1])])
    sig2 = np.array([[P[np.abs(f - fm) <= mode_half, i].sum() * df for fm in f_hz]
                     for i in range(v.shape[1])])
    return R, 2.0 * (w ** 2)[None, :] * sig2 / t_amp_s


def coherent_fraction(fs, y, lo=0.6, hi=1.8, segments=24):
    """Per sensor, how much of its in-band motion the four corners explain:
    multiple coherence with a0-a3, bias-corrected (4 predictors over K segments
    reads 4/K on pure noise). The corners themselves are 1 by definition."""
    seg = np.array_split(y, segments)
    n = min(len(s) for s in seg)
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


def identify(census, dc_pass, prior, segments=10, sigmas=4.0, half_hz=0.05,
             volts_per_count=5.02 / 1023, control_hz=100.0, t_amp_s=50.0,
             t_dc_s=20.0, say=print):
    """A new Rig from a quiet census and a DC step pass. EVERY measured number is
    refreshed. `prior` supplies only the frequency guesses, the wiring and the
    bias, which these two records cannot tell us."""
    n, nm = prior.n, prior.nm
    raw = np.loadtxt(census, delimiter=",", skiprows=1,
                     usecols=[0] + list(range(7, 7 + n)))
    clean, nbad = _despike(raw[:, 1:])
    fs, y = _uniform(raw[:, 0], clean)
    say("census %s: %d samples, %.0f s, %.1f Hz, %d torn sample(s) repaired"
        % (os.path.basename(census), len(y), len(y) / fs, fs, nbad))
    notes = []

    # 1. frequencies: consensus of the four corner sensors.
    f, Y = _spectrum(fs, y, pad=8)
    P = np.abs(Y) ** 2
    f_hz = np.zeros(nm)
    for m in range(nm):
        got = [_peak(f, P[:, i], prior.f_hz[m], half_hz) for i in range(4)]
        f_hz[m] = np.median([g[0] for g in got])
        spread = np.ptp([g[0] for g in got])
        if any(g[1] for g in got):
            notes.append("mode %d peak sits on the edge of its +-%.2f Hz search "
                         "window -- widen it, this frequency is not measured"
                         % (m, half_hz))
        say("  mode %d  %.5f Hz  (was %.5f, corner spread %.5f)"
            % (m, f_hz[m], prior.f_hz[m], spread))

    # 2. which geometric DOF each mode is. NOT cached: the tilts swapped once.
    G = {k: np.array(v, float) / 4.0 for k, v in GEOMETRY.items()}
    G["WARP"] = np.array(WARP, float) / 4.0
    dof = []
    for m in range(nm):
        band = np.abs(f - f_hz[m]) <= 0.02
        pw = {k: float((np.abs(Y[band, :4] @ g) ** 2).sum()) for k, g in G.items()}
        tot = sum(pw.values())
        best = max(GEOMETRY, key=pw.get)
        dof.append(best)
        say("  mode %d -> %-2s  %s" % (m, best, "  ".join(
            "%s %4.1f%%" % (k, 100 * v / tot) for k, v in pw.items())))
    if sorted(dof) != sorted(GEOMETRY):
        raise ValueError("modes map to %s, not one each of Z/T1/T2 -- the census "
                         "does not separate them; take a longer, quieter one" % dof)
    if dof != prior.dof:
        notes.append("mode labels CHANGED: %s -> %s" % (prior.dof, dof))

    # 3. Phi, MEASURED in counts, corners included. A mode is whatever the plate
    #    does at that frequency: it need not be a pure Z/T1/T2 (the highest mode
    #    was 66 % T1 with a0 near its node, 2026-10-05), and the four sensors need
    #    not share a counts-per-metre. The corner shape is the dominant
    #    eigenvector of the corners' in-band cross-spectrum, per time segment,
    #    scaled to the +-1 pattern's length and SIGNED to agree with the geometric
    #    pattern it is labelled as. There is no gauge to get wrong after that:
    #    A below is derived from this same Phi, so a flipped column flips its row.
    #    Every other sensor is regressed on that mode's coordinate in-band and
    #    kept only where it is resolved.
    phi = np.zeros((n, nm))
    seg = [rs[:min(len(x) for x in np.array_split(y, segments))]
           for rs in np.array_split(y, segments)]
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
        say("  Phi mode %d (%s): corners %s  +-%s   |cos| with +-1 pattern %.3f, "
            "rank-1 %.3f" % (m, k, " ".join("%+.3f" % x for x in phi[:4, m]),
                             "/".join("%.3f" % x for x in sem),
                             abs(phi[:4, m] @ g) / (2 * np.linalg.norm(phi[:4, m])),
                             rank1))
    pinv4 = np.linalg.pinv(phi[:4])
    say("  corner Phi: cond %.2f" % np.linalg.cond(phi[:4]))
    for m in range(nm):
        c = np.zeros((segments, n))
        for sgi, (fk, Yk) in enumerate(spec):
            Yb = Yk[np.abs(fk - f_hz[m]) <= bw]
            q = Yb[:, :4] @ pinv4[m]
            c[sgi] = np.real(Yb.T @ q.conj()) / float((np.abs(q) ** 2).sum())
        mean, sem = c.mean(0), c.std(0, ddof=1) / np.sqrt(segments)
        keep = np.abs(mean) >= sigmas * sem
        phi[4:, m] = np.where(keep[4:], mean[4:], 0.0)
        say("  Phi mode %d: %s" % (m, "  ".join(
            "a%d %+.4f+-%.4f%s" % (i, mean[i], sem[i], "" if keep[i] else " (0)")
            for i in range(4, n))))

    # 4. the DC matrix: one coil at a time, mean shift between its two levels.
    D, sD = dc_matrix(dc_pass, n, f_hz)
    slope, sigma = np.diag(D).copy(), np.diag(sD).copy()
    say("dc %s:" % os.path.basename(dc_pass))
    say("  own-sensor slope counts/V  " + "  ".join(
        "a%d %+.2f+-%.2f" % (i, slope[i], sigma[i]) for i in range(n)))
    flipped = [i for i in range(n)
               if prior.slope_sign[i] and np.sign(slope[i]) != prior.slope_sign[i]
               and abs(slope[i]) >= 3 * sigma[i]]
    if flipped:
        notes.append("slope SIGN CHANGED on %s since the prior rig"
                     % ", ".join("a%d" % i for i in flipped))

    # 5. A: the DC matrix projected onto the corner mode shapes, all coils.
    a_dc = pinv4 @ D[:4]
    s_a = np.sqrt((pinv4 ** 2) @ (sD[:4] ** 2))
    nsig = np.linalg.norm(a_dc, axis=0) / np.linalg.norm(s_a, axis=0)
    a_coils = [j for j in range(n) if nsig[j] >= 3.0]
    say("  A (counts/V), resolved coils %s:" % a_coils)
    for m in range(nm):
        say("    mode %d  %s" % (m, " ".join("%+8.2f" % x for x in a_dc[m])))
    say("    n-sigma %s   (a column under 3 is left out of the modal law)"
        % " ".join("%8.1f" % x for x in nsig))
    weak = [j for j in range(n) if j not in a_coils]
    if weak:
        notes.append("coils %s are unresolved in A (%s sigma) -- left out of the "
                     "modal law. Each gets PID only if its own-sensor slope is "
                     "resolved."
                     % (weak, "/".join("%.1f" % nsig[j] for j in weak)))

    # 6. the noise model the Kalman filters run on, and the per-channel gain.
    R, Q = noise_model(fs, y, f_hz, volts_per_count, control_hz, t_amp_s)
    K = steady_gain(R, Q, R / t_dc_s, f_hz, 1.0 / control_hz)
    say("  sqrt(R), counts  " + " ".join("%.2f" % x for x in np.sqrt(R) / volts_per_count))

    # 7. per-channel PID weight. Corners: equalise the colocated loop gain
    #    |slope| x gain to the weakest corner's, so no channel runs hotter than
    #    the ceiling was validated at (it can only REDUCE a gain). The rest: the
    #    fraction of their in-band motion that is the plate -- dissipation is
    #    linear in gain and injected noise quadratic, so that is the Wiener weight.
    ok = np.abs(slope) >= 3.0 * sigma
    weight = coherent_fraction(fs, y)
    weight[:4] = np.abs(slope[:4])[ok[:4]].min() / np.abs(slope[:4])
    weight = np.where(ok, np.minimum(weight, 1.0), 0.0)
    say("  pid weight       " + " ".join("%.2f" % x for x in weight))

    for line in notes:
        say("  !! " + line)
    return replace(
        prior, measured=min(_stamp(census), _stamp(dc_pass)), f_hz=f_hz, dof=dof,
        slope=slope, slope_sigma=sigma, phi=phi, a_dc=a_dc, a_coils=a_coils,
        dc=D, kalman_r=R, kalman_q=Q, kalman_k=K, pid_weight=weight, notes=notes,
        source=dict(
            dac_map=prior.source.get("dac_map", ""), bias=prior.source.get("bias", ""),
            f_hz=census, dof=census, phi=census + " (measured corner shapes)",
            slope=dc_pass, dc=dc_pass + " (ring fitted out)", a_dc=dc_pass,
            kalman_r=census, kalman_q=census, kalman_k="steady_gain(R, Q, f)",
            pid_weight="corners: equal |slope| x gain; others: coherent fraction, "
                       + census))
