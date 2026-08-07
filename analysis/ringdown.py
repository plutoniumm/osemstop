#!/usr/bin/env python3
"""Settle f0 and Q from RINGDOWN data. Offline; reads only, opens no port.

    .venv/bin/python analysis/ringdown.py --cache   # rebuild the segment cache
    .venv/bin/python analysis/ringdown.py           # analyse from the cache

Every number in analysis/ringdown.md comes from here.

What this does that analysis/kp040.py did not
---------------------------------------------
kp040.py fitted an ENVELOPE (analytic-signal magnitude, log-linear least
squares) to 8 s windows of the controller's own bandpassed channel `chN_bp`.
An envelope fit throws the frequency away, so it cannot tell one mode from two,
and it cannot tell resonant decay from a beat note between two modes. It also
did not require the loop gain to be zero: 4587 of its 6824 windows are DAMPING
windows at |gain| up to 0.035 (see section 6 below), so their gamma is the
CLOSED-loop decay.

Here every fit is a damped sinusoid on the RAW counts,

    x(t) = sum_k A_k exp(-gamma_k t) cos(2 pi f_k t + phi_k) + c0 + c1 t

so f and gamma come out of one fit, and K = 1 vs K = 2 is decided by BIC.

Sources
-------
  data/*_fast_lock.csv    every controller run on disk, 2026-08-03 .. 2026-08-06
  analysis/out/decay_windows.csv   the earlier ENVELOPE fits, re-read in Sec. 6

Estimators, and why there are three
-----------------------------------
  A. TRANSIENT fit. Fit the model above directly to a gain-off record that
     starts at large amplitude. This is the ringdown in the literal sense. It
     is biased by ambient re-excitation: the suspension is never free, the
     ground keeps pumping it, so a transient that has decayed into the ambient
     level stops decaying. Guarded by only fitting while the amplitude is above
     AMB_FLOOR_MULT x the ambient RMS of the same channel.

  B. AUTOCORRELATION fit. For a stationary resonance driven by broadband
     ground motion, R(tau) has exactly the same damped-sinusoid form with the
     same gamma. This is the standard way to get Q out of ambient data, it uses
     every gain-off sample rather than only the post-kick ones, and it has no
     large-amplitude selection bias. It does assume the drive is broadband
     across the linewidth.

  C. LORENTZIAN PSD width. Independent frequency-domain check, and the only
     one of the three that states its own resolution limit out loud: a record
     of length T cannot resolve a linewidth narrower than about 1/T.

They are three views of one number. If they disagree, the disagreement is the
result and gets reported as such.
"""

import csv
import glob
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "analysis", "out")
CACHE = os.path.join(OUT, "ringdown_segments.npz")

FS_D = 20.0            # decimated rate, Hz. Modes are near 1 Hz.
F_LO, F_HI = 0.25, 4.0  # analysis band, Hz (flat across both modes)
EDGE_S = 0.5           # discard this much at each end after FFT shaping
MIN_SEG_S = 6.0        # shortest gain-off segment worth a transient fit
RAIL_LO, RAIL_HI = 1, 1022
AMB_FLOOR_MULT = 2.0   # stop a transient fit when env < this x ambient RMS
BOOT = 400             # bootstrap resamples for the pooled uncertainties


def hdr(t):
    print("\n" + "=" * 78)
    print(t)
    print("=" * 78)


def writecsv(name, header, rows):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)
    print(f"    -> analysis/out/{name}  ({len(rows)} rows)")


# ===========================================================================
# 1. Cache: every contiguous stretch of samples with ALL loop gains == 0
# ===========================================================================
def stream_gain_off(path):
    """Yield (t, counts[nch,N], railed[nch]) for each maximal gain-off run.

    Gain-off means every channel's logged chN_gain is exactly 0.0. That is the
    controller's own record of what it multiplied the velocity estimate by, so
    a zero there means no feedback force, whatever the state label says. The
    actuator still holds a static bias (chN_out is constant through these
    stretches) and a static force cannot damp.
    """
    with open(path) as fh:
        head = fh.readline().rstrip("\n").split(",")
        it, ist = head.index("time_s"), head.index("state")
        nch = sum(1 for c in head if c.endswith("_counts"))
        ic = [head.index(f"ch{i}_counts") for i in range(nch)]
        ig = [head.index(f"ch{i}_gain") for i in range(nch)]
        T, C, states = [], [], set()

        def flush():
            if len(T) < 64:
                return None
            t = np.asarray(T, float)
            c = np.asarray(C, np.int32).T
            # fraction of samples pinned at an ADC rail, per channel
            railfrac = ((c <= RAIL_LO) | (c >= RAIL_HI)).mean(axis=1)
            return t, c, railfrac, "+".join(sorted(states))

        for line in fh:
            f = line.split(",")
            try:
                t = float(f[it])
                g = [float(f[j]) for j in ig]
            except (ValueError, IndexError):
                continue
            if any(v != 0.0 for v in g):
                r = flush()
                if r:
                    yield r
                T, C, states = [], [], set()
                continue
            try:
                row = [int(f[j]) for j in ic]
            except ValueError:
                continue
            # a serial glitch shows up as a count outside the 10-bit ADC range.
            # Drop the whole row rather than let it into a least-squares fit.
            if any(v < 0 or v > 1023 for v in row):
                continue
            C.append(row)
            T.append(t)
            states.add(f[ist])
        r = flush()
        if r:
            yield r


def band_decimate(t, x):
    """Uniform-grid resample -> raised-cosine band shaping -> subsample."""
    fs = 1.0 / np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    xu = np.interp(tu, t, x)
    n = len(xu)
    A = np.vstack([tu - tu[0], np.ones(n)]).T
    coef, *_ = np.linalg.lstsq(A, xu, rcond=None)
    xu = xu - A @ coef
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    H = np.ones_like(fr)
    lo0, lo1 = F_LO * 0.5, F_LO
    hi0, hi1 = F_HI, F_HI * 1.5
    H[fr <= lo0] = 0.0
    m = (fr > lo0) & (fr < lo1)
    H[m] = 0.5 * (1 - np.cos(np.pi * (fr[m] - lo0) / (lo1 - lo0)))
    m = (fr > hi0) & (fr < hi1)
    H[m] = 0.5 * (1 + np.cos(np.pi * (fr[m] - hi0) / (hi1 - hi0)))
    H[fr >= hi1] = 0.0
    xf = np.fft.irfft(np.fft.rfft(xu) * H, n)
    k = max(1, int(round(fs / FS_D)))
    e = int(EDGE_S * fs)
    return tu[e:n - e:k], xf[e:n - e:k], fs / k, fs


def build_cache():
    files = sorted(glob.glob(os.path.join(ROOT, "data", "*_fast_lock.csv")))
    store, meta = {}, []
    for path in files:
        base = os.path.basename(path)
        for k, (t, c, railfrac, st) in enumerate(stream_gain_off(path)):
            dur = t[-1] - t[0]
            if dur < 3.0:
                continue
            nch = c.shape[0]
            td = None
            ys = []
            for i in range(nch):
                td, y, fsd, fs_raw = band_decimate(t, c[i].astype(float))
                ys.append(y)
            key = f"{base}#{k}"
            store[key + "|t"] = td.astype(np.float32)
            store[key + "|y"] = np.asarray(ys, np.float32)
            store[key + "|rail"] = railfrac.astype(np.float32)
            store[key + "|rms_raw"] = np.asarray(
                [c[i].astype(float).std() for i in range(nch)], np.float32)
            meta.append([base, k, st, float(t[0]), float(dur), nch,
                         float(fs_raw), float(fsd)]
                        + [float(v) for v in railfrac]
                        + [0.0] * (8 - nch))
            print(f"  {base} seg{k:>2} {st:<26} {dur:7.2f}s "
                  f"fs={fs_raw:7.1f} railfrac="
                  + " ".join(f"{v:.4f}" for v in railfrac), flush=True)
    np.savez_compressed(CACHE, **store)
    writecsv("ringdown_inventory.csv",
             ["run", "seg", "states", "t0_s", "dur_s", "nch", "fs_raw_hz",
              "fs_dec_hz"] + [f"railfrac_ch{i}" for i in range(8)], meta)
    print(f"\n  cached {len(meta)} gain-off segments -> {CACHE}")
    return meta


def load_cache():
    z = np.load(CACHE)
    keys = sorted({k.split("|")[0] for k in z.files})
    segs = []
    inv = {}
    with open(os.path.join(OUT, "ringdown_inventory.csv")) as fh:
        for r in csv.DictReader(fh):
            inv[f"{r['run']}#{r['seg']}"] = r
    for k in keys:
        m = inv[k]
        segs.append(dict(key=k, run=m["run"], states=m["states"],
                         t0=float(m["t0_s"]), dur=float(m["dur_s"]),
                         fs=float(m["fs_dec_hz"]), fs_raw=float(m["fs_raw_hz"]),
                         rail=z[k + "|rail"].astype(float),
                         t=z[k + "|t"].astype(float),
                         y=z[k + "|y"].astype(float),
                         rms_raw=z[k + "|rms_raw"].astype(float)))
    return segs


# a channel is unusable in a segment if more than this fraction of its samples
# sat on an ADC rail: a clipped sinusoid is not the sinusoid we are fitting.
RAIL_MAX_FRAC = 1e-3


def usable(s, ch):
    return ch < s["y"].shape[0] and s["rail"][ch] <= RAIL_MAX_FRAC


# ===========================================================================
# 2. Damped-sinusoid fitting:  matrix pencil for the start, VARPRO to finish
# ===========================================================================
def matrix_pencil(y, dt, K):
    """Poles of a sum of complex exponentials. No initial guess needed."""
    N = len(y)
    L = max(2 * K + 2, N // 3)
    if N - L < 2 * K + 2:
        return np.array([], complex)
    Y = np.lib.stride_tricks.sliding_window_view(y, L + 1)
    U, s, Vh = np.linalg.svd(Y[:, :-1], full_matrices=False)
    r = min(2 * K, int((s > s[0] * 1e-10).sum()))
    if r < 2:
        return np.array([], complex)
    A = (np.diag(1.0 / s[:r]) @ U[:, :r].conj().T @ Y[:, 1:] @ Vh[:r].conj().T)
    z = np.linalg.eigvals(A)
    z = z[np.abs(z) > 0]
    return np.log(z.astype(complex)) / dt


# Hard bounds on the nonlinear parameters. gamma is allowed slightly negative
# so that "the record did not decay" can come out as a number instead of being
# clamped to zero and read as a suspiciously good high-Q result.
G_MIN, G_MAX = -0.30, 3.0


def clamp(p, band=None):
    flo, fhi = band or (F_LO, F_HI)
    p = np.asarray(p, float).copy()
    p[0::2] = np.clip(p[0::2], G_MIN, G_MAX)
    p[1::2] = np.clip(np.abs(p[1::2]), flo, fhi)
    return p


def _design(p, t, K):
    cols = []
    for k in range(K):
        e = np.exp(np.clip(-p[2 * k] * t, -700.0, 700.0))
        w = 2 * np.pi * p[2 * k + 1] * t
        cols += [e * np.cos(w), e * np.sin(w)]
    cols += [np.ones_like(t), t]
    return np.vstack(cols).T


def _resid(p, t, y, K):
    A = _design(p, t, K)
    if not np.isfinite(A).all():
        return np.full_like(y, np.inf), A, np.zeros(A.shape[1])
    try:
        c, *_ = np.linalg.lstsq(A, y, rcond=None)
    except np.linalg.LinAlgError:
        try:
            c = np.linalg.pinv(A) @ y
        except np.linalg.LinAlgError:
            return np.full_like(y, np.inf), A, np.zeros(A.shape[1])
    return y - A @ c, A, c


def _cost(r):
    v = float(r @ r)
    return v if np.isfinite(v) else np.inf


def fit_damped(t, y, K, p0, maxit=120, band=None):
    """Separable (VARPRO) Levenberg-Marquardt over (gamma_k, f_k) only."""
    p = clamp(p0, band)
    r, _, _ = _resid(p, t, y, K)
    cost = _cost(r)
    lam = 1e-3
    dp = np.zeros_like(p)
    for _ in range(maxit):
        J = np.empty((len(t), len(p)))
        bad = False
        for j in range(len(p)):
            h = max(1e-7, abs(p[j]) * 1e-5)
            pp = p.copy()
            pp[j] += h
            rp, _, _ = _resid(pp, t, y, K)
            J[:, j] = (rp - r) / h
            bad |= not np.isfinite(J[:, j]).all()
        if bad:
            break
        JTJ, JTr = J.T @ J, J.T @ r
        ok = False
        for _ in range(40):
            try:
                dp = np.linalg.solve(
                    JTJ + lam * np.diag(np.maximum(np.diag(JTJ), 1e-14)), -JTr)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            if not np.isfinite(dp).all():
                lam *= 10
                continue
            pn = clamp(p + dp, band)
            rn, _, _ = _resid(pn, t, y, K)
            cn = _cost(rn)
            if cn < cost:
                p, r, cost, ok = pn, rn, cn, True
                lam = max(lam * 0.3, 1e-10)
                break
            lam *= 3.0
        if not ok or np.linalg.norm(dp) < 1e-11:
            break
    r, A, c = _resid(p, t, y, K)
    return p, c, r, _cost(r)


def seed(y, dt, K, flo=None, fhi=None):
    """Matrix-pencil seeds, in-band, least damped first; fall back to 1.0/1.65."""
    flo = F_LO if flo is None else flo
    fhi = F_HI if fhi is None else fhi
    try:
        sp = matrix_pencil(y, dt, max(K, 3))
    except np.linalg.LinAlgError:
        sp = np.array([], complex)
    cand = [q for q in sp
            if np.isfinite(q) and q.imag > 0 and flo < q.imag / (2 * np.pi) < fhi]
    cand.sort(key=lambda q: abs(q.real))          # least damped first
    out = []
    for q in cand[:K]:
        out += [float(np.clip(-q.real, G_MIN, G_MAX)),
                float(q.imag / (2 * np.pi))]
    for j in range(len(out) // 2, K):
        out += [0.05, float(np.clip([1.0, 1.65, 0.71][j % 3], flo, fhi))]
    return out


def score(t, y, K, p0, band=None):
    p, c, r, cost = fit_damped(t, y, K, p0, band=band)
    n = len(y)
    ss = float(((y - y.mean()) ** 2) .sum())
    npar = 4 * K + 2
    bic = n * math.log(max(cost, 1e-300) / n) + npar * math.log(n)
    return dict(p=p, c=c, cost=cost, r2=1 - cost / max(ss, 1e-30), bic=bic,
                n=n, K=K)


def modes_of(fit):
    """(f, gamma, Q, amplitude) per component, strongest amplitude first."""
    p, c = fit["p"], fit["c"]
    out = []
    for k in range(fit["K"]):
        g, f = p[2 * k], p[2 * k + 1]
        a = math.hypot(c[2 * k], c[2 * k + 1])
        out.append((f, g, (math.pi * f / g if g != 0 else np.inf), a))
    out.sort(key=lambda v: -v[3])
    return out


def envelope(y, fs):
    n = len(y)
    Y = np.fft.fft(y - y.mean())
    h = np.zeros(n)
    h[0] = 1
    if n % 2 == 0:
        h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[1:(n + 1) // 2] = 2
    e = np.abs(np.fft.ifft(Y * h))
    k = max(1, int(fs))
    return np.convolve(e, np.ones(k) / k, mode="same")


# ===========================================================================
# 3. Estimator A -- transient (true ringdown) fits
# ===========================================================================
def ambient_floor(segs, nch=8):
    """Per-channel ambient RMS, in decimated counts.

    Uses only a run's OPENING calibration (t0 < 1 s, no FAULT in the segment):
    at that point no feedback has ever been applied and nothing has kicked the
    optic, so the motion there is the ambient seismic level and nothing else.
    Those segments carry a DAMPING label on their last sample or two -- the
    gain is still exactly 0.0 there, it has not begun to slew -- so the label
    is not a useful filter and the FAULT label is used instead.
    """
    acc = [[] for _ in range(nch)]
    for s in segs:
        if s["t0"] > 1.0 or "FAULT" in s["states"]:
            continue
        for i in range(s["y"].shape[0]):
            if not usable(s, i):
                continue
            acc[i].append(float(s["y"][i].std()))
    return np.array([np.median(a) if a else np.nan for a in acc])


def transient_fits(segs, amb):
    rows, keep = [], []
    for s in segs:
        if s["dur"] < MIN_SEG_S:
            continue
        t, fs = s["t"] - s["t"][0], s["fs"]
        for i in range(s["y"].shape[0]):
            if not usable(s, i) or not np.isfinite(amb[i]) or amb[i] <= 0:
                continue
            y = s["y"][i]
            env = envelope(y, fs)
            a0 = float(env[:int(fs)].mean())
            if a0 < 3.0 * amb[i]:
                continue                       # not a kick: nothing to ring down
            below = np.flatnonzero(env < AMB_FLOOR_MULT * amb[i])
            end = int(below[0]) if below.size else len(y)
            if (end / fs) < MIN_SEG_S:
                end = min(len(y), int(MIN_SEG_S * fs))
            tt, yy = t[:end], y[:end]
            if len(tt) < int(4 * fs):
                continue
            f1 = score(tt, yy, 1, seed(yy, 1 / fs, 1))
            f2 = score(tt, yy, 2, seed(yy, 1 / fs, 2))
            best = f2 if f2["bic"] < f1["bic"] else f1
            m = modes_of(best)
            for (f, g, q, a) in m:
                if not (F_LO < f < F_HI) or not (1e-4 < g < 3.0):
                    continue
                keep.append((s["key"], i, f, g, q, a, a0 / amb[i],
                             best["K"], best["r2"], tt[-1]))
            rows.append([s["key"], s["run"], s["states"], i, s["dur"],
                         tt[-1], a0, a0 / amb[i],
                         f1["r2"], f2["r2"], f1["bic"], f2["bic"], best["K"]]
                        + [v for mm in (m + [(np.nan,) * 4] * (2 - len(m)))[:2]
                           for v in mm[:4]])
    return rows, keep


# ===========================================================================
# 4. Estimator B -- autocorrelation fits (uses every gain-off sample)
# ===========================================================================
def acf_unbiased(y, maxlag):
    n = len(y)
    y = y - y.mean()
    nf = 1 << int(np.ceil(np.log2(2 * n)))
    S = np.fft.rfft(y, nf)
    r = np.fft.irfft(S * S.conj(), nf)[:maxlag + 1]
    denom = n - np.arange(maxlag + 1)
    return r / denom


def acf_stack(segs, ch, maxlag_s, min_dur):
    """Average the unbiased ACF over every usable gain-off segment."""
    acc, wsum, used, tot = None, 0.0, [], 0.0
    for s in segs:
        if s["dur"] < min_dur or not usable(s, ch):
            continue
        fs = s["fs"]
        ml = int(maxlag_s * fs)
        y = s["y"][ch]
        if len(y) < 2 * ml:
            continue
        r = acf_unbiased(y, ml)
        w = len(y)                       # weight by record length
        acc = r * w if acc is None else acc + r * w
        wsum += w
        used.append(s["key"])
        tot += s["dur"]
    if acc is None:
        return None, None, [], 0.0
    r = acc / wsum
    lag = np.arange(len(r)) / fs
    return lag, r, used, tot


ACF_FLO, ACF_FHI = 0.45, 2.60   # the band the suspension modes actually live in


def fit_acf(lag, r, K):
    y = r / r[0]
    return score(lag, y, K, seed(y, lag[1] - lag[0], K, ACF_FLO, ACF_FHI),
                 band=(ACF_FLO, ACF_FHI))


# ===========================================================================
# 5. Estimator C -- Lorentzian PSD widths, with the resolution stated
# ===========================================================================
def welch(y, fs, nseg):
    if len(y) < nseg:
        return None, None
    w = np.hanning(nseg)
    step = nseg // 2
    segs = [y[i:i + nseg] for i in range(0, len(y) - nseg + 1, step)]
    P = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2 for s in segs],
                axis=0) / (fs * (w ** 2).sum())
    return np.fft.rfftfreq(nseg, 1 / fs), P


def lorentz_fit(fr, P, f_guess, half=0.20):
    """Least squares of P(f) = A * (f0 g/pi)/((f^2-f0^2)^2 + (g f/pi)^2) + B
    over a window around f_guess. Returns f0, gamma. Grid + local refine."""
    m = (fr > f_guess - half) & (fr < f_guess + half)
    f, p = fr[m], P[m]
    if len(f) < 6:
        return np.nan, np.nan, np.nan
    best = None
    for f0 in np.linspace(f_guess - 0.08, f_guess + 0.08, 65):
        for g in np.logspace(np.log10(3e-3), np.log10(1.5), 80):
            shape = 1.0 / ((f ** 2 - f0 ** 2) ** 2 + (g * f / np.pi) ** 2)
            A = np.vstack([shape, np.ones_like(f)]).T
            c, *_ = np.linalg.lstsq(A, p, rcond=None)
            if c[0] <= 0:
                continue
            res = p - A @ c
            v = float(res @ res)
            if best is None or v < best[0]:
                best = (v, f0, g, c)
    if best is None:
        return np.nan, np.nan, np.nan
    ss = float(((p - p.mean()) ** 2).sum())
    return best[1], best[2], 1 - best[0] / max(ss, 1e-30)


# ===========================================================================
# 6. What the earlier ENVELOPE fits actually measured
# ===========================================================================
def reread_envelope_fits():
    path = os.path.join(OUT, "decay_windows.csv")
    if not os.path.exists(path):
        print("  analysis/out/decay_windows.csv absent, skipping")
        return
    rows = list(csv.DictReader(open(path)))
    g = np.array([float(r["gain_mean"]) for r in rows])
    gs = np.array([float(r["gain_std"]) for r in rows])
    gam = np.array([float(r["gamma_1_s"]) for r in rows])
    r2 = np.array([float(r["r2"]) for r in rows])
    st = np.array([r["state"] for r in rows])
    off = np.abs(g) < 1e-12
    print(f"  total windows                       {len(rows)}")
    print(f"  gain identically zero               {int(off.sum())}"
          f"   ({100*off.mean():.1f}%)")
    print(f"  loop gain NON-zero (closed loop)    {int((~off).sum())}"
          f"   ({100*(~off).mean():.1f}%)")
    hi = (~off) & (np.abs(g) > 0.02) & (r2 > 0.80) & (gs < 5e-4)
    print(f"\n  closed-loop windows, |gain|>0.02, r2>0.8, steady gain: "
          f"{int(hi.sum())}")
    print(f"    median gamma = {np.median(gam[hi]):.4f} 1/s"
          f"   -> Q(f=1.00 Hz) = {math.pi * 1.0 / np.median(gam[hi]):.1f}")
    print(f"    (this is the number the 'Q = 20-22' claim came from)")
    good = off & (r2 > 0.80)
    print(f"\n  gain-off windows                    {int(off.sum())}, "
          f"of which r2 > 0.80: {int(good.sum())} "
          f"({100*good.sum()/max(off.sum(),1):.1f}%)")
    print(f"    gain-off median r2   = {np.median(r2[off]):.3f}")
    print(f"    gain-off median gamma= {np.median(gam[off]):+.4f} 1/s"
          f"  (IQR {np.percentile(gam[off],25):+.4f} .. "
          f"{np.percentile(gam[off],75):+.4f})")
    print("    an r2 that low means the log-envelope is not a straight line:")
    print("    the open-loop envelope is not a decaying exponential at all.")
    for s in ("CALIBRATING", "FAULT", "DAMPING"):
        m = off & (st == s)
        if m.sum():
            print(f"      {s:<12} n={int(m.sum()):5d} "
                  f"median gamma={np.median(gam[m]):+.4f} "
                  f"median r2={np.median(r2[m]):.3f} "
                  f"frac gamma<0: {100*(gam[m]<0).mean():.0f}%")

    # -------- the windows that carried damping_vs_gain.csv's |gain|=0.0 rows
    from collections import Counter, defaultdict
    R = []
    for r in rows:
        R.append(dict(run=r["run"], st=r["state"], ch=int(r["ch"]),
                      t0=round(float(r["t0_s"]), 3), g=float(r["gain_mean"]),
                      gs=float(r["gain_std"]), gam=float(r["gamma_1_s"]),
                      r2=float(r["r2"]), rat=float(r["env_ratio"])))
    sib = defaultdict(dict)
    for r in R:
        sib[(r["run"], r["st"], r["t0"])][r["ch"]] = r
    # kp040's own selection, reproduced: DAMPING, R2>0.80, envelope falls,
    # gain constant, |gain| rounds to 0.000 -- the "|gain|=0.0" rows of
    # analysis/out/damping_vs_gain.csv, i.e. the source of "Q = 20-22".
    sel = [r for r in R if r["st"] == "DAMPING" and abs(r["g"]) < 5e-4
           and r["gs"] < 5e-4 and r["r2"] > 0.80 and r["rat"] < 0.90]
    cnt = Counter()
    gam_sel = []
    for r in sel:
        oth = [v["g"] for c, v in sib[(r["run"], r["st"], r["t0"])].items()
               if c != r["ch"]]
        cnt["others_driven" if any(abs(v) > 1e-9 for v in oth)
            else "all_channels_off"] += 1
        gam_sel.append(r["gam"])
    print(f"\n  THE 'Q = 20-22' WINDOWS, TRACED. damping_vs_gain.csv's "
          f"|gain|=0.0 rows are")
    print(f"  {len(sel)} DAMPING windows with that channel's own gain at 0 and "
          f"median gamma")
    print(f"  = {np.median(gam_sel):+.4f} 1/s -> Q(1.0 Hz) = "
          f"{math.pi / np.median(gam_sel):.1f}. Of those {len(sel)} windows:")
    for k, v in cnt.items():
        print(f"    {k:<20} {v}")
    allo = [r for r in R if r["st"] == "DAMPING" and abs(r["g"]) < 1e-12]
    cnt2 = Counter()
    for r in allo:
        oth = [v["g"] for c, v in sib[(r["run"], r["st"], r["t0"])].items()
               if c != r["ch"]]
        cnt2["others_driven" if any(abs(v) > 1e-9 for v in oth)
             else "all_channels_off"] += 1
    print(f"  and of ALL {len(allo)} gain-off DAMPING windows: {dict(cnt2)}")
    print("  All eight OSEMs are on ONE rigid body. Zeroing one channel's gain")
    print("  does not free the body while the other three damp it at 0.030.")
    # kp040's filters applied to windows that ARE genuinely open loop
    for s in ("CALIBRATING", "FAULT"):
        m = [r for r in R if r["st"] == s and r["r2"] > 0.80
             and r["rat"] < 0.90 and r["gs"] < 5e-4]
        n = sum(1 for r in R if r["st"] == s)
        print(f"  same filters on {s:<12} (all four channels off): "
              f"{len(m)} of {n} windows survive")


# ===========================================================================
# 7. Controls: what does this pipeline return when the answer is known?
# ===========================================================================
def null_transient(segs, amb, rng=np.random.default_rng(7)):
    """Run the transient selection and fit on records with NO kick in them.

    The opening calibration of every run is stationary ambient motion: nothing
    has ever driven the optic and no feedback has ever been applied. Slide the
    same window over it, keep the windows whose leading amplitude happens to
    exceed 3x the ambient RMS -- exactly the rule used to pick a ringdown --
    and fit. Whatever gamma comes out is selection bias and nothing else.
    """
    out = []
    for s in segs:
        if s["t0"] > 1.0 or "FAULT" in s["states"] or s["dur"] < 18.0:
            continue
        fs = s["fs"]
        nw = int(MIN_SEG_S * fs)
        for ch in range(s["y"].shape[0]):
            if not usable(s, ch) or not np.isfinite(amb[ch]) or amb[ch] <= 0:
                continue
            y = s["y"][ch]
            env = envelope(y, fs)
            for a in range(0, len(y) - nw, int(1.0 * fs)):
                if env[a:a + int(fs)].mean() < 3.0 * amb[ch]:
                    continue
                yy = y[a:a + nw]
                tt = np.arange(nw) / fs
                f = score(tt, yy, 1, seed(yy, 1 / fs, 1, ACF_FLO, ACF_FHI),
                          band=(ACF_FLO, ACF_FHI))
                m = modes_of(f)
                if m:
                    out.append([s["key"], ch, m[0][1], m[0][0], f["r2"]])
    return out


def _ar1_resonator(g, f0, fs, n, rng):
    """One complex pole driven by white noise: the stationary process whose
    ACF is exactly exp(-gamma tau) cos(2 pi f0 tau)."""
    z = np.exp((-g + 2j * np.pi * f0) / fs)
    w = rng.normal(size=n) + 1j * rng.normal(size=n)
    s = np.empty(n, complex)
    s[0] = 0
    for i in range(1, n):
        s[i] = z * s[i - 1] + w[i]
    return np.real(s)


def synth_recovery(lengths, fs, f0, gammas, nrep=8,
                   rng=np.random.default_rng(11)):
    """Known gamma in, recovered gamma out, at this data's record lengths."""
    rows = []
    ml = int(8.0 * fs)
    burn = int(60 * fs)
    for g in gammas:
        got_g, got_f = [], []
        for _ in range(nrep):
            acc, wsum = None, 0.0
            for L in lengths:
                y = _ar1_resonator(g, f0, fs, int(L * fs) + burn, rng)[burn:]
                if len(y) < 3 * ml:
                    continue
                r = acf_unbiased(y - y.mean(), ml)
                acc = r * len(y) if acc is None else acc + r * len(y)
                wsum += len(y)
            fit = fit_acf(np.arange(ml + 1) / fs, acc / wsum, 1)
            m = modes_of(fit)
            if m:
                got_g.append(m[0][1])
                got_f.append(m[0][0])
        gg = np.array(got_g)
        rows.append([g, float(gg.mean()), float(gg.std()), float(np.mean(got_f)),
                     math.pi * f0 / g if g > 0 else np.inf,
                     math.pi * f0 / gg.mean() if gg.mean() > 0 else np.nan,
                     len(gg)])
    return rows


# ===========================================================================
def boot_ci(v, w=None, n=BOOT, rng=None):
    v = np.asarray(v, float)
    if len(v) == 0:
        return np.nan, np.nan, np.nan
    rng = rng or np.random.default_rng(12345)
    idx = rng.integers(0, len(v), size=(n, len(v)))
    med = np.median(v[idx], axis=1)
    return float(np.median(v)), float(np.percentile(med, 16)), \
        float(np.percentile(med, 84))


def main():
    os.makedirs(OUT, exist_ok=True)
    if "--cache" in sys.argv or not os.path.exists(CACHE):
        hdr("0. CACHE: every contiguous gain-off stretch in data/*_fast_lock.csv")
        build_cache()
    segs = load_cache()

    hdr("1. GAIN-OFF INVENTORY")
    tot = sum(s["dur"] for s in segs)
    print(f"  segments (all gains identically 0): {len(segs)}")
    print(f"  total gain-off time:                {tot:.1f} s")
    longest = sorted(segs, key=lambda s: -s["dur"])[:8]
    print("  longest contiguous records:")
    for s in longest:
        print(f"    {s['run']:<36} seg{s['key'].split('#')[1]:>3} "
              f"{s['states']:<22} {s['dur']:7.2f}s  fs_raw={s['fs_raw']:7.1f}"
              f"  rail={s['rail']}")
    Tmax = longest[0]["dur"]
    print(f"\n  RESOLUTION CEILING. A record of length T resolves a Lorentzian")
    print(f"  FWHM no narrower than ~1/T. Longest gain-off record T = {Tmax:.1f} s")
    print(f"  -> narrowest resolvable FWHM = {1/Tmax:.4f} Hz")
    for q, f in ((20.0, 1.0), (50.0, 1.0), (288.0, 0.7149), (2334.0, 0.9941)):
        fw = f / q
        print(f"    Q={q:7.1f} at f={f:.4f} Hz -> FWHM {fw:.5f} Hz, "
              f"needs T > {1/fw:7.1f} s  "
              f"{'OK' if 1/fw < Tmax else 'IMPOSSIBLE with this data'}")

    amb = ambient_floor(segs)
    print("\n  ambient RMS floor (counts, 0.25-4 Hz, opening CALIBRATING only):")
    print("    " + "  ".join(f"ch{i}={a:6.2f}" for i, a in enumerate(amb)
                             if np.isfinite(a)))

    # -------------------------------------------------------------------
    hdr("1b. WHICH FREQUENCIES ARE ACTUALLY THERE? (peak census, gain-off only)")
    print("  Coherently-averaged periodogram over every gain-off segment >= 18 s,")
    print("  zero-padded x4, peaks taken above 20% of the channel maximum.\n")
    peak_rows = []
    for ch in range(8):
        acc, n, fr = None, 0, None
        for s in segs:
            if s["dur"] < 18.0 or not usable(s, ch):
                continue
            y = s["y"][ch]
            nn = int(18.0 * s["fs"])
            w = np.hanning(nn)
            P = np.abs(np.fft.rfft((y[:nn] - y[:nn].mean()) * w, 4 * nn)) ** 2
            acc = P if acc is None else acc + P
            fr = np.fft.rfftfreq(4 * nn, 1 / s["fs"])
            n += 1
        if acc is None or n < 3:
            continue
        P = acc / n
        m = (fr > 0.35) & (fr < 3.0)
        f, p = fr[m], P[m]
        pk = [i for i in range(1, len(p) - 1)
              if p[i] > p[i - 1] and p[i] > p[i + 1] and p[i] > 0.2 * p.max()]
        pk.sort(key=lambda i: -p[i])
        txt = "  ".join(f"{f[i]:.4f}Hz({p[i]/p.max():.2f})" for i in pk[:4])
        print(f"  ch{ch}  n={n:2d}  peaks: {txt}")
        for i in pk[:4]:
            peak_rows.append([ch, n, "all", float(f[i]), float(p[i] / p.max())])

    # Same census split by day, and by sample rate. A line locked to the DAQ
    # would sit at the same place regardless; a mechanical mode need not, and
    # the two epochs differ in sample rate by 3.1x (357.1 vs 1111.1 Hz).
    print("\n  by epoch (checks the line is mechanical, not an artefact of the"
          " DAQ):")
    for ch in (0, 1, 2, 3):
        for day in ("20260803", "20260804", "20260806"):
            acc, n, fr = None, 0, None
            for s in segs:
                if (s["dur"] < 18.0 or not usable(s, ch)
                        or not s["run"].startswith(day)):
                    continue
                y = s["y"][ch]
                nn = int(18.0 * s["fs"])
                w = np.hanning(nn)
                P = np.abs(np.fft.rfft((y[:nn] - y[:nn].mean()) * w,
                                       4 * nn)) ** 2
                acc = P if acc is None else acc + P
                fr = np.fft.rfftfreq(4 * nn, 1 / s["fs"])
                n += 1
            if acc is None or n < 2:
                continue
            P = acc / n
            out = []
            for a, b in ((0.60, 0.82), (0.88, 1.15), (1.50, 1.80)):
                m = (fr > a) & (fr < b)
                out.append(float(fr[m][np.argmax(P[m])]))
                peak_rows.append([ch, n, day, out[-1], float("nan")])
            fs_raw = {s["fs_raw"] for s in segs
                      if s["run"].startswith(day) and s["dur"] >= 18.0}
            print(f"    ch{ch} {day} n={n:2d} fs_raw={sorted(fs_raw)}  "
                  f"peaks: " + "  ".join(f"{v:.4f}" for v in out))
    writecsv("ringdown_peaks.csv", ["ch", "n_avg", "epoch", "f_hz",
                                    "rel_power"], peak_rows)

    # -------------------------------------------------------------------
    hdr("2. ESTIMATOR A -- damped-sinusoid fits to post-kick transients")
    rows, keep = transient_fits(segs, amb)
    writecsv("ringdown_transients.csv",
             ["key", "run", "states", "ch", "seg_dur_s", "fit_dur_s", "A0",
              "A0_over_ambient", "r2_K1", "r2_K2", "bic_K1", "bic_K2", "K_best",
              "f1_hz", "gamma1_1_s", "Q1", "amp1",
              "f2_hz", "gamma2_1_s", "Q2", "amp2"], rows)
    if rows:
        k2 = sum(1 for r in rows if r[12] == 2)
        print(f"  transient fits: {len(rows)}  "
              f"(BIC prefers K=2 in {k2}, K=1 in {len(rows)-k2})")
        print(f"  median r2: K=1 {np.median([r[8] for r in rows]):.3f}"
              f"   K=2 {np.median([r[9] for r in rows]):.3f}")

    # cluster the recovered components into modes by frequency
    if keep:
        arr = np.array([[k[2], k[3], k[4], k[5], k[1]] for k in keep], float)
        print("\n  recovered components, all channels pooled "
              f"(n={len(arr)}):")
        edges = [(0.25, 0.85), (0.85, 1.35), (1.35, 2.10), (2.10, 4.0)]
        band_rows = []
        for a, b in edges:
            m = (arr[:, 0] >= a) & (arr[:, 0] < b)
            if m.sum() < 3:
                continue
            f_m, f_lo, f_hi = boot_ci(arr[m, 0])
            g_m, g_lo, g_hi = boot_ci(arr[m, 1])
            q = math.pi * f_m / g_m if g_m > 0 else np.nan
            print(f"    {a:.2f}-{b:.2f} Hz  n={int(m.sum()):4d}  "
                  f"f = {f_m:.4f} [{f_lo:.4f},{f_hi:.4f}] Hz   "
                  f"gamma = {g_m:+.4f} [{g_lo:+.4f},{g_hi:+.4f}] 1/s   "
                  f"Q = {q:.1f}")
            band_rows.append([f"{a}-{b}", int(m.sum()), f_m, f_lo, f_hi,
                              g_m, g_lo, g_hi, q])
        writecsv("ringdown_transient_bands.csv",
                 ["band_hz", "n", "f_med", "f_lo", "f_hi", "gamma_med",
                  "gamma_lo", "gamma_hi", "Q"], band_rows)

        print("\n  per channel (dominant component of each fit):")
        pc = []
        for ch in range(8):
            m = arr[:, 4] == ch
            if m.sum() < 3:
                continue
            f_m, f_lo, f_hi = boot_ci(arr[m, 0])
            g_m, g_lo, g_hi = boot_ci(arr[m, 1])
            print(f"    ch{ch}  n={int(m.sum()):4d}  f={f_m:.4f} "
                  f"[{f_lo:.4f},{f_hi:.4f}]  gamma={g_m:+.4f} "
                  f"[{g_lo:+.4f},{g_hi:+.4f}]  Q={math.pi*f_m/g_m:7.1f}")
            pc.append([ch, int(m.sum()), f_m, f_lo, f_hi, g_m, g_lo, g_hi,
                       math.pi * f_m / g_m])
        writecsv("ringdown_transient_per_channel.csv",
                 ["ch", "n", "f_med", "f_lo", "f_hi", "gamma_med", "gamma_lo",
                  "gamma_hi", "Q"], pc)

    # -------------------------------------------------------------------
    hdr("3. ESTIMATOR B -- damped-sinusoid fits to the AUTOCORRELATION")
    print("  R(tau) of a broadband-driven resonance decays as exp(-gamma tau).")
    print("  Uses every gain-off sample, no large-amplitude selection.\n")
    acf_rows = []
    for ch in range(8):
        lag, r, used, tot_s = acf_stack(segs, ch, maxlag_s=8.0, min_dur=18.0)
        if lag is None or len(used) < 3:
            continue
        fits = [fit_acf(lag, r, K) for K in (1, 2, 3)]
        best = min(fits, key=lambda f: f["bic"])
        m = modes_of(best)
        txt = "  ".join(f"f={f:.4f} g={g:+.4f} Q={q:7.1f}"
                        for (f, g, q, a) in m)
        print(f"  ch{ch}  {len(used):3d} segs, {tot_s:7.1f}s  K*={best['K']}  "
              + " ".join(f"r2(K{K})={fits[K-1]['r2']:.3f}" for K in (1, 2, 3))
              + f"\n        {txt}")
        for j, (f, g, q, a) in enumerate(m):
            acf_rows.append([ch, len(used), tot_s, best["K"]]
                            + [fits[K - 1]["r2"] for K in (1, 2, 3)]
                            + [fits[K - 1]["bic"] for K in (1, 2, 3)]
                            + [j, f, g, q, a])
    writecsv("ringdown_acf.csv",
             ["ch", "n_seg", "total_s", "K_best", "r2_K1", "r2_K2", "r2_K3",
              "bic_K1", "bic_K2", "bic_K3", "mode", "f_hz", "gamma_1_s", "Q",
              "amp"], acf_rows)

    # ---- mode summary: same mode, seen through different channels
    print("\n  THE SAME MODE THROUGH DIFFERENT CHANNELS. gamma is a property of")
    print("  the mode, not of the sensor, so these columns must agree.")
    ms_rows = []
    for a, b, name in ((0.60, 0.82, "mode A"), (0.88, 1.15, "mode B"),
                       (1.50, 1.80, "mode C")):
        got = [(int(r[0]), r[-4], r[-3]) for r in acf_rows
               if a < r[-4] < b and r[3] >= 2 and r[6] > 0.95]
        if len(got) < 2:
            continue
        f = np.array([g[1] for g in got])
        gm = np.array([g[2] for g in got])
        print(f"    {name}  f = {f.mean():.4f} +/- {f.std(ddof=1):.4f} Hz   "
              f"(n={len(f)} channels: "
              + ", ".join(f"ch{c}" for c, _, _ in got) + ")")
        print(f"            gamma = " + ", ".join(f"{v:+.4f}" for v in gm)
              + f"   -> spread {gm.min():+.4f} .. {gm.max():+.4f} 1/s"
              + f"   ratio {gm.max()/max(gm.min(),1e-9):.0f}x")
        ms_rows.append([name, len(f), float(f.mean()), float(f.std(ddof=1)),
                        float(gm.min()), float(gm.max()), float(np.median(gm))])
    writecsv("ringdown_mode_summary.csv",
             ["mode", "n_ch", "f_mean_hz", "f_sd_hz", "gamma_min", "gamma_max",
              "gamma_median"], ms_rows)

    # ---- the direct bound: how much has the ACF envelope fallen at long lag?
    print("\n  LONG-LAG BOUND, no fitting at all. Stack the ACF of every")
    print("  gain-off record >= 24 s out to 12 s of lag. exp(-gamma*12) is the")
    print("  envelope ratio, so R(12 s)/R(0) bounds gamma directly.")
    def _ratio(sub, ch):
        lag, r, used, tot = acf_stack(sub, ch, maxlag_s=12.0, min_dur=24.0)
        if lag is None or len(used) < 2:
            return None, None, None
        fs = 1.0 / (lag[1] - lag[0])
        env = envelope(r / r[0], fs)
        e0 = float(env[:int(fs)].max())
        e12 = float(env[-int(2 * fs):].max())
        return e12 / max(e0, 1e-12), used, tot

    bound_rows = []
    for ch in range(8):
        ratio, used, tot_s = _ratio(segs, ch)
        if ratio is None:
            continue
        jk = []
        for drop in used:
            rr, _, _ = _ratio([s for s in segs if s["key"] != drop], ch)
            if rr is not None:
                jk.append(rr)
        n = len(jk)
        se = (math.sqrt((n - 1) / n * sum((v - np.mean(jk)) ** 2 for v in jk))
              if n > 3 else float("nan"))
        g = -math.log(max(ratio, 1e-6)) / 12.0
        # 1-sigma-low ratio -> 1-sigma-HIGH gamma: the upper limit
        g_hi = -math.log(max(min(ratio + se, 0.999999), 1e-6)) / 12.0 \
            if np.isfinite(se) else float("nan")
        g_lo_ratio = max(ratio - se, 1e-6) if np.isfinite(se) else np.nan
        g_up = (-math.log(min(g_lo_ratio, 0.999999)) / 12.0
                if np.isfinite(se) else float("nan"))
        print(f"    ch{ch}  {len(used):2d} records, {tot_s:6.1f}s  "
              f"R(12s)/R(0) = {ratio:5.3f} +/- {se:5.3f}  -> gamma = {g:+.4f} "
              f"1/s   1-sigma upper limit gamma < {g_up:.4f} 1/s"
              f"   -> Q(1.0 Hz) > {math.pi / g_up if g_up > 0 else float('inf'):6.0f}")
        bound_rows.append([ch, len(used), tot_s, ratio, se, g, g_up,
                           math.pi / g_up if g_up > 0 else np.inf])
    writecsv("ringdown_longlag.csv",
             ["ch", "n_records", "total_s", "acf_ratio_12s", "acf_ratio_se",
              "gamma_1_s", "gamma_upper_1sig", "Q_lower_at_1hz"], bound_rows)

    # ---- the same ratio turns every claimed Q into a prediction to test
    F_B = 0.9967
    print(f"\n  EVERY CLAIMED Q, AS A PREDICTION OF R(12s)/R(0) at "
          f"f = {F_B:.4f} Hz.")
    print(f"  R(12)/R(0) = exp(-pi*f*12/Q). Sigma is against ch0..ch3, the four")
    print(f"  channels with a real signal (ambient RMS 27-79 counts).")
    real = [r for r in bound_rows if r[0] < 4]
    excl_rows = []
    print(f"    {'Q':>7} {'gamma':>8} {'predicted':>10} | "
          + " ".join(f"ch{int(r[0])} sigma" for r in real))
    # A true ACF envelope ratio cannot exceed 1. The measured ones do, by up to
    # `bias`; that excess is this statistic's own positive bias. Subtract ALL of
    # it before quoting an exclusion, so the exclusion is the conservative one.
    bias = max(0.0, max(r[3] for r in real) - 1.0)
    print(f"  Largest measured ratio is {max(r[3] for r in real):.3f}; a true "
          f"envelope ratio cannot")
    print(f"  exceed 1, so {bias:.3f} of that is the statistic's own bias. The "
          f"'corr' columns")
    print(f"  subtract all of it from every channel, which is the conservative "
          f"choice.\n")
    for q in (20.0, 22.0, 50.0, 100.0, 200.0, 288.0, 500.0, 2334.0):
        g = math.pi * F_B / q
        pred = math.exp(-g * 12.0)
        sig = [(r[3] - pred) / r[4] if r[4] > 0 else float("nan") for r in real]
        cor = [(r[3] - bias - pred) / r[4] if r[4] > 0 else float("nan")
               for r in real]
        print(f"    {q:7.0f} {g:8.4f} {pred:10.3f} | "
              + " ".join(f"{v:8.1f}" for v in sig)
              + "  | corr " + " ".join(f"{v:6.1f}" for v in cor))
        excl_rows.append([q, g, pred] + sig + cor)
    writecsv("ringdown_exclusion.csv",
             ["Q_claimed", "gamma_implied", "predicted_acf_ratio_12s"]
             + [f"sigma_ch{int(r[0])}" for r in real]
             + [f"sigma_corr_ch{int(r[0])}" for r in real], excl_rows)

    # jackknife over segments, for a real uncertainty on the ACF numbers
    print("\n  leave-one-segment-out jackknife (ch with most data):")
    jk_rows = []
    for ch in range(8):
        lag, r, used, tot_s = acf_stack(segs, ch, 8.0, 18.0)
        if lag is None or len(used) < 5:
            continue
        vals = []
        for drop in used:
            sub = [s for s in segs if s["key"] != drop]
            lg, rr, u2, _ = acf_stack(sub, ch, 8.0, 18.0)
            if lg is None:
                continue
            fb = fit_acf(lg, rr, 2)
            mm = modes_of(fb)
            if mm:
                vals.append([mm[0][0], mm[0][1]])
        if len(vals) < 4:
            continue
        v = np.array(vals)
        n = len(v)
        mean = v.mean(axis=0)
        se = np.sqrt((n - 1) / n * ((v - mean) ** 2).sum(axis=0))
        q = math.pi * mean[0] / mean[1]
        dq = q * math.hypot(se[0] / mean[0], se[1] / max(abs(mean[1]), 1e-12))
        print(f"    ch{ch}  f = {mean[0]:.4f} +/- {se[0]:.4f} Hz   "
              f"gamma = {mean[1]:+.4f} +/- {se[1]:.4f} 1/s   "
              f"Q = {q:.1f} +/- {dq:.1f}   (n={n})")
        jk_rows.append([ch, n, mean[0], se[0], mean[1], se[1], q, dq])
    writecsv("ringdown_acf_jackknife.csv",
             ["ch", "n_jk", "f_hz", "f_se", "gamma_1_s", "gamma_se", "Q",
              "Q_se"], jk_rows)

    # -------------------------------------------------------------------
    hdr("4. ESTIMATOR C -- Lorentzian PSD widths on the longest gain-off record")
    long_segs = [s for s in segs if s["dur"] >= 18.0]
    print(f"  {len(long_segs)} gain-off segments >= 18 s")
    lor_rows = []
    for ch in range(4):
        acc, n = None, 0
        fr = None
        for s in long_segs:
            if not usable(s, ch):
                continue
            nseg = int(min(len(s["y"][ch]), 16 * s["fs"]))
            f_, P = welch(s["y"][ch], s["fs"], nseg)
            if P is None:
                continue
            if acc is None or len(P) == len(acc):
                acc = P if acc is None else acc + P
                fr = f_
                n += 1
        if acc is None or n == 0:
            continue
        P = acc / n
        # fit at the three lines the census actually found, not at a guess
        for guess in (0.712, 0.994, 1.640):
            f0, g, r2 = lorentz_fit(fr, P, guess, half=0.30)
            if not np.isfinite(f0):
                continue
            print(f"  ch{ch}  n_avg={n:2d}  df={fr[1]:.4f} Hz  "
                  f"f0={f0:.4f} Hz  gamma={g:.4f} 1/s  FWHM={g/np.pi:.4f} Hz  "
                  f"Q={math.pi*f0/g:7.1f}  r2={r2:.3f}"
                  f"{'   <-- FWHM below resolution, upper bound on gamma only'
                     if g/np.pi < 2*fr[1] else ''}")
            lor_rows.append([ch, n, fr[1], f0, g, g / np.pi,
                             math.pi * f0 / g, r2])
    writecsv("ringdown_lorentzian.csv",
             ["ch", "n_avg", "df_hz", "f0_hz", "gamma_1_s", "fwhm_hz", "Q",
              "r2"], lor_rows)

    # -------------------------------------------------------------------
    hdr("5. AMPLITUDE DEPENDENCE -- is the damping linear?")
    print("  Split every transient fit by its starting amplitude and refit.")
    if keep:
        arr = np.array([[k[2], k[3], k[6]] for k in keep], float)  # f, g, A0/amb
        for lo, hi in ((3, 6), (6, 12), (12, 30), (30, 1e9)):
            m = (arr[:, 2] >= lo) & (arr[:, 2] < hi)
            if m.sum() < 4:
                continue
            g_m, g_lo, g_hi = boot_ci(arr[m, 1])
            f_m, _, _ = boot_ci(arr[m, 0])
            print(f"    A0/ambient {lo:>4}-{hi if hi<1e8 else 'inf':>4}  "
                  f"n={int(m.sum()):4d}  gamma = {g_m:+.4f} "
                  f"[{g_lo:+.4f},{g_hi:+.4f}] 1/s   f={f_m:.3f} Hz")

    print("\n  Early vs late half of each usable transient (same record, so the")
    print("  comparison is free of any between-segment systematic):")
    eh, lh = [], []
    for s in segs:
        if s["dur"] < 12.0:
            continue
        fs = s["fs"]
        for ch in range(s["y"].shape[0]):
            if not usable(s, ch) or not np.isfinite(amb[ch]):
                continue
            y = s["y"][ch]
            env = envelope(y, fs)
            if env[:int(fs)].mean() < 3 * amb[ch]:
                continue
            h = len(y) // 2
            t = s["t"] - s["t"][0]
            for half, box in ((slice(0, h), eh), (slice(h, 2 * h), lh)):
                yy, tt = y[half], t[half] - t[half][0]
                f = score(tt, yy, 1, seed(yy, 1 / fs, 1))
                mm = modes_of(f)
                if mm and F_LO < mm[0][0] < F_HI and f["r2"] > 0.3:
                    box.append([mm[0][1], mm[0][0], float(np.std(yy))])
    if len(eh) >= 4 and len(lh) >= 4:
        e, l = np.array(eh), np.array(lh)
        print(f"    early half  n={len(e):3d}  rms={np.median(e[:,2]):7.2f} "
              f"counts  gamma = {np.median(e[:,0]):+.4f} 1/s")
        print(f"    late  half  n={len(l):3d}  rms={np.median(l[:,2]):7.2f} "
              f"counts  gamma = {np.median(l[:,0]):+.4f} 1/s")
        writecsv("ringdown_amplitude.csv",
                 ["half", "gamma_1_s", "f_hz", "rms_counts"],
                 [["early"] + list(r) for r in e] + [["late"] + list(r) for r in l])

    # -------------------------------------------------------------------
    hdr("6. WHAT analysis/out/decay_windows.csv ACTUALLY MEASURED")
    reread_envelope_fits()

    # -------------------------------------------------------------------
    hdr("7a. NULL CONTROL -- the same fit on records with no kick in them")
    nul = null_transient(segs, amb)
    if nul:
        gv = np.array([r[2] for r in nul])
        g_m, g_lo, g_hi = boot_ci(gv)
        print(f"  windows selected by the identical >3x-ambient rule from the")
        print(f"  opening calibration of each run (stationary ambient, no kick,")
        print(f"  no feedback ever applied):  n = {len(gv)}")
        print(f"    gamma = {g_m:+.4f} [{g_lo:+.4f},{g_hi:+.4f}] 1/s   "
              f"(median, 16-84% bootstrap)")
        print(f"    frac gamma > 0: {100*(gv>0).mean():.0f}%   "
              f"IQR {np.percentile(gv,25):+.4f} .. {np.percentile(gv,75):+.4f}")
        print("  Any positive gamma here is selection bias: the process is")
        print("  stationary by construction. Subtract it from Estimator A.")
        writecsv("ringdown_null.csv", ["key", "ch", "gamma_1_s", "f_hz", "r2"],
                 nul)

    # -------------------------------------------------------------------
    hdr("7b. RECOVERY CONTROL -- known gamma in, recovered gamma out")
    lengths = sorted((s["dur"] for s in segs if s["dur"] >= 18.0), reverse=True)
    print(f"  Simulated at f0 = 0.9990 Hz, driven by white noise, cut into the")
    print(f"  {len(lengths)} record lengths actually on disk "
          f"({lengths[0]:.1f} s down to {lengths[-1]:.1f} s), then pushed")
    print(f"  through the same ACF estimator (max lag 8 s).\n")
    print(f"  {'gamma_true':>10} {'Q_true':>8} | {'gamma_fit':>18} {'Q_fit':>8}"
          f" {'f_fit':>8}  {'bias':>7}")
    rec = synth_recovery(lengths, FS_D, 0.9990,
                         [0.0000, 0.0050, 0.0100, 0.0200, 0.0400, 0.0630,
                          0.1000, 0.1570, 0.2500])
    for g_t, g_f, g_sd, f_f, q_t, q_f, n in rec:
        qt = f"{q_t:8.1f}" if np.isfinite(q_t) else f"{'inf':>8}"
        err = (f"{(g_f - g_t) / g_t * 100:+6.0f}%" if g_t > 0
               else f"{'--':>7}")
        print(f"  {g_t:10.4f} {qt} | {g_f:9.4f} +/- {g_sd:6.4f} "
              f"{q_f:8.1f} {f_f:8.4f}  {err}")
    writecsv("ringdown_recovery.csv",
             ["gamma_true", "gamma_fit_mean", "gamma_fit_sd", "f_fit", "Q_true",
              "Q_fit", "n_rep"], rec)
    floor = [r for r in rec if r[0] == 0.0]
    if floor:
        gf, gs = floor[0][1], floor[0][2]
        print(f"\n  ESTIMATOR FLOOR. With gamma_true = 0 exactly (an undamped")
        print(f"  resonance) the estimator still returns "
              f"gamma = {gf:+.4f} +/- {gs:.4f} 1/s.")
        print(f"  Nothing below about {gf + 2*gs:.4f} 1/s is distinguishable "
              f"from zero here,")
        print(f"  i.e. Q above ~{math.pi * 0.999 / (gf + 2 * gs):.0f} at "
              f"1.0 Hz cannot be measured from this data at all.")
    ok = [r[0] for r in rec if r[0] > 0 and abs(r[1] - r[0]) < 0.3 * r[0]]
    if ok:
        print(f"  smallest gamma recovered to within 30%: {min(ok):.4f} 1/s"
              f"  (Q = {math.pi * 0.999 / min(ok):.0f})")


if __name__ == "__main__":
    main()
