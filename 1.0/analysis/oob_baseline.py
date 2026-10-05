"""Does `research.md` item 6b option (c) -- the always-on out-of-band baseline --
actually work?  Measured on the ~30 recorded runs in `data/`.

The claim under test: the loop has authority only inside BP_LOW_HZ..BP_HIGH_HZ
= 0.4-3.0 Hz, so a level measured OUTSIDE that band is open-loop at any gain.
Track it continuously, scale it by a once-characterised factor, and you get the
undamped in-band floor for free -- replacing the 20 s zero-gain calibration that
every controller here still pays for.

Four questions, one section each:

  1  is there a usable out-of-band band?          sec2_spectra
  2  does the out-of-band level survive the gain? sec3_invariance   <- load-bearing
  3  what is the scale factor, how stable is it?  sec4_scaling
  4  how fast does it converge?                   sec5_convergence

Reference quantity throughout is `baseline_rms` as the controllers compute it:
the RMS of the 0.4-3.0 Hz bandpassed sensor signal over a zero-gain window
(osem.v3.py Channel.finish_calibration).  Here it is measured with a brick-wall
FFT band instead of the controller's one-pole cascade; sec0 checks the two agree
to a fixed 1.16x scale factor, which any per-channel alpha absorbs.

Open loop means state CALIBRATING or FAULT -- in both the actuator output is
pinned at BIAS (verified: output std 0.000 V in CALIBRATING, <=0.002 V in FAULT).
Closed loop means DAMPING with the scheduled gain actually engaged.

Everything prints; no files are written.  Run from the repo root:

    .venv/bin/python analysis/oob_baseline.py
"""
import glob
import sys

import numpy as np

# ---------------------------------------------------------------- config

IN_BAND = (0.4, 3.0)          # the controller's own bandpass, BP_LOW_HZ..BP_HIGH_HZ

# Candidate out-of-band windows.  Below 0.4 Hz: drift and TIA offset wander.
# Above 3 Hz: the 6.19 Hz line on a4/a6/a7, 50 Hz mains, and whatever the loop
# injects.  6.5-8.0 deliberately straddles the gap between the 6.19 Hz line and
# the mains.
CAND_BANDS = [(0.05, 0.2), (0.1, 0.3), (0.2, 0.4),
              (3.0, 5.0), (4.0, 8.0), (6.5, 8.0),
              (8.0, 20.0), (20.0, 60.0), (60.0, 150.0)]

CALIBRATION_S = 20.0          # what the controllers spend today
WINDOW_S = 18.0               # analysis window.  The logged CALIBRATING segments
                              # run 19.3-19.9 s, so 20.0 would discard almost
                              # every run; 18 s keeps 22 of them.
GAIN_ENGAGED = 0.005          # |gain| above this counts as closed loop
ADC_V_PER_COUNT = 5.023 / 1023.0

STATES = {'CALIBRATING': 0, 'DAMPING': 1, 'FAULT': 2, 'RECOVER': 3, 'LOCKED': 4}
OPEN_LOOP = (0, 2)            # CALIBRATING, FAULT -- output pinned at BIAS

_CACHE = {}

# ---------------------------------------------------------------- loading


def run_paths():
    out = []
    for p in sorted(glob.glob("data/*.csv")):
        head = open(p).readline().strip().split(',')
        if 'state' in head and 'ch0_counts' in head:
            out.append(p)
    return out


def load(path, want_bp=False):
    key = (path, want_bp)
    if key in _CACHE:
        return _CACHE[key]
    head = open(path).readline().strip().split(',')
    nch = sum(1 for c in head if c.endswith('_counts'))
    si = head.index('state')
    names = ['time_s'] + [f'ch{i}_counts' for i in range(nch)] + \
            [f'ch{i}_gain' for i in range(nch)]
    if want_bp:
        names += [f'ch{i}_bp' for i in range(nch)]
    cols = [head.index(n) for n in names] + [si]
    conv = {si: lambda s: STATES.get(s.decode() if isinstance(s, bytes) else s, 9)}
    a = np.loadtxt(path, delimiter=',', skiprows=1, usecols=cols, converters=conv)
    d = dict(path=path, name=path.split('/')[-1][:15], day=path.split('/')[-1][:8],
             t=a[:, 0], counts=a[:, 1:1 + nch], gain=a[:, 1 + nch:1 + 2 * nch],
             state=a[:, -1].astype(int), nch=nch)
    if want_bp:
        d['bp'] = a[:, 1 + 2 * nch:1 + 3 * nch]
    d['fs'] = 1.0 / float(np.median(np.diff(d['t'])))
    _CACHE[key] = d
    return d


def segments(state, codes, minlen=1):
    """Contiguous index ranges where state is in `codes`."""
    if np.isscalar(codes):
        codes = (codes,)
    m = np.isin(state, codes).astype(np.int8)
    edge = np.flatnonzero(np.diff(np.concatenate([[0], m, [0]])))
    return [(a, b) for a, b in zip(edge[0::2], edge[1::2]) if b - a >= minlen]


# ---------------------------------------------------------------- signal


def bandlimit(x, fs, lo, hi):
    """Brick-wall band, zero phase.  Mean removed first."""
    n = len(x)
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(n, 1.0 / fs)
    X[(f < lo) | (f > hi)] = 0.0
    return np.fft.irfft(X, n)


def band_rms(x, fs, lo, hi):
    if hi >= fs / 2:
        return np.nan
    return float(np.sqrt((bandlimit(x, fs, lo, hi) ** 2).mean()))


def window_rms(y, w, step):
    c = np.cumsum(np.concatenate([[0.0], y ** 2]))
    idx = np.arange(0, len(y) - w + 1, step)
    return np.sqrt((c[idx + w] - c[idx]) / w)


def welch(x, fs, nperseg):
    nperseg = int(min(nperseg, len(x)))
    if nperseg < 64:
        return None, None
    step, win = nperseg // 2, np.hanning(nperseg)
    P, n = 0.0, 0
    for s in range(0, len(x) - nperseg + 1, step):
        seg = x[s:s + nperseg]
        P = P + np.abs(np.fft.rfft((seg - seg.mean()) * win)) ** 2
        n += 1
    f = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return f, P / n * (2.0 / (fs * (win ** 2).sum()))


def disp(v):
    """Robust 1-sigma spread of a positive quantity, as a fraction.
    68th percentile of |log v - median log v|, expressed as (exp(.) - 1)."""
    v = np.asarray([x for x in v if np.isfinite(x) and x > 0])
    if len(v) < 3:
        return np.nan
    l = np.log(v)
    return float(np.exp(np.percentile(np.abs(l - np.median(l)), 68)) - 1.0)


def pct(x):
    return "     -" if not np.isfinite(x) else f"{x * 100:5.0f}%"


def rule(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


# ---------------------------------------------------------------- sec 0


def sec0_sanity():
    rule("0. the reference quantity: brick-wall 0.4-3.0 Hz vs the controller's bp")
    print("RMS over one zero-gain window.  A fixed ratio is all that is needed --")
    print("a per-channel alpha absorbs it.\n")
    print(f"  {'run':16s} {'ch':>3s} {'rms(bp col) V':>14s} {'fft 0.4-3 V':>12s} {'ratio':>7s}")
    for path in ["data/20260804_112355_fast_lock.csv",
                 "data/20260806_171500_fast_lock.csv"]:
        d = load(path, want_bp=True)
        fs = d['fs']
        a, b = max(segments(d['state'], 0, minlen=100), key=lambda s: s[1] - s[0])
        a += int(2 * fs)                       # skip the filter's first samples
        for ch in range(d['nch']):
            r_bp = float(np.sqrt((d['bp'][a:b, ch] ** 2).mean()))
            r_ft = band_rms(d['counts'][a:b, ch], fs, *IN_BAND) * ADC_V_PER_COUNT
            print(f"  {d['name']:16s} {ch:>3d} {r_bp:>14.4f} {r_ft:>12.4f} "
                  f"{r_ft / r_bp:>7.3f}")


# ---------------------------------------------------------------- sec 1


def sec1_inventory():
    rule("1. inventory")
    print(f"  {'run':16s} {'nch':>4s} {'fs':>6s} {'dur':>7s} "
          f"{'open-loop s':>12s} {'closed-loop s':>14s}")
    for p in run_paths():
        d = load(p)
        fs = d['fs']
        if len(d['t']) < 3000:
            continue
        ol = np.isin(d['state'], OPEN_LOOP).sum() / fs
        cl = (d['state'] == 1).sum() / fs
        print(f"  {d['name']:16s} {d['nch']:>4d} {fs:>6.0f} {d['t'][-1]:>7.1f} "
              f"{ol:>12.1f} {cl:>14.1f}")


# ---------------------------------------------------------------- sec 2


def sec2_spectra():
    rule("2. Q1 -- where could an out-of-band window live?")
    print("Open-loop amplitude spectral density, counts/sqrt(Hz), median over all")
    print("zero-gain windows of every run at that sample rate.  A band is only")
    print("usable if it is (a) loop-free and (b) carries motion.  This shows (b).\n")

    edges = [0.05, 0.2, 0.4, 1.0, 3.0, 5.0, 8.0, 20.0, 45.0, 55.0, 150.0]
    for fs_group in (357, 1111):
        rows = {}
        for p in run_paths():
            d = load(p)
            if round(d['fs']) != fs_group or len(d['t']) < 3000:
                continue
            fs = d['fs']
            for (a, b) in segments(d['state'], 0, minlen=int(10 * fs)):
                nps = int(2 ** np.floor(np.log2(min(fs * 16, b - a))))
                for ch in range(d['nch']):
                    f, P = welch(d['counts'][a:b, ch], fs, nps)
                    if P is None:
                        continue
                    v = []
                    for lo, hi in zip(edges[:-1], edges[1:]):
                        m = (f >= lo) & (f < hi)
                        v.append(np.sqrt(P[m].mean()) if m.any() and hi < fs / 2 else np.nan)
                    rows.setdefault(ch, []).append(v)
        if not rows:
            continue
        print(f"  fs = {fs_group} Hz  ({len(next(iter(rows.values())))} windows)")
        print("   ch " + "".join(f"{lo:>6.3g}-{hi:<6.3g}" for lo, hi in zip(edges[:-1], edges[1:])))
        for ch in sorted(rows):
            med = np.nanmedian(np.array(rows[ch]), axis=0)
            print(f"   a{ch:<2}" + "".join(f"{x:>13.3f}" if np.isfinite(x) else f"{'-':>13s}"
                                           for x in med))
        print()

    print("  strongest lines, 15-150 Hz, open loop (frequency Hz, ASD counts/rtHz):")
    d = load("data/20260806_171500_fast_lock.csv")
    fs = d['fs']
    a, b = max(segments(d['state'], 0, minlen=int(10 * fs)), key=lambda s: s[1] - s[0])
    for ch in range(d['nch']):
        f, P = welch(d['counts'][a:b, ch], fs, 8192)
        m = (f > 15) & (f < 150)
        k = np.argsort(P[m])[-3:][::-1]
        line = "  ".join(f"{f[m][i]:6.2f} Hz {np.sqrt(P[m][i]):8.2f}" for i in k)
        print(f"   a{ch}: {line}")


# ---------------------------------------------------------------- sec 3


def sec3_invariance():
    rule("3. Q2 (load-bearing) -- does the out-of-band level survive the gain?")
    print("Paired windows either side of an open-loop -> DAMPING transition in the")
    print("SAME run, minutes apart at most.  T s of open loop immediately before,")
    print("T s of closed loop starting SETTLE s after, gain fully slewed in.")
    print("If the hypothesis holds the ratio is 1.00 for an out-of-band window")
    print("while the in-band ratio collapses.\n")
    T = SETTLE = 4.0
    rows = []
    for p in run_paths():
        d = load(p)
        fs, st = d['fs'], d['state']
        if len(d['t']) < 3000:
            continue
        w, s = int(T * fs), int(SETTLE * fs)
        filt = {}
        for ch in range(d['nch']):
            for key in [IN_BAND] + CAND_BANDS:
                filt[(ch, key)] = (bandlimit(d['counts'][:, ch], fs, *key)
                                   if key[1] < fs / 2 else None)
        for (a, _) in segments(st, 1, minlen=w + s):
            p0, p1, q0, q1 = a - w, a, a + s, a + s + w
            if p0 < 0 or q1 > len(st):
                continue
            if not np.all(np.isin(st[p0:p1], OPEN_LOOP)) or not np.all(st[q0:q1] == 1):
                continue
            for ch in range(d['nch']):
                rec = dict(run=d['name'], day=d['day'], ch=ch,
                           gain=float(np.abs(d['gain'][q0:q1, ch]).mean()))
                for key in [IN_BAND] + CAND_BANDS:
                    y = filt[(ch, key)]
                    if y is None:
                        rec[key] = np.nan
                        continue
                    pre = np.sqrt((y[p0:p1] ** 2).mean())
                    post = np.sqrt((y[q0:q1] ** 2).mean())
                    rec[key] = post / pre if pre > 0 else np.nan
                rows.append(rec)

    def table(sel, title):
        rs = [r for r in rows if sel(r)]
        if len(rs) < 3:
            return
        print(f"  --- {title}   ({len(rs)} pairs, {len(set(r['run'] for r in rs))} runs)")
        print(f"      {'band':>12s} {'p16':>7s}{'median':>8s}{'p84':>8s}   ratio closed/open")
        for key in [IN_BAND] + CAND_BANDS:
            v = np.array([r[key] for r in rs if np.isfinite(r[key]) and r[key] > 0])
            if len(v) < 3:
                continue
            nm = "IN-BAND" if key == IN_BAND else f"{key[0]:g}-{key[1]:g}"
            print(f"      {nm:>12s} {np.percentile(v, 16):7.2f}{np.median(v):8.2f}"
                  f"{np.percentile(v, 84):8.2f}")
        print()

    for day, note in [('20260803', 'loop rate 18 Hz'), ('20260804', 'loop rate 18 Hz'),
                      ('20260806', 'loop rate 900 Hz')]:
        table(lambda r, d=day: r['ch'] < 4 and r['gain'] > GAIN_ENGAGED and r['day'] == d,
              f"a0-a3 driven, {day} ({note})")
    table(lambda r: r['ch'] < 4 and r['gain'] <= GAIN_ENGAGED and r['day'] == '20260804',
          "control: a0-a3 NOT driven while another channel is, 20260804")


# ---------------------------------------------------------------- sec 4


def sec4_scaling():
    rule("4. Q3 -- the scale factor and its scatter")
    print("One point per (run, channel): median over that run's zero-gain windows")
    print(f"of {WINDOW_S:.0f} s.  alpha = in-band RMS / out-of-band RMS.\n")

    W = WINDOW_S
    pts = {}
    for p in run_paths():
        d = load(p)
        fs = d['fs']
        if len(d['t']) < 3000:
            continue
        w = int(W * fs)
        acc = {}
        for (a, b) in segments(d['state'], 0, minlen=w):
            for ch in range(d['nch']):
                x = d['counts'][a:b, ch]
                v = [window_rms(bandlimit(x, fs, *IN_BAND), w, w // 2)]
                for lo, hi in CAND_BANDS:
                    v.append(window_rms(bandlimit(x, fs, lo, hi), w, w // 2)
                             if hi < fs / 2 else np.full(len(v[0]), np.nan))
                acc.setdefault(ch, []).append(np.array(v))
        for ch, lst in acc.items():
            pts[(d['name'], ch)] = (np.median(np.concatenate(lst, axis=1), axis=1),
                                    d['day'], round(fs))

    runs = sorted(set(k[0] for k in pts))
    chs = sorted(set(k[1] for k in pts))
    print(f"  {len(runs)} runs contribute a {W:.0f} s zero-gain window\n")

    print("  alpha per channel per band: median, and robust 1-sigma scatter across runs")
    print(f"  {'band':>12s}" + "".join(f" | a{c}: alpha  scat" for c in chs if c < 4))
    for j, (lo, hi) in enumerate(CAND_BANDS):
        out = [f"  {lo:>5.3g}-{hi:<5.3g}"]
        for c in chs:
            if c >= 4:
                continue
            r = np.array([pts[(k, c)][0][0] / pts[(k, c)][0][1 + j] for k in runs
                          if (k, c) in pts and np.isfinite(pts[(k, c)][0][1 + j])
                          and pts[(k, c)][0][1 + j] > 0])
            out.append(f" | {np.median(r):8.2f} {pct(disp(r))}" if len(r) >= 3
                       else " |        -      ")
        print("".join(out))

    print("\n  same alpha, split by day and by sample rate (a0-a3 pooled after")
    print("  normalising each channel by its own all-run median):")
    print(f"  {'band':>12s} {'0803':>9s} {'0804':>9s} {'0806':>9s}  "
          f"{'357 Hz':>9s} {'1111 Hz':>9s}   {'all-day scatter':>15s}")
    for j, (lo, hi) in enumerate(CAND_BANDS):
        norm = {}
        for c in chs:
            if c >= 4:
                continue
            vals = {k: pts[(k, c)][0][0] / pts[(k, c)][0][1 + j] for k in runs
                    if (k, c) in pts and np.isfinite(pts[(k, c)][0][1 + j])
                    and pts[(k, c)][0][1 + j] > 0}
            if len(vals) < 3:
                continue
            m = np.median(list(vals.values()))
            for k, v in vals.items():
                norm[(k, c)] = (v / m, pts[(k, c)][1], pts[(k, c)][2])
        if not norm:
            continue
        g = lambda f: np.median([v[0] for v in norm.values() if f(v)]) if any(f(v) for v in norm.values()) else np.nan
        row = [g(lambda v: v[1] == '20260803'), g(lambda v: v[1] == '20260804'),
               g(lambda v: v[1] == '20260806'), g(lambda v: v[2] == 357),
               g(lambda v: v[2] == 1111)]
        print(f"  {lo:>5.3g}-{hi:<5.3g}" + "".join(f"{x:>9.2f}" if np.isfinite(x) else f"{'-':>9s}"
                                                   for x in row)
              + f"   {pct(disp([v[0] for v in norm.values()])):>15s}")

    print("\n  leave-one-run-out prediction of the in-band floor.  |error| of the")
    print("  predicted baseline, median and 90th percentile over runs.")
    print("    const  = per-channel constant, no out-of-band term at all (control)")
    print("    prop   = alpha * out-of-band level, the proposal as written")
    print("    power  = a*oob^b fitted in log space -- an optimistic upper bound,")
    print("             it is a different model from the one proposed")

    def loo(y, X, model):
        errs = []
        for i in range(len(y)):
            m = np.ones(len(y), bool)
            m[i] = False
            ly, lx = np.log(y[m]), np.log(X[m])
            if model == 'const':
                pred = np.exp(np.median(ly))
            elif model == 'prop':
                pred = np.exp(np.median(ly - lx)) * X[i]
            else:
                A = np.vstack([lx, np.ones(m.sum())]).T
                coef, *_ = np.linalg.lstsq(A, ly, rcond=None)
                pred = np.exp(coef[0] * np.log(X[i]) + coef[1])
            errs.append(pred / y[i] - 1.0)
        return np.abs(np.array(errs))

    for c in chs:
        ks = [k for k in runs if (k, c) in pts]
        y = np.array([pts[(k, c)][0][0] for k in ks])
        ok = y > 0
        if ok.sum() < 8:
            continue
        e = loo(y[ok], y[ok], 'const')
        tag = "" if c < 4 else "   <- control group, carries no motion signal"
        print(f"\n    a{c}  n={ok.sum()}  in-band median={np.median(y[ok]):8.2f} counts"
              f"   CONST p50={pct(np.median(e))} p90={pct(np.percentile(e, 90))}{tag}")
        for j, (lo, hi) in enumerate(CAND_BANDS):
            X = np.array([pts[(k, c)][0][1 + j] for k in ks])
            m = ok & np.isfinite(X) & (X > 0)
            if m.sum() < 8:
                continue
            ep, eq = loo(y[m], X[m], 'prop'), loo(y[m], X[m], 'power')
            A = np.vstack([np.log(X[m]), np.ones(m.sum())]).T
            coef, *_ = np.linalg.lstsq(A, np.log(y[m]), rcond=None)
            print(f"      {lo:>5.3g}-{hi:<5.3g} n={m.sum():3d}  "
                  f"prop p50={pct(np.median(ep))} p90={pct(np.percentile(ep, 90))}   "
                  f"power(b={coef[0]:+.2f}) p50={pct(np.median(eq))} "
                  f"p90={pct(np.percentile(eq, 90))}")

    print("\n  CONTROL GROUP.  a4/a6/a7 carry no motion signal (lock-in SNR 1.1-1.5,")
    print("  91-99% of their power in a 6.19 Hz line).  Pooled 10 s zero-gain windows")
    print("  from the five 8-channel runs: correlation of log(in-band) against")
    print("  log(out-of-band), and the scatter of the ratio.  A band that scores")
    print("  BETTER on these channels than on a0-a3 is measuring the instrument.")
    W2 = 10.0
    pool = {}
    for p2 in run_paths():
        d = load(p2)
        fs = d['fs']
        if d['nch'] < 8 or len(d['t']) < 3000:
            continue
        w = int(W2 * fs)
        for (a, b) in segments(d['state'], 0, minlen=w):
            for ch in range(8):
                x = d['counts'][a:b, ch]
                inb = window_rms(bandlimit(x, fs, *IN_BAND), w, w // 2)
                for j, (lo, hi) in enumerate(CAND_BANDS):
                    if hi >= fs / 2:
                        continue
                    ob = window_rms(bandlimit(x, fs, lo, hi), w, w // 2)
                    for i in range(len(inb)):
                        if inb[i] > 0 and ob[i] > 0:
                            pool.setdefault((ch, j), []).append((inb[i], ob[i]))
    print(f"  {'band':>12s}" + "".join(f" |a{c}:  r  scat" for c in range(8)))
    for j, (lo, hi) in enumerate(CAND_BANDS):
        out = [f"  {lo:>5.3g}-{hi:<5.3g}"]
        for c in range(8):
            v = np.array(pool.get((c, j), []))
            if len(v) < 8:
                out.append(" |     -      ")
                continue
            la, lb = np.log(v[:, 0]), np.log(v[:, 1])
            r = np.corrcoef(la, lb)[0, 1]
            out.append(f" |{r:+.2f} {pct(disp(np.exp(la - lb)))}")
        print("".join(out))
    print("  (a0-a3 healthy; a5 works at ~1/12 gain; a4/a6/a7 = control group)")

    return pts, runs, chs


# ---------------------------------------------------------------- sec 5


def sec5_convergence(pts, runs, chs):
    rule("5. Q4 -- how fast does it converge?")
    print(f"Reference: the in-band RMS over a full {WINDOW_S:.0f} s zero-gain window --")
    print("exactly what the controller calls baseline_rms today.  Two estimators")
    print("are given the FIRST T seconds of the same window:")
    print("    in-band(T)  just shorten the existing calibration")
    print("    alpha*oob(T)  the proposal, alpha fitted leave-one-run-out")
    print("Median |error| over (run, channel), a0-a3 only.\n")

    TS = [0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 18.0]
    W = WINDOW_S
    samples = []          # (run, ch, ref, {T: inb_T}, {(band,T): oob_T})
    for p in run_paths():
        d = load(p)
        fs = d['fs']
        if len(d['t']) < 3000:
            continue
        w = int(W * fs)
        for (a, b) in segments(d['state'], 0, minlen=w):
            for ch in range(min(d['nch'], 4)):
                x = d['counts'][a:a + w, ch]
                yb = bandlimit(x, fs, *IN_BAND)
                ref = float(np.sqrt((yb ** 2).mean()))
                if ref <= 0:
                    continue
                inb = {T: float(np.sqrt((yb[:int(T * fs)] ** 2).mean())) for T in TS}
                oob = {}
                for lo, hi in CAND_BANDS:
                    if hi >= fs / 2:
                        continue
                    yo = bandlimit(x, fs, lo, hi)
                    for T in TS:
                        oob[((lo, hi), T)] = float(np.sqrt((yo[:int(T * fs)] ** 2).mean()))
                samples.append((d['name'], ch, ref, inb, oob))
            break         # one window per run keeps runs equally weighted

    print(f"  {len(samples)} (run, channel) windows\n")
    print(f"  {'estimator':>16s}" + "".join(f"{T:>8.3g}s" for T in TS))
    err = {T: [] for T in TS}
    for (_, _, ref, inb, _) in samples:
        for T in TS:
            err[T].append(abs(inb[T] / ref - 1.0))
    print(f"  {'in-band(T)':>16s}" + "".join(f"{pct(np.median(err[T])):>9s}" for T in TS))

    for band in CAND_BANDS:
        ok = [s for s in samples if (band, TS[-1]) in s[4]]
        if len(ok) < 8:
            continue
        line = []
        for T in TS:
            e = []
            for i, (run, ch, ref, _, oob) in enumerate(ok):
                others = [(r2, o2[(band, TS[-1])]) for (rn, c2, r2, _, o2) in ok
                          if c2 == ch and rn != run and o2[(band, TS[-1])] > 0]
                if len(others) < 4 or oob[(band, T)] <= 0:
                    continue
                alpha = np.exp(np.median([np.log(r) - np.log(o) for r, o in others]))
                e.append(abs(alpha * oob[(band, T)] / ref - 1.0))
            line.append(pct(np.median(e)) if e else "     -")
        print(f"  {f'{band[0]:g}-{band[1]:g} Hz':>16s}" + "".join(f"{x:>9s}" for x in line))

    print("\n  For scale: how much does the in-band floor itself move between one")
    print(f"  {WINDOW_S:.0f} s open-loop window and the next, within the same run?")
    for c in range(4):
        ratios = []
        for p in run_paths():
            d = load(p)
            fs = d['fs']
            if len(d['t']) < 3000 or d['nch'] <= c:
                continue
            w = int(W * fs)
            for (a, b) in segments(d['state'], OPEN_LOOP, minlen=2 * w):
                y = bandlimit(d['counts'][a:b, c], fs, *IN_BAND)
                r = window_rms(y, w, w)
                ratios.extend(list(r[1:] / r[:-1]))
        if len(ratios) >= 3:
            print(f"    a{c}: n={len(ratios):3d}  consecutive-window ratio "
                  f"median={np.median(ratios):.2f}  scatter={pct(disp(ratios))}")


# ---------------------------------------------------------------- main


def main():
    if not run_paths():
        sys.exit("no data/*.csv found -- run from the repo root")
    sec0_sanity()
    sec1_inventory()
    sec2_spectra()
    sec3_invariance()
    pts, runs, chs = sec4_scaling()
    sec5_convergence(pts, runs, chs)
    print("\ndone.")


if __name__ == "__main__":
    main()
