"""Is the ambient 0.4-3 Hz disturbance phase-stable enough to cancel by
injecting an anti-phase drive (adaptive feedforward, no dither)?

The proposal's appeal is that it needs no excitation, so it carries none of the
railing risk that sank the stepped-sine and multisine attempts.  It lives or
dies on one number: how long the dominant narrowband component holds its phase.
If the phase wanders faster than a canceller can track it, half the time the
injection ADDS energy.

Five measurements, one section each:

  1  where the in-band power is, and in what lines        sec1_lines
  2  how fast the phase of each line wanders   <- headline  sec2_phase
  3  positive / negative controls on the same estimator    sec3_controls
  4  what a canceller would actually remove, simulated     sec4_canceller
  5  amplitude and frequency stationarity of the lines     sec5_stationarity

Everything is offline, on logs already in data/.  Nothing touches the serial
port and no test suite is run.

Two methodological points that changed the answer and are worth keeping:

  * Phase is tracked by a NON-UNIFORM lock-in straight on the raw irregular
    samples, with the phase referred to a common time origin.  Re-gridding
    first, or referring each window's phase to its own start, produces a
    spurious per-hop rotation of 2*pi*f0*hop -- at f0 = 1 Hz with a 0.5 s hop
    that is exactly pi, and it manufactures total decorrelation out of nothing.

  * The canceller simulation runs on `ch*_bp`, the controller's own causal
    0.4-3.0 Hz one-pole cascade as logged.  A brick-wall FFT bandpass applied to
    a whole window is non-causal: it leaks the application interval back into
    the estimation window and inflates the apparent cancellation by ~0.1 in
    power fraction.

Open loop means state CALIBRATING or FAULT -- in both the actuator is pinned at
BIAS (checked in analysis/oob_baseline.md).  Closed loop means DAMPING with the
scheduled gain slewed in; the first 8 s of every DAMPING segment is discarded
because the gain is still ramping.

Everything prints; no files are written.  Run from the repo root:

    .venv/bin/python analysis/passive_phase.py
"""
import glob
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

# ---------------------------------------------------------------- config

BAND = (0.4, 3.0)              # the controller's own bandpass, BP_LOW_HZ..BP_HIGH_HZ
LINES = (0.70, 1.00, 1.65)     # in-band lines, located in sec1
CONTROL_LINE = 6.09            # the known-coherent out-of-band line (positive control)
NULL_LINE = 2.30               # in-band, no line there (negative control)
FG = 20.0                      # uniform grid for spectra / canceller, Hz
FG_HI = 50.0                   # uniform grid when 6.09 / 12.4 Hz is wanted
MIN_OPEN_S = 15.0              # shortest usable open-loop window
LAGS = (0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0)

CACHE = os.path.join(tempfile.gettempdir(), "ligo_passive_phase_cache")

# ---------------------------------------------------------------- loading


def _awk(path, cols):
    """Pull a few columns out of a 100 MB csv without parsing all 108 of them."""
    head = open(path).readline().strip().split(",")
    idx = [head.index(c) for c in cols]
    prog = "NR>1{print " + ' " " '.join("$" + str(i + 1) for i in idx) + "}"
    out = subprocess.run(["awk", "-F,", prog, path], capture_output=True, text=True).stdout
    return np.fromstring(out, sep=" ").reshape(-1, len(cols))


def load(path):
    """-> t, counts(N,nch), bp(N,nch), state(N), nch.  state 0/1/2 = CAL/DMP/FLT."""
    st = os.stat(path)
    key = os.path.join(CACHE, f"{os.path.basename(path)}.{st.st_size}.{int(st.st_mtime)}.npy")
    head = open(path).readline().strip().split(",")
    nch = 8 if "ch7_counts" in head else 4
    if os.path.exists(key):
        a = np.load(key)
    else:
        si = head.index("state") + 1
        code = f'(($%d=="CALIBRATING")?0:($%d=="DAMPING")?1:($%d=="FAULT")?2:3)' % (si, si, si)
        cols = ["time_s"] + [f"ch{i}_counts" for i in range(nch)] + [f"ch{i}_bp" for i in range(nch)]
        idx = [head.index(c) for c in cols]
        prog = ("NR>1{print " + ' " " '.join("$" + str(i + 1) for i in idx)
                + ' " " ' + code + "}")
        out = subprocess.run(["awk", "-F,", prog, path], capture_output=True, text=True).stdout
        a = np.fromstring(out, sep=" ").reshape(-1, 2 * nch + 2)
        os.makedirs(CACHE, exist_ok=True)
        np.save(key, a)
    return a[:, 0], a[:, 1:1 + nch], a[:, 1 + nch:1 + 2 * nch], a[:, -1].astype(int), nch


def segments(t, st, want, min_dur):
    """Contiguous runs of one state lasting at least min_dur seconds."""
    m = (st == want).astype(np.int8)
    d = np.diff(np.concatenate(([0], m, [0])))
    out = []
    for s, e in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
        if e - s > 10 and t[e - 1] - t[s] >= min_dur:
            out.append((s, e))
    return out


def grid(t, x, fs):
    """Uniform re-grid by bin averaging.  The 50 ms boxcar also kills 50 Hz mains,
    which would otherwise alias down from the ~1 kHz stream."""
    n = int((t[-1] - t[0]) * fs)
    b = np.floor((t - t[0]) * fs).astype(int)
    ok = (b >= 0) & (b < n)
    b, xs = b[ok], np.atleast_2d(x[ok].T).T
    cnt = np.bincount(b, minlength=n).astype(float)
    y = np.stack([np.bincount(b, weights=xs[:, i], minlength=n) for i in range(xs.shape[1])], 1)
    good = cnt > 0
    y[good] /= cnt[good, None]
    if not good.all():
        ig, gg = np.where(~good)[0], np.where(good)[0]
        for i in range(y.shape[1]):
            y[ig, i] = np.interp(ig, gg, y[gg, i])
    return np.arange(n) / fs + t[0], y


def harvest(states, min_dur, want_fs, pattern="data/*_fast_lock.csv", drop_lead=0.0, chop=None):
    """Every segment in `states` long enough to use, as (tag, day, nch, t, counts, bp)."""
    out = []
    for p in sorted(glob.glob(pattern)):
        t, c, bp, st, nch = load(p)
        day = os.path.basename(p)[:8]
        for want in states:
            for s, e in segments(t, st, want, min_dur):
                fs = (e - s) / (t[e - 1] - t[s])
                if fs < want_fs:
                    continue
                s = s + int(drop_lead * fs)
                if e - s < 100 or t[e - 1] - t[s] < min_dur:
                    continue
                if np.nanmax(c[s:e]) > 1023 or np.nanmin(c[s:e]) < 0:
                    continue                       # corrupt line in the log
                pieces = [(s, e)]
                if chop:
                    L = int(chop * fs)
                    pieces = [(s + i, s + i + L) for i in range(0, e - s - L + 1, L)]
                for a, b in pieces:
                    out.append((os.path.basename(p), day, nch, t[a:b], c[a:b], bp[a:b]))
    return out


# ---------------------------------------------------------------- primitives


def asd(x, fs):
    """Single Hann-tapered periodogram, amplitude spectral density."""
    n = len(x)
    w = np.hanning(n)
    P = np.abs(np.fft.rfft((x - x.mean()) * w)) ** 2 / (fs * (w ** 2).sum()) * 2
    return np.fft.rfftfreq(n, 1 / fs), np.sqrt(P)


def nu_lockin(t, x, f0, Tw, hop):
    """Non-uniform lock-in on the raw irregular samples.  Phase is referred to a
    COMMON origin (t=0 of the segment), not to each window's own start."""
    zs, ts = [], []
    s = t[0]
    while s + Tw <= t[-1]:
        m = (t >= s) & (t < s + Tw)
        if m.sum() > 20:
            tt = t[m] - s
            xx = x[m] - x[m].mean()
            w = 0.5 - 0.5 * np.cos(2 * np.pi * tt / Tw)
            z = (xx * w * np.exp(-2j * np.pi * f0 * tt)).sum() / w.sum() * 2
            zs.append(z * np.exp(-2j * np.pi * f0 * s))
            ts.append(s + Tw / 2)
        s += hop
    return np.array(ts), np.array(zs)


def phase_structure(pairs, f0, lags, Tw=4.0, hop=0.5, detrend=True):
    """RMS *wrapped* phase increment vs lag, pooled over segments.

    `detrend` removes one best-fit linear phase ramp per segment, i.e. it grants
    the canceller a perfect estimate of the line's mean frequency over that
    segment.  Without it the number would also contain the fixed-frequency
    mismatch, which sec5 reports separately.

    A fully decorrelated phase gives pi/sqrt(3) = 1.81 rad.
    """
    acc = {L: [] for L in lags}
    amp = []
    for t, x in pairs:
        ts, z = nu_lockin(t, x, f0, Tw, hop)
        if len(z) < 8:
            continue
        amp.append(np.abs(z))
        if detrend:
            ph = np.unwrap(np.angle(z))
            z = z * np.exp(-1j * np.polyval(np.polyfit(ts, ph, 1), ts))
        for L in lags:
            k = int(round(L / hop))
            if k < 1 or k >= len(z) - 2:
                continue
            acc[L].append(np.angle(z[k:] * np.conj(z[:-k])))
    out = {}
    for L in lags:
        out[L] = (np.sqrt(np.mean(np.concatenate(acc[L]) ** 2)) if acc[L] else np.nan)
    return out, amp


def tau_1rad(o, lags):
    """Lag at which the RMS wrapped phase increment first reaches 1 radian."""
    prev_L, prev_v = 0.0, 0.0
    for L in lags:
        v = o[L]
        if np.isnan(v):
            continue
        if v >= 1.0:
            return prev_L + (1.0 - prev_v) / (v - prev_v) * (L - prev_L)
        prev_L, prev_v = L, v
    return np.inf


def cancel(d, fs, Test, tau, Tapp, ntone=1, skip=2.0, pad=8192, split_amp=False):
    """Simulate the canceller on one band-limited record.

    At each step, lock in over the previous `Test` seconds (Hann taper), take the
    `ntone` strongest tones in BAND, extrapolate them forward, and subtract over
    [t+tau, t+tau+Tapp].  The estimate uses no sample from the application
    interval, so there is no look-ahead.  Returns (residual power, original
    power) summed over disjoint application intervals.
    """
    n = len(d)
    ie, it, ia = (int(round(v * fs)) for v in (Test, tau, Tapp))
    f = np.fft.rfftfreq(pad, 1 / fs)
    b = (f >= BAND[0]) & (f <= BAND[1])
    fb = f[b]
    num = den = 0.0
    rows = []
    start = int(skip * fs)
    while start + ie + it + ia <= n:
        de = d[start:start + ie].copy()
        app = slice(start + ie + it, start + ie + it + ia)
        te = np.arange(ie) / fs
        ta = np.arange(app.start, app.stop) / fs - start / fs
        w = np.hanning(ie)
        pred = np.zeros(ia)
        for _ in range(ntone):
            X = np.fft.rfft(de * w, pad) * 2 / w.sum()
            k = int(np.argmax(np.abs(X[b])))
            f0, A = fb[k], X[b][k]
            pred += np.real(A * np.exp(2j * np.pi * f0 * ta))
            de = de - np.real(A * np.exp(2j * np.pi * f0 * te))
        r = float(np.sum((d[app] - pred) ** 2))
        p = float(np.sum(d[app] ** 2))
        num += r
        den += p
        rows.append((p, r))
        start += ia
    if split_amp:
        return num, den, rows
    return num, den, len(rows)


def banner(s):
    print("\n" + "=" * 78 + f"\n{s}\n" + "=" * 78)


# ---------------------------------------------------------------- sec 0


def sec0_inventory():
    banner("0  inventory")
    op = harvest((0, 2), MIN_OPEN_S, 2 * FG)
    dm = harvest((1,), 60.0, 2 * FG, drop_lead=8.0)
    tot = {}
    longest = 0.0
    for p in sorted(glob.glob("data/*_fast_lock.csv")):
        t, c, bp, st, nch = load(p)
        for w in (0, 1, 2):
            for s, e in segments(t, st, w, 4.0):
                tot[w] = tot.get(w, 0.0) + t[e - 1] - t[s]
                if w in (0, 2):
                    longest = max(longest, t[e - 1] - t[s])
    print(f"  logs                     : {len(glob.glob('data/*_fast_lock.csv'))}")
    print(f"  CALIBRATING / FAULT / DAMPING seconds : "
          f"{tot.get(0,0):.0f} / {tot.get(2,0):.0f} / {tot.get(1,0):.0f}")
    print(f"  longest CONTIGUOUS open-loop window   : {longest:.1f} s"
          "   <- caps every open-loop lag measurement")
    print(f"  usable open-loop windows (>={MIN_OPEN_S:.0f} s, fs>={2*FG:.0f} Hz)  : "
          f"{len(op)}  ({sum(w[3][-1]-w[3][0] for w in op):.0f} s)")
    print(f"  usable long closed-loop segments      : {len(dm)}  "
          f"({sum(w[3][-1]-w[3][0] for w in dm):.0f} s)")
    return op, dm


# ---------------------------------------------------------------- sec 1


def sec1_lines(op):
    banner("1  where the in-band power is")
    fgrid = np.arange(0.10, 9.9, 0.005)
    acc = {}
    for tag, day, nch, t, c, bp in op:
        tg, xg = grid(t, c, FG)
        for i in range(nch):
            f, a = asd(xg[:, i], FG)
            acc.setdefault(i, []).append(np.interp(fgrid, f, a))
    print("  open-loop ASD, counts/sqrt(Hz), median over windows")
    print(f"  {'ch':>3} {'n':>4} | " + "".join(f"{f'{l:.2f}Hz':>11}" for l in LINES)
          + f"{'2.30Hz':>11}{'6.09Hz':>11}")
    med = {}
    for i in sorted(acc):
        m = np.median(np.array(acc[i]), 0)
        med[i] = m
        vals = [m[np.argmin(np.abs(fgrid - l))] for l in list(LINES) + [NULL_LINE, CONTROL_LINE]]
        print(f"  a{i:<2} {len(acc[i]):>4} | " + "".join(f"{v:>11.1f}" for v in vals))
    # power fraction carried by each line (+-0.08 Hz) out of the whole 0.4-3 Hz band
    print("\n  fraction of 0.4-3.0 Hz power inside each line +-0.08 Hz")
    print(f"  {'ch':>3} | " + "".join(f"{f'{l:.2f}Hz':>10}" for l in LINES) + f"{'rest':>10}")
    for i in sorted(acc):
        m = med[i] ** 2
        b = (fgrid >= BAND[0]) & (fgrid <= BAND[1])
        tot = m[b].sum()
        fr = [m[(fgrid >= l - 0.08) & (fgrid <= l + 0.08)].sum() / tot for l in LINES]
        print(f"  a{i:<2} | " + "".join(f"{v:>10.2f}" for v in fr) + f"{1-sum(fr):>10.2f}")
    return med, fgrid


# ---------------------------------------------------------------- sec 2


def sec2_phase(op):
    banner("2  HOW FAST THE PHASE WANDERS  (headline)")
    print("  RMS wrapped phase increment, radians, vs lag.  Open loop, raw irregular")
    print("  samples, 4 s lock-in, best-fit mean frequency already removed.")
    print("  1.81 rad = completely random.  tau1 = lag at which it reaches 1 rad.")
    print(f"\n  {'line':>7} {'ch':>3} | " + "".join(f"{f'{L:g}s':>7}" for L in LAGS) + f"{'tau1':>9}")
    res = {}
    for f0 in LINES:
        for ch in range(4):
            pairs = [(t, c[:, ch]) for tag, day, nch, t, c, bp in op if ch < nch]
            o, amp = phase_structure(pairs, f0, LAGS)
            res[(f0, ch)] = (o, amp)
            t1 = tau_1rad(o, LAGS)
            print(f"  {f0:>7.2f} a{ch:<2} | " + "".join(f"{o[L]:>7.2f}" for L in LAGS)
                  + (f"{t1:>9.1f}" if np.isfinite(t1) else f"{'>10':>9}"))
        print()
    print("  what that costs a canceller: with an RMS phase error dphi, an otherwise")
    print("  perfect anti-phase injection leaves |1-exp(i.dphi)|^2 ~ dphi^2 of the power.")
    print(f"  {'dphi (rad)':>12} | " + "".join(f"{v:>8}" for v in
          ("0.2", "0.3", "0.5", "0.7", "1.0", "1.4")))
    print(f"  {'best supp.':>12} | " + "".join(
        f"{-10*np.log10(min(1.0, d*d)):>7.1f}dB" for d in (0.2, 0.3, 0.5, 0.7, 1.0, 1.4)))
    return res


# ---------------------------------------------------------------- sec 3


def sec3_controls(op):
    banner("3  controls on the same estimator")
    hi = [w for w in op if w[2] == 8 and (len(w[3]) / (w[3][-1] - w[3][0])) >= 100]
    print(f"  positive control: the {CONTROL_LINE} Hz line on a4/a6/a7 -- out of band, known")
    print(f"  coherent, and per analysis/where_is_power.py it carries 91-99% of those")
    print(f"  channels' power.  {len(hi)} open-loop windows with fs >= 100 Hz.")
    print(f"\n  {'line':>7} {'ch':>3} | " + "".join(f"{f'{L:g}s':>7}" for L in LAGS) + f"{'tau1':>9}")
    for ch in (4, 6, 7):
        pairs = [(t, c[:, ch]) for tag, day, nch, t, c, bp in hi]
        o, _ = phase_structure(pairs, CONTROL_LINE, LAGS)
        t1 = tau_1rad(o, LAGS)
        print(f"  {CONTROL_LINE:>7.2f} a{ch:<2} | " + "".join(f"{o[L]:>7.2f}" for L in LAGS)
              + (f"{t1:>9.1f}" if np.isfinite(t1) else f"{'>10':>9}"))
    print(f"\n  negative control: {NULL_LINE} Hz, in band, no line there.")
    for ch in range(4):
        pairs = [(t, c[:, ch]) for tag, day, nch, t, c, bp in op if ch < nch]
        o, _ = phase_structure(pairs, NULL_LINE, LAGS)
        t1 = tau_1rad(o, LAGS)
        print(f"  {NULL_LINE:>7.2f} a{ch:<2} | " + "".join(f"{o[L]:>7.2f}" for L in LAGS)
              + (f"{t1:>9.1f}" if np.isfinite(t1) else f"{'>10':>9}"))
    print("\n  white-noise floor of the estimator (synthetic, same window structure):")
    rng = np.random.default_rng(0)
    pairs = [(w[3], rng.normal(size=len(w[3]))) for w in op]
    o, _ = phase_structure(pairs, 1.00, LAGS)
    print(f"  {'white':>7} {'--':>3} | " + "".join(f"{o[L]:>7.2f}" for L in LAGS))


# ---------------------------------------------------------------- sec 4


def sec4_canceller(op, dm):
    banner("4  what the canceller would actually remove")
    print("  Fraction of 0.4-3.0 Hz power removed from ch*_bp, the controller's own")
    print("  causal bandpass.  Positive = cancels, NEGATIVE = pumps.")
    print("  K tones, Test s of lock-in, tau s of latency, applied for Tapp s.")

    def gridded(ws):
        out = []
        for tag, day, nch, t, c, bp in ws:
            tg, bg = grid(t, bp, FG)
            out.append((bg, nch))
        return out

    OP, DM = gridded(op), gridded(dm)
    DMc = gridded(harvest((1,), 60.0, 2 * FG, drop_lead=8.0, chop=20.0))

    def run(sets, K, Test, tau, Tapp):
        row = []
        for ch in range(4):
            N = D = 0.0
            for bg, nch in sets:
                if ch >= nch:
                    continue
                a, b, _ = cancel(bg[:, ch], FG, Test, tau, Tapp, ntone=K)
                N += a
                D += b
            row.append(1 - N / D if D > 0 else np.nan)
        return row

    print(f"\n  OPEN LOOP  ({len(OP)} windows)")
    print(f"  {'K':>2} {'Test':>5} {'tau':>5} {'Tapp':>5} | " + "".join(f"{f'a{c}':>9}" for c in range(4)))
    for K in (1, 2, 3):
        for Test in (4.0, 8.0, 12.0):
            for tau, Tapp in ((0.0, 1.0), (0.5, 1.0), (0.5, 2.0)):
                r = run(OP, K, Test, tau, Tapp)
                print(f"  {K:>2} {Test:>5.1f} {tau:>5.1f} {Tapp:>5.1f} | "
                      + "".join(f"{v:>+9.3f}" for v in r))

    print(f"\n  CLOSED LOOP, on top of the existing damper  ({len(DM)} long segments,"
          f" {len(DMc)} 20 s chunks)")
    print(f"  {'K':>2} {'Test':>5} {'tau':>5} {'Tapp':>5} {'set':>8} | "
          + "".join(f"{f'a{c}':>9}" for c in range(4)))
    for K in (1, 3):
        for Test in (8.0, 16.0):
            for tau, Tapp in ((0.5, 1.0), (0.5, 4.0)):
                for nm, S in (("full", DM), ("20s", DMc)):
                    if nm == "20s" and 2.0 + Test + tau + Tapp > 20.0:
                        continue                    # will not fit in a 20 s chunk
                    r = run(S, K, Test, tau, Tapp)
                    print(f"  {K:>2} {Test:>5.1f} {tau:>5.1f} {Tapp:>5.1f} {nm:>8} | "
                          + "".join(f"{v:>+9.3f}" for v in r))

    # quiet vs post-kick, by application-interval amplitude quartile
    print("\n  quiet vs loud, open loop, K=1 Test=8 tau=0.5 Tapp=1.")
    print("  Application intervals sorted by their own power; bottom vs top quartile.")
    print(f"  {'ch':>3} | {'quiet 25%':>11} {'mid 50%':>11} {'loud 25%':>11}")
    for ch in range(4):
        rows = []
        for bg, nch in OP:
            if ch >= nch:
                continue
            _, _, rr = cancel(bg[:, ch], FG, 8.0, 0.5, 1.0, ntone=1, split_amp=True)
            rows += rr
        rows = np.array(rows)
        q = np.argsort(rows[:, 0])
        n = len(q)
        cuts = [q[:n // 4], q[n // 4:3 * n // 4], q[3 * n // 4:]]
        print(f"  a{ch:<2} | " + "".join(
            f"{1 - rows[c,1].sum()/rows[c,0].sum():>+11.3f}" for c in cuts))


# ---------------------------------------------------------------- sec 5


def sec5_stationarity(op, dm):
    banner("5  amplitude and frequency stationarity")
    print("  Per-window lock-in amplitude of each line, 4 s windows, open loop.")
    print(f"  {'line':>7} {'ch':>3} | " + "".join(f"{d:>12}" for d in ("20260803", "20260804", "20260806"))
          + f"{'IQR/med':>10}")
    for f0 in LINES:
        for ch in range(4):
            cells, allA = [], []
            for day in ("20260803", "20260804", "20260806"):
                A = []
                for tag, d, nch, t, c, bp in op:
                    if d != day or ch >= nch:
                        continue
                    _, z = nu_lockin(t, c[:, ch], f0, 4.0, 1.0)
                    if len(z):
                        A.append(np.abs(z))
                if A:
                    A = np.concatenate(A)
                    allA.append(A)
                    cells.append(f"{np.median(A):>12.1f}")
                else:
                    cells.append(f"{'--':>12}")
            A = np.concatenate(allA)
            iqr = np.subtract(*np.percentile(A, [75, 25])) / np.median(A)
            print(f"  {f0:>7.2f} a{ch:<2} | " + "".join(cells) + f"{iqr:>10.2f}")
        print()

    print("  Frequency wander: peak of the 0.55-0.85 / 0.88-1.12 / 1.50-1.80 Hz")
    print("  band, one estimate per long closed-loop segment (best resolution there).")
    print(f"  {'segment':>28} {'dur':>6} | " + "".join(f"{f'{l:.2f}Hz':>10}" for l in LINES))
    peaks = {l: [] for l in LINES}
    for tag, day, nch, t, c, bp in dm:
        tg, xg = grid(t, c, FG)
        f, a = asd(xg[:, 0], FG)
        row = []
        for l, (lo, hi) in zip(LINES, ((0.55, 0.85), (0.88, 1.12), (1.50, 1.80))):
            b = (f >= lo) & (f <= hi)
            pk = f[b][np.argmax(a[b])]
            peaks[l].append(pk)
            row.append(pk)
        print(f"  {tag[:24]:>28} {tg[-1]-tg[0]:>6.0f} | " + "".join(f"{v:>10.4f}" for v in row))
    print(f"  {'spread (max-min)':>28} {'':>6} | "
          + "".join(f"{max(peaks[l])-min(peaks[l]):>10.4f}" for l in LINES))
    print(f"  {'as a fraction of f0':>28} {'':>6} | "
          + "".join(f"{(max(peaks[l])-min(peaks[l]))/l:>10.3f}" for l in LINES))
    return peaks


# ---------------------------------------------------------------- sec 6


def sec6_plant(peaks=None):
    banner("6  the plant at the line frequencies -- what a canceller would need")
    if not os.path.exists("gains.json"):
        print("  gains.json missing")
        return
    g = json.load(open("gains.json"))
    print(f"  gains.json generated {g['generated']} from {g['source']['raw_csv']}")
    print("  driven tones (Hz):", ", ".join(f"{f:.4f}" for f in g["measurement"]["freqs_hz"]))
    print("\n  fitted modes:")
    for m in g["plant"]["modes"]:
        gam = np.pi * m["f_hz"] / m["Q"]
        print(f"    f0={m['f_hz']:.4f} Hz  Q={m['Q']:.0f}  zeta={m['zeta']:.5f}"
              f"  =>  amplitude decay 1/gamma = {1/gam:.1f} s")
    print("\n  nearest driven tone to each line, and the phase risk:")
    tones = np.array(g["measurement"]["freqs_hz"])
    m0 = g["plant"]["modes"][0]

    def ph(f, f0, Q):
        """plant phase near a resonance, degrees."""
        w, w0 = 2 * np.pi * f, 2 * np.pi * f0
        return np.degrees(-np.arctan2(w * w0 / Q, w0 ** 2 - w ** 2))

    for l in LINES:
        k = int(np.argmin(np.abs(tones - l)))
        d = abs(tones[k] - l)
        p_lo = ph(l, m0["f_hz"], 21.0)     # Q from the ringdown envelopes
        p_hi = ph(l, m0["f_hz"], m0["Q"])  # Q from the multisine fit
        print(f"    {l:.2f} Hz: nearest tone {tones[k]:.4f} Hz, {d*1000:.0f} mHz away"
              f" | plant phase {p_lo:+.0f} deg (Q=21) vs {p_hi:+.0f} deg (Q={m0['Q']:.0f})"
              f" -> {abs(p_lo-p_hi):.0f} deg of ambiguity")
    if peaks:
        print("\n  and the same phase across the measured frequency wander of each line:")
        for l, m in zip(LINES[:2], g["plant"]["modes"]):
            lo, hi = min(peaks[l]), max(peaks[l])
            for Q in (21.0, m["Q"]):
                a = ph(lo, m["f_hz"], Q)
                b = ph(hi, m["f_hz"], Q)
                print(f"    {l:.2f} Hz line wanders {lo:.4f}-{hi:.4f} Hz  "
                      f"= {(hi-lo)/(m['f_hz']/Q):.1f} half-power widths at Q={Q:.0f}"
                      f"  ->  plant phase {a:+.0f} to {b:+.0f} deg, swing {abs(b-a):.0f} deg")
    print("\n  Q is contradicted between the two measurements on disk: the gain-off")
    print("  ringdown envelopes give Q = 20-22 (analysis/mimo_design.md), the multisine")
    print("  fit in gains.json gives 288 and 2334.  Near a resonance the plant phase")
    print("  rotates by 90 deg over f0/(2Q), so that disagreement is a sign ambiguity.")


# ---------------------------------------------------------------- main

if __name__ == "__main__":
    if not glob.glob("data/*_fast_lock.csv"):
        sys.exit("run from the repo root: .venv/bin/python analysis/passive_phase.py")
    op, dm = sec0_inventory()
    sec1_lines(op)
    sec2_phase(op)
    sec3_controls(op)
    sec4_canceller(op, dm)
    peaks = sec5_stationarity(op, dm)
    sec6_plant(peaks)
    print()
