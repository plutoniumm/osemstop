#!/usr/bin/env python3
"""joint_a.py -- A[mode, coil] by fitting a WHOLE coil pass as one dynamical
system, instead of one lock-in per point.

WHY THIS EXISTS
---------------
A's SIGNS are settled -- 12 of 12 across three determinations that share no
estimator (`CLAUDE.md` Sec What the 2026-08-18 00:00-00:30 session established).
A's MAGNITUDES are not. The two DC passes reproduce individual entries only to a
factor 0.23-1.8, and the driven pass now in `data/modal.json` carries
off-quadrature 0.624 / 0.749 / 1.196 where resonance wants ~0
(`data/modal.json`, key a.imag_frac).

THE CAUSE IS MEASURED AND IT IS NOT THE UNWIND LOGIC. tau > 138 s
(`analysis/ringdown.md` line 56) against a 74 s point, so the previous point is
still ringing when the next starts. |pre|/|drive| over all twelve points of
`data/20260817_231251_status_coils.csv`: 417.4, 31.3, 35.4, 96.0, 96.7, 3.5,
79.5, 26.6, 47.5, 36.7, 62.2, 29.5 % (`status.py:_point_multi` docstring, which
records the same list).

FOUR FIXES HAVE FAILED AND THEY SHARE ONE DEFECT: ALL FOUR ARE LOCAL.
Numbers from `CLAUDE.md` Sec 3:

    subtract the 6 s `pre` phasor   off-quadrature 0.65/0.83/1.23 -> 1.26/1.10/1.19
    fit the drive to c + b*t                                      -> 0.87/1.19/1.25
    MULTI_PRE_S = 30 s pre window   per-coil phase spread 41.3 deg -> 36.7 deg
                                    (the tool's own "no difference" line is 5 deg)
    multisine                       spread 73-89 -> 31.6/36.7/7.8 deg, but the
                                    SIGNS broke: 7 of 12 against 12 of 12

Each treats one point in isolation and removes the ring as if it were noise.

WHAT THIS FILE DOES INSTEAD
---------------------------
The ring is not noise. It is state, it is deterministic, and the recorded pass
contains the free decay that determines it -- 30 s of unwind and 8 s of quiet
per point, which every per-point lock-in throws away because it only reads the
drive window.

Per mode m, with the drive u_j(t) read from the file (never the amplitude that
was intended -- `CLAUDE.md` Sec Standing practice: always stream the raw data):

    qdd_m + 2 gamma_m qd_m + w_m^2 q_m = SUM_j A[m,j] u_j(t)
    y_i(t) = SUM_m Phi[i,m] q_m(t) + d_i(t) + n_i(t)

Phi is KNOWN and exact -- `status.GEO_PHI`, from the rig owner's stated geometry
plus the corner assignment that `dof.py` determined at 2x over the next pairing
(`CLAUDE.md` Sec The geometry). So there is no per-mode sign gauge to search.

GIVEN (w, gamma) THE MODEL IS LINEAR IN A, so this is separable least squares:
6 nonlinear parameters outside, one linear solve inside.

THE ONE STRUCTURAL SIMPLIFICATION WORTH KNOWING. On a0-a3 the three geometric
columns and the warp direction are mutually orthogonal with Phi^T Phi = 4 I, so
B = [Phi_A, Phi_B, Phi_C, warp] has B^T B = 4 I and B/2 is an orthogonal matrix.
Least squares over the four SENSORS with a free per-sensor offset is therefore
EXACTLY least squares over the four MODAL COORDINATES with a free per-coordinate
offset, and the three modes DECOUPLE COMPLETELY. Each mode is its own 2-parameter
nonlinear problem. Warp gets no parameters at all and its residual is reported.

WHAT IS AND IS NOT IDENTIFIABLE, and this is a result rather than a caveat
------------------------------------------------------------------------
Each recorded segment restarts `time_s` at 0 (`status.acquire` takes a fresh
origin per `_point`, and `_point` takes one per point), so the wall-clock gaps
BETWEEN points are not in the file. This module therefore does NOT chain the
points: every block carries its own free initial condition (q0, v0) per mode.
That is not a compromise -- within a block the pre / drive / unwind / quiet share
ONE origin, which is 74 s of continuous record, and over that span:

  - the leftover ring is a free decaying sinusoid at w_m,
  - the on-resonance driven response RAMPS as t*sin(wt) (`status.ramp_gain`),
  - and the unwind REVERSES the drive while the ring keeps going.

Those three are strongly separable. `--selftest` measures how strongly.

A DC STEP BLOCK CARRIES EXACTLY ZERO INFORMATION ABOUT A IN THIS MODEL, and the
proof is two lines: for a constant force c the solution from rest is
q(t) = (c/w^2)(1 - f0(t)) where f0 is the free response from (q0, v0) = (1, 0).
So the step response lies EXACTLY in the span of {constant, f0}, which are
already the block's own DC and initial-condition columns. DC blocks are kept for
the (w, gamma) fit -- their ringing shape is informative -- and excluded from the
A design. Their information about A is recovered separately and independently,
below.

THE INDEPENDENT MAGNITUDE CHECK. The DC blocks give d(counts_i)/d(bias_j) as a
difference of two 20 s means, which is immune to the ring because the ring is
zero-mean about the new equilibrium (`status._dc_point` docstring, same argument
as `analysis/bias_sweep.py`). Project it on Phi and it must equal the fitted
A[m,j] / w_m^2 converted to counts. That comparison shares no estimator with the
dynamic fit and is the sharpest available test of the MAGNITUDES.

UNITS
-----
A here is PHYSICAL: (sensor volts) * rad^2/s^2 per coil volt. `data/modal.json`
holds a different quantity -- `status.py` computes H = lockin(y)/lockin(u) over
the ramping 30 s drive window and exports -Im(H) projected on Phi, so the
shipping A is a lock-in ratio in V/V whose value depends on DWELL_S. `as_lockin`
converts, by simulating that same lock-in on this model.

WHAT IT MEASURED -- 2026-08-20, six recorded passes, no bench time
------------------------------------------------------------------
Logs: the scratchpad `joint_a_all.log` / `joint_a_selftest.log` for this session.

THE MACHINERY IS VALIDATED AGAINST THE SHIPPING TOOL. `lockin_A` on
`data/20260817_231251_status_coils.csv` reproduces the A in `data/modal.json`
-- which was written FROM that file -- to within 5 % on 10 of 12 entries, median
ratio 1.0037, 12 of 12 in sign (the outlier is C/coil0 at 1.54x, a drive-window
boundary difference). `dc_matrix` reproduces `status.py coils --replay`'s own DC
table to 1-4 %. So the block splitting, the modal projection, the drive
extraction and the units are right.

THE ESTIMATOR IS BETTER THAN THE LOCK-IN, ON SYNTHETIC DATA. `selftest
--ring-sweep`, 5 seeds, leftover ring 11-127 % of the driven peak:

    joint    |err| median 14.7-30.9 %, rms 32.6-46.1 %, signs 11-12 of 12
    lock-in  |err| median 26.4-79.7 %, rms 50.7-153.2 %, signs  8-11 of 12

AND IT DOES NOT HELP ON THE REAL RECORDS. Signs against the settled table, per
pass: 9, 10, 10 (one-at-a-time), 6 (the 23:59 multisine), 7 (the 01:18 damp pass,
which has NO unwind and NO quiet window -- exactly the windows this method needs).
Held-out skill on the on-resonance points is mostly NEGATIVE. Pooling all 111
driven blocks of all six passes into one solve gives 10 of 12; pooling only the
three comparable one-at-a-time passes gives 9 of 12, with mode A wrong on 3 of 4
coils. Cross-pass spread over those three passes: median x1.69, worst x6.08,
median sd/|mean| 27 %, and only 9 of 12 entries keep their sign.

THE REASON IS SNR, NOT THE RING MODEL, AND IT IS MEASURED. `headroom` on the
23:12 pass: the driven peak is 2.1-12.3x the pre-window ambient rms; every point
used 7-33x LESS than AMP_MAX; and the rail guard would have allowed 2.0-5.0x
more swing. R^2 of the drive term over a whole pass is 0.05-0.43, and 14-75 % of
what is left sits within 0.05 Hz of the mode line itself. Mode A is worst, which
fits: T1 carries the most ambient energy of the three, 24.3 counts rms against
Z 11.1 and T2 6.5 (`status.park_ramped`).

AND THERE IS NO CROSS-FREQUENCY REDUNDANCY TO RECOVER IT WITH. A resonant point
is 22-145x larger in amplitude than the same coil driven at another mode's
frequency (`leave_one_point_out`), so each A[m,j] is still determined by ONE
point.

A FULLY COHERENT WHOLE-PASS FIT IS NOT AVAILABLE IN PRINCIPLE EITHER. Fitting
each pass's two halves separately moves the mode frequency by up to
+0.00291 Hz (mode A, 23:59) and -0.00268 Hz (mode B, 23:59) against a formal
sigma of 2-5e-5 Hz. The modes drift INSIDE one 30-minute pass.

WHAT DOES DETERMINE THE MAGNITUDES: THE DC STEPS. `joint_a.py dc`, the same
five passes, the geometric projection of the measured DC matrix -- 12 of 12
signs against the settled table, ALL 12 entries keeping the same sign in every
pass, median cross-pass sd/|mean| 20 %, median max/min x1.58. The driven route
does not come close to that on this rig.

SATURATION: 0 blocks in any pass had u at the 0.00-0.50 V clamp and 0 blocks had
more than 2 % of a0-a3 at an ADC rail, so no pass is disqualified for it.

    python3 analysis/joint_a.py selftest              # the estimator, known answer
    python3 analysis/joint_a.py selftest --ring-sweep # 5 seeds, joint vs lock-in
    python3 analysis/joint_a.py fit data/20260817_231251_status_coils.csv
    python3 analysis/joint_a.py all --out /tmp/joint_a.json
    python3 analysis/joint_a.py pool /tmp/joint_a.json   # one A from every pass
    python3 analysis/joint_a.py dc                    # the static route, no fit
    python3 analysis/joint_a.py headroom              # what each point spent
"""
import os
import sys
import math
import cmath
import json
import time
import argparse

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import status as st                                      # noqa: E402

SCRATCH = os.environ.get(
    "JOINT_A_SCRATCH",
    "/private/tmp/claude-501/-Users-gojira-Documents-GitHub-nosync-ligo/"
    "e451dd35-cf65-416c-b489-d4c19bb2cea8/scratchpad")

# The settled sign table. Three determinations that share no estimator -- two
# static DC passes and one resonant lock-in -- agree on all twelve
# (`CLAUDE.md` Sec What the 2026-08-18 00:00-00:30 session established).
SIGNS_SETTLED = np.array([[-1, +1, +1, +1],      # mode A (T1)
                          [-1, +1, -1, -1],      # mode B (Z)
                          [-1, -1, -1, +1]],     # mode C (T2)
                         float)

# Per-mode envelope decay seeds, `analysis/out/ringdown_mode_summary.csv`
# gamma_median. NOT a measurement of the plant: the quiet record puts the whole
# plant at 0.0072 +/- 0.0023 /s and cannot exclude gamma = 0
# (`analysis/ringdown.md` lines 178, 441). Seeds only; gamma is FITTED here.
GAMMA_SEED = {"A": 0.0483, "B": 0.0082, "C": 0.0094}

# Ambient modal rms in counts, T1 / Z / T2. Measured and quoted in
# `status.park_ramped`'s docstring: "T1 carries the most ambient energy of the
# three (24.3 counts rms against Z 11.1 and T2 6.5)". Used only by `selftest`,
# to drive the synthetic plant the way the room drives the real one.
AMBIENT_COUNTS = (24.3, 11.1, 6.5)

FS_FIT = 100.0            # Hz. 60x mode C, 138x mode A. The coil is updated at
                          # status.UPDATE_HZ = 50 Hz, so a 10 ms bin holds at
                          # most one change and the bin MEAN is the right ZOH
                          # value. --fs re-runs at another rate as a check.
NCOIL = 4                 # a0-a3 / coils 0-3; a4-a7 carry no Phi row
NMODE = 3

PHASE_CODE = {"passive": 0, "dc": 1, "pre": 2, "drive": 3,
              "unwind": 4, "quiet": 5, "pulse": 6, "ring": 7}
PHASE_NAME = {v: k for k, v in PHASE_CODE.items()}


# ===========================================================================
# loading
# ===========================================================================
def _cache_path(path):
    os.makedirs(SCRATCH, exist_ok=True)
    base = os.path.basename(path).replace(".csv", "")
    return os.path.join(SCRATCH, "joint_a_%s.npz" % base)


def load_raw(path, use_cache=True):
    """time_s, seg, phase-code, coil, freq, u_v, counts[:, 0:8] for one pass.

    Cached to the scratchpad because a 719 170-row parse is not free and this
    module reads the same five files many times. Standing practice in
    `CLAUDE.md` is that raw data goes to disk; the cache is derived, so it is
    keyed on the source basename and regenerated with --no-cache.
    """
    cp = _cache_path(path)
    if use_cache and os.path.exists(cp) and \
            os.path.getmtime(cp) >= os.path.getmtime(path):
        z = np.load(cp)
        return {k: z[k] for k in z.files}
    num = np.loadtxt(path, delimiter=",", skiprows=1,
                     usecols=(0, 1, 3, 4, 6) + tuple(range(7, 7 + st.NCOLS)))
    ph = np.empty(num.shape[0], np.int8)
    with open(path) as fh:
        fh.readline()
        for k, line in enumerate(fh):
            if k >= ph.size:
                break
            a = line.find(",", line.find(",") + 1) + 1
            b = line.find(",", a)
            ph[k] = PHASE_CODE.get(line[a:b], 0)
    out = dict(t=num[:, 0], seg=num[:, 1].astype(np.int32), phase=ph,
               coil=num[:, 2].astype(np.int32), freq=num[:, 3],
               u_v=num[:, 4], counts=num[:, 5:])
    np.savez_compressed(cp, **out)
    return out


# ===========================================================================
# blocks
# ===========================================================================
class Block(object):
    """One stretch of record with a single continuous time origin.

    `status.acquire` restarts `time_s` at every fresh origin, and a fresh origin
    is taken per `_point` and per `_dc_point` segment, so a maximal run of
    non-decreasing `time_s` is exactly one shared-origin stretch. That merges
    pre + drive + unwind + quiet into ONE 74 s block, which is the whole point,
    and it splits off the 10 s between-coils hold that `coils_run` records
    without calling `rec.mark` (segments 14/28/42/56 of the 23:12 pass).
    """

    def __init__(self, t, u, z, phases, coil, freq, dt, raw_n, clipped, railed,
                 y=None, rest=None):
        self.t, self.u, self.z = t, u, z      # (K,), (K, NCOIL), (K, 4)
        self.y = y                            # (K, 4) per-sensor volts, or None
        self.rest = rest                      # resting counts a0-a7 in this block
        self.phases, self.coil, self.freq = phases, coil, freq
        self.dt, self.raw_n = dt, raw_n
        self.clipped, self.railed = clipped, railed
        self.driven = [j for j in range(NCOIL)
                       if np.max(np.abs(u[:, j])) > 1e-4]
        self.is_dc = (PHASE_CODE["dc"] in phases)
        self.dur = float(t[-1] - t[0]) if len(t) > 1 else 0.0

    def label(self):
        ph = "+".join(PHASE_NAME[p] for p in sorted(self.phases))
        return "coil %d  %-24s  %5.1f s  f %.4f  drives %s" % (
            self.coil, ph, self.dur, self.freq,
            ",".join(str(j) for j in self.driven) or "-")


def split_blocks(raw, fs=FS_FIT, drop_s=0.0):
    """Raw rows -> resampled Blocks on a uniform grid.

    Resampling is a BOXCAR over each grid bin, i.e. the mean of every raw sample
    that landed in it. That is the correct anti-alias for a 1 kHz stream carrying
    nothing above 1.7 Hz -- the first null sits at `fs` and the in-band droop is
    sinc(1.66/100) = 0.9995 -- and it is also the right thing to do to `u_v`,
    whose ZOH steps at 50 Hz are averaged into the bin the model integrates over.
    """
    t, u_v, coil = raw["t"], raw["u_v"], raw["coil"]
    counts = raw["counts"]
    n = len(t)
    cut = np.nonzero(np.diff(t) < -0.5)[0] + 1              # a time origin reset
    edges = np.concatenate(([0], cut, [n]))
    dt = 1.0 / fs
    # counts -> volts -> modal coordinates. Phi^T Phi = 4 I on a0-a3, so the
    # 1/4 makes each coordinate a sensor-volt-scaled quantity and the transform
    # orthogonal up to a factor 2 (see the module docstring).
    B = np.column_stack([st.GEO_PHI[:4, 0], st.GEO_PHI[:4, 1],
                         st.GEO_PHI[:4, 2], st.GEO_WARP])
    blocks = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a < 64:
            continue
        tt, cc = t[a:b], counts[a:b]
        if tt[-1] - tt[0] < 2.0:
            continue
        K = int(math.floor((tt[-1] - tt[0]) / dt)) + 1
        if K < 32:
            continue
        idx = np.clip(np.round((tt - tt[0]) / dt).astype(int), 0, K - 1)
        cnt = np.bincount(idx, minlength=K).astype(float)
        keep = cnt > 0
        if keep.sum() < 32:
            continue
        # per-coil drive about bias; only the coil named by the `coil` column is
        # modulated, every other coil is held at BIAS_V (status.acquire holds
        # `held = BIAS_V` for coil=None and only ever moves DAC_MAP[coil]).
        du = u_v[a:b] - st.BIAS_V
        cj = coil[a:b]
        U = np.zeros((K, NCOIL))
        for j in range(NCOIL):
            m = (cj == j)
            if not m.any():
                continue
            s = np.bincount(idx[m], weights=du[m], minlength=K)
            U[:, j] = np.where(keep, s / np.maximum(cnt, 1.0), 0.0)
        Y = np.empty((K, 4))
        v4 = cc[:, :4] * st.COUNTS_TO_V
        for i in range(4):
            s = np.bincount(idx, weights=v4[:, i], minlength=K)
            Y[:, i] = np.where(keep, s / np.maximum(cnt, 1.0), 0.0)
        Z = (Y @ B) / 4.0
        g = np.nonzero(keep)[0]
        tg = tt[0] + g * dt
        blk = Block(tg, U[g], Z[g],
                    set(int(x) for x in np.unique(raw["phase"][a:b])),
                    int(np.bincount(np.maximum(cj, 0)).argmax()),
                    float(np.median(raw["freq"][a:b])), dt, b - a,
                    clipped=bool(np.any(u_v[a:b] <= st.VMIN + 1e-9) or
                                 np.any(u_v[a:b] >= st.VMAX - 1e-9)),
                    railed=float(np.mean((cc[:, :4] <= st.RAIL_LOW) |
                                         (cc[:, :4] >= st.RAIL_HIGH))),
                    y=Y[g], rest=cc.mean(axis=0))
        if drop_s > 0 and blk.dur < drop_s:
            continue
        blocks.append(blk)
    return blocks


# ===========================================================================
# the resonator -- exact ZOH, no integrator
# ===========================================================================
def free_columns(tt, w, gamma):
    """(f0, f1): q(t) for initial conditions (1, 0) and (0, 1).

    Analytic, so there is no integrator to drift. With gamma = 0 the identity
    f0^2 + (w f1)^2 == 1 holds exactly for all t and `selftest` checks it over
    the full record length, which is the energy-conservation check `CLAUDE.md`
    asks for before trusting a fit.
    """
    wd = math.sqrt(max(w * w - gamma * gamma, 1e-18))
    e = np.exp(-gamma * tt)
    c, s = np.cos(wd * tt), np.sin(wd * tt)
    return e * (c + (gamma / wd) * s), e * s / wd


def forced_q(u, dt, w, gamma):
    """q[k] from rest for a ZOH input u[k]. Exact discretisation, vectorised.

    With the pole lam = -gamma + i wd and h(tau) = Im(exp(lam tau))/wd,

        Z[k+1] = a Z[k] + b u[k],   a = exp(lam dt),  b = (a - 1)/lam
        q[k]   = Im(Z[k]) / wd

    is exact for piecewise-constant u. The closed form
    Z[k] = a^k b SUM_{j<k} a^-(j+1) u[j] turns that recursion into a cumsum, and
    the scan is CHUNKED so |a^-n| never exceeds e^2 -- an unchunked scan over a
    74 s block at gamma = 0.2 would span 2.7e6 and eat six digits.
    """
    wd = math.sqrt(max(w * w - gamma * gamma, 1e-18))
    lam = complex(-gamma, wd)
    a = cmath.exp(lam * dt)
    b = (a - 1.0) / lam
    K = len(u)
    nmax = max(int(2.0 / max(gamma * dt, 1e-12)), 16)
    nmax = int(min(nmax, K))
    Z = np.empty(K, complex)
    carry = 0.0 + 0.0j
    s = 0
    while s < K:
        e = min(s + nmax, K)
        n = e - s
        k = np.arange(n)
        ap = a ** k
        c = (u[s:e] / a) / ap                       # a^-(j+1) u[j]
        C = np.concatenate(([0.0 + 0.0j], np.cumsum(c)[:-1]))
        Z[s:e] = ap * carry + ap * b * C
        carry = a * Z[e - 1] + b * u[e - 1]
        s = e
    return Z.imag / wd


def forced_state(u, dt, w, gamma):
    """(q, v) from rest for a ZOH input. Same recursion, both components.

    Z' = lam Z + u, so v = Im(lam Z)/wd = -gamma q + Re(Z). Only the synthetic
    generator needs the velocity -- the fit never does, because a block's
    initial state is a free parameter -- but the generator needs it to CARRY the
    true state from one point into the next, which is the whole thing being
    modelled.
    """
    wd = math.sqrt(max(w * w - gamma * gamma, 1e-18))
    lam = complex(-gamma, wd)
    a = cmath.exp(lam * dt)
    b = (a - 1.0) / lam
    K = len(u)
    nmax = max(int(2.0 / max(gamma * dt, 1e-12)), 16)
    nmax = int(min(nmax, K))
    Z = np.empty(K, complex)
    carry = 0.0 + 0.0j
    s = 0
    while s < K:
        e = min(s + nmax, K)
        n = e - s
        k = np.arange(n)
        ap = a ** k
        c = (u[s:e] / a) / ap
        C = np.concatenate(([0.0 + 0.0j], np.cumsum(c)[:-1]))
        Z[s:e] = ap * carry + ap * b * C
        carry = a * Z[e - 1] + b * u[e - 1]
        s = e
    q = Z.imag / wd
    return q, -gamma * q + Z.real


def propagate(x, dt, w, gamma):
    """Free state transition over dt. f0' = -w^2 f1 and f1' = f0 - 2 gamma f1,
    both by differentiating `free_columns` -- no matrix exponential needed."""
    f0, f1 = free_columns(np.array([dt]), w, gamma)
    f0, f1 = float(f0[0]), float(f1[0])
    return np.array([f0 * x[0] + f1 * x[1],
                     -w * w * f1 * x[0] + (f0 - 2.0 * gamma * f1) * x[1]])


# ===========================================================================
# the separable least squares
# ===========================================================================
class ModeProblem(object):
    """One mode's fit: A[m, :] global, (q0, v0, DC...) free per block.

    The linear unknowns split into 4 shared columns and a handful per block, so
    the normal equations are bordered block-diagonal and the block-local
    parameters are eliminated exactly by a Schur complement. What survives is a
    4x4 system, which is why a full nonlinear search over (w, gamma) is cheap:

        M   = SUM_b [ G'G - G'L (L'L)^-1 L'G ]
        r   = SUM_b [ G'y - G'L (L'L)^-1 L'y ]
        A   = M^-1 r
        RSS = SUM_b [ q_b - 2 A'r_b + A' M_b A ],  q_b = y'y - y'L (L'L)^-1 L'y

    q_b is the residual of the block fitted with NO drive term at all, so
    1 - RSS_b/q_b is the fraction of what is left after the ring and the drift
    that the drive term explains. That is the hold-out metric below.
    """

    def __init__(self, blocks, mi, dc="linear", ring=1):
        self.blocks, self.mi, self.dc = blocks, mi, dc
        self.ring = int(ring)
        self.ndc = {"const": 1, "linear": 2, "quad": 3}[dc]
        self.nl = 2 * self.ring + self.ndc
        self.n_used = sum(len(b.t) for b in blocks)

    def _local(self, blk, f0, f1):
        """The block's nuisance columns: the leftover ring, and the DC drift.

        `ring = 1` is the textbook free initial condition, two columns, and it
        is right only if nothing re-excites the mode inside the block. NOTHING
        ON THIS RIG SATISFIES THAT: the room drives these modes continuously --
        it is how Phi was measured passively at all -- so the leftover is a
        sinusoid at w_m whose amplitude and phase WANDER over 74 s. `ring = n`
        multiplies the two free-response columns by Chebyshev polynomials of
        order 0..n-1 in block time, which is a rank-2n approximation of exactly
        that wander.

        The driven response survives it because it is not merely a modulated
        sinusoid: on resonance it RAMPS, and at t = 36 s the unwind REVERSES it
        while the ambient keeps going. `selftest --ring-sweep` measures the
        trade -- too many envelope terms eventually eat the drive.

        DC drift is real and measured: ch3 moved +69.5 counts and ch4 +76.0
        between two clean records 37 minutes apart (CLAUDE.md Sec 13), so a
        single constant over a 74 s block is the floor, not the answer.
        """
        K = len(blk.t)
        x = np.linspace(-1.0, 1.0, K)
        cheb = [np.ones(K), x, 2 * x * x - 1.0,
                4 * x ** 3 - 3 * x, 8 * x ** 4 - 8 * x * x + 1.0]
        cols = []
        for P in cheb[:self.ring]:
            cols += [f0 * P, f1 * P]
        cols.append(np.ones(K))
        if self.ndc >= 2:
            cols.append(x)
        if self.ndc >= 3:
            cols.append(x * x - 1.0 / 3.0)
        return np.column_stack(cols)

    def pieces(self, w, gamma):
        """Per-block (M_b, r_b, q_b, GG_b, GL_b, LLi_b, Gy_b, Ly_b, yy_b)."""
        out = []
        for blk in self.blocks:
            tt = blk.t - blk.t[0]
            f0, f1 = free_columns(tt, w, gamma)
            L = self._local(blk, f0, f1)
            y = blk.z[:, self.mi]
            G = np.zeros((len(tt), NCOIL))
            if not blk.is_dc:            # see the module docstring: a DC step
                for j in blk.driven:     # response is exactly in span{1, f0}
                    G[:, j] = forced_q(blk.u[:, j], blk.dt, w, gamma)
            LL = L.T @ L
            LLi = np.linalg.pinv(LL, rcond=1e-12)
            GL = G.T @ L
            Gy, Ly = G.T @ y, L.T @ y
            GG = G.T @ G
            yy = float(y @ y)
            M = GG - GL @ LLi @ GL.T
            r = Gy - GL @ LLi @ Ly
            q = yy - float(Ly @ LLi @ Ly)
            out.append(dict(M=M, r=r, q=q, GL=GL, LLi=LLi, Ly=Ly, yy=yy,
                            n=len(tt), blk=blk))
        return out

    @staticmethod
    def solve(pieces, drop=(), ridge=0.0):
        M = np.zeros((NCOIL, NCOIL))
        r = np.zeros(NCOIL)
        for k, p in enumerate(pieces):
            if k in drop:
                continue
            M += p["M"]
            r += p["r"]
        d = np.diag(M).copy()
        live = d > d.max() * 1e-9 if d.max() > 0 else np.zeros(NCOIL, bool)
        A = np.zeros(NCOIL)
        if live.any():
            Ms = M[np.ix_(live, live)]
            if ridge > 0:
                Ms = Ms + ridge * np.eye(Ms.shape[0]) * np.trace(Ms) / Ms.shape[0]
            A[live] = np.linalg.solve(Ms, r[live])
        return A, M, r, live

    @staticmethod
    def rss_of(pieces, A, drop=()):
        s = 0.0
        for k, p in enumerate(pieces):
            if k in drop:
                continue
            s += p["q"] - 2.0 * float(A @ p["r"]) + float(A @ p["M"] @ A)
        return s

    def rss(self, w, gamma):
        p = self.pieces(w, gamma)
        A, _, _, _ = self.solve(p)
        return self.rss_of(p, A)


# ---------------------------------------------------------------------------
# nonlinear search. numpy only -- this repo's venv has no scipy, and a 2-D
# problem whose w-landscape is periodic wants a grid anyway, not a gradient.
# ---------------------------------------------------------------------------
def fit_mode(blocks, mi, f_seed, g_seed, dc="linear", ring=1,
             f_half=0.030, n_f=241, log=print):
    prob = ModeProblem(blocks, mi, dc=dc, ring=ring)

    def R(f, g):
        return prob.rss(2.0 * math.pi * f, g)

    # 1. coarse sweep in f at the seed gamma. Block coherence is 74 s, so the
    #    landscape has ~0.0135 Hz structure; 0.00025 Hz steps resolve it 54x.
    fs = np.linspace(f_seed - f_half, f_seed + f_half, n_f)
    rs = np.array([R(f, g_seed) for f in fs])
    f = float(fs[int(np.argmin(rs))])
    coarse = (fs, rs)
    # 2. alternate: gamma on a log grid, then f parabolic. Two rounds is enough
    #    -- the two parameters are nearly orthogonal (gamma sets an envelope, f
    #    sets a phase) and the third round moved f by < 1e-6 Hz on every pass.
    gs = np.exp(np.linspace(math.log(3e-4), math.log(0.35), 61))
    g = g_seed
    for _ in range(3):
        rg = np.array([R(f, x) for x in gs])
        g = float(gs[int(np.argmin(rg))])
        lo, hi = f - 0.004, f + 0.004
        for _ in range(28):                       # golden section, 1e-7 Hz
            a = hi - 0.618033988749895 * (hi - lo)
            b = lo + 0.618033988749895 * (hi - lo)
            if R(a, g) < R(b, g):
                hi = b
            else:
                lo = a
        f = 0.5 * (lo + hi)
        lo, hi = math.log(max(g * 0.3, 1e-4)), math.log(min(g * 3.0, 0.5))
        for _ in range(24):
            a = hi - 0.618033988749895 * (hi - lo)
            b = lo + 0.618033988749895 * (hi - lo)
            if R(f, math.exp(a)) < R(f, math.exp(b)):
                hi = b
            else:
                lo = a
        g = math.exp(0.5 * (lo + hi))
    w = 2.0 * math.pi * f
    p = prob.pieces(w, g)
    A, M, r, live = ModeProblem.solve(p)
    rss = ModeProblem.rss_of(p, A)
    ndat = sum(x["n"] for x in p)
    npar = NCOIL + len(p) * prob.nl
    dof = max(ndat - npar, 1)
    s2 = rss / dof
    cov = s2 * np.linalg.pinv(M, rcond=1e-12)
    # sigma on f and gamma from the curvature of RSS, Delta chi^2 = 1. The
    # samples are oversampled 10x from the raw stream and the noise is coloured,
    # so this is an OPTIMISTIC bar; the honest one is the leave-one-point-out
    # scatter reported alongside it.
    df = 2e-4
    sf = _curv_sigma(lambda x: R(x, g), f, df, rss, dof)
    dg = max(g * 0.05, 1e-4)
    sg = _curv_sigma(lambda x: R(f, x), g, dg, rss, dof)
    return dict(mi=mi, f=f, gamma=g, w=w, A=A, M=M, live=live, rss=rss,
                q=sum(x["q"] for x in p), ndat=ndat, dof=dof, sigma2=s2,
                A_sigma=np.sqrt(np.clip(np.diag(cov), 0, None)),
                f_sigma=sf, gamma_sigma=sg, pieces=p, prob=prob, coarse=coarse)


def _curv_sigma(fn, x0, h, rss0, dof):
    """1-sigma from the local parabola of RSS, scaled so Delta chi^2 = 1."""
    a, b = fn(x0 - h), fn(x0 + h)
    d2 = (a + b - 2.0 * rss0) / (h * h)
    if not np.isfinite(d2) or d2 <= 0:
        return float("nan")
    return float(math.sqrt(2.0 * (rss0 / dof) / d2))


# ===========================================================================
# the per-point lock-in, reimplemented on the same blocks
# ===========================================================================
def lockin_A(blocks, modes, log=print):
    """What `status.py` computes today, on exactly the data this module fits.

    H = lockin(z_m over the drive window) / lockin(u over the same window), i.e.
    `status._point` with the geometric projection of `status.coils_report`. It is
    reproduced here so the comparison is against the shipping estimator on the
    same rows rather than against a number copied out of a log.
    """
    out = np.full((NMODE, NCOIL), np.nan)
    for mi, (name, _) in enumerate(modes):
        for j in range(NCOIL):
            best = None
            for b in blocks:
                if b.is_dc or j not in b.driven:
                    continue
                if abs(b.freq - modes[mi][1]) > 0.02:
                    continue
                m = np.max(np.abs(b.u[:, j])) > 1e-4
                if not m:
                    continue
                dm = np.abs(b.u[:, j]) > 1e-5
                # the DRIVE window only: from the first modulated sample to the
                # point where the sign of the ramp reverses (the unwind).
                k = np.nonzero(dm)[0]
                k0 = k[0]
                k1 = k0 + int(round(st.DWELL_S / b.dt))
                k1 = min(k1, len(b.t) - 1)
                t = b.t[k0:k1]
                if len(t) < 64:
                    continue
                f = modes[mi][1]
                U = st.lockin(t, b.u[k0:k1, j], f)
                if abs(U) < 1e-7:
                    continue
                y = b.z[k0:k1, mi]
                H = st.lockin(t, y - y.mean(), f) / U
                best = H
            if best is not None:
                out[mi, j] = -best.imag       # status.save_modal's sign rule
    return out


def as_lockin(A_phys, w, gamma, seconds=None):
    """Physical A -> the V/V number `data/modal.json` carries, by simulation.

    `status._point` divides a lock-in of the response by a lock-in of the drive
    over a 30 s ramping window, so the ratio is NOT A/(2 gamma w): with tau > 138
    s against DWELL_S = 30 s the response never reaches steady state. This
    simulates the same window with the same estimator, which is the only honest
    way to put the two on one axis.
    """
    T = st.DWELL_S if seconds is None else seconds
    dt = 1.0 / FS_FIT
    n = int(T / dt)
    t = np.arange(n) * dt
    u = np.sin(w * t)
    q = forced_q(u, dt, w, gamma)
    f = w / (2.0 * math.pi)
    return float(-(st.lockin(t, q, f) / st.lockin(t, u, f)).imag) * A_phys


# ===========================================================================
# validation
# ===========================================================================
def leave_one_point_out(res, log=print):
    """Fit A without one driven block, then predict that block.

    M and r are SUMS over blocks, so dropping one is exact and free. For the
    held-out block the local parameters (initial condition, drift) are refitted
    -- they are nuisance and unknowable a priori -- and the number reported is

        skill = 1 - RSS(A from the others) / RSS(no drive term at all)

    i.e. the fraction of the block's post-ring, post-drift residual that a drive
    term predicted from OTHER points explains. A per-point lock-in cannot be
    given this test: it has exactly one measurement per entry and no redundancy,
    which is precisely how a pass in which every point was contaminated passed
    every check it had.
    """
    p = res["pieces"]
    rows = []
    for k, pk in enumerate(p):
        blk = pk["blk"]
        if blk.is_dc or not blk.driven:
            continue
        A_out, _, _, live = ModeProblem.solve(p, drop=(k,))
        rss = pk["q"] - 2.0 * float(A_out @ pk["r"]) + float(A_out @ pk["M"] @ A_out)
        skill = 1.0 - rss / pk["q"] if pk["q"] > 0 else float("nan")
        # what the in-sample fit gets on the same block, for reference
        rss_in = pk["q"] - 2.0 * float(res["A"] @ pk["r"]) \
            + float(res["A"] @ pk["M"] @ res["A"])
        rows.append(dict(k=k, coil=blk.coil, freq=blk.freq, dur=blk.dur,
                         skill=skill, skill_in=1.0 - rss_in / pk["q"],
                         A_out=A_out.copy()))
    return rows


def per_point_A(res):
    """A[m, j] from each single driven point on its own, one point per row.

    The joint fit's redundancy made visible: coil j is driven at three different
    frequencies, so its column is measured three times over. The scatter of
    those three IS the estimator's repeatability, measured offline, with no
    second pass and no bench time.
    """
    p = res["pieces"]
    rows = []
    for k, pk in enumerate(p):
        blk = pk["blk"]
        if blk.is_dc or not blk.driven:
            continue
        A1, M1, r1, live = ModeProblem.solve([pk])
        rows.append(dict(k=k, coil=blk.coil, freq=blk.freq,
                         A=A1.copy(), live=live.copy(),
                         info=float(np.diag(M1).max())))
    return rows


def dc_matrix(blocks):
    """d(counts_i)/d(bias_j) from the DC step blocks, and its Phi projection.

    A difference of two 20 s means. The ring is zero-mean about the new
    equilibrium so it cancels -- `status._dc_point`, same argument and the same
    0.8%-of-swing residual as `analysis/bias_sweep.py`. Shares NO estimator with
    the dynamic fit, which is what makes it a real check on the magnitudes.
    """
    per = {}
    for b in blocks:
        if not b.is_dc:
            continue
        # the block's own u tells which sign of the step it is
        j = b.coil
        s = float(np.median(b.u[:, j])) if j < NCOIL else 0.0
        if abs(s) < 1e-6:
            continue
        per.setdefault(j, {})[round(s, 4)] = (b.y.mean(axis=0) / st.COUNTS_TO_V,
                                              b.z.mean(axis=0))
    dcm = np.full((4, NCOIL), np.nan)
    dmod = np.full((4, NCOIL), np.nan)
    for j, d in per.items():
        ks = sorted(d)
        if len(ks) < 2:
            continue
        hi, lo = d[ks[-1]], d[ks[0]]
        dv = ks[-1] - ks[0]
        dcm[:, j] = (hi[0] - lo[0]) / dv
        dmod[:, j] = (hi[1] - lo[1]) / dv / st.COUNTS_TO_V
    return dcm, dmod


def residual_spectrum(res_by_mode, blocks, nfft_s=32.0):
    """Per-modal-coordinate residual PSD, averaged over blocks.

    Reported so that "what is left" is a measurement rather than an assumption.
    Warp is included and carries NO model at all: warp/rigid is 0.14-0.17 on
    ambient and peaks at the RIGID mode frequencies (`CLAUDE.md` Sec The
    geometry), so it lands here by construction.
    """
    acc, cnt, fr = None, 0, None
    for bi, blk in enumerate(blocks):
        K = len(blk.t)
        n = int(nfft_s / blk.dt)
        if K < n or n < 64:
            continue
        r = np.empty((K, 4))
        for mi in range(NMODE):
            res = res_by_mode[mi]
            pk = res["pieces"][bi]
            tt = blk.t - blk.t[0]
            f0, f1 = free_columns(tt, res["w"], res["gamma"])
            L = res["prob"]._local(blk, f0, f1)
            G = np.zeros((K, NCOIL))
            if not blk.is_dc:
                for j in blk.driven:
                    G[:, j] = forced_q(blk.u[:, j], blk.dt, res["w"], res["gamma"])
            y = blk.z[:, mi]
            l = pk["LLi"] @ (pk["Ly"] - pk["GL"].T @ res["A"])
            r[:, mi] = y - G @ res["A"] - L @ l
        # warp: no model, only the same nuisance columns, so that its residual
        # is comparable with the modal ones rather than inflated by its DC.
        yw = blk.z[:, 3]
        Xw = np.column_stack([np.ones(K), np.linspace(-1, 1, K)])
        r[:, 3] = yw - Xw @ np.linalg.lstsq(Xw, yw, rcond=None)[0]
        win = np.hanning(n)
        nseg = K // n
        for s in range(nseg):
            seg = r[s * n:(s + 1) * n] * win[:, None]
            P = np.abs(np.fft.rfft(seg, axis=0)) ** 2
            acc = P if acc is None else acc + P
            cnt += 1
            fr = np.fft.rfftfreq(n, blk.dt)
    if cnt == 0:
        return None, None
    return fr, acc / cnt


# ===========================================================================
# the whole pass
# ===========================================================================
def fit_pass(path, fs=FS_FIT, dc="linear", ring=1, use_cache=True, log=print,
             modes=None, max_blocks=None):
    modes = list(st.MODES) if modes is None else modes
    t0 = time.time()
    raw = load_raw(path, use_cache=use_cache)
    blocks = split_blocks(raw, fs=fs)
    if max_blocks:
        blocks = blocks[:max_blocks]
    log("\n=== %s" % os.path.basename(path))
    log("  %d raw rows -> %d blocks, %.0f s of record, grid %.0f Hz"
        % (len(raw["t"]), len(blocks),
           sum(b.dur for b in blocks), fs))
    nclip = sum(1 for b in blocks if b.clipped)
    nrail = sum(1 for b in blocks if b.railed > st.RAIL_FRAC_REJECT)
    log("  saturation: %d blocks with u at the %.2f-%.2f V clamp; %d blocks with"
        " >%.0f%% of a0-a3 at an ADC rail"
        % (nclip, st.VMIN, st.VMAX, nrail, 100 * st.RAIL_FRAC_REJECT))
    for b in blocks:
        log("    %s%s" % (b.label(),
                          "   << CLIPPED" if b.clipped else ""))
    out = dict(path=path, fs=fs, dc=dc, ring=ring, blocks=blocks, modes=modes)
    res = []
    for mi, (name, f_seed) in enumerate(modes):
        r = fit_mode(blocks, mi, f_seed, GAMMA_SEED.get(name, 0.01), dc=dc,
                     ring=ring, log=log)
        r["name"] = name
        r["f_seed"] = f_seed
        res.append(r)
        log("  mode %s  f %.5f +- %.5f Hz (seed %.5f, %+.5f)   gamma %.4f +- %.4f /s"
            "   R2 %.4f" % (name, r["f"], r["f_sigma"], f_seed, r["f"] - f_seed,
                            r["gamma"], r["gamma_sigma"],
                            1.0 - r["rss"] / r["q"] if r["q"] > 0 else float("nan")))
    out["res"] = res
    out["A"] = np.array([r["A"] for r in res])
    out["A_sigma"] = np.array([r["A_sigma"] for r in res])
    out["seconds"] = time.time() - t0
    return out


def report_pass(out, log=print, loo=True):
    res, blocks, modes = out["res"], out["blocks"], out["modes"]
    A = out["A"]
    log("\n  --- A[mode, coil], PHYSICAL, sensor-volt rad^2/s^2 per coil volt ---")
    log("        " + "".join("     coil %d" % j for j in range(NCOIL)))
    for mi, r in enumerate(res):
        log("    %s   %s" % (r["name"], " ".join("%+10.4f" % x for x in A[mi])))
    log("        " + "".join("      +-  " for _ in range(NCOIL)))
    for mi, r in enumerate(res):
        log("    %s   %s" % (r["name"],
                             " ".join("%10.4f" % x for x in r["A_sigma"])))

    # ---- 1. signs ----
    sg = np.sign(A)
    agree = int((sg == SIGNS_SETTLED).sum())
    log("\n  [1] SIGNS vs the settled table (three determinations, no shared"
        " estimator): %d of 12" % agree)
    for mi, r in enumerate(res):
        log("      %s  fit %s   settled %s   %s"
            % (r["name"],
               " ".join("%+d" % x for x in sg[mi]),
               " ".join("%+d" % x for x in SIGNS_SETTLED[mi]),
               "OK" if (sg[mi] == SIGNS_SETTLED[mi]).all() else "<-- DISAGREES"))

    # ---- 2. hold-out ----
    if loo:
        log("\n  [2] LEAVE-ONE-POINT-OUT. skill = 1 - RSS(A from the other"
            " points) / RSS(no drive term).")
        for mi, r in enumerate(res):
            rows = leave_one_point_out(r)
            if not rows:
                continue
            on = [x for x in rows if abs(x["freq"] - r["f"]) < 0.02]
            off = [x for x in rows if abs(x["freq"] - r["f"]) >= 0.02]
            log("      mode %s   ON-RESONANCE points (%d): held-out skill %s"
                % (r["name"], len(on),
                   " ".join("%+.3f" % x["skill"] for x in on)))
            log("               in-sample on the same points:            %s"
                % " ".join("%+.3f" % x["skill_in"] for x in on))
            if off:
                sk = np.array([x["skill_in"] for x in off])
                # An off-resonance point is not a weak measurement of A, it is
                # a NEGLIGIBLE one, and the factor is arithmetic rather than
                # empirical. Over T = DWELL_S from rest the resonant amplitude
                # is A U T/(4 w_m) and the off-resonant one A U/|w_m^2 - w_d^2|,
                # so the ratio is T |w_m^2 - w_d^2| / (4 w_m): 30.2 and 145 for
                # mode A driven at B and C, 22.0 and 83.6 for B, 63.2 and 50.0
                # for C, at status.MODES and DWELL_S = 30 s. So the hoped-for
                # 3x redundancy per coil column does not exist -- each A[m,j] is
                # still determined by ONE point, exactly as the lock-in's is.
                log("               OFF-resonance points (%d): in-sample skill"
                    " median %+.4f, max %+.4f -- a resonant point is 22-145x"
                    " larger in amplitude, so there is no usable"
                    " cross-frequency redundancy"
                    % (len(off), float(np.median(sk)), float(sk.max())))
            for x in on:
                log("         coil %d  drive f %.4f  %5.1f s   skill %+.3f"
                    "  (in-sample %+.3f)"
                    % (x["coil"], x["freq"], x["dur"], x["skill"], x["skill_in"]))

    # ---- redundancy: per-point A ----
    log("\n  [2b] A[m, j] from each single point, i.e. the estimator's own"
        " repeatability. The lock-in has one measurement per entry and cannot"
        " be given this test.")
    for mi, r in enumerate(res):
        rows = per_point_A(r)
        fm = r["f"]
        log("      mode %s   (* = the point driven ON this mode; the others are"
            " 0.27-0.66 Hz off resonance)" % r["name"])
        for j in range(NCOIL):
            vals, tags = [], []
            for x in rows:
                if not x["live"][j] or x["coil"] != j:
                    continue
                vals.append(x["A"][j])
                tags.append("*" if abs(x["freq"] - fm) < 0.02 else " ")
            if not vals:
                continue
            v = np.array(vals)
            same = "all same sign" if (np.sign(v) == np.sign(v[0])).all() \
                else "SIGNS DISAGREE"
            log("        coil %d  joint %+9.4f   per-point %s   |max|/|min| %6.2f"
                "   %s"
                % (j, r["A"][j],
                   " ".join("%+9.4f%s" % (x, t) for x, t in zip(v, tags)),
                   np.abs(v).max() / max(np.abs(v).min(), 1e-12), same))

    # ---- 3/6. the DC cross-check ----
    dcm, dmod = dc_matrix(blocks)
    if np.isfinite(dmod).any():
        log("\n  [6] STATIC CROSS-CHECK. A / w^2 is the DC gain; the DC step"
            " blocks measure it directly and share no estimator with the fit.")
        log("        coil        A/w^2 counts/V      DC step counts/V     ratio")
        fac, nsign = [], 0
        for mi, r in enumerate(res):
            for j in range(NCOIL):
                pred = A[mi, j] / (r["w"] ** 2) / st.COUNTS_TO_V
                meas = dmod[mi, j]
                if not np.isfinite(meas):
                    continue
                rt = pred / meas if abs(meas) > 1e-9 else float("nan")
                if np.isfinite(rt) and rt != 0:
                    fac.append(max(abs(rt), 1.0 / abs(rt)))
                    nsign += int(rt > 0)
                log("    %s  %d   %+16.3f %+20.3f %9.3f"
                    % (r["name"], j, pred, meas, rt))
        if fac:
            log("      SUMMARY  median |ratio| as a factor x%.2f, worst x%.2f,"
                " %d of %d entries agree in SIGN"
                % (float(np.median(fac)), max(fac), nsign, len(fac)))
            out["dc_factor_median"] = float(np.median(fac))
            out["dc_sign"] = nsign

    # ---- mode drift WITHIN the pass: first half vs second half ------------
    n_half = len(blocks) // 2
    log("\n  [drift] the same fit on the first and the second half of the pass."
        " A frequency that moves inside one pass is a limit on any coherent fit"
        " of it.")
    for mi, r in enumerate(res):
        try:
            r1 = fit_mode(blocks[:n_half], mi, r["f"], r["gamma"],
                          dc=out["dc"], ring=out["ring"], f_half=0.006, n_f=49)
            r2 = fit_mode(blocks[n_half:], mi, r["f"], r["gamma"],
                          dc=out["dc"], ring=out["ring"], f_half=0.006, n_f=49)
        except Exception as e:                     # pragma: no cover
            log("      mode %s  half fit failed: %s" % (r["name"], e))
            continue
        log("      %s   first half %.5f Hz   second half %.5f Hz   delta"
            " %+.5f Hz   (whole pass %.5f, its own sigma %.5f)"
            % (r["name"], r1["f"], r2["f"], r2["f"] - r1["f"], r["f"],
               r["f_sigma"]))

    # ---- 4. variance explained + residual spectrum ----
    log("\n  [4] VARIANCE EXPLAINED, per modal coordinate, over every block.")
    for mi, r in enumerate(res):
        log("      %s (%s)   R2 %.4f   rms residual %.5f V   (data rms of the"
            " de-ringed, de-drifted signal %.5f V)"
            % (r["name"], st.GEO_DOF[mi],
               1.0 - r["rss"] / r["q"] if r["q"] > 0 else float("nan"),
               math.sqrt(r["rss"] / r["ndat"]), math.sqrt(r["q"] / r["ndat"])))
    fr, P = residual_spectrum(res, blocks)
    if fr is not None:
        log("      residual PSD peaks (Hann, 32 s segments), per coordinate:")
        for c, nm in enumerate(["T1 (A)", "Z (B)", "T2 (C)", "WARP"]):
            band = (fr > 0.2) & (fr < 5.0)
            k = int(np.argmax(P[band, c]))
            tot = P[band, c].sum()
            log("        %-8s peak at %.4f Hz, %.1f%% of the 0.2-5 Hz residual"
                " power in that one bin"
                % (nm, fr[band][k],
                   100.0 * P[band, c][k] / tot if tot > 0 else 0.0))
        for mi, r in enumerate(res):
            band = np.abs(fr - r["f"]) < 0.05
            tot = ((fr > 0.2) & (fr < 5.0))
            log("        mode %s line leftover: %.1f%% of the 0.2-5 Hz residual"
                " power sits within 0.05 Hz of the fitted %.4f Hz"
                % (r["name"], 100.0 * P[band, mi].sum() / P[tot, mi].sum(),
                   r["f"]))

    # ---- 5. w, gamma ----
    log("\n  [5] FITTED w AND gamma. status.MODES and the plant number are"
        " printed next to them.")
    log("      mode      f Hz         sigma      MODES        delta"
        "      gamma /s     sigma     seed")
    for mi, r in enumerate(res):
        log("       %s    %.5f    %.5f   %.5f   %+8.5f    %.5f   %.5f   %.5f"
            % (r["name"], r["f"], r["f_sigma"], r["f_seed"],
               r["f"] - r["f_seed"], r["gamma"], r["gamma_sigma"],
               GAMMA_SEED.get(r["name"], float("nan"))))
    log("      the whole-plant number is 0.0072 +- 0.0023 /s and CANNOT EXCLUDE"
        " ZERO (analysis/ringdown.md lines 178, 441)")

    # ---- the shipping estimator on the same rows ----
    Hl = lockin_A(blocks, modes)
    Aj = np.array([[as_lockin(A[mi, j], res[mi]["w"], res[mi]["gamma"])
                    for j in range(NCOIL)] for mi in range(NMODE)])
    log("\n  --- against the SHIPPING per-point lock-in, same rows, same"
        " projection, converted to the same V/V convention ---")
    log("        coil      joint (as lock-in)      per-point lock-in      ratio")
    for mi, r in enumerate(res):
        for j in range(NCOIL):
            log("    %s  %d   %+18.4f %+22.4f %10.3f"
                % (r["name"], j, Aj[mi, j], Hl[mi, j],
                   Aj[mi, j] / Hl[mi, j] if abs(Hl[mi, j]) > 1e-12 else float("nan")))
    out["A_lockin_joint"] = Aj
    out["A_lockin_point"] = Hl
    out["dc_modal"] = dmod
    return out


# ===========================================================================
# selftest -- synthetic plant with a known A and a deliberately huge ring
# ===========================================================================
def selftest(log=print, seed=7, ring=1):
    log("=== selftest: exact-ZOH resonator, known A, deliberate leftover ring ===")
    rng = np.random.default_rng(seed)

    # 1. the discretisation itself. gamma = 0 must conserve energy EXACTLY over
    #    the full record length, which is the check CLAUDE.md asks for.
    dt = 1.0 / FS_FIT
    tt = np.arange(int(1900.0 / dt)) * dt        # a whole pass, 1900 s
    for f in (0.72194, 1.65607):
        w = 2 * math.pi * f
        f0, f1 = free_columns(tt, w, 0.0)
        e = f0 ** 2 + (w * f1) ** 2
        log("  energy at %.4f Hz over %.0f s: max |E-1| = %.3e"
            % (f, tt[-1], np.max(np.abs(e - 1.0))))
        assert np.max(np.abs(e - 1.0)) < 1e-9

    # 2. forced_q against a closed form: undamped, on-resonance, from rest,
    #    x(t) = (1/(2w)) (sin wt / w - t cos wt) for u = sin wt.
    #
    #    THE INPUT IS THE BIN AVERAGE, not the left-edge sample, and that is not
    #    a convenience: `split_blocks` averages every raw u_v that landed in a
    #    grid bin, which is the time-average of the 50 Hz ZOH the coil actually
    #    held. Feeding left-edge samples instead costs exactly a half-sample
    #    delay -- w*dt/2 = 0.0227 rad at 0.72194 Hz and 100 Hz, which is the
    #    2.26e-2 relative error it produces on a 120 s resonant ramp.
    w = 2 * math.pi * 0.72194
    t = np.arange(int(120.0 / dt)) * dt
    ubar = (np.cos(w * t) - np.cos(w * (t + dt))) / (w * dt)
    q = forced_q(ubar, dt, w, 0.0)
    exact = (np.sin(w * t) / w - t * np.cos(w * t)) / (2.0 * w)
    err = np.max(np.abs(q - exact)) / np.max(np.abs(exact))
    log("  forced_q vs the closed form, 120 s on resonance: max rel err %.3e" % err)
    assert err < 1e-3

    # 3. a synthetic pass, built to fail the way the real one does.
    #
    #    THE CONTAMINATION MECHANISM IS REPRODUCED, NOT PLANTED. The pass drives
    #    at `status.MODES` while the plant sits 0.0011-0.0018 Hz away, which is
    #    the rig's actual situation -- the modes moved 0.0170 Hz in 11 days and
    #    are stable only to ~0.001 Hz over hours (CLAUDE.md Sec Re-measure the
    #    mode frequencies). The anti-phase unwind is then no longer the exact
    #    reverse of the drive, so it leaves a ring, and the TRUE STATE IS CARRIED
    #    from each point into the next across a 0.3 s gap. Nothing sets the ring
    #    amplitude by hand; it is whatever the mistuned unwind leaves.
    A_true = np.array([[-3.1, +5.4, +9.2, +6.0],
                       [-4.9, +3.7, -2.4, -6.0],
                       [-2.2, -4.3, -4.9, +3.2]])
    f_true = np.array([0.72301, 0.99106, 1.65711])       # NOT status.MODES
    g_true = np.array([0.0120, 0.0060, 0.0180])
    w_true = 2 * math.pi * f_true
    f_cmd = np.array([f for _, f in st.MODES])           # what the pass drives
    log("  plant f  %s" % " ".join("%.5f" % x for x in f_true))
    log("  drive f  %s  (delta %s Hz)"
        % (" ".join("%.5f" % x for x in f_cmd),
           " ".join("%+.5f" % x for x in (f_cmd - f_true))))
    blocks = []
    x = np.zeros((NMODE, 2))
    ring_frac = []
    for j in range(NCOIL):
        for mi in range(NMODE):
            amp = [0.028, 0.020, 0.012][mi] * [1.0, 1.0, 0.5, 0.75][j]
            wc = 2 * math.pi * f_cmd[mi]
            n_pre, n_dr, n_q = int(6.0 / dt), int(30.0 / dt), int(8.0 / dt)
            K = n_pre + 2 * n_dr + n_q
            tb = np.arange(K) * dt
            # bin-averaged ZOH, matching what split_blocks hands the fit
            sbar = amp * (np.cos(wc * tb) - np.cos(wc * (tb + dt))) / (wc * dt)
            u = np.zeros((K, NCOIL))
            u[n_pre:n_pre + n_dr, j] = sbar[n_pre:n_pre + n_dr]
            u[n_pre + n_dr:n_pre + 2 * n_dr, j] = -sbar[n_pre + n_dr:n_pre + 2 * n_dr]
            Z = np.zeros((K, 4))
            for m2 in range(NMODE):
                # AMBIENT FORCING, which is the real source of the ring and is
                # OUTSIDE the model on purpose. The room drives these modes
                # continuously -- that is how Phi was measured passively at all
                # (status.py phi, 300 s of ambient) -- so the leftover is not a
                # free decay from one initial condition, it is re-excited the
                # whole time. Sized to the measured ambient modal rms, 24.3 /
                # 11.1 / 6.5 counts for T1 / Z / T2 (`status.park_ramped`
                # docstring). For white force of per-sample sd s_f,
                # Var(x) = s_f^2 dt / (4 gamma w^2), which fixes s_f.
                tgt = AMBIENT_COUNTS[m2] * st.COUNTS_TO_V
                s_f = math.sqrt(4.0 * g_true[m2] * w_true[m2] ** 2
                                * tgt ** 2 / dt)
                amb = rng.normal(0.0, s_f, K)
                qf, vf = forced_state(A_true[m2, j] * u[:, j] + amb, dt,
                                      w_true[m2], g_true[m2])
                f0, f1 = free_columns(tb, w_true[m2], g_true[m2])
                q = qf + x[m2, 0] * f0 + x[m2, 1] * f1
                Z[:, m2] = q
                v = vf + x[m2, 0] * (-w_true[m2] ** 2 * f1) \
                    + x[m2, 1] * (f0 - 2 * g_true[m2] * f1)
                x[m2] = propagate(np.array([q[-1], v[-1]]), 0.3,
                                  w_true[m2], g_true[m2])
                if m2 == mi:
                    dr = np.abs(q[n_pre:n_pre + n_dr]).max()
                    pr = np.abs(q[:n_pre]).max()
                    ring_frac.append(pr / max(dr, 1e-12))
            # warp: real, unexplained, and it peaks at the RIGID frequencies
            # (CLAUDE.md Sec The geometry), so it is generated FROM them.
            Z[:, 3] = 0.15 * Z[:, 0] - 0.10 * Z[:, 2]
            # per-sensor DC drift: ch3 moved +69.5 counts in 37 min between two
            # clean records = 0.031 counts/s (CLAUDE.md Sec 13)
            drift = 0.031 * st.COUNTS_TO_V * tb
            Z += drift[:, None] * rng.normal(0, 1, 4)[None, :]
            Z += rng.normal(0, 0.5 * st.COUNTS_TO_V, Z.shape)   # ADC dither
            blocks.append(Block(tb, u, Z, {PHASE_CODE["drive"]}, j,
                                f_cmd[mi], dt, K, False, 0.0))
    log("  synthetic pass: %d blocks, %.0f s" % (len(blocks),
                                                 sum(b.dur for b in blocks)))
    log("  RESULTING leftover ring, |pre|/|drive peak| in the DRIVEN mode's own")
    log("  coordinate: %s" % " ".join("%.0f%%" % (100 * v) for v in ring_frac))
    log("  (the 23:12 pass measured 3.5-417 %% by the same ratio)")

    res = []
    for mi in range(NMODE):
        r = fit_mode(blocks, mi, st.MODES[mi][1], GAMMA_SEED["ABC"[mi]], ring=ring)
        r["name"] = "ABC"[mi]
        res.append(r)
        log("  mode %s  f %.5f (true %.5f, err %+.5f Hz)   gamma %.4f (true %.4f)"
            % (r["name"], r["f"], f_true[mi], r["f"] - f_true[mi],
               r["gamma"], g_true[mi]))
    A_fit = np.array([r["A"] for r in res])
    log("\n  A recovery, joint fit:")
    log("      mode  coil     true        fit      err %")
    worst = 0.0
    for mi in range(NMODE):
        for j in range(NCOIL):
            e = 100.0 * (A_fit[mi, j] - A_true[mi, j]) / abs(A_true[mi, j])
            worst = max(worst, abs(e))
            log("        %s     %d   %+8.3f   %+8.3f   %+7.2f"
                % ("ABC"[mi], j, A_true[mi, j], A_fit[mi, j], e))
    err_j = np.abs(100.0 * (A_fit - A_true) / np.abs(A_true))
    log("  |error| median %.1f %%, rms %.1f %%, worst %.1f %%"
        % (np.median(err_j), math.sqrt(np.mean(err_j ** 2)), worst))

    Hl = lockin_A(blocks, [("A", f_true[0]), ("B", f_true[1]), ("C", f_true[2])])
    Aj = np.array([[as_lockin(A_true[mi, j], w_true[mi], g_true[mi])
                    for j in range(NCOIL)] for mi in range(NMODE)])
    log("\n  the SAME data through the per-point lock-in, converted to V/V:")
    log("      mode  coil   truth(as lock-in)   lock-in       err %   sign")
    lw = 0.0
    lsign = 0
    err_l = np.abs(100.0 * (Hl - Aj) / np.abs(Aj))
    for mi in range(NMODE):
        for j in range(NCOIL):
            e = 100.0 * (Hl[mi, j] - Aj[mi, j]) / abs(Aj[mi, j])
            lw = max(lw, abs(e))
            lsign += int(np.sign(Hl[mi, j]) == np.sign(Aj[mi, j]))
            log("        %s     %d   %+16.4f %+12.4f %+9.2f    %s"
                % ("ABC"[mi], j, Aj[mi, j], Hl[mi, j], e,
                   "ok" if np.sign(Hl[mi, j]) == np.sign(Aj[mi, j]) else "WRONG"))
    log("  lock-in |error| median %.1f %%, rms %.1f %%, worst %.1f %%,"
        " signs %d of 12" % (np.median(err_l), math.sqrt(np.mean(err_l ** 2)),
                             lw, lsign))
    jsign = int((np.sign(A_fit) == np.sign(A_true)).sum())
    log("  joint   |error| median %.1f %%, rms %.1f %%, worst %.1f %%,"
        " signs %d of 12" % (np.median(err_j), math.sqrt(np.mean(err_j ** 2)),
                             worst, jsign))
    return dict(worst_joint=worst, worst_lockin=lw,
                med_joint=float(np.median(err_j)), med_lockin=float(np.median(err_l)),
                rms_joint=float(math.sqrt(np.mean(err_j ** 2))),
                rms_lockin=float(math.sqrt(np.mean(err_l ** 2))),
                ring_frac=ring_frac,
                signs_joint=jsign, signs_lockin=lsign)




# ===========================================================================
# two checks that need no fit at all
# ===========================================================================
def dc_consistency(paths, log=print, use_cache=True):
    """The DC step matrix from every pass that has one, and its spread.

    WHY THIS IS THE STRONGER MAGNITUDE MEASUREMENT ON THIS RIG, and it is an
    SNR argument with numbers in it. A DC point holds the coil at +/-0.15 V --
    5 to 25x the amplitude the driven points used on the 23:12 pass (0.006 to
    0.028 V) -- and then averages 20 s. The ambient ring is a ~0.72-1.66 Hz
    sinusoid, so a 20 s mean suppresses it by ~1/(2 pi f T) = 1/90, where the
    driven lock-in suppresses it not at all: the ring is AT the frequency the
    lock-in reads. `status._dc_point` makes the same argument qualitatively;
    this puts the ratio on it.

    What limits it instead is DRIFT, and that is measured: the resting point
    moved +69.5 counts (ch3) and +76.0 (ch4) between two clean records 37 min
    apart (CLAUDE.md Sec 13), i.e. ~0.03 counts/s, so ~1.2 counts across a
    +/-20 s pair against a step of order 10 counts.
    """
    rows = []
    for q in paths:
        full = os.path.join(REPO, q) if not os.path.isabs(q) else q
        if not os.path.exists(full):
            continue
        raw = load_raw(full, use_cache=use_cache)
        blocks = split_blocks(raw)
        if not any(b.is_dc for b in blocks):
            continue
        dcm, dmod = dc_matrix(blocks)
        rows.append((os.path.basename(full)[:15], dcm, dmod))
    if not rows:
        log("  no pass in this list carries DC step blocks")
        return rows
    log("\n=== DC STEP MATRIX, every pass that has one ===")
    log("  d(counts_i)/d(coil volts), row = sensor, col = coil")
    for nm, dcm, _ in rows:
        log("    %s" % nm)
        for i in range(4):
            log("      a%d  %s" % (i, " ".join("%+8.2f" % x for x in dcm[i])))
    log("\n  the SAME thing projected on the geometric Phi, counts/V in the")
    log("  modal coordinate -- this is A/w^2 and it is what the dynamic fit must")
    log("  reproduce:")
    log("     mode coil  " + " ".join("%14s" % nm for nm, _, _ in rows)
        + "   |max|/|min|   signs")
    spreads, frac = [], []
    for mi in range(NMODE):
        for j in range(NCOIL):
            v = np.array([r[2][mi, j] for r in rows])
            g = v[np.isfinite(v)]
            rr = np.abs(g).max() / max(np.abs(g).min(), 1e-12) if len(g) > 1 \
                else float("nan")
            ok = "same" if len(g) > 1 and (np.sign(g) == np.sign(g[0])).all() \
                else ("FLIP" if len(g) > 1 else "-")
            if np.isfinite(rr):
                spreads.append(rr)
            log("      %s   %d  %s   %10.2f   %s   mean %+8.3f  sd %6.3f"
                " (%5.1f%%)"
                % ("ABC"[mi], j, " ".join("%+14.3f" % x for x in v), rr, ok,
                   g.mean(), g.std(ddof=1) if len(g) > 1 else float("nan"),
                   100.0 * g.std(ddof=1) / abs(g.mean()) if len(g) > 1 and
                   abs(g.mean()) > 0 else float("nan")))
            frac.append(100.0 * g.std(ddof=1) / abs(g.mean())
                        if len(g) > 1 and abs(g.mean()) > 0 else np.nan)
    if spreads:
        log("\n  worst entry spread x%.2f, median x%.2f over %d entries and %d"
            " passes" % (max(spreads), float(np.median(spreads)), len(spreads),
                         len(rows)))
        fa = np.array(frac, float)
        log("  per-entry sd/|mean|: median %.1f%%, worst %.1f%% -- a fairer"
            " statistic than max/min, which grows with the number of passes"
            % (np.nanmedian(fa), np.nanmax(fa)))
        log("  CLAUDE.md Sec Not established records the two DC passes as"
            " agreeing only to a factor 0.23-1.8, i.e. x1.8 worst at best; this"
            " is the same comparison over %d passes." % len(rows))
    log("\n  the settled sign table, for reference:")
    for mi in range(NMODE):
        log("      %s  %s" % ("ABC"[mi],
                              " ".join("%+d" % x for x in SIGNS_SETTLED[mi])))
    log("  signs of the mean DC projection:")
    M = np.nanmean(np.array([r[2] for r in rows]), axis=0)
    agree = 0
    for mi in range(NMODE):
        sg = np.sign(M[mi, :NCOIL])
        agree += int((sg == SIGNS_SETTLED[mi]).sum())
        log("      %s  %s   %s" % ("ABC"[mi], " ".join("%+d" % x for x in sg),
                                   "OK" if (sg == SIGNS_SETTLED[mi]).all()
                                   else "<-- DISAGREES"))
    log("  %d of 12" % agree)
    return rows


def headroom(paths, log=print, use_cache=True):
    """How much of the available drive each point actually used, and why it
    matters more than any estimator.

    Every point is a race between the driven response and the ambient ring, and
    the pass loses it: `status.choose_amp` sizes the amplitude from `DC_SEED`
    times `ramp_gain`, aiming at TARGET_SWING_COUNTS = 200. This measures what
    arrived. The ceiling is not AMP_MAX but the rail guard -- RAIL_GUARD_LOW /
    HIGH at 60 / 963 counts against resting points of 570-735 -- so the honest
    number is how far the nearest sensor got toward its own guard.
    """
    log("\n=== DRIVE HEADROOM: what each point spent, and what was available ===")
    for q in paths:
        full = os.path.join(REPO, q) if not os.path.isabs(q) else q
        if not os.path.exists(full):
            continue
        raw = load_raw(full, use_cache=use_cache)
        blocks = split_blocks(raw)
        log("\n  %s" % os.path.basename(full))
        log("    coil  drive Hz   amp V   AMP_MAX/amp   worst sensor swing"
            "   guard room   room/swing   driven/ambient")
        for b in blocks:
            if b.is_dc or not b.driven:
                continue
            j = b.driven[0]
            amp = float(np.max(np.abs(b.u[:, j])))
            if amp < 1e-5:
                continue
            c = b.y / st.COUNTS_TO_V
            rest = b.rest[:4]
            hi = c.max(axis=0) - rest
            lo = rest - c.min(axis=0)
            swing = np.maximum(hi, lo)
            room = np.minimum(st.RAIL_GUARD_HIGH - rest, rest - st.RAIL_GUARD_LOW)
            k = int(np.argmin(room / np.maximum(swing, 1e-9)))
            # driven vs ambient: the modal peak in the driven mode's own
            # coordinate against the rms of the 6 s pre window, same coordinate
            mi = int(np.argmin([abs(b.freq - f) for _, f in st.MODES]))
            n_pre = int(6.0 / b.dt)
            zz = b.z[:, mi] / st.COUNTS_TO_V
            amb = float(np.std(zz[:n_pre]))
            drv = float(np.abs(zz - zz.mean()).max())
            log("     %d    %.4f   %.4f   %8.1fx   a%d %7.1f counts %10.1f"
                " %10.1fx %12.2f"
                % (j, b.freq, amp, st.AMP_MAX / amp, k, swing[k], room[k],
                   room[k] / max(swing[k], 1e-9), drv / max(amb, 1e-9)))
    return None

# ===========================================================================
PASSES = [
    "data/20260817_182701_status_coils.csv",
    "data/20260817_194726_status_coils.csv",
    "data/20260817_231251_status_coils.csv",
    "data/20260817_235950_status_coils.csv",
    "data/20260818_011840_status_coils.csv",
    "data/20260817_172411_status_coils.csv",
]


def cross_pass(paths, fs=FS_FIT, dc="linear", ring=1, log=print):
    outs = []
    for p in paths:
        full = os.path.join(REPO, p) if not os.path.isabs(p) else p
        if not os.path.exists(full):
            log("  (missing) %s" % p)
            continue
        o = fit_pass(full, fs=fs, dc=dc, ring=ring, log=log)
        report_pass(o, log=log)
        outs.append(o)
    if len(outs) < 2:
        return outs
    log("\n\n=== [3] CROSS-PASS CONSISTENCY ===")
    log("  Each pass is an independent estimate of the same A. The reference to"
        " beat is the two DC passes, which reproduce individual entries only to"
        " a factor 0.23-1.8 (CLAUDE.md Sec Not established).")
    names = [os.path.basename(o["path"])[:15] for o in outs]
    log("\n  A/w^2 in counts/V -- the STATIC gain, which is pass-independent"
        " whatever each pass's drive amplitudes were.")
    log("    mode coil  " + " ".join("%12s" % n for n in names) + "   max/min")
    ratios = []
    for mi in range(NMODE):
        for j in range(NCOIL):
            v = []
            for o in outs:
                w = o["res"][mi]["w"]
                v.append(o["A"][mi, j] / (w * w) / st.COUNTS_TO_V)
            v = np.array(v)
            good = v[np.isfinite(v) & (np.abs(v) > 0)]
            rr = (np.abs(good).max() / np.abs(good).min()) if len(good) > 1 \
                else float("nan")
            if np.isfinite(rr):
                ratios.append(rr)
            log("     %s   %d   %s   %8.2f"
                % ("ABC"[mi], j, " ".join("%+12.3f" % x for x in v), rr))
    if ratios:
        log("\n  worst entry spread x%.2f, median x%.2f over %d entries"
            % (max(ratios), float(np.median(ratios)), len(ratios)))
    log("\n  fitted frequencies per pass:")
    for mi in range(NMODE):
        log("    mode %s  %s" % ("ABC"[mi],
                                 "  ".join("%.5f" % o["res"][mi]["f"] for o in outs)))
    log("    status.MODES  " + "  ".join("%.5f" % f for _, f in st.MODES))
    log("\n  fitted gamma per pass:")
    for mi in range(NMODE):
        log("    mode %s  %s" % ("ABC"[mi],
                                 "  ".join("%.4f" % o["res"][mi]["gamma"] for o in outs)))
    return outs




def pool(jsonpath, log=print, use_cache=True):
    """One A from ALL the driven data at once, at each pass's own w and gamma.

    M and r are sums over blocks and the blocks of different passes are
    independent, so pooling is exact: sum every pass's Schur-reduced normal
    equations and solve once. The per-pass frequencies differ -- they are
    measured to drift -- so each pass contributes its blocks evaluated at ITS
    OWN (w, gamma), read from the json `all` wrote.

    This is the best A the driven records can give, and its only purpose here is
    to be compared against the static DC determination, which shares no
    estimator with it. A disagreement is either the driven data's noise or a
    real difference between the coils' DC and 0.7-1.7 Hz response -- and
    `status.a_from_dc`'s docstring names that second possibility as the reason
    a DC-derived A is labelled provisional. Nothing here settles which.
    """
    with open(jsonpath) as fh:
        js = json.load(fh)
    log("\n\n=== POOLED A: every driven block of every pass, one solve ===")
    Msum = [np.zeros((NCOIL, NCOIL)) for _ in range(NMODE)]
    rsum = [np.zeros(NCOIL) for _ in range(NMODE)]
    wbar = np.zeros(NMODE)
    nb = 0
    for e in js:
        path = e["path"]
        if not os.path.exists(path):
            continue
        raw = load_raw(path, use_cache=use_cache)
        blocks = split_blocks(raw, fs=e["fs"])
        nb += len(blocks)
        for mi in range(NMODE):
            prob = ModeProblem(blocks, mi, dc=e["dc"], ring=e.get("ring", 1))
            w = 2.0 * math.pi * e["f"][mi]
            for pc in prob.pieces(w, e["gamma"][mi]):
                Msum[mi] += pc["M"]
                rsum[mi] += pc["r"]
            wbar[mi] += w
    wbar /= max(len(js), 1)
    A = np.zeros((NMODE, NCOIL))
    for mi in range(NMODE):
        A[mi], _, _, _ = ModeProblem.solve(
            [dict(M=Msum[mi], r=rsum[mi], q=0.0)])
    log("  %d passes, %d blocks" % (len(js), nb))
    log("  A pooled, physical:")
    for mi in range(NMODE):
        log("    %s  %s" % ("ABC"[mi], " ".join("%+10.4f" % x for x in A[mi])))
    sg = np.sign(A)
    log("  signs vs the settled table: %d of 12"
        % int((sg == SIGNS_SETTLED).sum()))
    for mi in range(NMODE):
        log("    %s  %s   %s" % ("ABC"[mi], " ".join("%+d" % x for x in sg[mi]),
                                 "OK" if (sg[mi] == SIGNS_SETTLED[mi]).all()
                                 else "<-- DISAGREES"))
    log("  physical A in counts (divide by status.COUNTS_TO_V) -> A/w^2:")
    for mi in range(NMODE):
        log("    %s  %s" % ("ABC"[mi], " ".join(
            "%+10.3f" % (A[mi, j] / (wbar[mi] ** 2) / st.COUNTS_TO_V)
            for j in range(NCOIL))))
    log("  compare with the DC step projection printed by `joint_a.py dc`.")
    return A, wbar


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("what", nargs="?", default="selftest",
                    choices=["selftest", "fit", "all", "dc", "headroom", "pool"])
    ap.add_argument("path", nargs="?", default=None)
    ap.add_argument("--fs", type=float, default=FS_FIT)
    ap.add_argument("--dc", default="linear", choices=["const", "linear", "quad"])
    ap.add_argument("--ring", type=int, default=1,
                    help="order of the leftover-ring envelope, 1 = a plain free "
                         "initial condition. See ModeProblem._local")
    ap.add_argument("--ring-sweep", action="store_true",
                    help="selftest only: recover A at ring = 1..4 and report")
    ap.add_argument("--no-cache", action="store_true")
    ap.add_argument("--out", default=None, help="write the fit to this json")
    a = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)   # CLAUDE.md standing practice
    except Exception:
        pass
    if a.what == "selftest":
        if a.ring_sweep:
            print("  seed  ring |    joint: med / rms / worst / signs  |"
                  "  lock-in: med / rms / worst / signs  | ring %")
            for sd in (7, 11, 23, 41, 97):
                for n in (1,):
                    r = selftest(ring=n, seed=sd, log=lambda *x: None)
                    print("  %4d  %3d  | %6.1f %6.1f %7.1f  %2d/12   |"
                          " %6.1f %6.1f %7.1f  %2d/12  | %3.0f-%3.0f"
                          % (sd, n, r["med_joint"], r["rms_joint"],
                             r["worst_joint"], r["signs_joint"],
                             r["med_lockin"], r["rms_lockin"],
                             r["worst_lockin"], r["signs_lockin"],
                             100 * min(r["ring_frac"]),
                             100 * max(r["ring_frac"])), flush=True)
            return
        selftest(ring=a.ring)
        return
    if a.what == "fit":
        if not a.path:
            sys.exit("fit needs a path")
        o = fit_pass(a.path, fs=a.fs, dc=a.dc, ring=a.ring,
                     use_cache=not a.no_cache)
        report_pass(o)
        outs = [o]
    elif a.what == "pool":
        if not a.path:
            sys.exit("pool needs the json written by `all --out`")
        pool(a.path, use_cache=not a.no_cache)
        return
    elif a.what == "dc":
        dc_consistency([a.path] if a.path else PASSES,
                       use_cache=not a.no_cache)
        return
    elif a.what == "headroom":
        headroom([a.path] if a.path else PASSES, use_cache=not a.no_cache)
        return
    else:
        outs = cross_pass(PASSES, fs=a.fs, dc=a.dc, ring=a.ring)
    if a.out:
        js = []
        for o in outs:
            js.append(dict(
                path=o["path"], fs=o["fs"], dc=o["dc"],
                ring=o["ring"],
                A=o["A"].tolist(), A_sigma=o["A_sigma"].tolist(),
                A_lockin_joint=o.get("A_lockin_joint",
                                     np.zeros((3, 4))).tolist(),
                A_lockin_point=np.nan_to_num(
                    o.get("A_lockin_point", np.zeros((3, 4)))).tolist(),
                f=[r["f"] for r in o["res"]],
                f_sigma=[r["f_sigma"] for r in o["res"]],
                gamma=[r["gamma"] for r in o["res"]],
                gamma_sigma=[r["gamma_sigma"] for r in o["res"]],
                r2=[1.0 - r["rss"] / r["q"] for r in o["res"]]))
        with open(a.out, "w") as fh:
            json.dump(js, fh, indent=1)
        print("\n  -> %s" % a.out)


if __name__ == "__main__":
    main()
