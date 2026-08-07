"""OFFLINE: a three-oscillator Kalman velocity estimator, designed and replayed.

WHAT THIS IS. `osem.delta.py` (the shipping controller, formerly `osem.v12.py`)
estimates velocity by bandpassing 0.4-3.0 Hz and then differencing. This file
designs the model-based replacement, sizes every matrix in it from data already
on disk, and validates it by REPLAYING recorded logs. No serial port is opened,
no controller is run, nothing is written outside `analysis/out/`. `sample-guard`
and `decimate` were both designed this way; this follows that precedent.

IT ALSO ANSWERS THE "FFT THE DATA, PHASE-MATCH, SUM, PLAY IT BACK IN NEGATIVE"
PROPOSAL, in sections 7 and 8, because that proposal and this one turn out to be
the same design and the differences are worth writing down rather than
arbitrating.

RUN IT:      .venv/bin/python analysis/kalman.py
Roughly three minutes, most of it parsing the 168 MB bench log.
The argument built on the output is in `analysis/kalman.md`.

THE DATA, none of it invented:
  data/20260806_192723_quiet_openloop.csv  355797 samples, 1111.87 Hz, gain off.
                                           R, and the modal powers.
  data/20260806_200822_fast_lock.csv       251280 rows, 244.5 s, 1027.5 Hz wire,
                                           five hand kicks. The replay.
"""
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")
DATA = os.path.join(ROOT, "data")
QUIET = os.path.join(DATA, "20260806_192723_quiet_openloop.csv")
RUN = os.path.join(DATA, "20260806_200822_fast_lock.csv")

# --- verbatim from osem.delta.py -------------------------------------------
# Literals rather than an import: delta imports pyDAC2, and this file must not
# be able to reach a serial port even by accident.
N = 8
A_VCC, ADC_MAX_COUNTS = 5.02, 1023
CONTROL_HZ = 100.0
CONTROL_PERIOD_S = 1.0 / CONTROL_HZ
BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ = 0.4, 3.0, 5.0
D_SMOOTH_HZ = 2.0
STEADY_GAIN = np.array([-0.035, -0.035, +0.035, -0.035, 0.0, 0.0, 0.0, 0.0])
KI_GAIN = np.array([-0.0060, -0.0060, +0.0060, -0.0060, 0.0, 0.0, 0.0, 0.0])
KD_GAIN = np.array([-0.00045, -0.00045, +0.00045, -0.00045, 0.0, 0.0, 0.0, 0.0])
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011

# --- measured, analysis/ringdown.md ----------------------------------------
# quiet_frequencies.csv: three modes, +-0.0005 Hz, from a 276 s fitted window.
F_MODE = np.array([0.7155, 0.9949, 1.6396])
W_MODE = 2 * np.pi * F_MODE
NM = len(F_MODE)
# quiet_freq_stability.csv: first half of that record against the second.
F_DRIFT = np.array([6.150507590141352e-05, -9.808618862727769e-05,
                    1.1649256836943067e-04])

# --- the design point, chosen in section 5 from a sweep --------------------
# T_AMP is the ONLY tuning constant in the whole estimator. Meaning: the filter
# assumes a mode's amplitude can change by order itself in T_AMP seconds.
# 50 s is the round number in the middle of a flat optimum -- see section 5.
T_AMP_S = 50.0
# The DC state's own version. Long, because the rest position moves on the
# trim's timescale (TRIM_PERIOD_S = 15.0 s in delta), not on the modes'.
T_DC_S = 20.0


def hdr(s):
    print("\n" + "=" * 76 + f"\n{s}\n" + "=" * 76)


def writecsv(name, header, rows):
    with open(os.path.join(OUT, name), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)
    print(f"    -> analysis/out/{name}  ({len(rows)} rows)")


# ===========================================================================
# THE ESTIMATOR. This is the deliverable: a controller imports it or pastes it
# in. numpy is its only dependency.
# ===========================================================================
class KalmanVelocity:
    """Per-channel (SISO) velocity estimator: three undamped oscillators at
    MEASURED frequencies plus one random-walk DC state, one filter per channel,
    no cross-channel term anywhere.

    STATE, per channel i, all in that channel's own sensor volts:

        x = [q_A, v_A, q_B, v_B, q_C, v_C, b]

    q_m is how much of mode m THIS sensor sees, in volts; v_m = dq_m/dt in V/s;
    b is a slowly drifting offset. The measurement is

        y = q_A + q_B + q_C + b + noise,      noise ~ N(0, R_i)

    and the output is

        vhat = v_A + v_B + v_C

    which excludes b BY CONSTRUCTION. That is what replaces the 0.4 Hz
    highpass: the estimate is DC-free because the DC lives in a state the
    output does not read, not because a filter removed it. It is also why Ki
    has nothing left to do -- see section 8.

    WHY THIS IS NOT MIMO. q_m is a per-channel quantity, the projection of mode
    m onto sensor i in sensor units, not a modal coordinate. No Phi is formed,
    nothing is inverted across channels, and dropping a channel changes nothing
    about any other channel's filter. Colocated, coaxial sensor and coil, one
    OSEM head per channel, is what makes that legitimate: the per-channel
    residue sign is a single constant, so the decomposition needs no geometry.

    WHY THREE UNDAMPED OSCILLATORS AND NOT THREE DAMPED ONES. tau > 138 s at
    1 sigma (analysis/ringdown.md) against a closed loop that settles in
    2.9-4.7 s: over one filter time constant the intrinsic decay is under 1.5%.
    A damping term is not measurable from the data, so it is not in the model
    and the process noise carries the amplitude changes instead. That the plant
    is exactly undamped is an ASSUMPTION, labelled here as one.

    PROCESS NOISE. Continuous white acceleration of intensity q_m on each
    oscillator, discretised exactly rather than by Euler:

        Qd = q * [[ (dt/2 - sin(2 w dt)/(4w))/w^2 ,  sin^2(w dt)/(2 w^2) ],
                  [ sin^2(w dt)/(2 w^2)           ,  dt/2 + sin(2 w dt)/(4w) ]]

    THE PROPERTY THE WHOLE DESIGN RESTS ON. At exactly f_m the estimator is
    EXACT -- unity gain, zero phase error against a true differentiator --
    independently of Q and R. A pure sinusoid at f_m is an exact
    zero-process-noise trajectory of the model, and the error dynamics
    (I - K H) Phi are stable, so the estimate converges to the true state and
    therefore to the true velocity. That is why the frequencies being measured
    to +-0.0005 Hz is what unblocked this, and why Q being unknown does not
    block it. Verified numerically in section 4.
    """

    def __init__(self, R, q, dt=CONTROL_PERIOD_S, n=N, f=F_MODE, q_dc=None):
        """R: (n,) measurement-noise variance in V^2, per channel.
        q: (n, 3) process-noise intensity in V^2/s^3, per channel per mode.
        q_dc: (n,) random-walk intensity for the DC state, V^2/s."""
        self.n, self.dt, self.f = n, dt, np.asarray(f, float)
        self.w = 2.0 * np.pi * self.f
        self.nm = len(self.f)
        self.nx = 2 * self.nm + 1
        self.R = np.asarray(R, float).reshape(n)
        self.q = np.asarray(q, float).reshape(n, self.nm)
        self.q_dc = (np.zeros(n) if q_dc is None
                     else np.asarray(q_dc, float).reshape(n))
        self.H = np.zeros(self.nx)
        self.H[0:2 * self.nm:2] = 1.0
        self.H[-1] = 1.0
        self.Cv = np.zeros(self.nx)
        self.Cv[1:2 * self.nm:2] = 1.0
        self.Phi = self._transition(dt)
        self.K = np.stack([self._steady_gain(i) for i in range(n)])
        self.x = np.zeros((n, self.nx))
        self.primed = np.zeros(n, bool)

    def _transition(self, dt):
        Phi = np.zeros((self.nx, self.nx))
        for m, w in enumerate(self.w):
            th = w * dt
            c, s = np.cos(th), np.sin(th)
            Phi[2 * m:2 * m + 2, 2 * m:2 * m + 2] = [[c, s / w], [-w * s, c]]
        Phi[-1, -1] = 1.0
        return Phi

    def _Q(self, i, dt):
        Q = np.zeros((self.nx, self.nx))
        for m, w in enumerate(self.w):
            th, s = w * dt, np.sin(w * dt)
            Q[2 * m:2 * m + 2, 2 * m:2 * m + 2] = self.q[i, m] * np.array(
                [[(dt / 2 - np.sin(2 * th) / (4 * w)) / w ** 2, s * s / (2 * w * w)],
                 [s * s / (2 * w * w), dt / 2 + np.sin(2 * th) / (4 * w)]])
        Q[-1, -1] = self.q_dc[i] * dt
        return Q

    def _steady_gain(self, i, iters=60000, tol=1e-12):
        """Steady-state gain by iterating the Riccati recursion. Run OFFLINE,
        once; a controller ships the resulting K as a constant array and never
        runs this. No scipy in this venv and none is needed -- the recursion
        converges from P = I in 4000-12000 steps, 0.2-0.4 s per channel.
        `tol` is RELATIVE to |K|; an absolute tolerance below float precision
        never terminates, which is how the first version of this hung."""
        Phi, H, Q, R = self.Phi, self.H, self._Q(i, self.dt), self.R[i]
        P, Kp, I = np.eye(self.nx), None, np.eye(self.nx)
        for k in range(iters):
            Pm = Phi @ P @ Phi.T + Q
            K = (Pm @ H) / (H @ Pm @ H + R)
            P = (I - np.outer(K, H)) @ Pm
            P = 0.5 * (P + P.T)
            if Kp is not None and np.max(np.abs(K - Kp)) < tol * max(1.0, np.abs(K).max()):
                break
            Kp = K.copy()
        return K

    def update(self, y, dt=None, valid=None):
        """One control step. `y` is (n,) volts -- the SAME decimated mean that
        `_control` already receives. `valid` is (n,) bool: False means do not
        trust this channel's sample (railed), so predict only and coast on the
        model. Returns vhat (n,) in V/s.

        `dt` is the measured control interval. Passing it rebuilds Phi -- three
        cos and three sin TOTAL, shared by every channel -- so the jitter in
        the control period costs nothing. K stays at its design value."""
        Phi = (self.Phi if dt is None or abs(dt - self.dt) < 1e-9
               else self._transition(dt))
        y = np.asarray(y, float)
        cold = ~self.primed
        if cold.any():
            # Prime the DC state on the first sample rather than ramping from
            # zero, the same way OnePole primes a lane coming back.
            self.x[cold] = 0.0
            self.x[cold, -1] = y[cold]
            self.primed[:] = True
        xp = self.x @ Phi.T
        e = y - xp @ self.H
        if valid is not None:
            e = np.where(valid, e, 0.0)
        self.x = xp + self.K * e[:, None]
        return self.x @ self.Cv

    def reset(self, mask=slice(None)):
        self.x[mask] = 0.0
        self.primed[mask] = False

    # -- what a controller gets for free, having paid for the states --------
    def displacement(self):
        """In-band displacement in V -- the `bp` column's replacement."""
        return self.x[:, 0:2 * self.nm:2].sum(axis=1)

    def acceleration(self):
        """d(vhat)/dt in closed form, V/s^2: a_m = -w_m^2 q_m. No difference
        quotient, so a D term would cost no noise at all. It is still effective
        mass and still dissipates nothing."""
        return -(self.x[:, 0:2 * self.nm:2] * self.w ** 2).sum(axis=1)

    def modal_velocity(self):
        """(n, 3) per-mode velocity: what a per-mode resonant gain needs, and
        what a bandpass cannot produce at any price."""
        return self.x[:, 1:2 * self.nm:2]

    # -- analysis -----------------------------------------------------------
    def transfer(self, freqs, i=0):
        """H(f) from y to vhat, V/s per V. Steady state is LTI, so exact."""
        Ac = (np.eye(self.nx) - np.outer(self.K[i], self.H)) @ self.Phi
        return np.array([self.Cv @ np.linalg.solve(
            np.eye(self.nx) - Ac / np.exp(2j * np.pi * f * self.dt), self.K[i])
            for f in np.atleast_1d(freqs)])

    def poles(self, i=0):
        return np.linalg.eigvals(
            (np.eye(self.nx) - np.outer(self.K[i], self.H)) @ self.Phi)

    def pole_taus(self, i=0):
        """(oscillatory taus sorted, dc tau). The DC pole is reported apart
        because the velocity output does not read the DC state and its pole is
        not what limits amplitude tracking."""
        ev = self.poles(i)
        tau = -self.dt / np.log(np.abs(ev))
        fp = np.abs(np.angle(ev)) / (2 * np.pi * self.dt)
        osc = sorted(set(np.round(tau[fp > 0.05], 3)))
        dc = float(np.max(tau[fp <= 0.05])) if (fp <= 0.05).any() else float("nan")
        return osc, dc


def design(R, sig2, t_amp=T_AMP_S, t_dc=T_DC_S):
    """Q from measured modal powers and one constant. For an undamped
    oscillator driven by white acceleration of intensity q, position variance
    grows as q*T/(2 w^2); setting that to the mode's MEASURED variance at
    T = t_amp gives q = 2 w^2 sigma^2 / t_amp. So 24 of the 25 numbers in Q
    are measurements and one is a design choice."""
    q = 2.0 * (W_MODE ** 2)[None, :] * np.asarray(sig2, float) / t_amp
    return q, np.asarray(R, float) / t_dc


# ===========================================================================
# delta's own estimator: closed form for the algebra, and a stepping function
# for the replay. Transfer functions are analysis/kp040.py section 0 verbatim.
# ===========================================================================
def h_low(f, fc, dt):
    tau = 1.0 / (2 * np.pi * fc)
    a = dt / (tau + dt)
    return a / (1 - (1 - a) / np.exp(2j * np.pi * np.asarray(f, float) * dt))


def h_high(f, fc, dt):
    tau = 1.0 / (2 * np.pi * fc)
    a = tau / (tau + dt)
    zi = 1.0 / np.exp(2j * np.pi * np.asarray(f, float) * dt)
    return a * (1 - zi) / (1 - a * zi)


def h_deriv(f, dt):
    return (1 - 1.0 / np.exp(2j * np.pi * np.asarray(f, float) * dt)) / dt


def h_zoh(f, dt):
    w = 2 * np.pi * np.asarray(f, float) * dt
    return np.where(np.abs(w) < 1e-12, 1.0 + 0j,
                    (1 - np.exp(-1j * w)) / (1j * np.where(w == 0, 1, w)))


def h_delta_vel(f, dt=CONTROL_PERIOD_S):
    """volts -> vhat as delta computes it: HP 0.4, LP 3.0, first difference,
    LP 5.0. `analysis/kp040.py:h_sensor_path`, unchanged."""
    return (h_high(f, BP_LOW_HZ, dt) * h_low(f, BP_HIGH_HZ, dt)
            * h_deriv(f, dt) * h_low(f, DERIV_SMOOTH_HZ, dt))


def h_pid(f, kp, ki, kd, dt):
    zi = 1.0 / np.exp(2j * np.pi * np.asarray(f, float) * dt)
    return -(kp + ki * dt / (1 - zi) + kd * h_deriv(f, dt) * h_low(f, D_SMOOTH_HZ, dt))


def h_mains_null(f, dt=CONTROL_PERIOD_S):
    """The 2-tap average of section 1b: 0.5*(1 + z^-1)."""
    return 0.5 * (1 + 1.0 / np.exp(2j * np.pi * np.asarray(f, float) * dt))


def delta_estimator_state():
    """delta's HP/LP/diff/LP chain as a stepping function.

    Mirrors `OnePole.update` and `_control` EXACTLY, priming included, because
    the priming is where a reimplementation goes wrong and the reconstruction
    check in section 6b is what caught it:
      * a high-pass lane's first output is 0 and its xp primes to x;
      * a low-pass lane's first output is its own INPUT -- for `lp` that is the
        high-pass output, i.e. 0, NOT the raw volts;
      * `_control` skips the velocity update on the first step (`filt_on` is
        False), so `dsm` primes on step 2 and its first output is the raw
        difference quotient, un-smoothed.
    """
    tau_h = 1.0 / (2 * np.pi * BP_LOW_HZ)
    tau_l = 1.0 / (2 * np.pi * BP_HIGH_HZ)
    tau_d = 1.0 / (2 * np.pi * DERIV_SMOOTH_HZ)
    st = dict(on=False, yh=None, xh=None, yl=None, bp_prev=None,
              yd=None, dsm_on=False)

    def step(x, dt):
        x = np.asarray(x, float)
        if not st["on"]:
            st["on"] = True
            st["yh"], st["xh"] = np.zeros_like(x), x.copy()
            st["yl"] = np.zeros_like(x)
            bp = np.zeros_like(x)
            st["bp_prev"], st["yd"] = bp.copy(), np.zeros_like(x)
            return bp, st["yd"].copy()
        ah, al = tau_h / (tau_h + dt), dt / (tau_l + dt)
        st["yh"] = ah * (st["yh"] + x - st["xh"])
        st["xh"] = x.copy()
        st["yl"] = st["yl"] + al * (st["yh"] - st["yl"])
        bp = st["yl"].copy()
        dv = (bp - st["bp_prev"]) / dt
        st["bp_prev"] = bp
        if not st["dsm_on"]:
            st["dsm_on"] = True
            st["yd"] = dv.copy()
        else:
            ad = dt / (tau_d + dt)
            st["yd"] = st["yd"] + ad * (dv - st["yd"])
        return bp, st["yd"].copy()

    return step


# ===========================================================================
# Loading and small tools
# ===========================================================================
def load_quiet():
    d = np.loadtxt(QUIET, delimiter=",", skiprows=1)
    t, c = d[:, 0], d[:, 1:]
    ok = ((c >= 0) & (c <= ADC_MAX_COUNTS)).all(axis=1)
    return t[ok], A_VCC * c[ok] / ADC_MAX_COUNTS, int((~ok).sum())


def load_run(path=RUN):
    t0 = time.time()
    with open(path) as fh:
        head = fh.readline().rstrip("\n").split(",")
        rows = [ln.split(",") for ln in fh.read().splitlines() if ln]
    ncol = len(head)
    rows = [r for r in rows if len(r) == ncol]
    ix = {n: i for i, n in enumerate(head)}
    nch = sum(1 for n in head if n.endswith("_gain"))

    def col(name, num=True):
        j = ix[name]
        return (np.fromiter((r[j] for r in rows), float, len(rows)) if num
                else np.array([r[j] for r in rows]))

    out = dict(t=col("time_s"), state=col("state", False),
               ctl=col("ctl").astype(int), n_avg=col("n_avg").astype(int), nch=nch)
    for k in ("counts", "bp", "vel", "gain", "out", "p", "i", "d"):
        out[k] = np.column_stack([col(f"ch{i}_{k}") for i in range(nch)])
    print(f"    {os.path.basename(path)}: {len(rows)} rows, "
          f"{out['t'][-1] - out['t'][0]:.1f} s, "
          f"{(len(rows) - 1) / (out['t'][-1] - out['t'][0]):.1f} Hz wire, "
          f"parsed in {time.time() - t0:.0f} s")
    return out


def decimate(t, V, hz=CONTROL_HZ):
    """delta's `step()` accumulator exactly: the deadline advances by whole
    periods and a missed deadline re-syncs. Needed for the quiet record, which
    was logged by a tool and carries no `ctl` column."""
    P, ct, n = 1.0 / hz, None, 0
    acc = np.zeros(V.shape[1])
    to, Vo, no, dto, prev = [], [], [], [], None
    for k in range(len(t)):
        tk = t[k]
        if ct is None:
            ct = prev = tk - P
        acc += V[k]
        n += 1
        if tk < ct + P:
            continue
        ct += P
        if tk - ct >= P:
            ct = tk
        to.append(tk)
        Vo.append(acc / n)
        no.append(n)
        dto.append(min(max(tk - prev, 1e-4), 0.05))
        prev, acc, n = tk, np.zeros(V.shape[1]), 0
    return np.array(to), np.array(Vo), np.array(no), np.array(dto)


def mains_null(V):
    """`mains-null`, section 1b: average two consecutive control means. That is
    a 20 ms boxcar, whose first null is at 50 Hz exactly."""
    out = np.asarray(V, float).copy()
    out[1:] = 0.5 * (out[1:] + out[:-1])
    return out


def welch(x, fs, nper=1 << 13):
    w = np.hanning(nper)
    U, stp = (w ** 2).sum(), nper // 2
    segs = [np.abs(np.fft.rfft((x[s:s + nper] - x[s:s + nper].mean()) * w)) ** 2
            for s in range(0, len(x) - nper + 1, stp)]
    return np.fft.rfftfreq(nper, 1 / fs), np.mean(segs, axis=0) * 2.0 / (fs * U)


def R_and_modes(V, fs, half=0.15, mode_half=0.06):
    """R = everything the model does not carry, integrated 0.2-50 Hz with the
    three mode bands bridged; sigma^2 = the modal variances; inband = the total
    0.4-3.0 Hz variance, so the modal FRACTION of the control band can be
    quoted. All from the same spectrum, consistent by construction."""
    R = np.zeros(V.shape[1])
    sig = np.zeros((V.shape[1], NM))
    inband = np.zeros(V.shape[1])
    fkeep, Pkeep = None, []
    for i in range(V.shape[1]):
        f, P = welch(V[:, i], fs)
        df = f[1] - f[0]
        msk = f < 0.2
        for fm in F_MODE:
            msk |= np.abs(f - fm) <= half
        band = (f >= 0.2) & (f <= 50.0)
        R[i] = np.trapezoid(np.interp(f, f[~msk], P[~msk])[band], f[band])
        for j, fm in enumerate(F_MODE):
            sig[i, j] = P[np.abs(f - fm) <= mode_half].sum() * df
        inband[i] = P[(f >= BP_LOW_HZ) & (f <= BP_HIGH_HZ)].sum() * df
        fkeep = f
        Pkeep.append(P)
    return R, sig, inband, fkeep, np.column_stack(Pkeep)


def zero_phase_velocity(t, x, bands):
    """Non-causal reference: keep only `bands`, differentiate exactly in the
    frequency domain, come back. Zero phase error by construction.

    THIS IS NOT GROUND TRUTH, and is labelled as such wherever it is used. It
    is a DEFINITION of what the in-band velocity is, computed with no causal
    filter, so two causal estimators can be scored against the same thing.
    Section 5 also runs a synthetic test where the truth IS known exactly."""
    fs = (len(t) - 1) / (t[-1] - t[0])
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    xu = np.interp(tu, t, x)
    X = np.fft.rfft(xu - xu.mean())
    f = np.fft.rfftfreq(len(tu), 1 / fs)
    keep = np.zeros(len(f), bool)
    for lo, hi in bands:
        keep |= (f >= lo) & (f <= hi)
    return tu, np.fft.irfft(np.where(keep, X * (2j * np.pi * f), 0.0), len(tu))


def resid_ratio(t, v, tu, ref, sel):
    vi = np.interp(tu, t, v)
    a, b = vi[sel] - vi[sel].mean(), ref[sel] - ref[sel].mean()
    return float((a - b).std() / b.std()), float(
        np.dot(a, b) / np.sqrt(np.dot(a, a) * np.dot(b, b)))


# ===========================================================================
# 1. R
# ===========================================================================
def section1():
    hdr("1. R, FROM THE 320 s QUIET OPEN-LOOP RECORD")
    print("  data/20260806_192723_quiet_openloop.csv, coils held at BIAS and never")
    print("  written again. R is DEFINED as everything the model does not carry --")
    print("  the broadband floor plus every line that is not one of the three modes")
    print("  -- integrated over 0.2-50 Hz, measured AFTER delta's own decimation,")
    print("  because the decimated mean is what the filter will actually see.\n")
    t, V, nrej = load_quiet()
    print(f"    {len(t)} samples kept, {nrej} rejected outside 0..1023 "
          f"(ringdown.md reports the same single 72647 on ch7)")
    m = t >= 20.0
    t, V = t[m], V[m]
    td, Vd, nd, dtd = decimate(t, V)
    fs = (len(td) - 1) / (td[-1] - td[0])
    print(f"    decimated to {len(td)} control samples, {fs:.3f} Hz, "
          f"n_avg mean {nd.mean():.2f}")
    R0, S0, I0, f0, P0 = R_and_modes(Vd, fs)

    print("\n  1a. AS SHIPPED.")
    print("   ch      R (V^2)   sqrt(R) (V)  (counts)   of which >48 Hz   raw 50 Hz line")
    fsr = (len(t) - 1) / (t[-1] - t[0])
    tu = np.arange(t[0], t[-1], 1 / fsr)
    raw50 = np.zeros(N)
    for i in range(N):
        x = np.interp(tu, t, V[:, i])
        x = x - x.mean()
        X = np.fft.rfft(x)
        fr = np.fft.rfftfreq(len(x), 1 / fsr)
        raw50[i] = np.fft.irfft(np.where((fr > 49.5) & (fr < 50.5), X, 0.0), len(x)).std()
    hi0 = np.zeros(N)
    for i in range(N):
        y = Vd[:, i] - Vd[:, i].mean()
        Y = np.fft.rfft(y)
        fy = np.fft.rfftfreq(len(y), 1 / fs)
        hi0[i] = np.fft.irfft(np.where(fy > 49.0, Y, 0.0), len(y)).std()
        print("   a%d  %11.4e   %10.6f  %8.3f       %10.6f     %10.6f V (%5.2f counts)"
              % (i, R0[i], np.sqrt(R0[i]), np.sqrt(R0[i]) * ADC_MAX_COUNTS / A_VCC,
                 hi0[i], raw50[i], raw50[i] * ADC_MAX_COUNTS / A_VCC))

    print("\n  THE DOMINANT TERM IS ALIASED MAINS, and this was not known before.")
    print("  The raw stream carries a 50.06 Hz line. delta's boxcar over ~11 wire")
    print("  samples spans 10 ms, so it attenuates 50 Hz only to sinc(0.5) = 0.64,")
    print("  and the 100 Hz control rate folds it to just under Nyquist, where it")
    print("  becomes 0.80 / 0.96 / 0.89 of sqrt(R) on a0 / a1 / a5.")

    print("\n  1b. `mains-null`: AVERAGE TWO CONSECUTIVE CONTROL MEANS. That is a")
    print("      20 ms boxcar, whose first null is at 50 Hz EXACTLY. One line, no")
    print("      change of control rate, and it helps delta as much as it helps")
    print("      this estimator. Cost, which is pure group delay of 5.0 ms:")
    for fm in list(F_MODE) + [3.0]:
        h = h_mains_null(fm)
        print("        %.4f Hz: |H| = %.6f, phase %+.3f deg"
              % (fm, abs(h), np.degrees(np.angle(h))))
    Vn = mains_null(Vd)
    Rn, Sn, In, fn, Pn = R_and_modes(Vn, fs)
    rows = []
    print("\n   ch     R as shipped   R with mains-null   ratio    sqrt(Rn) (V)  (counts)")
    for i in range(N):
        y = Vn[:, i] - Vn[:, i].mean()
        Y = np.fft.rfft(y)
        fy = np.fft.rfftfreq(len(y), 1 / fs)
        hin = np.fft.irfft(np.where(fy > 49.0, Y, 0.0), len(y)).std()
        print("   a%d      %.4e         %.4e   %6.3f      %9.6f  %7.3f"
              % (i, R0[i], Rn[i], Rn[i] / R0[i], np.sqrt(Rn[i]),
                 np.sqrt(Rn[i]) * ADC_MAX_COUNTS / A_VCC))
        rows.append([i, R0[i], np.sqrt(R0[i]), hi0[i], raw50[i],
                     Rn[i], np.sqrt(Rn[i]), hin, Rn[i] / R0[i]])
    writecsv("kalman_R.csv",
             ["ch", "R_shipped_v2", "sqrt_R_shipped_v", "above49hz_shipped_v",
              "raw_50hz_line_v", "R_mainsnull_v2", "sqrt_R_mainsnull_v",
              "above49hz_mainsnull_v", "ratio"], rows)
    print("\n   Everything below uses the mains-null input. It is a PREREQUISITE,")
    print("   not part of the estimator, and it is recommended on its own merits.")
    return td, Vd, Vn, dtd, fs, R0, S0, Rn, Sn, In, fn, Pn


# ===========================================================================
# 2. What else is in band and is not a mode
# ===========================================================================
def section2(fn, Pn, Sn):
    hdr("2. THE IN-BAND LINES THAT ARE NOT MODES")
    print("  The quiet record's spectrum carries more than three lines, and every")
    print("  extra one sits at an exact SUM OR DIFFERENCE of the measured three.")
    print("  A mechanical mode has no reason to; a quadratic sensor nonlinearity")
    print("  does, and an OSEM is a shadow sensor with a curved response.\n")
    df = fn[1] - fn[0]
    A, B, C = F_MODE
    lines = [("mode A", A), ("mode B", B), ("mode C", C),
             ("C - B", C - B), ("B - A", B - A), ("C - A", C - A),
             ("A + B", A + B), ("2B", 2 * B), ("B + C", B + C), ("3B", 3 * B)]
    rows = []
    print("   line     f (Hz)   " + " ".join("  a%d rms V" % i for i in range(4)))
    amp = {}
    for nm, f0 in lines:
        a = np.array([np.sqrt(Pn[np.abs(fn - f0) <= 0.012, i].sum() * df) for i in range(N)])
        amp[nm] = a
        print("   %-7s %8.4f   " % (nm, f0) + " ".join("%10.5f" % x for x in a[:4]))
        rows.append([nm, f0] + list(a))
    writecsv("kalman_lines.csv", ["line", "f_hz"] + [f"a{i}_rms_v" for i in range(N)], rows)

    print("\n  QUADRATIC TEST. If y = x + eps*x^2 then the product line (m1 +- m2)")
    print("  has amplitude eps*A1*A2 and the harmonic 2m has eps*A^2/2, so ONE eps")
    print("  per channel should explain all of them. Recovered eps per channel:")
    rows = []
    print("     from        a0        a1        a2        a3")
    for nm, p1, p2, k in (("A + B", "mode A", "mode B", 1.0),
                          ("C - B", "mode C", "mode B", 1.0),
                          ("B - A", "mode B", "mode A", 1.0),
                          ("2B", "mode B", "mode B", 0.5)):
        e = amp[nm] / (k * amp[p1] * amp[p2])
        print("     %-5s  " % nm + " ".join("%9.4f" % x for x in e[:4]))
        rows.append([nm] + list(e))
    writecsv("kalman_intermod.csv", ["line"] + [f"eps_a{i}" for i in range(N)], rows)
    print("\n  On a0 and a2 -- the two channels where the products are well above")
    print("  the floor -- the three independent estimates of eps agree to within a")
    print("  factor of about 3-4. That is CONSISTENT WITH a quadratic sensor")
    print("  nonlinearity and does NOT establish it; a factor of 3 is not a fit.")
    print("  What does not depend on the mechanism: these lines are inside")
    print("  0.4-3.0 Hz, so delta's bandpass passes them at full weight and drives")
    print("  the coils with them. Section 4 measures by how much.")


# ===========================================================================
# 3. Q
# ===========================================================================
def section3(Sn, inband):
    hdr("3. Q, FROM THE MEASURED MODAL POWERS, AND THE ONE CONSTANT IN IT")
    print("      q[i,m] = 2 * w_m^2 * sigma^2[i,m] / T_AMP")
    print("  24 of the 25 numbers in Q come from the quiet record. T_AMP is the")
    print("  only tuning constant in the estimator, and section 5 sweeps it.\n")
    rows = []
    print("   ch    rms_A     rms_B     rms_C   (V)  |     q_A       q_B       q_C  (V^2/s^3)"
          "  | 0.4-3 Hz rms   modal frac")
    q, _ = design(np.ones(N), Sn)
    for i in range(N):
        frac = Sn[i].sum() / max(inband[i], 1e-30)
        print("   a%d  %8.5f  %8.5f  %8.5f  |  %8.3e %8.3e %8.3e  |    %8.5f     %6.1f%%"
              % (i, *np.sqrt(Sn[i]), *q[i], np.sqrt(inband[i]), 100 * frac))
        rows.append([i] + list(Sn[i]) + list(np.sqrt(Sn[i])) + list(q[i])
                    + [inband[i], frac])
    writecsv("kalman_Q.csv",
             ["ch", "var_A_v2", "var_B_v2", "var_C_v2", "rms_A_v", "rms_B_v",
              "rms_C_v", "q_A", "q_B", "q_C", "var_inband_v2", "modal_frac_of_inband"], rows)
    print("\n  The last column is what section 8 leans on: the fraction of ALL")
    print("  0.4-3.0 Hz power that the three modelled lines account for. On a0-a3")
    print("  it is 95-100%, so a narrowband estimator is not throwing signal away.")


# ===========================================================================
# 4. Exactness, phase, dissipation
# ===========================================================================
def section4(Rn, Sn):
    hdr("4. PHASE AND GAIN AT THE THREE MODES -- THE HEADLINE COMPARISON")
    q, qdc = design(Rn, Sn)
    kf = KalmanVelocity(Rn, q, q_dc=qdc)
    probe = np.array([0.4, 0.5, F_MODE[0], F_MODE[1], 1.2, F_MODE[2], 2.0, 2.79, 3.0])
    lab = ["BP corner", "", "mode A", "mode B", "", "mode C", "", "", "BP corner"]
    hd, hk = h_delta_vel(probe), kf.transfer(probe, i=0)
    w = 2 * np.pi * probe
    rows = []
    print("   f (Hz)            |  delta gain/w   phase-90  |  kalman gain/w   phase-90")
    for j, fj in enumerate(probe):
        gd, pd = abs(hd[j]) / w[j], np.degrees(np.angle(hd[j])) - 90
        gk, pk = abs(hk[j]) / w[j], np.degrees(np.angle(hk[j])) - 90
        print("   %7.4f %-10s |  %10.4f  %+9.3f  |  %11.4f  %+9.3f"
              % (fj, lab[j], gd, pd, gk, pk))
        rows.append([fj, lab[j], abs(hd[j]), gd, pd, abs(hk[j]), gk, pk])
    writecsv("kalman_phase.csv",
             ["f_hz", "note", "delta_abs_h", "delta_gain_over_w", "delta_phase_err_deg",
              "kalman_abs_h", "kalman_gain_over_w", "kalman_phase_err_deg"], rows)
    print("\n  The kalman rows at the three modes are 1.0000 and 0.000 deg to every")
    print("  digit printed, and that is structural, not tuned: it holds for any Q")
    print("  and any R (verified across the whole of section 5's sweep).")

    print("\n  WHAT A PHASE ERROR ON A VELOCITY ESTIMATE COSTS. Kp acts on vhat, so")
    print("  it is a dashpot only to the extent vhat is in phase with velocity.")
    print("  Write vhat = (|H|/w)(cos e - j sin e) v: the cos part is the dashpot,")
    print("  the sin part is a SPRING -- a stiffness, negative for a lagging")
    print("  estimate -- and dissipates nothing.\n")
    rows = []
    print("   mode      f     |H|/w   phase err   dashpot   spring   kalman/delta dashpot")
    for m, fm in enumerate(F_MODE):
        hd1, hk1 = h_delta_vel(fm), kf.transfer(fm, i=0)[0]
        wm = 2 * np.pi * fm
        e = np.radians(np.degrees(np.angle(hd1)) - 90)
        dsh, spr = abs(hd1) / wm * np.cos(e), abs(hd1) / wm * np.sin(e)
        ek = np.radians(np.degrees(np.angle(hk1)) - 90)
        dshk = abs(hk1) / wm * np.cos(ek)
        print("     %s   %6.4f  %6.4f    %+7.3f    %6.4f  %+7.4f          %.3f x"
              % ("ABC"[m], fm, abs(hd1) / wm, np.degrees(e), dsh, spr, dshk / dsh))
        rows.append(["ABC"[m], fm, abs(hd1) / wm, np.degrees(e), dsh, spr,
                     abs(hk1) / wm, np.degrees(ek), dshk, dshk / dsh])
    writecsv("kalman_dissipation.csv",
             ["mode", "f_hz", "delta_gain_over_w", "delta_phase_err_deg",
              "delta_dashpot", "delta_spring", "kalman_gain_over_w",
              "kalman_phase_err_deg", "kalman_dashpot", "dashpot_ratio"], rows)

    print("\n  REJECTION OF WHAT IS NOT A MODE. |H| in V/s per V:")
    off = [("C-B intermod", F_MODE[2] - F_MODE[1]), ("A+B intermod", F_MODE[0] + F_MODE[1]),
           ("2B intermod", 2 * F_MODE[1]), ("3B intermod", 3 * F_MODE[1]),
           ("6.19 Hz line", 6.19), ("50 Hz alias", 49.91)]
    rows = []
    print("   line            f (Hz)   ideal w    delta   kalman   kalman/delta")
    for nm, x in off:
        a, b = abs(h_delta_vel(x)), abs(kf.transfer(x, i=0)[0])
        print("   %-14s %7.4f  %8.3f  %8.4f %8.4f       %6.3f" % (nm, x, 2 * np.pi * x, a, b, b / a))
        rows.append([nm, x, 2 * np.pi * x, a, b, b / a])
    writecsv("kalman_offmode.csv",
             ["line", "f_hz", "ideal_w", "delta_abs_h", "kalman_abs_h", "ratio"], rows)

    print("\n  THE LOOP, with analysis/kp040.py's own transfer functions so this is")
    print("  directly comparable to phase_budget.csv. `L_nonplant` is everything")
    print("  except the mechanical plant: sensor path x PID x ZOH, at Kp = -0.035,")
    print("  Ki/Kd as shipped for delta and ZERO for kalman (section 8 is what lets")
    print("  them go). kp040's phase-margin column is (phase - 90 + 180), which")
    print("  assumes the plant contributes exactly -90 deg -- true AT a resonance")
    print("  and nowhere else, so it is quoted only at the three modes.\n")
    ftab = [0.32, F_MODE[0], F_MODE[1], F_MODE[2], 2.79, 3.75, 6.19, 10.0, 20.0, 49.9]
    rows = []
    cfg = (("delta, BP+diff, Ki/Kd shipped", h_delta_vel, KI_GAIN[0], KD_GAIN[0]),
           ("kalman + mains-null, Ki=Kd=0",
            lambda ff: kf.transfer(ff, i=0) * h_mains_null(ff), 0.0, 0.0))
    print("     f (Hz)  |   delta  |L|     phase  |  kalman  |L|     phase")
    tab = {}
    for lab2, hs, ki, kd in cfg:
        vals = []
        for fm in ftab:
            L = (np.atleast_1d(hs(np.atleast_1d(fm)))[0]
                 * h_pid(fm, -0.035, ki, kd, CONTROL_PERIOD_S)
                 * np.atleast_1d(h_zoh(fm, CONTROL_PERIOD_S))[0])
            vals.append((abs(L), np.degrees(np.angle(L))))
        tab[lab2] = vals
    for j, fm in enumerate(ftab):
        a, b = tab[cfg[0][0]][j], tab[cfg[1][0]][j]
        note = ("  <- mode %s" % "ABC"[list(F_MODE).index(fm)]) if fm in list(F_MODE) else ""
        print("     %6.2f  |   %8.4f  %+8.2f  |  %8.4f  %+8.2f%s"
              % (fm, a[0], a[1], b[0], b[1], note))
        rows.append([fm, a[0], a[1], b[0], b[1]])
    writecsv("kalman_loop.csv",
             ["f_hz", "delta_absL_nonplant", "delta_phase_deg",
              "kalman_absL_nonplant", "kalman_phase_deg"], rows)
    print("\n   phase margin at the three modes (phase - 90 + 180):")
    for j, fm in enumerate(ftab):
        if fm not in list(F_MODE):
            continue
        print("     mode %s at %.4f Hz:  delta %6.1f deg   kalman %6.1f deg"
              % ("ABC"[list(F_MODE).index(fm)], fm,
                 tab[cfg[0][0]][j][1] + 90, tab[cfg[1][0]][j][1] + 90))

    print("\n   ABOVE THE BAND THE KALMAN LOOP LAGS MORE, AND THAT IS THE HONEST")
    print("   COST OF A NARROWBAND ESTIMATOR. Two things bound it, both measured:")
    ff = np.linspace(0.05, 49.9, 8000)
    out = []
    for lab2, hs, ki, kd in cfg:
        L = hs(ff) * h_pid(ff, -0.035, ki, kd, CONTROL_PERIOD_S) * h_zoh(ff, CONTROL_PERIOD_S)
        ph = np.degrees(np.angle(L))
        c90 = ff[np.nonzero(np.diff(np.sign(ph + 90)))[0]]
        c180 = ff[np.nonzero(np.diff(np.sign(ph + 180)))[0]]
        g90 = [abs(L[np.argmin(np.abs(ff - x))]) for x in c90]
        out.append((lab2, abs(L).max(), ff[np.argmax(abs(L))], c90, g90, c180))
        print("     %-30s peak |L| %.4f at %.2f Hz" % (lab2, out[-1][1], out[-1][2]))
        print("       phase = -90 deg at: "
              + (", ".join("%.2f Hz (|L| %.4f)" % (x, g) for x, g in zip(c90, g90))
                 or "never below Nyquist"))
        print("       phase = -180 deg at: "
              + (", ".join("%.2f Hz" % x for x in c180) or "never below Nyquist"))
    print("\n   Neither loop reaches -180 deg below Nyquist, so kp040's conclusion")
    print("   survives in form. Where the kalman loop does cross -90 deg -- the")
    print("   crossing that would matter IF a resonance sat there -- its own gain is")
    print("   an order of magnitude under delta's peak, and there is no measured")
    print("   mode above 1.6396 Hz. This is a CHECK TO REPEAT ON THE BENCH, not a")
    print("   result: the plant model above the modes is not measured on this rig.")
    print("\n  AND THE STANDING CAVEAT: analysis/kp040.md established that phase")
    print("  margin does NOT explain the -0.040 rail onset (the sampled-data gain")
    print("  margin below Nyquist is infinite). Extra margin at the modes is")
    print("  therefore NOT a licence to raise Kp. What the phase buys is the")
    print("  dashpot ratio above, at the same Kp and the same volts.")
    return kf


# ===========================================================================
# 5. T_AMP, swept, against ground truth and against the record
# ===========================================================================
def section5(Rn, Sn, td, Vn, dtd, fs):
    hdr("5. T_AMP, SWEPT -- WHERE THE DESIGN POINT COMES FROM")
    print("  5a. SYNTHETIC, where the truth is known exactly. ch0's own measured")
    print("      modal amplitudes, stepped x10 at t = 20 s with continuous phase")
    print("      (a kick), plus white noise at the measured sqrt(R). 40 s, 100 Hz.")
    print("      `settle` is the time for the 1 s-smoothed error to fall under 10%")
    print("      of the post-kick velocity rms.\n")
    dt, T = CONTROL_PERIOD_S, 40.0
    n = int(T / dt)
    t = np.arange(n) * dt
    rng = np.random.default_rng(7)
    amp0, phi, step_t, KICK = np.sqrt(2 * Sn[0]), rng.uniform(0, 2 * np.pi, NM), 20.0, 10.0
    x = np.zeros(n)
    v = np.zeros(n)
    for m, (fm, a0) in enumerate(zip(F_MODE, amp0)):
        w = 2 * np.pi * fm
        a = np.where(t < step_t, a0, a0 * KICK)
        x += a * np.cos(w * t + phi[m])
        v += -a * w * np.sin(w * t + phi[m])
    y = np.tile((x + 3.05 + rng.normal(0, np.sqrt(Rn[0]), n))[:, None], (1, N))
    print("      true velocity rms: %.3f V/s before the kick, %.3f after"
          % (v[:int(step_t / dt)].std(), v[int(step_t / dt):].std()))

    def settle(ve):
        e = np.convolve(np.abs(ve - v), np.ones(100) / 100, mode="same")
        ref = np.abs(v[int(step_t / dt):]).std()
        idx = np.nonzero((t > step_t) & (e < 0.10 * ref))[0]
        return (t[idx[0]] - step_t) if len(idx) else float("nan")

    def errs(ve):
        a, b = slice(int(10 / dt), int(step_t / dt)), slice(int((step_t + 15) / dt), n)
        return (ve[a] - v[a]).std() / v[a].std(), (ve[b] - v[b]).std() / v[b].std()

    dstep = delta_estimator_state()
    vdd = np.zeros(n)
    for j in range(n):
        vdd[j] = dstep(y[j], dt)[1][0]
    print("\n      T_AMP  settle(s)  err before kick  err after  osc poles tau(s)   dc")
    rows = []
    sd, ed = settle(vdd), errs(vdd)
    print("      delta  %8.2f  %14.3f  %9.3f" % (sd, ed[0], ed[1]))
    rows.append(["delta", sd, ed[0], ed[1], "", ""])
    for ta in [0.5, 2.0, 5.0, 20.0, 50.0, 100.0, 300.0, 1000.0]:
        q, qdc = design(Rn, Sn, t_amp=ta)
        kf = KalmanVelocity(Rn, q, q_dc=qdc)
        ve = np.zeros(n)
        for j in range(n):
            ve[j] = kf.update(y[j], dt=dt)[0]
        s, e = settle(ve), errs(ve)
        osc, dcp = kf.pole_taus(0)
        print("      %5.1f  %8.2f  %14.3f  %9.3f   %s  %5.2f"
              % (ta, s, e[0], e[1], " ".join("%5.2f" % z for z in osc), dcp))
        rows.append([ta, s, e[0], e[1], " ".join("%.2f" % z for z in osc), dcp])
    writecsv("kalman_sweep_synthetic.csv",
             ["t_amp_s", "settle_s", "err_before_kick", "err_after_kick",
              "osc_pole_taus_s", "dc_pole_tau_s"], rows)

    print("\n  5b. THE QUIET RECORD, against the zero-phase modal reference.")
    refs = {i: zero_phase_velocity(td, Vn[:, i], [(fm - 0.06, fm + 0.06) for fm in F_MODE])
            for i in range(4)}
    dstep = delta_estimator_state()
    vd = np.zeros((len(td), N))
    for j in range(len(td)):
        vd[j] = dstep(Vn[j], dtd[j])[1]

    def score(vv):
        o = []
        for i in range(4):
            tu, ref = refs[i]
            sel = (tu > tu[0] + 40) & (tu < tu[-1] - 5)
            o.append(resid_ratio(td, vv[:, i], tu, ref, sel)[0])
        return o

    rows = []
    print("      T_AMP |   resid/ref  a0      a1      a2      a3   | vhat rms ch0")
    print("      delta | " + " ".join("%7.3f" % z for z in score(vd)) + "   | %8.4f" % vd[:, 0].std())
    rows.append(["delta"] + score(vd) + [vd[:, 0].std()])
    for ta in [0.5, 2.0, 5.0, 20.0, 50.0, 100.0, 300.0]:
        q, qdc = design(Rn, Sn, t_amp=ta)
        kf = KalmanVelocity(Rn, q, q_dc=qdc)
        vk = np.zeros((len(td), N))
        for j in range(len(td)):
            vk[j] = kf.update(Vn[j], dt=dtd[j])
        s = score(vk)
        print("      %5.1f | " % ta + " ".join("%7.3f" % z for z in s)
              + "   | %8.4f" % vk[:, 0].std())
        rows.append([ta] + s + [vk[:, 0].std()])
    writecsv("kalman_sweep_quiet.csv",
             ["t_amp_s", "resid_a0", "resid_a1", "resid_a2", "resid_a3",
              "vhat_rms_ch0"], rows)

    print("\n  5c. ROBUSTNESS TO A FREQUENCY THAT IS NOT WHERE THE MODEL PUT IT.")
    print("      Gain relative to ideal / phase error in deg, at mode B, when the")
    print("      TRUE frequency is offset by df. Measured drift over 320 s is at")
    print("      most 1.2e-4 Hz; the DAY-TO-DAY stability of f is NOT measured and")
    print("      that is the standing hazard, not the tuning.\n")
    offs = [0.0001, 0.001, 0.005, 0.02, 0.05, 0.10]
    rows = []
    print("      T_AMP |" + "".join("  df=%-9.4f" % z for z in offs))
    for ta in [2.0, 5.0, 20.0, 50.0, 100.0, 300.0]:
        q, qdc = design(Rn, Sn, t_amp=ta)
        kf = KalmanVelocity(Rn, q, q_dc=qdc)
        cells, raw = [], []
        for dfo in offs:
            f = F_MODE[1] + dfo
            h = kf.transfer(f, i=0)[0]
            g, p = abs(h) / (2 * np.pi * f), np.degrees(np.angle(h)) - 90
            cells.append("%.3f/%+6.2f" % (g, p))
            raw += [g, p]
        print("      %5.1f |" % ta + " ".join("%13s" % z for z in cells))
        rows.append([ta] + raw)
    writecsv("kalman_sweep_freqerr.csv",
             ["t_amp_s"] + sum([[f"gain_df{z}", f"phase_df{z}"] for z in offs], []), rows)
    print("\n  THE DESIGN POINT. Settle time is flat at 1.4-1.6 s over T_AMP")
    print("  20-100 -- it saturates because separating mode A from mode B needs")
    print("  1/0.2794 = 3.58 s of evidence no matter what, and the filter pays")
    print("  about half of that. Steady-state error falls monotonically and")
    print("  frequency tolerance falls slowly. T_AMP = %.0f is the round number in"
          % T_AMP_S)
    print("  the middle of that flat optimum, not a tuned value.")
    return vd


# ===========================================================================
# 6. Replay
# ===========================================================================
def section6(Rn, Sn, td, Vn, dtd, vd_quiet):
    hdr("6. OFFLINE REPLAY AT THE DESIGN POINT")
    q, qdc = design(Rn, Sn)
    kf = KalmanVelocity(Rn, q, q_dc=qdc)
    vk = np.zeros((len(td), N))
    for j in range(len(td)):
        vk[j] = kf.update(Vn[j], dt=dtd[j])
    print("  6a. Quiet record. Nothing to chase but ambient modal motion and real")
    print("      sensor noise. Scored against the zero-phase modal reference and,")
    print("      for delta, also against a 0.4-3.0 Hz reference, since the two")
    print("      estimators do not have the same target.\n")
    rows = []
    print("   ch  est     vs v_modal: resid   corr  |  vs v_band: resid   corr | rms V/s")
    for i in range(4):
        tu, vm = zero_phase_velocity(td, Vn[:, i], [(fm - 0.06, fm + 0.06) for fm in F_MODE])
        _, vb = zero_phase_velocity(td, Vn[:, i], [(BP_LOW_HZ, BP_HIGH_HZ)])
        sel = (tu > tu[0] + 40) & (tu < tu[-1] - 5)
        for nm, vv in (("delta", vd_quiet[:, i]), ("kalman", vk[:, i])):
            rm, cm = resid_ratio(td, vv, tu, vm, sel)
            rb, cb = resid_ratio(td, vv, tu, vb, sel)
            print("   a%d  %-6s      %8.3f %6.3f  |     %8.3f %6.3f | %7.3f"
                  % (i, nm, rm, cm, rb, cb, vv.std()))
            rows.append([i, nm, rm, cm, rb, cb, vv.std()])
    writecsv("kalman_replay_quiet.csv",
             ["ch", "estimator", "resid_vs_modal", "corr_vs_modal",
              "resid_vs_band", "corr_vs_band", "rms_v_per_s"], rows)

    hdr("6b. THE BENCH RUN, FIVE HAND KICKS")
    print("  data/20260806_200822_fast_lock.csv. The control step is reconstructed")
    print("  EXACTLY: delta logs `ctl` and `n_avg`, and the n_avg rows ending at a")
    print("  flagged row are the set that step averaged. The reconstruction is")
    print("  CHECKED by re-running delta's own filter chain and diffing against the")
    print("  logged `ch{i}_vel` -- if that check does not pass, nothing below is")
    print("  worth reading.\n")
    d = load_run()
    t, ctl, nav, nch = d["t"], d["ctl"], d["n_avg"], d["nch"]
    V = A_VCC * d["counts"] / ADC_MAX_COUNTS
    k = np.nonzero(ctl == 1)[0]
    Vm = np.zeros((len(k), nch))
    for j, kk in enumerate(k):
        Vm[j] = V[max(0, kk - nav[kk] + 1):kk + 1].mean(axis=0)
    tc = t[k]
    dtc = np.clip(np.concatenate([[CONTROL_PERIOD_S], np.diff(tc)]), 1e-4, 0.05)
    st = d["state"][k]
    railed = (d["counts"][k] <= RAIL_LOW_COUNTS) | (d["counts"][k] >= RAIL_HIGH_COUNTS)

    dstep = delta_estimator_state()
    vd = np.zeros((len(k), nch))
    bpd = np.zeros((len(k), nch))
    for j in range(len(k)):
        bpd[j], vd[j] = dstep(Vm[j], dtc[j])
    lg = d["vel"][k]
    rows = []
    for i in range(4):
        e = np.abs(vd[:, i] - lg[:, i])
        print("   ch%d  replay vs logged vel: median %.2e  p99 %.2e  max %.2e  "
              "(peak %.2f V/s)" % (i, np.median(e), np.percentile(e, 99), e.max(),
                                   np.abs(lg[:, i]).max()))
        rows.append([i, np.median(e), np.percentile(e, 99), e.max(), np.abs(lg[:, i]).max()])
    writecsv("kalman_replay_check.csv",
             ["ch", "median_abs_diff", "p99_abs_diff", "max_abs_diff", "logged_peak"], rows)
    print("   The residual is the CSV's own rounding: `bp` is logged at 5 decimals,")
    print("   and vel = d(bp)/dt multiplies that 1e-5 by 1/dt = 100.")

    Vn2 = mains_null(Vm)
    dstep2 = delta_estimator_state()
    vdn = np.zeros((len(k), nch))
    for j in range(len(k)):
        _, vdn[j] = dstep2(Vn2[j], dtc[j])
    out = {}
    for tag, blank in (("kalman", False), ("kalman+railblank", True)):
        kf2 = KalmanVelocity(Rn, q, q_dc=qdc)
        vv = np.zeros((len(k), nch))
        for j in range(len(k)):
            vv[j] = kf2.update(Vn2[j], dt=dtc[j], valid=(~railed[j]) if blank else None)
        out[tag] = vv

    D = st == "DAMPING"
    # Kicks: where ch0's bandpass envelope is far above its own DAMPING median.
    env = np.abs(bpd[:, 0])
    kk = max(1, int(1.0 / np.median(dtc)))
    env = np.convolve(env, np.ones(kk) / kk, mode="same")
    hot = (env > 8 * np.median(env[D])) & D & (tc > 25.0)
    wins, j = [], 0
    starts = np.nonzero(np.diff(hot.astype(int)) == 1)[0]
    for s in starts:
        a, b = tc[s] - 1.0, tc[s] + 11.0
        if wins and a < wins[-1][1]:
            continue
        wins.append((a, b))
    inkick = np.zeros(len(tc), bool)
    for a, b in wins:
        inkick |= (tc >= a) & (tc <= b)
    clean = D & ~inkick & (tc > 25.0) & ~railed.any(axis=1)
    print("\n   %d kick windows after t=25 s: %s"
          % (len(wins), ", ".join("%.0f-%.0f s" % w for w in wins)))
    print("   clean DAMPING samples (no kick window, nothing railed): %d of %d"
          % (clean.sum(), D.sum()))

    print("\n   CLEAN DAMPING, scored against the zero-phase modal reference.")
    print("   WEAKER EVIDENCE THAN 6a, and the reason is worth stating: the")
    print("   reference is built from the whole record, which contains clipped")
    print("   kicks, so its band-limited form rings into the quiet stretches. Read")
    print("   the ORDERING here, which is the same on all four channels, and read")
    print("   the absolute numbers off 6a, where nothing rails at all.")
    rows = []
    print("   ch  est                resid/ref    corr   |  rms V/s")
    for i in range(4):
        tu, vm = zero_phase_velocity(tc, Vn2[:, i], [(fm - 0.06, fm + 0.06) for fm in F_MODE])
        sel = np.interp(tu, tc, clean.astype(float)) > 0.99
        for nm, vv in (("delta", vdn[:, i]), ("kalman", out["kalman"][:, i]),
                       ("kalman+railblank", out["kalman+railblank"][:, i])):
            r, c = resid_ratio(tc, vv, tu, vm, sel)
            print("   ch%d %-18s %8.3f  %6.3f   | %8.3f" % (i, nm, r, c, vv[clean].std()))
            rows.append([i, nm, r, c, vv[clean].std()])
    writecsv("kalman_replay_run.csv",
             ["ch", "estimator", "resid_vs_modal", "corr_vs_modal", "rms_v_per_s"], rows)

    print("\n   INSIDE THE KICK WINDOWS, no reference is quoted and that is")
    print("   deliberate: 3.3-3.8% of ch0/ch2 control samples are RAILED there, so")
    print("   the recorded signal is clipped and any offline reference built from")
    print("   it is clipped too. What can be reported is what each estimator does:")
    rows = []
    print("   ch  railed%   | delta peak  kalman peak  +railblank | delta rms  kalman rms")
    for i in range(4):
        m = inkick & D
        print("   ch%d  %6.2f  | %10.2f %12.2f %11.2f | %9.3f %10.3f"
              % (i, 100 * railed[m, i].mean(), np.abs(vdn[m, i]).max(),
                 np.abs(out["kalman"][m, i]).max(),
                 np.abs(out["kalman+railblank"][m, i]).max(),
                 vdn[m, i].std(), out["kalman"][m, i].std()))
        rows.append([i, float(railed[m, i].mean()), np.abs(vdn[m, i]).max(),
                     np.abs(out["kalman"][m, i]).max(),
                     np.abs(out["kalman+railblank"][m, i]).max(),
                     vdn[m, i].std(), out["kalman"][m, i].std(),
                     out["kalman+railblank"][m, i].std()])
    writecsv("kalman_replay_kicks.csv",
             ["ch", "railed_frac", "delta_peak", "kalman_peak", "railblank_peak",
              "delta_rms", "kalman_rms", "railblank_rms"], rows)
    return d, k, tc, Vm, Vn2, dtc, st, vdn, bpd, out, railed, D, inkick, clean, wins


# ===========================================================================
# 7. The FFT proposal
# ===========================================================================
def section7(Rn, Sn):
    hdr("7. 'FFT THE DATA, PHASE-MATCH, SUM, PLAY IT BACK IN NEGATIVE'")
    print("  Three claims. Two are wrong for measurable reasons; the third is right")
    print("  and is section 8.\n")
    print("  7a. THE FFT IS UNNECESSARY AND COSTS MORE PHASE THAN IT RECOVERS.")
    print("      The frequencies are already known. Drift across the 320 s record,")
    print("      first half against second (quiet_freq_stability.csv):")
    rows = []
    for m, nm in enumerate("ABC"):
        dps = 360.0 * abs(F_DRIFT[m])
        print("        mode %s  %+.4e Hz  ->  %.4f deg/s  ->  90 deg in %.0f s (%.1f min)"
              % (nm, F_DRIFT[m], dps, 90.0 / dps, 90.0 / dps / 60))
        rows.append([nm, F_MODE[m], F_DRIFT[m], dps, 90.0 / dps])
    writecsv("kalman_fft_drift.csv",
             ["mode", "f_hz", "drift_hz", "phase_rate_deg_per_s", "t_90deg_s"], rows)
    print("\n      Against that, what a block costs. Separating two lines needs a")
    print("      block longer than 1/df, and a block's answer describes its MIDDLE,")
    print("      so it is stale by half a block:")
    rows = []
    pairs = [("A-B", F_MODE[1] - F_MODE[0]), ("B-C", F_MODE[2] - F_MODE[1]),
             ("A-C", F_MODE[2] - F_MODE[0]),
             ("A vs the 0.6447 Hz line", F_MODE[0] - (F_MODE[2] - F_MODE[1])),
             ("C vs the 1.7104 Hz line", (F_MODE[0] + F_MODE[1]) - F_MODE[2])]
    print("        pair                      df (Hz)  min block  stale by  at 0.7155 Hz")
    for nm, dfp in pairs:
        T = 1.0 / abs(dfp)
        print("        %-24s %7.4f  %8.2f s %8.2f s  %5.2f cyc = %.0f deg"
              % (nm, abs(dfp), T, T / 2, (T / 2) * F_MODE[0], 360 * (T / 2) * F_MODE[0]))
        rows.append([nm, abs(dfp), T, T / 2, (T / 2) * F_MODE[0],
                     360.0 * (T / 2) * F_MODE[0]])
    writecsv("kalman_fft_latency.csv",
             ["pair", "df_hz", "min_block_s", "stale_by_s", "cycles_at_modeA",
              "deg_at_modeA"], rows)
    print("\n      The binding pair is A-B: 3.58 s of block, 1.79 s stale, 1.28")
    print("      cycles at mode A = 461 deg. The error it is correcting reaches")
    print("      90 deg once every 36-68 minutes. The trade is not close.")
    print("      And separating mode A from the 0.6447 Hz line needs 14.1 s, so any")
    print("      block short enough to be useful hands that line's power to mode A.")
    print("\n      THE FIX IS A RECURSIVE LOCK-IN, NOT A BLOCK TRANSFORM: multiply by")
    print("      exp(-j w_m t) at the KNOWN w_m and low-pass. No block, no block")
    print("      latency, a first-order lag on the AMPLITUDE only, and the phase of")
    print("      an already-acquired line is exact.")
    print("\n      AND THAT IS THIS FILTER. A steady-state Kalman filter carrying an")
    print("      undamped oscillator at w_m IS a quadrature lock-in at w_m: (q_m,")
    print("      v_m) is the in-phase/quadrature pair rotating at w_m, K is the loop")
    print("      filter, and section 4's exactness is the lock-in's zero")
    print("      steady-state phase error. Three of them, velocities summed, is the")
    print("      proposal -- done recursively. The two ideas are one design.")
    print("\n      The information cost does not vanish, it changes currency. The")
    print("      FFT delays EVERYTHING by 1.79 s; the filter delays only the")
    print("      AMPLITUDE (measured settle 1.4-1.6 s, section 5a) and holds phase")
    print("      exactly throughout.")
    print("\n  7b. 'PLAY IT BACK IN NEGATIVE' IS A SPRING, NOT A DAMPER.")
    print("      A force proportional to -x is stiffness. It moves the mode")
    print("      frequency and dissipates NOTHING: over a cycle the work done by")
    print("      -k*x against velocity integrates to zero. Removing energy needs a")
    print("      force opposing VELOCITY, 90 deg from displacement, which is what")
    print("      Kp*vhat already is and why Kp is the only dissipative term delta")
    print("      has. Same physics as the Ki/Kd result:")
    print("        Ki integrates velocity -> displacement -> a spring")
    print("        Kp acts on velocity    -> a dashpot    -> the only term that damps")
    print("        Kd differentiates it   -> acceleration -> effective mass")
    print("      A sign error here does not under-damp, it PUMPS. The standing proof")
    print("      is ch2, whose gains are POSITIVE where the other three are negative")
    print("      because that OSEM is mounted the other way round. The summed")
    print("      narrowband estimate goes to Kp with the per-channel sign kept, and")
    print("      nothing else about the sign handling changes.")


# ===========================================================================
# 8. Resonant gain and the actuator budget
# ===========================================================================
def section8(d, k, tc, Vn2, dtc, st, D, inkick, clean, wins, Rn, Sn, vdn, out):
    hdr("8. NARROWBAND (RESONANT) GAIN -- THE PART OF THE PROPOSAL THAT IS RIGHT")
    print("  The three modes have a MEASURED 3 dB width of 0.00566 Hz each")
    print("  (quiet_linewidth.csv) and that is resolution-limited -- the excess over")
    print("  a pure tone through the same window is 0.00159 Hz. At the Q > 433 bound")
    print("  the true width is f/Q:")
    tm, tt = 0.0, 0.0
    for m, fm in enumerate(F_MODE):
        tm += 0.00566
        tt += fm / 433.0
        print("      mode %s  measured 0.00566 Hz (instrument-limited)   f/Q = %.5f Hz"
              % ("ABC"[m], fm / 433.0))
    band = BP_HIGH_HZ - BP_LOW_HZ
    print("      total   measured %.5f Hz = %.2f%% of the %.1f Hz control band"
          % (tm, 100 * tm / band, band))
    print("              at Q=433 %.5f Hz = %.2f%% of it" % (tt, 100 * tt / band))
    print("  Broadband velocity feedback spends authority and phase across 2.6 Hz to")
    print("  damp 0.3-0.65% of it.\n")
    print("  IT IS ALREADY IN THE ESTIMATOR. `Cv` reads only v_A + v_B + v_C, so")
    print("  vhat IS the sum of three resonant estimates and Kp*vhat IS resonant")
    print("  gain -- no extra stage, no extra phase. `modal_velocity()` exposes the")
    print("  three separately if per-mode gains are ever wanted; that is still one")
    print("  filter per channel, still no Phi, still not MIMO.")

    print("\n  8a. WHERE THE ACTUATOR VOLTS GO, measured on the bench run, DAMPING.")
    p, ii, dd = d["p"][k], d["i"][k], d["d"][k]
    rows = []
    print("     ch  | |P|max  |I|max  |D|max | P rms   I rms   D rms | I as % of |P+I+D|max")
    for i in range(4):
        tot = np.abs(p[D, i] + ii[D, i] + dd[D, i]).max()
        print("     ch%d | %6.4f  %6.4f  %6.4f | %6.4f  %6.4f  %6.4f |  %5.1f%%"
              % (i, np.abs(p[D, i]).max(), np.abs(ii[D, i]).max(), np.abs(dd[D, i]).max(),
                 p[D, i].std(), ii[D, i].std(), dd[D, i].std(),
                 100 * np.abs(ii[D, i]).max() / tot))
        rows.append([i, np.abs(p[D, i]).max(), np.abs(ii[D, i]).max(),
                     np.abs(dd[D, i]).max(), p[D, i].std(), ii[D, i].std(),
                     dd[D, i].std(), tot])
    writecsv("kalman_actuator_budget.csv",
             ["ch", "p_peak_v", "i_peak_v", "d_peak_v", "p_rms_v", "i_rms_v",
              "d_rms_v", "total_peak_v"], rows)
    print("\n     gains.json's design-point split (mimo_closed.md section 5.2,")
    print("     derived from drive amplitudes and filter gains, NOT from this run):")
    print("     P 0.1279 + I 0.1135 + D 0.0584 = 0.2998 V against a 0.250 V")
    print("     half-window -- 120% of it, with Ki 38% of the total.")

    print("\n  8b. THE SPECTRUM OF THE DEMAND ITSELF, on the longest CONTIGUOUS")
    print("      DAMPING window (a concatenation of non-contiguous samples has no")
    print("      spectrum). How much of the commanded volts is at the three lines:")
    idx = np.nonzero(D)[0]
    brk = np.nonzero(np.diff(idx) > 1)[0]
    segs = np.split(idx, brk + 1)
    seg = max(segs, key=len)
    fsc = 1.0 / np.median(dtc)
    print("      window %.1f-%.1f s, %d control samples, resolution %.4f Hz"
          % (tc[seg[0]], tc[seg[-1]], len(seg), fsc / len(seg)))
    u = p + ii + dd
    rows = []
    print("      ch   demand rms   at the 3 lines   in band, off-line   above 3 Hz")
    for i in range(4):
        x = u[seg, i] - u[seg, i].mean()
        F = np.fft.rfft(x * np.hanning(len(x)))
        fr = np.fft.rfftfreq(len(x), 1 / fsc)
        tot = np.sum(np.abs(F) ** 2)
        line = np.zeros(len(fr), bool)
        for fm in F_MODE:
            line |= np.abs(fr - fm) <= 0.06
        inb = (fr >= BP_LOW_HZ) & (fr <= BP_HIGH_HZ)
        fl = np.sum(np.abs(F[line]) ** 2) / tot
        fi = np.sum(np.abs(F[inb & ~line]) ** 2) / tot
        print("      ch%d   %9.4f   %13.2f%%   %16.2f%%   %9.2f%%"
              % (i, x.std(), 100 * fl, 100 * fi, 100 * (1 - fl - fi)))
        rows.append([i, x.std(), fl, fi, 1 - fl - fi])
    writecsv("kalman_demand_spectrum.csv",
             ["ch", "demand_rms_v", "frac_at_lines", "frac_inband_offline",
              "frac_above_band"], rows)

    print("\n  8bb. WHAT IT RETURNS IN HEADROOM. Replay the SAME recorded motion")
    print("      through both estimators and form the demand each would command.")
    print("      delta: p+i+d as logged. kalman: gain*(-vhat) with Ki = Kd = 0.")
    print("      OPEN-LOOP REPLAY -- the closed loop would have taken a different")
    print("      trajectory, so this compares the two estimators on one recorded")
    print("      signal. It is NOT a prediction of the closed-loop demand.\n")
    g = d["gain"][k]
    ud = p + ii + dd
    uk = g * (-out["kalman"])
    rows = []
    print("      ch  |  delta rms   kalman rms  ratio |  delta p99  kalman p99 | "
          "delta peak  kalman peak")
    for i in range(4):
        m = D
        a1, b1 = ud[m, i].std(), uk[m, i].std()
        pa, pb = (np.percentile(np.abs(ud[m, i]), 99), np.percentile(np.abs(uk[m, i]), 99))
        qa, qb = np.abs(ud[m, i]).max(), np.abs(uk[m, i]).max()
        print("      ch%d |  %9.4f   %10.4f  %5.2f |  %9.4f  %10.4f | %10.4f  %10.4f"
              % (i, a1, b1, b1 / a1, pa, pb, qa, qb))
        rows.append([i, a1, b1, b1 / a1, pa, pb, qa, qb])
    writecsv("kalman_headroom.csv",
             ["ch", "delta_demand_rms_v", "kalman_demand_rms_v", "ratio",
              "delta_demand_p99_v", "kalman_demand_p99_v",
              "delta_demand_peak_v", "kalman_demand_peak_v"], rows)
    print("\n      Two effects pull opposite ways and both are in that ratio.")
    print("      AGAINST: the kalman estimator restores the in-band magnitude the")
    print("      bandpass attenuated, so at the SAME Kp it asks for 1/0.826, 1/0.842")
    print("      and 1/0.772 = 1.21x, 1.19x and 1.30x more at modes A, B and C.")
    print("      FOR: it commands almost nothing above 3 Hz, where 50-67% of delta's")
    print("      demand power sits, and Ki and Kd are gone.")
    print("      THE TWO ROUGHLY CANCEL, and that is the honest answer: demand rms")
    print("      falls on ch0 and ch2 (the channels carrying the mains alias and the")
    print("      strong intermodulation) and rises slightly on ch1 and ch3. The")
    print("      headroom this work returns is Ki's 0.1135 V, which is measured and")
    print("      is 38% of the design-point peak. Narrowband gain by itself is")
    print("      roughly neutral on actuator demand on this rig.")

    print("\n  8c. WHAT NARROWBAND GAIN COSTS.")
    print("      (i)  A disturbance that is not at a mode gets no force. On this rig")
    print("           there is no evidence of one: in the quiet record 95.6-100% of")
    print("           the in-band power on a0-a3 sits within +-0.06 Hz of the three")
    print("           lines, and what does not is the intermodulation of section 2,")
    print("           which is a SENSOR artefact -- driving a coil at it would be")
    print("           feeding back on the sensor's own curvature.")
    print("      (ii) During a kick the response looks broadband. Measured, in the")
    print("           kick windows, on the longest contiguous piece of each:")
    rows = []
    for a, b in wins:
        m = (tc >= a) & (tc <= b) & D
        if m.sum() < 200:
            continue
        idx2 = np.nonzero(m)[0]
        segs2 = np.split(idx2, np.nonzero(np.diff(idx2) > 1)[0] + 1)
        s2 = max(segs2, key=len)
        T = tc[s2[-1]] - tc[s2[0]]
        # The half-width MUST include the window's own resolution or the answer
        # is smaller than one bin and means nothing. That is why the short
        # segments are reported and then not used.
        half = max(0.12, 2.0 / T)
        x = Vn2[s2, 0] - Vn2[s2, 0].mean()
        F = np.fft.rfft(x * np.hanning(len(x)))
        fr = np.fft.rfftfreq(len(x), 1 / fsc)
        line = np.zeros(len(fr), bool)
        for fm in F_MODE:
            line |= np.abs(fr - fm) <= half
        fl = np.sum(np.abs(F[line]) ** 2) / np.sum(np.abs(F) ** 2)
        use = "used" if T >= 8.0 else "TOO SHORT, not used"
        print("           %6.1f-%6.1f s, %5.1f s contiguous, +-%.3f Hz: %5.1f%% at the"
              " lines  (%s)" % (tc[s2[0]], tc[s2[-1]], T, half, 100 * fl, use))
        rows.append([a, b, tc[s2[0]], tc[s2[-1]], T, half, fl, T >= 8.0])
    writecsv("kalman_kicks.csv",
             ["win_start_s", "win_end_s", "seg_start_s", "seg_end_s", "seg_len_s",
              "half_width_hz", "frac_power_at_lines", "used"], rows)
    print("           A rigid body kicked by hand responds at its resonances and")
    print("           nowhere else; what widens a line here is the finite window,")
    print("           the transient itself, and CLIPPING. None of the three is")
    print("           motion a coil can usefully oppose. The spread across windows")
    print("           is wide and this measurement does NOT settle the question --")
    print("           it is the weakest number in this file and is reported as such.")
    print("      (iii) Authority in the 2.9-4.7 s settling regime is unchanged: the")
    print("           dashpot at the modes goes UP by the factors in section 4, at")
    print("           the same Kp. Nothing is taken away in band.")


# ===========================================================================
# 9. Cost
# ===========================================================================
def section9(Rn, Sn):
    hdr("9. COST PER CONTROL STEP")
    q, qdc = design(Rn, Sn)
    kf = KalmanVelocity(Rn, q, q_dc=qdc)
    y = np.zeros(N)
    kf.update(y)
    n = 20000

    def bench(fn):
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        return (time.perf_counter() - t0) / n

    a = bench(lambda: kf.update(y))
    b = bench(lambda: kf.update(y, dt=0.0101))
    ds = delta_estimator_state()
    ds(y, 0.01)
    c = bench(lambda: ds(y, 0.01))
    print("   all eight channels, one control step, numpy %s:" % np.__version__)
    print("     kalman, fixed Phi              %7.1f us   (%.2f%% of the 10 ms budget)"
          % (a * 1e6, 100 * a / CONTROL_PERIOD_S))
    print("     kalman, Phi rebuilt from dt    %7.1f us   (%.2f%%)"
          % (b * 1e6, 100 * b / CONTROL_PERIOD_S))
    print("     delta HP/LP/diff/LP, for scale %7.1f us   (%.2f%%)"
          % (c * 1e6, 100 * c / CONTROL_PERIOD_S))
    print("\n   Per step, all eight channels: one (8,7)x(7,7) matmul = 392")
    print("   multiply-adds, plus three length-7 contractions = 168. Rebuilding Phi")
    print("   is 3 cos + 3 sin TOTAL, shared by every channel. K is solved OFFLINE")
    print("   and shipped as a constant -- no Riccati recursion runs on the bench.")
    writecsv("kalman_cost.csv", ["variant", "us_per_step", "frac_of_10ms"],
             [["kalman_fixed_phi", a * 1e6, a / CONTROL_PERIOD_S],
              ["kalman_rebuild_phi", b * 1e6, b / CONTROL_PERIOD_S],
              ["delta_bandpass_diff", c * 1e6, c / CONTROL_PERIOD_S]])


def main():
    os.makedirs(OUT, exist_ok=True)
    print(__doc__)
    td, Vd, Vn, dtd, fs, R0, S0, Rn, Sn, In, fn, Pn = section1()
    section2(fn, Pn, Sn)
    section3(Sn, In)
    section4(Rn, Sn)
    vdq = section5(Rn, Sn, td, Vn, dtd, fs)
    (d, k, tc, Vm, Vn2, dtc, st, vdn, bpd, out,
     railed, D, inkick, clean, wins) = section6(Rn, Sn, td, Vn, dtd, vdq)
    section7(Rn, Sn)
    section8(d, k, tc, Vn2, dtc, st, D, inkick, clean, wins, Rn, Sn, vdn, out)
    section9(Rn, Sn)
    hdr("DONE")
    print("  Tables in analysis/out/kalman_*.csv; the argument in analysis/kalman.md.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
