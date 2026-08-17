"""
tune.py -- measure the plant once, then compute every gain offline.
===================================================================
Two modes, deliberately far apart:

    measure   occupies the bench for ONE run and writes raw counts to disk.
    fit       occupies nothing, reads that file, and emits `gains.json`.

WHY NOT A SWEEP. Q ~ 50 at f0 ~ 1 Hz gives tau = Q/(pi f0) ~ 16 s, so a single
gain point costs 60-90 s before it has settled enough to score. Three gains x
four channels x six points is ~90 minutes of optic time, and it assumes the
gains are separable -- which on a body whose four coils drive two shared modes
they are not. A sweep also produces a fit, not a model: change the loop rate,
the coil map or the channel count and every number has to be bought again.

So: measure the PLANT (magnitude and phase of every coil -> every sensor across
the band), then the gains are arithmetic. Minutes of it, re-runnable, and it
generalises because it is a model.

WHAT ONE RUN MEASURES, AND WHAT FALLS OUT
-----------------------------------------
The run records raw counts while a known dither rides on top of a running
damping loop. From that one file:

  on-tone   the complex response matrix H(f) [sensor x coil], magnitude AND
            phase. A DC matrix has no phase in it and cannot size a damping
            gain, because damping is the component of force in phase with
            VELOCITY and phase is exactly what says how much of it there is.
  off-tone  everything between the tones is ambient + sensor noise, i.e. the
            disturbance spectrum -- for free, from the same samples. That is
            what sizes Kd (which amplifies noise by omega^2) and what predicts
            the residual and the headroom for any candidate gain vector.

H(f) is then factorised into the modal model the body actually has,

    H(f) = sum_m R_m / (w_m^2 - w^2 + 2 i zeta_m w_m w)   +   D_static

with R_m a RANK-ONE matrix (a mode is, by definition, one shape driven one
way) and D_static a real matrix for the direction a static push moves the body
in that does not resonate anywhere in band (`sim/server.py:_build_body8` found
that direction in the measured DC matrix and it is real).

CLOSED-LOOP DITHER, AND THE ONE THING IT CHANGES IN THE MATHS
-------------------------------------------------------------
Open loop the optic rings up and ch2 clips: v6 got 4 of 12 frequencies (ch2
railed in 17 of 48 steps) and the multisine variant lost all four passes. So
the loop stays closed and the dither rides on top.

`sysid.acquire` already divides by the drive ACTUALLY COMMANDED rather than the
amplitude intended, which is most of what closed loop needs -- it makes the
estimate immune to clamping and to the coil update rate. But it is not all of
it. In closed loop the four coil commands are no longer independent: dither the
coil of OSEM 0 and the loop answers on all four, so at that tone every column of
the drive is populated. `sysid.multisine`'s scalar `H = Y / U_j` assumes the
opposite and is only correct open loop.

The fix is small and exact. Rotate the tone assignment so every coil takes a
turn at every bin (`sysid.multisine` already rotates, for its own reason), then
at each bin stack the passes into matrices and solve

    H(f) = Y(f) . pinv( U(f) )                                          (*)

U(f) being coils x passes of the commanded drive and Y sensors x passes of the
response. Both sides are measured, so no model of the loop enters. This is the
joint input-output estimator, it is unbiased in closed loop, and cond(U) is a
reportable diagnostic that replaces the open-loop claim that the input matrix is
diagonal by construction. If the loop is weak, U is nearly diagonal and (*)
degenerates gracefully into the scalar division.

The remaining closed-loop bias -- disturbance correlated into the input through
the loop -- vanishes with dwell: the lock-in projects onto the dither reference,
which is uncorrelated with the disturbance by construction, so the incoherent
part averages down as 1/sqrt(T).

TWO DEGREES OF FREEDOM, TWELVE GAINS: WHERE THE ILL-POSEDNESS GOES
------------------------------------------------------------------
The body has 2 observable DOF, confirmed twice (v6's rank check 1.000 / 0.154 /
0.058 / 0.043, and passive spectra at 1.046 Hz on a0/a2 and 1.657 Hz on a1/a3).
Fitting twelve independent gains against two modes is ill-posed and the tool
says so rather than pretending otherwise. Three separate things save it:

1. The quantity the decentralised loop acts through is not S and A separately
   but their per-channel product. Write R_m = s_m a_m^T (rank one). The added
   damping in mode m is set by L_mm = sum_j P_mj C_j(i w_m) with

       P_mj = a_m[j] * s_m[j] = R_m[j][j]

   -- the DIAGONAL of the measured residue matrix. The split of a rank-one
   matrix into s and a is ambiguous by a scale alpha_m; the diagonal is not.
   So the number every gain decision rests on is measured directly and carries
   no split ambiguity at all.

2. The off-diagonal L_mn (mode mixing) does need the split, and there the
   ambiguity is a diagonal similarity D L D^-1. Similarity leaves the closed-
   loop characteristic polynomial det(diag(s^2+2 zeta w s+w^2) - L(s)) alone,
   because a diagonal matrix commutes with D. So the poles are well-posed even
   though the factorisation is not.

3. What is genuinely NOT determined: damping constrains exactly M = 2 linear
   functionals of the N-vector Kp, so with four channels a 2-dimensional family
   of gain vectors damps identically and with eight channels a 6-dimensional
   one. The tool reports the dimension and prints a basis for the undetermined
   directions, and picks a point in that family by MINIMISING ACTUATOR EFFORT --
   headroom being the rig's actual scarce resource. Every reported gain is
   therefore either measured or explicitly chosen by that regulariser, and the
   output says which.

THE THREE GAINS ARE ONE MEASUREMENT AND THREE OBJECTIVES
---------------------------------------------------------
The control law is a PID on DISPLACEMENT wearing velocity-loop clothing:

    p = Kp(-vel) = -Kp s bp,  i = int Ki(-vel) = -Ki bp,  d = -Kd s^2 bp

so Kp multiplies s (viscous damping), Ki multiplies 1 (a spring) and Kd
multiplies s^2 (added mass). Only the first removes energy. Measured today the
peaks are P = 0.0855 V, I = 0.0627 V, D = 0.0402 V against a +-0.25 V window --
about 40% of the authority spent on two terms that dissipate nothing, on a rig
whose central problem is headroom. So:

  Kp  OBJECTIVE: maximise the worst in-band mode's decay rate per volt of
      authority. Well posed, linear in Kp, solved exactly; the whole
      damping-vs-effort trade-off curve is reported, not one point.

  Ki  OBJECTIVE: a CONSTRAINT, not a maximum. Sweeping Ki for "best damping"
      drives it to zero, and its stated purpose -- drift and offset rejection --
      it cannot serve: the integrator sits BEHIND the 0.4 Hz highpass, so
      i = -Ki * bp exactly, and bp is DC-free by construction. Ki rejects no DC
      because the highpass already removed all of it. What it does do is
      measurable and mostly unwanted: a spring shift, plus (because the filter
      chain has net phase lag at the mode frequencies) a small NEGATIVE
      contribution to damping. The tool prints both, and proposes the largest
      |Ki| whose damping penalty and actuator share stay under thresholds --
      which is the honest reading of "how small before something gets worse".

  Kd  OBJECTIVE: also a constraint, and the binding one is noise. The s^2
      weighting makes it the loudest term in the actuator for the least return.
      The disturbance spectrum measured off-tone in the SAME run gives the
      actuator RMS the D path injects; the proposal is the largest |Kd| keeping
      that under a stated fraction of the P path's.

SAFETY
------
  * refuses to run against a controller declaring BENCH_STATUS = "broken";
  * every command clamped to the controller's own VMIN..VMAX window, and the
    dither budget is a fraction of it so the damping loop keeps its headroom;
  * |Kp| ceiling of 0.035 (CLAUDE.md), enforced as a hard box in the optimiser
    -- a value beyond it cannot be proposed without --override-ceiling, and the
    override is stamped into gains.json;
  * abort on a sustained rail or a pinned actuator;
  * all coils returned to bias in a `finally`, including on Ctrl+C;
  * raw counts streamed to disk line-buffered as they arrive, so a Ctrl+C keeps
    everything up to that point (the repo's standing practice, via
    `sysid.Recorder`).

`gains.json` IS A PROPOSAL, NOT CONFIG
--------------------------------------
Nothing loads it. Every controller in this repo is standalone and `bench.py`
prints the gain vectors out of the file that is about to run; a silently loaded
gain file would break exactly that check, on a rig where a wrong sign pumps the
optic. The tool prints a transcription block and a human types it in.

USAGE
-----
    .venv/bin/python tune.py self-test                 # no hardware, proves the maths
    .venv/bin/python tune.py fit  data/..._raw.csv     # no hardware
    .venv/bin/python tune.py measure --port /dev/... --controller osem.eta.py
"""

import argparse
import glob
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime

import numpy as np

import sysid

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# defaults. Every one of these is either a repo constant or a stated policy.
# ---------------------------------------------------------------------------
GAIN_CEILING = 0.035          # CLAUDE.md / analysis/kp040.md. Hard box.
DEFAULT_CONTROLLER = "osem.eta.py"     # newest rung; see ladder.py
DEFAULT_MODES_HZ = (1.046, 1.657)      # analysis/out/modes.csv; refitted, not assumed
DEFAULT_Q = 50.0                       # seed only; zeta is fitted

# Excitation. The tones are deliberately placed OFF the resonances: a Q~50 mode
# amplifies a drive on its peak by ~50x, which is what railed ch2 in v6. Away
# from the peak the response is set by (w_m^2 - w^2), known and large, so the
# residues are well determined at a fraction of the amplitude.
BAND_LO_HZ, BAND_HI_HZ, N_TONES = 0.35, 3.40, 15
DITHER_FRAC = 0.40            # of the half-window; the loop keeps the rest
SETTLE_S, RECORD_S = 12.0, 90.0        # settle >= ~1 ringdown at tau ~ 16 s
UPDATE_HZ = 100.0             # coil refresh; fire-and-forget, so it is cheap
PILOT_S = 12.0                # amplitude-sizing pilot, once, at the start

# Gates
SNR_MIN = 4.0                 # on-tone / local off-tone floor, per (sensor, coil)
RAIL_FRAC = 0.02              # sysid.railed's own default
RAIL_ABORT_S = 3.0            # sustained rail during measure -> stop the run
SAT_ABORT_N = 200             # consecutive pinned commands -> stop the run

# Proposal policy
KP_KNEE_FRAC = 0.90           # smallest effort reaching this share of max damping
KI_DAMPING_PENALTY_FRAC = 0.05    # Ki may cost at most this share of Kp's damping
KI_BUDGET_FRAC = 0.10             # ...and this share of the P term's authority
KD_NOISE_FRAC = 0.10              # D-path actuator noise vs the P path's
CREST = 4.0                   # peak / RMS assumed for a narrowband process
BOOTSTRAP_N = 200


def _now():
    """Wall clock, indirected so the self-test can drive a virtual one."""
    return time.time()


# ===========================================================================
# controller introspection -- the tool models the loop that will actually run
# ===========================================================================
class Controller:
    """The subset of a controller file this tool needs, loaded by path.

    The filter chain, the window, the coil map and the shipping gains all come
    from the file rather than from constants here, so a proposal is always a
    proposal FOR A NAMED CONTROLLER and says so in `gains.json`. That also means
    the tool cannot silently drift out of step with the loop it is sizing.
    """

    def __init__(self, path):
        import importlib.util
        self.path = os.path.abspath(path)
        name = os.path.basename(path)[:-3].replace(".", "_")
        spec = importlib.util.spec_from_file_location(name, self.path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = mod
        spec.loader.exec_module(mod)          # main() is behind __main__
        self.mod = mod

        g = lambda k, d=None: getattr(mod, k, d)
        self.tag = g("VERSION_TAG", os.path.basename(path))
        self.bench_status = g("BENCH_STATUS", "unknown")
        self.kind = g("KIND", "controller")
        self.n = int(g("N", 4))
        self.dac_channels = list(g("DAC_CHANNELS", [1, 3, 5, 7]))
        self.enable = [bool(v) for v in g("ENABLE_CHANNEL", [True] * self.n)]
        self.bias = np.asarray(g("BIAS", np.full(self.n, 0.25)), float)
        self.vmin, self.vmax = float(g("VMIN", 0.0)), float(g("VMAX", 0.5))
        self.kp = np.asarray(g("STEADY_GAIN", np.zeros(self.n)), float)
        self.kp_capture = np.asarray(g("CAPTURE_GAIN", self.kp), float)
        self.ki = np.asarray(g("KI_GAIN", np.zeros(self.n)), float)
        self.kd = np.asarray(g("KD_GAIN", np.zeros(self.n)), float)
        self.bp_low = float(g("BP_LOW_HZ", 0.4))
        self.bp_high = float(g("BP_HIGH_HZ", 3.0))
        self.deriv_smooth = float(g("DERIV_SMOOTH_HZ", 5.0))
        self.d_smooth = float(g("D_SMOOTH_HZ", 2.0))
        self.counts_to_v = float(g("A_VCC", 5.02)) / float(g("ADC_MAX_COUNTS", 1023))

    def onepole(self, hz, kind, n):
        """The controller's own OnePole if it has one, else a mirror of it.

        Preferring the real class is not tidiness: it is the difference between
        modelling the filter the loop runs and modelling one that looks like it.
        `_OnePole` below is checked against it numerically at construction, so a
        version whose filter has moved cannot pass unnoticed.
        """
        cls = getattr(self.mod, "OnePole", None)
        if cls is not None:
            try:
                return cls(hz, kind, n)
            except TypeError:
                pass
        return _OnePole(hz, kind, n)


class _OnePole:
    """Mirror of the controllers' OnePole. Kept local so `tune.py` never edits,
    subclasses or depends on the internals of a controller file.

        low   y[n] = y[n-1] + a (x[n] - y[n-1]),      a = dt / (tau + dt)
        high  y[n] = a (y[n-1] + x[n] - x[n-1]),      a = tau / (tau + dt)
    """

    def __init__(self, hz, kind="low", n=4):
        self.tau, self.low = 1.0 / (2.0 * np.pi * hz), kind == "low"
        self.y, self.xp, self.on = np.zeros(n), np.zeros(n), np.zeros(n, bool)

    def update(self, x, dt):
        new = ~self.on
        y0 = np.where(new, x if self.low else 0.0, self.y)
        a = dt / (self.tau + dt) if self.low else self.tau / (self.tau + dt)
        nxt = (y0 + a * (x - y0) if self.low
               else a * (y0 + x - np.where(new, x, self.xp)))
        self.y, self.xp = np.where(new, y0, nxt), np.asarray(x, float).copy()
        self.on[:] = True
        return self.y

    def reset(self, m=slice(None)):
        self.y[m], self.xp[m], self.on[m] = 0.0, 0.0, False


# ===========================================================================
# the loop's frequency response, from the same difference equations
# ===========================================================================
def _z(w, dt):
    return np.exp(1j * np.asarray(w, float) * dt)


def _lp(w, hz, dt):
    a = dt / (1.0 / (2 * np.pi * hz) + dt)
    zi = 1.0 / _z(w, dt)
    return a / (1.0 - (1.0 - a) * zi)


def _hp(w, hz, dt):
    a = (1.0 / (2 * np.pi * hz)) / (1.0 / (2 * np.pi * hz) + dt)
    zi = 1.0 / _z(w, dt)
    return a * (1.0 - zi) / (1.0 - a * zi)


def t_vel(w, ctl, dt):
    """sensor volts -> `vel`, exactly as the controller computes it.

    bandpass, then a BACKWARD DIFFERENCE (not an ideal differentiator -- it is
    what the code does and it is 10-13 deg of phase at the mode frequencies at a
    23.5 Hz loop rate), then the 5 Hz smoother.
    """
    zi = 1.0 / _z(w, dt)
    return (_hp(w, ctl.bp_low, dt) * _lp(w, ctl.bp_high, dt)
            * ((1.0 - zi) / dt) * _lp(w, ctl.deriv_smooth, dt))


def c_terms(w, ctl, dt):
    """(c_p, c_i, c_d): sensor volts -> actuator volts, per unit of each gain.

    u - bias = c_p Kp + c_i Ki + c_d Kd, all three sharing the velocity path,
    which is why one measurement sizes all three and why their objectives still
    have to differ.
    """
    zi = 1.0 / _z(w, dt)
    tv = t_vel(w, ctl, dt)
    c_p = -tv
    c_i = -tv * dt / (1.0 - zi)                       # i[n] = i[n-1] + Ki err dt
    c_d = -tv * _lp(w, ctl.d_smooth, dt) * (1.0 - zi) / dt
    return c_p, c_i, c_d


# ===========================================================================
# MEASURE
# ===========================================================================
class Damper:
    """NOT a controller. It damps only so the optic stays inside the ADC while
    the dither is measured, and it carries none of the state machine, none of
    the interlocks and none of the gain schedule that make a controller one.

    Pure P, because P is the only term that removes energy and this thing exists
    to remove energy. Its own gains are the shipping STEADY_GAIN of the named
    controller, clipped to the ceiling, so the plant is measured under a loop
    that has actually run on the bench.
    """

    def __init__(self, ctl, kp, authority):
        self.n = ctl.n
        self.kp = np.asarray(kp, float)
        self.authority = float(authority)
        self.hp = ctl.onepole(ctl.bp_low, "high", ctl.n)
        self.lp = ctl.onepole(ctl.bp_high, "low", ctl.n)
        self.dsm = ctl.onepole(ctl.deriv_smooth, "low", ctl.n)
        self.bp = np.zeros(ctl.n)
        self.prev_bp = np.zeros(ctl.n)
        self.vel = np.zeros(ctl.n)
        self.primed = False

    def update(self, volts, dt):
        self.bp = np.asarray(self.lp.update(self.hp.update(volts, dt), dt), float).copy()
        if self.primed and dt > 0:
            self.vel = np.asarray(self.dsm.update((self.bp - self.prev_bp) / dt, dt),
                                  float).copy()
        self.prev_bp, self.primed = self.bp, True

    def command(self):
        """Volts on top of bias. Clipped to its own authority so the dither's
        share of the window cannot be eaten by a transient."""
        return np.clip(self.kp * (-self.vel), -self.authority, self.authority)


def tone_plan(freqs, ncoil, npass=None):
    """Which coil owns which bins in which pass.

    Coil c drives bins freqs[(c + p) % C :: C]. No two coils share a bin inside
    one pass, and rotating over C passes gives every coil a turn at every bin --
    which is what makes the per-bin input matrix in (*) square and invertible.
    """
    C = ncoil
    npass = C if npass is None else npass
    return [[list(freqs[(c + p) % C::C]) for c in range(C)] for p in range(npass)]


def _dither(plan_for_pass, amp_per_coil, ncoil):
    """Sum of that pass's tones per coil, with a deterministic per-tone phase
    stagger so the peaks of a comb do not all land on the same instant."""
    per = [amp_per_coil / max(math.sqrt(max(len(fs), 1)), 1.0) for fs in plan_for_pass]

    def d(tt):
        return [sum(per[c] * math.sin(2 * math.pi * f * tt + 0.7 * k)
                    for k, f in enumerate(plan_for_pass[c])) for c in range(ncoil)]
    return d


def acquire_closed(dac, ctl, seconds, damper, dither, chans, rec, update_hz,
                   clock=None, guard=True):
    """Run the damping loop and the dither together for `seconds`.

    Returns (t, counts, u_ac) with `u_ac` the volts ACTUALLY commanded less
    bias, sample by sample -- the same contract `sysid.acquire` gives, and for
    the same reason: the analysis divides by what the coil got, not by what it
    was asked for, which is what makes the estimate immune to clamping.

    The damping term is refreshed from every sample; the WIRE is refreshed at
    `update_hz`, because a command costs board CPU even fire-and-forget.
    """
    clock = clock or _now
    n, ncoil = ctl.n, len(chans)
    t0 = clock()
    next_update = 0.0
    held = np.asarray(ctl.bias[:ncoil], float).copy()
    ts, rows, us = [], [], []
    prev_t, rail_since, sat_run = 0.0, None, 0
    while True:
        now = clock() - t0
        if now >= seconds:
            break
        s = dac.read_sample(n)
        if s is not None:
            counts = np.asarray(s, float)
            dt = max(now - prev_t, 1e-6)
            prev_t = now
            damper.update(counts * ctl.counts_to_v, dt)
            ts.append(now)
            rows.append(counts)
            us.append(held.copy())
            if rec is not None:
                rec.write(now, counts, held)
            if guard:
                bad = ((counts <= sysid.RAIL_LOW) | (counts >= sysid.RAIL_HIGH)).any()
                rail_since = (rail_since if bad else None) or (now if bad else None)
                if bad and rail_since is not None and now - rail_since > RAIL_ABORT_S:
                    raise RuntimeError(
                        "sensor railed continuously for %.1f s -- stopping and "
                        "parking at bias. Lower --amp or trim the bias."
                        % RAIL_ABORT_S)
        if now >= next_update:
            next_update = now + 1.0 / update_hz
            want = ctl.bias[:ncoil] + damper.command()[:ncoil] + np.asarray(
                dither(now), float)
            held = np.clip(want, ctl.vmin, ctl.vmax)
            pinned = np.any((held <= ctl.vmin + 1e-9) | (held >= ctl.vmax - 1e-9))
            sat_run = sat_run + 1 if pinned else 0
            if guard and sat_run > SAT_ABORT_N:
                raise RuntimeError(
                    "actuator pinned for %d consecutive commands -- stopping and "
                    "parking at bias." % SAT_ABORT_N)
            dac.set_many(chans, held)
    return (np.asarray(ts), np.asarray(rows, float),
            np.asarray(us, float) - np.asarray(ctl.bias[:ncoil], float))


def run_measure(args, dac=None, clock=None, tag="tune"):
    """One bench run. Everything else in this file is offline."""
    ctl = Controller(args.controller)
    # No BENCH_STATUS gate: the label went stale faster than it was updated.

    coils = list(range(min(args.ncoil, ctl.n)))
    chans = [ctl.dac_channels[c] for c in coils]
    half = min(ctl.bias.max() - ctl.vmin, ctl.vmax - ctl.bias.min())
    amp = args.amp if args.amp is not None else DITHER_FRAC * half
    authority = max(half - amp, 0.05)          # what the damping loop may use

    kp = np.clip(ctl.kp[:ctl.n], -args.ceiling, args.ceiling)
    freqs = sysid.snap_bins(args.lo, args.hi, args.tones, args.record)
    plan = tone_plan(freqs, len(coils))

    print("\n  tune.measure -- closed-loop dither against %s (%s)"
          % (os.path.basename(ctl.path), ctl.bench_status))
    print("  %d coils x %d tones, %d passes, %.0fs settle + %.0fs record each"
          % (len(coils), len(freqs), len(plan), args.settle, args.record))
    print("  tones (Hz): %s" % np.round(freqs, 4).tolist())
    print("  dither %.4f V peak/coil, damping authority %.4f V, window %.2f-%.2f V"
          % (amp, authority, ctl.vmin, ctl.vmax))
    print("  damping loop (P only): %s" % "  ".join("%+.4f" % v for v in kp))
    print("  estimated %.0f min. The loop IS damping throughout."
          % (len(plan) * (args.settle + args.record) / 60.0))

    own = dac is None
    if own:
        from pyDAC2 import FastDAC
        dac = FastDAC(port=args.port)
    for ch in chans:
        dac.set_voltage(ch, float(ctl.bias[coils[chans.index(ch)]]))
    if own:
        if not getattr(args, "yes", False):
            input("  Press Enter to start (Ctrl+C to stop)... ")
        dac.start_stream()

    rec = sysid.Recorder(tag, ctl.n, coils)
    meta = dict(kind="tune-raw", tool="tune.py", created=datetime.now().isoformat(),
                controller=os.path.basename(ctl.path), version_tag=ctl.tag,
                bench_status=ctl.bench_status, method="rotated-interleaved-multisine",
                coils=coils, dac_channels=chans, freqs_hz=list(map(float, freqs)),
                amp_v=float(amp), authority_v=float(authority),
                settle_s=float(args.settle), record_s=float(args.record),
                update_hz=float(args.update_hz), bias=ctl.bias[:ctl.n].tolist(),
                vmin=ctl.vmin, vmax=ctl.vmax, ceiling=float(args.ceiling),
                loop_kp=kp.tolist(), loop_ki=[0.0] * ctl.n, loop_kd=[0.0] * ctl.n,
                bp_low_hz=ctl.bp_low, bp_high_hz=ctl.bp_high,
                deriv_smooth_hz=ctl.deriv_smooth, d_smooth_hz=ctl.d_smooth,
                counts_to_v=ctl.counts_to_v,
                plan=[[list(map(float, fs)) for fs in p] for p in plan])
    side = rec.path[:-4] + "_plan.json"
    damper = Damper(ctl, kp, authority)
    try:
        for p, tones in enumerate(plan):
            print("\n  pass %d/%d:" % (p + 1, len(plan)))
            for c, fs in enumerate(tones):
                print("    coil %d (DAC ch%d) <- %s Hz"
                      % (coils[c], chans[c], np.round(fs, 4).tolist()))
            drive = _dither(tones, amp, len(coils))
            dac.drain()
            rec.mark("settle", -1, float(p))
            acquire_closed(dac, ctl, args.settle, damper, drive, chans, rec,
                           args.update_hz, clock)
            rec.mark("record", -1, float(p))
            t, counts, u = acquire_closed(dac, ctl, args.record, damper, drive,
                                          chans, rec, args.update_hz, clock)
            rate = len(t) / max(t[-1] if len(t) else 1.0, 1e-9)
            bad = sysid.railed(counts, RAIL_FRAC)
            print("    %d samples, %.1f Hz effective%s"
                  % (len(t), rate,
                     "" if not bad.any() else "   !! railed: " + ", ".join(
                         "ch%d" % i for i in np.where(bad)[0])))
    except KeyboardInterrupt:
        print("\n  Stopped by user -- everything up to here is on disk.")
    except RuntimeError as e:
        print("\n  ABORTED: %s" % e)
    finally:
        rec.close()                       # before anything that could itself fail
        with open(side, "w") as fh:
            json.dump(meta, fh, indent=1)
        print("  run metadata -> %s" % side)
        for c, ch in zip(coils, chans):
            try:
                dac.set_voltage(ch, float(ctl.bias[c]))
            except Exception:
                pass
        if own:
            time.sleep(0.1)
            dac.stop_stream()
            dac.close()
    print("\n  now, off the bench:   .venv/bin/python tune.py fit %s" % rec.path)
    return rec.path


# ===========================================================================
# FIT -- everything below runs off the recorded file and nothing else
# ===========================================================================
def read_raw(path, bias=sysid.BIAS_V, log=print):
    """`sysid.Recorder`'s CSV back into arrays, plus the sidecar if it exists.

    Tolerant of the SHORT-ROW defect `sysid.Recorder.write` documents and has
    since fixed: runs recorded before that fix wrote one `u` field where the
    header declares one per coil, which silently shifted every `a` column left.
    `data/20260804_121820_v6_stepped4_raw.csv` is such a file. Rather than
    refuse it, the driven value is scattered back into its own coil's slot and
    the rest held at bias -- which is what the hardware was doing -- because a
    real bench recording is worth more than a tidy reader.
    """
    with open(path) as fh:
        head = fh.readline().strip().split(",")
    idx = {k: i for i, k in enumerate(head)}
    ucols = sorted([i for k, i in idx.items() if k.startswith("u")],
                   key=lambda i: int(head[i][1:]))
    acols = sorted([i for k, i in idx.items()
                    if k.startswith("a") and head[i][1:].isdigit()],
                   key=lambda i: int(head[i][1:]))
    ncoil, nsens = len(ucols), len(acols)
    ic, iu = idx["coil"], ucols[0]
    t, seg, ph, co, fr, U, A = [], [], [], [], [], [], []
    short = 0
    with open(path) as fh:
        fh.readline()
        for line in fh:
            p = line.rstrip("\n").split(",")
            if len(p) < len(head):
                # pre-fix row: [.., coil, freq, u_driven, a0..aN]
                if len(p) != len(head) - ncoil + 1:
                    continue
                short += 1
                c = int(float(p[ic]))
                u = [bias] * ncoil
                if 0 <= c < ncoil:
                    u[c] = float(p[iu])
                a = [float(x) for x in p[iu + 1:iu + 1 + nsens]]
            else:
                u = [float(p[i]) for i in ucols]
                a = [float(p[i]) for i in acols]
            try:
                t.append(float(p[idx["time_s"]]))
                seg.append(int(float(p[idx["seg"]])))
                ph.append(p[idx["phase"]])
                co.append(int(float(p[ic])))
                fr.append(float(p[idx["freq_hz"]]))
            except (ValueError, IndexError):
                t.pop() if len(t) > len(seg) else None
                continue
            U.append(u)
            A.append(a)
    if short:
        log("  !! %d rows carried the pre-fix short-row layout and were repaired "
            "(see sysid.Recorder.write). The driven coil is trusted; the others "
            "are taken as held at bias, which is what the board was doing."
            % short)
    out = dict(t=np.asarray(t), seg=np.asarray(seg, int),
               phase=np.asarray(ph), coil=np.asarray(co, int),
               freq=np.asarray(fr), u=np.asarray(U, float),
               a=np.asarray(A, float), path=os.path.abspath(path))
    side = path[:-4] + "_plan.json"
    out["meta"] = json.load(open(side)) if os.path.exists(side) else {}
    return out


def _guard_bins(f, T, tones, k=(2, 3, 4, 5, 6, 7)):
    """Bins next to `f` that carry no tone -- the local incoherent floor.

    These are what make "the drive is bigger than the ambient" a number instead
    of a hope. v6's two useless frequencies (0.3797 and 0.4805 Hz, near-identical
    across coils) were ambient-dominated and nothing in the pipeline said so.
    """
    out = []
    for s in (-1, +1):
        for kk in k:
            g = f + s * kk / T
            if g <= 0:
                continue
            if min(abs(g - x) for x in tones) > 0.5 / T:
                out.append(g)
    return out


def estimate_plant(rec, tones=None, snr_min=SNR_MIN, log=print):
    """The joint input-output estimator, (*) in the module docstring.

    For each tone: lock in every sensor and every coil, per RECORD segment, then
    stack the segments into U (coils x segments) and Y (sensors x segments) and
    solve H = Y pinv(U). Open loop U comes out nearly diagonal and this reduces
    to `sysid.multisine`'s scalar division; closed loop it does not, and that is
    the whole point.
    """
    t, u, a = rec["t"], rec["u"], rec["a"]
    seg, phase = rec["seg"], rec["phase"]
    ncoil, nsens = u.shape[1], a.shape[1]
    meta = rec.get("meta") or {}
    if tones is None:
        tones = meta.get("freqs_hz")
    segs = [s for s in np.unique(seg)
            if (phase is None or (phase[seg == s] == "record").all())]
    if not segs:
        segs = list(np.unique(seg))

    if tones is None:
        # No sidecar. Stepped runs put the tone in the record's own `freq_hz`
        # column; multisine runs do not, but `sysid.save` wrote a JSON next to
        # them that names `raw_csv`, so look for it. Failing both, --tones.
        tones = sorted(set(float(f) for f in rec["freq"] if f > 0))
        if not tones:
            for j in sorted(glob.glob(os.path.join(
                    os.path.dirname(rec["path"]) or ".", "*.json"))):
                try:
                    d = json.load(open(j))
                except Exception:
                    continue
                if (d.get("raw_csv") and os.path.basename(str(d["raw_csv"]))
                        == os.path.basename(rec["path"]) and d.get("freqs")):
                    tones = sorted(float(f) for f in d["freqs"])
                    log("  tone list recovered from %s" % os.path.basename(j))
                    break
    tones = sorted(float(f) for f in tones)
    if not tones:
        raise SystemExit("  no tone list: neither a sidecar nor a freq_hz column.")

    U = np.full((len(tones), ncoil, len(segs)), np.nan, complex)
    Y = np.full((len(tones), nsens, len(segs)), np.nan, complex)
    Nf = np.full((len(tones), nsens, len(segs)), np.nan, float)
    railed = np.zeros((len(tones), nsens), bool)
    c2v = float(meta.get("counts_to_v", sysid.COUNTS_TO_V))

    for k, s in enumerate(segs):
        m = seg == s
        ts, us, as_ = t[m], u[m], a[m]
        if len(ts) < 64:
            continue
        ts = ts - ts[0]
        T = ts[-1] - ts[0]
        bad = sysid.railed(as_, RAIL_FRAC)
        railed |= bad[None, :]
        av = (as_ - as_.mean(axis=0)) * c2v
        uv = us - us.mean(axis=0)
        for fi, f in enumerate(tones):
            if f * T < 3:                       # fewer than 3 cycles: no estimate
                continue
            for j in range(ncoil):
                U[fi, j, k] = sysid.lockin(ts, uv[:, j], f)
            for i in range(nsens):
                Y[fi, i, k] = sysid.lockin(ts, av[:, i], f)
                g = _guard_bins(f, T, tones)
                if g:
                    Nf[fi, i, k] = np.sqrt(np.mean(
                        [abs(sysid.lockin(ts, av[:, i], gg)) ** 2 for gg in g]))

    # --- solve per tone -----------------------------------------------------
    H = np.zeros((len(tones), nsens, ncoil), complex)
    sig = np.full((len(tones), nsens, ncoil), np.inf)
    ok = np.zeros((len(tones), nsens, ncoil), bool)
    cond = np.full(len(tones), np.inf)
    for fi in range(len(tones)):
        cols = [k for k in range(len(segs))
                if np.isfinite(U[fi, :, k]).all() and np.isfinite(Y[fi, :, k]).all()]
        if len(cols) < ncoil:
            continue
        Um, Ym = U[fi][:, cols], Y[fi][:, cols]
        sv = np.linalg.svd(Um, compute_uv=False)
        cond[fi] = sv[0] / max(sv[-1], 1e-30)
        Ui = np.linalg.pinv(Um)
        H[fi] = Ym @ Ui
        # Noise on H: the sensor's incoherent floor pushed through the same
        # inverse. |pinv(U)| per column bounds how much a given sensor's noise
        # can land on a given coil's entry.
        nf = np.nanmean(Nf[fi][:, cols], axis=1)
        amp = np.sqrt(np.mean(np.abs(Um) ** 2, axis=1)) * math.sqrt(len(cols))
        for i in range(nsens):
            sig[fi, i, :] = nf[i] * np.linalg.norm(Ui, axis=0) if np.isfinite(
                nf[i]) else np.inf
            if not np.isfinite(sig[fi, i]).all():
                sig[fi, i, :] = np.abs(H[fi, i, :]) * 0.5 + 1e-12
        del amp
        snr = np.abs(H[fi]) / np.maximum(sig[fi], 1e-30)
        ok[fi] = (snr >= snr_min) & (~railed[fi])[:, None]

    log("\n  tones: %d   usable (sensor,coil) cells: %d of %d"
        % (len(tones), int(ok.sum()), ok.size))
    log("  cond(U) per tone (1.0 = coils independent; large = the loop has"
        " correlated them):")
    log("    " + "  ".join("%.4g" % c for c in cond))
    return dict(freqs=np.asarray(tones), H=H, sigma=sig, ok=ok, cond=cond,
                nsens=nsens, ncoil=ncoil, segs=len(segs),
                loop_hz=float(len(t) / max(t[-1] - t[0], 1e-9)), meta=meta,
                noise=np.nanmean(np.where(np.isfinite(Nf), Nf, np.nan), axis=(0, 2)))


# --- the modal factorisation ----------------------------------------------
def _basis(freqs, fm, zm):
    w = 2 * np.pi * np.asarray(freqs, float)[:, None]
    wm = 2 * np.pi * np.asarray(fm, float)[None, :]
    zeta = np.asarray(zm, float)[None, :]
    G = 1.0 / (wm ** 2 - w ** 2 + 2j * zeta * wm * w)
    return np.hstack([G, np.ones((len(freqs), 1))])     # + the static column


def _derotate(r1):
    """Remove the ONE global phase a rank-one residue may legitimately carry,
    without touching its sign.

    R_m = S[:,m] A[m,:] with S and A real, so the only complex content a correct
    measurement can put in it is a single overall phase from a timing offset.
    Rotating to the principal axis (phi = angle(sum r^2)/2) removes exactly that
    and leaves a +-1 ambiguity, which is then resolved by correlating against
    the UNROTATED real part.

    The obvious shortcut -- rotate until the largest entry is positive real --
    is wrong and was wrong here: on the 2026-08-04 stepped data the largest
    residue is ch2's, which is genuinely positive, but on any run where the
    largest entry is negative it silently flips the sign of the whole matrix and
    therefore the sign of every proposed gain. A wrong sign pumps the optic, so
    this is the one place in the file that must not be clever.
    """
    r1 = np.asarray(r1, complex)
    s = np.sum(r1 * r1)
    phi = 0.5 * np.angle(s) if np.isfinite(s) and abs(s) > 0 else 0.0
    cand = r1 * np.exp(-1j * phi)
    if float(np.sum(cand.real * r1.real)) < 0:
        cand = -cand
    return cand


def _linfit(freqs, H, sig, ok, fm, zm):
    """Given the mode frequencies and damping, solve for R_m and D_static."""
    X = _basis(freqs, fm, zm)
    nf, ns, nc = H.shape
    coef = np.zeros((X.shape[1], ns, nc), complex)
    resid = 0.0
    npt = 0
    for i in range(ns):
        for j in range(nc):
            m = ok[:, i, j]
            if m.sum() < X.shape[1]:
                continue
            wgt = 1.0 / np.maximum(sig[m, i, j], 1e-15)
            A = X[m] * wgt[:, None]
            b = H[m, i, j] * wgt
            c, *_ = np.linalg.lstsq(A, b, rcond=None)
            coef[:, i, j] = c
            r = A @ c - b
            resid += float(np.vdot(r, r).real)
            npt += m.sum()
    return coef, (resid / max(npt, 1)), npt


def _nelder_mead(f, x0, step, iters=400, tol=1e-9):
    """Small simplex search. numpy only -- there is no scipy in the venv."""
    n = len(x0)
    pts = [np.asarray(x0, float)]
    for i in range(n):
        p = np.asarray(x0, float).copy()
        p[i] += step[i]
        pts.append(p)
    val = [f(p) for p in pts]
    for _ in range(iters):
        o = np.argsort(val)
        pts = [pts[i] for i in o]
        val = [val[i] for i in o]
        if abs(val[-1] - val[0]) <= tol * (abs(val[0]) + tol):
            break
        cen = np.mean(pts[:-1], axis=0)
        xr = cen + (cen - pts[-1])
        fr = f(xr)
        if fr < val[0]:
            xe = cen + 2.0 * (cen - pts[-1])
            fe = f(xe)
            pts[-1], val[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < val[-2]:
            pts[-1], val[-1] = xr, fr
        else:
            xc = cen + 0.5 * (pts[-1] - cen)
            fc = f(xc)
            if fc < val[-1]:
                pts[-1], val[-1] = xc, fc
            else:
                for i in range(1, len(pts)):
                    pts[i] = pts[0] + 0.5 * (pts[i] - pts[0])
                    val[i] = f(pts[i])
    o = int(np.argmin(val))
    return pts[o], val[o]


def fit_modal(est, nmode=2, seeds=DEFAULT_MODES_HZ, q0=DEFAULT_Q, log=print):
    """H(f) -> {f_m, zeta_m, R_m (rank one), D_static}.

    The nonlinear part is 2*nmode numbers and is searched; the residues and the
    static term are linear given those and are solved exactly. R_m is then
    projected to rank one, which a mode must be, and the size of what that
    projection throws away is reported as a physics check rather than hidden.
    """
    freqs, H, sig, ok = est["freqs"], est["H"], est["sigma"], est["ok"]
    usable = int(ok.any(axis=(1, 2)).sum())
    if usable < 2 * nmode + 1:
        log("  !! only %d tones survived the gates against %d free mode parameters."
            % (usable, 2 * nmode))
        log("     The fit below is under-determined -- widen the tone list or"
            " raise the dither and re-run.")

    def cost(th):
        fm = np.exp(th[:nmode])
        zm = np.exp(th[nmode:])
        if fm.min() < 0.05 or fm.max() > 20 or zm.min() < 1e-4 or zm.max() > 0.5:
            return 1e30
        return _linfit(freqs, H, sig, ok, fm, zm)[1]

    cands = []
    s = list(seeds)[:nmode] + [1.0] * max(0, nmode - len(seeds))
    cands.append(np.concatenate([np.log(s), np.log([1.0 / (2 * q0)] * nmode)]))
    # A seed that assumes nothing: the biggest peaks of the summed response.
    mag = np.nansum(np.where(ok, np.abs(H), np.nan), axis=(1, 2))
    if np.isfinite(mag).any():
        order = np.argsort(-np.nan_to_num(mag))
        pk = []
        for i in order:
            if all(abs(freqs[i] - p) > 0.15 for p in pk):
                pk.append(float(freqs[i]))
            if len(pk) == nmode:
                break
        if len(pk) == nmode:
            cands.append(np.concatenate([np.log(sorted(pk)),
                                         np.log([1.0 / (2 * q0)] * nmode)]))

    best = None
    for c in cands:
        th, v = _nelder_mead(cost, c, [0.05] * nmode + [0.3] * nmode)
        if best is None or v < best[1]:
            best = (th, v)
    th = best[0]
    fm, zm = np.exp(th[:nmode]), np.exp(th[nmode:])
    o = np.argsort(fm)
    fm, zm = fm[o], zm[o]
    coef, res, npt = _linfit(freqs, H, sig, ok, fm, zm)

    R, rank1 = [], []
    for m in range(nmode):
        Rm = coef[m]
        u_, s_, vh = np.linalg.svd(Rm)
        r1 = s_[0] * np.outer(u_[:, 0], vh[0])
        rank1.append(float(s_[0] / max(np.linalg.norm(s_), 1e-30)))
        R.append(_derotate(r1))
    D = coef[nmode].real

    log("\n  MODAL FIT   %d modes, %d usable points, weighted residual %.3g"
        % (nmode, npt, res))
    for m in range(nmode):
        imag = float(np.linalg.norm(R[m].imag) / max(np.linalg.norm(R[m]), 1e-30))
        log("   mode %d  f = %.4f Hz   zeta = %.4g  (Q = %.0f, gamma_int = %.4f/s)"
            "   rank-1 %.3f   |Im|/|R| %.3f"
            % (m, fm[m], zm[m], 1 / (2 * zm[m]), zm[m] * 2 * np.pi * fm[m],
               rank1[m], imag))
    log("   static (non-resonant) coupling, counts/V:")
    for i in range(D.shape[0]):
        log("     a%d  %s" % (i, "  ".join("%+7.1f" % (v / sysid.COUNTS_TO_V)
                                           for v in D[i])))
    return dict(f=fm, zeta=zm, R=[r.real for r in R], R_c=R, D=D,
                resid=res, rank1=rank1, nmode=nmode)


# --- from the plant to the gains ------------------------------------------
def modal_split(Rm):
    """Rank-one R_m -> (s_m, a_m) with a symmetric split.

    Any split works: the ambiguity R -> (alpha s)(a/alpha) is a diagonal
    similarity on L = A C S, and a diagonal matrix commutes with the modal
    stiffness, so the closed-loop characteristic polynomial does not move. The
    quantity every first-order result uses -- P_mj = R_m[j][j] -- does not
    depend on the split at all.
    """
    u_, s_, vh = np.linalg.svd(np.asarray(Rm, float))
    return u_[:, 0] * math.sqrt(s_[0]), vh[0] * math.sqrt(s_[0])


def loop_matrix(plant, ctl, kp, ki, kd, dt, w):
    """L(i w) = A diag(C_j) S, the feedback seen by the modes."""
    nm = plant["nmode"]
    nc = len(kp)
    c_p, c_i, c_d = c_terms(w, ctl, dt)
    C = np.asarray(kp) * c_p + np.asarray(ki) * c_i + np.asarray(kd) * c_d
    S = np.zeros((plant["R"][0].shape[0], nm))
    A = np.zeros((nm, plant["R"][0].shape[1]))
    for m in range(nm):
        s_, a_ = modal_split(plant["R"][m])
        S[:, m], A[m, :] = s_, a_
    return A[:, :nc] @ np.diag(C) @ S[:nc, :], C


def damping_map(plant, ctl, dt, nc):
    """d(gamma_m) / d(gain_j) for each of Kp, Ki, Kd -- the whole first-order
    picture, and it is LINEAR, which is what makes the Kp problem exactly
    solvable rather than searched.

        gamma_m = -Im(L_mm(i w_m)) / (2 w_m),  L_mm = sum_j P_mj C_j(i w_m)
    """
    nm = plant["nmode"]
    P = np.zeros((nm, nc))
    for m in range(nm):
        P[m] = np.diag(plant["R"][m])[:nc]
    Gp = np.zeros((nm, nc))
    Gi = np.zeros((nm, nc))
    Gd = np.zeros((nm, nc))
    Sp = np.zeros((nm, nc))
    Si = np.zeros((nm, nc))
    Sd = np.zeros((nm, nc))
    for m in range(nm):
        w = 2 * np.pi * plant["f"][m]
        cp, ci, cd = c_terms(np.array([w]), ctl, dt)
        for arr, sarr, c in ((Gp, Sp, cp[0]), (Gi, Si, ci[0]), (Gd, Sd, cd[0])):
            arr[m] = -P[m] * np.imag(c) / (2 * w)      # damping
            sarr[m] = P[m] * np.real(c) / (2 * w)      # frequency (spring) shift
    return dict(P=P, Gp=Gp, Gi=Gi, Gd=Gd, Sp=Sp, Si=Si, Sd=Sd)


def solve_kp(G, ceiling, live, lam):
    """max_Kp min_m (G Kp)_m - lam ||Kp||^2, over the box |Kp_j| <= ceiling.

    Concave and tiny. For a fixed weighting w on the modes the ridge solution is
    closed form, Kp = G^T w / (2 lam) clipped to the box, so the only search is
    over w on the simplex -- one dimension for two modes.
    """
    nm, nc = G.shape
    best = (-np.inf, np.zeros(nc))
    for th in np.linspace(0.0, 1.0, 401) if nm == 2 else _simplex(nm):
        w = np.array([th, 1 - th]) if nm == 2 else th
        x = np.clip(G.T @ w / (2 * lam), -ceiling, ceiling)
        x = np.where(live, x, 0.0)
        v = float(np.min(G @ x))
        if v > best[0]:
            best = (v, x)
    return best[1], best[0]


def _simplex(nm, n=9):
    grid = [np.zeros(nm)]
    for _ in range(200 * nm):
        w = np.random.dirichlet(np.ones(nm))
        grid.append(w)
    grid[0] = np.ones(nm) / nm
    return grid


def _boot_gate(P, sigP, k=2.0):
    """A channel earns a gain only if its residue is bigger than its own error
    bar. a4/a6/a7 carry no motion (0.1-9% of their power in band): a gain there
    injects actuator noise and buys nothing, and this is what says so from the
    data rather than from the changelog."""
    return (np.abs(P) > k * np.maximum(sigP, 1e-30)).any(axis=0)


# ===========================================================================
# the proposal
# ===========================================================================
def propose(est, plant, ctl, dt, args, rec, log=print):
    nc = min(est["ncoil"], ctl.n)
    dm = damping_map(plant, ctl, dt, nc)
    P, Gp, Gi, Gd = dm["P"], dm["Gp"], dm["Gi"], dm["Gd"]

    # Per-entry uncertainty on P, from the measured noise floor pushed through
    # the same linear fit. Cheap parametric bootstrap on the residues only, with
    # the mode frequencies held: they are the best-determined part of the fit
    # (many off-resonance points constrain w_m tightly) and refitting them B
    # times costs minutes for a second-order effect.
    Pfull = _bootstrap_P(est, plant, args.bootstrap, nc)
    # The diagonal is what every gain decision uses and what `boot` has always
    # meant, so it is extracted here and nothing downstream changes. `boot_full`
    # is the new part: rows and columns, which is what qualifies a coil whose
    # own sensor cannot see it. See _bootstrap_P.
    Pb = (np.diagonal(Pfull, axis1=2, axis2=3)[:, :, :nc] if len(Pfull)
          else np.zeros((0, plant["nmode"], nc)))
    sigP = Pb.std(axis=0) if len(Pb) else np.zeros_like(P)
    live = _boot_gate(P, sigP) & np.asarray(ctl.enable[:nc], bool)

    log("\n" + "=" * 72)
    log("  MODAL SELF-GAIN  P[mode][ch] = R_m[j][j]   (V/V per rad^2/s^2)")
    log("  This is the number every gain rests on, and it is the DIAGONAL of a")
    log("  rank-one residue -- so it carries none of the S/A split ambiguity.")
    for m in range(plant["nmode"]):
        log("   mode %d (%.3f Hz)  %s" % (m, plant["f"][m],
            "  ".join("%+9.3f+-%.3f" % (P[m, j], sigP[m, j]) for j in range(nc))))
    log("   usable channels: %s"
        % (", ".join("ch%d" % j for j in np.where(live)[0]) or "NONE"))
    for j in np.where(~live)[0]:
        log("   !! ch%d: residue not distinguishable from its own noise -- "
            "proposing gain 0 (a gain there injects actuator noise and damps "
            "nothing)." % j)

    # --- signs, checked against what is shipping ---------------------------
    log("\n  SIGNS. Damping needs Kp_j * P_mj > 0, so the sign of each gain is a")
    log("  MEASURED property of that channel's residue, not a convention:")
    for j in range(nc):
        want = -1 if P[:, j][np.argmax(np.abs(P[:, j]))] < 0 else +1
        have = int(np.sign(ctl.kp[j])) or want
        flag = "" if want == have else "   <-- DISAGREES WITH THE SHIPPING SIGN"
        log("   ch%d  measured wants %+d, %s ships %+d%s"
            % (j, want, ctl.tag, have, flag))
    log("  ch2 running POSITIVE where the others run negative is reproduced here")
    log("  from the residue sign, not assumed: it is mounted the other way round.")

    # --- Kp: the trade-off curve -------------------------------------------
    kp_max, gmax = solve_kp(Gp, args.ceiling, live, lam=1e-9)
    curve = []
    for lam in np.geomspace(1e-9, 1e4, 90):
        x, v = solve_kp(Gp, args.ceiling, live, lam)
        curve.append((float(np.max(np.abs(x))), float(v), x, float(lam)))
    knee = None
    for peak, v, x, lam in sorted(curve, key=lambda r: r[0]):
        if v >= KP_KNEE_FRAC * gmax:
            knee = (peak, v, x, lam)
            break
    kp = knee[2] if knee else kp_max
    lam_used = knee[3] if knee else 1e-9
    dg_kp = Gp @ kp

    # what is NOT determined
    ns = _nullspace(Gp[:, live] if live.any() else Gp)
    log("\n  Kp  OBJECTIVE: maximise the worst mode's added decay rate per volt.")
    log("      max achievable at the %.3f ceiling: %.4f /s (worst mode)"
        % (args.ceiling, gmax))
    log("      proposed point: %.0f%% of that, at %.0f%% of the peak gain"
        % (100 * (min(dg_kp) / gmax if gmax else 0),
           100 * (np.max(np.abs(kp)) / max(np.max(np.abs(kp_max)), 1e-12))))
    log("      damping constrains %d linear functionals of a %d-vector, so a"
        % (plant["nmode"], int(live.sum())))
    log("      %d-dimensional family of gain vectors damps IDENTICALLY. The point"
        % ns.shape[1])
    log("      below is picked out of that family by minimum actuator effort;")
    log("      nothing in the measurement prefers it. Undetermined directions:")
    idx = np.where(live)[0]
    for k in range(ns.shape[1]):
        v = np.zeros(nc)
        v[idx] = ns[:, k]
        log("        %s" % "  ".join("%+.3f" % x for x in v))

    # --- Ki ----------------------------------------------------------------
    p_only = replay(rec, ctl, nc, kp, np.zeros(nc), np.zeros(nc))
    dki = _scale_to_budget(Gi, Gp, kp, ctl.ki[:nc], live, args.ceiling,
                           KI_DAMPING_PENALTY_FRAC, rec, ctl, nc, p_only["p_pk"])
    ki = dki["gain"]
    # --- Kd ----------------------------------------------------------------
    kd_out = _size_kd(est, plant, ctl, dt, nc, kp, live, args)
    kd = kd_out["gain"]

    budget = replay(rec, ctl, nc, kp, ki, kd)
    budget_now = replay(rec, ctl, nc, ctl.kp[:nc], ctl.ki[:nc], ctl.kd[:nc])
    dg_now = Gp @ ctl.kp[:nc] + Gi @ ctl.ki[:nc] + Gd @ ctl.kd[:nc]
    dg_new = Gp @ kp + Gi @ ki + Gd @ kd

    log("\n  Ki  OBJECTIVE: a constraint. The integrator sits BEHIND the %.1f Hz"
        % ctl.bp_low)
    log("      highpass, so i = -Ki * bp exactly and bp has no DC in it: Ki")
    log("      rejects no drift, because the highpass already removed it.")
    log("      What it does do, measured: spring shift %s"
        % ", ".join("%+.4f Hz" % v for v in _spring(dm, "Si", ki, plant)))
    log("      and a damping change of %s -- %s."
        % (", ".join("%+.5f /s" % v for v in (Gi @ ki)),
           "a penalty" if float(np.sum(Gi @ ki)) < 0 else "a small bonus"))
    log("      proposal is %.0f%% of the shipping Ki -- %s."
        % (100 * dki["scale"], dki["reason"]))
    log("      shipping Ki costs %s /s of damping and %.4f V of peak authority,"
        % (", ".join("%+.5f" % v for v in (Gi @ ctl.ki[:nc])), budget_now["i_pk"]))
    log("      i.e. %.0f%% of the whole actuator peak, to dissipate nothing."
        % (100 * budget_now["i_pk"] / max(budget_now["total_pk"], 1e-12)))

    log("\n  Kd  OBJECTIVE: a constraint, and the binding one is noise. The s^2")
    log("      weighting makes the D path the loudest term per unit of return.")
    log("      measured sensor floor -> D-path actuator RMS %.5f V against the"
        % kd_out["rms_d"])
    log("      P path's %.5f V (target <= %.0f%%)."
        % (kd_out["rms_p"], 100 * KD_NOISE_FRAC))

    log("\n" + "=" * 72)
    log("  PREDICTED, current vs proposed (first order, at the fitted modes):")
    for m in range(plant["nmode"]):
        g0 = plant["zeta"][m] * 2 * np.pi * plant["f"][m]
        log("   mode %d  %.3f Hz   intrinsic %.4f /s   now %+.4f -> %.4f /s"
            "   proposed %+.4f -> %.4f /s"
            % (m, plant["f"][m], g0, dg_now[m], g0 + dg_now[m],
               dg_new[m], g0 + dg_new[m]))
    log("   peak actuator, P/I/D:  now %.4f/%.4f/%.4f V   proposed %.4f/%.4f/%.4f V"
        % (budget_now["p_pk"], budget_now["i_pk"], budget_now["d_pk"],
           budget["p_pk"], budget["i_pk"], budget["d_pk"]))
    log("   total peak: now %.4f V, proposed %.4f V, of a %.3f V half-window."
        % (budget_now["total_pk"], budget["total_pk"], budget["half"]))
    nowd = float(np.min(dg_now)) or 1e-12
    log("   damping per volt of peak authority: now %.2f, proposed %.2f /s/V"
        % (nowd / max(budget_now["total_pk"], 1e-12),
           float(np.min(dg_new)) / max(budget["total_pk"], 1e-12)))
    dissip = lambda b: 1.0 - (b["i_pk"] + b["d_pk"]) / max(b["total_pk"], 1e-12)
    log("   share of authority spent on terms that REMOVE ENERGY: now %.0f%%, "
        "proposed %.0f%%" % (100 * dissip(budget_now), 100 * dissip(budget)))

    return dict(kp=kp, ki=ki, kd=kd, live=live, P=P, sigP=sigP, dm=dm, lam=lam_used,
                curve=[(c[0], c[1], c[2].tolist()) for c in curve],
                gmax=float(gmax), knee=knee is not None,
                dg_now=dg_now, dg_new=dg_new, budget=budget, budget_now=budget_now,
                nullspace=ns, kd_detail=kd_out, ki_detail=dki,
                boot=Pb, boot_full=Pfull)


def _nullspace(G, tol=1e-9):
    if G.size == 0:
        return np.zeros((G.shape[1], 0))
    _, s, vh = np.linalg.svd(G)
    r = int((s > tol * max(s[0], 1e-30)).sum())
    return vh[r:].T


def _spring(dm, key, gain, plant):
    return [float(v) / (2 * np.pi) for v in (dm[key] @ np.asarray(gain))]


def _scale_to_budget(Gi, Gp, kp, ki_now, live, ceiling, frac, rec, ctl, nc, p_pk):
    """Largest |Ki| along the shipping shape that clears BOTH constraints.

    Ki has no maximum to seek -- it removes no energy and, behind the highpass,
    rejects no DC -- so it is bounded rather than optimised, by the two things
    it measurably costs:

      damping   the filter chain has net phase lag at the mode frequencies, so
                the spring term lands partly on velocity with the WRONG sign.
                Capped at `frac` of what Kp buys on the worst mode.
      headroom  its peak share of the actuator, replayed over the recorded
                sensor data. Capped at KI_BUDGET_FRAC of the P term's.

    Zero is a defensible answer and the search includes it.
    """
    base = np.where(live, ki_now[:nc], 0.0)
    if not np.any(base):
        return dict(gain=np.zeros(nc), scale=0.0, damping_cost=0.0, peak_v=0.0,
                    reason="the controller ships Ki = 0; nothing measured asks "
                           "for a nonzero one")
    allow = frac * float(np.min(np.abs(Gp @ kp))) if np.any(kp) else 0.0
    # Both costs are exactly linear in a uniform scale of one gain vector -- the
    # damping map is linear by construction and the peak of |s * base_j * x_j|
    # over channels is s times the peak at s = 1. So the replay is run ONCE, on
    # the full shape, and the search is arithmetic. (The integrator's +-0.15 V
    # backstop is the only nonlinearity and it is a backstop, not an operating
    # point; if it binds, `i_pk` saturates and the scaling is conservative.)
    pk1 = replay(rec, ctl, nc, np.zeros(nc), base, np.zeros(nc))["i_pk"]
    best, s_best, pk_best, pen_best = np.zeros(nc), 0.0, 0.0, 0.0
    for s in np.linspace(0.0, 1.0, 101):
        g = base * s
        if np.max(np.abs(g)) > ceiling:
            continue
        pen = -float(np.min(Gi @ g))                     # positive = damping lost
        pk = s * pk1
        if pen <= allow and pk <= KI_BUDGET_FRAC * max(p_pk, 1e-12):
            best, s_best, pk_best, pen_best = g, s, pk, pen
    return dict(gain=best, scale=float(s_best), damping_cost=float(pen_best),
                peak_v=float(pk_best),
                reason="largest fraction of the shipping Ki whose damping penalty "
                       "stays under %.0f%% of Kp's gain AND whose peak actuator "
                       "share stays under %.0f%% of the P term's"
                       % (100 * frac, 100 * KI_BUDGET_FRAC))


def _size_kd(est, plant, ctl, dt, nc, kp, live, args):
    """Kd from the noise the same run measured, not from taste.

    The D path is a differentiator behind a 2 Hz smoother: its gain rises like
    w^2 until that smoother turns it over, so what it costs is set by the
    sensor floor at the TOP of the band, which the off-tone bins measured.
    """
    w = 2 * np.pi * np.linspace(max(ctl.bp_low * 0.5, 0.05), ctl.bp_high * 2.5, 400)
    c_p, _, c_d = c_terms(w, ctl, dt)
    floor = float(np.nanmedian(est["noise"][:nc])) if np.isfinite(
        est["noise"][:nc]).any() else 0.0
    # A flat floor across the band is the conservative reading of the off-tone
    # bins; they are measured per bin and are flat to within a factor of ~2.
    df = (w[1] - w[0]) / (2 * np.pi)
    rms_p = float(np.max(np.abs(kp))) * math.sqrt(np.sum(np.abs(c_p) ** 2) * df) * floor
    unit_d = math.sqrt(np.sum(np.abs(c_d) ** 2) * df) * floor
    kd_max = (KD_NOISE_FRAC * rms_p / unit_d) if unit_d > 0 else 0.0
    shape = np.where(live, np.sign(ctl.kd[:nc]), 0.0)
    if not np.any(shape):
        shape = np.where(live, np.sign(kp), 0.0)
    kd = shape * min(kd_max, abs(args.ceiling))
    return dict(gain=kd, rms_p=rms_p, rms_d=float(np.max(np.abs(kd))) * unit_d,
                kd_max=float(kd_max), floor=floor)


def replay(rec, ctl, nc, kp, ki, kd):
    """What each term WOULD have asked for, on the sensor data actually recorded.

    Not a crest-factor estimate off a spectrum: the controller's own filter
    chain is run forward over the recorded counts and the P, I and D terms are
    formed with the candidate gains. Headroom is this rig's binding problem, so
    it is worth measuring rather than modelling -- the answer is in volts on the
    same axis as the +-0.25 V window.

    CAVEAT, stated in `gains.json` too: the record was taken with the tool's own
    damping loop closed, so a different Kp would have produced a different
    sensor record. This is therefore the first-order cost of the candidate gains
    against the motion that was actually there, which is the right comparison
    for ranking gain vectors and an over-estimate for a stronger loop (which
    would damp the motion further and ask for less).
    """
    m = rec["phase"] == "record" if rec["phase"] is not None else slice(None)
    t, a = rec["t"][m], rec["a"][m]
    if len(t) < 32:
        return dict(p_pk=0.0, i_pk=0.0, d_pk=0.0, total_pk=0.0,
                    half=0.0, p_rms=0.0, i_rms=0.0, d_rms=0.0)
    hp, lp = _OnePole(ctl.bp_low, "high", nc), _OnePole(ctl.bp_high, "low", nc)
    dsm, dfl = _OnePole(ctl.deriv_smooth, "low", nc), _OnePole(ctl.d_smooth, "low", nc)
    kp, ki, kd = (np.asarray(v, float)[:nc] for v in (kp, ki, kd))
    prev_bp = prev_v = np.zeros(nc)
    integ = np.zeros(nc)
    P, I, D = [], [], []
    prev_t, primed = t[0], False
    for k in range(len(t)):
        dt = max(t[k] - prev_t, 1e-6)
        prev_t = t[k]
        if dt > 1.0:                       # a segment boundary, not a sample gap
            primed = False
            continue
        bp = np.asarray(lp.update(hp.update(a[k, :nc] * ctl.counts_to_v, dt), dt),
                        float).copy()
        if not primed:
            prev_bp, primed = bp, True
            continue
        vel = np.asarray(dsm.update((bp - prev_bp) / dt, dt), float).copy()
        prev_bp = bp
        dv = np.asarray(dfl.update((vel - prev_v) / dt, dt), float).copy()
        prev_v = vel
        integ = np.clip(integ + ki * (-vel) * dt, -0.15, 0.15)
        P.append(kp * (-vel))
        I.append(integ.copy())
        D.append(-kd * dv)
    P, I, D = (np.abs(np.asarray(x)) for x in (P, I, D))
    q = lambda x: float(np.percentile(np.max(x, axis=1), 99.5)) if len(x) else 0.0
    r = lambda x: float(np.sqrt(np.mean(x ** 2))) if len(x) else 0.0
    half = float(min(ctl.bias[:nc].max() - ctl.vmin, ctl.vmax - ctl.bias[:nc].min()))
    return dict(p_pk=q(P), i_pk=q(I), d_pk=q(D),
                p_rms=r(P), i_rms=r(I), d_rms=r(D),
                total_pk=q(P) + q(I) + q(D), half=half)


def _bootstrap_P(est, plant, B, nc):
    """Parametric bootstrap on the residues, mode frequencies held.

    Perturb every surviving H entry by a complex Gaussian of its own measured
    sigma, refit the LINEAR part, and re-extract the residue. That is the
    uncertainty that matters: the gains are linear in P.

    CHANGED 2026-08-06: returns the FULL residue matrices, (B, nmode, nsens,
    ncoil), where it used to keep only `np.diag(...)` and throw the rest away.
    The caller still takes the diagonal for everything it did before, so no
    number this tool already emitted moves. What the extra axes buy:

      * the DIAGONAL P_mj = s_m[j] a_m[j] is a PRODUCT, so a diagonal at the
        noise floor cannot say whether the sensor or the coil is the dead one.
        a5 is the standing case: baseline 51% of the median and visibly
        damping, coil -7 counts/V against a 4-count block noise.
      * COLUMN j over the sensitive rows measures the COIL:
            R_m[i][j] / R_m[i][0] = a_m[j] / a_m[0]      (s_m[i] cancels)
      * ROW j over the strong coils measures the SENSOR:
            R_m[j][i] / R_m[0][i] = s_m[j] / s_m[0]      (a_m[i] cancels)

    So a coil whose own OSEM is too insensitive to see it can still be
    qualified, on the sensors that are sensitive -- which is the only way
    channels 4-7 can be qualified at all.
    """
    if B <= 0:
        return np.zeros((0, plant["nmode"], est["nsens"], est["ncoil"]))
    rng = np.random.default_rng(20260806)
    H, sig, ok, freqs = est["H"], est["sigma"], est["ok"], est["freqs"]
    out = []
    for _ in range(B):
        s = np.where(np.isfinite(sig), sig, 0.0)
        pert = H + (rng.normal(size=H.shape) + 1j * rng.normal(size=H.shape)) * s / math.sqrt(2)
        coef, _, _ = _linfit(freqs, pert, sig, ok, plant["f"], plant["zeta"])
        row = np.zeros((plant["nmode"],) + H.shape[1:])
        for m in range(plant["nmode"]):
            u_, sv, vh = np.linalg.svd(coef[m])
            row[m] = _derotate(sv[0] * np.outer(u_[:, 0], vh[0])).real
        out.append(row)
    return np.asarray(out)


# ===========================================================================
# output
# ===========================================================================
def _git_rev():
    try:
        return subprocess.check_output(["git", "-C", HERE, "rev-parse", "--short",
                                        "HEAD"], stderr=subprocess.DEVNULL
                                       ).decode().strip()
    except Exception:
        return None


def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()[:16]


def write_gains(path, est, plant, ctl, dt, prop, args, log=print):
    nc = len(prop["kp"])
    lo = np.percentile(prop["boot"], 16, axis=0) if len(prop["boot"]) else None
    kp, ki, kd = prop["kp"], prop["ki"], prop["kd"]

    # Gain uncertainty: the gains are linear in P through a fixed optimiser, so
    # push the bootstrapped P back through the same solve.
    # The gains are the output of a deterministic solve on P, so the honest
    # error bar is that same solve run on the bootstrapped P -- not an error
    # propagated through a formula the optimiser does not use.
    kps = []
    for Pb in prop["boot"][: min(len(prop["boot"]), 120)]:
        G = np.zeros_like(prop["dm"]["Gp"])
        for m in range(plant["nmode"]):
            w = 2 * np.pi * plant["f"][m]
            cp = c_terms(np.array([w]), ctl, dt)[0][0]
            G[m] = -Pb[m] * np.imag(cp) / (2 * w)
        x, _ = solve_kp(G, args.ceiling, prop["live"], lam=prop["lam"])
        kps.append(x)
    kps = np.asarray(kps) if kps else np.zeros((0, nc))
    klo = np.percentile(kps, 16, axis=0) if len(kps) else kp
    khi = np.percentile(kps, 84, axis=0) if len(kps) else kp
    del lo

    doc = dict(
        schema="ligo-tune/1",
        kind="MEASUREMENT RECORD AND PROPOSAL -- not live config. Nothing loads "
             "this file. bench.py prints the gain vectors out of the controller "
             "that is about to run; a human transcribes.",
        generated=datetime.now().isoformat(timespec="seconds"),
        git_rev=_git_rev(),
        source=dict(raw_csv=est["meta"].get("path") or args.file,
                    sha256_16=_sha(args.file) if os.path.exists(args.file) else None,
                    sidecar=est["meta"] or None,
                    controller=os.path.basename(ctl.path),
                    version_tag=ctl.tag, bench_status=ctl.bench_status),
        measurement=dict(
            freqs_hz=est["freqs"].tolist(), segments=int(est["segs"]),
            usable_cells=int(est["ok"].sum()), total_cells=int(est["ok"].size),
            cond_U=[None if not np.isfinite(c) else float(c) for c in est["cond"]],
            record_sample_hz=float(est["loop_hz"]),
            assumed_controller_loop_hz=float(1.0 / dt),
            sensor_noise_floor_v=[None if not np.isfinite(v) else float(v)
                                  for v in est["noise"]]),
        plant=dict(
            modes=[dict(f_hz=float(plant["f"][m]), zeta=float(plant["zeta"][m]),
                        Q=float(1 / (2 * plant["zeta"][m])),
                        gamma_intrinsic_per_s=float(plant["zeta"][m] * 2 * np.pi
                                                    * plant["f"][m]),
                        rank1_fraction=float(plant["rank1"][m]),
                        residue_diag_P=prop["P"][m].tolist(),
                        residue_diag_P_sigma=prop["sigP"][m].tolist(),
                        residue_matrix=plant["R"][m].tolist())
                   for m in range(plant["nmode"])],
            static_coupling_v_per_v=plant["D"].tolist(),
            fit_residual=float(plant["resid"])),
        ill_posedness=dict(
            observable_dof=int(plant["nmode"]),
            free_gains_per_term=int(prop["live"].sum()),
            undetermined_dimension=int(prop["nullspace"].shape[1]),
            undetermined_basis=prop["nullspace"].T.tolist(),
            note="Damping constrains exactly `observable_dof` linear functionals "
                 "of the gain vector. Every direction listed here damps "
                 "identically; the proposal picks one by minimum actuator "
                 "effort, not by measurement."),
        current=dict(kp=ctl.kp[:nc].tolist(), ki=ctl.ki[:nc].tolist(),
                     kd=ctl.kd[:nc].tolist(),
                     added_damping_per_s=prop["dg_now"].tolist(),
                     peak_actuator_v=prop["budget_now"]),
        proposal=dict(
            ceiling=float(args.ceiling),
            ceiling_overridden=bool(args.ceiling > GAIN_CEILING),
            kp=dict(value=np.round(kp, 4).tolist(),
                    p16=np.round(klo, 4).tolist(), p84=np.round(khi, 4).tolist(),
                    objective="maximise the worst in-band mode's added decay "
                              "rate per volt of actuator authority; the reported "
                              "point is the smallest effort reaching %.0f%% of "
                              "the achievable maximum at the ceiling"
                              % (100 * KP_KNEE_FRAC),
                    max_achievable_worst_mode_per_s=prop["gmax"]),
            ki=dict(value=np.round(ki, 4).tolist(),
                    objective="CONSTRAINT, not a maximum. The integrator is "
                              "behind the %.1f Hz highpass so i = -Ki*bp and bp "
                              "is DC-free: Ki rejects no drift. Sized as the "
                              "largest fraction of the shipping shape whose "
                              "damping penalty stays under %.0f%% of Kp's gain."
                              % (ctl.bp_low, 100 * KI_DAMPING_PENALTY_FRAC),
                    scale_of_shipping=prop["ki_detail"]["scale"],
                    peak_actuator_v=prop["ki_detail"]["peak_v"],
                    budget_note=prop["ki_detail"]["reason"],
                    damping_change_per_s=(prop["dm"]["Gi"] @ ki).tolist(),
                    spring_shift_hz=_spring(prop["dm"], "Si", ki, plant)),
            kd=dict(value=[float("%.5f" % v) for v in kd],
                    objective="CONSTRAINT, noise-limited. Largest |Kd| whose "
                              "actuator RMS driven by the MEASURED sensor floor "
                              "stays under %.0f%% of the P path's."
                              % (100 * KD_NOISE_FRAC),
                    kd_max_from_noise=prop["kd_detail"]["kd_max"],
                    p_path_noise_rms_v=prop["kd_detail"]["rms_p"],
                    d_path_noise_rms_v=prop["kd_detail"]["rms_d"]),
            predicted=dict(added_damping_per_s=prop["dg_new"].tolist(),
                           peak_actuator_v=prop["budget"]),
            channels_zeroed=[int(j) for j in np.where(~prop["live"])[0]]),
        tradeoff=[dict(peak_gain=c[0], worst_mode_damping_per_s=c[1], kp=c[2])
                  for c in prop["curve"][::6]],
        caveats=_caveats(est, plant, ctl, prop, args))
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=1)
    log("\n  wrote %s" % path)
    return doc


def _caveats(est, plant, ctl, prop, args):
    out = []
    if est["ok"].sum() < 0.5 * est["ok"].size:
        out.append("Fewer than half the (sensor, coil) cells passed the SNR and "
                   "rail gates. Raise the dither or lengthen the record.")
    bad = [int(i) for i in np.where(~np.isfinite(est["cond"]))[0]]
    if bad:
        out.append("Tones %s produced no invertible input matrix -- not enough "
                   "passes reached them." % bad)
    if max(est["cond"][np.isfinite(est["cond"])], default=0) > 50:
        out.append("cond(U) exceeds 50 at some tone: the loop has correlated the "
                   "coil commands enough that the inverse is amplifying noise.")
    for m in range(plant["nmode"]):
        if plant["rank1"][m] < 0.9:
            out.append("Mode %d's residue is only %.0f%% rank one; a mode must be "
                       "rank one, so either the mode count is wrong or two modes "
                       "are unresolved." % (m, 100 * plant["rank1"][m]))
    if args.ceiling > GAIN_CEILING:
        out.append("CEILING OVERRIDDEN to %.4f, above the %.3f in CLAUDE.md. "
                   "Kp = -0.040 has never been applied on hardware (max ever "
                   "commanded is 0.035 across 21 runs, analysis/kp040.md)."
                   % (args.ceiling, GAIN_CEILING))
    out.append("The proposal is for %s's filter chain at an assumed loop rate of "
               "%.1f Hz. Loop rate falls with coil count -- 73 / 23.5 / 12.5 Hz "
               "at 1 / 4 / 8 coils -- and the phase it costs is in these numbers."
               % (ctl.tag, 1.0 / (est["meta"].get("assumed_dt") or
                                  (1.0 / max(est["loop_hz"], 1e-9)))))
    out.append("First-order pole shift. Valid while added damping is small "
               "against the mode's own stiffness, which it is here by ~40x.")
    return out


def transcription(prop, ctl, log=print):
    nc = len(prop["kp"])
    fmt = lambda a: "np.array([" + ", ".join("%+.4f" % v for v in a) + "])"
    log("\n" + "=" * 72)
    log("  TRANSCRIBE INTO %s BY HAND. Nothing loads gains.json, deliberately:"
        % os.path.basename(ctl.path))
    log("  every controller here is standalone and bench.py prints the vectors")
    log("  out of the file about to run. A wrong sign pumps the optic.")
    log("")
    log("  STEADY_GAIN  = %s" % fmt(prop["kp"]))
    log("  CAPTURE_GAIN = %s   # steady x 7/6, the shipping ratio"
        % fmt(np.clip(prop["kp"] * 7.0 / 6.0, -0.035, 0.035)))
    log("  KI_GAIN      = %s" % fmt(prop["ki"]))
    log("  KD_GAIN      = " + "np.array([" + ", ".join("%+.5f" % v
                                                       for v in prop["kd"]) + "])")
    log("")
    log("  was:")
    log("  STEADY_GAIN  = %s" % fmt(ctl.kp[:nc]))
    log("  KI_GAIN      = %s" % fmt(ctl.ki[:nc]))
    log("  KD_GAIN      = " + "np.array([" + ", ".join("%+.5f" % v
                                                       for v in ctl.kd[:nc]) + "])")
    for j in np.where(~prop["live"])[0]:
        log("  !! ch%d proposed at 0: it showed no measurable residue in this run."
            % j)


def run_fit(args, log=print):
    if not args.file:
        cands = sorted(glob.glob(os.path.join(HERE, "data", "*_raw.csv")))
        if not cands:
            sys.exit("  no raw file given and none found in data/.")
        args.file = cands[-1]
        log("  no file given -- using the newest: %s" % args.file)
    rec = read_raw(args.file)
    ctl = Controller(args.controller)
    tones = ([float(x) for x in args.tones.split(",")]
             if getattr(args, "tones", None) and isinstance(args.tones, str) else None)
    est = estimate_plant(rec, tones=tones, snr_min=args.snr, log=log)

    # REFUSE rather than propose from data that did not survive its own gates.
    # A tool that emits a gain vector no matter what teaches you to ignore it,
    # and these gains go onto a suspended optic where a wrong sign pumps.
    usable = int(est["ok"].any(axis=(1, 2)).sum())
    if usable < 2 * args.nmode + 1 and not args.force:
        sys.exit(
            "\n  REFUSING to propose gains: %d of %d tones survived the SNR and\n"
            "  rail gates, against %d free mode parameters. This is the v6 failure\n"
            "  mode -- open loop the optic rings up, ch2 clips, and what is left is\n"
            "  ambient rather than drive. Re-run `tune.py measure` (closed-loop\n"
            "  dither), or raise --snr's input by lengthening the record.\n"
            "  --force emits a proposal anyway, clearly marked; do not fly it."
            % (usable, len(est["freqs"]), 2 * args.nmode))

    # The rank check, unchanged, out of sysid: it is a real test of the
    # measurement and it independently reproduces "four coils drive two DOF".
    sysid.report(est["H"], est["ok"], est["freqs"], list(range(est["ncoil"])),
                 args.nmode, log=log)

    plant = fit_modal(est, args.nmode, log=log)
    dt = 1.0 / (args.loop_hz if args.loop_hz else _default_loop_hz(est, ctl))
    log("\n  controller loop period assumed %.1f ms (%.1f Hz). Loop rate falls with"
        % (1e3 * dt, 1.0 / dt))
    log("  coil count -- 73 / 23.5 / 12.5 Hz at 1 / 4 / 8 coils -- and it is phase,")
    log("  so it changes the answer. Override with --loop-hz.")
    prop = propose(est, plant, ctl, dt, args, rec, log=log)
    doc = write_gains(args.out, est, plant, ctl, dt, prop, args, log=log)
    transcription(prop, ctl, log=log)
    _sensitivity(est, plant, ctl, dt, args, prop, log=log)
    return dict(est=est, plant=plant, prop=prop, doc=doc, ctl=ctl, dt=dt)


def _default_loop_hz(est, ctl):
    """23.5 Hz at four coils is measured (analysis/out/loop_rate.csv) and is what
    a pyDAC-transport controller actually runs; v11 on FastDAC is far faster.
    Guessing wrong is phase, so the number is printed and overridable."""
    n = est["ncoil"]
    return {1: 73.0, 4: 23.5, 8: 12.5}.get(n, 23.5)


def _sensitivity(est, plant, ctl, dt, args, prop, log=print):
    log("\n  SENSITIVITY to the assumed loop rate (it is phase, so it matters):")
    for scale in (0.5, 1.0, 2.0):
        d = damping_map(plant, ctl, dt / scale, len(prop["kp"]))
        g = d["Gp"] @ prop["kp"]
        log("   %5.1f Hz   worst-mode added damping %.4f /s"
            % (scale / dt, float(np.min(g))))


# ===========================================================================
# SELF-TEST -- plant a known answer, recover it, then check the prediction
# ===========================================================================
class _SynthPlant:
    """Two modes on one rigid body, driven by coils at the corners, read by
    OSEMs at the corners, with ambient drive, sensor noise, an ADC and resting
    counts that are off mid-scale exactly as the bench measures them.

    The point of this class is that S, A, f_m, zeta_m and therefore
    P_mj = S[j][m] A[m][j] are all KNOWN, so the fit has a truth to be scored
    against and the recommended gains have a truth to be compared with.
    """

    def __init__(self, n=4, seed=7, dt=1.0 / 347.2):
        self.n, self.dt = n, dt
        self.rng = np.random.default_rng(seed)
        self.f = np.array([1.046, 1.657])
        self.zeta = np.array([0.010, 0.012])
        # Deliberately NOT symmetric: ch2 four times the others and the other
        # way round, the thing the bench measured and the thing a tool must not
        # smooth away.
        self.S = np.array([[1.00, 1.00],
                           [0.36, -0.65],
                           [1.21, 1.29],
                           [0.30, -0.60]])[:n]
        self.A = np.array([[-9.0, -6.0, +18.0, -4.5],
                           [-7.0, -5.5, +15.0, -3.5]])[:, :n]
        self.P = np.array([[self.S[j, m] * self.A[m, j] for j in range(n)]
                           for m in range(2)])
        self.q = np.zeros(2)
        self.qd = np.zeros(2)
        self.rest = np.array([615.0, 641.0, 717.0, 679.0])[:n]
        self.c2v = 5.02 / 1023.0
        self.drive = 0.9
        self.meas_counts = 1.2

    def step(self, u_ac):
        for m in range(2):
            w = 2 * np.pi * self.f[m]
            f = float(self.A[m] @ np.asarray(u_ac, float))
            a = (-w * w * self.q[m] - 2 * self.zeta[m] * w * self.qd[m] + f
                 + self.drive * self.rng.normal() * math.sqrt(1.0 / self.dt) * 0.03)
            self.qd[m] += a * self.dt
            self.q[m] += self.qd[m] * self.dt
        x = self.S @ self.q
        counts = self.rest + x / self.c2v + self.rng.normal(size=self.n) * self.meas_counts
        return np.clip(np.round(counts), 0, 1023)


class _FakeBoard:
    """Enough of `pyDAC2.FastDAC` for `acquire_closed`, driving virtual time so
    the self-test is deterministic and runs in seconds."""

    def __init__(self, plant, ctl, oversample=1):
        self.p, self.ctl = plant, ctl
        self.held = {ch: float(ctl.bias[i]) for i, ch in enumerate(ctl.dac_channels)}
        self.t = 0.0
        self.os = oversample

    def now(self):
        return self.t

    def set_voltage(self, ch, v):
        self.held[int(ch)] = float(v)

    def set_many(self, chans, volts):
        for c, v in zip(chans, volts):
            self.held[int(c)] = float(v)

    def drain(self):
        pass

    def start_stream(self):
        pass

    def stop_stream(self):
        pass

    def close(self):
        pass

    def read_sample(self, ncols):
        u = np.array([self.held[ch] - float(self.ctl.bias[i])
                      for i, ch in enumerate(self.ctl.dac_channels[:self.p.n])])
        s = None
        for _ in range(self.os):
            s = self.p.step(u)
            self.t += self.p.dt
        return list(s[:ncols])


def _truth_damping(plant, ctl, dt, kp):
    nm = 2
    g = np.zeros(nm)
    for m in range(nm):
        w = 2 * np.pi * plant.f[m]
        cp = c_terms(np.array([w]), ctl, dt)[0][0]
        g[m] = -float(plant.P[m] @ np.asarray(kp)) * np.imag(cp) / (2 * w)
    return g


def _ringdown(plant, ctl, dt_loop, kp, seconds=40.0):
    """Close the loop on the PLANTED plant with the recommended gains and
    measure the decay rate that actually results. This is the check that does
    not go through any of the fitting code."""
    p = _SynthPlant(n=plant.n, seed=11, dt=plant.dt)
    p.drive = 0.0                       # ringdown, not a forced response
    p.meas_counts = 0.0
    p.q = np.array([1.0, 1.0])
    d = Damper(ctl, kp, 1e9)
    u = np.zeros(p.n)
    t, env, next_u = 0.0, [], 0.0
    while t < seconds:
        c = p.step(u)
        d.update(c * p.c2v, p.dt)
        if t >= next_u:
            next_u = t + dt_loop
            u = np.clip(ctl.bias[:p.n] + d.command(), ctl.vmin, ctl.vmax) - ctl.bias[:p.n]
        env.append(abs(float(d.bp[0])))
        t += p.dt
    env = np.asarray(env)
    tt = np.arange(len(env)) * p.dt
    m = (tt > 8.0) & (env > 0)
    if m.sum() < 100:
        return float("nan")
    # peak envelope by decimated max, then a log-linear fit
    k = int(0.5 / p.dt)
    pk = np.array([env[i:i + k].max() for i in range(0, len(env) - k, k)])
    pt = np.arange(len(pk)) * 0.5
    sel = (pt > 8.0) & (pk > pk.max() * 1e-3)
    if sel.sum() < 6:
        return float("nan")
    sl = np.polyfit(pt[sel], np.log(pk[sel]), 1)[0]
    return -float(sl)


def run_self_test(args, log=print):
    """Plant a known plant, run the REAL measure loop against it, run the REAL
    fit, and score the recovery -- then verify the recommendation physically."""
    import shutil
    import tempfile

    ctl = Controller(args.controller)
    truth = _SynthPlant(n=min(args.ncoil, ctl.n))
    board = _FakeBoard(truth, ctl)

    tmp = tempfile.mkdtemp(prefix="tune_selftest_")
    cwd = os.getcwd()
    log("\n" + "=" * 72)
    log("  SELF-TEST -- planted plant, real acquisition loop, real fit")
    log("  planted modes: %s Hz   zeta %s"
        % (truth.f.tolist(), truth.zeta.tolist()))
    log("  planted P (mode x channel):")
    for m in range(2):
        log("    %s" % "  ".join("%+8.3f" % v for v in truth.P[m]))
    try:
        os.chdir(tmp)
        m_args = argparse.Namespace(**vars(args))
        m_args.settle, m_args.record = 8.0, args.record
        m_args.amp = args.amp
        m_args.tones = args.tones
        raw = run_measure(m_args, dac=board, clock=board.now, tag="selftest")
        f_args = argparse.Namespace(**vars(args))
        f_args.file = raw
        f_args.out = os.path.join(tmp, "gains.json")
        f_args.loop_hz = args.loop_hz or 23.5
        res = run_fit(f_args, log=log)
    finally:
        os.chdir(cwd)

    plant, prop, dt = res["plant"], res["prop"], res["dt"]
    ok = True
    log("\n" + "=" * 72)
    log("  SCORE")
    for m in range(2):
        e = abs(plant["f"][m] - truth.f[m]) / truth.f[m]
        log("   mode %d frequency   fitted %.4f  truth %.4f   error %.2f%%  %s"
            % (m, plant["f"][m], truth.f[m], 100 * e, "OK" if e < 0.02 else "FAIL"))
        ok &= e < 0.02
    Pf, Pt = prop["P"], truth.P[:, :prop["P"].shape[1]]
    # P carries the overall scale of the coil authority, which is what the fit
    # is for; score sign exactly and magnitude to a tolerance.
    sgn = np.all(np.sign(Pf) == np.sign(Pt))
    rel = np.abs(Pf - Pt) / np.maximum(np.abs(Pt), 1e-9)
    log("   residue diagonal   signs %s   median magnitude error %.1f%%   %s"
        % ("ALL CORRECT" if sgn else "WRONG", 100 * np.median(rel),
           "OK" if sgn and np.median(rel) < 0.20 else "FAIL"))
    ok &= bool(sgn) and float(np.median(rel)) < 0.20

    kp = prop["kp"]
    pred = prop["dm"]["Gp"] @ kp
    true_pred = _truth_damping(truth, ctl, dt, kp)
    e = np.max(np.abs(pred - true_pred) / np.maximum(np.abs(true_pred), 1e-12))
    log("   predicted added damping from the FIT   %s /s"
        % "  ".join("%.4f" % v for v in pred))
    log("   the same gains through the TRUTH       %s /s   worst error %.1f%%  %s"
        % ("  ".join("%.4f" % v for v in true_pred), 100 * e,
           "OK" if e < 0.20 else "FAIL"))
    ok &= e < 0.20

    g_open = _ringdown(truth, ctl, dt, np.zeros_like(kp))
    g_cl = _ringdown(truth, ctl, dt, kp)
    want = g_open + float(np.min(true_pred))
    log("   ringdown on the planted plant: open loop %.4f /s, with the proposed"
        % g_open)
    log("   gains %.4f /s. First order predicts %.4f /s (worst mode). %s"
        % (g_cl, want, "OK" if abs(g_cl - want) < 0.35 * max(want, 1e-9) else
           "outside 35%% -- modes mix, see below"))
    log("   proposed Kp: %s" % "  ".join("%+.4f" % v for v in kp))
    log("   ceiling honoured: %s"
        % ("YES" if np.max(np.abs(kp)) <= args.ceiling + 1e-12 else "NO -- BUG"))
    ok &= np.max(np.abs(kp)) <= args.ceiling + 1e-12

    log("\n  SELF-TEST %s" % ("PASSED" if ok else "FAILED"))
    if not args.keep:
        shutil.rmtree(tmp, ignore_errors=True)
    else:
        log("  artefacts kept in %s" % tmp)
    return 0 if ok else 1


# ===========================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="measure the plant once, then compute the gains offline")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--controller", default=os.path.join(HERE, DEFAULT_CONTROLLER),
                       help="the controller these gains are FOR (default %s)"
                            % DEFAULT_CONTROLLER)
        p.add_argument("--ncoil", type=int, default=4)
        p.add_argument("--ceiling", type=float, default=GAIN_CEILING)
        p.add_argument("--override-ceiling", action="store_true",
                       help="required to set --ceiling above %.3f" % GAIN_CEILING)

    m = sub.add_parser("measure", help="ON HARDWARE: one closed-loop dither run")
    common(m)
    m.add_argument("--port", required=True)
    m.add_argument("--amp", type=float, default=None, help="volts peak per coil")
    m.add_argument("--lo", type=float, default=BAND_LO_HZ)
    m.add_argument("--hi", type=float, default=BAND_HI_HZ)
    m.add_argument("--tones", type=int, default=N_TONES)
    m.add_argument("--settle", type=float, default=SETTLE_S)
    m.add_argument("--record", type=float, default=RECORD_S)
    m.add_argument("--update-hz", type=float, default=UPDATE_HZ)
    # The confirm prompt exists so nobody starts a 14-minute coil-driving run by
    # tab-completing a shell line. That reason does not apply once the run is
    # launched non-interactively, where the prompt is not a safety check, just a
    # process that blocks forever on a stdin nobody is attached to.
    m.add_argument("--yes", action="store_true",
                   help="skip the confirm prompt (for non-interactive launches)")

    f = sub.add_parser("fit", help="offline: raw csv -> gains.json")
    common(f)
    f.add_argument("file", nargs="?", default=None)
    f.add_argument("--out", default=os.path.join(HERE, "gains.json"))
    f.add_argument("--nmode", type=int, default=2)
    f.add_argument("--snr", type=float, default=SNR_MIN)
    f.add_argument("--loop-hz", type=float, default=None,
                   help="the loop rate the CONTROLLER will run at (73/23.5/12.5 "
                        "for 1/4/8 coils, measured)")
    f.add_argument("--bootstrap", type=int, default=BOOTSTRAP_N)
    f.add_argument("--force", action="store_true",
                   help="emit a proposal even when the data failed its own gates")
    f.add_argument("--tones", default=None,
                   help="comma-separated tone list, if the file carries neither a "
                        "sidecar nor a freq_hz column")

    s = sub.add_parser("self-test", help="no hardware: recover a planted answer")
    common(s)
    s.add_argument("--record", type=float, default=60.0)
    s.add_argument("--tones", type=int, default=13)
    s.add_argument("--amp", type=float, default=0.06)
    s.add_argument("--lo", type=float, default=BAND_LO_HZ)
    s.add_argument("--hi", type=float, default=BAND_HI_HZ)
    s.add_argument("--settle", type=float, default=8.0)
    s.add_argument("--update-hz", type=float, default=UPDATE_HZ)
    s.add_argument("--nmode", type=int, default=2)
    s.add_argument("--snr", type=float, default=3.0)
    s.add_argument("--loop-hz", type=float, default=23.5)
    s.add_argument("--bootstrap", type=int, default=40)
    s.add_argument("--out", default="gains.json")
    s.add_argument("--keep", action="store_true")
    s.add_argument("--port", default=None)
    s.add_argument("--force", action="store_true")

    args = ap.parse_args(argv)
    if args.ceiling > GAIN_CEILING and not args.override_ceiling:
        sys.exit("  --ceiling %.4f exceeds the %.3f in CLAUDE.md. Kp = -0.040 has "
                 "never been applied on hardware (analysis/kp040.md). Pass "
                 "--override-ceiling if you mean it; it is stamped into gains.json."
                 % (args.ceiling, GAIN_CEILING))
    if args.cmd == "measure":
        run_measure(args)
        return 0
    if args.cmd == "fit":
        run_fit(args)
        return 0
    return run_self_test(args)


if __name__ == "__main__":
    sys.exit(main())
