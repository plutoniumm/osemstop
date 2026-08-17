"""
zeta: epsilon's loop with a MODAL (MIMO) law in place of the per-channel one --
when, and only when, a measured Phi and A are on disk.

    q_dot_m  =  weighted LS over sensors of  v[i,m] / Phi[i,m]     (per mode)
    f        = -K_m * q_dot_m                                      (3 forces)
    u_C      =  A_C+ f                                             (min-norm)

then epsilon's clip, slew, anti-windup and every interlock, unchanged.

THE MATRICES ARE NOT IN THIS FILE. Phi and A are measured, live in
data/modal.json, and are written by `status.py --save-modal`. With no file, an
unparseable one, one measured at other frequencies, one older than
MODAL_MAX_AGE_S, one whose Phi and A come from different measurements, or one
that fails the colocation sign check, this file runs EPSILON'S DIAGONAL LAW and
prints which of those it was, before the coils are energised.

That refusal is the DEFAULT path and not a branch, which is the one lesson v13
left: it carried a modal law, defined its refusal as `_unused_main`, never
called it, and `make run V=v13` would have driven the optic from a file whose
own docstring said it must not (versions.md). A modal law with hard-coded
matrices is that defect again with better numbers.

WHY THIS IS REOPENED AT ALL, given CLAUDE.md closes it. The closure is correct
about the ARGUMENT -- a square Phi has no null space, so it cannot degrade and
its out-of-mode residual is identically zero -- and wrong about the NUMBER. Phi
was called 3x3 on the strength of one measurement, the 15-tone multisine behind
gains.json, whose tones miss all three modes by 59-68 half-widths because the
grid was designed before the ringdown knew where the modes were. That run put
1.5-1.7% of the available response on the sensors and passed 207 of 960 cells;
its own caveat reads "raise the dither or lengthen the record".

Re-measured from ambient motion on data/20260806_192723_quiet_openloop.csv --
already on disk, no bench time -- Phi is 5x3: a0 a1 a2 a3 a4 determined at all
three modes, cond 7.85, and dropping any one still leaves rank 3. a1, the sensor
the closure rests on, comes back at SNR 12.6 / 19.9 / 15.8.

WHAT IS DIFFERENT FROM THE mimo_closed.md DESIGN, and it is the whole reason
this is buildable now. That design projects a BROADBAND per-channel velocity
through Phi_S+, so the SENSOR SET has to separate the modes -- and Secs 4.2-4.3
are mostly about the damage that does, two clusters degenerate to under 2.5 deg,
`a0,a2` at cond 64, `a0,a5` singular, 256 cached pseudo-inverses to survive it.

epsilon already solved that and nobody noticed. `KalmanVelocity.modal_velocity()`
returns an (8, 3) array -- per channel, PER MODE -- because the estimator carries
one oscillator state per mode and separates them in frequency, exactly, at the
measured f_m. Its docstring says "nothing here reads it yet". So each mode is a
SCALAR least squares across sensors: no matrix inverse, no conditioning to
degrade, no cross-mode leakage. Sensor diversity now buys SNR only. The mask
table survives on the ALLOCATION side, where dropping coils really can make
A_C rank-deficient, and there it is still 256 lookups built once at start-up.

WATCH:
  * MODAL_GAIN_SCALE = 0.5. FIRST LIGHT IS AT HALF. Every sensor now reaches
    every coil, so a gain that was safe diagonally is NOT thereby safe here, and
    Kp = -0.040 is the documented rail onset with one sentence of evidence
    behind it (CLAUDE.md item 7).
  * A WRONG SIGN PUMPS. Phi and A are each determined only up to a shared sign
    per mode, so a Phi from one run paired with an A from another is a coin
    flip per mode. `gauge` refuses that pair outright, and the colocation
    identity A[m,j] = lambda_j Phi[j,m] is the physical cross-check: one head
    means one lambda per channel, so its sign must agree at every mode.
  * The out-of-mode residual is COMPUTED AND LOGGED AND NOT ACTED ON. It is the
    only check that can catch a sensor which is neither railed nor signal-less,
    and there is no measured threshold for it. The first bench run records the
    distribution; the threshold comes after.
  * `make check` CANNOT reach the modal law -- sim/server.py models two modes
    and this runs three -- and Controller's default is no modal data so the
    suite stays reproducible. `python osem.zeta.py --selftest` covers the modal
    math. Whether it damps the optic is a bench question only.
  * Everything epsilon warns about still applies: STEADY_GAIN[2] is POSITIVE by
    measurement, the rail interlock reads RAW COUNTS at the wire rate, and every
    baseline written before epsilon is refused.

Sensing is 5.02 V over 1023 counts; actuation a 2.5 V DAC restricted to 0..0.5 V
around BIAS = 0.25. `enabled` is static config, `healthy` runtime, and
`enabled & healthy` gates the output; absent timers are np.inf.
"""

import json
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime, timezone

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
# DACController.set_voltage() reads up to 50 lines waiting for `OK`, each a lost
# stream sample: 73.0 / 23.5 / 12.5 Hz driving 1 / 4 / 8 coils. FastDAC drops the
# ack, otherwise the same protocol and the same 0..2.5 V firmware clamp.
from pyDAC2 import FastDAC

# ===================== SETTINGS =====================
PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 8

# Sensor 0..7 -> DAC channel, from `ref.py` (provenance.md). NOTHING IN SOFTWARE
# CAN CHECK THIS: sim/server.py stores and reads back through the same map, so a
# wrong map round-trips cleanly and every interlock sees a plausible loop closed
# onto somebody else's coil. Verify on the bench, one coil at a time.
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]

VERSION_TAG = "zeta"
# A NAME, not a number. See ladder.py.
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend",
         "bias-trim", "soft-saturation", "baseline-floor",
         "sat-window", "fast-transport",
         "sample-guard", "decimate", "persist-baseline", "baseline-sanity",
         "kalman-velocity", "mains-null", "lock-quorum",
         # Added 2026-08-07, all three found on the bench, not in the simulator.
         # `runaway-quorum`  only a channel the loop DRIVES can have a runaway.
         # `runaway-peak`    growth is measured against the recent peak, not the
         #                   value at a fixed lag, so a ringdown from a kick is
         #                   no longer a runaway by construction.
         # `trim-quiet`      the bias trim steps and judges only while LOCKED,
         #                   reverts a step it could not evaluate, and stops at a
         #                   total bias excursion budget.
         "runaway-quorum", "runaway-peak", "trim-quiet",
         # Added 2026-08-15. The modal law, and the three things that make it
         # refusable rather than merely optional.
         # `modal-law`       per-mode scalar LS on the Kalman's own per-mode
         #                   velocity, then min-norm allocation through A+.
         # `modal-refuse`    no measured Phi/A -> epsilon's diagonal law, loudly.
         # `modal-colocation` A/Phi sign must be one constant per channel.
         # `modal-residual`  per-mode out-of-mode residual, logged, NOT acted on.
         "modal-law", "modal-refuse", "modal-colocation", "modal-residual")

# All eight. `enabled` buys a place in the quorum, the lock claim and the trim;
# every channel is filtered, rail-checked and watched regardless. Safe only via
# `baseline-floor`: a channel reading nothing calibrates ~0.002 V, every ratio
# explodes, and the breaker faults the rig (v5.5: ten times in 145 s).
# ch5 OFF. a5 is not a weak sensor, it is a disconnected input: measured
# 2026-08-17 across three independent records (96883, 26092 and 113412 samples)
# it takes EXACTLY ONE distinct value, 0.0 counts, with variance exactly zero. A
# live ADC line always carries at least +/-1 count of dither, so that pin is at
# hard ground. On 2026-08-06 it rested at 554.3 counts, so it was reading once.
#
# IT HAS TO BE DISABLED RATHER THAN LEFT TO DEMOTE. 0 counts is below
# RAIL_LOW = 12, so calibration calls it RAILED, and a rail during CALIBRATION is
# a whole-rig fault by design -- correctly, since a railed sensor during
# calibration normally means the optic is against a stop. Measured tonight: the
# rig sat in FAULT for 50 s straight with every gain at zero, damping nothing,
# because of one dead pin. Same shape as the LOCKED veto in CLAUDE.md item 1: one
# channel that cannot contribute must not be able to veto the seven that can.
#
# Turn it back on the moment the pin is fixed; nothing else here depends on it,
# and data/modal.json already carries a5's Phi row as identically zero.
ENABLE_CHANNEL = [True, True, True, True, True, False, True, True]

BIAS = np.full(N, 0.25)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 0.5, 2.0     # nominal; the trim moves the
                                               # per-channel window in __init__

# Kp. STEADY is bench-validated; CAPTURE is stronger, large-amplitude only, NOT
# independently validated.
#   STEADY_GAIN[2] is POSITIVE: ch2 is mounted the other way round (provenance.md
#   Table 1). Not a typo. A wrong sign PUMPS, it does not under-damp.
#   Kp = -0.040 is the documented instability / rail onset; do not exceed -0.035
#   unattended. The simulator provably cannot reproduce that limit, and the
#   estimator's extra phase margin is not a licence to go there (kalman.md §5).
# MEASURED 2026-08-06, tune.py fit of data/20260806_182231_tune_raw.csv, 717420
# samples at 880 Hz. ch2 +0.010 -> +0.035, the largest modal residue of the four
# (+13.573 against -7.851 / -8.867 / -5.282 at mode 0).
# a4-a7 ZERO GAIN, sensors only: 0.1-9% of power in 0.4-3 Hz, lock-in SNR 1.1-1.5,
# coil signs unknown (DC diagonals +8 / +4 / +7 counts/V on a 4-count mean
# |response|). Stepped sine, eight coils, both quadratures (CLAUDE.md 2b) fixes it.
STEADY_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                        +0.000, +0.000, +0.000, +0.000])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                         +0.000, +0.000, +0.000, +0.000])

# Ki is ZERO here and it is STRUCTURALLY unnecessary, not tuned to zero. In delta
# `i = -Ki*bp` exactly, the integrator sitting behind a 0.4 Hz highpass that had
# already removed the drift, so it rejected nothing and spent 0.1135 V of peak
# actuator doing it: 38% of a 0.2998 V design-point peak against a 0.250 V
# half-window (analysis/mimo_closed.md §5.2). Here the drift is the estimator's
# `b` state and v_hat never reads it, so there is nothing left for an integrator
# to reject. The ARRAY stays, all zeros: the clamp and the back-calculation are
# shared machinery and ripping them out is a bigger change than this version
# carries. Kd survives at delta's value; kalman.md §8 recommends cutting it too
# and §12 wants that done on its own, after the estimator has been measured.
KI_GAIN = np.array([+0.0000, +0.0000, +0.0000, +0.0000,
                    +0.0000, +0.0000, +0.0000, +0.0000])
# Same sign per channel as Kp, so a guessed sign would be wrong in two places at
# once. The derivative of velocity is ACCELERATION: effective mass, dissipating
# nothing.
KD_GAIN = np.array([-0.00045, -0.00045, +0.00045, -0.00045,
                    +0.00000, +0.00000, +0.00000, +0.00000])
D_SMOOTH_HZ, I_CLAMP_V, TRACK_TC_S = 2.0, 0.15, 0.5   # I_CLAMP is a backstop

CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
SCHEDULE_WINDOW_S, GAIN_SLEW_PER_S = 1.0, 0.02

# ===== decimate: the loop's own clock =====
# The ARRIVAL rate has been 12.5, 18, 23.5, 73 and ~1111 Hz here, set by coil count
# and transport; none of those are control decisions, this one is.
# 100 Hz is bracketed: 27x the fastest mode (2.79 Hz), 9.7 wire samples per step at
# 1111 Hz, and it is what puts `mains-null`'s first null on 50 Hz exactly. A slower
# wire runs a step per arrival.
CONTROL_HZ = 100.0
CONTROL_PERIOD_S = 1.0 / CONTROL_HZ

# ===== mains-null =====
# The raw stream carries a 50.06 Hz mains line: 0.1439 V rms on a0, 0.0778 V on a5,
# 0.0386 V on a1 (analysis/out/kalman_R.csv). A 10 ms boxcar attenuates 50 Hz only
# to sinc(0.5) = 0.64 and the 100 Hz step then folds it to just under Nyquist, where
# it is 0.80 / 0.96 / 0.89 of sqrt(R) on a0 / a1 / a5 -- i.e. on three channels the
# DOMINANT measurement noise at the control rate is aliased mains. Averaging two
# consecutive control means is a 20 ms boxcar whose first null is 50 Hz exactly:
# R falls to 0.354x on ch0, 0.057x on ch1 (its floor drops about 18x, and a1's
# floor is the number that closed modal control), 0.987x on ch2, 0.190x on ch5.
# Cost is 5.0 ms of FLAT group delay: -1.29 / -1.79 / -2.95 deg at the three modes,
# magnitude 0.9997 / 0.9995 / 0.9987.
# The null sits at 1/(2*dt) and lands on 50 Hz only because CONTROL_HZ is 100.
# Move the control rate and the null moves off the mains.
MAINS_NULL = True

# ===== the velocity estimator =====
# Three modes. Everything the estimator does that a bandpass cannot rests on
# these three numbers.
#
# RE-MEASURED 2026-08-17 (data/20260817_172150_status_sensors.csv, `status.py
# sensors`): consensus over the five sensors above SNR 8, inter-sensor spread
# 0.0005 Hz. THE MODES HAD MOVED, and this is the first session anybody checked:
#
#     mode   2026-08-06   2026-08-17   shift       half-widths at Q=433
#     A      0.7155       0.72294      +0.00744     9.1
#     B      0.9949       0.99193      -0.00297     2.6
#     C      1.6396       1.65657      +0.01697     9.0
#
# The half-width is f/(2Q); at n half-widths off, a drive reaches 1/sqrt(1+n^2)
# of the on-peak response and its PHASE is wrong. So these are not cosmetic
# digits -- the same 0.010-0.017 Hz error made status.py's anti-phase unwind
# PUMP the optic instead of cancelling (residual 0.188 -> 0.277 -> 0.422 V
# against a 0.259 V drive peak), because 60 s of drive-plus-unwind accumulates
# more than 180 deg of phase error at that offset.
#
# Quoted to five decimals, which is the precision the 90 s record supports. The
# 16-digit values that were here came from the 276 s ringdown fit
# (analysis/out/quiet_frequencies.csv) and are kept above as the 08-06 column;
# do NOT restore that precision on top of a 90 s measurement.
#
# THEIR STABILITY IS STILL NOT ESTABLISHED. Two observations 11 days apart is
# not a drift rate. Drift WITHIN one 320 s record is <= 1.2e-4 Hz. Re-measure at
# the START of every session, before driving anything -- `status.py sensors`
# prints the numbers and `status.py --freqs` takes them.
F_MODE_HZ = np.array([0.72294, 0.99193, 1.65657])
# T_AMP is the only tuning constant in the estimator: the time in which a mode's
# amplitude may change by order itself. 50 s is the middle of a flat optimum,
# settle 1.4-1.6 s over T_AMP 20-100 (analysis/out/kalman_sweep_synthetic.csv).
# T_DC is the DC state's version, long because the rest position moves on the
# trim's timescale (TRIM_PERIOD_S = 15 s), not the modes'.
T_AMP_S, T_DC_S = 50.0, 20.0
# R: everything the three-mode model does not carry -- broadband floor plus every
# line that is not a mode, 0.2-50 Hz with the mode bands bridged -- measured on
# data/20260806_192723_quiet_openloop.csv (355797 samples, 1111.87 Hz, coils held
# at bias), decimated the way the loop decimates and WITH `mains-null` applied,
# because the decimated mean is what the filter sees. V^2, per channel; the spread
# across channels is 14x, which is why all eight were measured.
KALMAN_R = np.array([4.673429582e-03, 3.735535655e-05, 3.647149147e-03,
                     4.858924713e-05, 3.316806993e-05, 5.845702231e-04,
                     1.201535857e-04, 2.386431425e-05])
# Q: q[i,m] = 2 w_m^2 sigma^2[i,m] / T_AMP, so 24 of the 25 numbers are the
# measured modal variances from that same spectrum and one is T_AMP. R and Q are
# therefore consistent by construction, and neither needs a per-channel knob:
# both enter q/R, which is what sets each filter's bandwidth. V^2/s^3.
KALMAN_Q = np.array([
    [1.352121665e-03, 1.537383088e-01, 2.491687224e-02],
    [4.413914307e-04, 2.130937682e-02, 1.365794376e-02],
    [2.776031283e-03, 3.323627307e-01, 5.187054209e-02],
    [3.262378479e-04, 1.603699462e-02, 1.059346571e-02],
    [1.146130994e-07, 3.329487040e-06, 1.710179103e-06],
    [1.301556016e-05, 2.676339313e-02, 3.848128557e-06],
    [2.485429127e-07, 6.636087553e-07, 1.259370922e-06],
    [1.305297122e-07, 1.553854780e-06, 5.111513956e-07]])
# a4/a6/a7 come out at q ~ 1e-7 and their filters will correctly estimate almost
# nothing. That is the plant, not the tuning: 17-50% of their in-band power is at
# the three lines against 95.5-99.8% on a0-a3.
KALMAN_Q_DC = KALMAN_R / T_DC_S
# THE STEADY-STATE GAIN, SOLVED OFFLINE. (8, 7), rows per channel, columns
# [q_A, v_A, q_B, v_B, q_C, v_C, b]. Produced by the Riccati recursion in
# analysis/kalman.py:KalmanVelocity._steady_gain from KALMAN_R / KALMAN_Q /
# KALMAN_Q_DC at dt = 0.01 s; converges from P = I in 4000-12000 iterations,
# 0.2-0.4 s per channel. NO RICCATI RUNS HERE: that solve at start-up would spend
# 2-3 s of the 10 s lock budget. Measured, 8 channels, one step, this machine:
# 11.3 us reusing Phi, 22.7 us rebuilding it from the measured dt, against
# 12.2 us for delta's HP/LP/diff/LP. The loop rebuilds, i.e. it pays the 22.7:
# the control step lands at 92.6 Hz on a 1111 Hz wire, so a fixed 10 ms Phi would
# have theta wrong by 8% at every mode, and 0.05 Hz of frequency error already
# costs 7.3 deg. 22.7 us is 0.23% of the 10 ms budget. Pass dt=None in _control
# to take the 11.3 instead.
# R and Q above are carried so this array is reproducible; neither is read in the
# loop. Checked at build time: |H|/w = 1.000 and phase error < 1e-11 deg at all
# three modes on every channel.
KALMAN_K = np.array([
    [1.075915568e-02, -1.589312360e-02, 6.570699385e-02, 3.549956589e-01,
     6.423534167e-03, 2.083006063e-01, 2.116536034e-02],
    [6.388558297e-02, -8.902332939e-02, 2.083406963e-01, 1.633623568e+00,
     -5.695089308e-02, 1.566330590e+00, 1.955971513e-02],
    [1.700381345e-02, -2.710233011e-02, 9.790950001e-02, 6.426183834e-01,
     7.965878210e-05, 3.505859775e-01, 2.078725300e-02],
    [4.863317164e-02, -7.305936963e-02, 1.713845945e-01, 1.210126789e+00,
     -3.135964768e-02, 1.273245026e+00, 1.989282605e-02],
    [1.211106735e-03, 1.971746109e-03, 4.608727054e-03, 1.199137089e-02,
     2.067573412e-03, 6.819056238e-03, 2.202372662e-02],
    [2.973111711e-03, -4.750591216e-03, 7.189085004e-02, 4.602122647e-01,
     1.475682248e-04, 7.562372853e-03, 2.125713224e-02],
    [9.048408069e-04, 1.903547734e-03, 1.093541948e-03, 2.668173450e-03,
     9.517588115e-04, 2.461766706e-03, 2.207907140e-02],
    [1.508168852e-03, 2.676466598e-03, 3.697354490e-03, 9.915296369e-03,
     1.339333699e-03, 4.206829638e-03, 2.203878805e-02]])
# `rail-blank`: skip the UPDATE on a railed sample and coast on the model. It cuts
# peak |v_hat| inside the kick windows from 21.85 to 15.95 V/s on ch0 and 29.92 to
# 20.75 on ch2 (analysis/out/kalman_replay_kicks.csv), and it is UNVALIDATED: a
# permanently railed sensor would coast forever and the bound that would stop it
# has no measurement behind it. OFF. It never touches an interlock either way --
# the estimator consumes `self.rail`, nothing consumes the estimator.
RAIL_BLANK = False

# ===== calibration: a ceiling and a convergence test, not a fixed duration =====
# CALIBRATION_S is the LONGEST it may take; the ceiling path is v5's estimator.
# CALIB_SUBWINDOW_S = 2.0 s is two cycles of the ~1 Hz resonance, the shortest
# window whose RMS is not dominated by its start phase, and divides the ceiling by
# 10. CALIB_AGREE_TOL = 1.20 sits in a measured gap: the trailing three agree to
# 1.09 then ~1.03 in a quiet lab, but run 1.19-1.66 for the whole 20 s under a
# 6 V/s kick inside the window. Hence _calib_stationary's SECOND test.
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # ceiling; subwindows >= 3
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20

# LOCK_SUSTAIN_S is the next cut, 5 s of a 10 s budget confirming a lock already
# had. NOT taken here: this version replaces the whole velocity path, and the
# bench run has to answer "did the estimator help" without also asking "or did the
# shorter window just make it look that way".
LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0
# RUNAWAY_SUSTAIN_S must be SEVERAL TIMES ENVELOPE_WINDOW_S, and beta through
# delta shipped them EQUAL at 2.0 s. That is not a margin, it is a collision: the
# envelope is a sliding RMS over ENVELOPE_WINDOW_S, so an impulse takes a full
# window to work through it and produces about that much monotonic rise on its
# own. A sustain of one window length is therefore satisfied by the window's own
# fill time, and every kick trips the breaker regardless of what the optic does
# next. Swept against the measured plant (kick, tau = 9 s, through the
# controller's own 2 s RMS, four kicks 35 s apart, peak reference excluded by
# ENVELOPE_WINDOW_S): 2.0 s gives 4 spurious trips, 4.0 s and above give none.
# 4.0 s is chosen rather than more because detection latency grows with it and
# nothing is bought: a genuine runaway doubling every 8 s is caught at 12.9 s
# here against 14.4 s for the lag test beta shipped, so this is BETTER on both
# counts, fewer false trips and faster true ones. The rail interlock is
# independent of all of this and still fires immediately.
ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE, RUNAWAY_SUSTAIN_S = 2.0, 1.8, 4.0
# The breaker also requires GROWTH, the envelope against itself
# RUNAWAY_TREND_LAG_S ago; 1.02 leaves 2% so envelope noise cannot read as growth.
# RUNAWAY_TREND_LAG_S = 10.0: 2.8x the longest mode beat (modes 0.7155 / 0.9949 /
# 1.6396 Hz beat at 1.082 / 1.551 / 3.578 s, analysis/ringdown.md), 5.6-13x under
# tau 56-130 s, so a real decay falls 7.4-16% across it against 1.5-3.6% at v9's
# 2.0 s, which read the beat instead.
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 10.0, 1.02

# ===== soft-saturation =====
# Clipping removes authority in ONE direction and the coil still pulls the other
# way at full strength, so the fault needs both halves: pinned for SAT_FRACTION of
# a SAT_SUSTAIN_S window AND the envelope not falling, SAT_DECAY_FRAC mirroring
# RUNAWAY_GROWTH_FRAC over the same lag and env_hist. Do NOT add a clip_excess
# condition; see _actuate().
# SECONDS, not samples: 30 consecutive samples meant 0.41 / 1.28 / 2.40 s on 1 / 4
# / 8 coils (73.0 / 23.5 / 12.5 Hz) and one clean sample erased the evidence, so
# this NEVER FIRED in the 240 s three-kick bench run of 2026-08-06. 1.30 s
# preserves four-coil behaviour (30 x 42.6 ms = 1.278 s); 0.80 is the rail
# check's own fraction.
SAT_SUSTAIN_S, SAT_FRACTION = 1.30, 0.80
SAT_DECAY_FRAC = 0.98

# ===== baseline-floor =====
# Smallest baseline a channel may calibrate and still be trusted, as a FRACTION of
# the median across enabled channels. Relative because the floor scales with the
# lab and with alignment (a5 reads the same motion at ~1/12 of a0's gain, two
# independent measurements 2026-08-06); 0.10 because all eight are on ONE rigid
# body, so gains spread in-band RMS by a factor of a few, not 10x. Bench brackets
# (`v55_all8.log`, fractions of the median): a4 0.010 / a7 0.014 / a6 0.036
# demoted, a5 0.579 kept. A dead channel costs the whole rig, so bias to KEEPING.
BASELINE_FLOOR_FRAC = 0.10

# ===== bias trim (from v7) =====
# No OSEM rests at mid-scale: 600 / 631 / 708 / 677 counts against 511.5 at bias
# 0.25 V, so every channel clips its TOP rail first and ch2 has least room. Coil
# bias is a DC force; on hardware 2026-08-04 the trim took total offset
# 565 -> 455 counts at no cost in lock time. NOT a one-shot least-squares solve,
# which wants -1.93..+3.77 V against a 2.5 V DAC. One quantum at a time, kept if
# the TOTAL offset improved, four coils driving two DOF (v6 rank check: 2
# directions above 10%), so revert-on-worse costs one step, not a railed sensor.
BIAS_QUANTUM = 0.25            # coarse: 2 DOF, 4 knobs, so fine steps would
BIAS_MIN, BIAS_MAX = 0.25, 1.25          # just chase each other
BIAS_SWING = 0.25              # +-this around each channel's own bias
MID_COUNTS = 511.5             # (ADC_MAX_COUNTS - 1) / 2
TRIM_PERIOD_S = 15.0           # >= one ringdown at Q~50, f0~1 Hz (~16 s)
TRIM_DEADBAND_COUNTS = 40.0    # inside this, leave it alone
TRIM_MAX_STEPS = 4             # per channel, per run
# Both of these are from the 2026-08-07 epsilon bench run, and both are there
# because that run destabilised the rig by trimming.
#
# The quiet gate. The trim steps, then judges the TOTAL offset a period later.
# Both readings come from `counts_mean`, so both are meaningless unless the optic
# is near rest. On 08-07 it stepped and judged straight through a runaway: ch0
# measurably got WORSE at every step (582 -> 636 -> 605 counts) while the totals
# it compared said 1229 -> 1213 -> 1151, so it "kept" three steps on pure noise
# and walked ch0 further out each time. Same line LOCKED uses, for the same
# reason: it is the threshold at which this rig is known to be at rest. The gate
# is LOCKED itself rather than a bare ratio, so it also inherits LOCK_SUSTAIN_S
# and cannot be fooled by an RMS window that a fault recovery has just reset.
# TRIM_MAX_TOTAL_EXCURSION_V. Per-channel limits alone do not bound the DC force
# on the optic, and the force is what moves the flag out of the linear
# partial-shadow region where the sensor gain stops being what the loop assumes.
# Measured 08-07: ch3 alone at 1.00 V (0.75 V of excursion) held lock at ratios
# 0.01-0.08 for 25 s. Adding ch0 at 0.50 V, total 1.00 V, gave three runaways in
# 40 s. So 0.75 permits the state that was proven good and refuses the state that
# broke it. It is a measured bound, not a safety margin, and it should be
# re-derived if the flags are ever realigned.
TRIM_MAX_TOTAL_EXCURSION_V = 0.75
# A bias step is a force step, ringing the pendulum at ~1 Hz right where the
# estimator lives, which is the band both TREND tests read, so it INVALIDATES the
# history: env_hist cleared, excess_since reset, trend halves resuming within
# RUNAWAY_TREND_LAG_S while the level halves keep working. `warm-restart` refuses
# a pre-step baseline.
# SLOPE_SIGN moves a channel's bias to reduce its counts,
#   step = -sign(error) * SLOPE_SIGN * BIAS_QUANTUM
# from the DIAGONAL of the per-coil DC matrix, one coil at a time, 2026-08-04
# (`bench/20260804/dcmatrix.log`), counts/V:
#     a0 -105   a1  -68   a2 +213   a3  -51    solidly measured
#     a4   +8   a5   -7   a6   +4   a7   +7    at that block's noise, mean 4
# ch2 is +1 because it is mounted the other way round. Do NOT use a common-mode
# sweep: four coils together read a3 as +43 counts/V, coil3 -> a3 alone -51.
SLOPE_SIGN = np.array([-1.0, -1.0, +1.0, -1.0,
                       +1.0, -1.0, +1.0, +1.0])

# Rail: raw counts, so there is no float-rounding ambiguity, and a FRACTION of a
# window rather than an unbroken run, so one noise sample dilutes the evidence
# instead of erasing it. Read at the WIRE rate, never through the estimator.
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80

# 5 s confirming a lock already had, same budget argument as LOCK_SUSTAIN_S and
# deferred for the same reason.
FAULT_CLEAR_SUSTAIN_S = 5.0
# Timed from rail-clear, and it now covers the ESTIMATOR's acquisition: measured
# settle 1.48 s after a x10 kick at T_AMP = 50 (analysis/out/kalman_sweep_*.csv),
# and _health restarts that channel's filter the moment its rail clears. delta
# justified the same 2.0 s as 5 tau of a 0.4 Hz highpass that no longer exists.
REARM_SUSTAIN_S = 2.0
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md §3)

# ===================== MODAL (MIMO) =====================
# Phi and A are MEASURED and are NOT in this file. They arrive in data/modal.json,
# written by `status.py --save-modal`. With no file, an unparseable file, a stale
# file or one that fails the checks below, this controller runs epsilon's DIAGONAL
# law and says which of those it was.
#
# THAT REFUSAL IS THE DEFAULT PATH, NOT A BRANCH. `versions.md` records why: v13
# carried a modal law, defined its refusal as `_unused_main`, never called it, and
# `make run V=v13` would have driven the optic from a file whose own docstring
# said it must not. A modal law with hard-coded matrices is that defect again with
# better numbers, so there are no matrices here to hard-code.
MODAL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "data", "modal.json")
MODAL_SCHEMA = "osem-modal-1"

# Per-mode Kp, applied to the modal velocity. Comparable to the diagonal |Kp| =
# 0.035 BY CONSTRUCTION, because Phi's columns and A's rows are normalised to unit
# 2-norm over the surviving sets at load time: q_dot is then a weighted mean of
# sensor velocities in V/s, the same units and roughly the same size as the `vel`
# the diagonal law multiplies, and the min-norm command comes back in coil volts.
# That normalisation is what makes one scale factor meaningful and is the reason
# it exists -- Phi's and A's own scales are arbitrary per mode.
MODAL_KP = np.array([0.035, 0.035, 0.035])
# FIRST LIGHT IS AT HALF. Kp = -0.040 is the documented instability / rail onset
# and it is the least-evidenced number in the repo (CLAUDE.md item 7): one
# sentence in a deleted docstring, and the simulator provably cannot reproduce it.
# The modal law's whole point is that every sensor now reaches every coil, so a
# gain that was safe diagonally is not thereby safe here. Raise it on the bench,
# with a scope, in its own run.
MODAL_GAIN_SCALE = 0.5
# Hard ceiling on |u| the modal allocator may ask for, before the per-channel clip
# sees it. The clip and `soft-saturation` are still there and unchanged; this
# bounds the ALLOCATION, so one badly-conditioned mask cannot demand 10 V and let
# the clip quietly turn it into a permanent rail.
MODAL_DEMAND_CAP_V = 0.20

# The gate, per surviving-coil mask. cond is a BACKSTOP, not the criterion --
# mimo_closed.md Sec 4.4 shows why a count or a cond alone is the wrong statistic.
MODAL_COND_MAX = 12.0
# A sensor row counts toward the modal estimate only if status.py determined it at
# every mode. Partial rows are the a1-2026-08-06 failure and are worse than absent:
# an undetermined entry is free to set the allocation at that mode.
MODAL_MIN_SENSORS = 2        # per mode; the estimate is a scalar LS, not an inverse
MODAL_MIN_COILS = 3          # rank(A_C) must reach the mode count
# Refuse an A that was not driven. `status.py --derive-a-colocation` can produce
# one from Phi and the DC matrix under the colocation identity, which is useful for
# testing the loop off the bench and is NOT a measurement of A.
MODAL_ALLOW_PROVISIONAL_A = os.environ.get("OSEM_MODAL_PROVISIONAL", "0") not in ("", "0")
# Modal data older than this is refused: the whole reason for status.py's pulse is
# that these numbers drift.
MODAL_MAX_AGE_S = 7 * 24 * 3600.0

# ===== when the fault path may borrow the baseline it already has =====
# An engagement of >= FAST_REFAULT_S resets the reuse budget, the loop having just
# validated that baseline; MAX_BASELINE_REUSE bounds reuses without one and
# BASELINE_MAX_AGE_S is the wall-clock backstop. See _why_reuse.
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0

# ===== persist-baseline =====
# The floor across process restarts. Anchored to THIS FILE's directory, not the
# cwd: `bench.py`, `make run` and a bare `python osem.epsilon.py` differ.
BASELINE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "data", "baseline.json")
BASELINE_FILE_SCHEMA = 1
# 6x the in-run BASELINE_MAX_AGE_S: 300 s bounds a floor a just-FAULTED loop leans
# on, 1800 s bounds "same lab session", 96x shorter than the two days over which
# the bench floor moved 2.0-2.4x.
BASELINE_FILE_MAX_AGE_S = 1800.0
# Measured, so a tolerance and not equality. 1.5x is above link jitter and below
# every real change: 12.5 -> 18 -> 23.5 -> 1111 Hz are all >= 1.4x apart.
WIRE_RATE_TOL = 1.5
# A MEASUREMENT, not a fingerprint: the loaded floor must agree per channel with
# the first BASELINE_WARMUP_S to within this. Same-session reproducibility is
# 1.21-1.35x, a moved flag 12x (a5 vs a0) to 200x. Biased toward REFUSING.
BASELINE_SANITY_RATIO = 3.0
# Consecutive fresh-baseline refusals before taking the fresh number anyway: a low
# floor only makes the breaker over-cautious, so this just bounds a livelock.
MAX_BASELINE_REFUSALS = 3
# As REARM_SUSTAIN_S, and on the same new justification: the estimator's 1.48 s
# acquisition, not a highpass time constant. It also covers RAIL_SUSTAIN_S = 0.5 s,
# arming the rail interlock before any coil moves.
BASELINE_WARMUP_S = 2.0

STATUS_PERIOD_S, CSV_FLUSH_EVERY_N = 5.0, 200

# ===== kick-cue =====
# The operator is at the optic, not the terminal, so the schedule gets spoken.
# `say` is macOS-only and its absence is silent. OFF BY DEFAULT: `make check` runs
# 200-odd scenarios. NON-BLOCKING (Popen): `say` takes ~700 ms, 70 missed steps at
# 100 Hz with the coils held where they were.
#     OSEM_KICK_CUE=25 make run V=epsilon      # cue every 25 s once damping
KICK_CUE_S = float(os.environ.get("OSEM_KICK_CUE", "0") or 0)
KICK_CUE_PHRASE = "jerk it"
_KICK_CUE_LEAD_S = 3.0          # first cue this long after DAMPING, not instantly


def kick_cue(phrase=KICK_CUE_PHRASE):
    """Speak `phrase` without blocking. Any failure is silent and harmless."""
    try:
        subprocess.Popen(["say", phrase],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, ValueError):
        pass                    # no `say` here

# ===== gain-ramp: measuring the Kp ceiling instead of asserting it =====
# CLAUDE.md item 7. `Kp = -0.040 is the onset of instability/rail` is one sentence
# in a deleted docstring, written two days before a report that never mentions it,
# and NO run has ever gone there: over 21 closed-loop runs the maximum gain ever
# commanded is exactly CAPTURE_GAIN, 0.035, on every channel (analysis/kp040.md,
# gain_census.csv). The number is outside the data, not under-sampled. The
# simulator cannot settle it either: it is linear apart from the ADC and sweeps to
# -0.600 with damping increasing monotonically and zero faults.
#
# So the ceiling has to be walked up on hardware. What makes that a measurement
# rather than a dare is that instability has a SIGNATURE that arrives before the
# rail does. Kp acting on velocity is a dashpot, and a dashpot cannot destabilise
# anything on its own -- so more gain must mean less residual motion. It stops
# meaning that when loop phase lag carries the feedback past quadrature, and from
# there more gain PUMPS. The turning point in `ratio` against |Kp| is the onset,
# and it is a number with a source.
#
# The ceiling is NOT one number and must not be quoted as one. Phase margin is
# 73.5 deg on four coils and 45.5 on eight, and the -0.040 claim was made with a
# single coil driving. epsilon's Kalman estimator should buy some of that back
# (velocity error 29.3% -> 0.6%, analysis/kalman.md), which would RAISE its
# ceiling above delta's. Each configuration owns its own measurement.
#
# OFF BY DEFAULT and it must stay that way: `make check` runs 200-odd scenarios
# and none of them should be walking a real optic toward instability.
#     OSEM_GAIN_RAMP=0.035:0.060:0.0025:20 make run V=epsilon
# is start:stop:step:dwell_s. ATTENDED ONLY, with a scope on the coil drive.
GAIN_RAMP_HARD_CAP = 0.060      # refuses to parse past this, whatever is asked
GAIN_RAMP_SETTLE_FRAC = 0.40    # of each dwell, discarded: the slew limit is live
GAIN_RAMP_RISE_TRIPS = 2        # consecutive rises in `ratio` that end the ramp
GAIN_RAMP_RAIL_FRAC = 0.02      # of a level's samples pinned, on any driven channel
# There is deliberately NO demand ceiling. An earlier version tripped on peak
# demand against the 0.250 V half-window, which is wrong twice over: it read the
# CLIPPED output, which saturates at the half-window by construction, and even
# measured correctly the raw demand already runs at 0.85 V p99 at the SHIPPING
# gains (kalman_headroom.csv). This loop lives in clipping; a demand threshold
# would have ended every ramp on its first level and measured nothing. Pathological
# clipping is already covered, and better, by the `soft-saturation` interlock,
# which faults on an output that is pinned AND not winning. Demand is reported per
# level as a diagnostic instead.


def _parse_gain_ramp(spec):
    """`start:stop:step:dwell_s` -> a level list and a dwell, or None.

    Refuses rather than clamps. A ramp that silently ran to a different ceiling
    than the one asked for would produce a number nobody could source, which is
    the exact failure this whole tool exists to correct.
    """
    if not spec:
        return None
    parts = spec.split(":")
    if len(parts) != 4:
        raise ValueError("OSEM_GAIN_RAMP wants start:stop:step:dwell_s, got %r" % spec)
    start, stop, step, dwell = (float(p) for p in parts)
    if not (0 < start <= stop) or step <= 0 or dwell <= 0:
        raise ValueError("OSEM_GAIN_RAMP needs 0 < start <= stop, step > 0, "
                         "dwell > 0, got %r" % spec)
    if stop > GAIN_RAMP_HARD_CAP:
        raise ValueError("OSEM_GAIN_RAMP stop %.4f exceeds the %.3f hard cap. "
                         "Raise GAIN_RAMP_HARD_CAP deliberately, in a diff, with "
                         "a reason." % (stop, GAIN_RAMP_HARD_CAP))
    levels, v = [], start
    while v <= stop + 1e-12:
        levels.append(round(v, 6))
        v += step
    return levels, dwell


class GainRamp:
    """Walks |Kp| up one level at a time and stops at the first sign of trouble.

    Scales the WHOLE shipping gain vector, so per-channel signs and the measured
    relative shape are untouched and the only thing moving is overall loop gain.
    That matters: the shape is not re-openable (2 observable degrees of freedom
    against 4 free gains leaves a 2-dimensional family that damps identically),
    but the magnitude has never been measured at all.

    Only advances while DAMPING. A level interrupted by a fault is discarded
    rather than averaged, because the gains freeze during FAULT and the samples
    would describe a rig that was not being driven.

    Four ways to stop, and the first one is the actual measurement:
      1. `ratio` rose for GAIN_RAMP_RISE_TRIPS consecutive levels. More gain is
         now producing more motion, which is the phase-lag crossover.
      2. a driven channel spent over GAIN_RAMP_RAIL_FRAC of the level railed.
      3. peak demand passed GAIN_RAMP_DEMAND_V of the 0.250 V half-window.
      4. the controller faulted during the level.
    On any of them the gain returns to the last level that passed and stays
    there. It does not abort the run: the point is to keep damping at the
    highest gain that was actually shown to work.
    """

    def __init__(self, levels, dwell_s, nominal):
        self.levels, self.dwell = levels, dwell_s
        # What the level list is expressed in: the largest magnitude in the
        # shipping vector. A level of 0.040 means "this vector, scaled so its
        # biggest entry is 0.040".
        self.nominal = float(np.max(np.abs(nominal))) or 1.0
        self.i, self.t0, self.armed = 0, None, False
        self.acc, self.rises, self.done, self.why = [], 0, False, None
        self.history = []            # (level, mean ratio, peak demand, rail frac)
        self.ceiling = None

    def scale(self):
        return self.levels[self.i] / self.nominal

    def _summarise(self):
        if not self.acc:
            return None
        r = np.array([a[0] for a in self.acc])
        return (float(np.nanmean(r)), max(a[1] for a in self.acc),
                float(np.mean([a[2] for a in self.acc])))

    def stop(self, why, back_to=None):
        """Freeze the ramp at `back_to`, defaulting to one level down.

        `self.i` is what `scale()` reads, so it has to be the level actually
        landed on. An earlier version announced a back-off in the message while
        leaving `self.i` alone, which both left the rig running at the gain that
        had just failed and recorded a ceiling one level above the real one.
        """
        self.done, self.why = True, why
        self.i = max(0, self.i - 1 if back_to is None else back_to)
        self.ceiling = self.levels[self.i]
        return why

    def sample(self, t, ratio, demand_v, railed, faulted):
        """One control step. Returns a message to announce, or None."""
        if self.done:
            return None
        if faulted:
            # Discard the level and retry it: a fault mid-level says nothing
            # about the gain that was being tested.
            self.acc, self.t0 = [], None
            return None
        if self.t0 is None:
            self.t0, self.acc = t, []
            return (f"[gain-ramp] level {self.i + 1}/{len(self.levels)}: "
                    f"|Kp| = {self.levels[self.i]:.4f}, holding {self.dwell:.0f}s")
        if t - self.t0 < self.dwell * GAIN_RAMP_SETTLE_FRAC:
            return None                     # slew limit still moving the gain
        self.acc.append((ratio, demand_v, railed))
        if t - self.t0 < self.dwell:
            return None

        got = self._summarise()
        if got is None:
            self.t0 = None
            return None
        mean_ratio, peak_demand, rail_frac = got
        self.history.append((self.levels[self.i], mean_ratio, peak_demand, rail_frac))
        head = (f"[gain-ramp] |Kp| = {self.levels[self.i]:.4f}: ratio "
                f"{mean_ratio:.3f}, peak demand {peak_demand:.3f} V, railed "
                f"{rail_frac * 100:.1f}%")

        if rail_frac > GAIN_RAMP_RAIL_FRAC:
            return head + "\n" + self.stop(
                f"railed {rail_frac * 100:.1f}% of the level, over the "
                f"{GAIN_RAMP_RAIL_FRAC * 100:.0f}% limit")
        if len(self.history) >= 2 and mean_ratio > self.history[-2][1]:
            self.rises += 1
            if self.rises >= GAIN_RAMP_RISE_TRIPS:
                # Back to the last level BEFORE the run of rises started, which
                # is the minimum of the curve and the answer being measured.
                back = max(0, self.i - GAIN_RAMP_RISE_TRIPS)
                why = self.stop(
                    f"ratio rose {GAIN_RAMP_RISE_TRIPS} levels running "
                    f"({self.history[-1 - GAIN_RAMP_RISE_TRIPS][1]:.3f} -> "
                    f"{mean_ratio:.3f}). More gain is making more motion: this "
                    f"is the phase-lag crossover, and the level below it is the "
                    f"measured ceiling", back_to=back)
                return (head + "\n" + why
                        + f"\n[gain-ramp] holding at {self.levels[self.i]:.4f}")
        else:
            self.rises = 0

        if self.i + 1 >= len(self.levels):
            return head + "\n" + self.stop(
                "reached the top of the requested range with no turning point. "
                "The ceiling is ABOVE this, not at it", back_to=self.i)
        self.i += 1
        self.t0 = None
        return head

    def summary(self):
        out = ["", "=" * 68, "  GAIN RAMP", "=" * 68,
               "  |Kp|      ratio    peak demand   railed"]
        for lv, r, d, f in self.history:
            out.append(f"  {lv:.4f}   {r:6.3f}   {d:9.3f} V   {f * 100:5.1f}%")
        if self.ceiling is not None:
            out += [f"\n  Highest |Kp| that held: {self.ceiling:.4f}",
                    f"  Stopped because: {self.why}"]
        else:
            out.append("\n  Ramp did not complete a level. Nothing measured.")
        out += ["", "  This ceiling belongs to THIS configuration: this many coils",
                "  driven, this velocity estimator, this wire rate. It is not a",
                "  property of the optic and does not transfer to another build.", ""]
        return "\n".join(out)


GAIN_RAMP = _parse_gain_ramp(os.environ.get("OSEM_GAIN_RAMP", "").strip())

_LOG = (("bp", ".5f"), ("vel", ".5f"), ("out", ".4f"), ("gain", ".5f"),
        ("ratio", ".4f"), ("p", ".5f"), ("i", ".5f"), ("d", ".5f"))
# `ctl` marks the row that completed a control step, `n_avg` how many. See csv_row.
# Run-level modal columns, after `n_avg` and before the per-channel block, the
# same place delta put `ctl`/`n_avg`. `mres*` is the out-of-mode residual, which
# nothing acts on: the first bench run exists to record its distribution so a
# threshold can be derived rather than guessed (mimo_closed.md Sec 4.5).
_MODAL_COLS = (["modal", "mmask", "mrank"]
               + [f"q{m}" for m in range(len(F_MODE_HZ))]
               + [f"kmode{m}" for m in range(len(F_MODE_HZ))]
               + [f"mres{m}" for m in range(len(F_MODE_HZ))])
CSV_HEADER = "time_s,state,ctl,n_avg," + ",".join(_MODAL_COLS) + "," + ",".join(
    f"ch{i}_{c}" for i in range(N)
    for c in ("counts", "V") + tuple(k for k, _ in _LOG) + ("rail", "locked", "healthy"))
# ====================================================


class Modal:
    """Phi and A, loaded from disk, validated, and turned into a per-mask table.

    ------------------------------------------------------------------------
    THE SENSING SIDE IS NOT A PSEUDO-INVERSE, AND THAT IS THE WHOLE CHANGE
    ------------------------------------------------------------------------

    `analysis/mimo_closed.md` Sec 3.2 projects a BROADBAND per-channel velocity
    through `Phi_S+`, and Secs 4.2-4.3 are then mostly about the damage that
    does: the sensing channels fall into two clusters degenerate to under 2.5
    degrees, so `a0,a2` comes out at cond 64 and `a0,a5` singular, and 256
    cached pseudo-inverses exist to survive it.

    That problem is an artefact of asking the SENSOR SET to separate the modes.
    It does not have to. `KalmanVelocity.modal_velocity()` already returns an
    (N, 3) array -- per channel, PER MODE -- because the estimator carries one
    oscillator state per mode and separates them in frequency, exactly, at the
    measured f_m. epsilon shipped that and nothing read it.

    So for mode m the measurement model is scalar:

        v[i, m]  =  Phi[i, m] * q_dot_m  +  noise_i

    and the estimate is an inverse-variance weighted least squares over the
    surviving sensors,

        q_dot_m  =  SUM_i w_im Phi_im v_im  /  SUM_i w_im Phi_im^2 ,
        w_im     =  1 / sigma_im^2

    There is no matrix to invert, no conditioning to degrade, and no way for one
    mode to leak into another through the estimator. Sensor diversity now buys
    SNR only -- it is no longer load-bearing for identifiability. `a0,a2`, the
    cond-64 case, is simply two noisy estimates of the same three scalars.

    THE ALLOCATION SIDE STILL NEEDS AN INVERSE, and that is where the mask table
    survives: `u_C = A_C+ f`, `A_C` is 3 x |C|, and dropping coils really can
    make it rank-deficient. All 2^8 coil masks are inverted once here, at
    start-up, and the control step does one lookup and one matrix-vector
    product. No matrix arithmetic in the loop, constant worst-case step cost.

    ------------------------------------------------------------------------
    WHAT IT REFUSES, AND WHY EACH REFUSAL IS SEPARATE
    ------------------------------------------------------------------------

    Every refusal below returns a REASON, and the controller prints it and runs
    diagonal. They are separate because "no file" and "the file is for a
    different rig" want different actions from whoever is at the bench.
    """

    def __init__(self, path=MODAL_PATH, n=N, nm=None, now=None):
        self.path, self.n = path, n
        self.nm = len(F_MODE_HZ) if nm is None else nm
        self.ok = False
        self.why = "not loaded"
        self.source = ""
        self.created = ""
        self.phi = np.zeros((n, self.nm))
        self.sigma = np.full((n, self.nm), np.inf)
        self.rowok = np.zeros((n, self.nm), bool)
        self.a = None
        self.a_coils = []
        self.imag_frac = [float("nan")] * self.nm
        self.notes = []
        self._load(now)

    # ---- loading ---------------------------------------------------------
    def _refuse(self, why):
        self.ok, self.why = False, why
        return False

    def _load(self, now=None):
        if self.path is None:
            return self._refuse("no modal path given -- this Controller was "
                                "built without one, which is the default "
                                "everywhere except main()")
        if not os.path.exists(self.path):
            return self._refuse("no %s -- run `status.py phi --save-modal`"
                                % os.path.basename(self.path))
        try:
            with open(self.path) as fh:
                d = json.load(fh)
        except (OSError, ValueError) as e:
            return self._refuse("%s is unreadable (%s)"
                                % (os.path.basename(self.path), e))
        if d.get("schema") != MODAL_SCHEMA:
            return self._refuse("schema is %r, this file wants %r"
                                % (d.get("schema"), MODAL_SCHEMA))
        if int(d.get("n_sensors", -1)) != self.n or int(d.get("n_modes", -1)) != self.nm:
            return self._refuse("measured for %s sensors x %s modes, this file is "
                                "%d x %d" % (d.get("n_sensors"), d.get("n_modes"),
                                             self.n, self.nm))
        # The frequencies are the one thing Phi is meaningless without: a shape
        # measured at 0.9949 Hz says nothing about a mode that has moved to 0.99.
        # epsilon's docstring already asks for these to be re-measured each
        # session and this is where that becomes enforceable rather than advice.
        fs = [m.get("f_hz") for m in d.get("modes", [])]
        if len(fs) != self.nm or any(
                abs(float(a) - float(b)) > 0.005 for a, b in zip(fs, F_MODE_HZ)):
            return self._refuse("measured at %s Hz, this file runs at %s Hz -- "
                                "re-measure, or the shapes do not apply"
                                % ([round(float(x), 4) for x in fs],
                                   [round(float(x), 4) for x in F_MODE_HZ]))
        self.created, self.source = d.get("created", ""), d.get("source", "")
        age = self._age(self.created, now)
        if age is not None and age > MODAL_MAX_AGE_S:
            return self._refuse("measured %.1f days ago (limit %.1f) -- these "
                                "numbers drift, re-run status.py"
                                % (age / 86400.0, MODAL_MAX_AGE_S / 86400.0))

        p = d.get("phi") or {}
        try:
            self.phi = np.asarray(p["value"], float).reshape(self.n, self.nm)
            self.sigma = np.abs(np.asarray(p["sigma"], float)).reshape(self.n, self.nm)
            self.rowok = np.asarray(p["ok"], bool).reshape(self.n, self.nm)
        except (KeyError, ValueError, TypeError) as e:
            return self._refuse("phi block is malformed (%s)" % e)
        self.imag_frac = [float(x) for x in p.get("imag_frac", [float("nan")] * self.nm)]
        for mi, f in enumerate(self.imag_frac):
            if f == f and f > 0.35:
                self.notes.append("mode %d shape is %.0f%% imaginary -- no real mode "
                                  "shape explains that; treat it as provisional"
                                  % (mi, 100.0 * f))

        a = d.get("a")
        if a is None:
            return self._refuse("phi is present but A is not -- run "
                                "`status.py coils --save-modal`. Sensing is "
                                "solved; allocation is not.")
        try:
            self.a = np.asarray(a["value"], float).reshape(self.nm, self.n)
            self.a_coils = [int(c) for c in a.get("coils", [])]
        except (KeyError, ValueError, TypeError) as e:
            return self._refuse("a block is malformed (%s)" % e)
        if a.get("provisional") and not MODAL_ALLOW_PROVISIONAL_A:
            return self._refuse("A is PROVISIONAL (%s) and was never driven. Set "
                                "OSEM_MODAL_PROVISIONAL=1 to run on it anyway, "
                                "which is a test configuration, not a bench one."
                                % a.get("provenance", "?"))
        self.provisional_a = bool(a.get("provisional"))

        # Normalise so MODAL_KP means what its comment says. Per mode, over the
        # channels that mode actually has: an unmeasured row contributes 0 and
        # must not be allowed to change the scale.
        for m in range(self.nm):
            s = np.linalg.norm(self.phi[self.rowok[:, m], m])
            if s > 0:
                self.phi[:, m] /= s
                self.sigma[:, m] /= s
            s = np.linalg.norm(self.a[m, self.a_coils]) if self.a_coils else 0.0
            if s > 0:
                self.a[m] /= s
        # --- the colocation sign check --------------------------------------
        # Phi and A must come from ONE factorisation. Each mode's Phi and A are
        # determined only up to a shared scale and sign, so a Phi from the
        # passive run paired with an A from the driven one has an INDEPENDENT
        # sign per mode -- and a wrong sign PUMPS, it does not under-damp.
        # `save_modal` writes both from one run and stamps `gauge`; this refuses
        # a pair that was assembled by hand.
        if (p.get("gauge") or "") != (a.get("gauge") or ""):
            return self._refuse("phi and A come from different measurements "
                                "(%r vs %r). Their per-mode signs are then "
                                "independent, and a wrong sign pumps. Re-run "
                                "`status.py coils --save-modal`, which writes "
                                "both from one factorisation."
                                % (p.get("gauge"), a.get("gauge")))

        # The physical cross-check, and it is independent of the gauge: sensor
        # and coil are the same OSEM head, so A[m,j] = lambda_j * Phi[j,m] with
        # ONE lambda per channel (mimo_closed.md Sec 3.2). lambda_j's SIGN must
        # therefore be the same at every mode. Where it is not, the pair is
        # inconsistent on that channel and the channel is dropped from the
        # allocator rather than trusted -- this is the same test Sec 2.3 passes
        # on ch0-ch3 with `sign(P_0j) = sign(P_1j)`.
        keep = []
        for j in self.a_coils:
            lam = [self.a[m, j] / self.phi[j, m]
                   for m in range(self.nm)
                   if self.rowok[j, m] and abs(self.phi[j, m]) > 1e-12]
            if len(lam) < 2:
                self.notes.append("coil %d: too few determined modes to check its "
                                  "colocation sign; kept, unverified" % j)
                keep.append(j)
            elif all(x > 0 for x in lam) or all(x < 0 for x in lam):
                keep.append(j)
            else:
                self.notes.append(
                    "coil %d DROPPED: A/Phi has inconsistent sign across modes "
                    "(%s). Sensor and coil are one head, so that ratio is one "
                    "constant per channel; disagreeing signs mean Phi and A "
                    "disagree, and a wrong sign pumps."
                    % (j, ", ".join("%+.3f" % x for x in lam)))
        if len(keep) < MODAL_MIN_COILS:
            return self._refuse("only %d coil(s) pass the colocation sign check "
                                "(need %d): %s"
                                % (len(keep), MODAL_MIN_COILS, self.notes[-1:]))
        self.a_coils = keep

        self._build_tables()
        self.ok = True
        self.why = "loaded"
        return True

    @staticmethod
    def _age(created, now=None):
        if not created:
            return None
        try:
            t = datetime.fromisoformat(created)
        except ValueError:
            return None
        ref = datetime.now() if now is None else now
        if t.tzinfo is not None:
            t = t.replace(tzinfo=None)
        return max((ref - t).total_seconds(), 0.0)

    def _build_tables(self):
        """One A_C+ per coil mask, plus the gate verdict, computed once.

        2^8 = 256 masks, each a 3 x |C| pseudo-inverse. ~30 kB and a few ms at
        start-up, against a data-dependent pseudo-inverse inside a 100 Hz loop.
        `mimo_closed.md` Sec 4.2 is the source of this and it is still right on
        the allocation side even though the sensing side no longer needs it.
        """
        self.pinv = [None] * (1 << self.n)
        self.cond = [float("inf")] * (1 << self.n)
        self.gate = [False] * (1 << self.n)
        self.rank = [0] * (1 << self.n)
        drivable = np.zeros(self.n, bool)
        drivable[self.a_coils] = True
        for mask in range(1 << self.n):
            cols = [j for j in range(self.n)
                    if (mask >> j) & 1 and drivable[j]]
            if len(cols) < MODAL_MIN_COILS:
                continue
            Ac = self.a[:, cols]
            U, sv, Vh = np.linalg.svd(Ac, full_matrices=False)
            if sv[0] <= 1e-9:
                continue
            self.cond[mask] = float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else float("inf")

            # TRUNCATE, DO NOT REFUSE. This used to compute a full pinv and then
            # veto the whole mask when cond exceeded MODAL_COND_MAX, which threw
            # the loop back to diagonal on ALL THREE modes because ONE modal
            # direction was badly reachable. Keeping the directions that ARE
            # reachable and dropping only the rest damps two modes modally
            # instead of none, which is strictly more damping for strictly less
            # actuator demand.
            #
            # `keep` is by conditioning against the LARGEST singular value, which
            # is what bounds the volts: a direction at sv[0]/sv[k] = 50 costs 50x
            # the command for the same modal force, and that is the number the
            # 0.250 V half-window cannot pay.
            keep = int(np.sum(sv >= sv[0] / MODAL_COND_MAX))
            keep = min(keep, int(np.linalg.matrix_rank(Ac, tol=1e-6)))
            if keep < 1:
                continue
            Uk, svk, Vhk = U[:, :keep], sv[:keep], Vh[:keep, :]

            # Weighted min-norm over the kept directions: mimo_closed.md Sec 3.4.
            # W is per-coil headroom, constant here because the window is; making
            # it track LIVE headroom would put a nonlinearity inside the loop and
            # is refused.
            P = (Vhk.conj().T * (1.0 / svk)) @ Uk.conj().T
            # The projector onto the commandable modal subspace. The loop applies
            # it to the modal VELOCITY before the gain, not to the force after it.
            # That ordering is what keeps the law dissipative: the realised modal
            # force is then -Proj K Proj qdot, and Proj K Proj is symmetric
            # positive semidefinite for any positive gain vector, so
            # qdot . f <= 0 always. Projecting the force instead gives
            # -Proj K qdot, which is NOT symmetric unless every gain is equal,
            # and can pump.
            self.pinv[mask] = (cols, np.real(P), np.real(Uk @ Uk.conj().T))
            self.rank[mask] = keep
            self.gate[mask] = keep >= 1

    # ---- the loop reads these -------------------------------------------
    def sense(self, vmodal, sense_ok):
        """(q_dot, residual) from the (N, nm) per-mode velocities.

        `residual` is the part of each mode's sensor pattern that is NOT
        proportional to Phi's column, normalised. No rigid-body motion can
        produce it, so a sustained non-zero is sensors DISAGREEING -- a bad
        sensor that is neither railed nor signal-less and is therefore invisible
        to every other interlock. It is computed and logged and NOT ACTED ON:
        there is no measured threshold, and adding an unmeasured fault source to
        a rig whose fault history is the main thing wrong with it is a bad trade
        (mimo_closed.md Sec 4.5). The first bench run records the distribution.

        Per MODE, not global, which is better than the design asked for: with
        |S| sensors determined at a mode the check has |S|-1 degrees of freedom
        instead of |S|-nm, so it survives down to two sensors.
        """
        q = np.zeros(self.nm)
        r = np.zeros(self.nm)
        nseen = np.zeros(self.nm, int)
        for m in range(self.nm):
            use = sense_ok & self.rowok[:, m]
            k = int(use.sum())
            nseen[m] = k
            if k < MODAL_MIN_SENSORS:
                continue
            phi = self.phi[use, m]
            v = vmodal[use, m]
            w = 1.0 / np.maximum(self.sigma[use, m], 1e-12) ** 2
            den = float(np.sum(w * phi * phi))
            if den <= 0:
                continue
            q[m] = float(np.sum(w * phi * v)) / den
            res = v - phi * q[m]
            nv = float(np.linalg.norm(v))
            r[m] = float(np.linalg.norm(res) / nv) if nv > 0 else 0.0
        return q, r, nseen

    def mask_of(self, drive_ok):
        """Coil-mask for a boolean per-coil vector. Bit j is coil j."""
        mask = 0
        for j, b in enumerate(drive_ok):
            if b:
                mask |= 1 << j
        return mask

    def project(self, qdot, drive_ok):
        """(commandable part of qdot, rank). Apply BEFORE the gain, never after.

        With rank reduction the allocator can only push in `rank` of the `nm`
        modal directions. Zeroing the modal velocity in the directions it cannot
        reach means the gain never asks for a force that will not be delivered,
        and -- the part that matters -- it keeps the law dissipative: the realised
        force becomes -Proj K Proj qdot, and Proj K Proj is symmetric PSD for any
        positive gain vector, so the modal power qdot . f is never positive.
        Applying the gain first and projecting the force gives -Proj K qdot,
        which is not symmetric unless all the gains are equal and can inject
        energy into the modes it drops.
        """
        mask = self.mask_of(drive_ok)
        e = self.pinv[mask]
        if not self.gate[mask] or e is None:
            return np.zeros(self.nm), 0
        return e[2] @ np.asarray(qdot, float), self.rank[mask]

    def allocate(self, f, drive_ok):
        """(u, mask, ok). Modal force 3-vector -> coil volts. One table lookup."""
        # Bit j is coil j. Written out rather than with np.packbits, which was
        # here first and had the array reversed: the mask then indexed a
        # DIFFERENT valid table entry, so it failed the gate and returned a zero
        # command instead of raising -- the loop just quietly stopped being
        # modal. Eight shifts at 100 Hz is not worth a subtlety.
        mask = 0
        for j, b in enumerate(drive_ok):
            if b:
                mask |= 1 << j
        if not self.gate[mask] or self.pinv[mask] is None:
            return np.zeros(self.n), mask, False
        cols, P, Proj = self.pinv[mask]
        u = np.zeros(self.n)
        # Project again here so `allocate` is correct on its own terms even if a
        # caller hands it an unprojected force: Proj is idempotent, so this is a
        # no-op on a force the loop already projected via `project()`.
        u[cols] = P @ (Proj @ f)
        # Bound the ALLOCATION, not just the output. The per-channel clip is
        # still below this and unchanged; this stops one poorly-conditioned mask
        # demanding a command the clip would silently turn into a standing rail.
        pk = float(np.max(np.abs(u))) if u.size else 0.0
        if pk > MODAL_DEMAND_CAP_V:
            u *= MODAL_DEMAND_CAP_V / pk
        return u, mask, True

    def report(self):
        """What preflight prints. The file about to run stays the thing you read."""
        out = ["[modal] %s" % ("LOADED" if self.ok else "REFUSED -- running DIAGONAL")]
        out.append("[modal] %s" % self.why)
        if self.source:
            out.append("[modal] source: %s" % self.source)
        if self.created:
            out.append("[modal] measured: %s" % self.created)
        for n in self.notes:
            out.append("[modal] NOTE: %s" % n)
        if not self.ok:
            return out
        rows = [i for i in range(self.n) if self.rowok[i].all()]
        out.append("[modal] Phi rows determined at every mode: %s" % rows)
        out.append("[modal] Phi (unit-norm columns), sigma below each:")
        for i in range(self.n):
            out.append("[modal]   a%d  %s   %s"
                       % (i, " ".join("%+8.4f" % x for x in self.phi[i]),
                          "".join("y" if x else "." for x in self.rowok[i])))
        out.append("[modal] A rows (unit-norm), coils %s:" % self.a_coils)
        for m in range(self.nm):
            out.append("[modal]   mode %d  %s"
                       % (m, " ".join("%+8.4f" % x for x in self.a[m])))
        full = int(sum(1 << j for j in self.a_coils))
        out.append("[modal] full coil set %s: cond %.2f, gate %s"
                   % (self.a_coils, self.cond[full],
                      "PASS" if self.gate[full] else "FAIL"))
        for j in self.a_coils:
            m2 = full & ~(1 << j)
            out.append("[modal]   drop coil %d -> cond %.2f, %s"
                       % (j, self.cond[m2], "MIMO" if self.gate[m2] else "diagonal"))
        npass = sum(1 for g in self.gate if g)
        out.append("[modal] %d of %d coil masks run MIMO; the rest fall back to "
                   "epsilon's diagonal law, which is a degradation and not a fault."
                   % (npass, 1 << self.n))
        out.append("[modal] Kp per mode %s x scale %.2f = %s"
                   % (np.round(MODAL_KP, 4), MODAL_GAIN_SCALE,
                      np.round(MODAL_KP * MODAL_GAIN_SCALE, 4)))
        return out


class KalmanVelocity:
    """Per-channel (SISO) velocity estimator: three undamped oscillators at the
    MEASURED frequencies plus one random-walk DC state. One filter per channel, no
    cross-channel term anywhere. Lifted from analysis/kalman.py, which designed and
    replayed it; the numbers are that file's.

        x    = [q_A, v_A, q_B, v_B, q_C, v_C, b]     per channel, in sensor volts
        y    = q_A + q_B + q_C + b + n               n ~ N(0, R_i)
        vhat = v_A + v_B + v_C

    vhat excludes b BY CONSTRUCTION. That is what replaces the 0.4 Hz highpass: the
    estimate is DC-free because the DC lives in a state the output does not read,
    not because a filter removed it, which is why Ki has nothing left to do.

    NOT MIMO. q_m is a per-channel quantity, the projection of mode m onto sensor i
    in sensor units, not a modal coordinate. No Phi across channels, nothing
    inverted, and dropping a channel changes nothing about any other filter.
    Colocated coaxial sensor and coil, one OSEM head per channel, is what makes
    that legitimate.

    UNDAMPED, and that is an assumption: tau > 138 s at 1 sigma
    (analysis/ringdown.md) against a closed loop settling in 2.9-4.7 s, so over one
    filter time constant the intrinsic decay is under 1.5% and the data cannot
    constrain a damping term. The process noise carries the amplitude changes.

    THE PROPERTY THE DESIGN RESTS ON. At exactly f_m the estimator is EXACT, unity
    gain and zero phase error against a true differentiator, for ANY Q and ANY R: a
    pure sinusoid at f_m is an exact zero-process-noise trajectory of the model and
    the error dynamics (I - K H) Phi are stable, so the estimate converges to the
    true state. That is why f_m measured to +-0.0005 Hz unblocked this and why Q
    being uncertain does not."""

    def __init__(self, K=KALMAN_K, n=N, f=F_MODE_HZ, dt=CONTROL_PERIOD_S):
        self.n, self.dt, self.f = n, dt, np.asarray(f, float)
        self.w = 2.0 * np.pi * self.f
        self.nm = len(self.f)
        self.nx = 2 * self.nm + 1
        self.K = np.asarray(K, float).reshape(n, self.nx)
        self.H = np.zeros(self.nx)
        self.H[0:2 * self.nm:2] = 1.0
        self.H[-1] = 1.0
        self.Cv = np.zeros(self.nx)
        self.Cv[1:2 * self.nm:2] = 1.0
        self.Phi = self._transition(dt)
        self.x = np.zeros((n, self.nx))
        self.primed = np.zeros(n, bool)

    def _transition(self, dt):
        """Exact ZOH, not Euler: each 2x2 block is the rotation
        [[cos th, sin th / w], [-w sin th, cos th]] at th = w*dt."""
        Phi = np.zeros((self.nx, self.nx))
        for m, w in enumerate(self.w):
            th = w * dt
            c, s = np.cos(th), np.sin(th)
            Phi[2 * m:2 * m + 2, 2 * m:2 * m + 2] = [[c, s / w], [-w * s, c]]
        Phi[-1, -1] = 1.0
        return Phi

    def update(self, y, dt=None, valid=None):
        """One control step. `y` is (n,) volts, the SAME decimated mean `_control`
        receives. `valid` is (n,) bool, False meaning do not trust this channel's
        sample: predict only and coast on the model (`rail-blank`, OFF by default).
        Returns vhat (n,) in V/s.

        Passing the MEASURED `dt` rebuilds Phi, three cos and three sin in total
        shared by every channel, so control-period jitter costs ~10 us and is
        absorbed exactly rather than ignored. K stays at its design value."""
        Phi = (self.Phi if dt is None or abs(dt - self.dt) < 1e-9
               else self._transition(dt))
        y = np.asarray(y, float)
        cold = ~self.primed
        if cold.any():
            # Prime the DC state on the first sample rather than ramping from zero,
            # the way OnePole primes a lane coming back.
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

    # == free, having paid for the states ==================================
    def displacement(self):
        """In-band displacement in V: `bp`'s replacement, and NOT the same signal a
        bandpass produced, which is why every stored baseline is refused."""
        return self.x[:, 0:2 * self.nm:2].sum(axis=1)

    def acceleration(self):
        """d(vhat)/dt in closed form, V/s^2: a_m = -w_m^2 q_m. No difference
        quotient, so a D term built on this would cost no noise at all. It is still
        effective mass and still dissipates nothing. Nothing here reads it yet."""
        return -(self.x[:, 0:2 * self.nm:2] * self.w ** 2).sum(axis=1)

    def modal_velocity(self):
        """(n, 3) per-mode velocity: what a per-mode resonant gain would need, and
        what a bandpass cannot produce at any price. Nothing here reads it yet."""
        return self.x[:, 1:2 * self.nm:2]


class OnePole:
    """One-pole low/high pass over N independent lanes. `on` is per lane because a
    lane whose D-term filter was reset must re-prime from its first sample rather
    than ramp up from zero. Only the D term uses this now."""

    def __init__(self, hz, kind="low", n=N):
        self.tau, self.low = 1.0 / (2.0 * np.pi * hz), kind == "low"
        self.y, self.xp, self.on = np.zeros(n), np.zeros(n), np.zeros(n, bool)

    def update(self, x, dt):
        new = ~self.on
        y0 = np.where(new, x if self.low else 0.0, self.y)
        a = dt / (self.tau + dt) if self.low else self.tau / (self.tau + dt)
        nxt = (y0 + a * (x - y0) if self.low
               else a * (y0 + x - np.where(new, x, self.xp)))
        self.y, self.xp = np.where(new, y0, nxt), x.copy()
        self.on[:] = True
        return self.y

    def reset(self, m=slice(None)):
        self.y[m], self.xp[m], self.on[m] = 0.0, 0.0, False


class SlidingRMS:
    """RMS over a time window, scalar or array. The controller keeps one per lane
    so a single channel's window can be cleared without resizing the others."""

    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = window_s, deque(), 0.0

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        # np.maximum is not padding: the sum is incremental, so a transient ageing
        # out leaves a residue of its own magnitude, measured at -4.5e-14 after a
        # 6 V/s kick. sqrt() of that is nan, every comparison against nan is False,
        # and the breaker would stop tripping and the lock detector stop firing.
        return (np.sqrt(np.maximum(self.sq, 0.0) / len(self.buf)) if self.buf
                else self.sq * 0.0)

    def reset(self):
        self.buf.clear()
        self.sq = 0.0


def _bank(window_s):
    return [SlidingRMS(window_s) for _ in range(N)]


def _rms(bank, t, x, m):
    """A lane outside `m` is not fed at all, so a blind channel's flatlined
    estimate never enters its window."""
    return np.array([bank[i].update(t, x[i]) if m[i] else 0.0 for i in range(N)])


class Actuator:
    """DAC writes for all N coils: 0.5 mV deadband, with the CONTROL STEP as the
    throttle. No second write throttle: one on time.time() would race the loop's
    own 10 ms period and drop a semi-random half of the writes. The budget fits,
    8 coils x 100 Hz x ~15 bytes = 12 kB/s against 50 kB/s at 500000 baud (the
    CLAUDE.md item 2c budget, written against 115200's 11.5 kB/s).

    ValueError is deliberately NOT caught: FastDAC raises it for a channel outside
    0..7 or a voltage outside 0..2.5 V, neither reachable since `self.out` is a
    slew-limited walk inside [0.0, 1.5] V, so it would mean a bug in the clip."""

    def __init__(self, dac, channels):
        self.dac, self.ch = dac, list(channels)
        self.t, self.v = np.zeros(len(self.ch)), np.full(len(self.ch), np.nan)

    def send(self, volts):
        now = time.time()
        due = np.isnan(self.v) | (abs(volts - self.v) >= 0.0005)
        for i in np.nonzero(due)[0]:
            try:
                self.dac.set_voltage(channel=self.ch[i], voltage=float(volts[i]))
                self.t[i], self.v[i] = now, volts[i]
            except RuntimeError:
                pass


def in_range(counts):
    """`sample-guard`: a 10-bit converter cannot produce a count outside 0..1023,
    so a row carrying one is a framing error. A free function because it must mean
    the same thing in `read_sample()`, at the wire, and in `Controller.step()`,
    which sim/server.py and harness.py reach without going through the parser."""
    return bool(np.all((counts >= 0) & (counts <= ADC_MAX_COUNTS)))


def read_sample(ser):
    """(counts, volts) as two length-N arrays, or None. arduino.ino streams eight
    columns, so a four-column firmware never satisfies `len(parts) >= N` and this
    returns None forever, which is the loud failure you want. The board's
    `OK ch=.. v=..` replies are nobody's now and arrive interleaved with data rows;
    the first-character test drops those.

    `sample-guard` is the rest: TEN rows in 241,813 of v11's bench log carry a count
    outside 0..1023 (5659, 7575, 65690, 522676, 730608) against ZERO in each of the
    four slower logs, two fields spliced by a buffer boundary so they parse cleanly.
    counts = 522676 is +45.6 V of step into any velocity estimator. Replayed on that
    log against delta's chain the guard takes ch0 rms 13.109 -> 5.413 V/s and peak
    1441 -> 41 V/s, while decimating WITHOUT it gives 23.4 V/s; the estimator here
    is narrowband but a step is broadband, so the guard is not less necessary."""
    if not ser.in_waiting:
        return None
    try:
        raw = ser.readline().decode("utf-8").strip()
        if not raw or raw[0] not in "0123456789-":
            return None                    # OK / ERR / STREAMING / junk
        parts = raw.split(",")
        if len(parts) < N:
            return None
        counts = np.array([int(p) for p in parts[:N]], dtype=float)
        if not in_range(counts):
            read_sample.rejected += 1      # torn row: a framing error, not data
            return None
        return counts, counts * (A_VCC / ADC_MAX_COUNTS)
    except (ValueError, IndexError):
        return None


# Counted here, the only place that sees the wire. Ten in 241,813 rows on v11's
# log; a climbing count means the link is losing framing.
read_sample.rejected = 0


# Channel attribute -> Controller array.
_VIEW = dict(zip("enabled healthy bias steady_gain capture_gain ki kd bp_out vel out "
                 "active_gain last_ratio p_term i_term d_term rail_fault locked "
                 "saturated_flag baseline_rms".split(),
                 "enabled healthy bias steady capture ki kd bp vel out "
                 "gain ratio p i d rail locked sat baseline".split()))


class Channel:
    """One lane of the controller's arrays. State lives in length-N arrays, but the
    logger, the status line and sim/server.py read and write
    `ctl.channels[i].<field>`, including live gain edits from the browser UI."""

    def __init__(self, ctl, idx):
        object.__setattr__(self, "ctl", ctl)
        object.__setattr__(self, "idx", idx)

    def __getattr__(self, k):
        if k in _VIEW:
            return getattr(self.ctl, _VIEW[k])[self.idx]
        raise AttributeError(k)

    def __setattr__(self, k, v):
        if k in _VIEW:
            getattr(self.ctl, _VIEW[k])[self.idx] = v
        else:
            object.__setattr__(self, k, v)


class Controller:
    """The fast-lock state machine. main() drives it from the serial stream and
    sim/server.py drives the same class from a simulated plant, so there is no
    second implementation of the loop."""

    def __init__(self, dac, dac_channels=None, enable=None, steady=None,
                 capture=None, ki=None, kd=None, bias=None, baseline_file=None,
                 modal_path=None):
        f = lambda v, d: np.array(d if v is None else v, dtype=float)
        self.enabled = np.array(ENABLE_CHANNEL if enable is None else enable, bool)
        self._pin_hist = []          # dead_pin's rolling window
        self._pin_said = np.zeros(N, bool)
        self.steady, self.capture = f(steady, STEADY_GAIN), f(capture, CAPTURE_GAIN)
        self.ki, self.kd, self.bias = f(ki, KI_GAIN), f(kd, KD_GAIN), f(bias, BIAS)
        self.act = Actuator(dac, DAC_CHANNELS if dac_channels is None else dac_channels)

        # One estimator, eight independent filters, K shipped as a constant.
        self.kf = KalmanVelocity()
        # Phi and A, or the reason there are none.
        #
        # `modal_path=None` means NO MODAL DATA, and that is the default so that
        # the suite and the simulator are REPRODUCIBLE: with the default reading
        # MODAL_PATH, `make check` would score differently depending on whether
        # somebody happened to have run status.py, which is exactly the kind of
        # hidden input this repo has been bitten by. Only `main()` -- the bench
        # path -- passes the real path, so opting in to the modal law is
        # explicit and lives in one place.
        #
        # The simulator could not test it anyway: `sim/server.py` models TWO
        # modes (SHAPE_8 is 8x2) and this file runs three. `--selftest` covers
        # the modal math instead, and says so.
        self.modal = Modal(modal_path)
        self.nmode = self.modal.nm
        self.qdot = np.zeros(self.nmode)          # modal velocity, V/s
        self.mode_gain = np.zeros(self.nmode)     # scheduled K_m, >= 0
        self.modal_res = np.zeros(self.nmode)     # out-of-mode residual, logged only
        self.mode_seen = np.zeros(self.nmode, int)
        self.modal_on = False                     # did THIS step run the modal law?
        self.modal_mask = 0
        self.modal_rank = 0
        self.modal_u = np.zeros(N)
        self.modal_says = None                    # one-shot reason, printed once
        self.dfilt = OnePole(D_SMOOTH_HZ)
        self.rms_sch, self.rms_run = _bank(SCHEDULE_WINDOW_S), _bank(ENVELOPE_WINDOW_S)
        self.env_hist = deque()          # (t, envelope) for the trend test
        self.env_valid_t = 0.0           # trend test is blind until the window refills
        self.rms_lock = _bank(LOCK_WINDOW_S)

        for k in ("bp vel p i d prev_vel clip_excess gain ratio baseline "
                  "sat_sum rail_sum").split():
            setattr(self, k, np.zeros(N))
        for k in "primed sat locked rail".split():
            setattr(self, k, np.zeros(N, bool))
        self.healthy = np.ones(N, bool)
        # Per-channel output window; the trim moves each channel's bias alone.
        self.vmin, self.vmax = self.bias - BIAS_SWING, self.bias + BIAS_SWING
        self.clear_t, self.excess_since = np.full(N, np.inf), np.full(N, np.inf)
        self.locked_since = np.full(N, np.inf)
        self.out, self.prev_out = self.bias.copy(), self.bias.copy()
        # `sat_hist` is the saturation counterpart of `rail_hist`: (t, pinned) over
        # the last SAT_SUSTAIN_S, a fraction of a window rather than a run.
        self.rail_hist, self.sat_hist = deque(), deque()
        self.calib, self.events = [], []
        self.channels = [Channel(self, i) for i in range(N)]
        self.state, self.calib_start, self.damping_start = "CALIBRATING", 0.0, None
        self.clear_since = self.lock_time = None
        self.locked_announced = False
        self.fault_count = self.demote_count = self.reuse_count = 0
        self.damped_for = None       # how long DAMPING lasted before the fault
        # ===== fast-calib bookkeeping =====
        self.sub_rms = []            # one RMS vector per COMPLETED sub-window
        self.sub_start, self.sub_n0 = 0.0, 0
        self.calib_took = None
        self.calib_early = False     # converged, or hit the ceiling
        # Kept so a LATER calibration can be checked against it. See _set_baseline.
        self.trusted_baseline = None
        self.baseline_refused = False
        self.refusals = 0
        # Held on the instance, not the module, so the suite's thousands of
        # Controllers cannot share one ramp's state. `base_steady`/`base_capture`
        # are the unscaled vectors: the ramp multiplies these rather than
        # compounding on its own previous output.
        self.ramp = (GainRamp(*GAIN_RAMP, nominal=self.steady)
                     if GAIN_RAMP else None)
        self.base_steady, self.base_capture = self.steady.copy(), self.capture.copy()
        # ===== warm-restart bookkeeping =====
        self.baseline_t = None       # when the live baseline was measured
        self.fault_railed = False    # was the fault a SENSOR failure?
        # ===== baseline-floor bookkeeping =====
        # Separate from the rail demotion: a rail clears and re-arms on a timer, a
        # signal-less sensor does not, and only a new baseline overturns it.
        self.floor_bad = np.zeros(N, bool)
        self.floor_count = 0
        # ===== bias-trim bookkeeping: trim_ref is the total offset judged against =====
        self.trim_steps = np.zeros(N, int)
        self.trim_frozen = np.zeros(N, bool)
        self.trim_last = None
        self.trim_pending = None       # (channel, previous_bias) awaiting judgement
        self.trim_budget_said = False   # announce the excursion budget once, not per period
        self.trim_ref = None           # total |counts - MID| before that step
        self.counts_mean = np.full(N, MID_COUNTS)
        self.trim_step_t = None
        # ===== decimate bookkeeping =====
        # A running SUM and a count, not a list: touched at the WIRE rate, ~1111 Hz,
        # where appending would allocate a thousand times a second.
        self.acc_c, self.acc_v, self.acc_n = np.zeros(N), np.zeros(N), 0
        self.ctl_t = None            # NOMINAL deadline; None = no step run yet
        self.ctl_prev = None
        self.ctl_dt = CONTROL_PERIOD_S
        self.stepped = False
        self.n_avg = 0
        self.ctl_steps = 0
        self.bad_samples = 0         # sample-guard rejections seen here
        self.wire_n, self.wire_t0 = 0, None      # for the measured wire rate
        # `mains-null`: the previous RAW control mean, so this step can average two.
        self.prev_mean_c = self.prev_mean_v = None
        # ===== persist-baseline bookkeeping =====
        # A loaded floor is held HERE, not in self.baseline: it is not adopted
        # until BASELINE_WARMUP_S has measured something to check it against.
        self.file_baseline = self.file_floor = None
        self.file_note = ""
        if baseline_file is not None:
            self.file_baseline = np.asarray(baseline_file[0], float)
            self.file_floor = np.asarray(baseline_file[1], bool)
        self.warm_used = False       # did this run engage on a file baseline?
        self.baseline_saveable = False   # main() reads and clears this

    # ===== helpers =====
    def drain_events(self):
        out, self.events = self.events, []
        return out

    def _say(self, msg):
        self.events.append("\n" + msg + "\n")

    # Sub-LSB. A live ADC line always carries at least +/-1 count of dither, so a
    # channel whose reading has not moved AT ALL across a calibration window is
    # not a sensor at a stop, it is a pin at a rail. Measured 2026-08-17 over
    # three records totalling 236 387 samples: a5 takes exactly ONE distinct
    # value, 0.0 counts, variance exactly zero, while a0-a4 in the same records
    # ran std 41-90 counts. There is no ambiguity to tune around here; the
    # threshold only has to be under one count.
    DEAD_PIN_SPAN_COUNTS = 1.0
    DEAD_PIN_WINDOW = 64          # samples; ~0.6 s at 100 Hz control

    def dead_pin(self, volts):
        """Which channels are at a rail AND not moving at all -- i.e. unwired.

        Kept deliberately narrow: it returns True only for channels that are
        ALREADY railed, so it cannot demote a healthy quiet channel. A real rail
        excursion clips and still wanders; a dead pin does not move.
        """
        v = np.asarray(volts, float)
        self._pin_hist.append(v)
        if len(self._pin_hist) > self.DEAD_PIN_WINDOW:
            self._pin_hist.pop(0)
        if len(self._pin_hist) < 8:
            return np.zeros(N, bool)
        H = np.asarray(self._pin_hist, float)
        span = (H.max(axis=0) - H.min(axis=0)) / (A_VCC / ADC_MAX_COUNTS)
        flat = span < self.DEAD_PIN_SPAN_COUNTS
        out = self.rail & flat
        for i in np.where(out & ~self._pin_said)[0]:
            self._say(f"[pin] ch{i} is railed and has not moved {span[i]:.2f} "
                      f"counts in {len(self._pin_hist)} samples -- treating it as "
                      f"an UNWIRED PIN, not a plant excursion. It is demoted, and "
                      f"it will not fault the rig. Fix the wiring or set "
                      f"ENABLE_CHANNEL[{i}] = False.")
            self._pin_said[i] = True
        return out

    def _fault(self, t, msg, railed=False):
        self._say("!! " + msg)
        # A fault with a trim step in flight condemns that step. It cannot be
        # judged -- the offsets are now the fault's, not the bias's -- and on
        # 2026-08-07 every one of three runaways followed a ch0 trim step inside
        # 5 s. Put it back and freeze the channel: twice is a pattern, and the
        # quiet gate alone would let it retry the same step after the recovery.
        if self.trim_pending is not None:
            i, was = self.trim_pending
            self.trim_pending = None
            self._moved(t, i, was)
            self.trim_frozen[i] = True
            self._say(f"[trim] ch{i} reverted to {was:.2f}V and frozen -- the rig "
                      f"faulted with that step in flight.")
        # Only an engagement that happened counts: damping_start read
        # unconditionally reports the PREVIOUS one after a rail during CALIBRATING.
        self.damped_for = (t - self.damping_start
                           if self.state == "DAMPING" and self.damping_start is not None
                           else None)
        self.fault_railed = bool(railed)
        self.state, self.clear_since = "FAULT", None
        self.fault_count += 1

    @staticmethod
    def _who(m):
        return ", ".join(f"ch{i}" for i in np.nonzero(m)[0])

    @staticmethod
    def _hold(cond, since, t):
        """Per-lane stopwatch: start where `cond` just became true, clear where it
        is false. np.inf is 'not running'."""
        return np.where(cond, np.where(np.isinf(since), t, since), np.inf)

    def _rail_check(self, counts, t):
        """RAW COUNTS, at the full wire rate. Never routed through the estimator: a
        railed sensor flatlines whatever you filter, and an RMS-only check reads a
        flatline as perfect stability."""
        railed = (counts <= RAIL_LOW_COUNTS) | (counts >= RAIL_HIGH_COUNTS)
        self.rail_hist.append((t, railed))
        self.rail_sum = self.rail_sum + railed
        while self.rail_hist and t - self.rail_hist[0][0] > RAIL_SUSTAIN_S:
            self.rail_sum = self.rail_sum - self.rail_hist.popleft()[1]
        # A full window must have elapsed, or the first samples trip it on n = 1.
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= RAIL_SUSTAIN_S * 0.9
        self.rail = np.logical_and(
            spanned, self.rail_sum / max(len(self.rail_hist), 1) >= RAIL_FRACTION)

    def _sat_check(self, pinned, t):
        """`sat-window`. `_rail_check` with `pinned` for `railed`, deliberately
        line-for-line the same, down to the 0.9 x window `spanned` guard. Maintained
        in EVERY state: in FAULT the outputs sit at bias and the window drains
        within SAT_SUSTAIN_S, so nothing latches."""
        self.sat_hist.append((t, pinned))
        self.sat_sum = self.sat_sum + pinned
        while self.sat_hist and t - self.sat_hist[0][0] > SAT_SUSTAIN_S:
            self.sat_sum = self.sat_sum - self.sat_hist.popleft()[1]
        spanned = bool(self.sat_hist) and t - self.sat_hist[0][0] >= SAT_SUSTAIN_S * 0.9
        self.sat = np.logical_and(
            spanned, self.sat_sum / max(len(self.sat_hist), 1) >= SAT_FRACTION)

    # ===== calibration =====
    def _close_subwindow(self, t):
        """Fold the samples since the last boundary into one RMS vector. The raw
        samples are KEPT as well: the ceiling path re-splits the whole window the
        way v5 does, and must give v5's answer."""
        a = np.asarray(self.calib[self.sub_n0:])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a ** 2).mean(0)))
        self.sub_n0, self.sub_start = len(self.calib), t

    def _calib_stationary(self):
        """Has the noise floor stopped moving? TWO tests, both needed.

        (1) the trailing CALIB_AGREE_N sub-windows agree with each other.
        (2) their median agrees with the median of every sub-window so far. A
            ringdown is slow (tau ~ 16 s, comparable to the whole window), so three
            sub-windows part-way down it look stationary while sitting far from the
            floor: with (1) alone a 6 V/s kick at t = 2 s reads as settled at
            t = 16.01 s (trailing three agree to 1.187) and stores a baseline skewed
            37.0%, past the 35% the suite asserts. Test (2) sees those windows 1.400
            from the median of the whole and runs on, for 17.6%."""
        w = self.sub_rms
        if len(w) < max(CALIB_MIN_SUBWINDOWS, CALIB_AGREE_N):
            return False
        a = np.asarray(w)
        tail = a[-CALIB_AGREE_N:]
        if not bool((tail.max(0) / np.maximum(tail.min(0), 1e-9)
                     <= CALIB_AGREE_TOL).all()):
            return False
        m_tail, m_all = np.median(tail, 0), np.median(a, 0)
        spread = (np.maximum(m_tail, m_all)
                  / np.maximum(np.minimum(m_tail, m_all), 1e-9))
        return bool((spread <= CALIB_AGREE_TOL).all())

    def _set_baseline(self, early):
        """`early` picks the estimator: the ceiling branch is v5's, median of
        CALIB_SUBWINDOWS sub-window RMSs over the whole window; the early branch
        takes the median of the trailing windows just shown to agree, not of
        everything, the first sub-window still carrying the estimator's acquisition
        transient (1.48 s, measured)."""
        if early and len(self.sub_rms) >= CALIB_AGREE_N:
            fresh = np.maximum(
                np.median(np.asarray(self.sub_rms[-CALIB_AGREE_N:]), 0), 1e-6)
        else:
            a = np.asarray(self.calib)
            parts = ([p for p in np.array_split(a, CALIB_SUBWINDOWS) if len(p)]
                     if len(a) else [])
            fresh = (
                np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0), 1e-6)
                if parts else np.full(N, 1e-6))

        # Is this a FLOOR, or a ringdown mistaken for one? BENCH 2026-08-06: after
        # a kick and four re-faults a forced calibration exited EARLY at 10.0 s on
        # "sub-windows agreed" while the optic still rang down, measuring
        # ch0 = 0.6112 V against a quiet 0.0530 V, 11.5x (ch1 13.4x, ch2 8.3x,
        # ch3 15.5x, ch5 10.5x). CALIB_AGREE_TOL tests STATIONARITY, not QUIETNESS:
        # at tau ~ 16 s a ringdown decays ~12% across a 2 s sub-window, inside the
        # 1.20 tolerance. Only the HIGH side is refused.
        ref = self.trusted_baseline
        self.baseline_refused = False
        if ref is not None:
            # NOT `~floor_bad`, which belongs to the PREVIOUS calibration since
            # `_baseline_floor` runs after this: measured 2026-08-06, reading it
            # here refused a whole calibration on ch4 (0.0163V vs 0.0028V, 5.9x)
            # and ch6 (0.0664V vs 0.0031V, 21.3x), whose trusted value is noise.
            live = ref > 0
            med = np.median(ref[live]) if live.any() else 0.0
            m = live & (ref >= med * BASELINE_FLOOR_FRAC)
            bad = m & (fresh > BASELINE_SANITY_RATIO * ref)
            # Refusing keeps a floor that may be too LOW, making the breaker
            # hypersensitive, so refuse/fault/recalibrate is a real livelock. A low
            # floor is the SAFE direction, so the bound is generous.
            if bad.any() and self.refusals >= MAX_BASELINE_REFUSALS:
                self._say(
                    f"!! fresh baseline refused {self.refusals} times running and "
                    "accepted anyway -- the floor really has moved, this is no "
                    "longer a ringdown. The runaway breaker is now scaled off it.")
                bad = np.zeros(N, bool)
            if bad.any():
                self.refusals += 1
                self.baseline_refused = True
                self._say(
                    "!! fresh baseline REFUSED -- "
                    + " / ".join(f"ch{i} measured {fresh[i]:.4f}V against a trusted "
                                 f"{ref[i]:.4f}V ({fresh[i] / ref[i]:.1f}x)"
                                 for i in np.nonzero(bad)[0])
                    + f", over {BASELINE_SANITY_RATIO:.1f}x. That is a ringdown, not "
                      "a floor -- a calibration window can be perfectly STATIONARY "
                      "and still sit far above the floor. Keeping the baseline "
                      "already in hand.")
                # Restore EXPLICITLY from `ref`: entering CALIBRATING clears
                # `self.baseline`, and without this it stayed all zeros with the
                # breaker denominator-less (measured 2026-08-06).
                self.baseline = np.maximum(ref, 1e-6)
                self.calib, self.sub_rms, self.sub_n0 = [], [], 0
                return

        self.baseline = fresh
        self.trusted_baseline = fresh.copy()
        self.refusals = 0
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0

    def _start_calibration(self, t):
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_start = self.sub_start = t

    def _warm_from_file(self, t):
        """`persist-baseline`, the run-time half. Preflight has already accepted the
        FILE on schema, age and a fingerprint of every gain, bias, mode frequency,
        rate and scale factor. What that cannot answer is whether this is the same
        OPTIC in the same alignment in the same room, so the loop measures for
        BASELINE_WARMUP_S before closing and the loaded floor must agree with what
        it sees, per channel, to within BASELINE_SANITY_RATIO.

        On refusal nothing is thrown away: the warm-up samples stay in `self.calib`,
        so the next step takes the ordinary `fast-calib` path with a head start.
        Channels the FILE demoted stay demoted and are excluded."""
        if t - self.calib_start < BASELINE_WARMUP_S:
            return
        a = np.asarray(self.calib)
        # Trailing 60% only: the first ~0.8 s is the estimator acquiring, which
        # reads high over a 2 s window.
        a = a[int(len(a) * 0.4):]
        seen = np.sqrt((a ** 2).mean(0)) if len(a) >= 2 else np.zeros(N)
        loaded, floor = self.file_baseline, self.file_floor
        self.file_baseline = None                 # decided either way, once
        m = ~floor & (loaded > 0)
        ratio = np.where(m, np.maximum(seen, 1e-9) / np.maximum(loaded, 1e-9), 1.0)
        bad = m & ((ratio > BASELINE_SANITY_RATIO)
                   | (ratio < 1.0 / BASELINE_SANITY_RATIO))
        if not m.any() or bad.any():
            self._say("!! stored baseline REFUSED at the warm-up check -- "
                      + (f"{self._who(bad)} measured "
                         + " / ".join(f"{seen[i]:.4f}V against a stored "
                                      f"{loaded[i]:.4f}V ({ratio[i]:.2f}x)"
                                      for i in np.nonzero(bad)[0])
                         if bad.any() else
                         "the file demoted every channel, so there is nothing "
                         "left to check it against")
                      + f", outside {BASELINE_SANITY_RATIO:.1f}x. Something in "
                      f"the rig or the room has moved. Measuring a fresh floor "
                      f"the long way (up to {CALIBRATION_S:.0f}s).")
            return
        self.baseline = np.maximum(loaded, 1e-6)
        # A warm start gets a cold start's guard: this floor is now the reference.
        self.trusted_baseline = self.baseline.copy()
        self.baseline_t = t
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_took, self.calib_early = t - self.calib_start, True
        self.reuse_count = 0
        self.floor_bad = floor.copy()
        self.healthy[floor], self.gain[floor] = False, 0.0
        self.clear_t[floor] = np.inf
        self.warm_used = True
        self.state, self.damping_start = "DAMPING", t
        self._arm_envelope(t)
        self.locked_announced = False
        self._say(f"[DAMPING] engaged on the STORED baseline after {self.calib_took:.1f}s "
                  f"of warm-up (a full calibration is {CALIBRATION_S:.0f}s): "
                  + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                  + ". Measured over the warm-up: "
                  + ", ".join(f"ch{i}={seen[i]:.4f}V" for i in np.nonzero(m)[0])
                  + f" -- worst disagreement {np.abs(np.log(ratio[m])).max():.2f} in log, "
                    f"inside the {BASELINE_SANITY_RATIO:.1f}x window."
                  + (f" {self._who(floor)} stays demoted -- the file says they "
                     f"were not measuring the optic." if floor.any() else "")
                  + f" Gain schedules {CAPTURE_GAIN[0]:+.3f} -> {STEADY_GAIN[0]:+.3f}, "
                    f"ramping from zero.")

    def _baseline_floor(self, t):
        """Demote any channel whose freshly-measured baseline is implausibly small
        NEXT TO THE OTHERS. Runs once, at the end of every calibration.

        The failure is a sensor that is powered, reading, not railed and not
        measuring the optic: a flag outside the partial shadow where a shadow sensor
        is linear. Its estimated in-band displacement is its own electronics noise,
        so it calibrates near zero, and every downstream test is a RATIO against
        that number:

            gain schedule    ratio = env / baseline    -> pinned to CAPTURE_GAIN
            lock detector    env < LOCK_RMS_FACTOR x baseline  -> never satisfied
            runaway breaker  env > RUNAWAY_MULTIPLE x baseline -> FAULTS THE RIG

        The third cost ten faults in 145 s on the bench (v5.5, 2026-08-04).

        The median reference (see BASELINE_FLOOR_FRAC) buys a safety property: the
        largest enabled baseline is >= the median > the threshold, so at least one
        enabled channel always survives and no more than half can ever be demoted.
        With most of the rig signal-less it does NOTHING, the median being itself a
        dead channel, which is intended.

        EVERY channel is tested, not just enabled ones: a config-disabled channel is
        still watched by the runaway breaker, and on the bench it was disabled
        channels that faulted the rig."""
        ref = self.baseline[self.enabled]
        if ref.size == 0:
            return
        med = float(np.median(ref))
        bad = self.baseline < med * BASELINE_FLOOR_FRAC
        self.floor_bad = bad
        if not bad.any():
            return
        # Demoted as a rail demotes: unhealthy, gain zeroed, output parked at bias.
        self.healthy[bad], self.gain[bad], self.clear_t[bad] = False, 0.0, np.inf
        self.floor_count += int(bad.sum())
        self._say("!! " + self._who(bad) + " demoted -- baseline "
                  + " / ".join(f"{self.baseline[i]:.4f}" for i in np.nonzero(bad)[0])
                  + f"V, under {BASELINE_FLOOR_FRAC:.0%} of the {med:.4f}V median "
                  f"across enabled channels. That sensor is not measuring the "
                  f"optic, so a ratio against it is a ratio against nothing -- "
                  f"held at bias and out of the interlocks. "
                  f"{int((self.enabled & self.healthy).sum())}/"
                  f"{int(self.enabled.sum())} still damping.")

    def _arm_envelope(self, t):
        """Drop the envelope history and hold the trend test off until the RMS
        window behind it has actually refilled.

        Called at every DAMPING entry, which means the first engagement and every
        fault recovery. Two separate faults, both measured on 2026-08-07:

        1. `_recover` resets `rms_run` but left `env_hist` standing. So `env`
           restarted from an empty 2 s window while `past` was still a PRE-FAULT
           entry 10-20 s old -- and those entries were themselves recorded while
           the window was filling at the previous DAMPING entry, so they were
           tiny. `growing` was therefore true on the first step after every
           recovery, `high` was true because the optic was genuinely ringing, and
           RUNAWAY_SUSTAIN_S = 2.0 s did the rest. Re-faults landed at 2.1, 2.2
           and 2.8 s, and the envelope was FALLING through all of them: ch3 went
           21.13 -> 8.13 -> 4.96 -> 2.51 across four consecutive "runaways".
           `_moved` already cleared the history for a bias step, with a comment
           saying why; a fault plus a gain ramp from zero invalidates it at least
           as much.

        2. Clearing alone is not enough. With an empty history `past` defaults to
           `env` and nothing can trip, but ENVELOPE_WINDOW_S later the oldest
           entry is the near-zero envelope of a window that had just been reset,
           and comparing against THAT reads as growth no matter what the optic is
           doing. So no sample is recorded until the window is full and carries
           real data.

        This is the same defect `runaway-trend` was written to fix, arriving
        through the recovery path rather than the level test. A ringdown must
        never read as a runaway, whatever route it takes.
        """
        self.env_hist.clear()
        self.env_valid_t = t + ENVELOPE_WINDOW_S

    def _health(self, t):
        """Demote a railed channel; re-arm REARM_SUSTAIN_S after its rail clears.
        Only RAIL demotes: a saturated channel measures fine and only its output is
        clipped, so parking it would remove authority at peak amplitude. Baseline
        carries across the gap, the short RMS windows do not."""
        drop = self.healthy & self.rail
        self.healthy[drop], self.gain[drop] = False, 0.0   # so gain ramps from zero
        self.clear_t[self.rail] = np.inf
        # `~floor_bad` is what makes `baseline-floor` stick: the re-arm path is
        # written for a rail, an event that ENDS, and a signal-less sensor never
        # railed, so the guard would otherwise hand it back every 2 s forever.
        idle = ~self.healthy & ~self.rail & ~self.floor_bad
        starting = idle & np.isinf(self.clear_t)
        self.clear_t[starting] = t
        # That channel's estimator has been fed railed volts for the whole outage,
        # so it restarts the moment the rail clears and re-acquires INSIDE
        # REARM_SUSTAIN_S: 1.48 s measured against 2.0 s of hold. A lane coming
        # back must not resume on a stale state.
        self.kf.reset(starting)
        back = idle & (t - self.clear_t >= REARM_SUSTAIN_S)
        self.healthy[back], self.clear_t[back] = True, np.inf
        for i in np.nonzero(back)[0]:
            self.rms_run[i].reset()
            self.rms_sch[i].reset()
        self.excess_since[back], self.ratio[back] = np.inf, 0.0
        return drop, back

    # ===== the loop =====
    @property
    def wire_hz(self):
        """The MEASURED arrival rate, never a constant here: 12.5, 18, 23.5, 73 and
        ~1111 Hz by coil count and transport, which is why `decimate` exists.
        Fingerprinted into the baseline file."""
        span = (self.wire_t - self.wire_t0) if self.wire_t0 is not None else 0.0
        return (self.wire_n / span) if span > 0.5 else float("nan")

    def step(self, counts, volts, t, dt):
        """`decimate`. Called once per arriving sample, signature and contract
        unchanged, but only the FULL-RATE half runs every time; the control step
        fires on elapsed time and consumes the MEAN of what arrived since the last,
        averaged with the previous mean (`mains-null`).

        Full-rate: the accumulator; `counts_mean`, the trim's rest-position EMA,
        whose time constant is in seconds so more samples is more evidence; and
        `_rail_check`, ON RAW COUNTS, because a rail is a fact about a SAMPLE and
        the mean of eleven of which nine are pinned is not pinned.

        `dt`, the WIRE interval from the caller, is used only by `counts_mean`;
        everything downstream gets `ctl_dt`, measured between control steps."""
        counts, volts = np.asarray(counts, float), np.asarray(volts, float)
        self.stepped = False
        # `sample-guard`, second of two places: sim/server.py and harness.py build
        # their own arrays and never go through read_sample(). See in_range.
        if not in_range(counts):
            self.bad_samples += 1
            return self.state
        if self.wire_t0 is None:
            self.wire_t0 = t
        self.wire_t, self.wire_n = t, self.wire_n + 1
        self.acc_c += counts
        self.acc_v += volts
        self.acc_n += 1
        # Where each OSEM RESTS, which is what the trim steers. A 2 s constant is
        # two cycles of the ~1 Hz resonance; maintained in every state incl. FAULT.
        self.counts_mean += (counts - self.counts_mean) * min(1.0, dt / 2.0)
        self._rail_check(counts, t)

        # Accumulate by TIME, never by a fixed sample count, or a baud change
        # becomes a control-rate change. `ctl_t` is a DEADLINE advancing by whole
        # periods: setting it to `t` rounds up to the wire's grid, measured at
        # 92.6 Hz on a 1111 Hz wire, 86.7 Hz on the simulator's 347 Hz and 83 Hz on
        # 500 Hz. No backlog either: on a stall several deadlines pass at once, so
        # re-sync. That branch also runs when the wire is slower than CONTROL_HZ.
        if self.ctl_t is None:
            self.ctl_t, self.ctl_prev = t - CONTROL_PERIOD_S, t - CONTROL_PERIOD_S
        if t < self.ctl_t + CONTROL_PERIOD_S:
            return self.state
        self.ctl_t += CONTROL_PERIOD_S
        if t - self.ctl_t >= CONTROL_PERIOD_S:
            self.ctl_t = t                     # missed deadline(s): re-sync
        n = self.acc_n
        mean_counts, mean_volts = self.acc_c / n, self.acc_v / n
        self.acc_c, self.acc_v, self.acc_n = np.zeros(N), np.zeros(N), 0
        # `mains-null`, the whole of it: this control mean averaged with the last,
        # a 20 ms boxcar nulling 50 Hz exactly. The RAW means are what is carried,
        # so this is a 2-tap FIR and not an IIR; the first step averages with
        # itself and passes through. See MAINS_NULL for the measured R ratios.
        if MAINS_NULL:
            prev_c, prev_v = ((mean_counts, mean_volts) if self.prev_mean_v is None
                              else (self.prev_mean_c, self.prev_mean_v))
            self.prev_mean_c, self.prev_mean_v = mean_counts, mean_volts
            mean_counts = 0.5 * (mean_counts + prev_c)
            mean_volts = 0.5 * (mean_volts + prev_v)
        # The REAL interval between control steps, not the nominal period; the
        # upper clamp keeps a wire stall out of the estimator's transition matrix.
        self.ctl_dt = min(max(t - self.ctl_prev, 1e-4), 0.05)
        self.ctl_prev, self.stepped, self.n_avg = t, True, n
        self.ctl_steps += 1
        return self._control(mean_counts, mean_volts, t, self.ctl_dt)

    def _control(self, counts, volts, t, dt):
        """One control step, on the mains-nulled mean of the samples since the last.
        Same state machine, interlocks and order as delta; only the velocity path
        and the rail check's home differ. `counts` and `volts` are FRACTIONAL here,
        being means, and every consumer scales linearly or compares against a
        threshold."""
        # The whole velocity path. One update produces both columns and there is no
        # priming special case: the estimator primes its DC state on the first
        # sample and returns zero velocity for it.
        self.vel = self.kf.update(
            volts, dt, valid=(~self.rail if RAIL_BLANK else None))
        self.bp = self.kf.displacement()

        if self.state == "CALIBRATING":
            self.calib.append(self.bp)
            # ENABLED CHANNELS ONLY, and a DISABLED one must not be able to fault
            # the rig. Measured 2026-08-17: ch5's pin is at hard ground, so it
            # reads 0 counts, which is below RAIL_LOW; this test faulted the whole
            # rig on it for 55 s with every gain at zero -- and kept doing so
            # after ENABLE_CHANNEL[5] was set False, because the test never
            # consulted ENABLE_CHANNEL. The console said `ch5:off` in the same
            # breath as faulting on ch5.
            #
            # A DEAD PIN IS NOT A PLANT EXCURSION, and the two must be separated
            # or the whole graceful-degradation ladder is defeated by one bad
            # solder joint. The rail fault here is correct in its original
            # meaning: a rail during calibration normally means the optic is
            # against a stop, which nothing downstream can measure around. But a
            # channel that is railed from the FIRST sample with no variance at all
            # is not against a stop -- it is not connected. Variance separates
            # them: a real excursion clips and still moves, a dead pin does not.
            rail_now = self.rail & self.enabled & ~self.dead_pin(volts)
            if rail_now.any():
                self._fault(t, f"{self._who(rail_now)} railed during calibration -- "
                                f"check alignment.", railed=True)
            elif self.file_baseline is not None:
                self._warm_from_file(t)
            else:
                if t - self.sub_start >= CALIB_SUBWINDOW_S:
                    self._close_subwindow(t)
                ceiling = t - self.calib_start >= CALIBRATION_S
                early = (not ceiling) and self._calib_stationary()
                if early or ceiling:
                    if ceiling and len(self.calib) > self.sub_n0:
                        self._close_subwindow(t)     # flush the partial tail
                    self.calib_took, self.calib_early = t - self.calib_start, early
                    self._set_baseline(early)
                    self.baseline_t = t
                    self.reuse_count = 0
                    self.state, self.damping_start = "DAMPING", t
                    self._arm_envelope(t)
                    self.locked_announced = False
                    self._say(
                        (f"[DAMPING] baseline KEPT after a refused {self.calib_took:.1f}s "
                         "calibration -- these are the numbers already in hand, not "
                         "freshly measured"
                         if self.baseline_refused else
                         f"[DAMPING] baseline set in {self.calib_took:.1f}s "
                         + ("(sub-windows agreed -- stopped early, ceiling is "
                            f"{CALIBRATION_S:.0f}s)" if early
                            else f"(ran the full {CALIBRATION_S:.0f}s ceiling -- the "
                                 "floor never settled, median of sub-windows used)"))
                        + ": "
                        + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                        + f". Gain schedules {CAPTURE_GAIN[0]:+.3f} -> "
                          f"{STEADY_GAIN[0]:+.3f}...")
                    # AFTER the banner: every baseline first, then the verdict.
                    self._baseline_floor(t)
                    # main() does the file I/O, so `harness.py` cannot overwrite it.
                    self.baseline_saveable = True

        elif self.state == "DAMPING":
            self._damping(t, dt)

        elif self.state == "FAULT":
            self.gain[:] = 0.0
            # `sat` is refreshed in _actuate() in every state, so this is a live
            # reading and not a latch: in FAULT it clears on its own.
            if self.rail.any() or self.sat.any():
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = t
            elif t - self.clear_since >= FAULT_CLEAR_SUSTAIN_S:
                self._recover(t)

        self._actuate(t, dt)
        return self.state

    def _moved(self, t, i, new):
        """Apply a bias change to channel `i` and invalidate everything measured
        against the old operating point: both trend interlocks compare the envelope
        against its own past, and a bias step injects a ~1 Hz transient into the
        band they read."""
        self.bias[i] = new
        self.vmin[i], self.vmax[i] = new - BIAS_SWING, new + BIAS_SWING
        self.trim_step_t = t
        self.env_hist.clear()
        self.excess_since[:] = np.inf
        # A bias step moves this channel's whole output WINDOW, so pinned samples
        # recorded against the old one answer a question no longer asked.
        self.sat_hist.clear()
        self.sat_sum[:] = 0.0

    def _trim(self, t):
        """Nudge one channel's bias a quantum toward mid-scale, then judge it.
        DAMPING only and no faster than TRIM_PERIOD_S, a bias step being a DC force
        step the pendulum must ring down from. Judged on the TOTAL offset because
        four coils drive two DOF, so centring a0 can push a2 out; a step that does
        not improve the total is put back and that channel frozen for the run."""
        if self.trim_last is None:
            self.trim_last = t
            return
        if t - self.trim_last < TRIM_PERIOD_S:
            return
        self.trim_last = t
        total = float(np.abs(self.counts_mean - MID_COUNTS).sum())
        # Is the optic at rest? Every number below is a mean of raw counts, and a
        # mean taken while the rig is ringing describes the ring, not the resting
        # point. The test is LOCKED on every driven channel, not an instantaneous
        # ratio: `_recover` and `_health` reset the RMS banks, so for the first
        # seconds after a fault `ratio` climbs from zero and reads quiet no matter
        # how hard the optic is swinging. Measured 2026-08-07: the ratio form
        # stepped ch0 immediately after a recovery, with the driven channels at
        # 5.11 / 19.40 / 4.82 / 24.31. `locked` cannot be fooled that way -- it
        # needs the windows full AND under the line for LOCK_SUSTAIN_S.
        watch = self.enabled & self.healthy
        claim = watch & ((self.steady != 0.0) | (self.capture != 0.0))
        quiet = bool(claim.any() and self.locked[claim].all())

        if self.trim_pending is not None:          # judge last period's step
            i, was = self.trim_pending
            self.trim_pending = None
            if not quiet:
                # Cannot evaluate it, so do not pretend to. Reverted rather than
                # kept-on-faith: the step was speculative and putting it back
                # costs nothing. NOT frozen -- the channel gets another attempt
                # once the rig is quiet, which is the whole point of the gate.
                self._moved(t, i, was)
                self.trim_steps[i] -= 1
                self._say(f"[trim] ch{i} reverted to {was:.2f}V -- not locked at "
                          f"judging time (worst driven ratio "
                          f"{float(np.max(self.ratio[claim])) if claim.any() else float('nan'):.2f}), "
                          f"so the offsets it would be judged on describe the "
                          f"motion, not the rest point.")
                return
            if total >= self.trim_ref:
                self._moved(t, i, was)
                self.trim_frozen[i] = True
                self._say(f"[trim] ch{i} reverted to {was:.2f}V and frozen -- total "
                          f"offset {self.trim_ref:.0f} -> {total:.0f} counts.")
                return
            self._say(f"[trim] ch{i} kept at {self.bias[i]:.2f}V -- total offset "
                      f"{self.trim_ref:.0f} -> {total:.0f} counts.")

        err = self.counts_mean - MID_COUNTS
        # A channel already against the edge of the bias box cannot move, and
        # picking it wastes a full step-and-judge cycle (2 x TRIM_PERIOD_S = 30 s)
        # before the total-offset test reverts a step that never happened and
        # freezes it. ch2 hits this every run: SLOPE_SIGN[2] is +1, so reducing its
        # counts means reducing its bias, and BIAS_MIN is already its operating
        # point. That floor is not arbitrary -- at bias 0.25 with BIAS_SWING 0.25
        # the output window bottom is 0.0 V, and any lower would command a negative
        # voltage the DAC rejects. So ch2, which carries the largest offset (+173
        # counts, measured 2026-08-07) and therefore always wins argmax below, is
        # structurally immovable until BIAS_SWING itself changes. Skip it rather
        # than rediscover it every run.
        if not quiet:
            return              # never START a step on a moving optic either
        step_all = -np.sign(err) * SLOPE_SIGN * BIAS_QUANTUM
        movable = np.abs(np.clip(self.bias + step_all, BIAS_MIN, BIAS_MAX)
                         - self.bias) > 1e-9
        # Total DC excursion budget, checked per candidate: it is the SUM of the
        # bias offsets that loads the optic, and nothing else here bounds it.
        affordable = (np.abs(self.bias - BIAS).sum()
                      - np.abs(self.bias - BIAS)
                      + np.abs(np.clip(self.bias + step_all, BIAS_MIN, BIAS_MAX)
                               - BIAS)) <= TRIM_MAX_TOTAL_EXCURSION_V
        elig = (self.enabled & self.healthy & ~self.trim_frozen & movable
                & affordable
                & (self.trim_steps < TRIM_MAX_STEPS)
                & (np.abs(err) > TRIM_DEADBAND_COUNTS))
        if not elig.any():
            if (movable & ~affordable).any() and not self.trim_budget_said:
                self.trim_budget_said = True
                self._say(f"[trim] stopping: total bias excursion is "
                          f"{np.abs(self.bias - BIAS).sum():.2f}V of the "
                          f"{TRIM_MAX_TOTAL_EXCURSION_V:.2f}V budget and the next "
                          f"step would exceed it. Two coils far from nominal is "
                          f"what gave three runaways on 2026-08-07.")
            return
        i = int(np.argmax(np.where(elig, np.abs(err), -1.0)))
        step = -np.sign(err[i]) * SLOPE_SIGN[i] * BIAS_QUANTUM
        new = float(np.clip(self.bias[i] + step, BIAS_MIN, BIAS_MAX))
        if abs(new - float(self.bias[i])) < 1e-9:   # already against a clamp
            self.trim_frozen[i] = True
            return
        self.trim_pending, self.trim_ref = (i, float(self.bias[i])), total
        self.trim_steps[i] += 1
        was_counts = self.counts_mean[i]
        self._moved(t, i, new)
        self._say(f"[trim] ch{i} rests at {was_counts:.0f} counts "
                  f"({err[i]:+.0f} off mid-scale) -- bias -> {new:.2f}V")

    def _damping(self, t, dt):
        drop, back = self._health(t)
        live, n_conf = self.enabled & self.healthy, int(self.enabled.sum())
        n = int(live.sum())
        for i in np.nonzero(drop)[0]:
            self.demote_count += 1
            self._say(f"!! ch{i} railed -- held at bias, {n}/{n_conf} still damping.")
        for i in np.nonzero(back)[0]:
            self._say(f"[ch{i} back] rail clear {REARM_SUSTAIN_S:.0f}s -- re-engaging on its "
                      f"pre-event baseline {self.baseline[i]:.4f}V, {n}/{n_conf} damping.")
        if n_conf and n < MIN_HEALTHY_CHANNELS:
            return self._fault(t, f"quorum lost -- {n}/{n_conf} healthy, need "
                                  f"{MIN_HEALTHY_CHANNELS}. Freezing everything.",
                               railed=True)

        # `gain-ramp`, before the schedule reads steady/capture. Scales both, so
        # the blend, the slew limit and every interlock below stay exactly as
        # shipped and the only thing moving is overall loop gain.
        if self.ramp is not None:
            driven = live & ((self.base_steady != 0.0) | (self.base_capture != 0.0))
            demand = float(np.max(self.demand[driven])) if driven.any() else 0.0
            msg = self.ramp.sample(t, float(np.nanmean(self.ratio[driven]))
                                   if driven.any() else float("nan"),
                                   demand, float(self.rail[driven].mean())
                                   if driven.any() else 0.0,
                                   self.state != "DAMPING")
            if msg:
                self._say(msg)
            s = self.ramp.scale()
            self.steady, self.capture = self.base_steady * s, self.base_capture * s

        # Gain schedule: capture -> steady blended on amplitude, slew-limited.
        self.ratio = np.where(live, _rms(self.rms_sch, t, self.bp, live) / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)

        # The modal gain, one scalar per MODE. There is deliberately NO
        # capture/steady blend here, and no amplitude term: `capture` exists to
        # grab a large amplitude, and the modal law's claim is better ALLOCATION
        # of a given force, not more reach. Adding a second unvalidated gain
        # vector to buy nothing the law claims is how a first bench run stops
        # being able to answer what it was run to answer.
        #
        # So the target is constant and the SLEW LIMIT is the whole schedule:
        # GAIN_SLEW_PER_S is the same limiter the diagonal law uses, so the soft
        # start is the one already on the bench, and `mode_gain` reaching its
        # target takes the same time a channel gain does.
        self.mode_gain = (self.mode_gain + np.clip(
            MODAL_KP * MODAL_GAIN_SCALE - self.mode_gain,
            -GAIN_SLEW_PER_S * dt, GAIN_SLEW_PER_S * dt)
            if self.modal.ok else np.zeros(self.nmode))

        # Gated on `healthy` not `live`: a blind channel measures nothing.
        env = _rms(self.rms_run, t, self.bp, self.healthy)
        # `runaway-trend`. A level test cannot tell a runaway from a ringdown, so it
        # trips on a timer after a kick: hardware 2026-08-04 (v8, 240 s, 3 kicks)
        # gave ten faults with the envelope falling through every one, 5.97 -> 4.61,
        # 3.00 -> 3.14, 2.72 -> 2.07, the only survivor at 1.64 under the 1.8 line.
        # Nothing is recorded until the RMS window behind `env` has refilled --
        # see _arm_envelope. A partially-filled window reads low, and a low
        # `past` turns any ringdown into a runaway.
        if t >= self.env_valid_t:
            self.env_hist.append((t, env.copy()))
        while self.env_hist and t - self.env_hist[0][0] > RUNAWAY_TREND_LAG_S * 2:
            self.env_hist.popleft()
        # `runaway-peak`. Compare against the RECENT PEAK, not the value at a fixed
        # lag. The lag form is what beta shipped and it faults on every kick, by
        # construction rather than by bad luck: for the whole RUNAWAY_TREND_LAG_S
        # after a disturbance the lagged sample is from BEFORE it, so it is small,
        # so `growing` is true no matter what the envelope is doing now. Any kick
        # that clears RUNAWAY_MULTIPLE therefore trips RUNAWAY_SUSTAIN_S later and
        # nothing can prevent it. Measured 2026-08-07 on epsilon: four faults in
        # 50 s with the envelope falling through all of them, ch3 at 19.00 then
        # 18.89 while ringing down at the measured tau of 9 s. It is also the most
        # likely reading of delta's 7 faults in 244.5 s.
        #
        # A runaway is the envelope setting NEW highs; a ringdown is the envelope
        # below its own recent peak. So the comparison is against the largest
        # value seen between RUNAWAY_SUSTAIN_S and 2 x RUNAWAY_TREND_LAG_S ago.
        # Replayed against a kick ringing down at tau = 9 s the lag form gives 5
        # spurious trips and this gives 0, and against a genuine runaway doubling
        # every 8 s BOTH trip at 21.8 s, so no detection is traded away. After a
        # kick this asks only whether the optic is climbing past the kick's own
        # peak, which is the question actually worth asking.
        # The exclusion is ENVELOPE_WINDOW_S and must NOT be RUNAWAY_SUSTAIN_S.
        # It exists only so the reference is not the current, still-filling window
        # comparing against itself. Tying it to the sustain instead re-creates the
        # very defect this replaced: at a sustain of 8 s the kick's own peak, which
        # lands 1-2 s after the impulse, falls INSIDE the excluded region, so
        # `past` reverts to a pre-kick value and every kick trips again. Measured
        # 2026-08-07, replaying this run's own CSV: at both faults the envelope was
        # far BELOW its recent peak (ch1 at 26% and 56% of it) and the breaker
        # fired anyway. With a 2 s exclusion the peak is in the reference and
        # neither trip happens.
        older = [e for ts, e in self.env_hist if t - ts >= ENVELOPE_WINDOW_S]
        past = np.maximum.reduce(older) if older else env
        growing = env > past * RUNAWAY_GROWTH_FRAC
        high = env > self.baseline * RUNAWAY_MULTIPLE
        # `runaway-quorum`, the same argument as `lock-quorum` and the same mask.
        # A runaway is the LOOP adding energy, so only a channel the loop actually
        # drives can have one. a5 is healthy, real and undriven (STEADY_GAIN[5] = 0),
        # so it is undamped by construction: kick the optic and it rings up and
        # stays up, which `high & growing` reads as a runaway every time. Measured
        # 2026-08-07: a5 alone faulted the rig three times in 60 s while a0-a3 were
        # recovering normally at ratios 0.22 / 0.86 / 0.44 / 0.94 against its 10.39.
        # Freezing every channel because an UNDAMPED one is ringing is backwards:
        # the answer to a disturbance is more damping, and the fault removes all of
        # it. a5 stays sensed, logged and rail-checked; it just stops voting here
        # too. If the optic is genuinely running away the driven channels show it,
        # so this gate costs no real detection.
        drives = (self.steady != 0.0) | (self.capture != 0.0)
        self.excess_since = self._hold(self.healthy & drives & high & growing,
                                       self.excess_since, t)
        run_trip = self.healthy & drives & (t - self.excess_since >= RUNAWAY_SUSTAIN_S)
        # `soft-saturation`, second half: a clipped output whose envelope falls is
        # winning, so only a flat or rising one is a fault. Same lag as the trend.
        sat_trip = self.healthy & self.sat & ~(env < past * SAT_DECAY_FRAC)
        if run_trip.any():
            return self._fault(t, f"{self._who(run_trip)} runaway -- freezing all "
                                  f"channels, entering FAULT.")
        if sat_trip.any():
            return self._fault(t, f"{self._who(sat_trip)} pinned against the rail, "
                                  f"still demanding more, and not winning -- freezing "
                                  f"all channels, entering FAULT.")

        # A demoted channel flatlines its estimate, which an RMS-only test reads as
        # the steadiest axis on the rack, so it may neither lock nor block a lock.
        quiet = live & (_rms(self.rms_lock, t, self.bp, live) < self.baseline * LOCK_RMS_FACTOR)
        self.locked_since = self._hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= LOCK_SUSTAIN_S)
        self._trim(t)
        # `lock-quorum`: a channel NOTHING DRIVES may not veto the lock. delta and
        # its two predecessors never once announced LOCKED, the deliverable, and
        # this is why: ENABLE_CHANNEL is True on all eight, a4/a6/a7 demote to NOSIG
        # and leave `live`, but a5 stays healthy (baseline 0.579 of the median, and
        # `baseline-floor` is right to keep it) with STEADY_GAIN[5] = 0, so nothing
        # damps it, it sits at ratio 1.26-2.53 against a 0.35 line, and one undriven
        # channel vetoes the rig permanently. a0-a3 reached lock together 6 times in
        # data/20260806_200822_fast_lock.csv while that was happening. a5 stays
        # sensed, logged and rail-checked; it just stops voting.
        claim = live & ((self.steady != 0.0) | (self.capture != 0.0))
        if claim.any() and self.locked[claim].all() and not self.locked_announced:
            self.lock_time = t - self.damping_start
            self._say(f"*** LOCKED -- {self.lock_time:.1f}s after gain was applied "
                      f"(t={t:.1f}s total), {int(claim.sum())}/{n_conf} channels "
                      f"driven and quiet"
                      + ("" if n == n_conf else f" -- DEGRADED, {n}/{n_conf} healthy")
                      + " ***")
            self.locked_announced = True

    def _actuate(self, t, dt):
        # Takes `t`: the saturation interlock is a time window, not a count.
        live = self.enabled & self.healthy & (self.state == "DAMPING")

        # ===== the modal law =====
        # Two sets, not one (mimo_closed.md Sec 4.1). SENSE_OK is "this sensor's
        # opinion counts"; DRIVE_OK is "this coil is driven". They differ, and a5
        # is the case that proves it: a live sensor with a dead coil is exactly
        # what a single `live` flag cannot represent.
        sense_ok = live & ~self.floor_bad
        drive_ok = live & ~self.floor_bad
        self.modal_on = False
        self.modal_u[:] = 0.0
        if self.modal.ok and self.state == "DAMPING":
            # The estimator already separates the modes -- per channel, per mode
            # -- so this is nm scalar least-squares fits, not a pseudo-inverse.
            # See Modal.sense: cond(Phi) stops being load-bearing entirely.
            vmodal = self.kf.modal_velocity()
            self.qdot, self.modal_res, self.mode_seen = self.modal.sense(vmodal, sense_ok)
            enough = self.mode_seen >= MODAL_MIN_SENSORS
            # PROJECT FIRST, THEN GAIN. The allocator may only reach `rank` of
            # the nm modal directions after truncation; zeroing the velocity in
            # the rest is what makes the realised force -Proj K Proj qdot, which
            # is guaranteed dissipative. See Modal.project.
            qd, self.modal_rank = self.modal.project(self.qdot, drive_ok)
            # A mode nobody can see gets zero force rather than a guess. The
            # OTHER modes are still damped: that is the degradation MIMO is for.
            f = np.where(enough, -self.mode_gain * qd, 0.0)
            u, self.modal_mask, ok = self.modal.allocate(f, drive_ok)
            if ok and enough.any():
                self.modal_on = True
                self.modal_u = u
            elif self.modal_says is None:
                self.modal_says = (
                    "[modal] falling back to DIAGONAL: %s. Not a fault -- "
                    "epsilon's law, epsilon's gains, restricted to the survivors."
                    % ("no modal direction is reachable (mask %d, cond %.2f,"
                       " rank %d of %d)"
                       % (self.modal_mask, self.modal.cond[self.modal_mask],
                          self.modal.rank[self.modal_mask], self.modal.nm)
                       if not ok else
                       "no mode has %d determined sensors" % MODAL_MIN_SENSORS))
                self._say(self.modal_says)

        err = -self.vel                                # setpoint is zero velocity
        self.p = self.gain * err
        self.prev_vel[~self.primed] = self.vel[~self.primed]
        self.primed[:] = True
        # Derivative on the MEASUREMENT, so it cannot kick if the setpoint moves.
        dv = (self.vel - self.prev_vel) / dt if dt > 0 else np.zeros(N)
        self.prev_vel = self.vel.copy()
        self.d = -self.kd * self.dfilt.update(dv, dt)
        # Back-calculation anti-windup: the integrator unwinds at TRACK_TC_S from
        # last sample's clip rather than freezing at a limit. Stepped ONLY where ki
        # is non-zero: with Ki = 0 the back-calculation is a one-way ratchet, since
        # nothing drives `i` back, and one clipped sample would leave a standing
        # offset of up to I_CLAMP_V against a 0.250 V half-window.
        self.i = np.where(
            self.ki != 0.0,
            np.clip(self.i + (self.ki * err - self.clip_excess / TRACK_TC_S) * dt,
                    -I_CLAMP_V, I_CLAMP_V),
            0.0)

        # The modal command REPLACES P, not adds to it: both are the proportional
        # velocity-feedback term and running them together would double the loop
        # gain. D survives on both paths (it is per-channel effective mass and is
        # unchanged by the allocation) and Ki is zero on both, structurally --
        # v_hat never reads the estimator's drift state, so there is nothing left
        # for an integrator to reject.
        if self.modal_on:
            self.p = np.where(live, self.modal_u, 0.0)
        raw = self.bias + self.p + self.i + self.d
        clipped = np.clip(raw, self.vmin, self.vmax)
        self.clip_excess = np.where(live, raw - clipped, 0.0)
        # What the law ASKED for, before the window. `out` is clipped and then
        # slew-limited, so |out - bias| saturates at BIAS_SWING and cannot report
        # demand at all: measured p99 demand is 0.85 V against a 0.25 V half-window
        # (analysis/out/kalman_headroom.csv), so this loop lives in clipping and a
        # test on the clipped value reads "at the limit" every step of every run.
        self.demand = np.abs(raw - self.bias)
        # Bumpless re-entry for every lane not actuating: FAULT, config-disabled and
        # demoted alike, so none resumes with a stale integral or a derivative step.
        # The ESTIMATOR is deliberately not reset here: it is a measurement, every
        # interlock downstream reads `bp`, and CALIBRATING has no live lane at all.
        dead = ~live
        if dead.any():
            self.p[dead] = self.i[dead] = self.d[dead] = self.prev_vel[dead] = 0.0
            self.primed[dead] = False
            self.dfilt.reset(dead)
        cap = MAX_SLEW_PER_S * dt
        target = np.where(live, clipped, self.bias)
        self.out = self.prev_out + np.clip(target - self.prev_out, -cap, cap)
        self.prev_out = self.out.copy()
        # Maintained HERE, so it refreshes in every state including FAULT, which is
        # what stops the v0/v1/v2 saturation latch.
        pinned = (self.out <= self.vmin + 1e-6) | (self.out >= self.vmax - 1e-6)
        # `sat-window`: a fraction of a window in SECONDS, not a run of
        # consecutive iterations. See _sat_check.
        self._sat_check(pinned, t)
        # `soft-saturation`. A clipped loop is weakened, not broken, and the
        # envelope at the trip below answers whether it is achieving anything. NOT
        # clip_excess: anti-windup drives the integrator down until `raw` stops
        # exceeding the rail, so it decays to zero exactly when saturation is worst
        # (the suite caught that as "0 fault(s)").
        self.act.send(self.out)

    def _why_reuse(self, t):
        """Whether the fault path may re-engage on the baseline in hand, and the
        one-line reason. Four cases must RE-MEASURE, each enforced below: nothing
        measured yet; a RAIL fault, so the floor came through a suspect sensor; a
        baseline older than BASELINE_MAX_AGE_S; MAX_BASELINE_REUSE reuses with no
        FAST_REFAULT_S engagement to revalidate it."""
        if not bool(self.baseline.all()):
            return False, "nothing measured yet"
        if self.fault_railed:
            return False, ("the fault was a RAIL -- a floor measured through a "
                           "suspect sensor is suspect with it")
        # A trim step does NOT invalidate the baseline: it is an RMS of
        # displacement(), which excludes DC by construction (the `b` state absorbs
        # it), and a bias step moves DC. A quantum is ~25-50 counts of 1023 against
        # thresholds like RUNAWAY_MULTIPLE = 1.8.
        age = None if self.baseline_t is None else t - self.baseline_t
        if age is not None and age > BASELINE_MAX_AGE_S:
            return False, (f"the baseline is {age:.0f}s old, over the "
                           f"{BASELINE_MAX_AGE_S:.0f}s limit")
        if self.reuse_count >= MAX_BASELINE_REUSE:
            return False, (f"already reused {self.reuse_count} times without a "
                           f"{FAST_REFAULT_S:.0f}s engagement in between")
        if self.damped_for is not None and self.damped_for < FAST_REFAULT_S:
            return True, (f"re-faulted after only {self.damped_for:.1f}s, so the "
                          f"disturbance is still there and a fresh window would "
                          f"measure it rather than the floor")
        return True, ("the previous engagement was healthy, so this baseline is "
                      "known good and the fault was a transient -- measuring a "
                      "new floor now would measure the ringdown")

    def _recover(self, t):
        # Leaving FAULT: back to the baseline in hand, or a full recalibration.
        if self.damped_for is not None and self.damped_for >= FAST_REFAULT_S:
            self.reuse_count = 0        # the loop itself validated this baseline
        reuse, why = self._why_reuse(t)
        keep, keep_t = self.baseline.copy(), self.baseline_t
        keep_floor = self.floor_bad.copy()
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        for k in "baseline gain ratio sat_sum clip_excess p i d prev_vel".split():
            getattr(self, k)[:] = 0.0
        # `sat_sum` is only half of it: its window must go too, or the next
        # SAT_SUSTAIN_S of drain carries pre-fault pinned samples into the loop.
        self.sat_hist.clear()
        self.primed[:] = self.sat[:] = self.locked[:] = False
        self.locked_since[:] = self.excess_since[:] = self.clear_t[:] = np.inf
        self.healthy[:] = True          # a recovery forgets every demotion...
        self.floor_bad[:] = False
        self.dfilt.reset()
        # Whatever put the rig in FAULT is in the estimator's state too; it
        # re-acquires in 1.48 s, inside the gain ramp from zero.
        self.kf.reset()
        for b in self.rms_sch + self.rms_run + self.rms_lock:
            b.reset()
        if reuse:
            self.reuse_count += 1
            self.baseline, self.baseline_t = keep, keep_t
            # ...except a floor demotion, which travels WITH its baseline: neither
            # was re-measured, and re-arming would restore the zero denominator.
            self.floor_bad = keep_floor
            self.healthy[keep_floor] = False
            self.state, self.damping_start = "DAMPING", t
            self._arm_envelope(t)
            self.locked_announced = False
            self._say(f"[recovered] re-engaging on the baseline already in hand -- "
                      f"{why} ({self.reuse_count}/{MAX_BASELINE_REUSE} before a "
                      f"forced re-calibration). Gain ramps from zero."
                      + (f" {self._who(keep_floor)} stays demoted -- same "
                         f"baseline, same verdict." if keep_floor.any() else ""))
        else:
            # A full recalibration is the only thing that can put one back in.
            self.reuse_count = 0
            self.baseline_t, self.damping_start = None, None
            self._start_calibration(t)
            self.state = "CALIBRATING"
            self._say(f"[recovered] all channels clear for {FAULT_CLEAR_SUSTAIN_S:.0f}s "
                      f"-- re-calibrating ({why}) for up to {CALIBRATION_S:.0f}s.")
        self.fault_railed = False
        self.clear_since = self.lock_time = None

    # ===== reporting =====
    def status_line(self, t):
        def one(i):
            if self.floor_bad[i]:
                # Distinct from DOWN: a rail re-arms itself, NOSIG wants a person.
                return f"ch{i}:NOSIG bp={self.bp[i]:+.3f}V base={self.baseline[i]:.4f}V"
            if not self.enabled[i]:
                return f"ch{i}:off  bp={self.bp[i]:+.3f}V"
            if not self.healthy[i]:
                return f"ch{i}:DOWN bp={self.bp[i]:+.3f}V"
            flags = ("LOCK" if self.locked[i] else "....") + ("!RAIL" if self.rail[i] else "")
            return (f"ch{i}:{flags} g={self.gain[i]:+.4f} "
                    f"bp={self.bp[i]:+.3f}V ratio={self.ratio[i]:.2f}")
        # A quietly changed wire rate has invalidated a threshold here twice.
        # `avg` of 1 means the wire is slower than CONTROL_HZ.
        rate = (f"{self.wire_hz:.0f}/{CONTROL_HZ:.0f}Hz avg{self.n_avg:d}"
                if self.wire_hz == self.wire_hz else f"--/{CONTROL_HZ:.0f}Hz")
        bad = f" bad{self.bad_samples}" if self.bad_samples else ""
        if self.modal.ok:
            mode = ("MIMO" if self.modal_on else "diag") + (
                " q=" + "/".join(f"{x:+.3f}" for x in self.qdot)
                + " res=" + "/".join(f"{x:.2f}" for x in self.modal_res))
        else:
            mode = "diag(no modal data)"
        return (f"[{t:7.1f}s] {self.state:11s} {rate}{bad} {mode} "
                + "  ".join(one(i) for i in range(N)))

    def csv_row(self, t, counts):
        """`healthy` is the 13th column and `rail` cannot replace it: a demotion
        outlasts the rail that caused it by REARM_SUSTAIN_S. Written for EVERY
        accepted sample, not every control step, the raw stream going to disk
        (CLAUDE.md); `ctl` marks the row that completed a control step and `n_avg`
        how many it averaged, so the decimation replays offline from the counts.
        `bp` is displacement(), not a bandpass output, so a floor from an older
        version is not comparable to one measured here."""
        row = [f"{t:.4f}", self.state,
               "1" if self.stepped else "0",
               str(self.n_avg if self.stepped else 0),
               "1" if self.modal_on else "0", str(self.modal_mask),
               str(self.modal_rank)]
        row += [f"{x:.6f}" for x in self.qdot]
        row += [f"{x:.5f}" for x in self.mode_gain]
        row += [f"{x:.5f}" for x in self.modal_res]
        for i in range(N):
            row += [str(int(counts[i])), f"{counts[i] * (A_VCC / ADC_MAX_COUNTS):.4f}"]
            row += [format(getattr(self, k)[i], f) for k, f in _LOG]
            row += [str(int(v[i])) for v in (self.rail, self.locked, self.healthy)]
        return ",".join(row)


# ===== persist-baseline: the file =====
# Module level, out of Controller: `harness.py` and `sim/server.py` step it
# thousands of times per suite and would overwrite the bench's measured floor.
def _fingerprint(bias, enable=None, steady=None, capture=None, ki=None, kd=None):
    """Everything that changes what a baseline MEANS. A mismatch REFUSES the file
    rather than adapting: every downstream test divides by the floor, and a wrong
    denominator both desensitises the runaway breaker and makes LOCKED easier to
    declare. `bias` is passed in because `bias-trim` moves it during a run, so what
    is stored is the bias the floor was MEASURED at.

    delta's `bp_low_hz` / `bp_high_hz` / `deriv_smooth_hz` are GONE, replaced by the
    mode list and T_AMP, and that is deliberate: the baseline is an RMS of `bp`, and
    `bp` is now displacement() rather than a bandpass output. Every delta-era file
    is refused, on the missing `modes_hz` and on the bandpass keys it carries that
    this build does not know (see load_baseline). A floor measured against the old
    definition is a wrong denominator, not a stale one."""
    g = lambda v, d: [round(float(x), 9) for x in (d if v is None else v)]
    return dict(
        n_channels=int(N),
        enable=[bool(v) for v in (ENABLE_CHANNEL if enable is None else enable)],
        steady=g(steady, STEADY_GAIN), capture=g(capture, CAPTURE_GAIN),
        ki=g(ki, KI_GAIN), kd=g(kd, KD_GAIN), bias=g(bias, BIAS),
        dac_channels=[int(c) for c in DAC_CHANNELS],
        # Move a mode, T_AMP or the mains null and the number is an RMS of
        # something else.
        velocity="kalman-3mode", modes_hz=g(None, F_MODE_HZ),
        t_amp_s=T_AMP_S, t_dc_s=T_DC_S, mains_null=bool(MAINS_NULL),
        d_smooth_hz=D_SMOOTH_HZ,
        # The rate the loop averages at, and the counts->volts scale.
        control_hz=CONTROL_HZ, a_vcc=A_VCC, adc_max_counts=int(ADC_MAX_COUNTS))


def _same(a, b, tol=1e-9):
    if isinstance(a, list) != isinstance(b, list):
        return False
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def save_baseline(ctl, path=BASELINE_PATH):
    """Write the floor just measured with the fingerprint it was measured under,
    through a temp file and os.replace. A floor half-written by a Ctrl+C must never
    be loadable: a truncated JSON array would either fail to parse or, worse, parse
    short and hand the loop a baseline of the wrong length."""
    payload = dict(
        schema=BASELINE_FILE_SCHEMA,
        written_by=VERSION_TAG,
        measured_unix=time.time(),
        measured_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        baseline_v=[float(v) for v in ctl.baseline],
        # Travels WITH the baseline: exactly as fresh as the numbers it came from.
        floor_bad=[bool(v) for v in ctl.floor_bad],
        calib_took_s=(None if ctl.calib_took is None else float(ctl.calib_took)),
        calib_early=bool(ctl.calib_early),
        # MEASURED, not configured, so it is compared with a tolerance on load.
        wire_hz=(float(ctl.wire_hz) if ctl.wire_hz == ctl.wire_hz else None),
        config=_fingerprint(ctl.bias, ctl.enabled, ctl.steady, ctl.capture,
                            ctl.ki, ctl.kd))
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
    os.replace(tmp, path)
    return payload


def load_baseline(path=BASELINE_PATH, fresh=False, now=None):
    """-> ((baseline, floor_bad) or None, [lines to print]).

    Every rejection returns a REASON and main() prints all of them before the first
    sample: the file about to run must be the thing you can read, so
    `persist-baseline` is only acceptable if it is loud."""
    say = [f"stored baseline: {path}"]
    if fresh:
        say.append("  IGNORED -- fresh calibration forced (--fresh / "
                   "OSEM_FRESH_CALIB). Measuring the floor the long way.")
        return None, say
    try:
        with open(path) as fh:
            d = json.load(fh)
    except FileNotFoundError:
        say.append(f"  none yet. Calibrating for up to {CALIBRATION_S:.0f}s and "
                   f"writing one at the end.")
        return None, say
    except (OSError, ValueError) as e:
        say.append(f"  UNREADABLE ({e}) -- ignored, calibrating from scratch.")
        return None, say

    def no(why):
        say.append(f"  REFUSED -- {why}. Calibrating from scratch "
                   f"(up to {CALIBRATION_S:.0f}s).")
        return None, say

    if d.get("schema") != BASELINE_FILE_SCHEMA:
        return no(f"schema {d.get('schema')!r}, this build reads "
                  f"{BASELINE_FILE_SCHEMA}")
    age = (time.time() if now is None else now) - float(d.get("measured_unix", 0.0))
    say.append(f"  written by {d.get('written_by', '?')} at "
               f"{d.get('measured_utc', '?')}, {age:.0f}s ago")
    if not (0 <= age <= BASELINE_FILE_MAX_AGE_S):
        return no(f"{age:.0f}s old against a {BASELINE_FILE_MAX_AGE_S:.0f}s limit"
                  if age >= 0 else
                  f"measured {-age:.0f}s in the FUTURE -- the clock moved")
    have, want = d.get("config", {}), _fingerprint(BIAS)
    # BOTH directions. A key this build does not know means the file was written by
    # a build whose baseline meant something else: every delta-era file carries
    # `bp_low_hz` and is refused here, before any value is compared.
    extra = sorted(set(have) - set(want))
    if extra:
        return no(f"the file records {', '.join('`%s`' % k for k in extra)}, which "
                  f"this build does not know -- it was written against a different "
                  f"velocity path, so its floor is an RMS of a different signal")
    for k in sorted(want):
        if k not in have:
            return no(f"the file does not record `{k}`")
        if not _same(have[k], want[k]):
            return no(f"`{k}` differs: file {have[k]!r}, this build {want[k]!r}")
    # Measured, not configured, so a ratio rather than equality: a bigger change
    # means a different transport or a different baud.
    fw = d.get("wire_hz")
    b = d.get("baseline_v") or []
    fl = d.get("floor_bad") or [False] * len(b)
    if len(b) != N or len(fl) != N:
        return no(f"{len(b)} baselines and {len(fl)} floor flags, expected {N}")
    if not all(isinstance(x, (int, float)) and np.isfinite(x) and x > 0 for x in b):
        return no("a baseline is zero, negative or not finite")
    say.append("  " + ", ".join(f"ch{i}={v:.4f}V" + ("(NOSIG)" if fl[i] else "")
                                for i, v in enumerate(b)))
    say.append(f"  measured at {('%.0f Hz' % fw) if fw else 'an unrecorded rate'} "
               f"on the wire, {CONTROL_HZ:.0f} Hz control step, calibration took "
               f"{d.get('calib_took_s') or float('nan'):.1f}s")
    say.append(f"  ACCEPTED provisionally. It is adopted only if the first "
               f"{BASELINE_WARMUP_S:.1f}s of live signal agree with it to within "
               f"{BASELINE_SANITY_RATIO:.1f}x, per channel; otherwise this run "
               f"calibrates normally and says so.")
    return (np.array(b, float), np.array(fl, bool)), say


def _selftest():
    """The modal math, against a plant whose Phi and A are known exactly.

    WHY THIS EXISTS AND THE SIMULATOR DOES NOT REPLACE IT. `sim/server.py`
    carries a measured eight-OSEM body, but it models **two** modes
    (`MODE_F0_8`, `SHAPE_8` is 8x2) and the ringdown has since measured
    **three**. So `make check` exercises this file's supervisor, its transport
    and its DIAGONAL law -- everything zeta shares with epsilon -- and cannot
    reach the modal path at all. That is a real gap and it is stated here rather
    than left for somebody to assume the suite covered it.

    What the bench still has to answer, and this cannot: whether the modal law
    damps the actual optic, and whether Kp = -0.040's rail onset moves when
    every sensor reaches every coil. The simulator's own README already records
    that it cannot tell you a safe gain.
    """
    import tempfile
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("   %-46s %s%s" % (name, "PASS" if cond else "FAIL",
                                 ("   " + detail) if detail else ""))

    rng = np.random.default_rng(20260815)
    nm = len(F_MODE_HZ)
    # A consistent pair, built the way physics builds one: sensor and coil are
    # the same head, so A[m,j] = lambda_j * Phi[j,m] with ONE lambda per channel
    # (mimo_closed.md Sec 3.2). Anything else is not a plant zeta should accept.
    Phi = np.sign(rng.normal(size=(N, nm))) * (0.5 + rng.random((N, nm)))
    Phi[6] = Phi[7] = 0.0                       # two blind sensors, as measured
    lam = np.array([1.0, -0.8, 1.4, -1.1, 0.2, 0.3, 0.0, 0.0])
    A = np.array([[lam[j] * Phi[j, m] for j in range(N)] for m in range(nm)])
    okrow = np.zeros((N, nm), bool)
    okrow[:6] = True
    d = dict(schema=MODAL_SCHEMA,
             created=datetime.now().isoformat(timespec="seconds"),
             git_rev="selftest", source="selftest", n_sensors=N, n_modes=nm,
             modes=[dict(name=c, f_hz=float(f))
                    for c, f in zip("ABC", F_MODE_HZ)],
             phi=dict(value=Phi.tolist(),
                      sigma=np.full((N, nm), 0.01).tolist(),
                      ok=okrow.tolist(), imag_frac=[0.0] * nm, gauge="selftest"),
             a=dict(value=A.tolist(), coils=[0, 1, 2, 3], gauge="selftest",
                    provisional=False, provenance="selftest"))
    tmp = os.path.join(tempfile.gettempdir(), "zeta_selftest_modal.json")
    with open(tmp, "w") as fh:
        json.dump(d, fh)

    print("\n  === osem.zeta.py modal selftest ===\n")
    m = Modal(tmp)
    check("a consistent Phi/A pair loads", m.ok, m.why)
    if not m.ok:
        return 1
    check("colocation sign check keeps all four coils", m.a_coils == [0, 1, 2, 3],
          str(m.a_coils))

    full = int(sum(1 << j for j in m.a_coils))
    check("the full coil mask passes the gate", m.gate[full],
          "cond %.2f" % m.cond[full])

    # --- sensing: plant a modal velocity, see whether it comes back ---------
    sense_ok = np.array([True] * 6 + [False, False])
    qtrue = np.array([0.30, -0.12, 0.07])[:nm]
    # From m.phi, NOT the Phi written to the file: _load normalises the columns
    # to unit norm so that MODAL_KP means the same thing as the diagonal Kp, and
    # q_dot is defined against the normalised shape. Synthesizing from the
    # unnormalised one measures the scale factor, not the estimator.
    v = m.phi * qtrue                            # v[i,m] = Phi[i,m] * qdot_m
    q, res, seen = m.sense(v, sense_ok)
    check("sensing recovers a planted modal velocity",
          np.allclose(q, qtrue, rtol=1e-9, atol=1e-12),
          "%s vs %s" % (np.round(q, 6), np.round(qtrue, 6)))
    check("out-of-mode residual is zero on a consistent reading",
          float(np.max(np.abs(res))) < 1e-9, "max %.2e" % float(np.max(np.abs(res))))

    # One sensor disagreeing is the failure NO other interlock can see: not
    # railed, not signal-less, just wrong. It must show up in the residual.
    vbad = v.copy()
    vbad[1] *= -3.0
    _, rbad, _ = m.sense(vbad, sense_ok)
    check("a disagreeing sensor raises the residual",
          float(np.max(rbad)) > 0.2, "max %.3f" % float(np.max(rbad)))

    # --- allocation: is the realised modal force the one asked for? ---------
    f = -np.array([0.02, 0.01, 0.015])[:nm]
    u, mask, gok = m.allocate(f, sense_ok & np.array([True] * 4 + [False] * 4))
    got = m.a[:, [0, 1, 2, 3]] @ u[[0, 1, 2, 3]]
    check("allocation is exact at full row rank (A A+ = I)",
          gok and np.allclose(got, f, rtol=1e-8, atol=1e-12),
          "%s vs %s" % (np.round(got, 6), np.round(f, 6)))

    # THE safety property. f = -K qdot with K > 0, and at full row rank the
    # realised force equals it, so the modal power is strictly negative: the law
    # removes energy. A sign error anywhere in Phi, A or the allocation flips
    # this, and a wrong sign PUMPS rather than under-damps.
    K = MODAL_KP[:nm] * MODAL_GAIN_SCALE
    fq = -K * qtrue
    uq, _, _ = m.allocate(fq, sense_ok & np.array([True] * 4 + [False] * 4))
    power = float((m.a[:, [0, 1, 2, 3]] @ uq[[0, 1, 2, 3]]) @ qtrue)
    check("the realised modal force DISSIPATES (f . qdot < 0)", power < 0,
          "f.qdot = %+.6f" % power)

    # --- degradation --------------------------------------------------------
    for j in m.a_coils:
        mm = full & ~(1 << j)
        dok = np.zeros(N, bool)
        for k in m.a_coils:
            if k != j:
                dok[k] = True
        # Project first, exactly as the loop does, so what is checked is the law
        # that actually runs and not a more favourable variant of it.
        qd, rk = m.project(qtrue, dok)
        fj = -K * qd
        uu, mk, gg = m.allocate(fj, dok)
        cols = [k for k in m.a_coils if k != j]
        if gg:
            realised = m.a[:, cols] @ uu[cols]
            # WHAT "CORRECT" MEANS AFTER TRUNCATION. At full rank the realised
            # force equals the demand. At reduced rank it equals the demand
            # PROJECTED onto the reachable directions -- that is the whole point,
            # and demanding exactness would be demanding the old refuse-instead
            # behaviour back. What must hold in BOTH cases is dissipation.
            exact = np.allclose(realised, fj, rtol=1e-6, atol=1e-9)
            power = float(realised @ qtrue)
            ok_here = mk == mm and power < 0 and (exact or rk < nm)
            check("drop coil %d -> rank %d of %d, dissipates%s"
                  % (j, rk, nm, ", and exact" if exact else " (truncated)"),
                  ok_here,
                  "cond %.2f, f.qdot %+.6f%s" % (m.cond[mm], power,
                  "" if exact else ", realised = demand projected"))
        else:
            # Still the designed floor: nothing reachable at all, so the loop
            # runs epsilon's diagonal law on the survivors. Checked here is that
            # it refuses CLEANLY rather than returning a wrong command.
            check("drop coil %d -> nothing reachable, refuses cleanly" % j,
                  mk == mm and not uu.any() and rk == 0,
                  "cond %.2f, rank 0" % m.cond[mm])
    for i in range(4):
        s2 = sense_ok.copy()
        s2[i] = False
        q2, _, seen2 = m.sense(v, s2)
        check("drop sensor a%d -> modal velocity still exact" % i,
              np.allclose(q2, qtrue, rtol=1e-8, atol=1e-12),
              "seen %s" % seen2)

    # --- every refusal actually refuses ------------------------------------
    print()
    def refuses(name, mutate):
        e = json.loads(json.dumps(d))
        mutate(e)
        p2 = tmp + ".bad"
        with open(p2, "w") as fh:
            json.dump(e, fh)
        mm = Modal(p2)
        check(name, not mm.ok, mm.why[:64])

    refuses("refuses a wrong schema", lambda e: e.__setitem__("schema", "nope"))
    refuses("refuses modes at other frequencies",
            lambda e: e["modes"][1].__setitem__("f_hz", 1.20))
    refuses("refuses a stale file",
            lambda e: e.__setitem__("created", "2020-01-01T00:00:00"))
    refuses("refuses a Phi/A gauge mismatch",
            lambda e: e["a"].__setitem__("gauge", "somewhere else"))
    refuses("refuses a provisional A by default",
            lambda e: e["a"].__setitem__("provisional", True))
    def flip(e):
        # One mode's sign flipped on A alone: exactly the pump, and exactly what
        # the colocation identity is there to catch.
        e["a"]["value"][0] = [-x for x in e["a"]["value"][0]]
    refuses("refuses a sign-flipped A (the pump)", flip)
    refuses("refuses a missing A", lambda e: e.__setitem__("a", None))
    check("refuses a file that is not there", not Modal(tmp + ".nope").ok)

    print("\n  %s\n" % ("ALL PASS -- the modal math is right and every refusal "
                        "refuses.\n  NOT tested here, and only the bench can: "
                        "whether it damps the optic." if ok else
                        "FAILURES ABOVE -- do not run this on the bench."))
    return 0 if ok else 1


def main(argv=None):
    # Line-buffer stdout: Python block-buffers when it is not a terminal, so every
    # print here is invisible for 8 kB at a time whenever the run is piped or
    # captured -- which is how bench logs are kept. Measured 2026-08-17: an eta run
    # captured by jerk.py showed nothing after t=30 s while the loop ran fine.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--selftest" in argv:
        sys.exit(_selftest())
    fresh = ("--fresh" in argv
             or os.environ.get("OSEM_FRESH_CALIB", "0") not in ("", "0"))
    loaded, report = load_baseline(BASELINE_PATH, fresh=fresh)
    print()
    for line in report:
        print(line)
    print()

    # Printed BEFORE the coils are energised and before any prompt, out of the
    # object that is about to run: whether this is a modal run or a diagonal one
    # is the single most important fact about it, and it must not be something
    # you discover from the traces.
    for line in Modal(MODAL_PATH).report():
        print(line)
    print()

    dac = FastDAC(port=PORT)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))
    # The prompt guards against a coil-driving run started by accident, and is
    # meaningless with no terminal: input() then raises EOFError AFTER the biases
    # are applied, leaving a board at bias with nothing driving it.
    if sys.stdin.isatty():
        input("DAC biases set. Press Enter to start fast-lock damping (Ctrl+C to stop)... ")
    else:
        print("DAC biases set. stdin is not a tty -- starting without the prompt.")
    dac.start_stream()
    # The ONE place the modal law is opted into. See Controller.__init__.
    ctl = Controller(dac, baseline_file=loaded, modal_path=MODAL_PATH)

    os.makedirs("data", exist_ok=True)
    path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    log = open(path, "w", buffering=1)
    log.write(CSV_HEADER + "\n")
    print(f"Logging to {path}, every raw sample, control step at {CONTROL_HZ:.0f} Hz "
          f"(`ctl`/`n_avg` mark which samples each step averaged).\n\n"
          + (f"[{ctl.state}] warming up for {BASELINE_WARMUP_S:.1f}s and then "
             f"checking the stored floor against it...\n" if loaded is not None else
             f"[{ctl.state}] measuring baseline noise (outputs held at bias, no "
             f"damping yet). This stops as soon as the floor settles, and after "
             f"{CALIBRATION_S:.0f}s at the latest...\n"))

    start = prev = time.time()
    last_status, rows = 0.0, 0
    next_cue = None                 # armed on the first DAMPING sample, not here
    if KICK_CUE_S > 0:
        print(f"[kick-cue] speaking \"{KICK_CUE_PHRASE}\" every {KICK_CUE_S:.0f}s "
              f"once damping starts. Unset OSEM_KICK_CUE to silence it.\n")
    if GAIN_RAMP:
        lv, dw = GAIN_RAMP
        print(f"[gain-ramp] {len(lv)} levels, |Kp| {lv[0]:.4f} -> {lv[-1]:.4f}, "
              f"{dw:.0f}s each, about {len(lv) * dw / 60:.0f} min of DAMPING.\n"
              f"[gain-ramp] ATTENDED ONLY. This walks the loop toward instability "
              f"deliberately. Keep a scope on the coil drive and be ready to "
              f"Ctrl+C.\n")
    try:
        while True:
            sample = read_sample(dac.ser)
            if sample is None:
                continue
            counts, volts = sample
            now = time.time()
            dt, prev = min(max(now - prev, 1e-4), 0.05), now
            t = now - start
            ctl.step(counts, volts, t, dt)
            # Only ever True after a REAL calibration, never a reuse or a loaded
            # floor, so a stale number cannot refresh its own timestamp.
            if ctl.baseline_saveable:
                ctl.baseline_saveable = False
                try:
                    save_baseline(ctl)
                    print(f"\n[baseline] written to {BASELINE_PATH} -- the next "
                          f"start-up engages in ~{BASELINE_WARMUP_S:.0f}s instead of "
                          f"{ctl.calib_took:.0f}s, if nothing in the fingerprint or "
                          f"the room has moved.\n")
                except OSError as e:
                    print(f"\n[baseline] could NOT be written ({e}) -- the run is "
                          f"unaffected, the next one just calibrates.\n")
            for msg in ctl.drain_events():
                print(msg)
            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                print(ctl.status_line(t))
            # Cue only while DAMPING: kicking during CALIBRATING poisons the floor
            # being measured, and FAULT freezes the gains. The clock is ABSOLUTE:
            # re-arming per DAMPING entry cued 8 times in 240 s on a run with 12
            # faults against the 5 asked for.
            if KICK_CUE_S > 0:
                if next_cue is None:
                    if ctl.state == "DAMPING":
                        next_cue = t + _KICK_CUE_LEAD_S     # arm once, on first damping
                elif t >= next_cue:
                    next_cue += KICK_CUE_S
                    # Skip a cue that came due while faulted: kicking into a
                    # recovery measures the recovery.
                    if ctl.state == "DAMPING":
                        kick_cue()
                        print(f"[kick-cue] t={t:.0f}s -- KICK NOW")
            log.write(ctl.csv_row(t, counts) + "\n")
            rows += 1
            if rows % CSV_FLUSH_EVERY_N == 0:
                log.flush()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # PARK FIRST, and guard every step separately. `stop_stream()` blocks
        # waiting for an ACK a busy board may never send -- measured 2026-08-18,
        # it survived 90 s of SIGINT to both the process and its group while
        # holding the port. With the park behind it, a hang there left the coils
        # ENERGISED, which is the one outcome this block exists to prevent, and
        # it is why they had to be parked by hand three times that session.
        # One `try` around several steps has the same defect: the first raise
        # skips the rest.
        try:
            log.close()
        except Exception:
            pass
        print("Returning DAC outputs to bias voltages...")
        for c, b in zip(DAC_CHANNELS, BIAS):
            try:
                dac.set_voltage(channel=c, voltage=float(b))
            except Exception:
                pass                      # best effort; the board resets on reconnect
        time.sleep(0.1)
        try:
            dac.stop_stream()
        except Exception:
            pass
        try:
            dac.close()
        except Exception:
            pass
        print(f"Log saved to {path}")
        print(f"{rows} raw samples, {ctl.ctl_steps} control steps "
              f"({rows / max(ctl.ctl_steps, 1):.1f} averaged per step), "
              f"wire {ctl.wire_hz:.0f} Hz.")
        # Loud, because a rising count is a LINK problem and no amount of control
        # tuning addresses it. v11's own 240 s log had ten.
        if ctl.ramp is not None:
            print(ctl.ramp.summary())
        print(f"sample-guard: {read_sample.rejected} torn row(s) rejected at the "
              f"wire, {ctl.bad_samples} at the controller."
              + (" v11 would have fed every one of those straight into the "
                 "velocity estimate." if read_sample.rejected or ctl.bad_samples else ""))


if __name__ == "__main__":
    main()
