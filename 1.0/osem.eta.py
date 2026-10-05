#!/usr/bin/env python3
"""eta -- modal damping whose velocity estimate IS the modal state, hybrid with diagonal.

    x     = [q_m, v_m] per mode + one DC state per sensor
    y_i   = sum_m Phi[i,m] q_m + d_i + n_i
    f_m   = -K_m * (Proj qdot)_m                       per-mode gain, projected first
    u_C   = A_C+ f                                     min-norm, row-norm balanced
    u_j   = c_j * Kp * (-vel_j)                        coils A does not cover
    g_j   = KP_REF * h_j / max h                       NORMALISED, not flat

HYBRID, and this is the safety argument: colocated velocity feedback is
unconditionally dissipative whatever the mode shapes are, so the total power is
the sum of two non-positive terms and the hybrid is dissipative if each part is.
That holds provided the diagonal term on channel j feeds back channel j's OWN
velocity with the correct sign, which is what the diagonal law already does.
c_j is that channel's MEASURED coherent fraction: dissipation is linear in gain
and noise injection quadratic, so a noisy channel is gain-capped, not
disqualified, and c_j is the Wiener-optimal cap. See HYBRID_COHERENT.

WHY IT EXISTS: the pure modal law drove 4 coils of 8. Measured over 28775 modal
DAMPING samples, data/20260817_193855_fast_lock.csv: |out-bias| rms 0.027/0.033/
0.026/0.082 V on a0-a3 and exactly 0.000 on a4-a7, while a4/a6/a7 carried 43.4/
61.3/32.6 counts rms of residual motion that nothing observed and nothing damped.
About half of a4's and a7's in-band motion, and a quarter of a6's, IS the optic.

Phi COMES FROM GEOMETRY, and that is what removes the worst failure mode here.
Phi and A are each determined only up to a shared per-mode sign, so a measured
pair has a gauge that can be got wrong -- and getting it wrong PUMPS. It did, on
hardware 2026-08-17: an A whose per-mode signs were chosen by maximising
colocation consistency took the median channel ratio from 1.5 under the diagonal
law to 2.3 under modal. Nothing offline catches that, because a wrong-signed A is
still exactly inverted by the allocator -- the demanded modal force is realised
perfectly in the model and backwards in the plant. The gauge freedom exists ONLY
because Phi is measured. Taken from geometry Phi has no free sign, so A's sign
follows from the driven measurement with nothing left to guess, and the measured
Phi becomes a CHECK: |cos| against geometry per mode, 0.971 / 0.978 / 0.963
today, warned under 0.90. See PHI_GEOM.

WHAT IS MEASURED AND WHAT IS NOT.
  * modes 0.71519 / 0.99231 / 1.65307 Hz, re-measured 2026-08-20 16:25 after the
    coils were re-seated: 300 s, 105 013 samples, no drive
    (data/20260820_162534_status_sensors.csv), a0-a3 agreeing to 0.00050 Hz.
  * Q > 433 (1 sigma), tau > 138 s (analysis/ringdown.md): undamped on a 2.9-4.7 s
    closed loop, which is what lets the filter treat the modes as free oscillators.
  * the modal law measured median ratio 0.08 against diagonal's 1.58 on ZETA
    (signtest.py, 70 s per law, one trial each). eta changes the estimator, so
    that is the number to reproduce, not one it inherits. A repeat and a kick test
    come before it is quoted: `ratio` is an amplitude, so holding the optic and
    damping it look identical in it.
  * chi2 per dof is LOGGED and NOT acted on -- its distribution on this rig has
    never been measured, so a threshold would be a guess (mimo_closed.md 4.5).
    THE THREE-MODE MODEL IS INCOMPLETE and that is the leading explanation for the
    residual, ahead of sensor disagreement: the 300 s ambient record shows the
    in-plane sensors resonating at NINE frequencies -- 0.715, 0.992, 1.431, 1.661,
    1.984, 2.354, 2.400, 3.138, 3.415 Hz. READING 1.43 Hz AS A FOURTH MODE IS
    WITHDRAWN: the "1123-1947x" behind it was against a running-median floor in a
    power spectrum, a flattering statistic, and measured instead as a lock-in
    against neighbouring frequencies its warp SNR is 4.4, with 2.40 Hz at 1.6.
    Do NOT add a fourth mode: it would also make Phi square on four determined
    rows and delete the null space the graceful degradation needs.
  * THE PER-CHANNEL GAINS ARE NOT FLAT, and that is 2026-08-20's change. The four
    in-plane OSEMs do not share a counts-per-metre -- a2's sensor is 3.37x as
    sensitive as a0's -- so a flat vector is flat in COUNTS and 3.37x out in
    PHYSICS, and coil 2 and only coil 2 clipped. The gains are scaled by the
    measured sensor gain h, fitted from a constraint a rigid plate cannot violate
    (warp residual 141 -> 5 counts). Every mode still dissipates, checked
    numerically rather than assumed from a colocation that does not hold here.
    See SENSOR_GAIN_H and dissipation_eig.
  * per-mode Kp ships FLAT. Retuning it is a bench job: see MODAL_KP, where the
    one measurement anyone has quoted for it is withdrawn.
"""

import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import stdlib as sl                                              # noqa: E402

# Re-exported because they are part of the version contract harness.py and
# sim/server.py drive this module through.
SlidingRMS = sl.SlidingRMS
Modal, KalmanVelocity, ModalKalman = sl.Modal, sl.KalmanVelocity, sl.ModalKalman
OnePole, InBand, Chi2Log = sl.OnePole, sl.InBand, sl.Chi2Log

VERSION_TAG, BENCH_STATUS = "eta", "untested"
KIND = "controller"

PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 8

# ADC ai -> DAC channel. Measured pairing, bench/20260804/dcmatrix.log.
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]

FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend",
         "bias-trim", "soft-saturation", "baseline-floor",
         "sat-window", "fast-transport",
         "sample-guard", "decimate", "persist-baseline", "baseline-sanity",
         "kalman-velocity", "mains-null", "lock-quorum",
         "runaway-quorum", "runaway-peak", "runaway-veto", "trim-quiet",
         "modal-law", "modal-refuse", "modal-colocation", "modal-residual",
         "modal-kalman", "modal-chi2", "dead-pin",
         "inband-floor", "hybrid-diagonal", "fault-clear-live", "per-mode-gain",
         "gain-normalise")

# ---------------------------------------------------------------------------
# channels and gains  (bench.py reads these five vectors out of this module)
# ---------------------------------------------------------------------------
# a5 WAS a disconnected pin -- exactly one distinct value, 0.0 counts, variance
# exactly zero, across 236 387 samples in three records. IT IS NOT ANY MORE.
# Measured 2026-08-20 after the coils were re-seated: std 9.16 counts over a
# quiet record with 36 distinct values, i.e. a live ADC input on a moving optic.
# No cause is recorded for either the failure or the recovery, which is why this
# is enabled on measurement rather than on trust -- `dead-pin` still demotes it
# by variance if it goes back to being a pin, and its gain stays at zero until
# its coherence with a0-a3 has been re-measured.
# ITS COIL IS A SEPARATE QUESTION AND THE ANSWER IS NO: coil 5 moves a5 by
# -0.39 +-0.93 counts/V, 0.4 sigma, UNRESOLVED (`slopesign.py` 2026-08-20). It
# does move a1/a2/a3 by +9.3/-18.4/+11.1, so the coil has authority on the
# a0-a3 plane -- just not on its own sensor.
ENABLE_CHANNEL = [True, True, True, True, True, True, True, True]
# 2026-08-20: ALL EIGHT ON. ch2 and ch5 were both off at various points and both
# are now physically live; a rail is the CODE's problem, not a reason to switch a
# channel off in config. `dead-pin` demotes a railed-and-motionless channel and
# `rail clear` re-engages it -- both were observed working on the 12:38 run.
#   ch2 rests at 905.7 counts with max touching 1023 and is PINNED 22 % of a
#     60 s record (data/20260820_122926_status_sensors.csv). It railed during
#     calibration and faulted the whole rig at t=10 s on the 12:35 run. A bias
#     sweep says the offset is ELECTROMAGNETIC and trimmable at -193 counts/V,
#     but reaching mid-scale needs ~2.29 V against a 0.5 V window, so it cannot
#     be rescued until the window opens.
#   ch5 read EXACTLY 0.0 counts with variance exactly 0.0 over 236 387 samples
#     in three records and was disabled as a dead pin. It now reads 546.6 +/- 11.6
#     and carries mode B at SNR 44.1. Something in its signal path was fixed.

# A PER-CHANNEL BIAS VECTOR WAS SOLVED 2026-08-20 AND BIAS SHIPS UNIFORM. Recorded
# here because the solve is what says centring is unreachable, not because it is
# in force -- see BIAS. The bias is a static force, so it sets where each flag sits
# in its OSEM's shadow and an OSEM is linear only near half-shadow (MID_COUNTS
# 511.5); a2 rested at 884.0 counts and railed at 1023 for 40 % of a QUIET record.
# Solved as `min ||M v - (511.5 - resting)||^2` over the DC matrix M from
# `slopesign.py` (data/20260820_155638_status_slopesign.csv), subject to
# `BIAS_SWING <= v <= VMAX - BIAS_SWING` so every coil keeps a full swing of
# authority BOTH ways -- a coil biased at 0 V can only push one direction.
#
# CENTRING ALL FOUR IS UNREACHABLE AT ANY VOLTAGE, and the reason is geometric.
# M has cond 72.2 (singular values 309.1 / 117.6 / 60.4 / 4.3) and the direction
# that would centre all four is essentially WARP, which no rigid-body motion
# produces -- so it costs ~72x the voltage of the rigid directions. The
# unconstrained solve asks for -6.87 to +7.50 V, and the DAC is unipolar 0-2.5 V.
#
# WHAT IT ACTUALLY BOUGHT, measured not predicted: a2 884.0 -> 794.7 counts,
# railed 40 % -> 0.4 %. The solve predicted 764, so extrapolating M from its
# 0.10-0.40 V fit out to 0.75 V runs ~30 counts optimistic. DO NOT EXTRAPOLATE IT
# FURTHER, and re-solve rather than scaling a vector.

# TRANSPORT FRAMING. Binary: a fixed 20-byte frame, not ~32 B of ASCII.
#
# WHY IT WAS WANTED. The sample rate stepped 351.9 -> 226.5 Hz at the exact
# instant DAMPING began on the 17:12 run and stayed there -- gap p50 5.40 ms,
# p99 5.60 ms, ZERO gaps over 50 ms across 91 s. A rock-steady step is
# contention, not a supply sag, which would jitter.
#
# THE MECHANISM, and the first reading of it was wrong. It is NOT that `SET`
# competes with the stream for wire: the UART is FULL DUPLEX, so host->board
# bytes cost the board CPU time to parse but take no board->host bandwidth
# (`pyDAC2`'s own header says exactly this). What binds is that the ASCII stream
# is ALREADY nearly the whole downstream on its own. A row measured off the wire
# is `563,632,914,668,670,534,588,586\r\n` -- exactly 32 bytes -- so 352 Hz is
# 11.3 kB/s against 11.52 kB/s at 115200 8N1, about 98 %. The board has no slack,
# and once four coils start writing it must also parse ~400 SET/s and format
# eight integers per sample. The sampling loop is what gives.
#
# WHY THAT MATTERS MORE THAN THROUGHPUT: the applied voltage lags, and velocity
# feedback with enough phase lag is positive feedback. Measured on the same run
# as `slope x (u - bias) x velocity`, band-limited to the mode band with the
# common mode removed, every driven channel PUMPED -- dissipating on only 24.2 /
# 41.4 / 49.1 / 46.1 % of steps. A wrong SIGN gives ~0 %; a 90 deg phase lag
# gives 50 %. These are phase, not sign, and a0 -- the ONE channel whose gain was
# NOT changed that day -- was the worst of the four.
#
# IT NEEDED A REFLASH, AND THE BOARD HAD BEEN LYING ABOUT ITS FIRMWARE. Probed
# 2026-08-20 BEFORE flashing: `MODE BIN`, `MODE ASCII`, `VER` and `INFO` each
# returned `ERR unknown command` and forcing MODE BIN produced no 0xA5 0xC3 sync in
# a full second. The board was running an OLDER sketch than this tree's
# arduino.ino, so every capability read out of that file was a claim about SOURCE
# and not about the bench. After `bench.py --flash` the probe returns
# `OK mode=bin`, `INFO nch=8 mode=ascii presc=32 ack=1 frame=20`, and the sync
# appears. 20 B/frame is 7.0 kB/s at 352 Hz, 61 % of the link instead of 98 %, with
# no itoa per channel per sample, and `seq` makes lost frames COUNTABLE.
#
# NEVER SET THIS TRUE ON FAITH. A host in binary mode against an ASCII board reads
# NOTHING, and that failure is indistinguishable from a dead rig. Re-probe after
# any reflash.
BINARY_TRANSPORT = True

# 0.50 V, UNIFORM. Set 2026-08-20 evening, between a value known to work and one
# measured to fail, and the supply is the constraint -- THE RIG RUNS ON A 3 A
# SUPPLY (rig owner, 2026-08-20). Nothing computable from this repo bounds the
# coil current, because the coil driver and its resistance are not in the tree.
#
# ATTEMPT 1, midday: 1.10 V on all eight coils at once. Every analog input went to
# zero and STAYED there through a revert to 0.50 and then to 0.25 V. The board
# kept streaming at 822 Hz with zero dropped frames, so the transport was healthy
# and the front end was not. The coils had to be reset by hand.
#
# ATTEMPT 2, evening: 0.75 V with BIAS_SWING raised 0.25 -> 0.75 as well. The
# swing is the part that matters -- it sets how much current a coil can pull, and
# tripling it triples the demand. The rig owner stopped the run with THE POWER
# SUPPLY AUDIBLY ALARMING AT ITS LIMIT, and the data agrees
# (`data/20260820_171242_fast_lock.csv`):
#   * common-mode rms over all eight sensors went 15.4 -> 132.3 counts, and on
#     a4-a7 -- ZERO gain, coils pinned at exactly 0.750 V for the whole run --
#     78-83 % of their variance was that common mode. Channels nothing drives
#     cannot move mechanically; that is the shared rail feeding the OSEM LEDs.
#   * every DC level fell together against the census taken 47 minutes earlier:
#     -185 -197 -286 -251 -247 -242 -238 -237 counts, uniform across channels
#     whose coils barely couple to each other.
#
# 0.25 V HAS RUN FOR MONTHS WITHOUT EITHER SIGNATURE. 0.50 doubles the static
# operating point but leaves BIAS_SWING at 0.25, so the peak per-coil demand is
# 0.75 V -- the bias that sagged, but reached only at the extreme of the swing
# rather than held continuously on all eight. The swing is TRANSIENT, BIAS is
# CONTINUOUS, and the static load is 8 x 0.5^2 = 2.00 V^2, identical to what
# these two runs were compared against. VERIFY IT RATHER THAN TRUSTING IT: 30 s
# at bias with the gains at zero, checking that common-mode rms stays near its
# 15-count quiet level and that the DC levels do not drop as a block. Both are
# measurable before any coil is driven.
#
# A PER-CHANNEL CENTRING VECTOR WAS TRIED -- [0.250, 0.494, 0.750, 0.750, ...],
# solved against the DC matrix to pull a2 off the top rail -- and a2 did not move:
# 845 / 857 / 859 / 861 / 860 counts across five steps where the model predicted
# 762, railed fraction 5.8 % -> 10.8 %. THE CONCLUSION DRAWN FROM THAT ("a2 is not
# electrically trimmable from here") IS WITHDRAWN. The sweep stepped coils 2 AND 3
# TOGETHER and on a2 those two OPPOSE -- coil 2 gives -204.7 counts/V, coil 3
# +50.5 (data/20260820_155638_status_slopesign.csv) -- so moving both largely
# cancels there. Coil 2 alone does move a2, at about -205 counts/V, confirmed by
# two closed-loop trims: 861 -> 810 -> 758 -> 702 counts in 0.25 V steps
# (data/20260820_180344_fast_lock.csv) and 860 -> 828.8 -> 816.4 -> 759.8 on the
# 18:56 run. a2 is trimmable but not FAR ENOUGH: 348 counts off mid-scale against
# three affordable quanta x 205.94 = 154 counts, 44 % of what is needed. That is a
# REACHABILITY limit, not an electrical one -- see TRIM_MIN_REACHABLE_FRAC.
BIAS = np.full(N, 0.50)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 2.5, 2.0
# OPENED 2026-08-20, and the bench forced it rather than suggested it.
#   The DAC's hard ceiling is 2.5000 V: `arduino.ino` clamps at 25000 units of
#   100 uV and maps 0..25000 onto the AD5628's 0..4095 codes on its internal
#   reference. It is UNIPOLAR -- there is no negative side, so a negative bias
#   cannot be commanded at any setting.
#   0.0-0.5 V used 20 % of that and there was never a recorded justification for
#   it (CLAUDE.md Sec 11). What made it binding: on the 12:40 run every one of
#   coils 0-3 slammed BOTH rails, out spanning 0.000 to 0.500 V with an rms
#   demand of 0.089-0.148 V against a 0.25 V half-window, and the loop PUMPED --
#   counts std 81.8/86.8/170.4/84.7 at zero gain against 136.6/153.8/273.8/180.7
#   with the gain live, a factor 1.61-2.13, steady over twelve 20 s slices.
#   The signs were re-measured the same session and are NOT the cause
#   (slopesign.py: a0 +70.92, a1 +67.75, a2 -184.26, a3 +81.08 counts/V, and
#   STEADY_GAIN already carries a2 inverted). Clipping is: once a coil clips the
#   realised force is no longer -Kp*v and the dissipation guarantee is void.
#   HIGHEST VOLTAGE EVER COMMANDED ON THIS HARDWARE WITHOUT INCIDENT IS 1.1 V
#   (analysis/bias_sweep.py). Above that is unmeasured, and the coil driver
#   between the DAC and the coil IS NOT IN THIS REPO, so 0.25 V may have encoded
#   a real current limit. BIAS is the CONTINUOUS one: it sits on every coil for
#   the whole run, so if the driver is a voltage source into a resistive coil
#   the static dissipation goes as BIAS^2.     # nominal; the trim moves the window

# ---------------------------------------------------------------------------
# THE SENSOR CALIBRATION, and why a FLAT gain vector is not a flat law
# ---------------------------------------------------------------------------
# A gain in this file is volts of coil demand per volt of SENSOR velocity, and a
# sensor volt is counts. The four in-plane OSEMs do NOT share a counts-per-metre,
# so a flat gain vector is flat in COUNTS and lopsided in PHYSICS: the most
# sensitive sensor reports the largest velocity for the same motion of the plate
# and its coil is asked for the largest voltage. That is measured, it is large,
# and it is why exactly one coil clips.
#
# MEASURED 2026-08-20, two closed-loop runs, DAMPING blocks only, window
# BIAS 0.50 +- BIAS_SWING 0.25:
#   data/20260820_181504_fast_lock.csv (26 782 samples, 2 611 control steps in
#   DAMPING, one hand kick at t=23 s):
#     ch   peak |u - bias|   of the 0.25 V half-window   pinned at a rail
#     u0        0.1476 V              59 %                    0.00 %
#     u1        0.1915 V              77 %                    0.00 %
#     u2        0.3009 V             120 %                    5.55 %
#     u3        0.1942 V              78 %                    0.00 %
#   data/20260820_180344_fast_lock.csv (quiet, no kick): peak |u - bias|
#     0.0589 / 0.0558 / 0.1102 / 0.0545 V, nothing pinned on any channel.
#
# A SECOND SATURATION, SIX TIMES MORE FREQUENT THAN THE RAIL, AND IT WAS NOT IN
# THE BRIEF. `PID.drive` slew-limits at MAX_SLEW_PER_S = 2.0 V/s. Over the same
# 18:15 DAMPING block, |d(out)/dt| evaluated at the CONTROL clock:
#     ch   p50      p99      at the 2.0 V/s cap
#     u0   0.764    1.527         0.00 % of control steps
#     u1   0.900    1.999         1.15 %
#     u2   1.614    2.018        34.41 %
#     u3   0.873    1.929         0.54 %
# Coil 2 spent a THIRD of the run rate-limited. A rate limit is a saturation like
# any other -- the applied voltage is not what the law asked for -- and it is the
# same defect: u2 alone is asked for 2-3x the swing at the same physical motion.
# It is ALSO a phase lag, and this file's own DAMP_SIGN note is the reason to care
# (velocity feedback with enough lag stops dissipating). That last step is an
# ARGUMENT, not a measurement: the 18:15 run was hand-kicked at t=23 s, so its
# counts std rising 23.8/27.6/58.1/31.3 -> 47.4/56.6/93.8/53.8 is not evidence of
# pumping and is not offered as any.
#
# THE FIRST FIGURE IN THE BRIEF FOR THIS WORK WAS AN ARTIFACT -- "u2 pinned at a
# rail 57.0 % of samples" on data/20260820_180344_fast_lock.csv, which is what you
# get by judging a TRIMMED channel against the UNTRIMMED nominal window. The
# bias-trim section has it; do not re-derive it.
#
# ---------------------------------------------------------------------------
# WHERE THE CALIBRATION COMES FROM: A RIGID PLATE CANNOT WARP
# ---------------------------------------------------------------------------
# a0-a3 are four coplanar sensors reading one axis, so their readings span three
# rigid-body quantities plus WARP = [+1,-1,-1,+1] (PHI_WARP), which no rigid-body
# motion produces. A coil pushes a rigid plate. Therefore for EVERY coil j the
# static response expressed in PHYSICAL units must have zero warp component:
#
#     sum_i  w_i * h_i * M[i,j]  =  0,     w = PHI_WARP[:4] = [+1,-1,-1,+1]
#
# with M[i,j] the measured DC response of sensor i to coil j in counts/V and
# h_i proportional to 1 / (counts per metre) for sensor i. That is ONE EQUATION
# PER COIL. Over eight coils it is four unknowns in eight equations, so it is
# OVER-DETERMINED AND CAN FAIL. h is the smallest right singular vector of
# W[j,i] = w_i * M[i,j].
#
# THIS IS NOT THE ATTEMPT CLAUDE.md RECORDS AS A FAILURE. That one nulled warp in
# Phi over the three mode shapes: 3 equations in 4 unknowns, where a solution
# always exists and therefore proves nothing, and its two independent checks came
# out mixed. This one is over-determined and its residual is what carries the
# evidence.
#
# THE RESULT, from data/20260820_155638_status_slopesign.csv, all eight coils,
# singular values 310.69 / 117.98 / 60.82 / 5.03 (smallest/largest 0.0162):
#
#     warp residual per coil, counts,  raw -> h-calibrated
#       coil 0  -127.90 -> +3.92          coil 4   -6.90 -> -3.34
#       coil 1   +57.90 -> -5.09          coil 5  +14.90 -> -0.22
#       coil 2  +141.10 -> +4.11          coil 6  -10.20 -> +4.47
#       coil 3   -65.10 -> -5.29          coil 7   -0.40 -> +0.96
#     rms warp / rms |response|:  0.6224 -> 0.0322, a 19.3x reduction.
#
# RE-DERIVED INDEPENDENTLY FROM THE SAME RAW CSV WHILE WRITING THIS, with a
# different settle window on each dwell: h = [1.6084, 1.0598, 0.4665, 0.8653],
# singular values 316.17 / 122.08 / 61.13 / 5.39, per-coil calibrated warp
# +2.93 / -4.12 / +2.70 / -4.26 counts. Agrees to 1.7 % on every entry.
#
# AND IT IS CONFIRMED BY A ROUTE THAT NEVER SAW THE OFF-DIAGONAL. h correlates
# +0.9604 with 1/|diagonal slope| ([70.50, 88.87, 205.94, 98.79] counts/V, the
# response of each sensor to ITS OWN coil). h was fitted from the off-diagonal
# warp constraint across all eight coils and the diagonal entered nowhere. Two
# unrelated measurements of the same four numbers.
#
# SO a2's SENSOR IS 2.10x AS SENSITIVE AS THE MEAN AND 3.37x AS SENSITIVE AS a0.
# Under a flat gain in counts it demands 3.37x a0's voltage for the same motion
# of the plate. Normalising the gain by the sensor gain is a UNIT CONVERSION, not
# a fudge.
#
# ONE 300 s DC pass, one session, and h is a property of the OSEMs and their
# electronics, so it moves when they are re-seated -- which they were on
# 2026-08-20. RE-MEASURE IT with `slopesign.py --coils 0,1,2,3` after any
# hardware work, and re-solve rather than scaling this vector.
SENSOR_GAIN_H = np.array([1.6057, 1.0667, 0.4761, 0.8514])
# The DC response matrix the calibration and the dissipation check are both built
# on, coils 0-3 against sensors a0-a3, counts/V, from the same file. Its DIAGONAL
# is SENSOR_SLOPE_COUNTS_PER_V[:4] to within 1.24 counts/V, i.e. well inside the
# 5.6-30.5 counts/V uncertainty on those entries -- asserted in the selftest,
# because a transposed or re-ordered matrix here would silently invert the whole
# argument. INDEX ORDER IS D[coil][sensor]; M = D.T is [sensor][coil].
DC_MATRIX_COUNTS_PER_V = np.array([
    [+70.4,  -6.3, +155.5,  -49.1],       # coil 0 -> a0 a1 a2 a3
    [ -2.7, +88.9,  -88.5,  +61.0],       # coil 1
    [-40.7, +39.5, -204.7,  +16.6],       # coil 2
    [+24.1, -60.4,  +50.5,  -99.1]])      # coil 3

# ---------------------------------------------------------------------------
# the per-channel gains  (bench.py reads these five vectors out of this module)
# ---------------------------------------------------------------------------
# THE CEILING IS UNCHANGED. |Kp| = 0.035 is the largest per-channel gain any
# channel carries here, before and after normalisation: -0.040 is the documented
# rail onset and the least-evidenced number in the repo (analysis/kp040.md).
# Normalisation may only ever REDUCE a gain -- `slope_gain` clamps at KP_REF and
# the selftest asserts it -- so nothing in this change can put a channel
# somewhere no channel has been.
KP_REF = 0.035
KD_REF = 0.00045
# The channels the normalisation applies to: exactly the four with a Phi row and
# a determined h. a4/a5/a6/a7 have neither, and their gains are set by the hybrid
# section, not here. The selftest asserts this list is PHI_GEOM's rows.
GAIN_NORM_CHANNELS = (0, 1, 2, 3)
# One switch, so the bench can put the flat law back without editing a vector.
GAIN_NORMALISE = True

# The velocity-feedback sign on a0-a3. It is DAMP_SIGN[:4], repeated here only
# because DAMP_SIGN is declared further down with the measurement that fixes it;
# the selftest asserts the two are identical, so they cannot drift apart.
DAMP_SIGN_A0A3 = np.array([+1.0, +1.0, -1.0, -1.0])


def slope_gain(kp_ref, h=SENSOR_GAIN_H, sign=None, channels=GAIN_NORM_CHANNELS,
               normalise=GAIN_NORMALISE):
    """kp_ref on the LEAST sensitive channel, scaled DOWN by sensor gain on the rest.

    g_j = kp_ref * (h_j / max h) * sign_j, so g_j / h_j is the same on every
    channel: equal coil demand per unit PHYSICAL velocity of the plate. Because
    the reference is the largest h, no |g| ever exceeds kp_ref, and the clamp
    below makes that true even if a re-measured h says otherwise.
    """
    h = np.abs(np.asarray(h, float))
    idx = np.asarray(channels, int)
    g = np.zeros(N)
    if sign is None:
        sign = DAMP_SIGN_A0A3
    scale = (h / h.max()) if normalise else np.ones(h.size)
    g[idx] = kp_ref * scale * np.asarray(sign, float)
    return np.clip(g, -abs(kp_ref), abs(kp_ref))


# SHIPPED. `gain_report()` prints the vectors and the h/max(h) scale behind them,
# computed from the constants above rather than transcribed here -- a hand-copied
# vector goes stale silently the first time h is re-measured.
#
# PROJECTED ONTO THE RUN THAT CLIPPED, at the same motion (P and D scale
# together, so the whole demand scales by h_j/max h) --
# data/20260820_181504_fast_lock.csv, DAMPING:
#     ch   peak |u - bias|      of the half-window     at the 2.0 V/s slew cap
#     u0   0.1476 -> 0.1476 V     59 % ->  59 %          0.00 % ->  0.00 %
#     u1   0.1915 -> 0.1272 V     77 % ->  51 %          1.15 % ->  0.00 %
#     u2   0.3009 -> 0.0892 V    120 % ->  36 %         34.41 % ->  0.00 %
#     u3   0.1942 -> 0.1030 V     78 % ->  41 %          0.54 % ->  0.00 %
# Both saturations go away, and the peak demand across all four falls to 59 % of
# the window, so no per-channel window is needed (see BIAS_SWING).
#
# WHAT IT COSTS, STATED PLAINLY BECAUSE IT IS NOT SMALL. Every mode still
# dissipates -- that is the constraint below and it is checked numerically, not
# assumed -- but the worst mode dissipates LESS. Eigenvalues of the symmetric
# part of S = A diag(g) Phi, A = pinv(PHI_GEOM[:4]) @ DC_MATRIX^T:
#     flat  [+0.035,+0.035,-0.035,-0.035]   1.9393 / 4.0678 / 10.0229
#     h-normalised, shipped                 0.7593 / 2.4882 /  6.0345
# a factor 0.39 on the worst mode. In the physically consistent form -- modal
# force from the h-calibrated response, demand from diag(g/h) -- it is
#     flat 2.0343 / 4.1678 / 9.8134   against   shipped 1.2797 / 2.3935 / 4.7210,
# a factor 0.63. BOTH FORMS ARE POSITIVE DEFINITE, which is the whole safety
# argument, and both are asserted in the selftest.
#
# THE HEADROOM TO BUY IT BACK IS MEASURED AND IT IS 1.7x: the shipped vector
# peaks at 59 % of the half-window where the flat one reached 120 %. Spend it
# with OSEM_GAIN_RAMP, ATTENDED, which is what that tool is for -- not with a
# desk edit. Raising KP_REF raises every channel together and keeps g/h flat,
# which is the property this change exists to establish.
STEADY_GAIN = slope_gain(KP_REF)
CAPTURE_GAIN = slope_gain(KP_REF)
# Ki dissipates nothing and ate the headroom: peak 0.1135 -> 0.0056 V when it
# went to zero, which took total peak demand 0.2998 -> 0.200 V.
#
# AND IT IS WHY THE ANTI-WINDUP PATH NEEDS NOTHING DOING FOR A PINNED CHANNEL,
# checked while fixing the clipping. `PID.terms` feeds `clip_excess` back inside
# `np.where(self.ki != 0.0, ..., 0.0)`, so with KI_GAIN identically zero there is
# no state to wind up and the clip is MEMORYLESS: `p` and `d` are recomputed from
# the current velocity every step and the output leaves the rail on the first step
# the velocity reverses. u2 was pinned 5.55 % of the 18:15 DAMPING block and its
# `i` column is exactly 0.0000 for all 26 782 rows. stdlib gates the
# back-calculation on ki != 0 deliberately -- at ki = 0 it is a one-way ratchet and
# one clipped sample would leave a standing offset of up to I_CLAMP_V. Nothing to
# change; recorded so nobody hunts a windup bug that cannot exist yet. IT BECOMES
# LIVE THE MOMENT ANY KI_GAIN ENTRY IS MADE NON-ZERO.
KI_GAIN = np.array([+0.0000, +0.0000, +0.0000, +0.0000,
                    +0.0000, +0.0000, +0.0000, +0.0000])
# NORMALISED BY THE SAME FACTOR AS Kp, deliberately: the D term is in the same
# units, it saturates the same coil, and CLAUDE.md Sec 2 is the record of what
# happens when it is left out of an actuator budget -- it is added AFTER the
# modal cap and it is what took coil 3 to 0.4229 V against a 0.250 V half-window.
# Scaling P and D by the same factor also leaves each channel's P/D ratio, and so
# the loop shape, exactly where it was. Measured contribution on the run that
# clipped: |d| peaked at 0.0144 / 0.0190 / 0.0280 / 0.0173 V on a0-a3, i.e. 9 %
# of u2's peak demand -- small, but the same lopsidedness, 1.9x a0 on the channel
# whose sensor is 3.37x as sensitive.
KD_GAIN = slope_gain(KD_REF)

def gain_report():
    """What the bench needs to see before the coils are energised, and why.

    The gain vector is no longer flat, and a printed vector that is not flat
    reads as a typo unless the reason is printed with it. Everything here is
    computed from the constants above, so it cannot describe a different vector
    from the one about to be applied.
    """
    e_counts = dissipation_eig(STEADY_GAIN, "counts")
    e_phys = dissipation_eig(STEADY_GAIN, "physical")
    flat = KP_REF * np.concatenate([DAMP_SIGN_A0A3, np.zeros(N - 4)])
    if not GAIN_NORMALISE:
        return ["[gain] NORMALISATION OFF -- flat %s on a0-a3, the pre-2026-08-20 "
                "law. This is the null test, not the shipped configuration."
                % np.round(STEADY_GAIN[:4], 5)]
    return [
        "[gain] SLOPE-NORMALISED: a0-a3 carry %s against a flat %+.3f, because "
        "the four in-plane OSEMs do not share a counts-per-metre. Sensor gains "
        "h = %s (1 / counts-per-metre, mean 1), from the warp-null fit on "
        "data/20260820_155638_status_slopesign.csv."
        % (np.round(STEADY_GAIN[:4], 5), KP_REF * DAMP_SIGN_A0A3[0],
           np.round(SENSOR_GAIN_H, 4)),
        "[gain] g/h is %.6f on all four -- equal coil demand per unit PHYSICAL "
        "velocity. Under the flat vector it was %s, i.e. a2's coil was asked for "
        "%.2fx a0's at the same motion of the plate, which is why coil 2 and only "
        "coil 2 clipped (5.55 %% of DAMPING samples pinned, peak 0.3009 V against "
        "a %.2f V half-window) and spent 34.4 %% of the run at the %.1f V/s slew "
        "cap."
        % (float(np.abs(STEADY_GAIN[0]) / abs(SENSOR_GAIN_H[0])),
           np.round(np.abs(flat[:4]) / np.abs(SENSOR_GAIN_H), 4),
           float(np.abs(SENSOR_GAIN_H)[0] / np.abs(SENSOR_GAIN_H).min()),
           BIAS_SWING, MAX_SLEW_PER_S),
        "[gain] EVERY MODE STILL DISSIPATES, checked numerically and not assumed "
        "(colocation does not hold on this rig): sym(A diag(g) Phi) eigenvalues "
        "%s counts-form and %s physical-form, all positive. The flat vector gave "
        "%s and %s, so the worst mode loses a factor %.2f -- that is what this "
        "costs, and the headroom to buy it back with OSEM_GAIN_RAMP is measured "
        "at 1.7x."
        % (np.round(e_counts, 4), np.round(e_phys, 4),
           np.round(dissipation_eig(flat, "counts"), 4),
           np.round(dissipation_eig(flat, "physical"), 4),
           float(e_counts.min() / dissipation_eig(flat, "counts").min())),
    ]


CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
SCHEDULE_WINDOW_S, GAIN_SLEW_PER_S = 1.0, 0.02
D_SMOOTH_HZ, I_CLAMP_V, TRACK_TC_S = 2.0, 0.15, 0.5

# The control clock is decoupled from the wire: the wire has been 12.5-1113 Hz.
CONTROL_HZ = 100.0
CONTROL_PERIOD_S = 1.0 / CONTROL_HZ
MAINS_NULL = True              # two-step boxcar = a null at 50 Hz

# ---------------------------------------------------------------------------
# the modes and the estimator
# ---------------------------------------------------------------------------
# Empty room, 840 s, data/20260817_215634_status_sensors.csv: spread 0.00000 Hz
# across all four in-plane sensors. A SMALL CORRECTION, NOT A FIX -- the previous
# values differ by <= 0.001 Hz, half a half-width. KALMAN_K below is a
# steady-state gain solved at those previous values and is not re-solved for a
# 0.14 % shift.
F_MODE_HZ = np.array([0.71519, 0.99231, 1.65307])
      # RE-MEASURED 2026-08-20 16:25 on the bench, AFTER the coils were re-seated
      # and the per-channel bias applied -- 300 s, 105 013 samples, no drive
      # (`data/20260820_162534_status_sensors.csv`). Consensus over a0-a3, whose
      # per-sensor peaks agree to 0.00050 / 0.00050 / 0.00000 Hz.
      #
      # a5 is DELIBERATELY EXCLUDED from the consensus even though the census
      # includes it. It carries mode B at SNR 30.1 but modes A and C at 6.4 and
      # 2.4, and its mode-C peak lands at 1.61857 against a0-a3's unanimous
      # 1.65307 -- a 0.0345 Hz outlier, 41 half-widths, which is a noise peak and
      # not a measurement. Averaging it in is what made the tool's own printed
      # C spread 0.03450.
      #
      # Was [0.71394, 0.99393, 1.65182] at 12:29, which was itself measured this
      # morning: the shifts are +0.00125 / -0.00162 / +0.00125 Hz, i.e. 1.5 / 1.9
      # / 1.5 half-widths (the half-width is 0.00083 Hz). Small, but a half-width
      # is the unit that matters -- a drive off by n half-widths gets
      # 1/sqrt(1+n^2) of the on-peak response AND the wrong phase.
      #
      # Against the 2026-08-06 set the shifts are -0.00675 / +0.00038 / -0.00300,
      # so mode A is 8.1 half-widths off and anything still carrying 0.72194 is
      # driving off resonance.
NMODE = len(F_MODE_HZ)
T_AMP_S, T_DC_S = 50.0, 20.0   # amplitude and offset wander timescales, measured

# Per-channel measurement variance, V^2 (analysis/kalman.md 2).
KALMAN_R = np.array([4.673429582e-03, 3.735535655e-05, 3.647149147e-03,
                     4.858924713e-05, 3.316806993e-05, 5.845702231e-04,
                     1.201535857e-04, 2.386431425e-05])
# Per-channel, per-mode process variance V^2/s^3 (analysis/kalman.md 3).
KALMAN_Q = np.array([
    [1.352121665e-03, 1.537383088e-01, 2.491687224e-02],
    [4.413914307e-04, 2.130937682e-02, 1.365794376e-02],
    [2.776031283e-03, 3.323627307e-01, 5.187054209e-02],
    [3.262378479e-04, 1.603699462e-02, 1.059346571e-02],
    [1.146130994e-07, 3.329487040e-06, 1.710179103e-06],
    [1.301556016e-05, 2.676339313e-02, 3.848128557e-06],
    [2.485429127e-07, 6.636087553e-07, 1.259370922e-06],
    [1.305297122e-07, 1.553854780e-06, 5.111513956e-07]])
KALMAN_Q_DC = KALMAN_R / T_DC_S
# Steady-state per-sensor gains, one row per channel, 7 states.
#
# KEPT AS LITERALS, and `stdlib.steady_state_k` re-derives them from
# (KALMAN_R, KALMAN_Q, F_MODE_HZ, CONTROL_PERIOD_S) if you want to check.
# THE TWO DISAGREE BY UP TO 51x AND IT DOES NOT MATTER, which is the useful
# finding. Re-deriving flips the sign of column 1 -- mode A's velocity state -- on
# a0-a3 and a5 and makes the DC column 9x larger, yet over the same 41 313
# decimated control steps of data/20260820_173738_fast_lock.csv the two velocity
# estimates agree against a zero-phase reference: corr(vel, true dv/dt)
# 0.597/0.710/0.747/0.749 for these literals against 0.519/0.723/0.706/0.748
# derived. The filter output is insensitive to K here, so swapping a constant with
# hardware history for one that measures no better is not a trade worth making.
# THE DISAGREEMENT IS NOT EXPLAINED, and it is not the 2026-08-20 frequency
# re-measurement: at the 2026-08-06 frequencies it is 50.1x against 51.2x today.
# Either the literals came from a different Q convention than `steady_state_k`
# assumes, or they were never this model's steady-state gain. Open item.
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
RAIL_BLANK = False             # blanking a railed sample costs more phase than it saves

# ---------------------------------------------------------------------------
# calibration, lock, interlocks
# ---------------------------------------------------------------------------
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # ceiling; subwindows >= 3
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20

LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0

ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE = 2.0, 1.8
# 4.0 -> 5.0 ON 2026-08-20, and the number is measured, not chosen. It moved
# because stdlib.Breaker changed underneath it: a recession now VETOES the trip
# instead of CLEARING the latch, so the latch a hand kick can accumulate is no
# longer chopped up by the plate's own beat ripple and the sustain has to cover
# the whole kick.
#
# Sourced from every closed-loop bench record in data/, replayed through the real
# stdlib.Breaker (channels a0-a3, DAMPING only): 13 runs x 4 channels = 52
# channel-instances covering nine hand-kick runs, three quiet closed-loop runs and
# the pumped run 20260817_191710. At 5.0 s NOTHING trips. At 4.0 s exactly one
# does -- 20260818_024302 ch2 at t=39.1 s, the FIRST hand kick of the only valid
# kick set this rig has -- and that is CLAUDE.md 1 all over again.
# The longest continuous `high & growing` a hand kick produced anywhere in that
# corpus is 5.28 s (20260818_024302 ch3, t=150.7-156.0, envelope 0.245 -> 0.630 V,
# i.e. ratio 2.0 -> 5.2 at a ring-up rate of 0.27 /s).
#
# WHAT IT COSTS: one extra second before a runaway trips. On the same records
# multiplied by exp(g t) the trip still fires at 0.05, 0.0739 and 0.15 /s (2, 3
# and 4 of the four channels, and the controller trips on `.any()`), where the
# rule that shipped until today caught NONE of them.
#
# RAISING IT FURTHER IS NOT FREE, and 8.0 was tried and reverted once already.
# Measured now: at 6.0 s the 0.05 /s runaway -- roughly what the simulator's
# pumped resonance does -- stops tripping on every channel, and stdlib's own
# "a ring-up longer than one period is treated as a push" fixture stops firing.
RUNAWAY_SUSTAIN_S = 5.0
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 10.0, 1.02

SAT_SUSTAIN_S, SAT_FRACTION = 1.30, 0.80
SAT_DECAY_FRAC = 0.98

BREAKER_REF_FLOOR_FRAC = 0.10   # of the median driven reference. Same number and
                                # same argument as BASELINE_FLOOR_FRAC below: a
                                # channel whose own reference is a small fraction
                                # of what everything else reads cannot have a
                                # meaningful RATIO against it.
BASELINE_FLOOR_FRAC = 0.10     # of the median IN-BAND amplitude across enabled

RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80
DEAD_PIN_STD_COUNTS = 1.3      # sqrt(0.853 x 1.998): dead a6/a7 max 0.853, sim
                               # occlusion min 1.998, live never below 4.80

FAULT_CLEAR_SUSTAIN_S = 5.0
# A runaway must also have come DOWN before the loop re-engages. Under
# RUNAWAY_MULTIPLE = 1.8, so the breaker cannot re-trip on the sample after the
# gate opens.
FAULT_CLEAR_RATIO = 1.4
# ...but not forever. A breaker that never re-closes is not safer than one that
# re-trips, and a re-trip costs one FAULT_CLEAR_SUSTAIN_S. Same pattern as
# MAX_BASELINE_REFUSALS: hold, then say so and go anyway.
#
# WAS 120.0, AND THAT IS EXACTLY WHY IT NEVER FIRED. Measured 2026-08-18, both
# runs faulted on the first hand kick and sat there to the end: diagonal FAULT
# 118.7 s (data/20260818_001841_fast_lock.csv), modal 94.5 s
# (data/20260818_002229_fast_lock.csv). The diagonal run MISSED THE CEILING BY
# 1.3 s. A ceiling longer than a whole jerk.py run is not a ceiling.
# The re-derivation, from measurements in this repo: the LOOP is what brings
# motion down, at a measured re-quiet of 5.3 s modal / 17.4 s diagonal
# (CLAUDE.md), while at zero gain the plant decays at 0.0072 /s
# (analysis/ringdown.md) -- so ringing down from a trip at ratio 3.8 to
# FAULT_CLEAR_RATIO takes ln(3.8/1.4)/0.0072 = 139 s and ANY ceiling under that is
# the operative path. 30.0 s is longer than the slowest re-quiet, so the loop gets
# a full chance first, and far under the 138 s intrinsic. THE REMAINING FACTOR IS
# A CHOICE: 30 s is 1.7x the 17.4 s re-quiet and 6x the FAULT_CLEAR_SUSTAIN_S a
# re-trip costs, and the conservative direction is LONGER.
FAULT_CLEAR_MAX_HOLD_S = 30.0
REARM_SUSTAIN_S = 2.0

# ---------------------------------------------------------------------------
# the RUNNING loop's own quiet level
# ---------------------------------------------------------------------------
# `baseline` is measured during CALIBRATING, which is a ZERO-GAIN window, so it
# describes the UNDAMPED plate. Measured 2026-08-18 against one such window: the
# modal law holds per-channel `ratio` 0.117 and the diagonal law 1.449 / 1.832 /
# 1.769 -- about 13x apart, from the same reference. Anything scaled off it is
# therefore scaled off the open-loop plant, which is right for "is the loop making
# this worse" and wrong for "is this loop where it normally lives".
#
# WINDOW. 30 s. Two bounds, both from measurements here: it must hold several
# cycles of the slowest mode (0.71519 Hz, 1.398 s) so the envelope is a level and
# not a phase; and at the 20th percentile the answer survives a disturbance
# occupying up to 80 % of the window, so with the diagonal law's measured 17.4 s
# re-quiet it needs 17.4/0.8 = 21.8 s at minimum. 30 s leaves the quiet 42 %
# setting the answer through a full diagonal re-quiet.
QUIET_WINDOW_S = 30.0
# PERCENTILE, not a mean: the number is read exactly when the loop has been
# kicked, and a mean follows the kick. 20th leaves 4x headroom against the
# 17.4 s/30 s worst case above.
QUIET_PERCENTILE = 20.0
# The envelope is already a 2 s sliding RMS, so samples one control step apart are
# not independent. 10 Hz keeps 300 points in the window and costs nothing.
QUIET_KEEP_HZ = 10.0
# Blind until the window is this full. Until then `level()` is None, every caller
# falls back to `baseline`, and behaviour is bit-identical to what shipped.
QUIET_MIN_FILL = 0.80
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md 3)

# ---------------------------------------------------------------------------
# bias trim
# ---------------------------------------------------------------------------
# EXAMINED 2026-08-20 AND DELIBERATELY NOT CHANGED. The brief for the clipping
# work asked whether the trim is helping or hurting on ch2, since it had walked
# that channel to 1.00 V and ch2 still appeared to clip. Three findings, in the
# order they matter:
#
# 1. THE TRIM DID NOT CAUSE THE CLIPPING, AND ch2 DID NOT CLIP ON THAT RUN.
#    "u2 pinned at a rail 57.0 % of samples" on data/20260820_180344_fast_lock.csv
#    is what comes out of judging a TRIMMED channel against the UNTRIMMED nominal
#    window, 0.25-0.75 V; `_moved` moves vmin/vmax with the bias. Against its own
#    window ch2 touched a rail on 0.0000 of that run's 29 733 DAMPING samples,
#    and its peak demand about bias was 0.1102 V, 44 % of the half-window. The
#    real clipping is on the 18:15 run, 5.55 %, with the bias still at 0.50 V.
#
# 2. THE TRIM WORKED. Three steps at t = 35 / 50 / 65 s took ch2's bias 0.50 ->
#    0.75 -> 1.00 -> 1.25 V and a2's resting counts went 861 -> 810 -> 758 -> 702,
#    i.e. -204 to -224 counts/V, within 10 % of a2's measured DC slope of
#    -205.94 counts/V over the whole range. This is what settled the BIAS
#    comment's stepped-sweep negative result -- that sweep moved coils 2 and 3
#    together and they oppose on a2.
#
# 3. WHAT IT DOES COST IS NOT CLIPPING, IT IS THE SUPPLY, and it is unremarked
#    anywhere else. BIAS_MAX 1.25 plus BIAS_SWING lets one coil be commanded to
#    BIAS_MAX + BIAS_SWING = 1.50 V, and the 18:03 run reached 1.329 V on coil 2.
#    The BIAS comment above reasons about a peak per-coil demand of 0.75 V, and
#    records that 1.10 V is the highest voltage ever commanded on this hardware
#    without incident and that 1.10 V on all eight at once took every analog
#    input to zero. BIAS_MAX predates BIAS being raised 0.25 -> 0.50 and was
#    never re-examined against it. NOT CHANGED, for one reason: the 18:03 run held coil
#    2 at 1.25 V for 20 s and finished clean and very quiet (a2 counts std 5.3),
#    so cutting BIAS_MAX would remove authority that has been measured to work on
#    the evidence of a reasoning gap. It is a bench decision, not a desk one.
#    Note also that TRIM_MAX_TOTAL_EXCURSION_V = 0.75 is exactly three
#    BIAS_QUANTUM steps, so ONE channel can spend the entire excursion budget --
#    which is what ch2 did.
BIAS_QUANTUM = 0.25            # coarse: 2 DOF, 4 knobs, so fine steps would
BIAS_MIN, BIAS_MAX = 0.25, 1.25          # just chase each other
# ONE GLOBAL HALF-WINDOW, AND A PER-CHANNEL ONE IS NOT NEEDED ONCE THE GAINS ARE
# NORMALISED. On the run that clipped, peak |u - bias| was 59 / 77 / 120 / 78 %
# of the half-window; with the h-normalised gains the four come to 59 / 51 / 36 /
# 41 %, which is one window fitting all four. A per-channel window would also
# have to be justified against a coil driver that is not in this repo
# (CLAUDE.md Sec 11). NOT ADDED.
#
# 0.25 SHIPS, AND THE CASE FOR 0.50 IS RECORDED HERE RATHER THAN ACTED ON.
# THE CASE FOR RAISING IT, measured 2026-08-20. A hand kick during a DIAGONAL run
# took `ratio` 0.31 -> 4.99 (16.25x) and THE LOOP NEVER RECOVERED: ratios were
# still 1.5-3.2 a full 20 s later and ch2 had been dropped from the loop. The
# commanded voltages over that DAMPING block, against BIAS 0.50 +- 0.25:
#
#     coil    min      max                a2's counts spanned 0 to 1023,
#     u0     0.250    0.750  <- BOTH      i.e. the FULL ADC scale.
#     u1     0.250    0.750  <- BOTH
#     u3     0.250    0.750  <- BOTH
#     u2     0.380    0.702
#
# Three of four coils on BOTH rails. Once a coil clips the realised force is no
# longer -Kp x velocity and the dissipation guarantee is void (CLAUDE.md Sec 2),
# so that kick was lost to RUNNING OUT OF ACTUATOR, not to the law: the same law
# locks at ratio 0.01-0.07 unsaturated (`LOCKED` at 12.8 s, 2026-08-20 19:4x).
#
# WHY IT IS NOT TAKEN. 0.50 has never been on hardware; 0.25 is the value the
# 12.8 s lock was measured at, and this file is being handed on. And 0.50 opens a
# voltage nobody has measured: `_trim` can walk a coil to BIAS_MAX and `PID.drive`
# clips to vmin/vmax only, so the peak commandable is BIAS_MAX + BIAS_SWING --
# 1.50 V at 0.25, which is the pre-existing exposure, against 1.75 V at 0.50.
# The highest ever commanded here without incident is 1.10 V
# (analysis/bias_sweep.py; the 18:03 run reached 1.329 V on coil 2). Raising the
# swing is gated on somebody characterising the coil driver and the 3 A supply.
#
# WHY BIAS_SWING AND NOT BIAS IF IT IS EVER RAISED. BIAS is a CONTINUOUS load on
# all eight coils, so the steady dissipation goes as BIAS^2; the swing is only
# reached at the extremes of a correction. Raising BIAS to 0.75 V AND the swing
# to 0.75 V together had THE POWER SUPPLY AUDIBLY ALARMING -- common-mode rms
# 15.4 -> 132.3 counts, every DC level falling together by 185-286 counts
# (data/20260820_171242_fast_lock.csv, full account in the BIAS comment).
#
# MODAL_TOTAL_HEADROOM is a FRACTION of this, so the cap on the TOTAL output
# follows it automatically (theta names that number BUDGET_V). MODAL_DEMAND_CAP_V
# is an ABSOLUTE volt and deliberately does not -- see its own comment. Nothing in
# `_trim` reads BIAS_SWING; only `_moved`/`_bias_ramp` do, to slide vmin/vmax.
BIAS_SWING = 0.25              # +-this around each channel's own bias
MID_COUNTS = 511.5             # (ADC_MAX_COUNTS - 1) / 2

# ---------------------------------------------------------------------------
# A BIAS STEP IS A FORCE STEP. RAMP IT.
# ---------------------------------------------------------------------------
# ADDED 2026-08-20 EVENING, and the bench found this, not a desk. On
# data/20260820_185657_fast_lock.csv -- the first run with the normalised gains,
# all four Phi channels locked at ratio 0.05-0.09 -- the rig owner saw "stray
# waves" and "bumps" during lock, and every one of them is a trim step. T2 modal
# residual in 2 s windows against ch2's commanded bias:
#
#     t(s)   u2 mean   a2 mean   T2 rms
#      12     0.500     860.0     1.85      settled
#      14     0.619     828.8    15.28      TRIM STEP
#      16     0.753     816.4     9.99
#      24     0.750     813.1     2.00      re-settled, about 10 s later
#      30     0.993     759.8    13.35      TRIM STEP
#      36     1.000     758.6     4.12      still recovering
#
# 7-8x on the residual, about 10 s to recover, and TRIM_PERIOD_S was 15.0 s, so
# the steps STACKED and the loop never got clear air. Band power over the whole
# DAMPING block against CALIBRATING says the same thing and says where it goes:
# the mode band 0.60-1.90 Hz is DOWN 8.5x -- the loop works -- while 0.02-0.30 Hz
# is UP 2950x. A velocity damper has no mechanism to inject sub-0.3 Hz motion.
# The trim does.
#
# THE MECHANISM IS ALREADY WRITTEN DOWN IN THIS REPO, in `status.park_ramped`:
# a bias change is a FORCE STEP into an undamped plant, which is exactly the
# principle `_dc_point` uses to MEASURE the DC matrix, and it has to be spread
# over about ten periods of the slowest mode to be adiabatic. The trim did not
# ramp. It handed the new bias straight to the actuator, where the only thing
# slowing it was MAX_SLEW_PER_S = 2.0 V/s -- 0.25 V in about 125 ms, roughly 120x
# too fast. That ramps the COIL, not the PLATE.
#
# 15.0 s is 10.7 periods of the slowest mode (0.71519 Hz, 1.398 s), the same
# number `status.BIAS_RAMP_S` uses and for the same reason. Excitation by a ramp
# falls with its duration; a step is the worst case.
BIAS_RAMP_S = 15.0
# The commanded bias moves at most this fast, in volts per second. One
# BIAS_QUANTUM spread over BIAS_RAMP_S, i.e. 0.0167 V/s against a MAX_SLEW_PER_S
# of 2.0 -- 120x slower, which is the whole point. Derived, not typed, so
# changing the quantum cannot silently change the ramp.
BIAS_RAMP_PER_S = BIAS_QUANTUM / BIAS_RAMP_S
# WAS 15.0 s, AND THAT WAS SHORTER THAN THE RECOVERY IT CAUSED. The measurement
# above puts recovery at about 10 s after a stepped trim, so a 15 s period let
# the next step land on the tail of the last. 45.0 s is the ramp (15 s) plus a
# full QUIET_WINDOW_S (30 s) of settled data before the step is judged or another
# is started. It costs step COUNT on a short run -- the 18:03 run took three
# steps in 51 s of DAMPING and would now take one -- and that is the intended
# trade: a correction that costs 10 s of lock is one to make rarely.
TRIM_PERIOD_S = 45.0
TRIM_DEADBAND_COUNTS = 40.0    # inside this, leave it alone
# DO NOT SPEND A DISTURBANCE ON A CORRECTION THAT CANNOT CONVERGE. A trim step
# costs real motion, so it is only worth taking if the budget left can actually
# close the offset. Before starting a step this asks how far the channel could
# still move -- steps left, the total excursion budget and the BIAS_MIN/BIAS_MAX
# clamps, whichever binds first -- multiplies by that channel's MEASURED slope,
# and refuses if the answer cannot cover this fraction of the offset.
#
# IT REFUSES EXACTLY THE TWO CHANNELS THIS FILE ALREADY DOCUMENTS AS UNREACHABLE,
# and it refuses them from measured numbers rather than from a name:
#   a2  rests near 860 counts, 348 off mid-scale, slope -205.94 counts/V. Three
#       affordable steps of 0.25 V reach 154 counts, 44 % of the offset. REFUSED.
#   a4  drifts 34 counts, slope +5.06 counts/V. Its whole budget is worth 1.3
#       counts, 4 % of the offset -- the existing note above says 14 % using the
#       full four-step budget and it is the same conclusion either way. REFUSED.
# A channel with a modest offset is unaffected: 100 counts on a1 (+88.87
# counts/V) is 67 % reachable and still trims.
#
# WHAT THIS IS NOT: a claim that a2 is electrically untrimmable. Two closed-loop
# runs measure the trim moving it at -204 to -224 counts/V (see BIAS). It cannot
# move it FAR ENOUGH inside its own budget, which is a different statement.
TRIM_MIN_REACHABLE_FRAC = 0.5
# NOT LOWERED, AND LOWERING IT WOULD NOT HELP. a4 drifted 34.0 counts over 150 s
# on 2026-08-17 (data/20260817_233920_status_sensors.csv; every other channel moved
# <= 4.2 counts, and a4 carries 26 % of its power under 0.1 Hz, peaking at
# 0.0230 Hz -- a 43.5 s period at 9.3 counts). That sits just inside this deadband
# forever, so the trim never sees it. But the trim could not correct it either:
# a4's measured slope is +4.83 counts/V, so one BIAS_QUANTUM step moves it
# 0.25 x 4.83 = 1.21 counts and TRIM_MAX_STEPS = 4 gives 4.8 counts of total
# authority against 34 counts of drift -- 14 % of what is needed. The deadband is
# not the binding constraint; the quantum and the step budget are. a4 also has no
# Phi row, so nothing else acts on it. NOT FIXED, recorded.
TRIM_MAX_STEPS = 4             # per channel, per run
TRIM_MAX_TOTAL_EXCURSION_V = 0.75   # two coils far from nominal gave three
                                    # runaways on 2026-08-07
# SIGN OF d(counts)/d(bias volts) PER CHANNEL -- the PHYSICAL slope, and nothing
# else. Which way the sensor moves when its own coil pushes.
#
# THIS VECTOR USED TO HOLD TWO DIFFERENT CONVENTIONS AT ONCE, AND THAT WAS A LIVE
# SIGN BUG. Until 2026-08-20 it read [-1,-1,+1,-1, +1,-1,+1,+1]. The a0-a3
# entries were never measured -- the comment said so, "from the per-channel decay
# fits" -- they were copied from STEADY_GAIN, which is the NEGATED slope. The
# a4/a6/a7 entries came from an actual DC step (+4.83, +10.07, +8.05 counts/V)
# and so were the slope itself, unnegated. `HYBRID_GAIN` then multiplied the lot
# by HYBRID_KP, giving a4/a6/a7 a gain with the SAME sign as their slope where
# a0-a3 get the opposite -- positive feedback on three channels.
# It never fired: `inband-floor` demoted a4/a6/a7 at engage on both 2026-08-18
# runs, |gain| exactly 0.0000 for 100 % of DAMPING samples (CLAUDE.md Sec 6). The
# fix is to keep the slope and the gain sign in SEPARATE names so they cannot
# drift into each other again.
#
# ALL EIGHT MEASURED 2026-08-20 after the coils were re-seated, one coil at a
# time, 0.10 -> 0.40 V, 12 s dwell (`slopesign.py`,
# `data/20260820_155638_status_slopesign.csv`), counts/V on the channel's own
# sensor, with the uncertainty of the MEAN over 1 s blocks:
#   a0  +70.50 +-5.63   12.5 sigma        a4   +5.06 +-0.46   10.9 sigma
#   a1  +88.87 +-10.15   8.8 sigma        a5   -0.39 +-0.93    0.4 sigma  UNRESOLVED
#   a2 -205.94 +-30.46   6.8 sigma        a6  +10.37 +-0.30   34.9 sigma
#   a3  -98.79 +-19.81   5.0 sigma        a7   +8.07 +-0.17   46.2 sigma
# a6/a7 reproduce the 2026-08-17 values (+10.07, +8.05) to well inside sigma, so
# the reset did not disturb them. a4 is now 10.9 sigma where it was 2.9.
#
# a3 CHANGED SIGN IN THE RESET: +81.08 counts/V before it, -98.79 after. That is
# not drift, it is the coil or the OSEM remounted the other way round, and it is
# why STEADY_GAIN[3] is now positive. Nothing in software could have caught it.
#
# a5's coil does not move a5: -0.39 +-0.93 counts/V, 0.4 sigma. The SENSOR is
# alive (std 9.16 counts over a quiet record, 36 distinct values, where it was a
# hard-grounded pin at exactly 0.0 for 236 387 samples), so this is an actuator
# result, not a sensor one. Its gain stays zero.
# THE MAGNITUDES ARE NOW LOAD-BEARING TOO, so they are kept rather than reduced
# to a sign at the point of measurement. Their RATIOS are what says a flat gain
# vector is not a flat law -- see SENSOR_GAIN_H, which is fitted from a different
# part of the same DC pass and agrees with 1/|slope| at correlation +0.9604.
# a5 is UNRESOLVED at 0.4 sigma and is never normalised against: it is not in
# GAIN_NORM_CHANNELS, its STEADY_GAIN is zero, and HYBRID_CHANNEL[5] is False.
SENSOR_SLOPE_COUNTS_PER_V = np.array([+70.50, +88.87, -205.94, -98.79,
                                      +5.06,   -0.39,  +10.37,  +8.07])
SENSOR_SLOPE_SIGMA = np.array([5.63, 10.15, 30.46, 19.81,
                               0.46,  0.93,  0.30,  0.17])
# DERIVED, not restated. Copying the signs by hand into a second vector is
# exactly the two-conventions bug described above, so there is no second vector.
SLOPE_SIGN = np.sign(SENSOR_SLOPE_COUNTS_PER_V)

# The gain sign for velocity feedback, and it CARRIES the slope's sign.
#
# THIS WAS -SLOPE_SIGN UNTIL 2026-08-20 AND THAT MADE THE LOOP PUMP. The algebra
# is short and it is not ambiguous. `Pid.terms` sets p = gain x (-vel), so the
# demand about bias is (u - bias) = -gain x vel. The force a coil applies along
# its own sensor's coordinate is proportional to slope x (u - bias), so the power
# delivered is
#
#     slope x (u - bias) x vel  =  -slope x gain x vel^2
#
# and vel^2 >= 0, so DISSIPATION REQUIRES slope x gain > 0. Same sign. With
# gain = -sign(slope) every driven channel pumps, which is what the bench
# measured.
#
# MEASURED, on `data/20260820_173738_fast_lock.csv`, evaluated at the CONTROL
# CLOCK (the CSV logs every wire sample, but bp/vel/out only change once per
# control step, so correlating against the 100 Hz staircase inside a 581 Hz
# record understates the velocity term -- an earlier reading of this same file
# got the diagnosis backwards for exactly that reason):
#
#   ch   corr(u,vel)   corr(vel,d(bp)/dt)   dissipating on
#   a0     +0.919           +0.782              5.4 % of steps
#   a1     +0.624           +0.703             11.0 %
#   a2     -0.678           +0.722             22.6 %
#   a3     -0.623           +0.763             12.3 %
#
# Read those together: corr(vel, d(bp)/dt) of 0.70-0.78 says the FILTER is
# genuinely producing velocity, and corr(u, vel) of 0.62-0.92 says the LAW is
# genuinely feeding it back. Neither of those was the fault. Dissipating on
# 5-23 % of steps is not a phase error -- a 90 deg lag sits at 50 % -- it is an
# inverted sign, and it inverts because slope x gain was negative on all four.
#
# WHY THE OLD SIGN SURVIVED SO LONG: a0-a3's slopes had NEVER BEEN MEASURED. The
# comment on SLOPE_SIGN said so outright ("from the per-channel decay fits"), and
# `slopesign.py` was only ever run on a4-a7 until 2026-08-20. So the pairing of
# STEADY_GAIN's signs against the physical slopes was never checked on hardware,
# and the 2026-08-17 decay results cannot be used to defend it -- they were
# measured without knowing what the slopes were.
DAMP_SIGN = +SLOPE_SIGN

# ---------------------------------------------------------------------------
# the hybrid law: modal on A's coils, per-channel velocity feedback on the rest
# ---------------------------------------------------------------------------
# One switch, so the bench can kill the whole feature without editing logic.
HYBRID_DIAGONAL = True
# COIL AUTHORITY, measured static |rigid| force: coils 4/6/7 give 3.10-3.60
# counts/V against coils 0-3's 9.92-34.28, i.e. 1/3 to 1/10 and NOT the ~1/100
# this repo used to claim (A_DC_COUNTS_PER_V), and their output is almost entirely
# ONE TILT. "So they add authority where it is least needed" WAS WRITTEN HERE AND
# IS WITHDRAWN, on two counts: against a control it is B and C that modal damping
# wins on and A that it does not, so with the loop closed A is the mode still
# needing help (CLAUDE.md, per-mode residual); and which tilt is which mode was
# EXCHANGED on 2026-08-20 (PHI_GEOM), so any tilt-to-mode label from before that
# date is unsafe. What the hybrid is worth here is still unmeasured.
# WHY A NOISY CHANNEL IS STILL WORTH DRIVING. Dissipation is LINEAR in gain, noise
# injection is QUADRATIC: energy removed goes as g x (the part of the signal
# correlated with true velocity), energy added as g^2 x (uncorrelated power). For
# small enough g the useful term always wins, so a noisy channel is not
# disqualified -- its GAIN IS CAPPED. And the cap is not a fudge: the
# minimum-variance velocity estimate from a noisy measurement is the measurement
# scaled by its coherent fraction, so gain proportional to coherent fraction is
# Wiener-optimal.
#
# RE-MEASURED 2026-08-20 after the coils were re-seated, and it reshuffled
# completely. Multiple coherence of each channel against a0-a3 as a block over
# the 0.6-1.8 Hz mode band, `data/20260820_162534_status_sensors.csv` (105 013
# samples, 300 s, 350 Hz, 24 Hann segments of 8192, bias floor p/K = 4/24 =
# 0.167), raw -> bias-corrected, with the in-band rms each number is computed
# from:
#
#   ch   raw    corrected   in-band rms      was (2026-08-17)
#   a4  0.240     0.09        0.115 counts       0.46
#   a5  0.751     0.70        0.237 counts       0.00   (was a grounded pin)
#   a6  0.271     0.12        0.038 counts       0.26
#   a7  0.209     0.05        0.034 counts       0.50
#
# READ a4/a6/a7 CAREFULLY -- THIS IS NOT "THEY WENT BLIND". Their in-band rms is
# 0.03-0.12 counts, at or under ADC dither, and a coherence estimated on 0.03
# counts of signal is not a measurement of the wiring. a4 senses X and a6/a7
# sense Y, axes orthogonal to the damped Z/T1/T2, so small in-band amplitude is
# expected BY CONSTRUCTION. This is the same trap the repo has fallen into four
# times (CLAUDE.md Sec 6): a test that cannot tell a broken sensor from one
# pointed where the excitation is not. What the number is legitimately used for
# is the Wiener weight, and there it is right for the right reason -- a channel
# carrying 0.03 counts of in-band motion should be weighted at ~0 whatever the
# cause.
#
# a5 IS THE REAL CHANGE: 0.00 -> 0.70, the strongest of the four non-Phi
# channels, because it stopped being a hard-grounded pin. Its gain is STILL zero,
# and not for lack of signal -- see HYBRID_CHANNEL below.
# ONE 300 s ambient record. Re-measure it; these are not constants of the rig.
HYBRID_COHERENT = np.array([1.00, 1.00, 1.00, 1.00, 0.09, 0.70, 0.12, 0.05])
# a5 STAYS OFF, AND ITS 0.70 COHERENCE IS NOT THE REASON TO TURN IT ON.
# The whole dissipation argument for the per-channel term is COLOCATION: sensor j
# and coil j act on the same coordinate, so -Kp x velocity is a force opposing
# motion no matter what the plant does, and it is dissipative at any positive
# gain without needing a model. Coil 5 does not move a5 -- -0.39 +-0.93 counts/V,
# 0.4 sigma (`slopesign.py` 2026-08-20). It moves a1/a2/a3 by +9.3/-18.4/+11.1.
# So feeding a5's velocity into coil 5 is a NON-COLOCATED loop, and non-colocated
# rate feedback is not dissipative for free -- it needs A, which Sec 3 of
# CLAUDE.md says is not established in magnitude. a5 is a good SENSOR with no
# actuator of its own; the place to use it is a Phi row, not this term.
HYBRID_CHANNEL = [True, True, True, True, True, False, True, True]
# Magnitude is the shipped diagonal law's own reference |Kp| (KP_REF), not a new
# number. It is NOT normalised by SENSOR_GAIN_H: h is determined only for a0-a3,
# from the warp constraint over four coplanar sensors, and a4/a6/a7 are not in
# that plane and have no h. `slope_gain`'s clamp says the same thing from the
# other side -- their |slopes| are the SMALLEST on the rig (+5.06 / +10.37 /
# +8.07 counts/V), so any normalisation against them could only RAISE a gain,
# and raising is exactly what is forbidden.
#
# The sign is DAMP_SIGN, which since 2026-08-20 IS the measured slope, not its
# negation. `Pid.terms` sets p = gain x (-vel), so the demand about bias is
# -gain x vel and the power delivered goes as -slope x gain x vel^2: dissipation
# needs slope x gain > 0, the SAME sign. STEADY_GAIN carries that sign on a0-a3
# (+,+,-,- against slopes +,+,-,-) and the identical rule sets a4/a6/a7. It was
# -SLOPE_SIGN until 2026-08-20 and every driven channel pumped; see DAMP_SIGN for
# the measurement.
HYBRID_KP = 0.035
# Masked by HYBRID_CHANNEL as well, so a channel that is not a hybrid channel has
# an IDENTICALLY ZERO gain rather than a nonzero one that `_hyb_arm` happens to
# gate. a5 is why: its coherence is now 0.70, so the unmasked product would be
# -0.0245 sitting in the vector waiting for one gate change to become live.
HYBRID_GAIN = (HYBRID_KP * DAMP_SIGN * HYBRID_COHERENT
               * np.array(HYBRID_CHANNEL, float))
# A channel whose in-band motion is only fractionally the optic cannot have its
# amplitude ratio judged against a lock threshold -- one such channel vetoing the
# rig is CLAUDE.md item 1. So the hybrid channels are DRIVEN and WATCHED but are
# witnesses in the lock quorum, not voters. This 0.75 is a choice, not a
# measurement.
LOCK_COHERENT_MIN = 0.75
# NOT ESTABLISHED, and none of it is a formality: the fractions come from ONE
# 300 s ambient record; the vertical mode frequencies have never been measured;
# and whether the hybrid improves the measured DECAY RATE is untested -- `jerk.py`
# is that test, not `ratio`.
# The driven test (data/20260817_194726_status_coils.csv, on-resonance) puts
# a4/a6/a7 at |H| 0.02-0.39 V per V of coil drive and SNR 0-6, against a0-a3's
# 4-14 V/V and SNR 20-140. That bounds COIL AUTHORITY on the side and vertical
# DOFs, not sensing -- coils 0-3 barely push those directions, which is why the
# ambient coherence above is the right measurement and disagrees with it. a4 is
# the one to revisit first (SNR 5-6 at modes A and B, best of the three); what
# would settle its coil authority is SNR >= 10 driving its OWN coil.

# ---------------------------------------------------------------------------
# the sensor -> mode matrix, FROM GEOMETRY
# ---------------------------------------------------------------------------
# The plate being damped sits in the middle of the assembly with its normal along
# one axis. a0-a3 are four coplanar sensors on the far side reading that normal;
# a4/a5 read left-right; a6/a7 read vertical. Four coplanar sensors on one axis
# give exactly three rigid-body quantities plus one that no rigid body produces.
#
# CORNER ASSIGNMENT, determined from data and not assumed: a0 is diagonal to a3
# and a1 to a2. A rigid plate cannot warp, so the pairing minimising warp is the
# true one. RE-CHECKED 2026-08-20 after the coils were re-seated and IT DID NOT
# CHANGE (dof.py on data/20260820_162534_status_sensors.csv): the three candidate
# warp vectors come out 12.61 / 20.09 / 48.17 counts rms, and [+1,-1,-1,+1] is
# still the smallest. It won by 1.6x here against 2x on 2026-08-17.
#
# WARP ITSELF GREW: warp/loudest-rigid 0.139 -> 0.262. Nothing here explains that,
# and CLAUDE.md's standing warning applies -- per-sensor gain mismatch was tested
# as the cause on 2026-08-17 and FAILED both independent checks. Do not write down
# that warp is calibration. It is 12.61 counts rms that no gain can damp, because
# it is not a rigid-body DOF.
#
# MODE IDENTIFICATION -- AND MODES A AND C SWAPPED WHICH TILT THEY ARE.
# Measured 2026-08-20, 300 s, 0.0427 Hz bins (same file), share of each mode's
# total power:
#     0.7264 Hz -> T2  81.6 %      (was T1 90.3 % on 2026-08-17)
#     0.9828 Hz -> Z   73.7 %      (unchanged, was 88.2 %)
#     1.6665 Hz -> T1  89.9 %      (was T2 89.4 %)
# So the two tilt columns below are EXCHANGED relative to every version before
# this one. Getting this wrong does not attenuate the loop, it points the modal
# force at the wrong coordinate.
#
# ROWS a4-a7 ARE ZERO ON PURPOSE -- DO NOT "FIX" THEM. They read axes orthogonal
# to Z/T1/T2. Their re-measured coherence with the optic is 0.09 / 0.70 / 0.12 /
# 0.05 for a4/a5/a6/a7, and the hybrid diagonal path above already drives a4/a6/a7
# on their own colocated loops.
PHI_BASIS = "geometric"
PHI_GEOM = np.zeros((N, NMODE))
PHI_GEOM[:4, 0] = [+1.0, -1.0, +1.0, -1.0]      # T2 tilt  -> mode A 0.71519 Hz
PHI_GEOM[:4, 1] = [+1.0, +1.0, +1.0, +1.0]      # Z normal -> mode B 0.99231 Hz
PHI_GEOM[:4, 2] = [+1.0, +1.0, -1.0, -1.0]      # T1 tilt  -> mode C 1.65307 Hz

# Sensors that span the damped modes at all -- the only ones whose amplitude
# RATIO means anything, and so the only ones allowed to vote on a runaway.
# a4/a5 sense left-right and a6/a7 vertical, both orthogonal to Z/T1/T2.
PHI_GEOM_ROWS = np.abs(PHI_GEOM).sum(axis=1) > 0
# The leftover direction. No rigid-body motion produces it, so it is what the
# out-of-mode residual measures; it is real at 0.14-0.17 of rigid and its cause
# is NOT established (CLAUDE.md).
PHI_WARP = np.zeros(N)
PHI_WARP[:4] = [+1.0, -1.0, -1.0, +1.0]


# ---------------------------------------------------------------------------
# THE CONSTRAINT EVERY PER-CHANNEL GAIN VECTOR HAS TO SATISFY
# ---------------------------------------------------------------------------
# The diagonal law is u_j - bias = -g_j vel_j, and vel_j = sum_m Phi[j,m] qdot_m,
# so the modal force is f = -S qdot with
#
#     S = A diag(g) Phi,      A = pinv(Phi) @ M,   M[sensor, coil] = DC_MATRIX.T
#
# and the power delivered to the plate is qdot . f = -qdot^T sym(S) qdot. EVERY
# MODE DISSIPATES IF AND ONLY IF sym(S) IS POSITIVE DEFINITE. It is not enough
# that each channel has the right sign: that argument is COLOCATION, sensor j and
# coil j acting on one coordinate, and colocation does NOT hold on this rig
# (CLAUDE.md, and coil 1 gives a0 +37.2 and a1 +36.1 counts/V). One eigenvalue
# going negative is a mode the loop PUMPS, and nothing else in this file catches
# it -- the 2026-08-17 sign-of-A failure passed the selftest, the modal gate and
# the colocation check and still took the median channel ratio 1.5 -> 2.3 on
# hardware.
#
# TWO FORMS, and the selftest asserts BOTH, because they use different parts of
# the calibration and could disagree:
#   `counts`   Phi = PHI_GEOM as it stands, everything in counts. This is what
#              the loop literally computes today. It is only exact if the four
#              sensors share a counts-per-metre, and SENSOR_GAIN_H says they do
#              not, so read it as the conservative of the two.
#   `physical` the modal force taken from the h-calibrated response
#              pinv(Phi) @ diag(h) @ M, and the demand from diag(g/h) Phi. This
#              is the physically consistent statement and it is the one that
#              survives if the modal side moves to normalised sensor readings.
# Measured 2026-08-20 (see the gain block): flat gains give 1.9393 / 4.0678 /
# 10.0229 counts-form and 2.0343 / 4.1678 / 9.8134 physical; the shipped
# h-normalised vector gives 0.7593 / 2.4882 / 6.0345 and 1.2797 / 2.3935 /
# 4.7210. All eight positive.
#
# THE `physical` FORM IS AN OFFLINE CHECK AND IT IS NOT AN INVITATION TO FOLD h
# INTO Phi. That was tried on the bench on 2026-08-20 AND IT IS A NEGATIVE
# RESULT: cond 3.32 -> 9.41, chi2/dof 3.47 -> 844.70, and the run PUMPED. The
# chi2 blow-up is a units artifact -- KALMAN_R is a variance in RAW volts, so
# rescaling Phi's rows leaves the innovation covariance wrong -- which means
# doing it properly needs the readings normalised AND R rescaled by h^2 in the
# same change. That is an open item owned elsewhere. NOTHING IN THIS FILE APPLIES
# h TO Phi, TO KALMAN_R OR TO THE READINGS: h appears only in the per-channel
# DIAGONAL gains, where it is a scalar per channel and touches no covariance.


def dissipation_eig(g, form="counts", phi=None, dc=None, h=None):
    """Eigenvalues of sym(A diag(g) Phi). ALL MUST BE > 0 or some mode is pumped."""
    phi = PHI_GEOM[:4] if phi is None else np.asarray(phi, float)
    m = (DC_MATRIX_COUNTS_PER_V if dc is None else np.asarray(dc, float)).T
    h = np.abs(SENSOR_GAIN_H if h is None else np.asarray(h, float))
    g = np.asarray(g, float)[:phi.shape[0]]
    if form == "physical":
        a = np.linalg.pinv(phi) @ (np.diag(h) @ m)
        s = a @ np.diag(g / h) @ phi
    elif form == "counts":
        s = (np.linalg.pinv(phi) @ m) @ np.diag(g) @ phi
    else:
        raise ValueError("form must be 'counts' or 'physical', got %r" % (form,))
    return np.linalg.eigvalsh(0.5 * (s + s.T))
# A channel with no Phi row watches an axis the in-band is not defined on, so its
# in-band amplitude is small BY CONSTRUCTION and the cross-channel in-band floor
# grades it blind whatever its health. Derived from the geometry rather than
# listed, so giving a4 a Phi row later automatically puts it back under the
# ordinary test. See Health.floor: these are judged on their own broadband motion.
INBAND_EXEMPT = ~PHI_GEOM.any(axis=1)
# Below this per-mode |cos| between a measured Phi and the geometric one, the
# geometry or the corner assignment is wrong. Warned, never a gate.
PHI_COS_WARN = 0.90

# STATIC rigid-body force per volt, counts/V, one row per mode in F_MODE_HZ
# order, one column per coil. All eight coils.
#
# RE-MEASURED AND RE-PROJECTED 2026-08-20, AND THE ROW LABELS WERE WRONG BEFORE
# THAT. Two defects at once, the first the dangerous one:
#
# 1. THE PAIRING. That day's census swapped which tilt modes A and C are
#    (0.7264 Hz -> T2 at 81.6 % of its power, 1.6665 Hz -> T1 at 89.9 %, where
#    2026-08-17 had A -> T1 and C -> T2) and PHI_GEOM's tilt columns were swapped
#    to match. THIS CONSTANT WAS NOT, so rows 0 and 2 were paired with the wrong
#    Phi columns -- which does not attenuate the loop, it points the modal force
#    at the wrong coordinate. Live whenever OSEM_MODAL_PROVISIONAL=1, which is how
#    MIMO has been run. The labels below are read off `status.GEO_DOF`.
# 2. THE DATA. The old numbers (data/20260817_194726_status_coils.csv and
#    data/20260817_215348_status_slopesign.csv) predate the coils being re-seated,
#    and that re-seat changed a3's slope sign outright, +81.08 -> -98.79 counts/V.
#
# So RECOMPUTED, not re-ordered: today's DC matrix (`slopesign.py`,
# data/20260820_155638_status_slopesign.csv, all eight coils, 0.10 -> 0.40 V,
# 12 s dwell) projected onto today's geometric Phi as
# `A[m, j] = (Phi_m . d[:, j]) / (Phi_m . Phi_m)`, with d[:, j] coil j's measured
# counts/V on a0-a3. Phi's columns are orthogonal with norm^2 = 4, so that is
# exactly pinv(Phi) @ d and the divisor is not a normalisation choice.
A_DC_COUNTS_PER_V = np.array([
    [+70.33, -60.28, -75.38, +58.52, +1.08, -11.02, +10.05, +1.45],   # A  T2 tilt
    [+42.62, +14.68, -47.32, -21.23, +2.88,  -0.83,  +1.25, -0.95],   # B  Z normal
    [-10.57, +28.43, +46.73,  +3.07, -3.13,  +2.82,  +0.05, +1.70]])  # C  T1 tilt
# A DC measurement is the true A DIVIDED by omega_m^2: the static response of mode
# m to a force is f/(m omega_m^2). Multiply it back, per mode. Modal unit-norms
# each A row, so a positive per-mode factor cancels in today's allocation
# (asserted in the selftest); what it buys is that A_DC is a FORCE matrix, which
# is what a per-mode gain has to be derived against. Modal mass is unknown and
# common to all coils, so it stays absorbed in the gain.
A_DC = A_DC_COUNTS_PER_V * (2.0 * np.pi * F_MODE_HZ[:, None]) ** 2
# Coils 0-3 only: the 4/6/7 columns are recorded but not allocated to (see the
# hybrid section). DC-derived and NEVER validated closed-loop, so it is
# provisional and refused unless OSEM_MODAL_PROVISIONAL=1 -- which is also what
# keeps the suite on the diagonal law and reproducible.
A_DC_COILS = [0, 1, 2, 3]
A_DC_PROVENANCE = ("DC static, geometry basis, "
                   "data/20260820_155638_status_slopesign.csv; "
                   "never closed-loop validated")

# ---------------------------------------------------------------------------
# modal data and law
# ---------------------------------------------------------------------------
MODAL_PATH = os.path.join(HERE, "data", "modal.json")
MODAL_SCHEMA = "osem-modal-1"

# PER MODE, and that is the point of a modal law. Ships FLAT. Modal velocity rms
# came out A 0.64 / B 2.61 / C 3.15 over 28775 samples, and READING THAT AS "B and
# C are 4-5x worse damped" IS WITHDRAWN (CLAUDE.md Sec 6): the run behind it was in
# FAULT for 1085 of 1168 s, and a residual with no open-loop reference cannot tell
# a badly-damped mode from a hard-driven one. Against a control it is B and C that
# modal wins on. Retuning per mode needs the bench, not this number.
MODAL_KP = np.array([0.035, 0.035, 0.035])
MODAL_GAIN_SCALE = 0.5         # start at half the diagonal ceiling
# AN ABSOLUTE VOLT, deliberately, where MODAL_TOTAL_HEADROOM below is a fraction.
# NO MEASUREMENT HERE HAS EVER SHOWN THIS CAP BINDING: CLAUDE.md Sec 2 is the one
# recorded run where a coil clipped with the modal law live, and it is explicit
# that "the modal allocation stayed inside its own 0.20 V cap the whole time"
# while the TOTAL on coil 3 reached 0.4229 V, 169 % of the 0.250 V half-window.
# What saturated was the TOTAL, which is what MODAL_TOTAL_HEADROOM bounds. Raise
# this when a run reports the allocation actually pressing against it --
# `Modal.allocate` scales the whole vector uniformly, so a run that hits it says so.
MODAL_DEMAND_CAP_V = 0.20      # per-vector, uniform: direction preserved
MODAL_TOTAL_HEADROOM = 0.90    # of BIAS_SWING, on the TOTAL output not the modal
                               # part: the old cap let coil 3 reach 0.4229 V
                               # against a 0.250 V half-window. A FRACTION, so it
                               # tracks BIAS_SWING rather than being retyped.
MODAL_COND_MAX = 12.0
ALLOC_BALANCE = (12, 0.5, 1.05)     # iters, step, row-norm tolerance
MODAL_MIN_SENSORS = 2        # per mode; the estimate is a scalar LS, not an inverse
MODAL_MIN_COILS = 3          # rank(A_C) must reach the mode count
MODAL_ALLOW_PROVISIONAL_A = os.environ.get("OSEM_MODAL_PROVISIONAL", "0") not in ("", "0")
MODAL_MAX_AGE_S = 7 * 24 * 3600.0
MODAL_CHI2_LEVELS = (1.0, 2.0, 3.0, 5.0, 10.0)


def _modal(path, phi=PHI_GEOM, basis=PHI_BASIS, a=A_DC, a_coils=A_DC_COILS,
           allow=MODAL_ALLOW_PROVISIONAL_A):
    """The shipped source: Phi from GEOMETRY, A from `path` or the DC fallback.

    A measured Phi in `path` is only cross-checked against the geometry. A
    measured A in it must be stamped `basis` -- an A derived against some other
    Phi has an independent per-mode sign, and a wrong sign pumps.
    """
    return sl.Modal(path, N, F_MODE_HZ, MODAL_SCHEMA, MODAL_MIN_SENSORS,
                    MODAL_MIN_COILS, MODAL_COND_MAX, MODAL_MAX_AGE_S, allow,
                    ALLOC_BALANCE, MODAL_DEMAND_CAP_V,
                    phi=phi, basis=basis, phi_cos_warn=PHI_COS_WARN, a=a,
                    a_coils=a_coils, a_provisional=True,
                    a_provenance=A_DC_PROVENANCE)

# ---------------------------------------------------------------------------
# the stored floor
# ---------------------------------------------------------------------------
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0
BASELINE_PATH = os.path.join(HERE, "data", "baseline.json")
BASELINE_FILE_SCHEMA = 1
BASELINE_FILE_MAX_AGE_S = 1800.0
BASELINE_SANITY_RATIO = 3.0
MAX_BASELINE_REFUSALS = 3
BASELINE_WARMUP_S = 2.0

STATUS_PERIOD_S, CSV_FLUSH_EVERY_N = 5.0, 200

# ---------------------------------------------------------------------------
# bench aids
# ---------------------------------------------------------------------------
KICK_CUE_S = float(os.environ.get("OSEM_KICK_CUE", "0") or 0)
KICK_CUE_PHRASE = "jerk it"
_KICK_CUE_LEAD_S = 3.0          # first cue this long after DAMPING, not instantly


def kick_cue(phrase=KICK_CUE_PHRASE):
    """Speak the cue. Best effort; `say` is macOS-only."""
    try:
        subprocess.Popen(["say", phrase],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, ValueError):
        pass


GAIN_RAMP_HARD_CAP = 0.075      # RAISED 0.060 -> 0.075 on 2026-08-18, deliberately
                                # and with a measurement, which is what the
                                # parser demands. Two ramps that evening walked
                                # |Kp| from 0.030 to 0.050 in 0.005 and 0.002
                                # steps and NOTHING RAILED at any level -- 0.0%
                                # pinned samples throughout -- while peak demand
                                # never exceeded 0.089 V against the 0.25 V
                                # half-window, i.e. 36% of the actuator range.
                                # The documented "-0.040 rail onset"
                                # (analysis/kp040.md, one unreproduced run of
                                # 2026-07-15) DID NOT REPRODUCE.
                                # 0.050 is where the sweep STOPPED, not where
                                # anything broke, so the upper limit is
                                # UNEXPLORED rather than known. Extrapolating
                                # demand linearly the window would not bind
                                # until about 0.14. 0.075 is the next step, not
                                # a verdict; raise it again the same way if it
                                # too comes back clean. STILL ATTENDED ONLY.
GAIN_RAMP_SETTLE_FRAC = 0.40    # of each dwell, discarded: the slew limit is live
GAIN_RAMP_RISE_TRIPS = 2        # consecutive rises in `ratio` that end the ramp
GAIN_RAMP_RAIL_FRAC = 0.02      # of a level's samples pinned, on any driven channel


def _parse_gain_ramp(spec):
    """OSEM_GAIN_RAMP=start:stop:step:dwell_s -> (levels, dwell) or None."""
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
        raise ValueError("OSEM_GAIN_RAMP stop %.4f exceeds the %.3f hard cap. Raise "
                         "GAIN_RAMP_HARD_CAP deliberately, in a diff, with a reason."
                         % (stop, GAIN_RAMP_HARD_CAP))
    levels, v = [], start
    while v <= stop + 1e-12:
        levels.append(round(v, 6))
        v += step
    return levels, dwell


class GainRamp:
    """Walks |Kp| up and stops at the first turning point in `ratio`. ATTENDED ONLY."""

    def __init__(self, levels, dwell_s, nominal):
        self.levels, self.dwell = levels, dwell_s
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
        self.done, self.why = True, why
        self.i = max(0, self.i - 1 if back_to is None else back_to)
        self.ceiling = self.levels[self.i]
        return why

    def sample(self, t, ratio, demand_v, railed, faulted):
        if self.done:
            return None
        if faulted:
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
                back = max(0, self.i - GAIN_RAMP_RISE_TRIPS)
                why = self.stop(
                    f"ratio rose {GAIN_RAMP_RISE_TRIPS} levels running "
                    f"({self.history[-1 - GAIN_RAMP_RISE_TRIPS][1]:.3f} -> "
                    f"{mean_ratio:.3f}). More gain is making more motion: this is "
                    f"the phase-lag crossover, and the level below it is the "
                    f"measured ceiling", back_to=back)
                return (head + "\n" + why
                        + f"\n[gain-ramp] holding at {self.levels[self.i]:.4f}")
        else:
            self.rises = 0
        if self.i + 1 >= len(self.levels):
            return head + "\n" + self.stop(
                "reached the top of the requested range with no turning point. The "
                "ceiling is ABOVE this, not at it", back_to=self.i)
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
                "  driven, this velocity estimator, this wire rate.", ""]
        return "\n".join(out)


GAIN_RAMP = _parse_gain_ramp(os.environ.get("OSEM_GAIN_RAMP", "").strip())

# ---------------------------------------------------------------------------
# the commanded self-kick
# ---------------------------------------------------------------------------
# WHY IT EXISTS. Every decay rate in this repo was measured off a HAND kick: a
# person shoves the table when jerk.py asks. Three kicks per law, one session,
# and the two sets are comparable only because their peaks happened to overlap --
# modal 3.96-4.70 against diagonal 3.66-4.44 (CLAUDE.md). A commanded burst is
# the SAME excitation every time and costs nobody's attention, which is what makes
# an unattended sweep -- five configurations x three kicks -- possible at all.
#
# OFF BY DEFAULT, armed only by OSEM_SELF_KICK, the same pattern as
# OSEM_GAIN_RAMP and OSEM_MODAL_PROVISIONAL. Unset -> `SELF_KICK` is None ->
# `Controller.kick` is None -> not one line of this executes and a `make run` is
# bit-identical to what shipped. Asserted in the selftest.
#
#     OSEM_SELF_KICK=3                    3 bursts, everything else default
#     OSEM_SELF_KICK=3:2:10:0.05          3 bursts, mode C, 10 cycles, 0.05 V
#
# AMPLITUDE. 0.028 V peak on one coil. PROVENANCE: the 2026-08-17 19:47 coil pass
# drove coil 0 at 0.02796 V and coil 1 at 0.02743 V at mode A
# (data/20260817_194726_status_coils.csv, `phase` = drive, column `amp_v`), and
# status.py derived those rather than typing them -- amp = TARGET_SWING_COUNTS /
# (g * pi * f * DWELL_S) is the constant-amplitude drive that reaches 200 counts
# peak-to-peak on the loudest sensor after DWELL_S = 30 s on resonance.
# SO THIS IS NOT A HAND KICK AND MUST NOT BE READ AS ONE. On resonance a
# constant-amplitude drive builds LINEARLY, swing_pp(T) = 2 g amp pi f T, so at
# coil 1's measured mode-A authority of -60.28 counts/V (A_DC_COUNTS_PER_V,
# re-measured 2026-08-20) the default 0.028 V buys 7.6 counts p2p per second --
# about 64 counts over a 6-cycle 8.4 s burst, against the 868 counts p2p five
# sensors saw from ONE hand kick (CLAUDE.md REQUEST 1). Closed loop it is smaller
# again: the amplitude tops out at rate/decay, 54 counts p2p against the measured
# modal 0.1393 /s. THESE NUMBERS DOUBLED WHEN A_DC_COUNTS_PER_V WAS RECOMPUTED on
# 2026-08-20 and they are still an eighteenth of a hand kick.
# WHAT A BURST ACTUALLY REACHES ON THIS RIG IS NOT ESTABLISHED. `banner()` prints
# that arithmetic before the run and `summary()` prints the peak envelope each
# burst really produced, so the next session raises `cycles` or `amp_v` against a
# measurement instead of a guess.
SELF_KICK_AMP_V = 0.028
# HARD CAP, refused past it whatever is asked. status.py's own AMP_MAX -- the
# largest coil drive this repo has commanded on this rig in a measured pass, and
# 80 % of the 0.25 V half-window. Its provenance is that MEASURED pass and not a
# fraction of the window, so a wider window would not be a reason to move it.
# Raising it is CLAUDE.md 11 and is not free: the coil driver between the DAC and
# the coil is not in this repo and 0.25 V may encode a real current limit.
SELF_KICK_AMP_CAP_V = 0.20
# COIL 1, AND THE JUSTIFICATION FOR IT IS PARTLY STALE. It was picked as the
# strongest-but-one coil into mode A; on the 2026-08-20 A_DC_COUNTS_PER_V it is
# THIRD of the four (coil 2 -75.38, coil 0 +70.33, coil 1 -60.28, coil 3 +58.52),
# and |rigid| is now 82.9 / 68.3 / 100.5 / 62.3 counts/V, so "coil 0 is 3x weaker
# than 1/2/3" is gone too. Coil 1 remains defensible -- not the coil that saturated
# (0), not the one that clipped in 2026-08-18's modal run (3), not the one whose
# sensor is 3.37x a0's (2) -- and is left rather than re-argued from a desk:
# which coil to fire a bench instrument from is a bench question.
SELF_KICK_COIL = 1
# MODE A: where the 0.028 V provenance above was measured (F_MODE_HZ[0], not a
# literal -- it has moved twice on 2026-08-20 alone). Which mode is easiest to
# measure a decay on is NOT established here -- see MODAL_KP -- which is why the
# mode is a spec field rather than a fixed choice.
SELF_KICK_MODE = 0
# WHOLE CYCLES, so the burst starts and ends at exactly zero volts: no step at
# either edge, only a slope, and 2 pi f amp = 0.13 V/s against the actuator's
# 2.0 V/s slew limit.
SELF_KICK_CYCLES = 6
# THE FIRE GATE, and every number in it is one this file already uses.
#   * state DAMPING. Never CALIBRATING -- that window measures the floor at zero
#     gain and a commanded burst inside it would BE the floor -- and never FAULT.
#   * the loop's OWN running quiet must exist. `QuietLevel.level()` is None until
#     80 % of its 30 s window is closed-loop time, and reads 0.0 for a channel
#     without its own evidence, so those are excluded rather than treated as quiet.
#   * the envelope must be under SELF_KICK_QUIET_MULTIPLE of that level on every
#     channel allowed to vote on a runaway, held for SELF_KICK_QUIET_SUSTAIN_S.
# 1.4 is FAULT_CLEAR_RATIO's number and its argument -- the line this repo already
# uses for "the amplitude has come down", on the same envelope statistic -- and
# 5.0 s is FAULT_CLEAR_SUSTAIN_S, the hold that gate already requires. NO NEW
# THRESHOLD. (jerk.py reads its pre-kick level as a median over 5.0 s ending 1.0 s
# before the crossing, so this hold covers all but the last second of that window;
# that second is the burst's own first second, and `ratio` is a 1.0 s sliding RMS
# which has not risen yet.)
SELF_KICK_QUIET_MULTIPLE = FAULT_CLEAR_RATIO
SELF_KICK_QUIET_SUSTAIN_S = FAULT_CLEAR_SUSTAIN_S
# HOW LONG THE RUNAWAY TEST MAY STAY BLIND after the burst ends.
# FAULT_CLEAR_MAX_HOLD_S's number and its derivation: 1.7x the slowest re-quiet
# this rig has measured (17.4 s under the diagonal law), far under the 138 s
# intrinsic ringdown. Reaching it means the ringdown did NOT come back, and that
# stops the schedule -- do not keep exciting a rig that is not recovering.
SELF_KICK_BLIND_MAX_S = FAULT_CLEAR_MAX_HOLD_S
# Say out loud, once, if the gate has not opened after a full quiet window of
# DAMPING. A schedule that silently never fires is CLAUDE.md's dead-stream lesson.
SELF_KICK_WAIT_NOTE_S = QUIET_WINDOW_S
SELF_KICK_MAX_KICKS = 20        # parse ceiling; three per configuration is the use


def _parse_self_kick(spec):
    """OSEM_SELF_KICK=n[:mode:cycles:amp_v] -> constructor kwargs, or None."""
    if not spec:
        return None
    parts = spec.split(":")
    if len(parts) not in (1, 4):
        raise ValueError("OSEM_SELF_KICK wants n_kicks, or "
                         "n_kicks:mode:cycles:amp_v, got %r" % spec)
    n = int(parts[0])
    mode = int(parts[1]) if len(parts) == 4 else SELF_KICK_MODE
    cycles = int(parts[2]) if len(parts) == 4 else SELF_KICK_CYCLES
    amp = float(parts[3]) if len(parts) == 4 else SELF_KICK_AMP_V
    if not 1 <= n <= SELF_KICK_MAX_KICKS:
        raise ValueError("OSEM_SELF_KICK wants 1..%d bursts, got %d"
                         % (SELF_KICK_MAX_KICKS, n))
    if not 0 <= mode < NMODE:
        raise ValueError("OSEM_SELF_KICK mode must be 0..%d (%s Hz), got %d"
                         % (NMODE - 1, np.round(F_MODE_HZ, 5).tolist(), mode))
    if cycles < 1:
        raise ValueError("OSEM_SELF_KICK wants at least one WHOLE cycle so the "
                         "burst starts and ends at zero volts, got %r" % (parts[2],))
    if not 0.0 < amp <= SELF_KICK_AMP_CAP_V:
        raise ValueError("OSEM_SELF_KICK amp %r V is outside (0, %.3f]. That cap is "
                         "status.py's AMP_MAX, the largest coil drive ever commanded "
                         "on this rig in a measured pass; raise SELF_KICK_AMP_CAP_V "
                         "deliberately, in a diff, with a reason."
                         % (amp, SELF_KICK_AMP_CAP_V))
    return dict(n_kicks=n, mode=mode, cycles=cycles, amp_v=amp)


class SelfKick:
    """A commanded, repeatable excitation: one coil, one mode, N bursts.

    THE POINT IS REPEATABILITY, NOT SIZE. A hand kick is whatever the person at
    the table did; this is the same voltage into the same coil at the same
    frequency for the same whole number of cycles every time, so two
    configurations can be compared without waiting for their peaks to match. It is
    NOT a substitute for a hand kick's energy -- see SELF_KICK_AMP_V, where the
    arithmetic says 30 counts p2p against a hand kick's 868.

    THE RUNAWAY BREAKER IS BLIND TO THE WINDOW THIS OPENS, AND ONLY TO IT. The
    breaker exists to catch the LOOP adding energy on its own; a disturbance the
    controller commanded is by definition not that, and the controller is the one
    thing on the rig that knows it caused it. So from the first sample of the
    burst until the envelope is back under the quiet line -- or
    SELF_KICK_BLIND_MAX_S after the burst ends, whichever is first -- the breaker's
    `judge` mask is empty and `run_trip` cannot fire. EVERY OTHER INTERLOCK STAYS
    LIVE: the rail fault, the saturation trip, the dead-pin demotion, the in-band
    floor, the healthy quorum and the fault-clear gate are untouched, and a FAULT
    from any of them stops the schedule.

    WHAT THE SUPPRESSION COSTS, stated rather than hidden: an empty judge mask
    clears `Breaker`'s `excess_since` latch to inf every step, so a genuine runaway
    beginning inside the window needs a fresh RUNAWAY_SUSTAIN_S = 5.0 s of
    `high & growing` AFTER the re-arm before it trips. That is why the window is
    bounded at both ends, why reaching the bound stops the schedule instead of
    kicking again, and why the re-arm is announced with its time.

    NOT ESTABLISHED, and nothing here pretends otherwise: what amplitude a burst
    reaches on this rig, whether a commanded kick and a hand kick decay the same
    way, and whether 6 cycles at 0.028 V is enough for jerk.py to grade. The first
    two are bench questions; the third this class measures and reports.
    """

    def __init__(self, n_kicks, mode=SELF_KICK_MODE, cycles=SELF_KICK_CYCLES,
                 amp_v=SELF_KICK_AMP_V, coil=SELF_KICK_COIL, n=N):
        self.n_kicks, self.mode = int(n_kicks), int(mode)
        self.f = float(F_MODE_HZ[self.mode])
        self.cycles = int(cycles)
        self.burst_s = self.cycles / self.f
        self.amp, self.coil, self.n = float(amp_v), int(coil), int(n)
        self.u = np.zeros(self.n)
        self.phase = "waiting"          # waiting | burst | ringdown | done
        self.fired = 0
        self.t0 = self.t1 = self.quiet_since = self.t_damping = None
        self.quiet_v = None             # the quiet LEVEL this burst fired against
        self.peak = self.peak_v = 0.0   # x quiet, and volts, since the burst began
        self.history = []               # (n, t0, t1, t_rearm, how, peak, peak_v)
        self.stopped = None
        self._said_wait = self._said_coil = False

    # -- what the rest of the loop asks it ----------------------------------
    @property
    def suppressed(self):
        """True exactly while the runaway TEST is blind: bursting or ringing down."""
        return self.phase in ("burst", "ringdown")

    @property
    def code(self):
        """The `kick` CSV column: 0 nothing, 1 burst driving, 2 suppressed ringdown."""
        return {"burst": 1, "ringdown": 2}.get(self.phase, 0)

    def command(self, t):
        """Volts to ADD to the raw demand this control step. Zero unless bursting.

        A whole number of cycles of a sine started at zero, so the command leaves
        and returns to exactly 0.000 V with no step at either edge.
        """
        self.u[:] = 0.0
        if self.phase == "burst" and self.t0 is not None:
            s = t - self.t0
            if 0.0 <= s <= self.burst_s:
                self.u[self.coil] = self.amp * np.sin(2.0 * np.pi * self.f * s)
        return self.u

    # -- the schedule -------------------------------------------------------
    def on_engage(self, t):
        """A new engagement: the quiet window was reset with it, so re-gate."""
        self.quiet_since, self.t_damping = None, t
        if self.phase == "burst":
            self.t1 = t
            self.phase = "ringdown"

    def stop(self, why):
        if self.phase == "done":
            return None
        self.phase, self.stopped = "done", why
        return ("[self-kick] STOPPED after %d of %d burst(s): %s. No further "
                "excitation will be commanded this run." % (self.fired, self.n_kicks, why))

    def abort(self, t, why):
        """A FAULT. Kill the drive; stop the schedule only if a burst was in flight."""
        if self.phase == "done":
            return None
        if not self.suppressed:
            self.quiet_since = None     # merely waiting: re-gate after recovery
            return None
        self.t1 = t if self.t1 is None else self.t1
        return self.stop("the rig FAULTED with burst %d in flight (%s). A commanded "
                         "excitation is not a runaway, but a fault during one is a "
                         "fault, and nothing unattended should answer it by kicking "
                         "again" % (self.fired, why))

    def _track(self, env, quiet, m):
        if quiet is None or not m.any():
            return
        e, q = np.asarray(env, float)[m], np.asarray(quiet, float)[m]
        self.peak_v = max(self.peak_v, float(np.max(e)))
        self.peak = max(self.peak, float(np.max(e / q)))

    def _rearm(self, t, how):
        self.history.append((self.fired, self.t0, self.t1, t, how,
                             self.peak, self.peak_v))
        blind = t - self.t0
        self.phase, self.quiet_since = "waiting", None
        return ("[self-kick] runaway trip RE-ARMED at t=%.1fs -- %s. Blind for "
                "%.1fs (burst %.1fs + ringdown %.1fs). Peak envelope %.2fx the "
                "loop's own quiet level (%.4f V). The breaker's latch was held "
                "clear throughout, so it now needs a fresh %.1fs of sustained "
                "growth to trip."
                % (t, how, blind, self.burst_s, t - self.t1, self.peak,
                   self.peak_v, RUNAWAY_SUSTAIN_S))

    def update(self, t, env, quiet, vote, coil_live):
        """One DAMPING control step, after the breaker. Returns a line, or None."""
        if self.phase == "done":
            return None
        if self.t_damping is None:
            self.t_damping = t
        m = np.asarray(vote, bool).copy()
        if quiet is not None:
            m &= np.asarray(quiet, float) > 0.0        # 0.0 means "no evidence"
        under = bool(quiet is not None and m.any()
                     and (np.asarray(env, float)[m]
                          <= SELF_KICK_QUIET_MULTIPLE * np.asarray(quiet, float)[m]).all())

        if self.phase == "burst":
            self._track(env, quiet, m)
            if t - self.t0 < self.burst_s:
                return None
            self.t1, self.phase = t, "ringdown"
            return None
        if self.phase == "ringdown":
            self._track(env, quiet, m)
            if under:
                return self._rearm(t, "the envelope came back under the "
                                      "%.2fx quiet line" % SELF_KICK_QUIET_MULTIPLE)
            if t - self.t1 >= SELF_KICK_BLIND_MAX_S:
                msg = self._rearm(t, "the %.0fs CEILING, not the envelope"
                                     % SELF_KICK_BLIND_MAX_S)
                return msg + "\n" + self.stop(
                    "the ringdown from burst %d did not come back under %.2fx the "
                    "loop's own quiet level within %.0fs of the burst ending (peak "
                    "%.2fx, still %.2fx at the ceiling). The breaker is armed again "
                    "and the loop keeps damping; it is the EXCITATION that stops"
                    % (self.fired, SELF_KICK_QUIET_MULTIPLE, SELF_KICK_BLIND_MAX_S,
                       self.peak,
                       (float(np.max(np.asarray(env, float)[m]
                                     / np.asarray(quiet, float)[m]))
                        if (quiet is not None and m.any()) else float("nan"))))
            return None

        # waiting
        if self.fired >= self.n_kicks:
            return self.stop("all %d commanded bursts fired" % self.n_kicks)
        if not coil_live:
            if not self._said_coil:
                self._said_coil = True
                return ("[self-kick] holding: coil %d is not enabled, healthy and "
                        "un-demoted, so nothing would reach the optic. Waiting for "
                        "it rather than kicking a coil that is held at bias."
                        % self.coil)
            return None
        if not under:
            self.quiet_since = None
        elif self.quiet_since is None:
            self.quiet_since = t
        if self.quiet_since is None or t - self.quiet_since < SELF_KICK_QUIET_SUSTAIN_S:
            if (not self._said_wait and self.fired == 0
                    and t - self.t_damping >= SELF_KICK_WAIT_NOTE_S):
                self._said_wait = True
                return ("[self-kick] %.0fs of DAMPING and the gate has not opened: "
                        "the loop has not held under %.2fx its own quiet level for "
                        "%.0fs together%s. Still waiting -- nothing fires until it "
                        "does." % (t - self.t_damping, SELF_KICK_QUIET_MULTIPLE,
                                   SELF_KICK_QUIET_SUSTAIN_S,
                                   " (the running quiet is not measured yet)"
                                   if quiet is None else ""))
            return None
        self.fired += 1
        self.t0, self.t1 = t, None
        self.peak = self.peak_v = 0.0
        self.quiet_v = (float(np.max(np.asarray(quiet, float)[m]))
                        if (quiet is not None and m.any()) else float("nan"))
        self.phase = "burst"
        return ("[self-kick] BURST %d/%d at t=%.1fs -- coil %d, mode %s %.5f Hz, "
                "%.4f V peak, %d whole cycles (%.2fs).\n"
                "[self-kick] THE RUNAWAY TRIP IS SUPPRESSED from this sample until "
                "the envelope is back under %.2fx the loop's own quiet level "
                "(%.4f V) or %.0fs after the burst ends, whichever is first. The "
                "loop commanded this disturbance, so it is not the loop running "
                "away. Rails, saturation, dead-pin, the healthy quorum and the "
                "fault-clear gate all stay LIVE."
                % (self.fired, self.n_kicks, t, self.coil, "ABC"[self.mode], self.f,
                   self.amp, self.cycles, self.burst_s, SELF_KICK_QUIET_MULTIPLE,
                   self.quiet_v, SELF_KICK_BLIND_MAX_S))

    # -- reporting ----------------------------------------------------------
    def banner(self):
        g = float(A_DC_COUNTS_PER_V[self.mode, self.coil])
        rate = 2.0 * abs(g) * self.amp * np.pi * self.f     # counts p2p per second
        return [
            "[self-kick] ARMED: %d burst(s), coil %d, mode %s %.5f Hz, %.4f V peak, "
            "%d whole cycles = %.2fs each." % (self.n_kicks, self.coil,
                                               "ABC"[self.mode], self.f, self.amp,
                                               self.cycles, self.burst_s),
            "[self-kick] PREDICTED SIZE, from measured numbers and NOT from a bench "
            "run: coil %d gives %+.2f counts/V into mode %s (A_DC_COUNTS_PER_V), and "
            "a constant-amplitude resonant drive builds 2 g amp pi f = %.2f counts "
            "p2p per second, so this burst reaches about %.0f counts p2p OPEN LOOP. "
            "Closed loop it cannot exceed rate/decay = %.0f counts p2p at the "
            "measured modal 0.1393 /s. ONE HAND KICK MOVED 868 counts p2p."
            % (self.coil, g, "ABC"[self.mode], rate, rate * self.burst_s,
               rate / 0.1393),
            "[self-kick] THE RUNAWAY TRIP IS SUPPRESSED for the burst and its "
            "ringdown, and ONLY the runaway trip -- every other interlock stays "
            "live. Each suppression and each re-arm is printed with its time and "
            "marked in the CSV (`kick` 1 = burst, 2 = suppressed ringdown; "
            "`kick_n`, `kick_ch`, `kick_v`).",
        ]

    def summary(self):
        out = ["", "=" * 68, "  COMMANDED SELF-KICK", "=" * 68,
               "  coil %d, mode %s %.5f Hz, %.4f V peak, %d cycles (%.2fs)"
               % (self.coil, "ABC"[self.mode], self.f, self.amp, self.cycles,
                  self.burst_s),
               "   #    t0      burst   ringdown  blind   peak/quiet   peak V   re-armed by"]
        for (i, t0, t1, tr, how, pk, pv) in self.history:
            out.append("  %2d  %7.1f  %6.2f   %7.2f  %6.2f   %8.2fx  %8.4f   %s"
                       % (i, t0, t1 - t0, tr - t1, tr - t0, pk, pv, how))
        if not self.history:
            out.append("  no burst completed.")
        out += ["", "  fired %d of %d." % (self.fired, self.n_kicks)]
        if self.stopped:
            out.append("  stopped because: %s" % self.stopped)
        out += ["",
                "  `peak/quiet` is the envelope this burst produced against the "
                "loop's own",
                "  20th-percentile quiet level -- the first measurement anyone here "
                "has of",
                "  what a commanded burst is worth. A hand kick reached 3.7-4.7x "
                "its own",
                "  baseline. If these are far under that, raise `cycles` or `amp_v` "
                "-- the",
                "  spec is OSEM_SELF_KICK=n:mode:cycles:amp_v and the cap is %.2f V."
                % SELF_KICK_AMP_CAP_V, ""]
        return "\n".join(out)


SELF_KICK = _parse_self_kick(os.environ.get("OSEM_SELF_KICK", "").strip())

# ---------------------------------------------------------------------------
# the CSV
# ---------------------------------------------------------------------------
_LOG = (("bp", ".5f"), ("vel", ".5f"), ("out", ".4f"), ("gain", ".5f"),
        ("ratio", ".4f"), ("p", ".5f"), ("i", ".5f"), ("d", ".5f"))


def csv_cols():
    """The columns actually present, in order. Read by header name, never index."""
    cols = [("time_s", ".4f"), ("state", None), ("ctl", None), ("n_avg", None),
            ("modal", None), ("mmask", None), ("mrank", None), ("dmask", None)]
    cols += [(f"q{m}", ".6f") for m in range(NMODE)]
    cols += [(f"kmode{m}", ".5f") for m in range(NMODE)]
    cols += [(f"mres{m}", ".5f") for m in range(NMODE)]
    cols += [("chi2", ".5f"), ("ndof", None)]
    cols += [(f"qls{m}", ".6f") for m in range(NMODE)]
    # THE COMMANDED SELF-KICK, so an offline tool can find the window the
    # controller itself excited. Kept in the fixed prefix, ahead of the
    # per-channel block, so the per-channel columns stay contiguous.
    #   kick     0 nothing, 1 the burst is DRIVING, 2 the burst is over and the
    #            runaway trip is still suppressed while the ringdown comes down
    #   kick_n   1-based burst index, 0 before the first
    #   kick_ch  the coil being driven, -1 when the feature is off
    #   kick_v   volts added to that coil at the last control step, signed
    # Present on every run and identically zero when OSEM_SELF_KICK is unset.
    cols += [("kick", None), ("kick_n", None), ("kick_ch", None),
             ("kick_v", ".5f")]
    for i in range(N):
        cols += [(f"ch{i}_counts", None), (f"ch{i}_V", ".4f")]
        cols += [(f"ch{i}_{k}", f) for k, f in _LOG]
        cols += [(f"ch{i}_{k}", None) for k in ("rail", "locked", "healthy")]
    return cols


CSV_COLS = csv_cols()

# Per-channel view names, for the harness and the simulator.
_VIEW = dict(zip("enabled healthy bias steady_gain capture_gain ki kd bp_out vel out "
                 "active_gain last_ratio p_term i_term d_term rail_fault locked "
                 "saturated_flag baseline_rms".split(),
                 "enabled healthy bias steady capture ki kd bp vel out "
                 "gain ratio p i d rail locked sat baseline".split()))


class Channel:
    """A per-channel view onto the Controller's arrays."""

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


class Controller(sl.Loop):
    """CALIBRATING -> DAMPING (-> LOCKED) -> FAULT, on stdlib's building blocks."""

    DEAD_PIN_STD_COUNTS = DEAD_PIN_STD_COUNTS

    def __init__(self, dac, dac_channels=None, enable=None, steady=None,
                 capture=None, ki=None, kd=None, bias=None, baseline_file=None,
                 modal_path=None, self_kick=None):
        # modal_path defaults to None so the suite is reproducible; only main()
        # passes the real path.
        # self_kick: None reads the environment (and is None unless OSEM_SELF_KICK
        # is set), False forces it off, or pass a SelfKick. The suite constructs
        # its own; nothing is armed by default.
        sl.Loop.__init__(self)
        f = lambda v, d: np.array(d if v is None else v, dtype=float)
        self.enabled = np.array(ENABLE_CHANNEL if enable is None else enable, bool)
        self.steady, self.capture = f(steady, STEADY_GAIN), f(capture, CAPTURE_GAIN)
        self.ki, self.kd, self.bias = f(ki, KI_GAIN), f(kd, KD_GAIN), f(bias, BIAS)
        self.hybrid_ch = np.array(HYBRID_CHANNEL, bool)
        self.hybrid_gain = np.array(HYBRID_GAIN, float)
        self.coherent = np.array(HYBRID_COHERENT, float)

        self.guard = sl.SampleGuard(N, A_VCC, ADC_MAX_COUNTS)
        self.dec = sl.Decimator(CONTROL_HZ, N, MAINS_NULL)
        self.act = sl.Actuator(dac, DAC_CHANNELS if dac_channels is None
                               else dac_channels)
        self.health = sl.Health(N, RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS,
                                RAIL_SUSTAIN_S, RAIL_FRACTION, SAT_SUSTAIN_S,
                                SAT_FRACTION, DEAD_PIN_STD_COUNTS, REARM_SUSTAIN_S,
                                BASELINE_FLOOR_FRAC)
        self.brk = sl.Breaker(N, ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE,
                              RUNAWAY_SUSTAIN_S, RUNAWAY_TREND_LAG_S,
                              RUNAWAY_GROWTH_FRAC, SAT_DECAY_FRAC,
                              MIN_HEALTHY_CHANNELS)
        self.pid = sl.PID(N, self.ki, self.kd, D_SMOOTH_HZ, I_CLAMP_V,
                          TRACK_TC_S, MAX_SLEW_PER_S)
        self.pid.bias(self.bias)
        self.base = sl.Baseline(N, CALIB_SUBWINDOW_S, CALIB_SUBWINDOWS,
                                CALIB_MIN_SUBWINDOWS, CALIB_AGREE_N,
                                CALIB_AGREE_TOL, BASELINE_SANITY_RATIO,
                                MAX_BASELINE_REFUSALS, BASELINE_FLOOR_FRAC)
        self.inband = sl.InBand(F_MODE_HZ, N)
        self.rms_sch = sl.RMSBank(SCHEDULE_WINDOW_S, N)
        self.rms_lock = sl.RMSBank(LOCK_WINDOW_S, N)
        self.quiet = sl.QuietLevel(N, QUIET_WINDOW_S, QUIET_PERCENTILE,
                                   QUIET_KEEP_HZ, QUIET_MIN_FILL)
        self._quiet_said = False

        self.kf = sl.KalmanVelocity(KALMAN_K, N, F_MODE_HZ, CONTROL_PERIOD_S)
        self.modal = _modal(modal_path)
        self.nmode = self.modal.nm
        self.mkf = (sl.ModalKalman(self.modal.phi, self.modal.rowok, KALMAN_R,
                                   KALMAN_Q, KALMAN_Q_DC, F_MODE_HZ,
                                   CONTROL_PERIOD_S, N, T_AMP_S)
                    if self.modal.ok else None)
        self.chi2_log = {s: sl.Chi2Log(MODAL_CHI2_LEVELS)
                         for s in ("CALIBRATING", "DAMPING", "FAULT")}

        self.qdot = np.zeros(self.nmode)          # modal velocity, V/s
        self.qls = np.zeros(self.nmode)           # per-mode LS estimate, logged only
        self.mode_gain = np.zeros(self.nmode)     # scheduled K_m, >= 0
        self.modal_res = np.zeros(self.nmode)     # out-of-mode residual, logged only
        self.mode_seen = np.zeros(self.nmode, int)
        self.chi2, self.chi2_dof = float("nan"), 0
        self.modal_mask, self.modal_rank = 0, 0
        self.modal_u = np.zeros(N)
        self.modal_cols = np.zeros(N, bool)       # coils the allocator commanded
        self.diag_on = np.zeros(N, bool)          # coils the hybrid diagonal drove
        self.modal_on = False
        self.modal_scaled = 1.0
        self.modal_says = None

        for k in "bp vel gain ratio baseline counts_mean inband_v".split():
            setattr(self, k, np.zeros(N))
        self.counts_mean = np.full(N, MID_COUNTS)
        self.locked = np.zeros(N, bool)
        self.locked_since = np.full(N, np.inf)
        self.bias = np.array(self.bias, float)
        self.vmin, self.vmax = self.bias - BIAS_SWING, self.bias + BIAS_SWING

        self.channels = [Channel(self, i) for i in range(N)]
        self.state, self.calib_start, self.damping_start = "CALIBRATING", 0.0, None
        self.demote_count = self.reuse_count = self.floor_count = 0
        self.bad_samples = 0
        self.calib_took, self.calib_early = None, False
        self.baseline_t, self.baseline_refused = None, False
        self.baseline_saveable, self.warm_used = False, False
        self.damped_for, self.fault_railed, self.fault_amp = None, False, False
        self.fault_t, self._hold_said, self.t_now = None, False, 0.0
        self.ctl_dt, self.n_avg, self.stepped = CONTROL_PERIOD_S, 0, False
        self._pin_said = np.zeros(N, bool)
        self._pin_none_said = False
        self._floor_witness_said = False

        self.ramp = GainRamp(*GAIN_RAMP, nominal=self.steady) if GAIN_RAMP else None
        self.base_steady, self.base_capture = self.steady.copy(), self.capture.copy()

        # The commanded self-kick. None unless OSEM_SELF_KICK is set, and every
        # line it adds to `_damping` and `_actuate` is behind `is not None`, so an
        # unarmed run executes exactly what shipped. `kick_v` exists either way
        # because `csv_row` reads it at the wire rate.
        self.kick = ((SelfKick(**SELF_KICK) if SELF_KICK else None)
                     if self_kick is None else (self_kick or None))
        self.kick_v = np.zeros(N)

        self.trim_steps = np.zeros(N, int)
        self.trim_frozen = np.zeros(N, bool)
        self.trim_last = self.trim_step_t = self.trim_pending = self.trim_ref = None
        self.trim_budget_said = False
        self.trim_reach_said = np.zeros(N, bool)
        # WHERE THE BIAS IS GOING, as against where it IS. `_moved` sets this and
        # `_bias_ramp` walks `self.bias` towards it at BIAS_RAMP_PER_S. Nothing
        # else may write `self.bias`: a bias step is a force step (see BIAS_RAMP_S)
        # and every path that changes it -- a trim, a revert, a fault revert --
        # has to be adiabatic, not just the ones somebody remembered.
        self.bias_target = self.bias.copy()

        self.file_baseline = self.file_floor = None
        if baseline_file is not None:
            self.file_baseline = np.asarray(baseline_file[0], float)
            self.file_floor = np.asarray(baseline_file[1], bool)
        self.base.start(0.0)

    # -- shared arrays ------------------------------------------------------
    @property
    def ctl_steps(self):
        return self.dec.steps

    @property
    def wire_hz(self):
        return self.dec.wire_hz

    @staticmethod
    def _who(m):
        return ", ".join(f"ch{i}" for i in np.nonzero(m)[0])

    def _hyb_arm(self):
        """Channels the hybrid drives: enabled, outside the diagonal gains, coherent."""
        if not HYBRID_DIAGONAL:
            return np.zeros(N, bool)
        return (self.enabled & self.hybrid_ch & (self.hybrid_gain != 0.0)
                & (self.steady == 0.0) & (self.capture == 0.0))

    def _driven(self):
        """Channels something is commanding: a diagonal gain, the hybrid, or a modal coil.

        Deliberately NOT masked by `enabled`. It is the breaker's evidence mask,
        and a disabled channel still watches the same rigid body: measured, masking
        it costs a real detection -- the pumped-resonance scenario peaks at ratio
        2.58 against the 1.8 line and goes uncaught for 60 s, because the channel
        that crosses first is one with a smaller baseline that this run does not
        drive. `_hyb_arm` IS masked, which is the part that matters: the hybrid
        must never command a coil that is switched off.
        """
        d = (self.steady != 0.0) | (self.capture != 0.0) | self._hyb_arm()
        return d | (self.modal_cols if self.modal_on else False)

    def _quorum(self):
        """Who may VOTE on LOCKED: driven, and coherent enough to be judged."""
        return self._driven() & (self.coherent >= LOCK_COHERENT_MIN)

    def _amp_ref(self):
        """Per-channel volts the AMPLITUDE INTERLOCKS measure against.

        The LARGER of the calibration baseline and the running loop's own quiet
        level, and both halves are load-bearing:

          - the baseline is the plant with the gain OFF, so it is a FLOOR on the
            trip line that is physics rather than tuning: motion under the
            open-loop level cannot be the loop's fault, and the loop is doing
            better than nothing.
          - the running quiet is where this loop actually lives. Measured
            2026-08-18, the diagonal law sat at ratio 1.449 / 1.832 / 1.769
            against a RUNAWAY_MULTIPLE of 1.8 -- the loop's ordinary operating
            point WAS the trip line. Scaling off the running quiet moves that line
            to 1.8x where the loop sits, which is what the number was always meant
            to mean.

        Never TIGHTER than what shipped, which is the only line ever validated on
        hardware, and exactly what shipped until the estimator arms.

        RE-POINTED HERE: the runaway breaker's level test, and the fault-clear
        amplitude gate. NOT re-pointed, deliberately: the gain scheduler
        (`CAPTURE_*_FRAC` off `ratio`) and the lock detector (`LOCK_RMS_FACTOR`),
        which are statements about the loop's EFFECT on the undamped plate and
        only mean anything against the zero-gain window -- see `_damping`.
        """
        q = self.quiet.level()
        ref = self.baseline if q is None else np.maximum(self.baseline, q)
        # AND FLOORED ACROSS CHANNELS. A relative excursion only means something
        # against a reference that itself means something. Measured on the rig
        # 2026-08-18 02:31: ch4's baseline came out 0.0038 V against ch0's
        # 0.2141 V and a driven median near 0.28 V -- 56x smaller, because a4
        # senses an axis ORTHOGONAL to the three damped modes, so its in-band
        # level is small by construction rather than because it is quiet. Any
        # motion at all is then a huge ratio, and `!! ch4 runaway` faulted the
        # run twice within 25 s.
        #
        # This is NOT the same as dropping low-baseline channels from the
        # evidence set, which `_driven` explains costs a real detection: a
        # genuinely pumped resonance still crosses a floored reference, because
        # the floor is a fraction of what the loud channels are doing. It only
        # stops a channel whose reference is an artefact from voting on a ratio
        # against that artefact.
        drv = self._driven()
        if drv.any():
            med = float(np.median(ref[drv]))
            if med > 0:
                ref = np.maximum(ref, BREAKER_REF_FLOOR_FRAC * med)
        return ref

    # -- faults -------------------------------------------------------------
    def _fault(self, t, msg, railed=False, amp=False):
        self.say("!! " + msg)
        if self.kick is not None:
            # A commanded burst is not a runaway, but a rig that faults DURING one
            # is a rig that faulted, and nothing running unattended should answer
            # that by kicking again. Only aborts if a burst was actually in
            # flight; a fault while merely waiting just re-gates.
            m = self.kick.abort(t, msg)
            if m:
                self.say(m)
        if self.trim_pending is not None:
            i, was = self.trim_pending
            self.trim_pending = None
            self._moved(t, i, was)
            self.trim_frozen[i] = True
            self.say(f"[trim] ch{i} reverted to {was:.2f}V and frozen -- the rig "
                     f"faulted with that step in flight.")
        self.damped_for = (t - self.damping_start
                           if self.state == "DAMPING" and self.damping_start is not None
                           else None)
        self.fault_railed, self.fault_amp = bool(railed), bool(amp)
        self.fault_t, self._hold_said = t, False
        self.state, self.clear_since = "FAULT", None
        self.fault_count += 1

    def _pin_note(self, dead, witness):
        """One line per newly demoted pin, and one for the no-witness case."""
        if not witness:
            if not self._pin_none_said:
                self._pin_none_said = True
                self.say("[pin] EVERY enabled channel is railed and motionless (std "
                         f"{np.array2string(self.rail_std[self.enabled], precision=3)}"
                         f" counts over {RAIL_SUSTAIN_S:.1f}s). That is not a set of "
                         "dead pins -- it is a dead ADC, an unplugged loom, or the "
                         "optic hard against a stop. Nothing is demoted and the rail "
                         "fault stands.")
            return
        for i in np.where(dead & ~self._pin_said)[0]:
            self.say(f"[pin] ch{i} is railed and its raw counts have a std of "
                     f"{self.rail_std[i]:.3f} over {RAIL_SUSTAIN_S:.1f}s, under the "
                     f"{DEAD_PIN_STD_COUNTS:.2f}-count line (dead a6/a7 reach 0.853, "
                     f"the quietest railed-and-MOVING channel in evidence is 1.998) "
                     f"-- an UNWIRED PIN, not a plant excursion. Demoted, and it will "
                     f"not fault the rig. Fix the wiring or set "
                     f"ENABLE_CHANNEL[{i}] = False.")
            self._pin_said[i] = True

    # -- the step -----------------------------------------------------------
    def step(self, counts, volts, t, dt):
        counts, volts = np.asarray(counts, float), np.asarray(volts, float)
        self.stepped = False
        if not self.guard.in_range(counts):
            self.bad_samples += 1
            return self.state
        self.counts_mean += (counts - self.counts_mean) * min(1.0, dt / 2.0)
        self.health.rails(counts, t)                  # wire rate, not control rate
        got = self.dec.feed(counts, volts, t)
        if got is None:
            return self.state
        mc, mv, cdt, n = got
        self.ctl_dt, self.stepped, self.n_avg = cdt, True, n
        return self._control(mc, mv, t, cdt)

    def _control(self, counts, volts, t, dt):
        self.vel = self.kf.update(volts, dt,
                                  valid=(~self.rail if RAIL_BLANK else None))
        self.bp = self.kf.displacement()

        sense_ok = self.enabled & self.healthy & ~self.floor_bad
        if self.mkf is not None:
            # A channel with no determined Phi row carries no mode information, so
            # it would only add a degree of freedom to chi2 that its own DC state
            # absorbs.
            self.qdot = self.mkf.update(
                volts, sense_ok & ~self.rail & self.modal.rowok.any(axis=1), dt)
            self.chi2, self.chi2_dof = self.mkf.chi2_per_dof(), self.mkf.dof
            self.mode_seen = self.mkf.seen
            log = self.chi2_log.get(self.state)
            if log is not None:
                log.add(self.chi2)
        if self.modal.ok:
            # Diagnostics in every state: the residual must not freeze when the
            # actuators do.
            self.qls, self.modal_res, _ = self.modal.sense(
                self.kf.modal_velocity(), sense_ok)

        if self.state == "CALIBRATING":
            self._calibrating(t, volts)
        elif self.state == "DAMPING":
            self._damping(t, dt)
        elif self.state == "FAULT":
            self._faulted(t)
        self._actuate(t, dt)
        return self.state

    # -- CALIBRATING --------------------------------------------------------
    def _calibrating(self, t, volts):
        self.base.add(self.bp)
        self.inband.add(t, volts)
        dead, witness = self.health.dead_pin(self.enabled)
        self._pin_note(dead, witness)
        rail_now = self.rail & self.enabled & ~dead
        if rail_now.any():
            return self._fault(t, f"{self._who(rail_now)} railed during "
                                  f"calibration -- check alignment.", railed=True)
        if self.file_baseline is not None:
            return self._warm_from_file(t)
        if t - self.base.sub_start >= CALIB_SUBWINDOW_S:
            self.base.close_subwindow(t)
        ceiling = t - self.calib_start >= CALIBRATION_S
        early = (not ceiling) and self.base.stationary()
        if not (early or ceiling):
            return
        if ceiling and len(self.base.acc) > self.base.sub_n0:
            self.base.close_subwindow(t)              # flush the partial tail
        self.calib_took, self.calib_early = t - self.calib_start, early
        fresh, note = self.base.settle(early)
        self.baseline, self.baseline_refused = fresh, self.base.refused
        self.inband_v = self.inband.rms()
        if note:
            self.say(note)
        self.baseline_t, self.reuse_count = t, 0
        self._engage(t)
        self.say((f"[DAMPING] baseline KEPT after a refused {self.calib_took:.1f}s "
                  "calibration -- these are the numbers already in hand, not freshly "
                  "measured" if self.baseline_refused else
                  f"[DAMPING] baseline set in {self.calib_took:.1f}s "
                  + ("(sub-windows agreed -- stopped early, ceiling is "
                     f"{CALIBRATION_S:.0f}s)" if early
                     else f"(ran the full {CALIBRATION_S:.0f}s ceiling -- the floor "
                          "never settled, median of sub-windows used)"))
                 + ": "
                 + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                 + f". Gain schedules {CAPTURE_GAIN[0]:+.3f} -> {STEADY_GAIN[0]:+.3f}...")
        self._baseline_floor(t)
        self.baseline_saveable = True

    def _engage(self, t):
        self.state, self.damping_start = "DAMPING", t
        self.brk.arm(t)
        # A new engagement describes a new loop -- gains ramp from zero, coils may
        # differ, the law may have fallen back to diagonal. The old floor is not
        # this loop's floor, so the interlocks go back to the baseline until this
        # one has measured its own.
        self.quiet.reset()
        self._quiet_said = False
        self.locked_announced = False
        if self.kick is not None:
            self.kick.on_engage(t)

    def _warm_from_file(self, t):
        """Adopt a stored floor only if the first BASELINE_WARMUP_S agree with it."""
        if t - self.calib_start < BASELINE_WARMUP_S:
            return
        a = np.asarray(self.base.acc)
        a = a[int(len(a) * 0.4):]                     # drop the estimator's settle
        seen = np.sqrt((a ** 2).mean(0)) if len(a) >= 2 else np.zeros(N)
        loaded, floor = self.file_baseline, self.file_floor
        self.file_baseline = None                     # decided either way, once
        m = ~floor & (loaded > 0)
        ratio = np.where(m, np.maximum(seen, 1e-9) / np.maximum(loaded, 1e-9), 1.0)
        bad = m & ((ratio > BASELINE_SANITY_RATIO)
                   | (ratio < 1.0 / BASELINE_SANITY_RATIO))
        if not m.any() or bad.any():
            self.say("!! stored baseline REFUSED at the warm-up check -- "
                     + (f"{self._who(bad)} measured "
                        + " / ".join(f"{seen[i]:.4f}V against a stored "
                                     f"{loaded[i]:.4f}V ({ratio[i]:.2f}x)"
                                     for i in np.nonzero(bad)[0])
                        if bad.any() else
                        "the file demoted every channel, so there is nothing left to "
                        "check it against")
                     + f", outside {BASELINE_SANITY_RATIO:.1f}x. Something in the rig "
                     f"or the room has moved. Measuring a fresh floor the long way "
                     f"(up to {CALIBRATION_S:.0f}s).")
            return
        self.baseline = np.maximum(loaded, 1e-6)
        self.base.trusted = self.baseline.copy()
        self.baseline_t = t
        self.base.start(t)
        self.calib_took, self.calib_early = t - self.calib_start, True
        self.reuse_count = 0
        self.floor_bad = floor.copy()
        self.healthy[floor], self.gain[floor] = False, 0.0
        self.clear_t[floor] = np.inf
        self.warm_used = True
        self._engage(t)
        self.say(f"[DAMPING] engaged on the STORED baseline after {self.calib_took:.1f}s "
                 f"of warm-up (a full calibration is {CALIBRATION_S:.0f}s): "
                 + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                 + ". Measured over the warm-up: "
                 + ", ".join(f"ch{i}={seen[i]:.4f}V" for i in np.nonzero(m)[0])
                 # A PLAIN FACTOR, not a log. Measured 2026-08-18 this printed
                 # "0.72 in log", which is 2.05x and reads as small. It is not: the
                 # breaker, the fault-clear gate and the lock detector all scale off
                 # this number. NOT TIGHTENED, and not overridden by the warm-up
                 # either -- that is ~1.2 s of data, and the 20 s calibration length
                 # exists precisely because a short window cannot be trusted
                 # (worst-channel skew from one 6 V/s kick: 74.3 % at 5 s against
                 # 20.1 % at 20 s, CLAUDE.md). So the warm-up stays a check, not a
                 # replacement. Which was right on 2026-08-18 is NOT ESTABLISHED.
                 + f" -- worst disagreement {np.exp(np.abs(np.log(ratio[m])).max()):.2f}x, "
                   f"inside the {BASELINE_SANITY_RATIO:.1f}x window. Every amplitude "
                   f"threshold scales off this floor, so that factor is inherited by "
                   f"all of them."
                 + (f" {self._who(floor)} stays demoted -- the file says they were not "
                    f"measuring the optic." if floor.any() else "")
                 + f" Gain schedules {CAPTURE_GAIN[0]:+.3f} -> {STEADY_GAIN[0]:+.3f}, "
                   f"ramping from zero.")

    def _baseline_floor(self, t):
        """Demote any OSEM whose IN-BAND amplitude is under 10% of the median.

        In-band, not total: a4 measured 0.0030 V total against a 0.0953 V median
        and was demoted on 2026-08-17 while grading GOOD for in-band SNR the same
        evening -- the median was set by out-of-band interference on a0-a3, which
        leaks into the estimator's displacement at 0.16-0.44 (6.19 Hz).
        """
        x = self.inband_v if bool((self.inband_v > 0).any()) else self.baseline
        # Channels with no Phi row read an axis the in-band is not defined on, so
        # they are judged on their own broadband raw-count motion against the
        # measured dead-pin line instead. Without this the hybrid law never runs:
        # measured 2026-08-18, a4/a6/a7 were demoted at engage in BOTH runs
        # ("in-band 0.0027 / 0.0000 / 0.0026 / 0.0024V, under 10% of the 0.0834V
        # median") and their |gain| was exactly 0.0000 for 100 % of every DAMPING
        # sample. Their bias-corrected coherence with the optic was RE-MEASURED
        # 2026-08-20 after the coils were re-seated and it reshuffled completely:
        # a4/a5/a6/a7 are 0.09 / 0.70 / 0.12 / 0.05, not the 0.46/0.26/0.50 that
        # stood here for a4/a6/a7 (data/20260820_162534_status_sensors.csv, 105 013
        # samples, 300 s). Their in-band rms is 0.03-0.12 counts, at or under ADC
        # dither, so those four numbers are Wiener weights and not a verdict on
        # the wiring. See HYBRID_COHERENT.
        bad, med, witness = self.health.floor(x, self.enabled,
                                              exempt=INBAND_EXEMPT,
                                              motion=self.health.rail_std)
        self.floor_bad = bad
        if not witness and not self._floor_witness_said:
            self._floor_witness_said = True
            self.say("!! every enabled channel is under the in-band floor line. That "
                     "is not a set of blind sensors, it is a quiet room or a dead "
                     "loom. Nothing demoted.")
        if not bad.any():
            return
        self.healthy[bad], self.gain[bad], self.clear_t[bad] = False, 0.0, np.inf
        self.floor_count += int(bad.sum())
        band, still = bad & ~INBAND_EXEMPT, bad & INBAND_EXEMPT
        why = []
        if band.any():
            why.append(self._who(band) + " -- in-band "
                       + " / ".join(f"{x[i]:.4f}" for i in np.nonzero(band)[0])
                       + f"V, under {BASELINE_FLOOR_FRAC:.0%} of the {med:.4f}V "
                       f"median across the channels that carry the band")
        if still.any():
            why.append(self._who(still) + " -- raw counts std "
                       + " / ".join(f"{self.health.rail_std[i]:.3f}"
                                    for i in np.nonzero(still)[0])
                       + f", under the {DEAD_PIN_STD_COUNTS:.2f}-count dead-pin "
                       f"line. These read an axis with no Phi row, so they are "
                       f"judged on whether they MOVE, not on an in-band amplitude "
                       f"they cannot carry")
        self.say("!! demoted: " + "; ".join(why)
                 + ". That sensor is not measuring the optic, so a ratio against it "
                 f"is a ratio against nothing -- held at bias and out of the "
                 f"interlocks. "
                 f"{int((self.enabled & self.healthy).sum())}/"
                 f"{int(self.enabled.sum())} still damping.")

    # -- DAMPING ------------------------------------------------------------
    def _health(self, t):
        drop, starting, back = self.health.rearm(t, kf_reset=self.kf.reset)
        self.gain[drop] = 0.0                         # so the gain ramps from zero
        if back.any():
            self.brk.reset(back)
            self.rms_sch.reset(back)
            self.excess_since[back], self.ratio[back] = np.inf, 0.0
        return drop, back

    def _damping(self, t, dt):
        drop, back = self._health(t)
        live, n_conf = self.enabled & self.healthy, int(self.enabled.sum())
        n = int(live.sum())
        for i in np.nonzero(drop)[0]:
            self.demote_count += 1
            self.say(f"!! ch{i} railed -- held at bias, {n}/{n_conf} still damping.")
        for i in np.nonzero(back)[0]:
            self.say(f"[ch{i} back] rail clear {REARM_SUSTAIN_S:.0f}s -- re-engaging "
                     f"on its pre-event baseline {self.baseline[i]:.4f}V, "
                     f"{n}/{n_conf} damping.")
        if n_conf and n < MIN_HEALTHY_CHANNELS:
            return self._fault(t, f"quorum lost -- {n}/{n_conf} healthy, need "
                                  f"{MIN_HEALTHY_CHANNELS}. Freezing everything.",
                               railed=True)

        if self.ramp is not None:
            driven = live & ((self.base_steady != 0.0) | (self.base_capture != 0.0))
            msg = self.ramp.sample(
                t, float(np.nanmean(self.ratio[driven])) if driven.any() else float("nan"),
                float(np.max(self.demand[driven])) if driven.any() else 0.0,
                float(self.rail[driven].mean()) if driven.any() else 0.0,
                self.state != "DAMPING")
            if msg:
                self.say(msg)
            s = self.ramp.scale()
            self.steady, self.capture = self.base_steady * s, self.base_capture * s

        self.ratio = np.where(live, self.rms_sch.update(t, self.bp, live)
                              / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC)
                       / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        # The hybrid channels take their gain from the same schedule and the same
        # slew limit, at HYBRID_KP scaled by their measured coherent fraction.
        want = np.where(self._hyb_arm(), self.hybrid_gain, want)
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)
        # Per mode, slewed like the diagonal gains so nothing steps.
        self.mode_gain = (self.mode_gain + np.clip(
            MODAL_KP * MODAL_GAIN_SCALE - self.mode_gain,
            -GAIN_SLEW_PER_S * dt, GAIN_SLEW_PER_S * dt)
            if self.modal.ok else np.zeros(self.nmode))

        # The breaker watches everything the loop DRIVES, hybrid channels included:
        # a wrong sign there shows up as their own amplitude growing. A spurious
        # trip on their incoherent motion now costs one FAULT_CLEAR_SUSTAIN_S, not a
        # deadlock.
        ref = self._amp_ref()
        # WHO MAY VOTE ON A RUNAWAY: the Phi rows, and this is a HARDWARE verdict
        # over a simulator one. `_driven` keeps every driven channel as EVIDENCE
        # because a small baseline crosses first, which is what catches a pumped
        # resonance in the simulator (peak ch2 ratio 2.58 against the 1.8 line,
        # 0 faults if the evidence set is narrowed) -- so `judge` is deliberately
        # not narrowed to the lock quorum either. On the rig the same property
        # does the opposite: a4's reference came out 56x under ch0's because it
        # senses an orthogonal axis (`_amp_ref` has the measurement), and
        # `!! ch4 runaway` then ended three consecutive runs within 40 s of
        # engaging, every time on a deliberate hand kick. Flooring the reference
        # was tried first and is NOT enough -- the floor that keeps a4 quiet is far
        # below what a kick moves it. A channel may only vote on a RATIO if the
        # denominator means something, and for a Phi-less axis it does not. They
        # stay driven, watched, railed-checked and demotable; they simply do not
        # get to fault the rig.
        # FROM THE GEOMETRY, not from whether a file loaded. `PHI_GEOM` is always
        # available; `modal.rowok` needs data/modal.json, so keying on it left the
        # DIAGONAL configuration with no mask at all -- measured 2026-08-18 02:58,
        # the first diagonal run after this fix faulted on `!! ch4 runaway` within
        # 40 s for exactly that reason. Which axes a mode can reach is a property
        # of the rig, not of which control law happens to be running.
        vote = self._driven() & PHI_GEOM_ROWS
        if not vote.any():
            vote = self._driven()
        # THE ONE INTERLOCK A COMMANDED BURST TURNS OFF, AND IT IS THE ONLY ONE.
        # `judge` names who may TRIP on amplitude; emptying it makes `run_trip`
        # unreachable and holds `Breaker`'s `excess_since` latch clear, while
        # `drives` still carries the whole evidence set, so the envelope and the
        # SATURATION trip are computed exactly as before. Scope and cost are in
        # SelfKick's docstring; every other interlock is untouched.
        judge = vote
        if self.kick is not None and self.kick.suppressed:
            judge = np.zeros(N, bool)
        got = self.brk.sample(t, self.bp, self.healthy, vote, ref, self.sat,
                              judge=judge)
        # Fed ONLY here, so the window holds closed-loop time and a FAULT freezes
        # it rather than filling it with the open-loop plant. A commanded burst is
        # frozen out for the same reason: this number is where the LOOP lives, and
        # a disturbance the controller ordered is not that.
        if self.kick is None or not self.kick.suppressed:
            self.quiet.add(t, got["env"], live)
        if self.quiet.ready() and not self._quiet_said:
            self._quiet_said = True
            q = self.quiet.level()
            m = live & (self.baseline > 0)
            self.say("[quiet] the running loop's own floor is measured: "
                     + ", ".join("ch%d=%.4fV (%.2fx baseline)"
                                 % (i, q[i], q[i] / self.baseline[i])
                                 for i in np.nonzero(m)[0])
                     + f" -- {QUIET_PERCENTILE:.0f}th percentile over "
                       f"{QUIET_WINDOW_S:.0f}s of DAMPING. The runaway breaker and "
                       f"the fault-clear gate now scale off max(baseline, this); "
                       f"the gain schedule and LOCKED still scale off the "
                       f"zero-gain baseline, which is what they mean.")
        if got["run_trip"].any():
            return self._fault(t, f"{self._who(got['run_trip'])} runaway -- freezing "
                                  f"all channels, entering FAULT.", amp=True)
        if got["sat_trip"].any():
            return self._fault(t, f"{self._who(got['sat_trip'])} pinned against the "
                                  f"rail, still demanding more, and not winning -- "
                                  f"freezing all channels, entering FAULT.")

        # The self-kick schedule, after both trips so a fault this step ends it
        # rather than being kicked into. It fires only from DAMPING, only once the
        # loop has held under its own running quiet level, and it is the thing
        # that sets `suppressed` for the NEXT step -- so no sample is ever driven
        # with the runaway trip still armed.
        if self.kick is not None:
            msg = self.kick.update(t, got["env"], self.quiet.level(), vote,
                                   bool(live[self.kick.coil]))
            if msg:
                self.say(msg)

        # AGAINST THE ZERO-GAIN BASELINE, ON PURPOSE, and it is the one place in
        # this file where that is the right reference. LOCKED's definition is a
        # statement about the loop's EFFECT -- motion under LOCK_RMS_FACTOR of what
        # it was with the gain off -- and re-pointing it at the running quiet would
        # make it self-referential, since that quiet IS a low percentile of the
        # loop's own envelope. The measurement would become a tautology.
        # NOT A FALSE POSITIVE TODAY, measured: under modal the driven channels
        # read 1.98/1.52/1.07/1.30 at t=5 s and 0.26/0.12/0.37/0.33 at t=15 s, and
        # LOCKED was announced at 20.7 s against a settled floor of 0.117 -- the
        # 0.35 line sat 3x above the floor (data/20260818_002207_jerk_eta.log).
        # NOT ESTABLISHED: whether 0.35 stays a real bar if the loop improves. It is
        # a fixed fraction of a fixed reference, so a loop holding 0.01 clears it
        # instantly and the lock TIME stops meaning anything. So the LOCKED line
        # reports its margin against the running quiet -- logged, acted on by
        # nothing, as this repo does with a threshold it has not measured.
        quiet = live & (self.rms_lock.update(t, self.bp, live)
                        < self.baseline * LOCK_RMS_FACTOR)
        self.locked_since = sl.Health.hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= LOCK_SUSTAIN_S)
        self._trim(t)

        # The lock quorum counts only channels something DRIVES and whose motion is
        # coherently the optic. An undriven channel cannot meet the lock ratio, and
        # one of those vetoed the rig for three versions: LOCKED was never printed.
        claim = live & self._quorum()
        if claim.any() and self.locked[claim].all() and not self.locked_announced:
            self.lock_time = t - self.damping_start
            q = self.quiet.level()
            self.say(f"*** LOCKED -- {self.lock_time:.1f}s after gain was applied "
                     f"(t={t:.1f}s total), {int(claim.sum())}/{n_conf} channels driven "
                     f"and quiet"
                     + ("" if n == n_conf else f" -- DEGRADED, {n}/{n_conf} healthy")
                     + " ***"
                     + ("" if q is None else
                        " -- worst driven ratio %.3f against the %.2f line, and "
                        "%.2f of this loop's own measured floor. A ratio far under "
                        "the line means the line is loose, not that the lock is "
                        "good; nothing acts on this."
                        % (float(np.max(self.ratio[claim])), LOCK_RMS_FACTOR,
                           float(np.max(self.ratio[claim]))
                           / max(float(np.max(q[claim] / np.maximum(
                               self.baseline[claim], 1e-12))), 1e-12))))
            self.locked_announced = True

    # -- FAULT --------------------------------------------------------------
    def _faulted(self, t):
        """Frozen actuators, LIVE measurement.

        Freezing the actuators is the safety action; freezing the measurement was
        the bug. Measured 2026-08-17: ratio 3.59 / 3.26 identical across every 5 s
        print for 1085 s with gains at zero, because the clear gate read
        `rail.any()` unmasked and ch5 -- a DISABLED, disconnected pin sitting at
        0.0 counts -- was permanently railed. Nothing could ever clear it.
        """
        self.gain[:] = 0.0
        self.t_now = t
        self._observe(t)
        dead, witness = self.health.dead_pin(self.enabled)
        self._pin_note(dead, witness)
        watch = self.enabled & ~self.floor_bad
        stuck = self.rail & watch & ~dead
        blocked = bool(stuck.any() or (self.sat & watch).any() or self._still_hot())
        if self.clear_gate(t, blocked, FAULT_CLEAR_SUSTAIN_S):
            self._recover(t)

    def _observe(self, t):
        """Keep the amplitude statistic and the envelope running while frozen."""
        m = self.enabled & ~self.floor_bad & (self.baseline > 0)
        if not m.any():
            return
        self.ratio = np.where(m, self.rms_sch.update(t, self.bp, m)
                              / np.maximum(self.baseline, 1e-12), self.ratio)
        self.brk.sample(t, self.bp, m, np.zeros(N, bool), self._amp_ref(),
                        np.zeros(N, bool))

    def _still_hot(self):
        """An amplitude fault re-engages on amplitude, not on a fixed 5 s."""
        if not self.fault_amp:
            return False
        if (self.fault_t is not None
                and self.t_now - self.fault_t >= FAULT_CLEAR_MAX_HOLD_S):
            if not self._hold_said:
                self._hold_said = True
                self.say(f"!! held out of DAMPING for "
                         f"{FAULT_CLEAR_MAX_HOLD_S:.0f}s by amplitude alone. The "
                         f"disturbance is not going away, so the amplitude gate is "
                         f"dropped and the loop re-engages; if it re-trips that "
                         f"costs {FAULT_CLEAR_SUSTAIN_S:.0f}s, which is cheaper than "
                         f"a breaker that never re-closes.")
            return False
        # Judged on the quorum: a channel whose in-band motion is only fractionally
        # the optic has a `ratio` that means little, and gating recovery on it would
        # reproduce the veto this fix exists to remove.
        m = self.enabled & ~self.floor_bad & self._quorum() & (self.baseline > 0)
        # Against the running loop's own floor where one is known, not against the
        # zero-gain window alone. Measured 2026-08-18: through 95 s of FAULT the
        # channels read ch0 1.4-3.8, ch1 1.5-2.5, ch2 1.5-2.6, ch3 2.1-3.2, and ch3
        # essentially never went under the absolute 1.4, so the gate never opened.
        # `maximum(1.0, ...)` keeps this from ever asking for LESS than the
        # open-loop level: with the gain frozen the plate returns to exactly that,
        # so a line under 1.0 could not be met at all. Under modal the running
        # quiet is 0.117 and this is the identity -- which is the honest result,
        # and it is why the ceiling above, not this gate, is the fix that matters.
        line = FAULT_CLEAR_RATIO * np.maximum(
            1.0, self._amp_ref() / np.maximum(self.baseline, 1e-12))
        return bool(m.any() and (self.ratio[m] > line[m]).any())

    def _why_reuse(self, t):
        if not bool(self.baseline.all()):
            return False, "nothing measured yet"
        if self.fault_railed:
            return False, ("the fault was a RAIL -- a floor measured through a "
                           "suspect sensor is suspect with it")
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
        return True, ("the previous engagement was healthy, so this baseline is known "
                      "good and the fault was a transient -- measuring a new floor now "
                      "would measure the ringdown")

    def _recover(self, t):
        if self.damped_for is not None and self.damped_for >= FAST_REFAULT_S:
            self.reuse_count = 0        # the loop itself validated this baseline
        reuse, why = self._why_reuse(t)
        keep, keep_t = self.baseline.copy(), self.baseline_t
        keep_floor = self.floor_bad.copy()
        self.base.start(t)
        self.inband.reset()
        for k in "baseline gain ratio sat_sum clip_excess p i d prev_vel".split():
            getattr(self, k)[:] = 0.0
        self.health.sat_hist.clear()
        self.primed[:] = self.sat[:] = self.locked[:] = False
        self.locked_since[:] = self.excess_since[:] = self.clear_t[:] = np.inf
        self.healthy[:] = True          # a recovery forgets every demotion...
        self.floor_bad[:] = False
        self.pid.reset()
        self.kf.reset()
        if self.mkf is not None:
            self.mkf.reset()
        self.brk.reset()
        self.rms_sch.reset()
        self.rms_lock.reset()
        if reuse:
            self.reuse_count += 1
            self.baseline, self.baseline_t = keep, keep_t
            self.floor_bad = keep_floor          # ...except this one
            self.healthy[keep_floor] = False
            self._engage(t)
            self.say(f"[recovered] re-engaging on the baseline already in hand -- "
                     f"{why} ({self.reuse_count}/{MAX_BASELINE_REUSE} before a forced "
                     f"re-calibration). Gain ramps from zero."
                     + (f" {self._who(keep_floor)} stays demoted -- same baseline, "
                        f"same verdict." if keep_floor.any() else ""))
        else:
            self.reuse_count = 0
            self.baseline_t, self.damping_start = None, None
            self.calib_start = t
            self.state = "CALIBRATING"
            self.say(f"[recovered] all channels clear for {FAULT_CLEAR_SUSTAIN_S:.0f}s "
                     f"-- re-calibrating ({why}) for up to {CALIBRATION_S:.0f}s.")
        self.fault_railed = self.fault_amp = self._hold_said = False
        self.fault_t = self.clear_since = self.lock_time = None

    # -- bias trim ----------------------------------------------------------
    def _moved(self, t, i, new):
        """AIM the bias at `new`. `_bias_ramp` is what actually gets it there.

        This used to assign `self.bias[i]` directly, which put a 0.25 V force
        step into an undamped plant and threw the modal residual 7-8x for about
        10 s every time it fired (BIAS_RAMP_S has the measurement). Setting a
        TARGET instead means every caller ramps, including the two revert paths,
        without any of them having to know that.
        """
        self.bias_target[i] = new
        self.trim_step_t = t
        self.brk.hist.clear()
        self.excess_since[:] = np.inf
        self.health.sat_hist.clear()
        self.sat_sum[:] = 0.0

    def _bias_ramp(self, dt):
        """Walk the commanded bias towards its target at BIAS_RAMP_PER_S. Adiabatic.

        Runs in EVERY state, from `_actuate`, so a target set just before a fault
        still arrives rather than freezing half way. The window moves with the
        live bias, not with the target, so saturation accounting stays honest
        while the ramp is in flight.
        """
        step = BIAS_RAMP_PER_S * max(dt, 0.0)
        d = self.bias_target - self.bias
        moving = np.abs(d) > 1e-12
        if not moving.any():
            return
        # IN PLACE, deliberately. `self.bias`, `self.vmin` and `self.vmax` are
        # handed out by reference (Channel, the CSV row, the console line), so
        # rebinding the name here would leave every one of those holding the
        # array from before the first trim.
        self.bias += np.clip(d, -step, step)
        self.vmin[:] = self.bias - BIAS_SWING
        self.vmax[:] = self.bias + BIAS_SWING

    def _bias_settled(self):
        """True when nothing is ramping. A trim never starts on top of a trim."""
        return bool(np.abs(self.bias_target - self.bias).max() <= 1e-9)

    def _trim(self, t):
        """One coarse bias step per TRIM_PERIOD_S, judged on total offset, quiet only."""
        if self.trim_last is None:
            self.trim_last = t
            return
        if t - self.trim_last < TRIM_PERIOD_S:
            return
        self.trim_last = t
        total = float(np.abs(self.counts_mean - MID_COUNTS).sum())
        watch = self.enabled & self.healthy
        claim = watch & self._quorum()
        quiet = bool(claim.any() and self.locked[claim].all())

        if self.trim_pending is not None:          # judge last period's step
            i, was = self.trim_pending
            self.trim_pending = None
            if not quiet:
                self._moved(t, i, was)
                self.trim_steps[i] -= 1
                worst = float(np.max(self.ratio[claim])) if claim.any() else float("nan")
                self.say(f"[trim] ch{i} reverted to {was:.2f}V -- not locked at judging "
                         f"time (worst driven ratio {worst:.2f}), so the offsets it "
                         f"would be judged on describe the motion, not the rest point.")
                return
            if total >= self.trim_ref:
                self._moved(t, i, was)
                self.trim_frozen[i] = True
                self.say(f"[trim] ch{i} reverted to {was:.2f}V and frozen -- total "
                         f"offset {self.trim_ref:.0f} -> {total:.0f} counts.")
                return
            self.say(f"[trim] ch{i} kept at {self.bias[i]:.2f}V -- total offset "
                     f"{self.trim_ref:.0f} -> {total:.0f} counts.")

        err = self.counts_mean - MID_COUNTS
        if not quiet:
            return              # never START a step on a moving optic either
        if not self._bias_settled():
            return              # and never on top of a ramp that is still running
        step_all = -np.sign(err) * SLOPE_SIGN * BIAS_QUANTUM
        nxt = np.clip(self.bias + step_all, BIAS_MIN, BIAS_MAX)
        movable = np.abs(nxt - self.bias) > 1e-9
        room = TRIM_MAX_TOTAL_EXCURSION_V - (np.abs(self.bias - BIAS).sum()
                                             - np.abs(self.bias - BIAS))
        affordable = np.abs(nxt - BIAS) <= room
        # CAN IT EVEN GET THERE? The furthest this channel could still be taken,
        # with the step budget, the excursion budget and the BIAS_MIN/BIAS_MAX
        # clamps all applied, converted to counts through its MEASURED slope. A
        # step costs real motion (BIAS_RAMP_S), so one that cannot close the
        # offset is a disturbance bought for nothing.
        left = np.maximum(TRIM_MAX_STEPS - self.trim_steps, 0)
        far = np.clip(self.bias + np.sign(step_all) * left * BIAS_QUANTUM,
                      BIAS_MIN, BIAS_MAX)
        far = np.clip(far, BIAS - room, BIAS + room)
        reach = np.abs(far - self.bias) * np.abs(SENSOR_SLOPE_COUNTS_PER_V)
        reachable = reach >= TRIM_MIN_REACHABLE_FRAC * np.abs(err)
        want = (watch & ~self.trim_frozen & movable & affordable
                & (self.trim_steps < TRIM_MAX_STEPS)
                & (np.abs(err) > TRIM_DEADBAND_COUNTS))
        for i in np.nonzero(want & ~reachable & ~self.trim_reach_said)[0]:
            self.trim_reach_said[i] = True
            self.say(f"[trim] ch{i} NOT TRIMMED and it is not a fault: it rests "
                     f"{err[i]:+.0f} counts off mid-scale and everything the trim "
                     f"has left -- {np.abs(far[i] - self.bias[i]):.2f}V at its "
                     f"measured {SENSOR_SLOPE_COUNTS_PER_V[i]:+.1f} counts/V -- is "
                     f"worth {reach[i]:.0f} counts, {100 * reach[i] / abs(err[i]):.0f}% "
                     f"of the offset against a {100 * TRIM_MIN_REACHABLE_FRAC:.0f}% "
                     f"line. A step costs about 10s of lock, so it is not spent on "
                     f"a correction that cannot converge.")
        elig = want & reachable
        if not elig.any():
            if (movable & ~affordable).any() and not self.trim_budget_said:
                self.trim_budget_said = True
                self.say(f"[trim] stopping: total bias excursion is "
                         f"{np.abs(self.bias - BIAS).sum():.2f}V of the "
                         f"{TRIM_MAX_TOTAL_EXCURSION_V:.2f}V budget and the next step "
                         f"would exceed it. Two coils far from nominal is what gave "
                         f"three runaways on 2026-08-07.")
            return
        i = int(np.argmax(np.where(elig, np.abs(err), -1.0)))
        new = float(np.clip(self.bias[i] - np.sign(err[i]) * SLOPE_SIGN[i] * BIAS_QUANTUM,
                            BIAS_MIN, BIAS_MAX))
        if abs(new - float(self.bias[i])) < 1e-9:   # already against a clamp
            self.trim_frozen[i] = True
            return
        self.trim_pending, self.trim_ref = (i, float(self.bias[i])), total
        self.trim_steps[i] += 1
        was_counts = self.counts_mean[i]
        self._moved(t, i, new)
        self.say(f"[trim] ch{i} rests at {was_counts:.0f} counts "
                 f"({err[i]:+.0f} off mid-scale) -- bias -> {new:.2f}V")

    # -- the output ---------------------------------------------------------
    def _actuate(self, t, dt):
        # BEFORE anything reads `self.bias`, `self.vmin` or `self.vmax` this step.
        # A bias step is a force step, so the bias only ever ARRIVES, over
        # BIAS_RAMP_S; see `_bias_ramp` and BIAS_RAMP_S.
        self._bias_ramp(dt)
        live = self.enabled & self.healthy & (self.state == "DAMPING")
        self.modal_on = False
        self.modal_u[:] = 0.0
        self.modal_cols[:] = False
        self.diag_on[:] = False

        # THE COMMANDED SELF-KICK, computed here so the modal cap below can see it
        # and yield to it rather than the two of them overrunning the window
        # together. DAMPING only: a fault zeroes the drive on the same control
        # step, without waiting for the schedule to notice.
        if self.kick is not None:
            self.kick_v[:] = (self.kick.command(t) if self.state == "DAMPING"
                              else 0.0)

        if self.modal.ok and self.state == "DAMPING":
            # SENSE_OK is "this sensor's opinion counts"; DRIVE_OK is "this coil is
            # driven". They differ, and a5 -- live sensor, dead coil -- is the case
            # that proves one `live` flag cannot represent both.
            drive_ok = live & ~self.floor_bad
            enough = self.mode_seen >= MODAL_MIN_SENSORS
            qd, self.modal_rank = self.modal.project(self.qdot, drive_ok)
            f = np.where(enough, -self.mode_gain * qd, 0.0)
            u, self.modal_mask, cols, ok = self.modal.allocate(f, drive_ok)
            if ok and enough.any():
                self.modal_on, self.modal_u, self.modal_cols = True, u, cols
            elif self.modal_says is None:
                self.modal_says = (
                    "[modal] falling back to DIAGONAL: %s. Not a fault -- the "
                    "diagonal law, its gains, restricted to the survivors."
                    % ("no modal direction is reachable (mask %d, cond %.2f, rank %d "
                       "of %d)" % (self.modal_mask, self.modal.cond[self.modal_mask],
                                   self.modal.rank[self.modal_mask], self.modal.nm)
                       if not ok else
                       "no mode has %d determined sensors" % MODAL_MIN_SENSORS))
                self.say(self.modal_says)

        p, _, _ = self.pid.terms(self.vel, self.gain, dt)
        if self.modal_on:
            # Hybrid: modal on the coils A covers, the per-channel term kept on the
            # rest, summed -- never replacing p wholesale. Both are separately
            # non-positive in power, so the sum is too. The gate is "this channel
            # measures the optic": enabled, healthy, not floor-demoted, coherent.
            # Includes a coil A covers that the allocator dropped this step: it gets
            # its own diagonal term back rather than nothing.
            self.diag_on = (self.hybrid_ch & live & ~self.floor_bad
                            & ~self.modal_cols) if HYBRID_DIAGONAL \
                else np.zeros(N, bool)
            self.pid.p = np.where(self.modal_cols, self.modal_u,
                                  np.where(self.diag_on, p, 0.0))
        raw = self.bias + self.pid.p + self.pid.i + self.pid.d

        self.modal_scaled = 1.0
        if self.modal_on and self.modal_cols.any():
            # ONE factor on the whole modal vector, SOLVED against the TOTAL
            # output: a parallel underestimate is still dissipative, a per-channel
            # clip is not. The diagonal channels are colocated and stay dissipative
            # under their own clip, so they do not scale the modal vector.
            #
            # SOLVED, not estimated: `cap_scale` returns the largest scale for
            # which the TOTAL is inside the cap. The previous form scaled the MODAL
            # part by the total's peak ratio, which only reaches the cap when the
            # modal part is the whole demand -- and it is not. The D term is added
            # after, and it is what took coil 3 to 0.4229 V against a 0.250 V
            # half-window (169 %) while the allocation stayed inside its own 0.20 V
            # cap for the entire run (CLAUDE.md 2). Once a coil clips the realised
            # force is no longer A_C^+ f and the dissipation guarantee is void; the
            # 5x was measured with that happening.
            #
            # THE ALTERNATIVE IS TO RAISE THE WINDOW, AND IT IS NOT FREE. The
            # binding constraint is the 3 A SUPPLY, not the DAC (whose ceiling is
            # 2.5 V): 0.75 V of bias on all eight coils with BIAS_SWING at 0.75
            # had it AUDIBLY ALARMING, common-mode rms 15.4 -> 132.3 counts and
            # every DC level falling together by 185-286 counts
            # (data/20260820_171242_fast_lock.csv). See BIAS and BIAS_SWING. The
            # coil driver between the DAC and the coil is NOT in this repo, so
            # nothing computable from this tree bounds the coil current, and the
            # highest voltage ever commanded here without incident is 1.1 V.
            # Capping the total costs authority; guessing the window costs
            # hardware.
            cap = MODAL_TOTAL_HEADROOM * BIAS_SWING
            # The burst counts as fixed demand here for the same reason the D
            # term does: it is added after the allocation, and CLAUDE.md 2 is what
            # happens when the cap is measured on the allocation alone.
            fixed = self.pid.i + self.pid.d
            if self.kick is not None and self.kick_v.any():
                fixed = fixed + self.kick_v
            self.modal_scaled = sl.cap_scale(self.modal_u, fixed,
                                             cap, self.modal_cols)
            if self.modal_scaled < 1.0:
                self.modal_u = self.modal_u * self.modal_scaled
                self.pid.p = np.where(self.modal_cols, self.modal_u, self.pid.p)
                raw = self.bias + self.pid.p + self.pid.i + self.pid.d

        if self.kick is not None and self.kick_v.any():
            # Added to the RAW demand, so it goes through the same clip, the same
            # slew limit and the same saturation accounting as every other volt
            # this loop commands -- and the total on the kicked coil is held
            # inside MODAL_TOTAL_HEADROOM x BIAS_SWING, the cap the modal
            # allocation already respects. A commanded excitation must not be the
            # thing that takes a coil past the window.
            raw = raw + self.kick_v
            j = self.kick.coil
            room = MODAL_TOTAL_HEADROOM * BIAS_SWING
            raw[j] = self.bias[j] + float(np.clip(raw[j] - self.bias[j],
                                                  -room, room))

        out = self.pid.drive(raw, self.bias, self.vmin, self.vmax, live, dt)
        pinned = (out <= self.vmin + 1e-6) | (out >= self.vmax - 1e-6)
        self.health.sats(pinned, t)
        self.act.send(out)

    # -- reporting ----------------------------------------------------------
    def status_line(self, t):
        def one(i):
            if self.floor_bad[i]:
                return f"ch{i}:NOSIG bp={self.bp[i]:+.3f}V base={self.baseline[i]:.4f}V"
            if not self.enabled[i]:
                return f"ch{i}:off  bp={self.bp[i]:+.3f}V"
            if not self.healthy[i]:
                return f"ch{i}:DOWN bp={self.bp[i]:+.3f}V"
            flags = (("LOCK" if self.locked[i] else "....")
                     + ("!RAIL" if self.rail[i] else "")
                     + ("+d" if self.diag_on[i] else ""))
            return (f"ch{i}:{flags} g={self.gain[i]:+.4f} "
                    f"bp={self.bp[i]:+.3f}V ratio={self.ratio[i]:.2f}")

        rate = (f"{self.wire_hz:.0f}/{CONTROL_HZ:.0f}Hz avg{self.n_avg:d}"
                if self.wire_hz == self.wire_hz else f"--/{CONTROL_HZ:.0f}Hz")
        bad = f" bad{self.bad_samples}" if self.bad_samples else ""
        if self.kick is not None and self.kick.code:
            bad += (" KICK%d %s RUNAWAY-TRIP-SUPPRESSED"
                    % (self.kick.fired,
                       "driving" if self.kick.code == 1 else "ringdown"))
        if self.modal.ok:
            mode = ("MIMO" if self.modal_on else "diag") + (
                " q=" + "/".join(f"{x:+.3f}" for x in self.qdot)
                + " res=" + "/".join(f"{x:.2f}" for x in self.modal_res)
                + f" chi2/dof={self.chi2:.2f}({self.chi2_dof:d})")
        else:
            mode = "diag(no modal data)"
        return sl.Console.line(t, self.state, rate + bad, mode,
                               [one(i) for i in range(N)])

    def csv_row(self, t, counts):
        row = [t, self.state, "1" if self.stepped else "0",
               str(self.n_avg if self.stepped else 0),
               "1" if self.modal_on else "0", str(self.modal_mask),
               str(self.modal_rank), str(int(sl.Modal.mask_of(self.diag_on)))]
        row += list(self.qdot) + list(self.mode_gain) + list(self.modal_res)
        row += [self.chi2, str(int(self.chi2_dof))] + list(self.qls)
        k = self.kick
        row += [str(0 if k is None else k.code),
                str(0 if k is None else k.fired),
                str(-1 if k is None else k.coil),
                0.0 if k is None else float(self.kick_v[k.coil])]
        for i in range(N):
            row += [str(int(counts[i])), counts[i] * (A_VCC / ADC_MAX_COUNTS)]
            row += [getattr(self, k)[i] for k, _ in _LOG]
            row += [str(int(v[i])) for v in (self.rail, self.locked, self.healthy)]
        return row


sl.share(Controller, "health",
         "rail rail_std sat sat_sum healthy floor_bad clear_t")
sl.share(Controller, "pid", "p i d out prev_out prev_vel clip_excess demand primed")
sl.share(Controller, "brk", "excess_since")


# ---------------------------------------------------------------------------
# the stored floor: this module's fingerprint and its two entry points
# ---------------------------------------------------------------------------
def _fingerprint(bias, enable=None, steady=None, capture=None, ki=None, kd=None):
    """Anything that changes what a baseline MEANS goes in here."""
    g = lambda v, d: [float(x) for x in (d if v is None else v)]
    return sl.fingerprint(
        n_channels=int(N),
        enable=[bool(v) for v in (ENABLE_CHANNEL if enable is None else enable)],
        steady=g(steady, STEADY_GAIN), capture=g(capture, CAPTURE_GAIN),
        ki=g(ki, KI_GAIN), kd=g(kd, KD_GAIN), bias=g(bias, BIAS),
        dac_channels=[int(c) for c in DAC_CHANNELS],
        velocity="kalman-3mode", modes_hz=g(None, F_MODE_HZ),
        t_amp_s=T_AMP_S, t_dc_s=T_DC_S, mains_null=bool(MAINS_NULL),
        d_smooth_hz=D_SMOOTH_HZ,
        control_hz=CONTROL_HZ, a_vcc=A_VCC, adc_max_counts=int(ADC_MAX_COUNTS))


def save_baseline(ctl, path=BASELINE_PATH):
    return sl.save_baseline(path, sl.baseline_payload(
        BASELINE_FILE_SCHEMA, VERSION_TAG, ctl.baseline, ctl.floor_bad,
        ctl.calib_took, ctl.calib_early, ctl.wire_hz,
        _fingerprint(ctl.bias, ctl.enabled, ctl.steady, ctl.capture, ctl.ki, ctl.kd),
        time.time()))


def load_baseline(path=BASELINE_PATH, fresh=False, now=None):
    return sl.load_baseline(path, BASELINE_FILE_SCHEMA, _fingerprint(BIAS), N,
                            BASELINE_FILE_MAX_AGE_S,
                            time.time() if now is None else now, fresh,
                            CALIBRATION_S, CONTROL_HZ, BASELINE_WARMUP_S,
                            BASELINE_SANITY_RATIO)


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------
def _synth(mk, steps, seed, live=None, corrupt=None, proc_noise=True,
           meas_scale=1.0, step_at=None, step_ch=0, step_v=0.0,
           drop_at=None, dt=CONTROL_PERIOD_S, keep_from=0.5):
    """Drive `mk` with data its own model generated, and score the second half."""
    rng = np.random.default_rng(seed)
    mk.reset()
    mk._transition(dt)
    nm, n = mk.nm, mk.n
    lm = np.ones(n, bool) if live is None else np.asarray(live, bool)
    B = mk.Phi[:2 * nm, :2 * nm]
    L = np.linalg.cholesky(mk.Qd[:2 * nm, :2 * nm])
    xt = np.zeros(2 * nm)
    xt[0:2 * nm:2] = np.sqrt(mk.var_q)
    dt_true = np.zeros(n)
    sr = np.sqrt(mk.r)
    err, tru, chis, pmin = [], [], [], np.inf
    cut = int(steps * keep_from)
    for k in range(steps):
        xt = B @ xt
        if proc_noise:
            xt = xt + L @ rng.normal(size=2 * nm)
            dt_true = dt_true + np.sqrt(mk.q_dc * dt) * rng.normal(size=n)
        y = mk.phi @ xt[0:2 * nm:2] + dt_true + sr * meas_scale * rng.normal(size=n)
        if step_at is not None and k >= step_at:
            y[step_ch] += step_v
        if corrupt is not None:
            i, kind, amt = corrupt
            if kind == "gain":
                y[i] *= amt
            elif kind == "noise":
                y[i] += amt * sr[i] * rng.normal()
            elif kind == "offset":
                y[i] += amt
            elif kind == "stuck":
                y[i] = amt
        use = lm
        if drop_at is not None and k >= drop_at[0]:
            use = lm & np.asarray(drop_at[1], bool)
        mk.update(y, use, dt)
        if k >= cut:
            err.append(mk.qdot() - xt[1:2 * nm:2])
            tru.append(xt[1:2 * nm:2])
            chis.append(mk.chi2_per_dof())
        if k % 250 == 0:
            pmin = min(pmin, float(np.linalg.eigvalsh(mk.P).min()))
    err, tru, chis = np.asarray(err), np.asarray(tru), np.asarray(chis)
    good = np.isfinite(chis)
    return dict(rel=np.sqrt((err ** 2).mean(0)) / np.sqrt((tru ** 2).mean(0)),
                chi2=float(chis[good].mean()) if good.any() else float("nan"),
                chi2_max=float(chis[good].max()) if good.any() else float("nan"),
                pmin=pmin, finite=bool(np.isfinite(err).all()))


class _NoDac:
    def set_voltage(self, channel=0, voltage=0.0):
        pass


def _step(what, fn, *a):
    """One teardown step. A step that fails must not skip the steps after it.

    The park is the one that must never be skipped, so it goes first and every
    step is isolated. KeyboardInterrupt is caught too: a second Ctrl+C during
    shutdown used to abandon the rest of the teardown.
    """
    try:
        return fn(*a)
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as e:
        print(f"!! could not {what}: {e.__class__.__name__}: {e} -- carrying on "
              f"with the rest of the shutdown.")
        return None


def _park_on_signals():
    """Route SIGTERM and SIGHUP through the same unwind Ctrl+C uses.

    Python's default SIGTERM handler terminates the interpreter WITHOUT unwinding,
    so `main`'s `finally` never runs and every coil stays at whatever voltage the
    loop last commanded. jerk.py sends SIGINT and escalates to SIGTERM when the
    controller does not exit in time; that escalation fired twice on 2026-08-18 and
    the coils were parked by hand both times. Raising KeyboardInterrupt puts
    SIGTERM on the Ctrl+C path, and the `finally` parks either way. Same fix, same
    reason, as status.py's.
    """
    def _term(signum, frame):
        raise KeyboardInterrupt("signal %d" % signum)
    installed = []
    for name in ("SIGTERM", "SIGHUP"):
        s = getattr(signal, name, None)
        if s is None:
            continue
        try:
            signal.signal(s, _term)
            installed.append(name)
        except (ValueError, OSError, AttributeError):
            pass               # not on the main thread, or no such signal here
    return installed


def _selftest():
    import tempfile
    ok = True
    # A few checks are about WHICH reference a line of this file uses, which is a
    # property of the source and not of any run. Reading it is the honest way to
    # assert it; the alternative is a comment nobody tests.
    try:
        with open(__file__) as _fh:
            _SRC = _fh.read()
    except OSError:
        _SRC = ""

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("   %-52s %s%s" % (name, "PASS" if cond else "FAIL",
                                 ("   " + detail) if detail else ""))

    rng = np.random.default_rng(20260815)
    nm = NMODE
    Phi = np.sign(rng.normal(size=(N, nm))) * (0.5 + rng.random((N, nm)))
    Phi[6] = Phi[7] = 0.0                       # two blind sensors, as measured
    lam = np.array([1.0, -0.8, 1.4, -1.1, 0.2, 0.3, 0.0, 0.0])
    A = np.array([[lam[j] * Phi[j, m] for j in range(N)] for m in range(nm)])
    okrow = np.zeros((N, nm), bool)
    okrow[:6] = True

    def doc(coils=(0, 1, 2, 3), a=None):
        return dict(schema=MODAL_SCHEMA,
                    created=datetime.now().isoformat(timespec="seconds"),
                    git_rev="selftest", source="selftest", n_sensors=N, n_modes=nm,
                    modes=[dict(name=c, f_hz=float(f))
                           for c, f in zip("ABC", F_MODE_HZ)],
                    phi=dict(value=Phi.tolist(),
                             sigma=np.full((N, nm), 0.01).tolist(),
                             ok=okrow.tolist(), imag_frac=[0.0] * nm,
                             gauge="selftest"),
                    a=dict(value=(A if a is None else a).tolist(),
                           coils=[int(c) for c in coils], gauge="selftest",
                           provisional=False, provenance="selftest"))

    def geodoc(phi_value, a=None, coils=None, basis=PHI_BASIS, provisional=False):
        """A file in the GEOMETRIC basis: its Phi is only a cross-check."""
        e = doc(coils=A_DC_COILS if coils is None else coils,
                a=A_DC if a is None else a)
        e["phi"]["value"] = np.asarray(phi_value, float).tolist()
        e["a"]["basis"] = basis
        e["a"]["provisional"] = provisional
        return e

    d = doc()
    tmp = os.path.join(tempfile.gettempdir(), "eta_selftest_modal.json")
    with open(tmp, "w") as fh:
        json.dump(d, fh)

    print("\n  === osem.eta.py selftest ===\n")

    # ---- Phi from GEOMETRY, and the measured Phi as a check on it -----------
    P4, W4 = PHI_GEOM[:4], PHI_WARP[:4]
    check("the geometric Phi's three columns are mutually orthogonal",
          np.allclose(P4.T @ P4, 4.0 * np.eye(nm)),
          "Gram / 4 = I, off-diagonal max %.1e"
          % float(np.abs(P4.T @ P4 - 4.0 * np.eye(nm)).max()))
    check("...and WARP is in their null space -- no rigid body produces it",
          np.allclose(PHI_GEOM.T @ PHI_WARP, 0.0) and np.abs(W4).min() == 1.0,
          "Phi^T warp %s" % np.round(PHI_GEOM.T @ PHI_WARP, 12).tolist())
    check("rows a4-a7 are zero: they read axes orthogonal to Z/T1/T2",
          (not PHI_GEOM[4:].any()) and bool((PHI_GEOM[:4] != 0).all()),
          "a0-a3 fully determined, a4-a7 carry no row")

    w2 = (2.0 * np.pi * F_MODE_HZ) ** 2
    check("the omega^2 correction is applied -- a DC A is the true A / omega_m^2",
          np.allclose(A_DC, A_DC_COUNTS_PER_V * w2[:, None]) and bool(np.all(w2 > 0)),
          "x %s per mode" % np.round(w2, 2))
    mdef = _modal(None, allow=False)
    check("the DC fallback A is REFUSED by default: never validated closed-loop",
          (not mdef.ok) and "never validated closed-loop" in mdef.why,
          mdef.why[:56])
    mg = _modal(None, allow=True)
    drive4 = np.zeros(N, bool)
    drive4[A_DC_COILS] = True
    full4 = int(sum(1 << j for j in A_DC_COILS))
    Kmg = MODAL_KP * MODAL_GAIN_SCALE
    qd_g = np.array([0.30, -0.12, 0.07])[:nm]
    u_g, _, _, gok_g = mg.allocate(-Kmg * mg.project(qd_g, drive4)[0], drive4)
    check("geometry alone loads a law: no file, no gauge, no 300 s record",
          mg.ok and mg.a_from == "supplied" and mg.rank[full4] == nm, mg.why)
    check("...and the DC-derived A dissipates on it (f . qdot < 0)",
          gok_g and float((mg.a[:, A_DC_COILS] @ u_g[A_DC_COILS]) @ qd_g) < 0,
          "cond %.2f, f.qdot %+.6f"
          % (mg.cond[full4],
             float((mg.a[:, A_DC_COILS] @ u_g[A_DC_COILS]) @ qd_g)))
    mgs = _modal(None, allow=True, a=A_DC_COUNTS_PER_V)
    u_s, _, _, _ = mgs.allocate(-Kmg * mgs.project(qd_g, drive4)[0], drive4)
    check("...and A's unit-row normalisation cancels omega^2, so today it changes "
          "no allocation", np.allclose(u_g, u_s, atol=1e-14),
          "max |du| %.1e V -- the correction is for the per-mode gain, not the mix"
          % float(np.abs(u_g - u_s).max()))

    pgeo, pgbad = tmp + ".geo", tmp + ".geo-wrong"
    near = PHI_GEOM + 0.08 * rng.normal(size=(N, nm)) * (PHI_GEOM != 0)
    with open(pgeo, "w") as fh:
        json.dump(geodoc(near), fh)
    wrong = near.copy()
    wrong[:4, 2] = W4                            # mode C handed the warp direction
    with open(pgbad, "w") as fh:
        json.dump(geodoc(wrong), fh)
    mgood, mwrong = _modal(pgeo), _modal(pgbad)
    check("a measured Phi that agrees is a CHECK, and A comes from the file",
          mgood.ok and (not mgood.phi_cos_bad) and mgood.a_from == "file",
          "|cos| %s" % np.array2string(mgood.phi_cos, precision=3))
    check("...one that disagrees warns loudly at the mode that disagrees, and is "
          "still not a gate",
          mwrong.ok and mwrong.phi_cos_bad and mwrong.phi_cos[2] < PHI_COS_WARN
          and any(s.startswith("!!") for s in mwrong.notes),
          "|cos| %s against a %.2f line"
          % (np.array2string(mwrong.phi_cos, precision=3), PHI_COS_WARN))
    real = _modal(MODAL_PATH)
    have = os.path.exists(MODAL_PATH) and bool(np.isfinite(real.phi_cos).all())
    check("the SHIPPED measured Phi confirms the geometry at |cos| >= %.2f"
          % PHI_COS_WARN,
          (not have) or float(real.phi_cos.min()) >= PHI_COS_WARN,
          ("|cos| %s" % np.array2string(real.phi_cos, precision=3)) if have
          else "no measured Phi in %s -- nothing to cross-check"
               % os.path.basename(MODAL_PATH))
    mmix = _modal(tmp)
    mmirror = sl.Modal(pgeo, N, F_MODE_HZ, MODAL_SCHEMA)
    check("A and Phi are never paired across bases: a measured-gauge A against the "
          "geometric Phi is refused",
          (not mmix.ok) and "basis" in mmix.why, mmix.why[:56])
    check("...and the mirror case, a geometry-stamped A against a measured Phi",
          not mmirror.ok, mmirror.why[:56])

    print()
    m = sl.Modal(tmp, N, F_MODE_HZ, MODAL_SCHEMA, MODAL_MIN_SENSORS,
                 MODAL_MIN_COILS, MODAL_COND_MAX, MODAL_MAX_AGE_S, False,
                 ALLOC_BALANCE, MODAL_DEMAND_CAP_V)
    check("a consistent Phi/A pair loads", m.ok, m.why)
    if not m.ok:
        return 1
    check("every declared coil is kept -- colocation is not a gate",
          m.a_coils == [0, 1, 2, 3], str(m.a_coils))
    full = int(sum(1 << j for j in m.a_coils))
    check("the full coil mask passes the gate", m.gate[full],
          "cond %.2f" % m.cond[full])

    # ---- the modal Kalman ---------------------------------------------------
    print()
    mk = sl.ModalKalman(m.phi, m.rowok, KALMAN_R, KALMAN_Q, KALMAN_Q_DC,
                        F_MODE_HZ, CONTROL_PERIOD_S, N, T_AMP_S)
    check("filter is 2 x %d modes + %d sensor DC states" % (nm, N),
          mk.nx == 2 * nm + N, "%d states" % mk.nx)
    check("an undetermined Phi entry enters H as zero",
          float(np.abs(mk.H0[6:8, 0:2 * nm:2]).max()) == 0.0
          and float(np.abs(mk.phi[6:8]).max()) == 0.0)
    check("q_modal is the summed measured modal variances",
          np.allclose(mk.q_mode,
                      [KALMAN_Q[m.rowok[:, j], j].sum() for j in range(nm)]),
          np.array2string(mk.q_mode, precision=5))

    mk._transition(CONTROL_PERIOD_S)
    worst = 0.0
    for mi in range(nm):
        B = mk.Phi[2 * mi:2 * mi + 2, 2 * mi:2 * mi + 2]
        st = np.array([1.0, 0.0])
        e0 = mk.w[mi] ** 2 * st[0] ** 2 + st[1] ** 2
        for _ in range(100000):
            st = B @ st
        worst = max(worst, abs((mk.w[mi] ** 2 * st[0] ** 2 + st[1] ** 2) / e0 - 1.0))
    check("exact-ZOH conserves oscillator energy, 1e5 steps", worst < 1e-9,
          "worst drift %.2e over all three modes" % worst)
    eu = np.array([[1.0, CONTROL_PERIOD_S],
                   [-mk.w[0] ** 2 * CONTROL_PERIOD_S, 1.0]])
    st = np.array([1.0, 0.0])
    for _ in range(100000):
        st = eu @ st
    eu_drift = abs((mk.w[0] ** 2 * st[0] ** 2 + st[1] ** 2) / mk.w[0] ** 2 - 1.0)
    check("...and a Euler step at the same dt does NOT", eu_drift > 1e-3,
          "Euler drift %.3e over the same 1e5 steps" % eu_drift)

    allrows = m.rowok.any(axis=1)
    r0 = _synth(mk, 6000, 4001, live=allrows, proc_noise=False, meas_scale=1e-3)
    check("recovers a planted modal velocity, all rows",
          float(r0["rel"].max()) < 0.01,
          "rel err %s" % np.array2string(r0["rel"], precision=5))
    d1 = allrows.copy()
    d1[0] = False
    r1 = _synth(mk, 6000, 4001, live=d1, proc_noise=False, meas_scale=1e-3)
    check("...still, with sensor a0's row DELETED", float(r1["rel"].max()) < 0.01,
          "rel err %s" % np.array2string(r1["rel"], precision=5))
    d2 = d1.copy()
    d2[1] = False
    r2 = _synth(mk, 6000, 4001, live=d2, proc_noise=False, meas_scale=1e-3)
    check("...and with a0 and a1 both DELETED", float(r2["rel"].max()) < 0.01,
          "rel err %s" % np.array2string(r2["rel"], precision=5))
    rd = _synth(mk, 6000, 4001, live=allrows, proc_noise=False, meas_scale=1e-3,
                drop_at=(3000, d2), keep_from=0.75)
    check("sensors leaving mid-run: still exact, still finite",
          rd["finite"] and float(rd["rel"].max()) < 0.01,
          "rel err %s after two rows go at t=30s"
          % np.array2string(rd["rel"], precision=5))

    # ---- chi2, logged and not acted on -------------------------------------
    print()
    base = _synth(mk, 8000, 4002, live=allrows)
    check("chi2/dof is order 1 when the model fits", 0.7 < base["chi2"] < 1.4,
          "mean %.4f, max %.2f over %d dof" % (base["chi2"], base["chi2_max"],
                                               int(allrows.sum())))
    check("P stays positive definite across the run", base["pmin"] > 0,
          "min eigenvalue %.3e" % base["pmin"])
    b1 = _synth(mk, 8000, 4002, live=d1)
    b2 = _synth(mk, 8000, 4002, live=d2)
    check("...and stays order 1 as rows are deleted",
          0.7 < b1["chi2"] < 1.4 and 0.7 < b2["chi2"] < 1.4,
          "%d rows %.3f, %d rows %.3f" % (int(d1.sum()), b1["chi2"],
                                          int(d2.sum()), b2["chi2"]))

    print()

    def rises(name, kw, floor):
        got = _synth(mk, 8000, 4002, live=allrows, **kw)
        check(name, got["chi2"] > floor,
              "chi2/dof %.3f against a baseline %.3f" % (got["chi2"], base["chi2"]))
        return got["chi2"]

    rises("extra noise on one sensor, 3 sigma", dict(corrupt=(3, "noise", 3.0)), 1.5)
    rises("extra noise on one sensor, 5 sigma", dict(corrupt=(3, "noise", 5.0)), 2.5)
    rises("one sensor's GAIN wrong by 1.5x", dict(corrupt=(3, "gain", 1.5)), 2.0)
    rises("one sensor's SIGN inverted", dict(corrupt=(3, "gain", -1.0)), 5.0)
    rises("one sensor STUCK at a constant reading", dict(corrupt=(3, "stuck", 1.0)), 5.0)

    noisy = _synth(mk, 8000, 4002, live=allrows, corrupt=(0, "gain", 1.5))
    quiet = _synth(mk, 8000, 4002, live=allrows, corrupt=(3, "gain", 1.5))
    check("the same fault shows more on a QUIETER sensor",
          quiet["chi2"] > noisy["chi2"],
          "a0 (sqrt R %.3f V) %.2f, a3 (%.3f V) %.2f"
          % (np.sqrt(KALMAN_R[0]), noisy["chi2"], np.sqrt(KALMAN_R[3]), quiet["chi2"]))

    offs = _synth(mk, 8000, 4002, live=allrows, corrupt=(3, "offset", 0.5))
    check("a STATIC offset is invisible (the DC state absorbs it)",
          abs(offs["chi2"] - base["chi2"]) < 0.1,
          "chi2/dof %.4f against a baseline %.4f -- the design, not a gap"
          % (offs["chi2"], base["chi2"]))
    early = _synth(mk, 8000, 4002, live=allrows, step_at=4000, step_ch=3,
                   step_v=0.5, keep_from=0.5)
    late = _synth(mk, 8000, 4002, live=allrows, step_at=4000, step_ch=3,
                  step_v=0.5, keep_from=0.9)
    check("a STEP offset is visible, then absorbed",
          early["chi2"] > base["chi2"] * 1.5 and late["chi2"] < base["chi2"] * 1.5,
          "0-40s after the step %.3f, 40-80s after %.3f, baseline %.3f"
          % (early["chi2"], late["chi2"], base["chi2"]))

    # ---- the law dissipates -------------------------------------------------
    print()
    mk.reset()
    mk._transition(CONTROL_PERIOD_S)
    B = mk.Phi[:2 * nm, :2 * nm]
    xt = np.zeros(2 * nm)
    xt[0:2 * nm:2] = np.sqrt(mk.var_q)
    Kmode = MODAL_KP[:nm] * MODAL_GAIN_SCALE
    drive = np.zeros(N, bool)
    drive[[0, 1, 2, 3]] = True
    powers = []
    srn = np.random.default_rng(4003)
    for k in range(4000):
        xt = B @ xt
        y = mk.phi @ xt[0:2 * nm:2] + np.sqrt(mk.r) * 1e-3 * srn.normal(size=N)
        qd_hat = mk.update(y, allrows, CONTROL_PERIOD_S)
        if k > 2000:
            qd, rk = m.project(qd_hat, drive)
            u, _, _, gok = m.allocate(-Kmode * qd, drive)
            realised = m.a[:, [0, 1, 2, 3]] @ u[[0, 1, 2, 3]]
            powers.append(float(realised @ xt[1:2 * nm:2]))
    powers = np.asarray(powers)
    check("the filter's own force DISSIPATES against TRUE qdot",
          powers.mean() < 0 and (powers < 0).mean() > 0.99,
          "mean f.qdot %+.3e, negative on %.1f%% of steps"
          % (powers.mean(), 100.0 * (powers < 0).mean()))

    mk.reset()
    y0 = np.zeros(N)
    mk.update(y0, allrows, CONTROL_PERIOD_S)
    t0 = time.perf_counter()
    for _ in range(5000):
        mk.update(y0, allrows, CONTROL_PERIOD_S)
    us = (time.perf_counter() - t0) / 5000 * 1e6
    check("one modal step costs a small fraction of the budget",
          us < 0.2 * CONTROL_PERIOD_S * 1e6,
          "%.1f us/step = %.2f%% of the %.0f ms period"
          % (us, 100.0 * us / (CONTROL_PERIOD_S * 1e6), CONTROL_PERIOD_S * 1e3))

    # ---- sensing, projection, allocation ------------------------------------
    print()
    sense_ok = np.array([True] * 6 + [False, False])
    qtrue = np.array([0.30, -0.12, 0.07])[:nm]
    v = m.phi * qtrue                            # v[i,m] = Phi[i,m] * qdot_m
    q, res, seen = m.sense(v, sense_ok)
    check("sensing recovers a planted modal velocity",
          np.allclose(q, qtrue, rtol=1e-9, atol=1e-12),
          "%s vs %s" % (np.round(q, 6), np.round(qtrue, 6)))
    check("out-of-mode residual is zero on a consistent reading",
          float(np.max(np.abs(res))) < 1e-9, "max %.2e" % float(np.max(np.abs(res))))
    vbad = v.copy()
    vbad[1] *= -3.0
    _, rbad, _ = m.sense(vbad, sense_ok)
    check("a disagreeing sensor raises the residual",
          float(np.max(rbad)) > 0.2, "max %.3f" % float(np.max(rbad)))

    coils4 = sense_ok & np.array([True] * 4 + [False] * 4)
    f = -np.array([0.02, 0.01, 0.015])[:nm]
    u, mask, cols, gok = m.allocate(f, coils4)
    got = m.a[:, [0, 1, 2, 3]] @ u[[0, 1, 2, 3]]
    check("allocation is exact at full row rank (A A+ = I)",
          gok and np.allclose(got, f, rtol=1e-8, atol=1e-12),
          "%s vs %s" % (np.round(got, 6), np.round(f, 6)))
    rn = np.linalg.norm(m.pinv[full][1], axis=1)
    check("the row-balanced allocator VERIFIES A P = I, else falls back",
          m.balanced[full] and np.allclose(m.a[:, m.a_coils] @ m.pinv[full][1],
                                           m.pinv[full][2], rtol=1e-7, atol=1e-10),
          "row-norm spread %.4f, balanced %s" % (rn.max() / rn.min(), m.balanced[full]))

    K = MODAL_KP[:nm] * MODAL_GAIN_SCALE
    uq, _, _, _ = m.allocate(-K * qtrue, coils4)
    power = float((m.a[:, [0, 1, 2, 3]] @ uq[[0, 1, 2, 3]]) @ qtrue)
    check("the realised modal force DISSIPATES (f . qdot < 0)", power < 0,
          "f.qdot = %+.6f" % power)

    # Per-mode gain: raising one mode's K changes only that mode's force.
    K2 = K.copy()
    K2[1] *= 3.0
    f1 = -K * qtrue
    f2 = -K2 * qtrue
    check("MODAL_KP is per mode: mode B's gain moves only mode B's force",
          abs(f2[1] / f1[1] - 3.0) < 1e-12
          and np.allclose(np.delete(f1, 1), np.delete(f2, 1)),
          "K %s -> %s, f %s -> %s" % (np.round(K, 4), np.round(K2, 4),
                                      np.round(f1, 5), np.round(f2, 5)))
    check("...and ships FLAT -- per-mode retuning is a bench job (MODAL_KP)",
          len(MODAL_KP) == nm and np.allclose(MODAL_KP, MODAL_KP[0])
          and bool(np.all(MODAL_KP >= 0)),
          "MODAL_KP %s" % np.round(MODAL_KP, 4))

    for j in m.a_coils:
        mm = full & ~(1 << j)
        dok = np.zeros(N, bool)
        for k in m.a_coils:
            if k != j:
                dok[k] = True
        qd, rk = m.project(qtrue, dok)
        fj = -K * qd
        uu, mkk, _, gg = m.allocate(fj, dok)
        cols = [k for k in m.a_coils if k != j]
        if gg:
            realised = m.a[:, cols] @ uu[cols]
            exact = np.allclose(realised, fj, rtol=1e-6, atol=1e-9)
            power = float(realised @ qtrue)
            check("drop coil %d -> rank %d of %d, dissipates%s"
                  % (j, rk, nm, ", and exact" if exact else " (truncated)"),
                  mkk == mm and power < 0 and (exact or rk < nm),
                  "cond %.2f, f.qdot %+.6f" % (m.cond[mm], power))
        else:
            check("drop coil %d -> nothing reachable, refuses cleanly" % j,
                  mkk == mm and not uu.any() and rk == 0,
                  "cond %.2f, rank 0" % m.cond[mm])
    for i in range(4):
        s2 = sense_ok.copy()
        s2[i] = False
        q2, _, seen2 = m.sense(v, s2)
        check("drop sensor a%d -> modal velocity still exact" % i,
              np.allclose(q2, qtrue, rtol=1e-8, atol=1e-12), "seen %s" % seen2)

    # ---- the hybrid law -----------------------------------------------------
    print()
    # Coils 4 and 5 are outside A here. Give them a TRUE colocated force direction
    # lam_j * Phi[j,:] and a per-channel gain of the matching sign, scaled by a
    # planted coherent fraction, then measure the TOTAL realised modal power
    # against the true modal velocity.
    hyb_j = [4, 5]
    hyb_lam = np.array([0.6, -0.9])
    hyb_coh = np.array([0.46, 0.50])
    drive_all = np.zeros(N, bool)
    drive_all[[0, 1, 2, 3]] = True

    def hybrid_power(sign, coh):
        tot = []
        rg = np.random.default_rng(99)
        for _ in range(400):
            qd = rg.normal(size=nm)
            umod, _, _, _ = m.allocate(-K * m.project(qd, drive_all)[0], drive_all)
            fm = m.a[:, [0, 1, 2, 3]] @ umod[[0, 1, 2, 3]]
            for j, lj, cj in zip(hyb_j, hyb_lam, coh):
                uj = sign * np.sign(lj) * HYBRID_KP * cj * (-float(Phi[j] @ qd))
                fm = fm + lj * Phi[j] * uj
            tot.append(float(fm @ qd))
        return np.asarray(tot)

    good = hybrid_power(+1.0, hyb_coh)
    check("hybrid total realised modal power is NON-POSITIVE",
          bool(np.all(good <= 1e-15)),
          "max f.qdot %+.3e over %d random modal velocities" % (good.max(), good.size))
    bad = hybrid_power(-1.0, hyb_coh)
    check("...and POSITIVE with the per-channel sign wrong -- why the sign is measured",
          bool((bad > 0).any()), "%.0f%% of steps pump" % (100.0 * (bad > 0).mean()))
    zero = hybrid_power(+1.0, np.zeros(2))
    check("...and a channel at coherent fraction 0 contributes nothing at all",
          bool(np.all(zero <= 1e-15)),
          "max f.qdot %+.3e with both hybrid gains zeroed" % zero.max())

    check("HYBRID_GAIN is HYBRID_KP x DAMP_SIGN x the measured coherent fraction",
          np.allclose(HYBRID_GAIN, HYBRID_KP * DAMP_SIGN * HYBRID_COHERENT
                      * np.array(HYBRID_CHANNEL, float))
          and abs(abs(HYBRID_GAIN[4]) / HYBRID_KP - 0.09) < 1e-12
          and abs(abs(HYBRID_GAIN[6]) / HYBRID_KP - 0.12) < 1e-12
          and abs(abs(HYBRID_GAIN[7]) / HYBRID_KP - 0.05) < 1e-12,
          "a4 %.5f a6 %.5f a7 %.5f against a full %.5f"
          % (HYBRID_GAIN[4], HYBRID_GAIN[6], HYBRID_GAIN[7], HYBRID_KP))
    # THE DISSIPATION CONDITION, asserted as algebra rather than as a remembered
    # sign. p = gain x (-vel), so the demand about bias is -gain x vel, the force
    # goes as slope x (u - bias), and the power delivered is
    #     slope x (u - bias) x vel = -slope x gain x vel^2
    # which is negative -- dissipating -- if and only if slope x gain > 0.
    # This shipped INVERTED until 2026-08-20 and every driven channel pumped:
    # dissipating on 5.4 / 11.0 / 22.6 / 12.3 % of steps on a0-a3
    # (data/20260820_173738_fast_lock.csv, evaluated at the control clock).
    check("slope x gain > 0 on every driven channel -- the dissipation condition",
          bool(np.all(SLOPE_SIGN[:4] * np.sign(STEADY_GAIN[:4]) > 0))
          and np.array_equal(DAMP_SIGN, SLOPE_SIGN),
          "slope %s x gain %s = %s (all must be +1)"
          % (SLOPE_SIGN[:4].astype(int), np.sign(STEADY_GAIN[:4]).astype(int),
             (SLOPE_SIGN[:4] * np.sign(STEADY_GAIN[:4])).astype(int)))
    check("...and the hybrid channels use the SAME rule, not the opposite one",
          bool(np.all(SLOPE_SIGN[[4, 6, 7]] * np.sign(HYBRID_GAIN[[4, 6, 7]]) > 0)),
          "a4/a6/a7 slope %s x hybrid gain %s -- these carried the OPPOSITE "
          "convention to a0-a3 until 2026-08-20, which is the two-conventions bug"
          % (SLOPE_SIGN[[4, 6, 7]].astype(int),
             np.sign(HYBRID_GAIN[[4, 6, 7]]).astype(int)))

    # ---- the sensor calibration, and the gain vector built on it -----------
    # SIGNS ARE NOT ENOUGH ON THIS RIG. The block above says each channel pushes
    # the right way against its OWN sensor; that argument is colocation and
    # colocation does not hold here. What follows asserts the thing that actually
    # has to be true -- that the MODAL damping matrix is positive definite, so
    # every mode loses energy -- and it asserts it for the shipped vector, for
    # the flat vector it replaced, and against a vector known to pump.
    print()
    m_dc = DC_MATRIX_COUNTS_PER_V.T                 # [sensor][coil]
    check("DC_MATRIX's diagonal IS the measured own-coil slope, not a transpose",
          bool(np.all(np.abs(np.diag(DC_MATRIX_COUNTS_PER_V)
                             - SENSOR_SLOPE_COUNTS_PER_V[:4])
                      <= SENSOR_SLOPE_SIGMA[:4])),
          "diag %s against slopes %s, worst gap %.2f counts/V inside sigma %s"
          % (np.round(np.diag(DC_MATRIX_COUNTS_PER_V), 1),
             np.round(SENSOR_SLOPE_COUNTS_PER_V[:4], 1),
             float(np.abs(np.diag(DC_MATRIX_COUNTS_PER_V)
                          - SENSOR_SLOPE_COUNTS_PER_V[:4]).max()),
             np.round(SENSOR_SLOPE_SIGMA[:4], 1)))
    # A RIGID PLATE CANNOT WARP. This is the whole basis of SENSOR_GAIN_H, and it
    # is over-determined -- one equation per coil, four unknowns -- so it is a
    # real test and not a fit that always succeeds. Coils 0-3 only, because that
    # is the part of the pass this file keeps; the published solve used all eight.
    warp_raw = PHI_WARP[:4] @ m_dc
    warp_cal = PHI_WARP[:4] @ (np.diag(SENSOR_GAIN_H) @ m_dc)
    check("h nulls the warp a rigid plate cannot have -- 4 unknowns, 4 equations "
          "here and 8 in the published solve",
          float(np.abs(warp_cal).max()) < 0.06 * float(np.abs(warp_raw).max()),
          "per coil %s -> %s counts, worst |warp| %.1f -> %.1f"
          % (np.round(warp_raw, 1), np.round(warp_cal, 1),
             float(np.abs(warp_raw).max()), float(np.abs(warp_cal).max())))
    hcorr = float(np.corrcoef(np.abs(SENSOR_GAIN_H),
                              1.0 / np.abs(SENSOR_SLOPE_COUNTS_PER_V[:4]))[0, 1])
    check("...and h agrees with 1/|own-coil slope|, which it never saw",
          hcorr > 0.95,
          "corr %.4f: h is fitted from the OFF-diagonal warp constraint, the "
          "slopes are the DIAGONAL, and neither entered the other" % hcorr)

    # THE CONSTRAINT. sym(A diag(g) Phi) positive definite, in both forms, for
    # the shipped vector AND for the flat one it replaced. Numbers measured
    # 2026-08-20; see the gain block for where each comes from.
    flat_g = KP_REF * np.concatenate([DAMP_SIGN_A0A3, np.zeros(N - 4)])
    for form, want_flat, want_ship in (("counts", 1.9393, 0.7593),
                                       ("physical", 2.0343, 1.2797)):
        e_ship = dissipation_eig(STEADY_GAIN, form)
        e_flat = dissipation_eig(flat_g, form)
        check("EVERY MODE DISSIPATES under the shipped gains -- sym(S) is "
              "positive definite, %s form" % form,
              bool(np.all(e_ship > 0.0)),
              "eig %s (all must be > 0); the flat vector this replaced gives %s"
              % (np.round(e_ship, 4), np.round(e_flat, 4)))
        check("...and the reference numbers have not drifted (%s)" % form,
              abs(float(e_flat.min()) - want_flat) < 5e-3
              and abs(float(e_ship.min()) - want_ship) < 5e-3,
              "worst mode flat %.4f (recorded %.4f), shipped %.4f (recorded "
              "%.4f) -- a factor %.2f, which is what normalisation COSTS"
              % (e_flat.min(), want_flat, e_ship.min(), want_ship,
                 e_ship.min() / e_flat.min()))
    # The check has to be able to FAIL, or it asserts nothing. Inverting one
    # channel is the 2026-08-20 sign bug in miniature, and it must show up here.
    bad_g = STEADY_GAIN.copy()
    bad_g[0] = -bad_g[0]
    e_bad = dissipation_eig(bad_g, "counts")
    check("...and the test can fail: inverting ONE channel's gain pumps a mode",
          bool(np.any(dissipation_eig(-STEADY_GAIN, "counts") < 0.0))
          and bool(np.any(e_bad < 0.0)),
          "gain vector negated -> eig %s; ch0 alone inverted -> eig %s"
          % (np.round(dissipation_eig(-STEADY_GAIN, "counts"), 3),
             np.round(e_bad, 3)))

    # WHAT NORMALISATION IS FOR, asserted as the property rather than as four
    # literals: equal coil demand per unit PHYSICAL velocity of the plate. The
    # flat vector was 3.37x out on a2, which is the whole reason coil 2 clipped
    # and spent 34.41 % of the 18:15 DAMPING block at the 2.0 V/s slew cap.
    phys_ship = np.abs(STEADY_GAIN[:4]) / np.abs(SENSOR_GAIN_H)
    phys_flat = np.abs(flat_g[:4]) / np.abs(SENSOR_GAIN_H)
    check("the shipped gains are FLAT IN PHYSICS -- g/h equal on all four",
          float(phys_ship.max() / phys_ship.min() - 1.0) < 1e-9,
          "g/h %s (spread %.2e); the flat vector spread %s, a factor %.2f"
          % (np.round(phys_ship, 6), float(phys_ship.max() / phys_ship.min() - 1.0),
             np.round(phys_flat, 4), float(phys_flat.max() / phys_flat.min())))
    check("normalisation only ever REDUCES a gain -- nothing goes past KP_REF",
          bool(np.all(np.abs(STEADY_GAIN) <= KP_REF + 1e-15))
          and bool(np.all(np.abs(STEADY_GAIN[:4]) <= np.abs(flat_g[:4]) + 1e-15))
          and bool(np.all(np.abs(KD_GAIN) <= KD_REF + 1e-15)),
          "max |steady| %.5f of a %.5f ceiling, max |kd| %.6f of %.6f"
          % (float(np.abs(STEADY_GAIN).max()), KP_REF,
             float(np.abs(KD_GAIN).max()), KD_REF))
    check("...even if a re-measured h says otherwise: the clamp holds",
          float(np.abs(slope_gain(KP_REF, h=[0.01, 1.0, 1.0, 1.0])).max())
          <= KP_REF + 1e-15,
          "an h with a 100x outlier still gives max |g| %.5f"
          % float(np.abs(slope_gain(KP_REF, h=[0.01, 1.0, 1.0, 1.0])).max()))
    check("P and D are normalised by the SAME factor, so P/D per channel is flat",
          np.allclose(STEADY_GAIN[:4] / KP_REF, KD_GAIN[:4] / KD_REF, atol=1e-15)
          and np.array_equal(STEADY_GAIN, CAPTURE_GAIN),
          "steady/KP_REF %s == kd/KD_REF %s"
          % (np.round(STEADY_GAIN[:4] / KP_REF, 6),
             np.round(KD_GAIN[:4] / KD_REF, 6)))
    check("the normalised channels are exactly Phi's rows, and nothing else moved",
          list(GAIN_NORM_CHANNELS) == list(np.nonzero(PHI_GEOM_ROWS)[0])
          and not STEADY_GAIN[4:].any() and not KD_GAIN[4:].any(),
          "normalised %s, Phi rows %s, a4-a7 gains %s"
          % (list(GAIN_NORM_CHANNELS), list(np.nonzero(PHI_GEOM_ROWS)[0]),
             np.round(STEADY_GAIN[4:], 6)))
    check("DAMP_SIGN_A0A3 is DAMP_SIGN[:4] -- the two cannot drift apart",
          np.array_equal(DAMP_SIGN_A0A3, DAMP_SIGN[:4])
          and np.array_equal(np.sign(STEADY_GAIN[:4]), DAMP_SIGN_A0A3),
          "gain signs %s, DAMP_SIGN[:4] %s"
          % (np.sign(STEADY_GAIN[:4]).astype(int), DAMP_SIGN[:4].astype(int)))
    check("GAIN_NORMALISE=False puts the flat law back EXACTLY, for a null test",
          np.array_equal(slope_gain(KP_REF, normalise=False), flat_g),
          "flat %s" % np.round(slope_gain(KP_REF, normalise=False)[:4], 4))
    # h is per SENSOR and appears only in the per-channel diagonal gains. Folding
    # it into Phi was tried on the bench on 2026-08-20 and PUMPED (cond 3.32 ->
    # 9.41, chi2/dof 3.47 -> 844.70) because KALMAN_R is a variance in raw volts.
    # Assert the separation, so that change cannot arrive here by accident.
    check("h touches the diagonal gains ONLY -- Phi is still exactly +-1",
          bool(np.all(np.abs(PHI_GEOM[:4]) == 1.0))
          and np.allclose(PHI_GEOM.T @ PHI_GEOM, 4.0 * np.eye(NMODE)),
          "PHI_GEOM rows %s -- normalising the READINGS by h needs KALMAN_R "
          "rescaled by h^2 in the same change, and that is not this change"
          % np.abs(PHI_GEOM[:4]).max())

    # ---- a bias step is a force step: the trim must RAMP -------------------
    # 2026-08-20, data/20260820_185657_fast_lock.csv: every stepped trim threw
    # the T2 modal residual 7-8x (1.85 -> 15.28 counts rms) and took about 10 s
    # to recover, and 0.02-0.30 Hz band power over the DAMPING block was 2950x
    # its CALIBRATING level while the mode band was 8.5x DOWN. See BIAS_RAMP_S.
    print()
    check("the ramp is ten periods of the slowest mode, like status.park_ramped",
          BIAS_RAMP_S / (1.0 / F_MODE_HZ[0]) > 9.0
          and abs(BIAS_RAMP_PER_S - BIAS_QUANTUM / BIAS_RAMP_S) < 1e-15,
          "%.1f s = %.1f periods of %.5f Hz, %.4f V/s -- %.0fx under the %.1f V/s "
          "slew limit that used to be the only thing slowing a step"
          % (BIAS_RAMP_S, BIAS_RAMP_S * F_MODE_HZ[0], F_MODE_HZ[0],
             BIAS_RAMP_PER_S, MAX_SLEW_PER_S / BIAS_RAMP_PER_S, MAX_SLEW_PER_S))
    check("...and the period leaves the ramp AND a full quiet window to settle",
          TRIM_PERIOD_S >= BIAS_RAMP_S + QUIET_WINDOW_S,
          "%.0f s >= %.0f s ramp + %.0f s window; it was 15.0 s, shorter than the "
          "10 s recovery it caused, so steps stacked"
          % (TRIM_PERIOD_S, BIAS_RAMP_S, QUIET_WINDOW_S))

    cr = Controller(_NoDac())
    cr._moved(0.0, 2, float(cr.bias[2]) + BIAS_QUANTUM)
    seen, dtc, tgt = [], CONTROL_PERIOD_S, float(cr.bias_target[2])
    for k in range(int(3.0 * BIAS_RAMP_S / dtc)):
        prev = float(cr.bias[2])
        cr._bias_ramp(dtc)
        seen.append((abs(float(cr.bias[2]) - prev) / dtc, float(cr.bias[2])))
        if abs(float(cr.bias[2]) - tgt) < 1e-9:
            arrived = (k + 1) * dtc
            break
    else:
        arrived = float("inf")
    fastest = max(r for r, _ in seen)
    check("A TRIM NEVER MOVES THE BIAS FASTER THAN BIAS_RAMP_PER_S",
          fastest <= BIAS_RAMP_PER_S + 1e-12,
          "fastest %.5f V/s against the %.5f V/s limit, over %d control steps"
          % (fastest, BIAS_RAMP_PER_S, len(seen)))
    check("...and it does arrive, in about BIAS_RAMP_S rather than never",
          abs(arrived - BIAS_RAMP_S) < 0.5 * BIAS_RAMP_S
          and abs(float(cr.bias[2]) - tgt) < 1e-9,
          "reached %.4f V after %.2f s (BIAS_RAMP_S %.1f s)"
          % (cr.bias[2], arrived, BIAS_RAMP_S))
    check("...and the window follows the LIVE bias, not the target",
          abs(float(cr.vmin[2]) - (float(cr.bias[2]) - BIAS_SWING)) < 1e-12
          and abs(float(cr.vmax[2]) - (float(cr.bias[2]) + BIAS_SWING)) < 1e-12,
          "vmin %.4f vmax %.4f about bias %.4f -- saturation accounting stays "
          "honest while a ramp is in flight" % (cr.vmin[2], cr.vmax[2], cr.bias[2]))
    # The arrays are handed out by reference, so a rebind here would leave every
    # holder on the pre-trim array. Assert identity, not just value.
    b0, v0 = cr.bias, cr.vmin
    cr._moved(0.0, 1, float(cr.bias[1]) - BIAS_QUANTUM)
    cr._bias_ramp(dtc)
    check("...and the ramp mutates in place, so no holder goes stale",
          cr.bias is b0 and cr.vmin is v0,
          "bias and vmin are the same arrays Channel and csv_row were given")

    # A step costs about 10 s of lock, so it is not spent where it cannot converge.
    cq2 = Controller(_NoDac())
    cq2.counts_mean = np.full(N, MID_COUNTS)
    cq2.counts_mean[2] = 860.0                 # as measured on the 18:56 run
    cq2.counts_mean[4] = MID_COUNTS + 34.0     # a4's documented drift
    # a1 BELOW mid-scale, deliberately: from BIAS 0.50 the trim has three
    # quanta of room UP to BIAS_MAX 1.25 and only one DOWN to BIAS_MIN 0.25, so
    # a downward correction of the same size is refused as unreachable. That
    # asymmetry is real and this is where it is visible.
    cq2.counts_mean[1] = MID_COUNTS - 100.0
    err2 = cq2.counts_mean - MID_COUNTS
    step2 = -np.sign(err2) * SLOPE_SIGN * BIAS_QUANTUM
    room2 = TRIM_MAX_TOTAL_EXCURSION_V - (np.abs(cq2.bias - BIAS).sum()
                                          - np.abs(cq2.bias - BIAS))
    far2 = np.clip(cq2.bias + np.sign(step2)
                   * np.maximum(TRIM_MAX_STEPS - cq2.trim_steps, 0) * BIAS_QUANTUM,
                   BIAS_MIN, BIAS_MAX)
    far2 = np.clip(far2, BIAS - room2, BIAS + room2)
    frac2 = (np.abs(far2 - cq2.bias) * np.abs(SENSOR_SLOPE_COUNTS_PER_V)
             / np.maximum(np.abs(err2), 1e-9))
    check("a trim it cannot finish is REFUSED -- a2 and a4, on measured slopes",
          frac2[2] < TRIM_MIN_REACHABLE_FRAC and frac2[4] < TRIM_MIN_REACHABLE_FRAC,
          "a2 reaches %.0f%% of a %+.0f-count offset at %+.1f counts/V, a4 %.0f%% "
          "of %+.0f at %+.1f, against a %.0f%% line"
          % (100 * frac2[2], err2[2], SENSOR_SLOPE_COUNTS_PER_V[2],
             100 * frac2[4], err2[4], SENSOR_SLOPE_COUNTS_PER_V[4],
             100 * TRIM_MIN_REACHABLE_FRAC))
    check("...while a channel it CAN finish is untouched by the new gate",
          frac2[1] >= TRIM_MIN_REACHABLE_FRAC,
          "a1 reaches %.0f%% of a %+.0f-count offset at %+.1f counts/V -- three "
          "quanta UP to BIAS_MAX; the same offset the other way has one quantum "
          "DOWN to BIAS_MIN and would be refused"
          % (100 * frac2[1], err2[1], SENSOR_SLOPE_COUNTS_PER_V[1]))

    carm = Controller(_NoDac())
    check("the hybrid arms a4/a6/a7 on the shipped enable vector",
          bool(carm._hyb_arm()[[4, 6, 7]].all()) and not carm._hyb_arm()[5],
          "armed %s" % Controller._who(carm._hyb_arm()))
    # Measured on the suite's ch4/ch6/ch7-disabled scenario: without the `enabled`
    # gate the hybrid armed all three, and the breaker then faulted the rig 6 times
    # on channels whose whole baseline is 0.0004 V of out-of-band interference and
    # which are never commanded at all.
    coff = Controller(_NoDac(),
                      enable=[True, True, True, True, False, True, False, False])
    check("...and never arms a DISABLED coil",
          not coff._hyb_arm()[[4, 6, 7]].any() and coff._hyb_arm().sum() == 0,
          "armed %s" % (Controller._who(coff._hyb_arm()) or "nothing"))

    # A covering every coil: the hybrid must reduce EXACTLY to the pure modal law.
    A8 = np.zeros((nm, N))
    A8[:, :4] = A[:, :4]
    A8[:, 4:6] = np.array([hyb_lam[k] * Phi[j] for k, j in enumerate(hyb_j)]).T
    p8 = tmp + ".all"
    with open(p8, "w") as fh:
        json.dump(doc(coils=(0, 1, 2, 3, 4, 5), a=A8), fh)
    m8 = sl.Modal(p8, N, F_MODE_HZ, MODAL_SCHEMA, MODAL_MIN_SENSORS, MODAL_MIN_COILS,
                  MODAL_COND_MAX, MODAL_MAX_AGE_S, False, ALLOC_BALANCE,
                  MODAL_DEMAND_CAP_V)
    all6 = np.zeros(N, bool)
    all6[[0, 1, 2, 3, 4, 5]] = True
    u8, _, cols8, _ = m8.allocate(-K * m8.project(qtrue, all6)[0], all6)
    # _actuate's own expression, with 1.0 standing in for a non-zero diagonal term.
    diag8 = np.array(HYBRID_CHANNEL) & all6 & ~cols8
    p8_out = np.where(cols8, u8, np.where(diag8, 1.0, 0.0))
    check("with A covering every coil the hybrid IS the pure modal law",
          m8.ok and bool(cols8[all6].all()) and not diag8.any()
          and np.allclose(p8_out, u8),
          "%d coils commanded, no diagonal term left to add" % int(cols8.sum()))

    # ---- refusals -----------------------------------------------------------
    print()

    def refuses(name, mutate):
        e = json.loads(json.dumps(d))
        mutate(e)
        p2 = tmp + ".bad"
        with open(p2, "w") as fh:
            json.dump(e, fh)
        mm = sl.Modal(p2, N, F_MODE_HZ, MODAL_SCHEMA, MODAL_MIN_SENSORS,
                      MODAL_MIN_COILS, MODAL_COND_MAX, MODAL_MAX_AGE_S, False,
                      ALLOC_BALANCE, MODAL_DEMAND_CAP_V)
        check(name, not mm.ok, mm.why[:64])
        return mm

    refuses("refuses a wrong schema", lambda e: e.__setitem__("schema", "nope"))
    refuses("refuses modes at other frequencies",
            lambda e: e["modes"][1].__setitem__("f_hz", 1.20))
    refuses("refuses a stale file",
            lambda e: e.__setitem__("created", "2020-01-01T00:00:00"))
    refuses("refuses a Phi/A gauge mismatch",
            lambda e: e["a"].__setitem__("gauge", "somewhere else"))
    refuses("refuses a provisional A by default",
            lambda e: e["a"].__setitem__("provisional", True))
    refuses("refuses a missing A", lambda e: e.__setitem__("a", None))
    check("refuses a file that is not there",
          not sl.Modal(tmp + ".nope", N, F_MODE_HZ, MODAL_SCHEMA).ok)

    # A sign-flipped A row used to be REFUSED here. It is now loaded and logged:
    # the sign test that refused it also refused a good A, and it refused all
    # three losing patterns in signtest.py, which is why their `modal%` was 0.
    e = json.loads(json.dumps(d))
    e["a"]["value"][0] = [-x for x in e["a"]["value"][0]]
    pflip = tmp + ".flip"
    with open(pflip, "w") as fh:
        json.dump(e, fh)
    mflip = sl.Modal(pflip, N, F_MODE_HZ, MODAL_SCHEMA, MODAL_MIN_SENSORS,
                     MODAL_MIN_COILS, MODAL_COND_MAX, MODAL_MAX_AGE_S, False,
                     ALLOC_BALANCE, MODAL_DEMAND_CAP_V)
    check("a sign-flipped A LOADS and is logged loudly, not refused",
          mflip.ok and mflip.a_coils == [0, 1, 2, 3]
          and any("SIGN DISAGREES" in s for s in mflip.notes),
          "%d coil(s) flagged; signtest.py is the arbiter"
          % sum(1 for s in mflip.notes if "SIGN DISAGREES" in s))

    # ---- the Controller -----------------------------------------------------
    print()
    cdef = Controller(_NoDac())
    check("Controller's DEFAULT has the geometry but only the provisional DC A, so "
          "no modal filter runs and the suite stays diagonal",
          (not cdef.modal.ok) and cdef.mkf is None
          and cdef.modal.basis == PHI_BASIS, cdef.modal.why[:48])
    bad_doc = json.loads(json.dumps(d))
    bad_doc["schema"] = "not-this-one"
    pbad = tmp + ".refused"
    with open(pbad, "w") as fh:
        json.dump(bad_doc, fh)
    cbad = Controller(_NoDac(), modal_path=pbad)
    check("a REFUSED file leaves no modal filter to run on",
          (not cbad.modal.ok) and cbad.mkf is None, cbad.modal.why[:48])
    cmix = Controller(_NoDac(), modal_path=tmp)
    check("...and so does an A from another basis, whatever else is right",
          (not cmix.modal.ok) and cmix.mkf is None and "basis" in cmix.modal.why,
          cmix.modal.why[:48])
    cok = Controller(_NoDac(), modal_path=pgeo)
    check("an ACCEPTED A builds one, on the GEOMETRIC Phi",
          cok.modal.ok and cok.mkf is not None
          and np.allclose(cok.mkf.phi, cok.modal.phi)
          and np.allclose(cok.modal.phi[:4], PHI_GEOM[:4] / 2.0),
          "%d states, Phi columns unit-norm" % (cok.mkf.nx if cok.mkf else 0))
    # a5 came back on 2026-08-20 (std 9.16 counts, 36 distinct values, where it
    # had been exactly 0.0 with zero variance over 236 387 samples). So it is
    # enabled as a SENSOR. What must stay true is that it never actuates: its own
    # coil does not move it (-0.39 +-0.93 counts/V, 0.4 sigma), and its coherent
    # fraction has not been re-measured since the recovery.
    check("a5 is enabled as a sensor but still cannot actuate -- coherent 0.70, "
          "yet its own coil moves it only 0.4 sigma, so the loop is NOT colocated",
          ENABLE_CHANNEL[5] and not HYBRID_CHANNEL[5]
          and HYBRID_COHERENT[5] == 0.70 and HYBRID_GAIN[5] == 0.0
          and STEADY_GAIN[5] == 0.0 and CAPTURE_GAIN[5] == 0.0,
          "enabled %s, hybrid %s, coherent %.2f but gain %.5f (masked), "
          "steady %+.4f"
          % (ENABLE_CHANNEL[5], HYBRID_CHANNEL[5], HYBRID_COHERENT[5],
             HYBRID_GAIN[5], STEADY_GAIN[5]))

    # ---- dead pins, as measured on the rig ----------------------------------
    print()

    def drive(counts_fn, seconds=3.0, wire_hz=1100.0, ctl=None):
        c = Controller(_NoDac()) if ctl is None else ctl
        dt = 1.0 / wire_hz
        t0 = 0.0 if ctl is None else drive.t
        for k in range(int(seconds * wire_hz)):
            tt = t0 + k * dt
            cts = np.clip(np.round(counts_fn(tt)), 0, ADC_MAX_COUNTS).astype(float)
            c.step(cts, cts * (A_VCC / ADC_MAX_COUNTS), tt, dt)
        drive.t = t0 + int(seconds * wire_hz) * dt
        c.drain_events()
        return c

    drive.t = 0.0
    rngp = np.random.default_rng(20260817)
    # Resting counts as measured 2026-08-17 17:59; a5 at 0.0, a6/a7 bottom-railed.
    REST = np.array([680.3, 585.4, 636.2, 699.6, 558.6, 0.0, 4.3, 8.8])

    def as_measured(t):
        c = REST.copy()
        c[:5] += (50.0 * np.sin(2.0 * np.pi * F_MODE_HZ[0] * t)
                  + 5.0 * rngp.normal(size=5))
        c[6] += 0.563 * rngp.normal()
        c[7] += 0.476 * rngp.normal()
        return c

    c1 = drive(as_measured)
    check("a6/a7 bottom-railed AS MEASURED do not fault the rig",
          c1.fault_count == 0 and c1.state == "CALIBRATING"
          and bool(c1._pin_said[6] and c1._pin_said[7]),
          "std %s counts, demoted %s, %d fault(s), state %s"
          % (np.array2string(c1.rail_std[[6, 7]], precision=3),
             Controller._who(c1._pin_said) or "nothing", c1.fault_count, c1.state))
    W = np.asarray([e[2] for e in c1.health.rail_hist], float)
    check("the running-sum std matches a direct one over the same window",
          np.allclose(c1.rail_std, W.std(axis=0), atol=1e-9),
          "max |diff| %.2e over %d samples"
          % (float(np.abs(c1.rail_std - W.std(axis=0)).max()), len(W)))

    def railed_but_moving(t):
        c = as_measured(t)
        c[6] = 3.0 if (t % 0.2) < 0.17 else 400.0
        return c

    drive.t = 0.0
    c2 = drive(railed_but_moving)
    check("a railed channel that is still MOVING still faults the rig",
          c2.fault_count > 0 and c2.state == "FAULT" and not c2._pin_said[6],
          "std %.1f counts against the %.2f line, %d fault(s), state %s"
          % (c2.rail_std[6], DEAD_PIN_STD_COUNTS, c2.fault_count, c2.state))
    drive.t = 0.0
    c3 = drive(lambda t: np.zeros(N))
    check("every enabled channel railed and motionless still faults",
          c3.fault_count > 0 and not c3._pin_said.any() and c3._pin_none_said,
          "%d fault(s), %d demotion(s), no-witness announced %s"
          % (c3.fault_count, int(c3._pin_said.sum()), c3._pin_none_said))
    check("the dead-pin line sits between the two measured populations",
          0.853 < DEAD_PIN_STD_COUNTS < 1.998,
          "%.2f counts, against 0.853 (loudest dead window measured, a7) and 1.998 "
          "(quietest railed-and-moving window in evidence)" % DEAD_PIN_STD_COUNTS)

    # ---- the in-band floor --------------------------------------------------
    print()
    # a0-a3 carry a loud 6.19 Hz line and real in-band motion; a4 carries a real
    # in-band signal 30x smaller and NO line. Total amplitude demotes a4; in-band
    # keeps it. That is the 2026-08-17 case: a4 graded GOOD for in-band SNR and
    # was demoted anyway.
    tt = np.arange(0.0, 20.0, CONTROL_PERIOD_S)
    tot = sl.RMSBank(1e9, N)
    ib = sl.InBand(F_MODE_HZ, N)
    for t in tt:
        y = np.full(N, 3.3)
        y[:4] += (0.030 * np.sin(2 * np.pi * F_MODE_HZ[1] * t)
                  + 0.400 * np.sin(2 * np.pi * 6.19 * t))
        y[4] += 0.030 * np.sin(2 * np.pi * F_MODE_HZ[1] * t + 0.4)
        y[6:] += 0.400 * np.sin(2 * np.pi * 6.19 * t)
        ib.add(t, y)
        total = tot.update(t, y - 3.3, np.ones(N, bool))
    en = np.array([True] * 5 + [False, True, True])
    hh = sl.Health(N, RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS, RAIL_SUSTAIN_S,
                   RAIL_FRACTION, SAT_SUSTAIN_S, SAT_FRACTION,
                   DEAD_PIN_STD_COUNTS, REARM_SUSTAIN_S, BASELINE_FLOOR_FRAC)
    bad_tot, med_tot, _ = hh.floor(total, en)
    bad_ib, med_ib, _ = hh.floor(ib.rms(), en)
    check("TOTAL amplitude demotes a4 -- a real in-band channel next to loud "
          "out-of-band neighbours",
          bad_tot[4], "a4 %.4fV = %.1f%% of a %.4fV median"
          % (total[4], 100 * total[4] / med_tot, med_tot))
    check("IN-BAND amplitude keeps it, and still demotes the blind a6/a7",
          (not bad_ib[4]) and bad_ib[6] and bad_ib[7],
          "a4 %.5fV = %.0f%% of a %.5fV median; a6 %.1f%%, a7 %.1f%%"
          % (ib.rms()[4], 100 * ib.rms()[4] / med_ib, med_ib,
             100 * ib.rms()[6] / med_ib, 100 * ib.rms()[7] / med_ib))
    check("the floor keeps the same 10% relative criterion and the witness rule",
          hh.floor_frac == BASELINE_FLOOR_FRAC == 0.10
          and not hh.floor(np.zeros(N), en)[0].any(),
          "%.0f%% of the median" % (100 * BASELINE_FLOOR_FRAC))

    # ---- the fault-clear deadlock -------------------------------------------
    print()
    # A large transient trips the runaway breaker, then decays. The measurement
    # must keep running while the actuators are frozen, or the statistic the
    # clear is gated on stays at exactly the value that tripped it.
    def transient(decay):
        rg = np.random.default_rng(31337)
        base_c = np.array([600.0, 620.0, 640.0, 660.0, 580.0, 0.0, 590.0, 570.0])

        # THE QUIET PHASE MUST OUTLAST THE ARMING, and until 2026-08-20 it did
        # not. CALIBRATION_S is 20 s and `Breaker.arm` is blind for a further
        # ENVELOPE_WINDOW_S, so nothing before t = 22 s can latch; a ramp starting
        # at 16 s put only 4 s of its 10 s growth in front of an armed breaker and
        # tripped a 4.0 s sustain with ZERO margin -- the same zero margin
        # CLAUDE.md 1 is about, in the fixture rather than on the rig. The ramp now
        # starts at 26 s, which leaves 10 s of growth against a 5.0 s sustain.
        def fn(t):
            c = base_c.copy()
            if t < 26.0:
                amp = 3.0                                     # quiet: calibrates
            elif t < 36.0:
                amp = 3.0 * np.exp(0.55 * (t - 26.0))         # grows: trips
            else:
                amp = (720.0 * np.exp(-0.7 * (t - 36.0)) if decay else 720.0)
            c[:5] += amp * np.sin(2.0 * np.pi * F_MODE_HZ[1] * t) + 1.0 * rg.normal(size=5)
            c[6] += 1.0 * rg.normal()
            c[7] += 1.0 * rg.normal()
            return np.clip(c, 0, ADC_MAX_COUNTS)
        return fn

    def run_transient(decay, seconds=70.0, wire_hz=400.0):
        c = Controller(_NoDac())
        fn = transient(decay)
        dt = 1.0 / wire_hz
        seen_fault, frozen_ratio, cleared, said = False, [], None, []
        for k in range(int(seconds * wire_hz)):
            t = k * dt
            cts = fn(t)
            c.step(cts, cts * (A_VCC / ADC_MAX_COUNTS), t, dt)
            if c.state == "FAULT":
                if not seen_fault:
                    c.fault_at = t
                seen_fault = True
                frozen_ratio.append(float(c.ratio[0]))
            elif seen_fault and cleared is None:
                cleared = t
            said += c.drain_events()
        c.said = said
        return c, seen_fault, np.asarray(frozen_ratio), cleared

    cd, faulted_d, rat_d, cleared_d = run_transient(True)
    check("a large transient trips the breaker", faulted_d, "%d fault(s)" % cd.fault_count)
    check("the amplitude statistic KEEPS UPDATING while the actuators are frozen",
          rat_d.size > 50 and float(rat_d.max() - rat_d.min()) > 0.5,
          "ratio spanned %.2f..%.2f over %d frozen samples"
          % (rat_d.min() if rat_d.size else float("nan"),
             rat_d.max() if rat_d.size else float("nan"), rat_d.size))
    check("...so the fault CLEARS on its own once the motion decays",
          cleared_d is not None and cd.state in ("DAMPING", "CALIBRATING"),
          "cleared at t=%s, state %s"
          % ("never" if cleared_d is None else "%.1fs" % cleared_d, cd.state))
    # The AMPLITUDE gate must not open while the motion persists...
    cp, faulted_p, rat_p, cleared_p = run_transient(False, seconds=50.0)
    check("and does NOT clear on amplitude while the motion persists",
          faulted_p and cleared_p is None and cp.state == "FAULT",
          "state %s at 50s, ratio %.2f against the %.1f clear line"
          % (cp.state, float(cp.ratio[0]), FAULT_CLEAR_RATIO))
    # ...but the ceiling must, or one hand kick ends the run. Measured
    # 2026-08-18: FAULT ran 118.7 s and 94.5 s to the end of two runs, both under
    # the 120.0 s ceiling that was supposed to break exactly this deadlock.
    ch, faulted_h, _, cleared_h = run_transient(False, seconds=110.0)
    held = None if not faulted_h or cleared_h is None else \
        cleared_h - getattr(ch, "fault_at", 0.0)
    check("A FAULT THAT CANNOT CLEAR ON AMPLITUDE RE-ENGAGES AT THE CEILING",
          cleared_h is not None and held is not None
          and held <= FAULT_CLEAR_MAX_HOLD_S + FAULT_CLEAR_SUSTAIN_S + 2.0,
          "re-engaged %s after the trip, against a %.0fs ceiling plus a %.0fs "
          "sustain" % ("never" if held is None else "%.1fs" % held,
                       FAULT_CLEAR_MAX_HOLD_S, FAULT_CLEAR_SUSTAIN_S))
    check("...and says so, rather than re-engaging silently",
          any("held out of DAMPING" in s for s in ch.said),
          "%d event line(s)" % len(ch.said))
    check("the ceiling is short enough to fire inside a bench run at all",
          FAULT_CLEAR_MAX_HOLD_S + FAULT_CLEAR_SUSTAIN_S < 94.5,
          "%.0fs + %.0fs against the SHORTER of the two faults measured on "
          "2026-08-18 (94.5s and 118.7s, both under the old 120s ceiling)"
          % (FAULT_CLEAR_MAX_HOLD_S, FAULT_CLEAR_SUSTAIN_S))
    # ch5 sat at 0.0 counts and held FAULT for 1085 s. That used to be prevented
    # by ENABLE_CHANNEL[5] = False, which is no longer available -- a5 came back
    # on 2026-08-20 and is enabled. The protection now has to come from `dead-pin`
    # demoting it on VARIANCE, which is the general fix and does not depend on
    # anyone having disabled the right channel in advance.
    check("A RAILED, MOTIONLESS PIN CANNOT VETO THE CLEAR -- on variance now, "
          "not on ENABLE_CHANNEL, because a5 is enabled again",
          bool(cd.rail[5]) and ENABLE_CHANNEL[5] and cleared_d is not None,
          "ch5 railed %s, enabled %s, cleared %s"
          % (bool(cd.rail[5]), ENABLE_CHANNEL[5],
             "never" if cleared_d is None else "%.1fs" % cleared_d))

    # ---- THE ACCEPTANCE TEST: one hand kick must not end a run --------------
    print()
    # Both runs of 2026-08-18 died on the FIRST kick, under both laws, and jerk.py
    # now discards every fit taken outside DAMPING -- so a breaker that trips on a
    # kick does not merely bias the result, it returns zero valid kicks and wastes
    # the session. Numbers are the measured ones: quiet at ratio 0.117 (modal) and
    # 1.24 (diagonal), kicked to 4.73, decaying at the measured re-quiet rates.
    def kick_run(quiet_ratio, peak_ratio, decay, seconds=90.0, wire_hz=400.0,
                 t_kick=30.0):
        rg = np.random.default_rng(4711)
        # Mid-scale, because the modal excursion is 40x: off-centre resting points
        # would rail at the peak and the rail interlock, not the breaker, would
        # answer. a5 stays at ground -- it is a dead pin and must stay demoted.
        base_c = np.array([511.5] * 5 + [0.0] + [511.5] * 2)
        c, dt = Controller(_NoDac()), 1.0 / wire_hz
        # The EXCURSION is what the breaker sees, and it is the measured quantity:
        # 0.117 -> 4.73 is 40.4x under the modal law, 1.240 -> 4.73 is 3.8x under
        # the diagonal. Stepping the amplitude at engage instead would model the
        # loop changing the motion instantly, which it does not, and the step
        # itself reads as sustained growth.
        # Beat DERIVED, not typed: F_MODE_HZ moved twice on 2026-08-20 and a
        # hand-copied 0.26999 would have gone on rippling at the old spacing.
        beat = float(F_MODE_HZ[1] - F_MODE_HZ[0])
        a_cal, states, ex = 10.0, set(), peak_ratio / quiet_ratio
        for k in range(int(seconds * wire_hz)):
            t = k * dt
            if t < t_kick:
                amp = a_cal
            else:
                # Ring up over ONE PERIOD of the dominant mode -- what an impulse
                # does to a resonator -- then decay at the measured re-quiet rate,
                # with the beat ripple three modes produce.
                s, rise = t - t_kick, 1.0 / F_MODE_HZ[1]
                env = (min(1.0, s / rise) if s < rise
                       else np.exp(-decay * (s - rise)))
                amp = a_cal * (1.0 + (ex - 1.0) * env)
                amp *= 1.0 + 0.10 * np.sin(2 * np.pi * beat * s)
            cts, band = base_c.copy(), amp * np.sin(2.0 * np.pi * F_MODE_HZ[1] * t)
            cts[:4] += band + 1.0 * rg.normal(size=4)
            # The kick goes to the sensors that CARRY the modes, which is where
            # both measured trips happened (ch0/ch2 modal, ch3 diagonal). The
            # orthogonal axes carry their own broadband motion so they survive the
            # floor and are driven, but no in-band content -- this check is about
            # the breaker, not about them.
            # A RISK THIS EXPOSES AND DOES NOT COVER: with in-band content added to
            # a4/a6/a7 as well, ch6 trips the breaker about 1 s after the kick, and
            # its latch starts BEFORE the kick -- its `bp` is still converging
            # because KALMAN_K's row 6 is fitted to a6's own (tiny) measured noise.
            # Keeping those channels healthy is new, so nothing on the bench has
            # ever exercised it. Watch for a ch4/ch6/ch7 runaway on the next run.
            for j in (4, 6, 7):
                cts[j] += 5.0 * rg.normal()
            c.step(np.clip(cts, 0, ADC_MAX_COUNTS),
                   np.clip(cts, 0, ADC_MAX_COUNTS) * (A_VCC / ADC_MAX_COUNTS), t, dt)
            if t > t_kick:
                states.add(c.state)
            c.drain_events()
        return c, states

    for qr, dk, law in ((0.117, 0.1393, "modal"), (1.240, 0.0278, "diagonal")):
        ck, st = kick_run(qr, 4.73, dk)
        check("ONE KICK 0.117->4.73 DOES NOT END THE RUN (%s law)" % law,
              ck.fault_count == 0 and st == {"DAMPING"},
              "quiet ratio %.3f, peak 4.73, decay %.4f /s: %d fault(s), states "
              "after the kick %s" % (qr, dk, ck.fault_count, sorted(st)))
    check("every driven channel stays in the breaker's evidence set",
          bool(ck._driven()[6]),
          "narrowing it to the lock quorum loses the pumped-resonance detection "
          "(peak ch2 ratio 2.58 against the 1.8 line, 0 faults) -- tried, reverted")

    # ---- the running quiet level, in the loop -------------------------------
    print()
    cq0 = Controller(_NoDac())
    check("with no running-quiet estimate the interlocks use the BASELINE exactly",
          cq0.quiet.level() is None
          and np.array_equal(cq0._amp_ref(), cq0.baseline),
          "blind at t=0, so behaviour is bit-identical to what shipped")
    cqr, _ = kick_run(1.240, 4.73, 0.0278, seconds=90.0)
    lv = cqr.quiet.level()
    check("...and it arms during a normal run, at the level the loop actually held",
          lv is not None and cqr.quiet.ready(),
          "%s V" % ("none" if lv is None else np.array2string(lv[:4], precision=4)))
    check("...and the reference is never TIGHTER than the baseline it replaces",
          bool((cqr._amp_ref() >= cqr.baseline - 1e-15).all()),
          "max(baseline, quiet), so no line moves down")
    check("the gain schedule and LOCKED keep the ZERO-GAIN baseline, on purpose",
          "self.baseline * LOCK_RMS_FACTOR" in _SRC
          and "/ self.baseline" in _SRC,
          "a lock against the loop's own floor would be true by construction")

    # ---- the total cap, not the allocation's own cap -------------------------
    print()
    big_d = sl.cap_scale(np.array([0.05, -0.19, 0.12, 0.02]),
                         np.array([0.01, -0.02, 0.22, 0.00]),
                         MODAL_TOTAL_HEADROOM * BIAS_SWING,
                         np.ones(4, bool))
    tot = np.abs(big_d * np.array([0.05, -0.19, 0.12, 0.02])
                 + np.array([0.01, -0.02, 0.22, 0.00])).max()
    check("the TOTAL demand is capped, D term included -- coil 3 reached 0.4229 V",
          tot <= MODAL_TOTAL_HEADROOM * BIAS_SWING + 1e-12,
          "%.4f V against %.3f V (%.0f%% of the %.2f V half-window)"
          % (tot, MODAL_TOTAL_HEADROOM * BIAS_SWING,
             100 * tot / BIAS_SWING, BIAS_SWING))
    check("...and the cap is measured on the TOTAL, not on the allocation alone",
          "self.pid.i + self.pid.d" in _SRC and "cap_scale" in _SRC,
          "MODAL_DEMAND_CAP_V %.2f V bounds the allocation; this bounds the sum"
          % MODAL_DEMAND_CAP_V)

    # ---- the coils must be parked on a plain kill ---------------------------
    print()
    import signal as _sig
    _old = [(_s, _sig.getsignal(_s)) for _s in
            (getattr(_sig, "SIGTERM", None), getattr(_sig, "SIGHUP", None)) if _s]
    got = _park_on_signals()
    raised = False
    try:
        _sig.getsignal(_sig.SIGTERM)(_sig.SIGTERM, None)
    except KeyboardInterrupt:
        raised = True
    for _s, _h in _old:
        try:
            _sig.signal(_s, _h)
        except (ValueError, OSError, TypeError):
            pass
    check("SIGTERM and SIGHUP route to the same park path as Ctrl+C",
          "SIGTERM" in got and raised,
          "installed %s; the handler raises KeyboardInterrupt, so the `finally` "
          "runs and the coils go back to bias" % ", ".join(got))
    check("...and the park is the FIRST teardown step, so nothing can skip it",
          _SRC.index("_step(\"park the coils\"") < _SRC.index("_step(\"stop the stream\""),
          "stop_stream raises on its 5 s deadline; it used to run first and take "
          "the park with it")

    # ---- LOCKED, which had never been printed -------------------------------
    print()
    # ch4 is enabled, healthy and has ZERO steady and capture gain, so nothing
    # drives it and it can never meet the lock ratio. Before `lock-quorum` that
    # one channel vetoed the rig for the whole run, across three versions.
    quiet_c = np.array([600.0, 620.0, 640.0, 660.0, 580.0, 0.0, 590.0, 570.0])

    def quiet_run(seconds=48.0, wire_hz=400.0):
        # Constant while it calibrates, then decaying, so `ratio` can actually fall
        # under LOCK_RMS_FACTOR -- the counts here do not respond to the coils.
        rg = np.random.default_rng(77)
        c = Controller(_NoDac())
        dt = 1.0 / wire_hz
        for k in range(int(seconds * wire_hz)):
            t = k * dt
            amp = 8.0 if t < 16.0 else 8.0 * np.exp(-0.4 * (t - 16.0))
            cts = quiet_c.copy()
            cts[:4] += amp * np.sin(2.0 * np.pi * F_MODE_HZ[1] * t) + 1.0 * rg.normal(size=4)
            # a4 keeps moving whatever the coils do: over half of its in-band motion
            # is not the optic, so its ratio never comes down and it never locks.
            cts[4] += (8.0 * np.sin(2.0 * np.pi * F_MODE_HZ[0] * t + 1.3)
                       + 1.0 * rg.normal())
            cts[6] += 1.0 * rg.normal()
            cts[7] += 1.0 * rg.normal()
            c.step(cts, cts * (A_VCC / ADC_MAX_COUNTS), t, dt)
            c.drain_events()
        return c

    cq = quiet_run()
    outside = cq.enabled & cq.healthy & ~cq._quorum()
    check("a channel with no vote is enabled, healthy and NOT locked",
          bool(outside.any()) and bool((~cq.locked[outside]).any()),
          "%s outside the quorum, locked %s"
          % (Controller._who(outside), cq.locked[outside].tolist()))
    check("...and cannot prevent LOCKED: only driven, coherent channels vote",
          cq.lock_time is not None and cq.locked_announced,
          "LOCKED at %s" % ("never" if cq.lock_time is None
                            else "%.1fs after gain" % cq.lock_time))
    check("a4 survives the in-band floor and IS driven, at the scaled gain",
          (not cq.floor_bad[4]) and abs(cq.gain[4] - HYBRID_GAIN[4]) < 1e-6
          and abs(cq.gain[4]) < 0.6 * HYBRID_KP,
          "gain %+.5f against a full %+.5f (coherent fraction %.2f)"
          % (cq.gain[4], HYBRID_KP * SLOPE_SIGN[4], HYBRID_COHERENT[4]))
    check("...but does not vote: coherent fraction %.2f under the %.2f line"
          % (HYBRID_COHERENT[4], LOCK_COHERENT_MIN),
          bool(cq._driven()[4]) and not bool(cq._quorum()[4]))
    check("a5 never actuates -- coherent fraction 0, and its pin is at ground",
          cq.gain[5] == 0.0 and abs(cq.out[5] - cq.bias[5]) < 1e-12,
          "gain %.5f, out %.4f V against a %.4f V bias"
          % (cq.gain[5], cq.out[5], cq.bias[5]))

    # ---- the hybrid channels must survive a band they cannot carry ----------
    print()
    # Measured 2026-08-18: a4/a6/a7 were floor-demoted at engage in BOTH runs
    # ("in-band 0.0027 / 0.0000 / 0.0026 / 0.0024V, under 10% of the 0.0834V
    # median") and their |gain| was exactly 0.0000 for 100 % of every DAMPING
    # sample, so the hybrid law has never once run. Their axes are orthogonal to
    # the three damped modes, so a small in-band amplitude is what they are
    # SUPPOSED to have; their bias-corrected coherence with the optic, re-measured
    # 2026-08-20, is 0.09 / 0.70 / 0.12 / 0.05 for a4/a5/a6/a7 -- it was
    # 0.46 / 0.26 / 0.50 for a4/a6/a7 before the coils were re-seated.
    check("the exemption is derived from Phi, not listed by hand",
          list(np.nonzero(INBAND_EXEMPT)[0]) == [4, 5, 6, 7]
          and not INBAND_EXEMPT[:4].any(),
          "channels with no Phi row: %s" % list(np.nonzero(INBAND_EXEMPT)[0]))

    def floor_run(motion, seconds=26.0, wire_hz=400.0):
        """Calibrate with a0-a3 carrying the band and a4/a6/a7 carrying `motion`."""
        rg = np.random.default_rng(2026)
        base_c = np.array([600.0, 620.0, 640.0, 660.0, 580.0, 0.0, 590.0, 570.0])
        c, dt = Controller(_NoDac()), 1.0 / wire_hz
        for k in range(int(seconds * wire_hz)):
            t = k * dt
            cts = base_c.copy()
            cts[:4] += 30.0 * np.sin(2 * np.pi * F_MODE_HZ[1] * t) + rg.normal(size=4)
            # orthogonal axes: real broadband motion, but almost nothing in band
            for j in (4, 6, 7):
                cts[j] += motion * rg.normal()
            c.step(np.clip(cts, 0, ADC_MAX_COUNTS),
                   np.clip(cts, 0, ADC_MAX_COUNTS) * (A_VCC / ADC_MAX_COUNTS), t, dt)
            c.drain_events()
        return c

    live_c = floor_run(30.0)      # a4/a6/a7 move as much as a0-a3 do
    check("A CHANNEL WITH LOW IN-BAND BUT REAL MOTION IS KEPT -- the hybrid can run",
          not live_c.floor_bad[4] and not live_c.floor_bad[6]
          and not live_c.floor_bad[7],
          "a4/a6/a7 in-band %.4f/%.4f/%.4fV, but raw std %.1f/%.1f/%.1f counts "
          "against a %.1f-count median on a0-a3"
          % (live_c.inband_v[4], live_c.inband_v[6], live_c.inband_v[7],
             live_c.health.rail_std[4], live_c.health.rail_std[6],
             live_c.health.rail_std[7],
             float(np.median(live_c.health.rail_std[:4]))))
    check("...and it is actually DRIVEN, which is the point of keeping it",
          abs(live_c.gain[4] - HYBRID_GAIN[4]) < 1e-6 and HYBRID_GAIN[4] != 0.0,
          "gain %+.5f at coherent fraction %.2f" % (live_c.gain[4],
                                                    HYBRID_COHERENT[4]))
    dead_c = floor_run(0.0)       # and a genuinely dead pin still goes
    check("...while a channel that does not move at all IS demoted: a5 is real",
          dead_c.floor_bad[4] and dead_c.floor_bad[6] and dead_c.floor_bad[7]
          and dead_c.gain[4] == 0.0,
          "raw std %.2f/%.2f/%.2f counts, under the %.2f-count dead-pin line"
          % (dead_c.health.rail_std[4], dead_c.health.rail_std[6],
             dead_c.health.rail_std[7], DEAD_PIN_STD_COUNTS))
    dith_c = floor_run(1.5)       # dither only: over the pin line, far under a0-a3
    check("...and so is one that returns only DITHER -- an absolute line misses it",
          dith_c.floor_bad[4] and dith_c.floor_bad[6] and dith_c.floor_bad[7],
          "raw std %.2f counts, over the %.2f-count pin line but under %.0f%% of "
          "the %.1f-count median that a0-a3 move"
          % (dith_c.health.rail_std[4], DEAD_PIN_STD_COUNTS,
             100 * BASELINE_FLOOR_FRAC,
             float(np.median(dith_c.health.rail_std[:4]))))
    check("a0-a3 are still judged on the band, not on whether they merely move",
          not live_c.floor_bad[:4].any(),
          "the exemption is scoped to rows Phi does not span")

    print("\n  %s\n" % (
        "ALL PASS -- Phi comes from GEOMETRY: its columns are orthogonal, warp is in\n"
        "  their null space, the measured Phi only cross-checks it (|cos| 0.971 /\n"
        "  0.978 / 0.963 on the shipped file) and an A from another basis is refused\n"
        "  rather than silently paired,\n"
        "  the modal filter recovers a planted velocity with rows deleted,\n"
        "  chi2/dof is order 1 when the model fits and rises when a sensor is wrong,\n"
        "  the hybrid law is dissipative at the coherence-scaled gain and reduces to\n"
        "  the pure modal law when A covers every coil, the in-band floor keeps a real\n"
        "  channel that total amplitude threw away, a frozen fault clears on decaying\n"
        "  motion and holds on persistent motion, and a channel with no vote can no\n"
        "  longer veto LOCKED.\n"
        "  AND THE PER-CHANNEL GAINS ARE NOT FLAT: they are scaled by the measured\n"
        "  sensor gain h, which is fitted from a constraint a rigid plate cannot\n"
        "  violate, and sym(A diag(g) Phi) is asserted POSITIVE DEFINITE in both the\n"
        "  counts and the physical form -- so no mode is pumped, checked numerically\n"
        "  rather than assumed from a colocation that does not hold here.\n"
        "  NOT tested here, and only the bench can: whether it damps the optic, what\n"
        "  chi2/dof distributes as on this rig, which sign pattern A needs, and\n"
        "  whether the hybrid improves the measured DECAY RATE -- jerk.py is that\n"
        "  test, and `ratio` is not."
        if ok else
        "FAILURES ABOVE -- do not run this on the bench."))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
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
    got = _park_on_signals()
    if got:
        print("  %s routed to the same park path as Ctrl+C -- a plain `kill` now "
              "returns the coils to bias instead of leaving them energised."
              % " and ".join(got))
    fresh = ("--fresh" in argv
             or os.environ.get("OSEM_FRESH_CALIB", "0") not in ("", "0"))
    loaded, report = load_baseline(BASELINE_PATH, fresh=fresh)
    print()
    for line in report:
        print(line)
    print()

    for line in gain_report():
        print(line)
    print()

    pre = _modal(MODAL_PATH)
    for line in pre.report(MODAL_KP, MODAL_GAIN_SCALE):
        print(line)
    if pre.ok:
        for line in sl.ModalKalman(pre.phi, pre.rowok, KALMAN_R, KALMAN_Q,
                                   KALMAN_Q_DC, F_MODE_HZ, CONTROL_PERIOD_S, N,
                                   T_AMP_S).report():
            print(line)
        outside = [j for j in range(N) if j not in pre.a_coils and HYBRID_CHANNEL[j]]
        print("[hybrid] %s: modal on coils %s, diagonal velocity feedback on %s"
              % ("ON" if HYBRID_DIAGONAL else "OFF", pre.a_coils, outside or "nothing")
              + (" (their STEADY_GAIN is %s -- armed and silent until a bench run "
                 "measures a sign)" % np.round(STEADY_GAIN[outside], 4)
                 if outside else ""))
    print()

    dac = sl.open_dac(PORT, binary=BINARY_TRANSPORT)
    sl.park(dac, DAC_CHANNELS, BIAS)
    if sys.stdin.isatty():
        input("DAC biases set. Press Enter to start fast-lock damping "
              "(Ctrl+C to stop)... ")
    else:
        print("DAC biases set. stdin is not a tty -- starting without the prompt.")
    dac.start_stream()
    ctl = Controller(dac, baseline_file=loaded, modal_path=MODAL_PATH)

    os.makedirs("data", exist_ok=True)
    path = os.path.join("data",
                        datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    log = sl.Recorder(path, CSV_COLS, CSV_FLUSH_EVERY_N)
    print(f"Logging to {path}, every raw sample, control step at {CONTROL_HZ:.0f} Hz "
          f"(`ctl`/`n_avg` mark which samples each step averaged).\n\n"
          + (f"[{ctl.state}] warming up for {BASELINE_WARMUP_S:.1f}s and then checking "
             f"the stored floor against it...\n" if loaded is not None else
             f"[{ctl.state}] measuring baseline noise (outputs held at bias, no "
             f"damping yet). This stops as soon as the floor settles, and after "
             f"{CALIBRATION_S:.0f}s at the latest...\n"))

    start = prev = time.time()
    console = sl.Console(STATUS_PERIOD_S)
    next_cue = None                 # armed on the first DAMPING sample, not here
    if KICK_CUE_S > 0:
        print(f"[kick-cue] speaking \"{KICK_CUE_PHRASE}\" every {KICK_CUE_S:.0f}s once "
              f"damping starts. Unset OSEM_KICK_CUE to silence it.\n")
    if ctl.kick is not None:
        for _line in ctl.kick.banner():
            print(_line)
        print("[self-kick] ATTENDED OR NOT, THIS DRIVES THE OPTIC ON PURPOSE. It "
              "fires only from DAMPING and only\n"
              "[self-kick] once the loop has held under its own quiet level; it "
              "stops on any FAULT and on a\n"
              "[self-kick] ringdown that does not come back. Unset OSEM_SELF_KICK "
              "to disable.\n")
    if GAIN_RAMP:
        lv, dw = GAIN_RAMP
        print(f"[gain-ramp] {len(lv)} levels, |Kp| {lv[0]:.4f} -> {lv[-1]:.4f}, "
              f"{dw:.0f}s each, about {len(lv) * dw / 60:.0f} min of DAMPING.\n"
              f"[gain-ramp] ATTENDED ONLY. This walks the loop toward instability "
              f"deliberately. Keep a scope on the coil drive and be ready to "
              f"Ctrl+C.\n")
    try:
        while True:
            sample = ctl.guard.read(dac)
            if sample is None:
                continue
            counts, volts = sample
            now = time.time()
            dt, prev = min(max(now - prev, 1e-4), 0.05), now
            t = now - start
            ctl.step(counts, volts, t, dt)
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
            if console.due(now):
                print(ctl.status_line(t))
            if KICK_CUE_S > 0:
                if next_cue is None:
                    if ctl.state == "DAMPING":
                        next_cue = t + _KICK_CUE_LEAD_S
                elif t >= next_cue:
                    next_cue += KICK_CUE_S
                    if ctl.state == "DAMPING":
                        kick_cue()
                        print(f"[kick-cue] t={t:.0f}s -- KICK NOW")
            log.write(ctl.csv_row(t, counts))
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # PARK FIRST, AND LET NOTHING BEFORE IT FAIL. The order used to be
        # stop_stream -> park, and `stop_stream` raises on its 5 s deadline: one
        # raise there skipped the park entirely and left every coil at whatever the
        # loop last commanded -- CLAUDE.md 15's outcome reached from a plain Ctrl+C.
        # The coils were parked by hand three times on 2026-08-17/18. Measured
        # 2026-08-18, `stop_stream` also survived 90 s of SIGINT to the process and
        # its group while blocked waiting for an ACK a busy board never sent.
        # Parking needs the port OPEN, so it comes before stop_stream and close;
        # `Actuator.send` writes the same `SET` mid-stream every control step, so
        # this is the proven path. EACH STEP IN ITS OWN try/except -- one wrapper
        # round all of them has the identical defect. Last-resort fallback, and NOT
        # an excuse for a broken teardown: arduino.ino zeroes all eight coils in
        # setup(), which runs when the port is opened.
        print("Returning DAC outputs to bias voltages...")
        _step("park the coils", sl.park, dac, DAC_CHANNELS, BIAS)
        _step("flush the log", log.close)
        _step("stop the stream", dac.stop_stream)
        _step("settle", time.sleep, 0.1)
        _step("close the port", dac.close)
        print(f"Log saved to {path}")
        print(f"{log.rows} raw samples, {ctl.ctl_steps} control steps "
              f"({log.rows / max(ctl.ctl_steps, 1):.1f} averaged per step), "
              f"wire {ctl.wire_hz:.0f} Hz.")
        if ctl.ramp is not None:
            print(ctl.ramp.summary())
        if ctl.kick is not None:
            print(ctl.kick.summary())
        if ctl.mkf is not None:
            print("\n" + "=" * 68)
            print("  chi2 PER DEGREE OF FREEDOM -- the out-of-mode check, calibrated")
            print("=" * 68)
            print("  Order 1 means the three-mode model explains the readings to")
            print("  within the MEASURED per-channel noise. NOTHING ACTED ON THIS.")
            for st in ("CALIBRATING", "DAMPING", "FAULT"):
                print(ctl.chi2_log[st].line(st))
            print("\n  A static per-sensor offset is INVISIBLE to this by design -- "
                  "each\n  sensor owns a DC state whose job is to absorb one. What it "
                  "sees is a\n  sensor whose dynamics disagree: wrong gain, wrong "
                  "sign, extra noise,\n  stuck, or a step while the DC state catches "
                  "up.")
            print("\n  Also on every CSV row (`chi2`, `ndof`), so the distribution is "
                  "on disk.")
            if ctl.mkf.n_singular:
                print("\n  !! S was singular on %d step(s). Those steps predicted and "
                      "did not\n     update. A climbing count means the measurement "
                      "model is degenerate." % ctl.mkf.n_singular)
            print()
        print(f"sample-guard: {ctl.guard.rejected} torn row(s) rejected at the wire, "
              f"{ctl.bad_samples} at the controller.")


if __name__ == "__main__":
    main()
