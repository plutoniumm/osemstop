"""
status.py -- what every sensor and every coil can actually do, and whether that
is enough to build a modal (MIMO) loop. It is the tool that produced Phi and A.

    status.py sensors     passive census: is each OSEM seeing the optic?
    status.py phi         Phi from AMBIENT motion -- no drive, and it runs on
                          quiet logs already on disk
    status.py coils       active census: does each coil move it, and where?
    status.py mimo        the gate: is Phi TALLER than it is wide?
    status.py pulse       one calibrated kick; drift against a stored signature
    status.py all         sensors + coils + mimo, one session

`sensors` and `coils` end in a graded table with the number that decided each
grade next to it. `--color never` for a file, `--color always` for `| less -R`;
the words are always there so a colourless paste still reads.

    sensor  GOOD  all three modes above SNR 8, headroom to be driven, and
                  coherence saying it watches the OPTIC
            OK    damps, but no Phi row -- too few modes, no headroom, or
                  ambiguous coupling
            DEAD  railed, flat, blind on the optic, or reading another channel

    coil    GOOD  all three modes above SNR 8, and a direction no other coil
                  already covers
            OK    drives something, but misses a mode, rails a sensor, or sits
                  within cosine 0.95 of another coil
            DEAD  moves nothing measurable at the amplitude it was given

WEAK IS NOT BAD. A coil at 1/20 gain that drives all three modes independently is
GOOD and needs volts, not repair. A loud coil parallel to its neighbour is OK and
no gain fixes it -- coils 4-7 measured +0.966 pairwise.

WHAT IT MEASURED, 2026-08-17
    Phi  4x3 on rows a0-a3, cond 2.44 raw / 1.63 unit-norm columns, null space
         dimension 1, and dropping any single row still leaves rank 3 (cond
         2.43-3.46). `data/20260817_182021_status_phi.csv`, 113 412 samples,
         300 s, passive. The four in-plane corner sensors span three rigid-body
         DOF plus one warp direction, and the warp IS the out-of-mode residual.
    A    coils 0-3, modal matrix rank 3 of 3, cond 2.98,
         `data/20260817_182701_status_coils.csv`. CAVEAT: that pass ran with the
         defective unwind below, off-quadrature 0.736 / 2.088 / 0.812 where
         resonance wants small, so A's PHASE is contaminated. Its SIGN survives.
         A cleaner 19:47 pass was refused by the colocation check on 2 of 4
         coils, which is now understood as the check being wrong, not the data.
    modes 0.72294 / 0.99193 / 1.65657 Hz, consensus of 5 sensors, spread
         0.0005 Hz -- up to 0.0170 Hz off the 2026-08-06 values. `MODES` below is
         only the SEED for the +-0.05 Hz fine scan; the tool reports the peak it
         finds.
Downstream: `signtest.py` scored that pair 0.08 median ratio against diagonal's
1.58, and `jerk.py` measured 0.1393 /s decay against diagonal's 0.0278.

THE SIGN OF A IS PHYSICS, NOT A FIT. On resonance the response lags the drive by
90 degrees, so with Phi rotated real, A = -Im(a * conj(rot_phi)). Verified against
this repo's own lock-in: Phi*A > 0 gives H = -1.2024i, Phi*A < 0 gives +1.2024i.
The version before this picked the per-mode sign by maximising colocation
consistency and PUMPED -- median ratio 1.5 diagonal -> 2.3 modal.

COLOCATION DOES NOT HOLD ON THIS RIG, so `A[m,j] = lambda_j * Phi[j,m]` is the
wrong identity and the "12 min instead of 36" shortcut it implied is void.
Reported by the rig owner: four coils point the same way, two are diametrically
opposed, three different planes. Evidence: two independent coil passes gave
different lambda signs (+,+,+,+ vs +,-,-,+), and a colocation-constrained rank-1
fit came out second/first singular value 0.67 where colocation predicts ~0. The
`DAC_MAP` heuristic below ("coil j dominates sensor j") is WEAK EVIDENCE, not a
wiring verdict: one coil moves the whole suspended body, and coil 1 gave a0 +37.2
and a1 +36.1 counts/V.

WHY A RAMP AND NOT A STEADY STATE. tau > 138 s (`analysis/ringdown.md`), so
on-resonance steady state is 5 tau > 11 min PER POINT. Unnecessary: an undamped
oscillator driven on resonance from rest grows LINEARLY,

    x(t) / x_static  =  pi * f * t          for t << tau,  capped at Q

so 30 s buys 67-155x with no waiting, and linear growth is what lets the rail
guard keep up. The ramp does not contaminate anything: `pi*f*T` is one scalar per
(coil, mode) common to all eight sensors, Phi is the ratio BETWEEN sensors inside
a point and A the ratio between coils at one dwell, so it cancels exactly.

SAFETY -- READ BEFORE RUNNING `coils` OR `pulse`. First tool in the repo that
drives ON a resonance of a Q > 433 plant.

1. AMPLITUDE IS DERIVED, NEVER TYPED: amp = TARGET_SWING_COUNTS /
   (dc_gain_seed * pi * f * DWELL_S), clamped to AMP_MAX. `--dry-run` prints
   every amplitude before anything moves.
2. THE RAIL GUARD UNWINDS, IT DOES NOT STOP. Stopping at 950 counts leaves the
   optic ringing at 950 counts for ten minutes. The drive inverts on the same
   continuous time base and the point is retried at half amplitude.
3. EVERY POINT UNWINDS AND IS CHECKED QUIET, or the next point measures the
   previous point's ringing.
4. Output stays inside VMIN..VMAX around each channel's own BIAS_CH, and
   `plan` prints AMP_MAX against the narrowest half-window those leave.
5. Nothing damps while this runs. All eight coils park on exit: Ctrl+C, SIGTERM
   and SIGHUP all route through the same unwind (see `main`).

THE UNWIND, AND THE TWO BUGS IN IT, both measured 2026-08-17 and both fixed.
    * RETRYING THE UNWIND PUMPS. There is no feedback in it: the same command is
      reissued against a state now near rest, i.e. a fresh excitation. Residuals
      grew 0.188 -> 0.277 -> 0.422 V against a 0.259 V peak. `_point` now
      unwinds ONCE, then measures and REPORTS the residual per point.
    * THE COMPARISON WAS BROKEN. `peak` was a lock-in over the WHOLE ramping
      drive, which averages to about half the end-of-drive amplitude, so
      `resid <= QUIET_FRAC*peak` was really half that and failed points that
      had cancelled fine. `peak` is now measured over the last QUIET_S of the
      drive, the same window and estimator as `resid`.
    The next pass ran with ZERO retries.

THE PULSE fires one Hann impulse on one coil, watches the free ring, and reduces
it to a fingerprint: complex gain per (sensor, mode) plus resting counts.
`--save-signature` stores it; later runs diff gain in dB, phase in degrees, DC in
counts. It divides by the LOCK-IN OF THE RECORDED DRIVE, not the intended
amplitude, so shape, clamping and update rate cancel and in-run equals offline.
Repeated pulses SUBTRACT the residual rather than waiting it out (tau > 138 s, so
pulse 2 lands on pulse 1's ring): the pre-window is locked in on the same time
origin, so the residual is a known phasor, decayed by the measured per-mode gamma.
A pulse whose pre-level exceeds PULSE_PRE_MAX_FRAC of the expected ring is
refused. It CANNOT catch a drift common to drive and sensor -- it is a ratio.

`--selftest` asserts PLANTED properties, not whatever the code printed, and
writing it found two real defects:
  * `mimo_gate`'s sigma on Phi was `sqrt(sum sigma^2)/ncoils`, optimistic by
    sqrt(ncoils) = 2.83, and declared a row "determined at 4.1 sigma" that had
    been planted at EXACTLY zero. Now var(phi_i) = sum_j sigma_ij^2 |a_j|^2.
  * a6 graded OK on a marginal SNR of 4.6 while its in-band coherence with every
    sensor on the optic sat at the bias floor. Coherence now outranks the lock-in.
A third, found on the bench: `railed_frac` counted a5's constant 0.0 counts as a
rail, so all four coils reported "railed a5" and graded OK with nothing wrong with
them. Channels outside the guard band at rest are now excluded, as in `guard_mask`.

Every mode streams raw counts to `data/` as they arrive and replays from that file
with `--replay`, so nothing here is one-shot.

    python status.py sensors                      90 s, no drive, safe
    python status.py phi --seconds 300            Phi needs >= 192 s undisturbed
    python status.py coils --dry-run              the plan and the amplitudes
    python status.py coils --coils 0,1,2,3 --save-modal    -> data/modal.json
    python status.py mimo --replay data/<f>.csv   the gate, offline
    python status.py pulse --save-signature       store today's fingerprint
    python status.py --selftest                   synthetic plant, no bench
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Last-resort fallback only -- `resolve_port` reuses bench.py's chooser. A
# hardcoded port pinned `make status` to whatever tty the board enumerated as on
# 2026-08-15, which was a different board.
PORT = "/dev/cu.usbserial-1120"
NCOLS = 8

A_VCC, ADC_MAX_COUNTS = 5.02, 1023
COUNTS_TO_V = A_VCC / ADC_MAX_COUNTS
MID_COUNTS = 511.5

# The nominal scalar bias, used only where no coil is in scope (a passive
# segment's recorded `u`, and `slopesign.py`'s hi/lo label). What is APPLIED is
# BIAS_CH, per channel, defined below.
#
# IT IS DERIVED FROM BIAS_CH AND MUST NOT BE A SEPARATE LITERAL. It was 0.25
# while BIAS_CH held 0.50, so every passive segment logged `u_v = 0.25` against
# hardware sitting at 0.50 -- a recorded drive column that disagreed with the
# rig by 2x, silently, in the file the analysis reads back. Found 2026-08-20 by
# an audit, not by anything failing. Same bug class as the hand-copied mode
# frequencies in the census headers, which were also wrong.
# The nominal is the value MOST coils sit at, not the mean: coil 2 is
# deliberately offset (see BIAS_CH) and must not drag the nominal with it.
BIAS_V = 0.50

# PER-CHANNEL BIAS, 2026-08-20, after the coils were re-seated. The bias is a
# static force, so it sets where each flag sits in its OSEM's shadow, and an
# OSEM is only linear near half-shadow (MID_COUNTS).
#
# 0.50 V is `osem.eta.py`'s BIAS: a coil pass measures A at whatever operating
# point it runs at, so measuring at a bias the controller never uses produces an
# A the controller cannot use.
#
# EXACT CENTRING IS NOT REACHABLE AT ANY VOLTAGE. Solving
# `min ||M v - (MID_COUNTS - resting)||^2` over the DC matrix M measured by
# `slopesign.py` (`data/20260820_155638_status_slopesign.csv`) asks for -6.87 to
# +7.50 V: M has cond 72.2 (singular values 309.1 / 117.6 / 60.4 / 4.3) and the
# direction that would centre all four is essentially WARP, which no rigid-body
# motion produces, so it costs ~72x the voltage of the rigid directions. M also
# does not extrapolate: fitted over 0.10-0.40 V, it ran ~30 counts optimistic at
# 0.75 V (predicted 764, measured a2 884.0 -> 794.7 counts, railed 40 % -> 0.4 %).
BIAS_CH = np.full(NCOLS, 0.50)

# COIL 2 SITS HIGHER, and a2 was vetoing the whole measurement without it. a2
# rests near 860 counts against `in_guard_band`'s 80..943, i.e. ~83 counts of
# headroom: measured 2026-08-20, a coil pass at 0.50 V tripped the guard at 963
# and 972 counts and retried the drive down 0.0283 -> 0.0141 -> 0.0071 ->
# 0.0035 V, an 8x reduction that buries the response in noise.
# Coil 2 moves a2 at about -205 counts/V -- two closed-loop trims agree (861 ->
# 810 -> 758 -> 702 counts in 0.25 V steps, `data/20260820_180344_fast_lock.csv`)
# -- so +0.55 V buys ~113 counts of headroom, taking a2 to roughly 750.
# An earlier sweep read "a2 is not electrically trimmable"; that is WITHDRAWN. It
# stepped coils 2 AND 3 together and on a2 they OPPOSE (-204.7 against +50.5
# counts/V), so the two largely cancelled and a2 sat still.
# NOTE this is the one channel whose bias differs from the controller's uniform
# 0.50 V, so coil 2's A is measured at an operating point eta does not run at.
BIAS_CH[2] = 1.05
# BIAS_V is the nominal for segments with no coil in scope; it must equal what
# most coils actually hold, or the recorded `u` column lies about the hardware.
assert BIAS_V == float(np.bincount(
    (BIAS_CH * 100).astype(int)).argmax()) / 100.0, (
    "BIAS_V %.2f is not the value most of BIAS_CH holds (%s)" % (BIAS_V, BIAS_CH))


def bias_of(coil):
    """The bias on one coil, or the nominal scalar when no coil is in scope."""
    return BIAS_V if coil is None else float(BIAS_CH[coil])


VMIN, VMAX = 0.0, 2.5                    # the controllers' own window
# OPENED 2026-08-20, and the bench forced it rather than suggested it.
#   The DAC's hard ceiling is 2.5000 V: `arduino.ino` clamps at 25000 units of
#   100 uV onto the AD5628's 0..4095 codes on its internal reference, and it is
#   UNIPOLAR -- a negative bias cannot be commanded at any setting.
#   The old 0.0-0.5 V used 20 % of that with no recorded justification
#   (CLAUDE.md Sec 11). What made it binding: on the 12:40 run every one of coils
#   0-3 slammed BOTH rails, spanning 0.000 to 0.500 V with an rms demand of
#   0.089-0.148 V against a 0.25 V half-window, and the loop PUMPED -- counts std
#   81.8/86.8/170.4/84.7 at zero gain against 136.6/153.8/273.8/180.7 with the
#   gain live, a factor 1.61-2.13 steady over twelve 20 s slices. The signs were
#   re-measured the same session and are NOT the cause (slopesign.py: a0 +70.92,
#   a1 +67.75, a2 -184.26, a3 +81.08 counts/V). Clipping is: once a coil clips
#   the realised force is no longer -Kp*v and dissipation is not guaranteed.
#   HIGHEST VOLTAGE EVER COMMANDED ON THIS HARDWARE WITHOUT INCIDENT IS 1.1 V
#   (analysis/bias_sweep.py). Above that is unmeasured, and the coil driver
#   between the DAC and the coil IS NOT IN THIS REPO, so 0.25 V may have encoded
#   a real current limit. BIAS is the CONTINUOUS one -- it sits on every coil for
#   the whole run -- so a resistive coil dissipates as BIAS^2.

# Restoring the bias after a board reset is a FORCE STEP unless it is ramped.
# 15 s is ~10 periods of the slowest mode (0.72194 Hz). See `park_ramped`.
BIAS_RAMP_S = 15.0
BIAS_RAMP_HZ = 20.0

# Sensor index -> DAC channel. NOTHING IN SOFTWARE CAN CHECK THIS: sim/server.py
# round-trips through whatever map it is handed, and the previous map [0,2,4,6]
# now points at the coils for sensors 4-7 (`CLAUDE.md` § Where the constants came
# from). The DC pass below is the only check there is, and it is WEAK -- one coil
# moves the whole rigid body, so "coil j is loudest on sensor j" is evidence, not
# a verdict.
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]

RAIL_LOW, RAIL_HIGH = 12, 1011           # hard rails, same as the controllers
RAIL_GUARD_LOW, RAIL_GUARD_HIGH = 60, 963    # unwind here, ~50 counts of margin
RAIL_FRAC_REJECT = 0.02                  # >2% of a window pinned -> discard it
GUARD_MARGIN_COUNTS = 20                 # a sensor resting nearer than this to
                                         # the guard band's edge cannot be a
                                         # trip source -- see `in_guard_band`
FLAT_STD_COUNTS = 0.5                    # under this a channel is not moving

# ---------------------------------------------------------------------------
# The three modes. MEASURED 2026-08-20 16:25, after the coils were re-seated and
# the per-channel bias applied: 300 s, 105 013 samples, no drive
# (`data/20260820_162534_status_sensors.csv`). Consensus over a0-a3, which agree
# to 0.00050 / 0.00050 / 0.00000 Hz. a5 is excluded on purpose -- it carries mode
# B at SNR 30.1 but its mode-C peak lands 0.0345 Hz out, 41 half-widths, which is
# a noise peak.
#
# THIS IS A DEFAULT, NOT A MEASUREMENT: re-measure at the start of every session
# (`status.py sensors`, 90 s, no drive) and drive with `--freqs`. Against the
# 2026-08-06 values the shifts are -0.00675 / +0.00038 / -0.00300 Hz, so mode A
# is 8.1 half-widths off and anything still driving at the 2026-08-17 value of
# 0.72194 is off resonance. A stale frequency does not merely attenuate a driven
# point, it PUMPS: 2 pi Df T over a 60 s drive-plus-unwind is 216-367 degrees,
# past anti-phase, so the "unwind" re-drives.
# ---------------------------------------------------------------------------
MODES = (("A", 0.71519),
         ("B", 0.99231),
         ("C", 1.65307))

# ---------------------------------------------------------------------------
# Phi FROM GEOMETRY. Exact, signed, and it costs no bench time.
#
# a0-a3 are four coplanar sensors on the back of the plate all reading along its
# normal (the rig owner's layout, CLAUDE.md § The geometry). Four readings of one
# axis at four corners give exactly three rigid-body quantities plus one thing no
# rigid body can do:
#
#     Z  = (a0+a1+a2+a3)/4     T1 = (a0+a1-a2-a3)/4
#     T2 = (a0-a1+a2-a3)/4     WARP = (a0-a1-a2+a3)/4
#
# THE CORNER ASSIGNMENT IS DETERMINED, not assumed: a rigid plate cannot warp, so
# the pairing that minimises warp is the true one. a0 is diagonal to a3.
# RE-CHECKED 2026-08-20 after the coils were re-seated and UNCHANGED -- the three
# candidate warp vectors come out 48.17 / 20.09 / 12.61 counts rms and
# [+1,-1,-1,+1] is still the smallest (`dof.py` on data/20260820_162534). It wins
# by 1.6x now against 2x on 2026-08-17, because WARP GREW: warp/loudest-rigid
# 0.139 -> 0.262. That growth is a per-sensor gain change -- see SENSOR_H below,
# which nulls the warp over all eight coils.
#
# WHICH MODE IS WHICH DOF is measured too -- AND MODES A AND C SWAPPED TILTS ON
# 2026-08-20. 300 s, 0.0427 Hz bins, `data/20260820_162534_status_sensors.csv`,
# share of each mode's power:
#
#     A 0.7264 Hz -> T2  81.6%      B 0.9828 Hz -> Z  73.7%      C 1.6665 Hz -> T1  89.9%
#
# against A -> T1 90.3%, B -> Z 88.2%, C -> T2 89.4% on 2026-08-17. The two tilt
# columns below are therefore EXCHANGED relative to every version before this one.
# The 2026-08-17 cross-check -- a measured Phi from cross-spectral
# eigendecomposition agreeing with the geometry at |cos| 0.971 / 0.978 / 0.963,
# using no geometry as input -- is a property of THAT record and does not transfer
# across the re-seating. Re-run `status.py phi` to earn it again.
#
# WHY THIS MATTERS MORE THAN THE 2% IT COSTS IN ACCURACY. A measured Phi is
# determined only up to a sign per mode, and A independently so; the pair has to
# agree, and picking wrong does not under-damp, it PUMPS -- measured, median
# channel ratio 1.5 diagonal -> 2.3 modal on 2026-08-17. That gauge freedom exists
# ONLY because Phi is measured. Fix Phi to these exact columns and A's sign is
# fully determined by the driven measurement: no gauge, no signtest, no 300 s
# passive record, no jackknife. The measured Phi becomes a CHECK on the geometry
# instead of the thing the loop depends on.
#
# a4-a7 are zero here and that is not a gap: a4/a5 sense left-right and a6/a7
# vertical, both orthogonal to Z/T1/T2, and neither pair resonates anywhere in
# 0.3-25 Hz. Their coherent content is cross-coupling from these same three modes
# (CLAUDE.md § 6), so they carry no Phi row and are handled by the hybrid PID.
# ---------------------------------------------------------------------------
# PER-SENSOR GAIN CALIBRATION, and it is what finally explains WARP.
#
# THE PROBLEM WITH A BARE GEOMETRIC Phi. `[+-1, +-1, +-1, +-1]` is the mode shape
# in PHYSICAL units and it assumes the four in-plane sensors share a
# counts-per-metre. They do not: their measured DC slopes are 70.5 / 88.9 / 205.9
# / 98.8 counts/V, so a2 is 2.9x a0. Projecting raw COUNTS onto a physical mode
# shape is therefore a unit error, and it skews every modal coordinate, every
# residual, and the warp channel most of all.
#
# THE CONSTRAINT THAT PINS IT DOWN. A rigid plate cannot warp, so for EVERY coil
# the measured static response expressed in physical units must be warp-free:
#
#     sum_i  w_i * (M[i,j] / g_i)  =  0,     w = [+1, -1, -1, +1]
#
# one equation per coil. Eight coils, four unknowns: OVER-DETERMINED, so it can
# fail. CLAUDE.md records an earlier attempt at this same idea and calls it a
# failure -- but that one used only the three mode shapes, 3 equations in 4
# unknowns, where a solution always exists and therefore proves nothing. This one
# is a real fit. Solved as the smallest right singular vector of
# W[j,i] = w_i * M[i,j], from `data/20260820_155638_status_slopesign.csv`.
#
# MEASURED 2026-08-20. Singular values 310.69 / 117.98 / 60.82 / 5.03.
#     h = 1/g, mean|h| = 1:   [1.6057, 1.0667, 0.4761, 0.8514]
# Warp residual, rms over the eight coils, against rms |response|:
#     raw 0.6224  ->  calibrated 0.0322, a 19.3x reduction.
# Per coil, counts:  -127.90 -> +3.92,  +57.90 -> -5.09,  +141.10 -> +4.11,
#     -65.10 -> -5.29,  -6.90 -> -3.34,  +14.90 -> -0.22,  -10.20 -> +4.47,
#     -0.40 -> +0.96.
#
# INDEPENDENTLY CONFIRMED, which is why this is not just a curve fit: h correlates
# +0.960 with 1/|diagonal slope| ([70.50, 88.87, 205.94, 98.79]). h was fitted
# from the OFF-DIAGONAL warp constraint over all eight coils and never saw the
# diagonal. Two unrelated routes, the same numbers.
#
# CONSEQUENCE FOR CLAUDE.md: "WARP IS REAL AND UNEXPLAINED" is superseded, and so
# is "do not write down that it is calibration". On this data it IS calibration,
# and the 0.139 -> 0.262 growth in warp/rigid across the re-seating is a change in
# the sensor gains, which is what re-seating a coil would do.
#
# The full measured slope vector, `slopesign.py` on the same record, counts/V:
#     [+70.50, +88.87, -205.94, -98.79, +5.06, -0.39, +10.37, +8.07]
# a3 FLIPPED SIGN in the re-seating (+81.08 -> -98.79), which is why any damping
# gain has to carry `slope x gain > 0` per channel: the inverted convention
# dissipated on 5.4 / 11.0 / 22.6 / 12.3 % of steps against 90-97 % once
# corrected (`osem.eta.py`'s DAMP_SIGN carries the corrected signs).
#
# APPLIED AS A NORMALISATION, NEVER BY BENDING Phi -- scaling Phi's rows would
# destroy the orthogonality of its columns, which is a real property of the mode
# shapes in physical units. `to_physical` is that normalisation; note that
# nothing in THIS file projects through it yet, so every modal quantity printed
# here is still in raw counts.
SENSOR_H = np.array([1.6057, 1.0667, 0.4761, 0.8514, 1.0, 1.0, 1.0, 1.0])


def to_physical(counts):
    """Counts -> a common physical scale, so the geometric Phi actually applies."""
    return np.asarray(counts, float) * SENSOR_H


GEO_PHI = np.zeros((NCOLS, 3))
GEO_PHI[:4, 0] = [+1, -1, +1, -1]        # mode A -> T2 tilt   (was T1 pre-reset)
GEO_PHI[:4, 1] = [+1, +1, +1, +1]        # mode B -> Z, along the plate normal
GEO_PHI[:4, 2] = [+1, +1, -1, -1]        # mode C -> T1 tilt   (was T2 pre-reset)
GEO_DOF = ("T2 tilt", "Z normal", "T1 tilt")
GEO_WARP = np.array([+1., -1., -1., +1.])    # what no rigid body produces
GEO_COS_WARN = 0.90                      # measured Phi below this vs geometry

# Per-mode envelope decay, `analysis/out/ringdown_mode_summary.csv` gamma_median.
# Used ONLY to decay the residual phasor between a pulse's pre-window and its
# ring window. The quiet record puts gamma at 0 +/- 0.0072 /s, so these are an
# upper bound on the correction, not a precise one.
MODE_GAMMA = {"A": 0.0483, "B": 0.0082, "C": 0.0094}

# Q lower bound at 1 sigma, `analysis/ringdown.md`. Only ever used to CAP the
# predicted ramp gain, so using the lower bound is the conservative direction:
# it can only make the tool predict a smaller response and therefore command a
# LARGER amplitude... which is the unsafe direction. Hence the rail guard is
# what actually protects the rig, and this is only for the printed estimate.
Q_FLOOR = 433.0

# ---------------------------------------------------------------------------
# d(counts_i)/d(bias_j), row = sensor, col = coil. Measured 2026-08-04, one coil
# stepped at a time: `bench/20260804/dcmatrix.log`.
#
# THIS IS A SEED, NOT A RESULT. It exists to choose a starting amplitude for
# each coil so that a coil known to be 20x weaker does not get 20x too little
# drive. The `coils` pass re-measures the whole matrix and reports its own.
#
# Read it and the shape of the problem is already visible: mean |response| is
# 66 counts/V inside the a0-a3 x coil0-3 block and 4 counts/V inside the
# a4-a7 block, and 4 counts/V IS that measurement's noise floor.
# ---------------------------------------------------------------------------
DC_SEED = np.array([
    [-105., -107.,  -48.,  +19.,  -29.,  -21.,  -28.,  -26.],   # a0
    [ -49.,  -68.,  -33.,  +18.,   +5.,   +7.,   +4.,   +8.],   # a1
    [ +35.,  +66., +213., -141.,  -25.,  -22.,  -13.,  -26.],   # a2
    [ +36.,  +14.,  +58.,  -51.,   -1.,   -2.,   +2.,   -1.],   # a3
    [  +0.,   +2.,   +2.,   +1.,   +8.,   +8.,   +2.,   +1.],   # a4
    [  +4.,   +0.,   +0.,   -6.,   -0.,   -7.,   -1.,   +8.],   # a5
    [  +1.,   +4.,   +2.,   +3.,   +3.,   +3.,   +4.,   +6.],   # a6
    [  -0.,   +3.,   +1.,   +0.,   +1.,   -1.,   +6.,   +7.],   # a7
])
DC_SEED_FLOOR = 4.0                      # counts/V; the a4-a7 block's own noise

# ---- the active pass ------------------------------------------------------
DWELL_S = 30.0                # drive; 30 s is 67-155x static and still << tau
UNWIND_S = 30.0               # anti-phase, same length: linear ramp reverses
QUIET_S = 8.0                 # residual check before the next point
PRE_S = 6.0                   # residual measured BEFORE the drive
DC_STEP_S = 20.0              # static step per coil, for sign and wiring
DC_STEP_V = 0.15              # +/- about bias; the lowest bias is 0.50 V, so the
                              # step stays well inside VMIN..VMAX
SETTLE_BETWEEN_COILS_S = 10.0

TARGET_SWING_COUNTS = 200.0   # what the loudest sensor should reach at the end
AMP_MAX = 0.20                # V peak on the coil, against the narrowest
                              # half-window BIAS_CH leaves; `plan` prints it
AMP_MIN = 0.0005
AMP_RETRY_SCALE = 0.5         # on a guard trip
AMP_RETRIES = 2
QUIET_FRAC = 0.25             # residual must fall below this x the peak
# ONE unwind per point, never a retry, and the reason is measured: a repeated
# unwind does not converge, it PUMPS. 2026-08-17, mode C on coil 0, residual
# 0.0781 -> 0.1710 -> 0.3062 V over three attempts against a 0.1416 V peak, and
# every retried point in that run grew the same way -- there is no feedback in an
# unwind, so a failed cancellation is retried with the command that failed.
# `_point` measures the residual and REPORTS it instead, per point, so a
# contaminated point is identifiable offline.

UPDATE_HZ = 50.0              # coil refresh; 30 points/cycle at the fastest mode

# ---- is anything actually arriving? ---------------------------------------
# The board's serial rate is probed, not assumed -- see pyDAC2.BAUD_CANDIDATES
# and pyDAC2.resolve_baud, which own that list for every tool in the tree.
PROGRESS_EVERY_S = 5.0        # live line during any segment longer than this
PREFLIGHT_S = 2.0             # read this long before occupying the bench
PREFLIGHT_MIN_HZ = 50.0       # the wire measures 418-435 Hz at 115200 baud with
                              # 8 ASCII channels (CLAUDE.md), so 50 Hz is ~12% of
                              # the slowest rate seen. Above it the stream exists;
                              # below it there is nothing to measure.

# ---- the pulse ------------------------------------------------------------
PULSE_COIL = 0
PULSE_W_S = 0.40              # Hann width; flat to ~1/W = 2.5 Hz, covers A/B/C
PULSE_AMP_V = 0.15
PULSE_RING_S = 30.0
PULSE_REPEATS = 3
PULSE_PRE_MAX_FRAC = 0.35     # refuse to fire if the residual is this big
SIGNATURE_PATH = os.path.join("data", "status_signature.json")

# ---- the gate -------------------------------------------------------------
PHI_WINDOW_S = 32.0           # passive-Phi averaging window. NOT a free knob:
                              # at 8 s, mode A's CSD comes out rank 0.70 and the
                              # gate loses a0; the second eigenvalue collapses
                              # 0.476 -> 0.0034 between 4 s and 32 s, so 8 s was
                              # simply not resolving mode A. Measured on
                              # data/20260806_192723_quiet_openloop.csv. Longer
                              # is better and costs only record length.
PHI_MIN_WINDOWS = 6           # fewer than this and the jackknife has no spread
# Fine-scan half-width for "where is this mode actually?". 0.05 Hz is 26
# half-widths at Q=433, so it covers a drift far larger than anything that would
# still be recognisable as the same mode, and at 300 s it is 15 resolution
# elements wide. The measured 2026-08-06 -> 2026-08-17 drift was 0.0170 Hz on
# mode C -- and note that the FIRST answer this scan gave for that drift was
# 0.0100 Hz, because at the old +/-0.010 Hz width the peak sat on the edge of the
# window. The width was under-reporting the very quantity it exists to measure.
MODE_SCAN_HZ = 0.05
MODE_SCAN_N = 201             # 0.0005 Hz steps

ROW_SNR = 3.0                 # |Phi_mi| must exceed this many sigma to count
RANK1_MIN = 0.80              # a single mode's response matrix must be rank 1
IMAG_FRAC_MAX = 0.35          # of Phi that no real mode shape explains, or of A
                              # that is not in quadrature -- above it, provisional
PHASE_SPREAD_OK_DEG = 20.0    # one mode, one phase up to a sign: scatter above
                              # this is contamination (73-89 deg on the 23:12 pass)
PHASE_DIFF_DEG = 5.0          # two estimators closer than this are the same
MODES_N = len(MODES)


def _git_rev():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "unknown"


def clamp(v):
    return min(max(v, VMIN), VMAX)


# ===========================================================================
# the graded census -- colour, and what the three colours MEAN
# ===========================================================================
class Ink:
    """ANSI colour, off whenever the output is not a terminal.

    A census gets kept by redirecting it into a file, and a file full of escape
    codes is a file nobody greps: auto by default, `--color always` for
    `| less -R`, NO_COLOR (no-color.org) or a non-tty turns it off. Colour is
    never the only carrier -- every coloured cell also says GOOD / OK / DEAD in
    words, because a red cell in a plain-text paste is just a cell.
    """

    CODES = dict(green="\033[32m", yellow="\033[33m", red="\033[31m",
                 grey="\033[90m", bold="\033[1m", off="\033[0m")

    def __init__(self, mode="auto"):
        self.on = bool(mode == "always" or
                       (mode == "auto" and sys.stdout.isatty()
                        and not os.environ.get("NO_COLOR")))

    def __call__(self, s, c):
        if not self.on or c not in self.CODES:
            return s
        return self.CODES[c] + s + self.CODES["off"]

    def pad(self, s, c, width):
        """Pad FIRST, then colour. An escape code has zero printed width and
        `%-*s` counts its bytes, so colouring first shifts every following
        column by nine characters on exactly the rows that are coloured."""
        return self("%-*s" % (width, s), c)


INK = Ink("auto")

GRADE_COLOUR = {"GOOD": "green", "OK": "yellow", "DEAD": "red", "?": "grey"}

# ---------------------------------------------------------------------------
# Which DOF each OSEM watches. Reported by the rig owner 2026-08-17: a0-a3 are
# the four in-plane corner sensors, a4/a5 the two side sensors, a6/a7 VERTICAL.
# WHICH CORNER EACH OF a0-a3 OCCUPIES IS NOT ESTABLISHED and is deliberately not
# encoded -- the coordinates first given were withdrawn as illustrative.
#
# This has to be in the tool: grading a VERTICAL sensor by its lock-in at
# 0.72-1.66 Hz and its coherence with a0-a3 in that band is a test it must fail
# however healthy it is, and on 2026-08-17 a6/a7 duly came back "BLIND", which is
# true, expected, and says nothing about whether they work. So the horizontal
# verdict is OUT OF SCOPE for them rather than DEAD -- except for the failures
# that are DOF-independent: a railed channel is dead whatever it watches, and so
# is one with no variance at all. The VERTICAL mode frequencies are NOT measured,
# so these channels cannot be graded at all yet, and saying so is the output.
VERTICAL_CH = (6, 7)

# Power-spectrum bands for "if it is not in the mode band, where is it?". The
# top of the range is set by what the wire can carry: 420 Hz sampling at 115200
# baud gives a 210 Hz Nyquist, and these all sit far below it.
PROFILE_BANDS = (("DC-0.5", 0.05, 0.5), ("modes", 0.6, 1.8), ("1.8-3", 1.8, 3.0),
                 ("3-6", 3.0, 6.0), ("6.19", 5.9, 6.5), ("6.5-10", 6.5, 10.0),
                 ("10-20", 10.0, 20.0), ("20-60", 20.0, 60.0))

# A band holding this many times its own bandwidth share of the power counts as
# structure rather than noise. White noise puts exactly its bandwidth share in
# every band, so the ratio is 1.0 for a dead line by construction -- 2.5 is a
# margin over that, not a tuned number. Measured 2026-08-17: a6/a7's bands came
# out at 0.7-1.1x their bandwidth share across 20-60 Hz, 10-20 Hz and 6.5-10 Hz,
# i.e. white, while a0-a4 put 92-98% of their power in a band that is 2% of the
# range -- a ratio of 46-49.
PROFILE_STRUCTURE = 2.5

# A sensor "carries" a mode at this SNR. Deliberately stricter than the gate's
# own ROW_SNR = 3: the gate is the binding test and it runs on the estimator
# that actually builds Phi, so the table has no business being the more
# permissive of the two. A sensor green here is one the gate will not argue
# about.
MIMO_SNR = 8.0

def headroom_min():
    """Counts to the nearer guard rail that a sensor must have. Not a taste:
    the active pass aims for TARGET_SWING_COUNTS peak-to-peak, so a channel
    with less than half of that in hand cannot survive the census's own
    nominal drive, let alone a kick. Read live, not frozen at import, because
    `--target` moves it."""
    return TARGET_SWING_COUNTS / 2.0

# A coil "drives" a mode if some non-railed sensor sees it at this SNR.
COIL_SNR = 8.0

# Two coils this parallel in modal space are ONE direction, not two.
#
# The threshold comes from the measured failure it has to catch: coils 4-7 sit
# at pairwise cosine +0.966 (CLAUDE.md), which is why they are useless -- not
# their gain. At that cosine the second coil adds a component of only
# sqrt(1 - 0.966^2) = 0.26 that the first does not already cover.
#
# Note what is NOT used here: the residual of coil j against the span of ALL
# the others. That measure is identically zero for any rig with more coils than
# modes, because the response has exactly MODES_N degrees of freedom, so every
# column is trivially in the span of the rest. It looks like a strong statement
# and is arithmetic. Pairwise cosine and leave-one-out conditioning are the two
# questions that stay meaningful.
COIL_TWIN_COS = 0.95


def _headroom(mean_counts):
    """Counts to the nearer guard rail. Negative means already outside it."""
    return min(RAIL_GUARD_HIGH - mean_counts, mean_counts - RAIL_GUARD_LOW)


def band_profile(t, counts, nfft=8192):
    """Per channel: where is its power, and is that structure or just bandwidth?

    THE POINT OF THE RATIO. White noise puts exactly its bandwidth share of the
    power in every band, so a band's share divided by its bandwidth share is 1.0
    for a dead line and needs no threshold to interpret. Reading the raw
    percentages instead is how a railed channel gets mistaken for a live one: on
    2026-08-17 a6 put 51.8% of its power in 20-60 Hz, which looks like a lot
    until you notice that band is 67% of the range.

    Returns per channel a dict of band -> (share, ratio) plus the loudest band
    by ratio, or None if the record is too short.
    """
    t = np.asarray(t, float)
    x = np.asarray(counts, float)
    if len(t) < 2 or len(x) < 2 * nfft:
        nfft = max(256, 1 << int(math.log2(max(len(x) // 4, 256))))
    if len(x) < 2 * nfft:
        return None
    fs = len(t) / (t[-1] - t[0])
    win = np.hanning(nfft)
    idx = range(0, len(x) - nfft, nfft // 2)
    F = np.array([np.fft.rfft((x[i:i + nfft] - x[i:i + nfft].mean(axis=0))
                              * win[:, None], axis=0) for i in idx])
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    P = (np.abs(F) ** 2).mean(axis=0)
    lo, hi = PROFILE_BANDS[0][1], PROFILE_BANDS[-1][2]
    span = (f > lo) & (f < hi)
    total = P[span].sum(axis=0)
    width = hi - lo
    out = []
    for i in range(x.shape[1]):
        bands = {}
        for name, a, b in PROFILE_BANDS:
            m = (f > a) & (f < b)
            share = float(P[m, i].sum() / total[i]) if total[i] > 0 else 0.0
            bands[name] = (share, share / ((b - a) / width))
        # The verdict EXCLUDES the DC band. A railed or drifting line puts a
        # large excess below 0.5 Hz for reasons that have nothing to do with
        # what it is pointed at -- measured 2026-08-17, a6/a7 came out at 16.9x
        # and 15.2x there while being white everywhere above it. Calling that
        # "structure" would report a wandering dead channel as a working one.
        cand = [k for k in bands if k != PROFILE_BANDS[0][0]]
        best = max(cand, key=lambda k: bands[k][1])
        out.append(dict(bands=bands, best=best, best_ratio=bands[best][1],
                        dc_ratio=bands[PROFILE_BANDS[0][0]][1],
                        structured=bands[best][1] >= PROFILE_STRUCTURE))
    return out


def grade_sensor(r):
    """GOOD / OK / DEAD for one sensor, and the number that decided it.

    The order of the tests is the argument:

        1. railed or flat      -- it is not returning a signal at all
        2. ELECTRICAL          -- it IS returning a signal, of another channel.
                                  This is the case that cost this repo days,
                                  and a loud wiring artefact grades above a
                                  quiet real sensor on every amplitude test.
        3. no mode at all      -- real, but blind on the optic
        4. headroom            -- real and sighted, but cannot be driven
        5. fewer than 3 modes  -- damps fine; no modal row
    """
    n3 = sum(1 for m in r["modes"].values() if m["snr"] >= MIMO_SNR)
    got = [k for k, m in sorted(r["modes"].items()) if m["snr"] >= MIMO_SNR]
    head = _headroom(r["mean"])
    xt = (r.get("xtalk") or "?").split(" ")[0]
    prof = r.get("profile")

    # The two failures that are DEGREE-OF-FREEDOM INDEPENDENT come first, so a
    # vertical channel still gets condemned for the right reasons. A railed
    # channel is dead whatever it watches -- it is at the end of its range, and
    # clipping is nonlinear, so even the motion it does show is untrustworthy.
    if r["verdict"] == "RAILED":
        return "DEAD", "pinned for %.0f%% of the record (limit %.0f%%) -- a rail" \
            " is not a measurement whatever the sensor watches" \
            % (100 * r["railed"], 100 * RAIL_FRAC_REJECT), n3
    if r["verdict"] == "FLAT":
        return "DEAD", "std %.2f counts -- the line is not moving" % r["std"], n3
    # Headroom is degree-of-freedom independent too: a channel resting this close
    # to a guard rail will rail as soon as it is driven, whatever it watches. The
    # selftest caught this being checked AFTER the vertical branch, which let a
    # vertical channel with 63 counts of headroom grade GOOD.
    if head < headroom_min():
        return "OK", "rests at %.0f counts, only %.0f to the %s guard (drive" \
            " needs %.0f)" % (r["mean"], head,
                              "top" if RAIL_GUARD_HIGH - r["mean"] < head + 1
                              else "bottom", headroom_min()), n3

    # Not railed, not flat, has headroom, and not looking at these modes: OUT OF
    # DEAD. The horizontal test cannot grade it and saying so is the honest
    # answer -- the vertical mode frequencies are not measured.
    if r["ch"] in VERTICAL_CH:
        where = ""
        if prof:
            where = "; power is %s (%.1fx its bandwidth share)" \
                % (prof["best"], prof["best_ratio"]) if prof["structured"] else \
                "; no band holds more than %.1fx its bandwidth share, i.e. the" \
                " spectrum is white" % prof["best_ratio"]
        if n3 >= MODES_N:
            return "GOOD", "a VERTICAL channel that nevertheless carries all" \
                " three horizontal modes%s" % where, n3
        return "?", "VERTICAL channel -- this census scores the three" \
            " HORIZONTAL modes and cannot grade it%s" % where, n3

    if xt == "ELECTRICAL":
        return "DEAD", "coherent out of band as well as in it -- it is reading" \
            " another channel, not the optic", n3
    if xt == "BLIND" and n3 == 0:
        # The coherence test outranks the lock-in here, and it should: a6 comes
        # back at SNR 4.6 on mode B in the 2026-08-06 quiet record, which is
        # over the ROW_SNR line, while its in-band coherence with every sensor
        # that IS on the optic sits at the 1/K bias floor. A marginal lock-in on
        # a channel that shares no motion with the optic is a marginal lock-in
        # on something else. (This defers to a0-a3 being real; if they were not,
        # everything would read BLIND and the whole table would be suspect.)
        return "DEAD", "no in-band coherence with any sensor on the optic --" \
            " its best SNR of %.1f is not the optic's motion" % r["best_snr"], n3
    if r["best_snr"] < ROW_SNR:
        return "DEAD", "best SNR %.1f at any mode, under %.0f -- blind on the" \
            " optic" % (r["best_snr"], ROW_SNR), n3
    if n3 < MODES_N:
        return "OK", ("carries %s only" % ",".join(got) if got else
                      "reaches no mode above SNR %.0f (best %.1f)"
                      % (MIMO_SNR, r["best_snr"])) \
            + " -- damps, but no modal row (needs 3 of 3 above SNR %.0f)" \
            % MIMO_SNR, n3
    if xt == "AMBIGUOUS":
        return "OK", "3 of 3 modes, but the in/out-of-band contrast does not" \
            " separate optic from wiring", n3
    return "GOOD", "3 of 3 modes above SNR %.0f, %.0f counts of headroom" \
        % (MIMO_SNR, head), n3


def coil_summary(res, dcm=None):
    """Per coil: what it drives, how hard, and whether it is a NEW direction.

    The direction columns are the ones that are not obvious. A coil can be loud,
    linear and perfectly healthy and still be worth nothing to a modal
    allocator, because it pushes where another coil already pushes. That is not
    hypothetical here: coils 4-7 were measured at pairwise cosine +0.966 and
    that, not their gain, is why they are useless.

    A coil's direction is its column of the modal matrix, recovered per mode by
    the same rank-1 factorisation the gate uses. Each mode's row is normalised
    to unit length -- so the three modes are weighted equally in the cosine and
    the answer does not depend on which mode happens to be easiest to drive.
    """
    coils = sorted({p["coil"] for p in res})
    out = {}
    for j in coils:
        pts = {p["mode"]: p for p in res if p["coil"] == j}
        row = dict(coil=j, dac=DAC_MAP[j], modes={}, amp=np.nan,
                   peak=0.0, peak_ch=-1, railed=[])
        for name, _ in MODES:
            p = pts.get(name)
            if p is None:
                row["modes"][name] = dict(snr=np.nan, mag=np.nan, ch=-1)
                continue
            snr = np.where(p["sigma"] > 0, np.abs(p["H"]) / np.maximum(p["sigma"], 1e-30), 0.0)
            snr = np.where(p["ok"], snr, 0.0)
            k = int(np.argmax(snr))
            row["modes"][name] = dict(snr=float(snr[k]), mag=float(abs(p["H"][k])), ch=k)
            row["amp"] = float(p["amp"])
            mag = np.where(p["ok"], np.abs(p["H"]), 0.0)
            b = int(np.argmax(mag))
            if mag[b] > row["peak"]:
                row["peak"], row["peak_ch"] = float(mag[b]), b
            row["railed"] += ["a%d" % i for i in np.where(~p["ok"])[0]]
        row["nmodes"] = sum(1 for m in row["modes"].values()
                            if np.isfinite(m["snr"]) and m["snr"] >= COIL_SNR)
        row["drives"] = [k for k, m in sorted(row["modes"].items())
                         if np.isfinite(m["snr"]) and m["snr"] >= COIL_SNR]
        if dcm is not None and np.isfinite(dcm[:, j]).any():
            k = int(np.nanargmax(np.abs(dcm[:, j])))
            row["dc"], row["dc_ch"] = float(dcm[k, j]), k
        else:
            row["dc"], row["dc_ch"] = np.nan, -1
        out[j] = row

    # The modal matrix, one row per mode, over the coils that actually DRIVE
    # something. A coil that reaches no mode has no measured direction -- only
    # noise -- and including it corrupts everyone else's geometry: on a synthetic
    # 4-coil replay a dead coil's noise direction came out at cosine +0.894 with
    # a healthy one, and 0.95 is the line at which that would have demoted the
    # healthy coil to OK on the strength of the dead one's noise.
    #
    # Only modes measured on EVERY coil in the set are used, too: a mode missing
    # from one coil would otherwise read as a direction nobody else covers,
    # which is an absent measurement scoring as a capability.
    live = [j for j in coils if out[j]["nmodes"] > 0]
    common = [n for n, _ in MODES
              if all(np.isfinite(out[j]["modes"][n]["snr"]) for j in live)] if live else []
    M = np.zeros((len(common), len(live)), complex)
    for mi, name in enumerate(common):
        pts = [next(p for p in res if p["coil"] == j and p["mode"] == name)
               for j in live]
        Psi = np.column_stack([np.where(p["ok"], p["H"], 0.0) for p in pts])
        _, _, Vh = np.linalg.svd(Psi, full_matrices=False)
        M[mi, :] = Vh[0, :]                  # unit norm per mode, by construction

    rank0, cond0 = 0, np.inf
    if M.size:
        sv0 = np.linalg.svd(M, compute_uv=False)
        rank0 = int(np.linalg.matrix_rank(M, tol=1e-6))
        cond0 = float(sv0[0] / sv0[-1]) if sv0[-1] > 1e-12 else np.inf
    for j in coils:
        # A coil that drives nothing: no direction, and dropping it from the
        # allocator changes literally nothing, so report the set unchanged.
        out[j].update(authority=np.nan, twin=-1, cos=np.nan,
                      drop_rank=rank0, drop_cond=cond0)
    for n, j in enumerate(live):
        col = M[:, n]
        nrm = float(np.linalg.norm(col))
        out[j]["authority"] = nrm
        others = [(k, M[:, k]) for k in range(len(live))
                  if k != n and np.linalg.norm(M[:, k]) > 0]
        if nrm > 0 and others:
            cos = [abs(np.vdot(v, col)) / (np.linalg.norm(v) * nrm) for _, v in others]
            b = int(np.argmax(cos))
            out[j]["twin"], out[j]["cos"] = live[others[b][0]], float(cos[b])
        # Leave-one-out: what the allocator is left with if this coil is lost.
        # The coil-side mirror of the `drop a%d -> rank` block the gate prints.
        keep = [k for k in range(len(live)) if k != n]
        if keep and M.size:
            sv = np.linalg.svd(M[:, keep], compute_uv=False)
            out[j]["drop_rank"] = int(np.linalg.matrix_rank(M[:, keep], tol=1e-6))
            out[j]["drop_cond"] = float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else np.inf
        else:
            out[j]["drop_rank"], out[j]["drop_cond"] = 0, np.inf
    out["_modes"] = common
    out["_M"] = M
    out["_live"] = live
    return out


def grade_coil(row):
    """GOOD / OK / DEAD for one coil, and the number that decided it."""
    n, cos = row["nmodes"], row.get("cos", np.nan)
    if n == 0:
        best = max((m["snr"] for m in row["modes"].values()
                    if np.isfinite(m["snr"])), default=0.0)
        return "DEAD", "no mode above SNR %.0f -- best was %.1f at %.4f V of" \
            " drive" % (COIL_SNR, best, row["amp"])
    if row["railed"]:
        return "OK", "drives %s, but railed %s -- reduce the amplitude and" \
            " re-measure" % (",".join(row["drives"]),
                             ",".join(sorted(set(row["railed"]))))
    if np.isfinite(cos) and cos >= COIL_TWIN_COS:
        return "OK", "drives %s, but points where coil %d already points" \
            " (cosine %+.3f) -- one direction, not two" \
            % (",".join(row["drives"]), row["twin"], cos)
    if n < MODES_N:
        return "OK", "drives %s only -- %s not reached above SNR %.0f" \
            % (",".join(row["drives"]),
               ",".join(k for k, _ in MODES if k not in row["drives"]), COIL_SNR)
    return "GOOD", "drives all three modes, nearest other coil at cosine" \
        " %+.3f" % (cos if np.isfinite(cos) else 0.0)


def census_sensors(res, log=print):
    """The table. One row per sensor, graded, with the number behind the grade."""
    graded = [(r,) + grade_sensor(r) for r in res]
    log("\n  " + INK("=== SENSOR CENSUS ===", "bold"))
    log("  " + INK("GOOD", "green") + " = a modal row: 3 of 3 modes above SNR"
        " %.0f, headroom, and mechanically coupled." % MIMO_SNR)
    log("  " + INK("OK  ", "yellow") + " = damps, but cannot carry a Phi row"
        " -- the reason is in the last column.")
    log("  " + INK("DEAD", "red") + " = railed, flat, blind on the optic, or"
        " reading another channel.")
    log("")
    log("   ch  grade   rest  headroom   floor      A@%.4f   B@%.4f   C@%.4f  modes  coupling"
        % (MODES[0][1], MODES[1][1], MODES[2][1]))
    log("           counts  to guard    volts        SNR         SNR         SNR   of 3")
    for r, g, why, n3 in graded:
        head = _headroom(r["mean"])
        xt = (r.get("xtalk") or "?").split(" ")[0]
        # SNR IS MEANINGLESS ON A RAILED OR FLAT CHANNEL and must be marked as
        # such, because it does not go DOWN -- it goes UP. It is amplitude over
        # the channel's own off-mode floor, and a pinned line's floor collapses
        # with its signal: measured 2026-08-17, a6's floor was 0.00003 V against
        # a0's 0.00765 V, 250x smaller, so a6 reported "2 of 3 modes above SNR 8"
        # while moving 5 counts peak-to-peak. The grade is right because the rail
        # check runs first; the printed number would still mislead a reader.
        junk = r["verdict"] in ("RAILED", "FLAT")
        cells = "".join(
            INK("%9.1f" % r["modes"][n]["snr"], "grey") + "   " if junk
            else "%9.1f   " % r["modes"][n]["snr"] for n, _ in MODES)
        log("   a%d  %s %6.1f    %+6.0f  %7.5f   %s%s  %s"
            % (r["ch"], INK.pad(g, GRADE_COLOUR[g], 6), r["mean"], head,
               r["floor_v"], cells,
               INK("%d/3" % n3, "grey" if junk else
                   "green" if n3 == MODES_N else "yellow" if n3 else "red"),
               INK(xt, {"MECHANICAL": "green", "ELECTRICAL": "red",
                        "BLIND": "red"}.get(xt, "yellow"))))
    if any(r["verdict"] in ("RAILED", "FLAT") for r, _, _, _ in graded):
        log("   greyed SNR: railed or flat. That number is amplitude over the")
        log("   channel's OWN floor, and on a pinned line both collapse together,")
        log("   so it reads HIGH for a dead channel. Ignore it; read the grade.")
    log("")
    for r, g, why, _ in graded:
        log("   a%d  %s %s" % (r["ch"], INK.pad(g, GRADE_COLOUR[g], 5), why))
    tally = {k: sum(1 for _, g, _, _ in graded if g == k) for k in GRADE_COLOUR}
    log("\n   %s sensors, %s, %s, %s"
        % (INK("%d GOOD" % tally["GOOD"], "green"),
           INK("%d OK" % tally["OK"], "yellow"),
           INK("%d DEAD" % tally["DEAD"], "red"),
           INK("%d ungradeable" % tally["?"], "grey")))
    log("   MIMO needs %d GOOD (%d modes + one spare row). This record has %d."
        % (MODES_N + 1, MODES_N, tally["GOOD"]))
    if tally["?"]:
        log("   The ungradeable ones are VERTICAL channels: this census scores the")
        log("   three HORIZONTAL modes, so it cannot pass or fail them. Their mode")
        log("   frequencies are not measured. They are NOT counted against MIMO --")
        log("   %d horizontal channels is what the gate has to work with."
            % (NCOLS - len(VERTICAL_CH)))
    log("   That is an indication, not the verdict: the gate below runs on the")
    log("   estimator that actually builds Phi. `status.py phi` is the test.")
    return graded


def census_coils(res, dcm=None, log=print):
    """One row per coil: what it drives, whether it rails anything, and whether
    it is a direction the allocator does not already have."""
    rows = coil_summary(res, dcm)
    order = sorted(k for k in rows if isinstance(k, int))
    graded = [(rows[j],) + grade_coil(rows[j]) for j in order]
    log("\n  " + INK("=== COIL CENSUS ===", "bold"))
    log("  " + INK("GOOD", "green") + " = drives all three modes above SNR %.0f"
        " AND points somewhere no other coil points." % COIL_SNR)
    log("  " + INK("OK  ", "yellow") + " = drives something, but misses a mode,"
        " rails a sensor, or duplicates another coil.")
    log("  " + INK("DEAD", "red") + " = moves nothing measurable at the"
        " amplitude it was given.")
    log("")
    log("   coil  DAC  grade    amp V   loudest |H|      A SNR     B SNR     C SNR   auth   twin        DC counts/V")
    for row, g, why in graded:
        m = row["modes"]
        cos, tw = row["cos"], row["twin"]
        log("    %d    ch%d  %s  %6.4f   %6.2f -> a%d   %8.1f  %8.1f  %8.1f  %5s   %s   %s"
            % (row["coil"], row["dac"], INK.pad(g, GRADE_COLOUR[g], 6),
               row["amp"], row["peak"], row["peak_ch"],
               m["A"]["snr"], m["B"]["snr"], m["C"]["snr"],
               "%.2f" % row["authority"] if np.isfinite(row["authority"]) else ".",
               INK.pad("%+.3f a%d" % (cos, tw) if np.isfinite(cos) else ".",
                       "yellow" if np.isfinite(cos) and cos >= COIL_TWIN_COS
                       else "green", 9),
               ("%+7.1f -> a%d" % (row["dc"], row["dc_ch"])
                + ("" if row["dc_ch"] == row["coil"] else
                   INK("  <-- not a%d" % row["coil"], "yellow")))
               if np.isfinite(row["dc"]) else "      ."))
    log("   auth = length of this coil's modal column, the three mode rows")
    log("   normalised to 1 each. It is RELATIVE authority within this set, not")
    log("   volts per newton. twin = nearest other coil and their cosine.")
    log("")
    for row, g, why in graded:
        log("   coil %d  %s %s" % (row["coil"], INK.pad(g, GRADE_COLOUR[g], 5), why))
    tally = {k: sum(1 for _, g, _ in graded if g == k) for k in GRADE_COLOUR}
    log("\n   %s coils, %s, %s"
        % (INK("%d GOOD" % tally["GOOD"], "green"),
           INK("%d OK" % tally["OK"], "yellow"),
           INK("%d DEAD" % tally["DEAD"], "red")))

    # The coil-side mirror of the gate's `drop a%d -> rank` block: the allocator
    # needs MODES_N independent columns, and a spare to survive losing one.
    M = rows["_M"]
    if M.size:
        sv = np.linalg.svd(M, compute_uv=False)
        rank = int(np.linalg.matrix_rank(M, tol=1e-6))
        log("\n   modal matrix over the %d coils that drive something (%s) and"
            " modes %s:"
            % (len(rows["_live"]), ",".join("%d" % j for j in rows["_live"]),
               ",".join(rows["_modes"])))
        log("   rank %d of %d, cond %.2f"
            % (rank, len(rows["_modes"]),
               float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else np.inf))
        for row, g, why in graded:
            bad = row["drop_rank"] < len(rows["_modes"])
            log("     drop coil %d -> rank %d of %d, cond %s%s"
                % (row["coil"], row["drop_rank"], len(rows["_modes"]),
                   "%.2f" % row["drop_cond"] if np.isfinite(row["drop_cond"])
                   else "inf",
                   INK("   <-- LOSES A MODE", "red") if bad else ""))
    return graded


# ===========================================================================
# recording
# ===========================================================================
class Recorder:
    """Every raw sample to disk as it arrives, line-buffered, closed in a
    `finally`. Standing practice, `CLAUDE.md`: a 36-minute sweep reduced to one
    matrix is one-shot, and this repo has been bitten twice by that.

    `phase` is the tag the offline pass keys on -- pre|drive|unwind|quiet|dc|
    pulse|ring|passive -- so it never has to guess where a segment ended.
    """

    COLS = ("time_s,seg,phase,coil,freq_hz,amp_v,u_v,"
            + ",".join("a%d" % i for i in range(NCOLS)))

    def __init__(self, tag):
        os.makedirs("data", exist_ok=True)
        self.path = os.path.join(
            "data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_status_%s.csv" % tag)
        self.fh = open(self.path, "w", buffering=1)
        self.fh.write(self.COLS + "\n")
        self.seg, self.phase, self.coil, self.freq, self.amp = 0, "passive", -1, -1.0, 0.0
        self.n = 0
        print("  raw samples -> %s" % self.path)

    def mark(self, phase, coil=-1, freq=-1.0, amp=0.0):
        self.seg += 1
        self.phase, self.coil, self.freq, self.amp = phase, coil, freq, amp

    def write(self, t, counts, u):
        self.fh.write("%.5f,%d,%s,%d,%.6f,%.5f,%.5f,%s\n"
                      % (t, self.seg, self.phase, self.coil, self.freq, self.amp,
                         u, ",".join(str(int(c)) for c in counts)))
        self.n += 1

    def close(self):
        if self.fh and not self.fh.closed:
            self.fh.close()
            print("\n  raw data: %d samples -> %s" % (self.n, self.path))


class Reader:
    """`dac.read_sample` with the range check the ASCII path does not do.

    README Sec Serial protocol: at 500000 baud an `OK ch=..` reply landing on a
    buffer boundary splices two fields into a row that PARSES FINE -- v11's
    240 s log has ten rows outside 0..1023, one of them +45.6 V into the
    bandpass. `pyDAC2.FastDAC` range-checks in BINARY mode only. Any new tool
    on this stream needs this, so it lives here rather than in the caller.
    """

    def __init__(self, dac):
        self.dac, self.kept, self.dropped = dac, 0, 0

    def read(self):
        s = self.dac.read_sample(NCOLS)
        if s is None:
            return None
        for v in s:
            if v < 0 or v > ADC_MAX_COUNTS:
                self.dropped += 1
                return None
        self.kept += 1
        return s

    def report(self):
        tot = self.kept + self.dropped
        if self.dropped:
            print("  sample-guard: dropped %d of %d rows (%.3f%%) outside 0..%d"
                  % (self.dropped, tot, 100.0 * self.dropped / max(tot, 1), ADC_MAX_COUNTS))
        else:
            print("  sample-guard: 0 of %d rows out of range" % tot)


# ===========================================================================
# analysis primitives
# ===========================================================================
def lockin(t, y, f):
    """Complex amplitude of `y` at `f`, integrating with the real dt.

    Both quadratures at once, so the sign never depends on having guessed one --
    the mistake that produced a wrong actuation matrix once already. Host arrival
    times are not uniform, hence dt rather than an assumed rate.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if len(t) < 8:
        return 0j
    span = t[-1] - t[0]
    if span <= 0:
        return 0j
    dt = np.gradient(t)
    return 2.0 * np.sum(y * np.exp(-2j * math.pi * f * t) * dt) / span


def decohere(t, y, ramp=False, freqs=None):
    """`y` with DC, a linear trend and all three mode lines least-squares removed.

    What is left is the noise, and the point of removing the lines first is that
    the noise has to be measured NEXT TO a signal that is up to 300x bigger than
    it. A rectangular window leaks as 1/delta, so at the nearest probe -- 5.7
    bins out -- a 1.9 V ramping response still puts ~50 mV into a bin where the
    true noise is 20 uV. Measured that way the probe returns the DRIVE, and
    every quiet channel gets the same error bar as every loud one.

    `ramp=True` adds t*cos and t*sin per line, which is the on-resonance drive's
    actual shape (x ~ t sin wt) rather than a stationary sinusoid. Without it
    the fit leaves the ramp's growth behind as residual and calls it noise.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if len(t) < 16:
        return y - y.mean()
    tt = t - t.mean()
    cols = [np.ones_like(tt), tt]
    for f in (freqs if freqs is not None else [m[1] for m in MODES]):
        w = 2.0 * math.pi * f
        cols += [np.cos(w * t), np.sin(w * t)]
        if ramp:
            cols += [tt * np.cos(w * t), tt * np.sin(w * t)]
    X = np.column_stack(cols)
    try:
        beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    except np.linalg.LinAlgError:
        return y - y.mean()
    return y - X @ beta


def lockin_sigma(t, y, f, avoid=None, ramp=False):
    """(phasor at f, sigma) with sigma measured OFF-TONE in the same window.

    The error bar is the ONLY thing that decides whether a row of Phi is
    determined, so it has to be measured rather than assumed. This is what
    `tune._bootstrap_P` throws away by keeping `np.diag(...)`
    (`analysis/mimo_closed.md` hole H2), and it is why ch1's mode-1 entry could
    sit at +/-9x its own value without anything noticing.

    OFF-TONE AND NOT BLOCK-SCATTER, and the difference is not cosmetic. The
    obvious estimator -- split the dwell into blocks and take the scatter of
    the per-block phasors -- is WRONG here, and wrong in a way that looks
    perfectly healthy. The on-resonance response RAMPS, so block 8 has eight
    times block 1's amplitude and the scatter is dominated by the ramp rather
    than by the noise. Written that way this file returned SNR 52.2 on every
    channel of a synthetic plant whose per-channel noise spanned 21x
    (`--selftest`): identical SNR across channels is the signature.

    Off-tone also keeps the COLOUR. sigma_z^2 = 2 S_n(f) / T, so what is needed
    is the noise power spectral density near f, not a white-noise count of
    samples: at ~1000 rows/s a white estimator divides by sqrt(2/30000) and
    reports an SNR the rig does not have. The probes measure the PSD where the
    signal is not, which is the same quantity with the colour left in.

    `decohere` runs first so the probes see noise rather than the drive's own
    spectral leakage. That is a model, but only for what is SUBTRACTED; the
    error bar itself is still read off the data.

    Probe offsets skip anything within 0.05 Hz of a mode, or the "noise" would
    be another mode's line. `sigma` is the PER-COMPONENT sd: |z| of complex
    Gaussian noise is Rayleigh, whose median is sigma*sqrt(2 ln 2), so the
    median of the probes is divided by that.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    z = lockin(t, y, f)
    return z, noise_at(t, decohere(t, y, ramp), f, avoid)


# Far enough out that the drive tone's own spectral leakage is negligible: a
# 30 s window resolves 0.033 Hz, so the nearest probe is 5.7 bins away and the
# furthest 18. Irregularly spaced on purpose -- an even grid would let one
# periodic interferer land on several probes at once.
NOISE_OFFSETS = (0.19, 0.27, 0.34, 0.43, 0.53, 0.61)
RAYLEIGH_MEDIAN = math.sqrt(2.0 * math.log(2.0))       # 1.1774


def noise_at(t, y, f, avoid=None):
    """Per-component sd of the lock-in at `f`, measured where no signal is."""
    modes = [m[1] for m in MODES] if avoid is None else list(avoid)
    probes = []
    for d in NOISE_OFFSETS:
        for s in (-1.0, +1.0):
            g = f + s * d
            if g < 0.08:
                continue
            if any(abs(g - m) < 0.05 for m in modes):
                continue
            probes.append(abs(lockin(t, y, g)))
    if not probes:
        return float("inf")
    return float(np.median(probes) / RAYLEIGH_MEDIAN)


def to_volts(counts):
    """A counts window -> volts about its own per-channel mean."""
    c = np.asarray(counts, float)
    return (c - c.mean(axis=0)) * COUNTS_TO_V


def response(t, counts, U, f, ramp=True):
    """(H, sigma) per sensor at `f`, divided by the LOCK-IN OF THE RECORDED
    DRIVE `U` -- never by the amplitude that was intended. That is the
    `multisine` defect in `CLAUDE.md` standing practice, and dividing by the
    recorded drive is what made the in-run and offline answers agree to 2e-4."""
    v = to_volts(counts)
    H, S = np.zeros(NCOLS, complex), np.zeros(NCOLS)
    for i in range(NCOLS):
        z, sd = lockin_sigma(t, v[:, i], f, ramp=ramp)
        H[i], S[i] = z / U, sd / abs(U)
    return H, S


def worst_lockin(t, counts, f):
    """Loudest per-sensor lock-in amplitude in a window, volts; 0 if too short.

    The one estimator behind `pre`, `peak` and `resid`, so they compare
    like for like -- which they did not before 2026-08-17 (see `_point`).
    """
    if len(t) <= 8:
        return 0.0
    v = to_volts(counts)
    return max(abs(lockin(t, v[:, i], f)) for i in range(NCOLS))


# Sensors the rail guard may never trip on. See `guard_mask`.
GUARD_EXCLUDE = (2,)


def in_guard_band(mean_counts):
    """Which channels rest inside the rail guard band. One owner for the test."""
    m = np.asarray(mean_counts, float)
    return ((m > RAIL_GUARD_LOW + GUARD_MARGIN_COUNTS)
            & (m < RAIL_GUARD_HIGH - GUARD_MARGIN_COUNTS))


def railed_frac(counts, rest=None):
    """Fraction of samples pinned at either rail, per channel.

    Channels already outside the guard band AT REST score 0: the drive cannot
    have railed something that was against the rail before it started, and the
    same exclusion is what `guard_mask` applies to the trip source. Measured
    2026-08-17: a5 is a disconnected pin (exactly one distinct value, 0.0 counts,
    variance exactly zero over 236 387 samples), so every coil point reported
    `railed a5` and `grade_coil` returned OK -- "reduce the amplitude and
    re-measure" -- on four coils with nothing wrong with them.

    `rest` is the pre-drive window. Without it the drive window's own mean is
    used, which is weaker: a channel the drive pins for essentially the WHOLE
    window has its mean at the rail too and is excluded. Pass `rest` where there
    is one.
    """
    c = np.asarray(counts, float)
    if c.size == 0:
        return np.zeros(NCOLS)
    f = ((c <= RAIL_LOW) | (c >= RAIL_HIGH)).mean(axis=0)
    ref = c if (rest is None or len(rest) == 0) else np.asarray(rest, float)
    return np.where(in_guard_band(ref.mean(axis=0)), f, 0.0)


def ramp_gain(f, seconds):
    """Response / static response for an on-resonance drive from rest.

    Undamped: x(t) = (F/2 m w) t sin(wt), so x/x_static = pi f t, growing
    without bound. Damping caps it at Q. tau > 138 s and the dwell is 30 s, so
    the cap is never reached here -- it is in the expression because the tool
    must not silently mispredict if somebody raises DWELL_S.
    """
    return min(math.pi * f * seconds, Q_FLOOR)


def choose_amp(coil, f, seconds, target=TARGET_SWING_COUNTS):
    """Drive amplitude for this point, from the seed matrix. Never typed."""
    g = float(np.max(np.abs(DC_SEED[:, coil])))
    g = max(g, DC_SEED_FLOOR)             # an unmeasurably weak coil gets AMP_MAX
    amp = target / (g * ramp_gain(f, seconds))
    return float(np.clip(amp, AMP_MIN, AMP_MAX)), g


# ===========================================================================
# acquisition
# ===========================================================================
def acquire(dac, rdr, rec, seconds, drive=None, coil=None, t0=None,
            guard=None, on_guard=None, hold_v=None, t_ref=None):
    """Hold or modulate ONE coil for `seconds` and collect samples.

    `drive(t) -> volts about bias`, or None to hold. `t0` lets a caller keep ONE
    continuous time base across pre/drive/unwind, which is what makes the
    anti-phase unwind actually cancel and the pulse's residual subtraction have
    a meaningful phase. `t_ref` is where `drive`'s own argument is measured from
    when that differs from the shared origin -- the pulse needs the shared
    origin for its phasors and its own window start for its envelope.

    `guard` is a boolean mask over sensors, or None for no guard. It is a MASK
    and not a flag because a sensor already resting outside the guard band --
    a2 near 860 counts on 2026-08-20 -- would otherwise trip it on the first
    sample of every point, forever. `guard_mask` builds it.

    `hold_v` is the volts the coil is ALREADY held at when `drive` is None. It
    only affects what gets recorded, and it has to be right: the DC pass holds
    a coil at bias +/- 0.15 V, and recording that segment as `u = bias` would
    put a step into the file that the hardware never saw.

    Returns (t, counts, u, tripped) with `u` the volts ACTUALLY commanded at
    each sample, less bias -- which is what `response` divides by.
    """
    origin = time.time() if t0 is None else t0
    start = time.time() - origin
    base = start if t_ref is None else t_ref
    ch = None if coil is None else DAC_MAP[coil]
    nxt = start
    held = bias_of(coil) if hold_v is None else clamp(hold_v)
    ts, rows, us = [], [], []
    tripped = False
    last_prog = start
    while True:
        now = time.time() - origin
        if now - start >= seconds:
            break
        # A live line, because this loop used to be SILENT. Three header-only
        # recordings were made on 2026-08-17 before anybody could tell that no
        # samples were arriving: `rdr.read()` returns None on a timeout and the
        # loop just spins, so a dead stream and a healthy one looked identical
        # for 60 s -- and would have looked identical for 36 minutes on `coils`.
        # A rate of 0 Hz here is now visible within PROGRESS_EVERY_S.
        if PROGRESS_EVERY_S and now - last_prog >= PROGRESS_EVERY_S:
            last_prog = now
            el = max(now - start, 1e-9)
            print("      +%4.0fs/%.0fs  %6.1f Hz  counts %s"
                  % (el, seconds, len(ts) / el,
                     " ".join("%4.0f" % v for v in
                              (rows[-1] if rows else [0] * NCOLS))),
                  flush=True)
        if drive is not None and now >= nxt:
            nxt = now + 1.0 / UPDATE_HZ
            held = clamp(bias_of(coil) + drive(now - base))
            dac.set_voltage(ch, held)
        s = rdr.read()
        if s is None:
            continue
        t = time.time() - origin
        ts.append(t)
        rows.append(s)
        us.append(held)
        if rec is not None:
            rec.write(t, s, held)
        if guard is not None:
            w = np.asarray(s)[guard]
            if w.size and (w.min() <= RAIL_GUARD_LOW or w.max() >= RAIL_GUARD_HIGH):
                tripped = True
                if on_guard is not None:
                    on_guard(t, s)
                break
    out = (np.asarray(ts), np.asarray(rows, float),
           np.asarray(us, float) - bias_of(coil))
    return out + (tripped,)


def guard_mask(counts, log=print):
    """Which sensors the rail guard may watch: those resting inside the band.

    A sensor outside it is either genuinely pinned or simply parked high, and
    either way it cannot be a trip source. Reported, not silently dropped.
    """
    m = counts.mean(axis=0) if len(counts) else np.full(NCOLS, MID_COUNTS)
    keep = in_guard_band(m)
    # A SENSOR PARKED TOO CLOSE TO ITS RAIL CANNOT BE ALLOWED TO VETO THE WHOLE
    # MEASUREMENT. Measured 2026-08-20: a2 rests near 860 counts against a guard
    # band of 80..943, so it has ~83 counts of headroom, and a coil pass tripped
    # the guard at 963 and 972 and retried the drive down 0.0283 -> 0.0141 ->
    # 0.0071 -> 0.0035 V. An 8x reduction buries the response in noise, so a2 was
    # costing the pass every point it was supposed to measure.
    # Phi is 4x3 and a0/a1/a3 alone span all three modes, so dropping a2 from the
    # GUARD still leaves a determined measurement -- it is a trip source that is
    # removed, not a row. Its own response is still recorded and still exported;
    # what is given up is a2's protection, which for a sensor already sitting
    # 350 counts off mid-scale is protection against nothing.
    keep[list(GUARD_EXCLUDE)] = False
    if not keep.all():
        log("      guard watches %s; %s rest outside the guard band and cannot"
            " be a trip source"
            % ([("a%d" % i) for i in np.where(keep)[0]],
               [("a%d@%.0f" % (i, m[i])) for i in np.where(~keep)[0]]))
    return keep


def park(dac, coils=range(NCOLS)):
    for c in coils:
        try:
            dac.set_voltage(DAC_MAP[c], clamp(BIAS_CH[c]))
        except Exception:
            pass


def park_ramped(dac, coils=range(NCOLS), seconds=BIAS_RAMP_S):
    """Bring the coils to bias SLOWLY. A bias step is a force step.

    MEASURED 2026-08-18, and it is why this exists. The board zeroes every coil
    on reset (arduino.ino:363) and the reset fires on port open, so after a
    handover the coils sit at 0 V and parking them steps four coils to 0.25 V at
    once. That is a force step into an undamped plant -- exactly the principle
    `_dc_point` relies on to measure the DC matrix -- and it re-excites what the
    settle just removed. The evidence: |pre| at mode A ROSE from 0.10-0.21 V
    (no settling, 23:12 pass) to 0.32-0.36 V (with settling, 01:18 pass), i.e.
    the handover was undoing more than the damping achieved. Mode A is worst
    because T1 carries the most ambient energy of the three (24.3 counts rms
    against Z 11.1 and T2 6.5).

    The slowest mode is 0.72194 Hz, a 1.4 s period, so a ramp over `seconds`
    covers ~10 periods. Excitation by a ramp falls with its duration; a step is
    the worst case and a slow ramp is adiabatic.
    """
    n = max(int(seconds * BIAS_RAMP_HZ), 1)
    for k in range(1, n + 1):
        for c in coils:
            try:
                dac.set_voltage(DAC_MAP[c], clamp(BIAS_CH[c] * k / float(n)))
            except Exception:
                pass
        time.sleep(1.0 / BIAS_RAMP_HZ)
    park(dac, coils)


# ===========================================================================
# pass 1 -- the passive sensor census
# ===========================================================================
def sensors_analyse(t, counts, log=print):
    """Per sensor: is it in range, is it moving, and is what it sees the optic?

    The last question is the one this repo has got wrong twice. `README` Sec 0
    keeps the methodological note and it is worth restating: the original
    "a4-a7 are not wired" call came from a DC-response threshold and a raw
    time-domain correlation, and BOTH ARE BLIND TO A SMALL COHERENT SIGNAL.
    So the test here is a lock-in at the three known mode frequencies against
    an off-mode floor measured in the same record, never a threshold on the
    raw standard deviation.
    """
    res = []
    span = t[-1] - t[0] if len(t) > 1 else 0.0
    for i in range(NCOLS):
        c = counts[:, i]
        v = c * COUNTS_TO_V
        v = v - v.mean()
        row = dict(ch=i, n=len(c), mean=float(c.mean()), std=float(c.std()),
                   lo=float(c.min()), hi=float(c.max()),
                   railed=float(((c <= RAIL_LOW) | (c >= RAIL_HIGH)).mean()),
                   up=float(RAIL_HIGH - c.mean()), down=float(c.mean() - RAIL_LOW))
        # Off-mode floor: the same lock-in, same window, evaluated where no mode
        # is. One estimator throughout, so the ratio is a real SNR rather than
        # two numbers arrived at by two different methods.
        vd = decohere(t, v)
        row["floor_v"] = float(np.median([noise_at(t, vd, f) for _, f in MODES]))
        row["modes"] = {}
        for name, f in MODES:
            # WHERE IS THE PEAK? A shift means the modes have moved since
            # 2026-08-06 and every number in this tool with a frequency in it
            # needs re-deriving; epsilon's docstring asks for this check each
            # session and 2026-08-17 was the first time anybody ran it.
            #
            # The window was +/-0.010 Hz, and that was too narrow twice over. On
            # a 90 s record 1/T = 0.011 Hz, so the whole scan was under one
            # resolution element -- and mode C's peak landed exactly on the upper
            # edge, which means the scan RAILED and could not say where the mode
            # was, only that it was at least that far away. A peak at the edge of
            # its search window is not a peak that has been found, so the width
            # is now a named constant and hitting the edge is reported.
            grid = np.linspace(f - MODE_SCAN_HZ, f + MODE_SCAN_HZ, MODE_SCAN_N)
            mags = np.array([abs(lockin(t, v, g)) for g in grid])
            k = int(np.argmax(mags))
            fp = float(grid[k])
            # Measure the mode WHERE IT IS. The lock-in used to be evaluated at
            # the stale frequency, and at 5.3 half-widths off resonance that
            # reads ~19% of the true amplitude -- understating every SNR in the
            # table and grading healthy sensors down for the rig's drift.
            z, sd = lockin_sigma(t, v, fp)
            row["modes"][name] = dict(
                f=f, f_peak=fp, shift=fp - f,
                scan_railed=bool(k == 0 or k == len(grid) - 1),
                amp_v=float(abs(z)), sigma_v=float(sd),
                snr=float(abs(z) / row["floor_v"]) if row["floor_v"] > 0 else 0.0,
                phase_deg=float(np.degrees(np.angle(z))))
        best = max(row["modes"].values(), key=lambda m: m["snr"])
        row["best_snr"] = best["snr"]
        row["verdict"] = ("LIVE" if best["snr"] >= MIMO_SNR else
                          "WEAK" if best["snr"] >= ROW_SNR else "BLIND")
        if row["railed"] > RAIL_FRAC_REJECT:
            row["verdict"] = "RAILED"
        if row["std"] < FLAT_STD_COUNTS:
            row["verdict"] = "FLAT"
        res.append(row)

    log("\n  === SENSORS: %d samples, %.1f s, %.0f Hz ===\n"
        % (len(t), span, len(t) / span if span > 0 else 0))
    log("   ch   rest    std   headroom      railed   floor  %s   verdict"
        % "".join("  %s@%.4f" % (n, f) for n, f in MODES))
    log("      counts counts  up/down          frac      V      SNR       SNR"
        "       SNR")
    for r in res:
        log("   a%d  %6.1f %6.2f  %4.0f/%-4.0f   %6.4f  %6.4f  %7.1f   %7.1f   %7.1f   %s"
            % (r["ch"], r["mean"], r["std"], r["up"], r["down"], r["railed"],
               r["floor_v"], r["modes"]["A"]["snr"], r["modes"]["B"]["snr"],
               r["modes"]["C"]["snr"], r["verdict"]))

    log("\n  in-band amplitude at each mode, volts rms-equivalent (+/- 1 sigma):")
    log("   ch   " + (" " * 10).join("%s %.4f" % (n, f) for n, f in MODES))
    for r in res:
        log("   a%d  %8.5f+-%-7.5f %8.5f+-%-7.5f %8.5f+-%-7.5f"
            % (r["ch"],
               r["modes"]["A"]["amp_v"], r["modes"]["A"]["sigma_v"],
               r["modes"]["B"]["amp_v"], r["modes"]["B"]["sigma_v"],
               r["modes"]["C"]["amp_v"], r["modes"]["C"]["sigma_v"]))

    log("\n  peak location, +/-%.3f Hz around the MODES above. A shift here"
        % MODE_SCAN_HZ)
    log("  invalidates every frequency in this file and in the controllers:")
    log("   ch      A found      B found      C found")
    for r in res:
        log("   a%d   %10.5f   %10.5f   %10.5f"
            % (r["ch"], r["modes"]["A"]["f_peak"], r["modes"]["B"]["f_peak"],
               r["modes"]["C"]["f_peak"]))

    # The consensus over the sensors that can actually see, because that is what
    # a re-measurement is: a5-a7 return the argmax of their own noise and would
    # drag a plain mean anywhere. Half-width is f/(2Q), so a shift in half-widths
    # is the number that says whether a drive at the old frequency still lands on
    # the mode -- and it is the statistic `gains.json` was destroyed by.
    seeing = [r for r in res if r["best_snr"] >= MIMO_SNR]
    log("\n  " + INK("=== HAVE THE MODES MOVED? ===", "bold"))
    if not seeing:
        log("  no sensor sees a mode well enough to locate one.")
    else:
        log("  consensus over the %d sensors above SNR %.0f: %s"
            % (len(seeing), MIMO_SNR, ",".join("a%d" % r["ch"] for r in seeing)))
        log("   mode        MODES           now        shift    half-widths   spread")
        for name, f in MODES:
            fs = np.array([r["modes"][name]["f_peak"] for r in seeing])
            fp = float(np.median(fs))
            hw = f / (2.0 * Q_FLOOR)                     # f/(2Q), Q at its 1s floor
            nhw = abs(fp - f) / hw
            rail = any(r["modes"][name]["scan_railed"] for r in seeing)
            log("    %s     %10.5f  %10.5f   %+8.5f     %7.1f   %.5f%s"
                % (name, f, fp, fp - f, nhw, float(fs.max() - fs.min()),
                   INK("  <-- SCAN RAILED, the peak is outside the window",
                       "red") if rail else
                   INK("  <-- OFF RESONANCE", "red") if nhw > 1.0 else ""))
        log("  A drive at the old frequency reaches 1/sqrt(1+n^2) of the on-peak")
        log("  response at n half-widths, and its PHASE is wrong -- which is half")
        log("  of what an actuation matrix is for. Above 1 half-width, re-measure")
        log("  before driving: `--freqs` takes the `now` column.")

    live = [r["ch"] for r in res if r["verdict"] == "LIVE"]
    weak = [r["ch"] for r in res if r["verdict"] == "WEAK"]
    log("\n  LIVE  (SNR >= %.0f at some mode): %s" % (MIMO_SNR, live or "none"))
    log("  WEAK  (%.0f <= SNR < %.0f):          %s" % (ROW_SNR, MIMO_SNR,
                                                       weak or "none"))
    log("  other:                         %s"
        % ([("a%d:%s" % (r["ch"], r["verdict"])) for r in res
            if r["verdict"] not in ("LIVE", "WEAK")] or "none"))
    xt = crosstalk(t, counts, log=log) or {}
    prof = band_profile(t, counts)
    for i, r in enumerate(res):
        r["xtalk"] = xt.get(r["ch"])
        r["profile"] = prof[i] if prof else None

    if prof:
        log("\n  === WHERE IS EACH CHANNEL'S POWER? ===")
        log("  Share of 0.05-60 Hz power per band, and underneath it that share")
        log("  divided by the band's BANDWIDTH share. White noise gives 1.0 in")
        log("  every band by construction, so the second number is the one that")
        log("  says signal -- a railed channel can hold half the power in")
        log("  20-60 Hz purely because that band is 67% of the range.")
        log("  DC-0.5 is excluded from the verdict: a railed line wanders, and")
        log("  drift is not a degree of freedom. It is shown, not counted.")
        log("   ch  " + "".join("%9s" % b[0] for b in PROFILE_BANDS) + "   verdict")
        for i, r in enumerate(res):
            p = prof[i]
            log("   a%d  %s"
                % (r["ch"], "".join("%8.1f%%" % (100 * p["bands"][b[0]][0])
                                    for b in PROFILE_BANDS)))
            log("       " + "".join("%8.1fx" % p["bands"][b[0]][1]
                                    for b in PROFILE_BANDS)
                + "   " + (INK("structure at %s" % p["best"], "green")
                           if p["structured"] else
                           INK("WHITE above DC -- no structure anywhere", "red"))
                + ("" if p["dc_ratio"] < PROFILE_STRUCTURE else
                   INK("  (+%.0fx drift below 0.5 Hz)" % p["dc_ratio"], "yellow")))

    census_sensors(res, log=log)
    return res


COH_BANDS = (("modes 0.6-1.8Hz", 0.6, 1.8),
             ("3-6 Hz", 3.0, 6.0),
             ("6.19 line", 5.9, 6.5),
             ("8-20 Hz", 8.0, 20.0),
             ("30-60 Hz", 30.0, 60.0))


def crosstalk(t, counts, log=print, nfft=8192):
    """In-band vs out-of-band coherence against a0-a3. Mechanical or electrical?

    THIS IS THE CHECK THE REPO HAS NEEDED TWICE. "a4-a7 are not wired" and "the
    flags are misaligned" both stood for days as confident statements with no
    measurement under them (`CLAUDE.md`, standing practice), and both came from
    tests -- a DC threshold, a raw time-domain correlation -- that cannot tell
    a weak real sensor from a wiring artefact.

    Coherence can, because the two have OPPOSITE frequency signatures:

        electrical (shared ground, shared cable, ADC crosstalk)
            -> a scaled copy at EVERY frequency. Coherence is flat and high
               out of band as well as in it.
        mechanical (the sensor is genuinely watching the optic)
            -> coherence only where the optic actually moves, i.e. in the
               0.6-1.8 Hz mode band, and at the floor everywhere else.

    A sensor can therefore be near-blind and still real, which is exactly the
    case this rig kept mis-calling. The contrast between the bands is the
    answer, never the absolute level.

    The bias floor is 1/K for K segments; anything at that level is zero.
    """
    t = np.asarray(t, float)
    fs = len(t) / (t[-1] - t[0])
    x = np.asarray(counts, float) - np.asarray(counts, float).mean(axis=0)
    if len(x) < 4 * nfft:
        nfft = max(256, 1 << int(math.log2(max(len(x) // 4, 256))))
    win = np.hanning(nfft)
    idx = range(0, len(x) - nfft, nfft // 2)
    F = np.array([np.fft.rfft(x[i:i + nfft] * win[:, None], axis=0) for i in idx])
    if len(F) < 8:
        log("\n  crosstalk: record too short for a coherence estimate.")
        return None
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    S = np.einsum('kfi,kfj->fij', F, F.conj()) / len(F)
    K = len(F)

    log("\n  === MECHANICAL OR ELECTRICAL? ===")
    log("  magnitude-squared coherence against a0-a3, %d Hann segments of %d"
        " (%.3f Hz bins). Bias floor is 1/K = %.3f." % (K, nfft, f[1], 1.0 / K))
    log("  A sensor watching the OPTIC is coherent only in the mode band. A")
    log("  sensor wired alongside another is coherent at every frequency.")
    verdict = {}
    for i in range(NCOLS):
        log("\n   a%d vs        %s" % (i, "".join("   a%d   " % j for j in range(4)
                                                  if j != i)))
        vals = {}
        for label, lo, hi in COH_BANDS:
            m = (f > lo) & (f < hi)
            row = []
            for j in range(4):
                if j == i:
                    continue
                c = (np.abs(S[:, i, j]) ** 2
                     / (S[:, i, i].real * S[:, j, j].real + 1e-30))
                row.append(float(c[m].mean()))
            vals[label] = row
            log("    %-16s %s" % (label, " ".join("%6.3f" % v for v in row)))
        # PER PARTNER, not maxed over partners. a4 is coherent out of band with
        # a1 alone (0.246) and with nobody else (0.011-0.048); maxing over
        # partners lets that one contaminated pair mask the three clean ones and
        # returns AMBIGUOUS for a sensor whose contrast against a3 is 10.8x.
        floor = 1.0 / K
        partners = [j for j in range(4) if j != i]
        inb = dict(zip(partners, vals["modes 0.6-1.8Hz"]))
        oob = {j: max(vals[k][n] for k, _, _ in COH_BANDS
                      if k != "modes 0.6-1.8Hz")
               for n, j in enumerate(partners)}

        # Order matters: "in-band coherence with nothing" has to be tested
        # BEFORE the electrical branch, or a6 -- 0.031 in band, 0.588 with a2 at
        # 30-60 Hz -- is called blind and its wiring artefact goes unreported.
        if all(inb[j] < 3.0 * floor for j in partners):
            v = "BLIND -- no in-band coherence with any sensor on the optic"
            hot = [j for j in partners if oob[j] > 6.0 * floor]
            if hot:
                v += "; but coupled OUT of band to %s (%s) -- wiring, not optic" \
                     % (", ".join("a%d" % j for j in hot),
                        ", ".join("%.3f" % oob[j] for j in hot))
        else:
            con = {j: inb[j] / max(oob[j], floor) for j in partners}
            med = float(np.median([con[j] for j in partners]))
            if med > 2.0:
                v = "MECHANICAL -- median in/out contrast %.1fx" % med
            elif med < 0.5:
                v = "ELECTRICAL -- median in/out contrast %.2fx; a wiring" \
                    " artefact, not the optic" % med
            else:
                v = "AMBIGUOUS -- median in/out contrast %.1fx" % med
            odd = [j for j in partners if con[j] < 2.0 <= med]
            if odd:
                v += " (but only %s against %s -- that pair shares something"
                v = v % (", ".join("%.1fx" % con[j] for j in odd),
                         ", ".join("a%d" % j for j in odd))
                v += " electrical)"
        verdict[i] = v
        log("    -> %s" % v)
    return verdict


def sensors_run(dac, seconds, rec, log=print):
    log("\n  passive census: %.0f s, all coils held at bias, nothing driven." % seconds)
    park(dac)
    rdr = Reader(dac)
    rec.mark("passive")
    t, c, _, _ = acquire(dac, rdr, rec, seconds)
    rdr.report()
    if len(t) < 100:
        sys.exit("  only %d samples -- the board is not streaming." % len(t))
    return sensors_analyse(t, c, log=log)


# ===========================================================================
# pass 2 -- the active coil census
# ===========================================================================
def _point(dac, rdr, rec, coil, name, f, amp, log, skip_unwind=False):
    """One (coil, mode) point: pre, drive, unwind, quiet. Returns a dict or None.

    ONE time origin for the whole point. The unwind is the drive with its sign
    flipped on that same origin, which is why it cancels: the ramp is linear
    and linear is reversible. Break the shared origin and the unwind pumps.
    """
    t0 = time.time()
    w = 2.0 * math.pi * f
    tripped_at = {}

    def on_guard(t, s):
        tripped_at["t"], tripped_at["counts"] = t, list(s)

    rec.mark("pre", coil, f, amp)
    _, cp, _, _ = acquire(dac, rdr, rec, PRE_S, None, coil, t0)
    watch = guard_mask(cp, log)

    rec.mark("drive", coil, f, amp)
    td, cd, ud, trip = acquire(dac, rdr, rec, DWELL_S,
                               lambda t: amp * math.sin(w * t), coil, t0,
                               guard=watch, on_guard=on_guard, t_ref=0.0)

    # Unwind whether or not it tripped. On a trip the drive has to REVERSE, not
    # stop: stopping at 950 counts leaves it ringing at 950 counts for the next
    # ten minutes and the following point measures that.
    done = td[-1] - td[0] if len(td) > 1 else DWELL_S
    if skip_unwind:
        # The anti-phase unwind removes only 14-70% (measured over the twelve
        # points of the 23:12 pass) and costs DWELL_S per point. When a real
        # controller settles between points it does the same job properly --
        # 0.0683 /s modal against the plant's 0.0072 /s -- so the unwind is pure
        # cost. Park and let the caller hand the port over.
        park(dac, [coil])
        resid = peak = float("nan")
    else:
        rec.mark("unwind", coil, f, amp)
        acquire(dac, rdr, rec, done, lambda t: -amp * math.sin(w * t), coil, t0,
                t_ref=0.0)
        rec.mark("quiet", coil, f, amp)
        tq, cq, _, _ = acquire(dac, rdr, rec, QUIET_S, None, coil, t0)
        # LIKE FOR LIKE, and it was not before. `resid` is a lock-in over a steady
        # QUIET_S window, so it is the true current amplitude. `peak` used to be a
        # lock-in over the WHOLE drive, which ramps linearly from zero and
        # therefore averages to about HALF the end-of-drive amplitude -- so the
        # test `resid <= 0.25 * peak` was really `resid <= 0.125 * final` and
        # failed points that had cancelled perfectly well. Measure the peak over
        # the last QUIET_S of the drive instead: same window length, same
        # estimator, and the ramp only covers the final ~27% there.
        m = td >= td[-1] - QUIET_S if len(td) > 8 else np.zeros(len(td), bool)
        peak = worst_lockin(td[m], cd[m], f)
        resid = worst_lockin(tq, cq, f)
        if peak > 0 and resid > QUIET_FRAC * peak:
            # Reported, never retried -- a second unwind PUMPS. `resid` goes back
            # with the point, so a contaminated measurement is identifiable
            # offline rather than guessed at.
            log("      unwind: residual %.4f V vs end-of-drive %.4f V (%.0f%%)"
                % (resid, peak, 100.0 * resid / peak))
    park(dac, [coil])

    if trip:
        log("      !! RAIL GUARD at t=%.1fs, counts %s -- unwound"
            % (tripped_at.get("t", -1), tripped_at.get("counts")))
        return None
    if len(td) < 64:
        log("      !! only %d samples -- skipped" % len(td))
        return None

    U = lockin(td, ud, f)                # what the coil was ACTUALLY given
    if abs(U) < 1e-7:
        log("      !! no drive measured on the coil -- skipped")
        return None

    bad = railed_frac(cd, rest=cp) > RAIL_FRAC_REJECT
    H, S = response(td, cd, U, f)
    swing = cd.max(axis=0) - cd.min(axis=0)
    log("      swing %s counts, |H| %s"
        % (" ".join("%4.0f" % s for s in swing),
           " ".join("%6.2f" % abs(h) for h in H)))
    if bad.any():
        log("      !! railed and discarded: %s"
            % ", ".join("a%d" % i for i in np.where(bad)[0]))
    return dict(coil=coil, mode=name, f=f, amp=amp, H=H, sigma=S, ok=~bad,
                swing=swing, n=len(td), resid=resid, peak=peak)


def _point_multi(dac, rdr, rec, coil, amps, log):
    """ONE point per coil driving ALL THREE modes at once. 4 points, not 12.

    WHY THIS EXISTS, and it is a measured failure and not a speed-up. The
    one-mode-at-a-time pass leaves each point ringing into the next: tau > 138 s
    against a 74 s point, and the anti-phase unwind removes only 14-70%. Measured
    on `data/20260817_231251_status_coils.csv`, |pre|/|drive| over all twelve
    points: 417%, 31%, 35%, 96%, 97%, 3%, 80%, 27%, 48%, 37%, 62%, 30%. Every
    point was contaminated.

    WHY NOTHING CAUGHT IT. The leftover is the SAME MODE, so it enters the
    response matrix as Psi[:,j] += phi_m * c_j -- the same mode shape with a
    different complex weight per coil -- and Psi stays EXACTLY RANK 1. The
    rank-1 fraction is structurally blind to it and read 0.9968-0.9994 on the
    very pass whose phase was ruined.

    WHAT DID CATCH IT, and this is now `phase_consistency` below: for one mode,
    every coil must respond at the SAME PHASE up to a sign. A coil sets magnitude
    and sign; the MODE sets phase. The 23:12 pass scattered by 73-89 degrees.

    Driving the three tones together removes the inter-mode carryover entirely --
    there is no "next mode" to leak into. Carryover between COILS survives, which
    is why there are still only three transitions to damp by hand.

    AND ON RESONANCE IT IS A NEGATIVE RESULT. Run 2026-08-18: the per-coil phase
    spread improved (73-89 deg -> 31.6 / 36.7 / 7.8) and THE SIGNS BROKE -- 7 of
    12 agreed with the two DC passes, against 12 of 12 for the one-at-a-time
    pass. The reason is structural, not tuning: an on-resonance response RAMPS,
    so it has broad 1/f^2 skirts rather than a clean line, and the loudest mode
    leaks into the quietest mode's bin. Mode A is strongest (|H| 8.58) and mode B
    weakest (|H| 3.03), and mode B is exactly where the signs go wrong. The flag
    stays because it is the right trade OFF resonance; do not spend another 18
    minutes rediscovering that it is not the right trade on it.

    THE TONES DO NOT INTERFERE. They are 0.27 Hz apart and the window is 30 s, so
    the lock-in resolution is 0.033 Hz -- eight resolution elements between
    neighbours. Each tone is recovered against the lock-in of the RECORDED drive
    at its own frequency, which is what makes a per-tone amplitude error cancel
    (the `multisine` defect: dividing by the amplitude intended rather than the
    one commanded put a spurious offset into every recovered phase).

    Phases are Schroeder-spread so three tones do not all peak together; with the
    measured amplitudes summing to 0.06 V against AMP_MAX 0.20 V there is room
    either way, but the crest factor is free to fix and a railed point is not.
    """
    t0 = time.time()
    ws = [2.0 * math.pi * f for _, f in MODES]
    # Schroeder phases: phi_k = -pi k(k-1)/n. Flat spectrum, low crest factor.
    phs = [-math.pi * k * (k - 1) / MODES_N for k in range(MODES_N)]
    tripped_at = {}

    def on_guard(t, s):
        tripped_at["t"], tripped_at["counts"] = t, list(s)

    def drive(t, sign=1.0):
        return sign * sum(a * math.sin(w * t + p)
                          for a, w, p in zip(amps, ws, phs))

    # A LONG PRE WINDOW, and the length is the whole point. The leftover ring is
    # an initial condition and it has to be measured as well as the response is,
    # or subtracting it adds more noise than it removes -- which is exactly what
    # happened when this was tried at PRE_S = 6 s against the 23:12 pass: the
    # off-quadrature fraction went 0.65/0.83/1.23 -> 1.26/1.10/1.19, because 6 s
    # is 4.3 cycles of mode A. At MULTI_PRE_S the pre window matches the drive
    # window, so the two phasors carry comparable SNR and the difference is
    # meaningful. Affordable only because the multisine turned twelve points
    # into four.
    rec.mark("pre", coil, -1.0, max(amps))
    tp, cp, _, _ = acquire(dac, rdr, rec, MULTI_PRE_S, None, coil, t0)
    watch = guard_mask(cp, log)

    rec.mark("drive", coil, -1.0, max(amps))
    td, cd, ud, trip = acquire(dac, rdr, rec, DWELL_S, drive, coil, t0,
                               guard=watch, on_guard=on_guard, t_ref=0.0)

    done = td[-1] - td[0] if len(td) > 1 else DWELL_S
    rec.mark("unwind", coil, -1.0, max(amps))
    acquire(dac, rdr, rec, done, lambda t: drive(t, -1.0), coil, t0, t_ref=0.0)
    rec.mark("quiet", coil, -1.0, max(amps))
    tq, cq, _, _ = acquire(dac, rdr, rec, QUIET_S, None, coil, t0)
    park(dac, [coil])

    if trip:
        log("      !! RAIL GUARD at t=%.1fs, counts %s -- unwound"
            % (tripped_at.get("t", -1), tripped_at.get("counts")))
        return []
    if len(td) < 64:
        log("      !! only %d samples -- skipped" % len(td))
        return []

    out = []
    bad = railed_frac(cd, rest=cp) > RAIL_FRAC_REJECT
    for mi, (name, f) in enumerate(MODES):
        U = lockin(td, ud, f)            # the RECORDED drive, at THIS tone
        if abs(U) < 1e-7:
            log("      !! mode %s: no drive measured at that tone -- skipped" % name)
            continue
        Hraw, S = response(td, cd, U, f)
        # One shared time origin covers pre and drive -- `_point`'s own note says
        # that is what makes the anti-phase unwind cancel -- so a free oscillation
        # is a CONSTANT phasor across both windows, decaying only at gamma. The
        # subtraction is therefore arithmetic, not a model.
        dec = math.exp(-MODE_GAMMA[name] * (float(td.mean()) - float(tp.mean())))
        vp = to_volts(cp)
        H = np.array([Hraw[i] - (lockin(tp, vp[:, i], f) if len(tp) > 8 else 0j)
                      * dec / U for i in range(NCOLS)])
        # Residual at this tone, and the leftover it started from. Both reported
        # per tone so a contaminated point is identifiable offline rather than
        # guessed at -- the 23:12 pass had no such number until it was replayed.
        pre = worst_lockin(tp, cp, f)
        resid = worst_lockin(tq, cq, f)
        m = td >= td[-1] - QUIET_S if len(td) > 8 else np.zeros(len(td), bool)
        peak = worst_lockin(td[m], cd[m], f)
        log("      %s @ %.4f  |H| %s" % (name, f, " ".join("%6.2f" % abs(h) for h in H)))
        log("           started on %.0f%% of its own drive, ended on %.0f%%"
            % (100.0 * pre / peak if peak > 0 else 0.0,
               100.0 * resid / peak if peak > 0 else 0.0))
        out.append(dict(coil=coil, mode=name, f=f, amp=amps[mi], H=H, sigma=S,
                        ok=~bad, swing=cd.max(axis=0) - cd.min(axis=0),
                        n=len(td), resid=resid, peak=peak, pre=pre, H_raw=Hraw))
    if bad.any():
        log("      !! railed and discarded: %s"
            % ", ".join("a%d" % i for i in np.where(bad)[0]))
    return out


MULTI_PRE_S = 30.0            # pre window for the multisine point. Same length
                              # as the drive, so the leftover ring is measured to
                              # the same SNR as the response it contaminates.
# ---- settling between points, by the CONTROLLER rather than by hand ---------
# 2026-08-18. A hand-rolled diagonal loop was tried here first and measured
# NEUTRAL on hardware: 55.45 -> 53.73 counts over 45 s, i.e. it did nothing --
# and it ran with the INVERTED sign convention (see SENSOR_H), so neutral was the
# best it could have done. The real controller is measured at 0.0683 /s modal and
# 0.0434 /s diagonal against the plant's intrinsic 0.0072 /s, so it takes a ring
# down 10x in 34 s or 53 s. Use that rather than a second implementation.
#
# WHY THIS IS SAFE EVEN THOUGH THE BREAKER IS UNFIXED. The breaker trips on
# `high AND growing` sustained RUNAWAY_SUSTAIN_S. What killed both runs of
# 2026-08-18 was a HAND KICK, which keeps growing while the gain is live. A ring
# left over from a coil drive is high but DECAYING, so `growing` goes false and
# the trip never arms. Settling after a drive is the case the breaker was built
# to tolerate; a kick during damping is the case it gets wrong.
SETTLE_VERSION = "eta"
# CHANGED FROM "zeta" 2026-08-20, AND IT WAS PUMPING THE PLATE BETWEEN POINTS.
# `osem.zeta.py:191` still carries `STEADY_GAIN = [-0.035, -0.035, +0.035,
# -0.035]`, which is the pre-2026-08-20 INVERTED sign convention: dissipation
# requires `slope x gain > 0` and that vector is the negated slope, measured at
# 5.4 / 11.0 / 22.6 / 12.3 % of steps dissipating against 90-97 % once corrected.
# Its ch3 entry is stale on top of that, since a3's physical slope flipped in the
# re-seating (+81.08 -> -98.79 counts/V). So handing the port to zeta to "settle"
# between coil points drove the modes UP, which contaminates the very next point
# -- exactly the leftover-ring problem the settle exists to remove.
# `eta` carries the corrected, slope-normalised gains and is the rung that has
# actually locked on this rig (13.2 s, ratios 0.05-0.09, 2026-08-20 18:56).
SETTLE_CTL_S = 40.0           # 34 s is 10x at the measured MODAL rate 0.0683 /s,
                              # plus ~2 s warm-up when the stored baseline is
                              # fresh (BASELINE_FILE_MAX_AGE_S = 1800 s), which it
                              # is when the pass relaunches every ~75 s
PORT_SETTLE_S = 3.0           # both sides of a handover; measured, see below
SETTLE_GRACE_S = 10.0         # SIGINT grace before SIGKILL. Long enough for a
                              # clean exit to flush its log, short enough that a
                              # hung teardown does not cost the pass. Measured:
                              # 90 s was not enough, because the hang is
                              # unbounded, not slow.

SETTLE_KP = 0.030             # validated on hardware; -0.040 is the rail onset
SETTLE_MAX_S = 90.0           # ceiling; it exits early on the amplitude test
SETTLE_TARGET = 0.15          # of the amplitude it started at
SETTLE_MIN_S = 8.0            # never call it settled off one window
SETTLE_HZ = 100.0             # control clock, DECOUPLED from the ~420 Hz wire
SETTLE_RATE_HZ = 350.0        # nominal wire rate; it only sizes the rms window
SETTLE_WARMUP_S = 3.0         # the estimator starts at the first sample, so
                              # measuring the starting amplitude before it has
                              # converged under-reports it and every later ratio
                              # is wrong
SETTLE_ABORT = 1.5            # x the starting amplitude -> stop, it is pumping


def settle_controller(seconds=SETTLE_CTL_S, version=SETTLE_VERSION, log=print):
    """Hand the port to a real controller for `seconds`, then take it back.

    Launching `bench.py <version>` as a subprocess is the only way to get the
    measured damping without a second implementation of the loop drifting out of
    step with the first (the since-deleted `signtest.py` did the same).

    THE CALLER MUST HAVE CLOSED ITS OWN PORT. Opening resets the board (~2 s), so
    the coils return to a known state on both handovers; park before closing.

    SIGINT, NEVER SIGTERM: the controller parks its coils in a `finally` and
    SIGTERM does not unwind it. The coils were left energised three times on
    2026-08-17/18 by exactly that. The wait is long for the same reason.
    """
    log("    settling: handing the port to `%s` for %.0f s" % (version, seconds))
    # THE BOARD DOES NOT GO QUIET THE INSTANT THE PORT CLOSES. Measured
    # 2026-08-18: handing over immediately made the controller exit rc=1 -- its
    # READY probe saw leftover stream bytes rather than READY -- and the
    # subsequent reopen then hit a board mid-reset and raised. Both sides of the
    # handover need the line to drain.
    time.sleep(PORT_SETTLE_S)
    env = dict(os.environ)
    proc = None
    for attempt in range(2):
        # start_new_session so the SIGINT can be delivered to the whole GROUP.
        # `bench.py` runs the controller as a CHILD, so signalling the wrapper
        # alone leaves the loop running and holding the port -- measured
        # 2026-08-18, it survived 90 s of waiting. jerk.py hit the same thing and
        # had to escalate to SIGTERM, which skips the coil park.
        proc = subprocess.Popen([sys.executable, "bench.py", version],
                                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=env,
                                start_new_session=True)
        time.sleep(6.0)                      # its own preflight + port open
        if proc.poll() is None:
            break
        log("      `%s` exited rc=%s on attempt %d -- the line was probably not"
            " drained; retrying once." % (version, proc.returncode, attempt + 1))
        time.sleep(PORT_SETTLE_S * 2)
    if proc is None or proc.poll() is not None:
        log("      could not hand the port over. Continuing WITHOUT settling --"
            " the next point's `pre` records what it started from.")
        time.sleep(PORT_SETTLE_S)
        return False
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            if proc.poll() is not None:
                log("      the controller exited early (rc=%s) -- continuing;"
                    " the next point's `pre` records what it started from."
                    % proc.returncode)
                return False
            time.sleep(0.5)
        # SIGINT FIRST, BRIEFLY, then SIGKILL. A clean exit flushes the log and
        # prints the run summary, so it is worth a short wait -- but it CANNOT be
        # depended on: every controller's `finally` calls dac.stop_stream()
        # BEFORE parking its coils, and stop_stream blocks waiting for an ACK
        # that a busy board may never send. Measured 2026-08-18: the controller
        # survived 90 s of SIGINT, to the process and to the process group, and
        # held the port the whole time.
        #
        # WHY KILLING IT IS SAFE, and this is the part that makes the trade work:
        # the board RESETS when the port is opened, and arduino.ino:363 runs
        #     for (uint8_t c = 0; c < 8; c++) writeCode(c, 0);
        # before anything else, so every coil goes to 0 V -- no drive, no current.
        # The caller then reopens and parks to bias. That is a STRONGER guarantee
        # than the teardown it replaces, because it does not depend on the dying
        # process doing anything at all.
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=SETTLE_GRACE_S)
            log("      settled for %.0f s, exited cleanly." % (time.time() - t0))
        except subprocess.TimeoutExpired:
            log("      did not exit in %.0fs -- SIGKILL. The board zeroes every"
                " coil on the reset that the next open triggers, and the caller"
                " parks to bias after that." % SETTLE_GRACE_S)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                proc.kill()
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                log("      !! it survived SIGKILL -- check the coils by hand.")
                return False
        time.sleep(PORT_SETTLE_S)
        return True
    except KeyboardInterrupt:
        # Ctrl-C on the PASS must not leave a controller holding the port. Same
        # rule as above: ask nicely, then kill, and let the board's own reset
        # zero the coils.
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=SETTLE_GRACE_S)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                proc.kill()
        raise


SETTLE_TAU_S = 0.10           # transport delay to compensate, seconds. From the
                              # 23:12 coil pass: 127 / 112 / 86 ms implied at the
                              # three modes. NOT well determined -- the three do
                              # not agree, so this is an order, and `--settle-tau`
                              # exists to measure it against damping achieved.
DEMOD_HZ = 0.15               # demodulator lowpass. 1/DEMOD_HZ = 6.7 s is several
                              # periods of every mode, so the phasor tracks a
                              # ringdown (tau > 138 s) without smearing it.
MODAL_DEMAND_CAP_V = 0.20     # per-vector, uniform; same number the controllers use


class ModalDamper:
    """The modal law, in-process, with a narrowband velocity estimate.

    WHY NOT REUSE THE CONTROLLER. Handing the port to `bench.py` works and damps
    (3-8x per 40 s window, measured 2026-08-18), but the handover itself ruins the
    next measurement: the board RESETS on port open and arduino.ino:363 zeroes
    every coil, so the coils step 0.25 V -> 0 -> 0.25 V around each settle. A bias
    step is a FORCE step -- that is the principle `_dc_point` measures the DC
    matrix with -- so the handover re-excites what the settle just removed.
    Measured: |pre| at mode A ROSE from 0.10-0.21 V (no settling) to 0.32-0.36 V
    (with settling), and sign agreement against the two DC passes fell 12/12 ->
    10/12. In-process there is no reset and no step.

    WHY THE ESTIMATOR IS NARROWBAND. Bandpassing and differentiating measured
    NEUTRAL on hardware (55.45 -> 53.73 counts over 45 s): differentiating
    broadband noise to recover a velocity at a KNOWN frequency throws the
    knowledge away. Here each mode is demodulated against its own e^{-i w t}, so
    the modal coordinate becomes a slowly varying complex phasor z_m and

        q_m(t)     = 2 Re[z_m e^{i w t}]
        qdot_m(t)  = -2 w Im[z_m e^{i w t}]

    exactly. The only lag is the demodulator's lowpass, and at DEMOD_HZ against
    a mode at 0.72-1.66 Hz that lag is negligible IN THE BAND THAT MATTERS,
    because the signal being filtered sits at DC after demodulation. Phase is
    what decides whether feedback dissipates or pumps, so this is the whole
    design.

    Dissipation: f = -K qdot is anti-parallel to the modal velocity by
    construction, and the allocator realises it exactly while no coil clips --
    which is why the total demand is capped, not the modal part (CLAUDE.md Sec 2:
    the derivative term added after the cap took coil 3 to 169% of its window).

    AND IT HAS NOT BEEN SHOWN TO WORK. Measured 2026-08-18 over nine values of
    tau, the best was 0.68 of the starting amplitude over 45 s -- 0.0086 /s
    against the plant's own 0.0072 /s, i.e. nothing, where a controller does
    0.0683 /s. A demodulator locked to an ASSUMED frequency produces a phasor
    that rotates at the difference, so its velocity phase error grows without
    bound: mode B moved 1.3 half-widths in one night (0.99193 -> 0.99343 Hz),
    which is 24 degrees of drift across a 45 s settle and still accumulating. No
    fixed `tau` can track that. Treat `--settle` as UNPROVEN, and prefer
    `settle_controller`, which hands the port to a rung that is measured.
    """

    def __init__(self, path=None, kp=None, tau=None, phase_deg=None, log=print):
        self.ok, self.why = False, ""
        path = MODAL_PATH if path is None else path   # bound at call time: the
        # constant is defined with the export code, further down the file
        try:
            d = json.load(open(path))
        except Exception as exc:
            self.why = "no usable %s (%s)" % (path, type(exc).__name__)
            return
        if not d.get("a"):
            self.why = "%s has no A block" % path
            return
        if d["a"].get("basis") != "geometric":
            self.why = ("A in %s is basis %r, not geometric -- pairing it with a "
                        "geometric Phi is the mismatch that pumped on 2026-08-17"
                        % (path, d["a"].get("basis")))
            return
        self.phi = np.array(d["phi"]["value"], float)          # (n, nm)
        A = np.array(d["a"]["value"], float)                   # (nm, n)
        self.coils = [int(c) for c in d["a"]["coils"]]
        self.f = np.array([m["f_hz"] for m in d["modes"]], float)
        self.w = 2.0 * np.pi * self.f
        self.nm = len(self.f)
        Ac = A[:, self.coils]
        # Unit-norm rows, exactly as the controllers do, so the gain means the
        # same thing here as it does there.
        nrm = np.linalg.norm(Ac, axis=1, keepdims=True)
        nrm[nrm == 0] = 1.0
        self.Ac = Ac / nrm
        self.P = np.linalg.pinv(self.Ac)                       # (ncoil, nm)
        self.kp = float(SETTLE_KP if kp is None else kp)
        self.tau = float(SETTLE_TAU_S if tau is None else tau)
        # PER-MODE PHASE, which a single tau cannot express. A delay forces the
        # correction to scale with frequency; if what is actually wrong is a
        # fixed quadrature error, every mode needs the SAME degrees. Measured
        # 2026-08-18: retarding helps monotonically (ratio 0.89 / 0.87 / 0.68 at
        # tau -50 / -100 / -150 ms), so the loop LEADS, and the question is
        # whether the right correction is constant in time or in phase.
        self.phase = (None if phase_deg is None
                      else np.radians(np.asarray(phase_deg, float)))
        self.z = np.zeros(self.nm, complex)
        self.primed = False
        self.ok = True
        log("    modal damper: %d modes, coils %s, cond(A_C) %.2f, Kp %.3f,"
            " tau %.0f ms (advance %s deg)"
            % (self.nm, self.coils, np.linalg.cond(self.Ac), self.kp,
               1e3 * self.tau,
               " ".join("%.0f" % np.degrees(w * self.tau) for w in self.w)))

    def step(self, t, counts, dt):
        """Return the volts to add to bias on `self.coils`."""
        y = (np.asarray(counts, float) - np.asarray(counts, float).mean()) * COUNTS_TO_V
        q = np.array([float(self.phi[:, m] @ y) / max(float(self.phi[:, m] @ self.phi[:, m]), 1e-12)
                      for m in range(self.nm)])
        a = min(2.0 * np.pi * DEMOD_HZ * dt, 1.0)
        e = np.exp(-1j * self.w * t)
        self.z += a * (q * e - self.z)
        if not self.primed:
            self.primed = True
            return np.zeros(len(self.coils))
        # PHASE ADVANCE FOR THE TRANSPORT DELAY. Without it this measured
        # NEUTRAL on hardware (0.2043 -> 0.2214 V) while damping 300x in a
        # zero-delay simulation, and neutral is the signature of a 90 degree
        # error: a force orthogonal to velocity does no net work.
        #
        # The delay is measured, not assumed. Excess lag beyond the -90 degrees
        # that resonance requires, from the 23:12 coil pass: 33 deg at 0.722 Hz,
        # 40 deg at 0.992 Hz, 51 deg at 1.656 Hz -- rising with frequency, which
        # is what a constant TIME delay looks like, and implying 127 / 112 / 86 ms.
        # It is NOT enough on its own -- see the tau sweep in the class docstring.
        adv = self.w * self.tau if self.phase is None else self.phase
        qdot = -2.0 * self.w * np.imag(self.z * np.exp(1j * (self.w * t + adv)))
        f = -self.kp * qdot
        u = self.P @ f
        pk = float(np.max(np.abs(u))) if len(u) else 0.0
        if pk > MODAL_DEMAND_CAP_V:                 # cap the TOTAL, uniformly:
            u = u * (MODAL_DEMAND_CAP_V / pk)       # direction preserved
        return u


def settle_modal(dac, rdr, rec, log=print, max_s=SETTLE_MAX_S,
                 target=SETTLE_TARGET, damper=None):
    """Damp in-process with the modal law. No handover, no reset, no bias step."""
    if damper is None or not damper.ok:
        log("    modal damper unavailable (%s) -- NOT settling"
            % (damper.why if damper else "not built"))
        return False
    log("    settling in-process, <=%.0f s, exit at %.0f%% of the starting"
        " amplitude" % (max_s, 100 * target))
    rec.mark("settle", -1, -1.0, 0.0)
    t0 = time.time()
    prev_t, last_send, last_report = None, -1.0, 0.0
    win, amp0, acc = [], None, []
    u = BIAS_CH.copy()
    try:
        while time.time() - t0 < max_s:
            s = rdr.read()
            if s is None:
                continue
            now = time.time() - t0
            c = np.asarray(s, float)
            rec.write(now, c, float(u[damper.coils[0]]))
            if prev_t is None:
                prev_t = now
                continue
            dt = max(now - prev_t, 1e-4)
            prev_t = now
            win.append(float(np.sqrt(np.mean(((c[:4] - c[:4].mean())
                                              * COUNTS_TO_V) ** 2))))
            if len(win) > int(2.0 * SETTLE_RATE_HZ):
                win.pop(0)
            acc.append(c)
            # 100 Hz control clock, DECOUPLED from the ~420 Hz wire, on the MEAN
            # of the samples that arrived in the period. At the sample rate the
            # SET traffic is ~182 kbit/s against a 115200 baud link: the applied
            # voltage then lags by a GROWING delay, which turned a damper into a
            # driver on 2026-08-18 (40 -> 84 counts).
            if now - last_send < 1.0 / SETTLE_HZ:
                continue
            last_send = now
            cbar = np.mean(np.asarray(acc, float), axis=0)
            acc = []
            du = np.asarray(damper.step(now, cbar, dt), float)
            for n, j in enumerate(damper.coils):
                u[j] = clamp(BIAS_CH[j] + float(du[n]))
                dac.set_voltage(DAC_MAP[j], u[j])
            if len(win) >= int(1.5 * SETTLE_RATE_HZ) and now >= SETTLE_WARMUP_S:
                a = float(np.mean(win))
                if amp0 is None:
                    amp0 = a
                if now - last_report >= 10.0:
                    log("      %4.0fs  in-band rms %.4f V (%.0f%% of start)"
                        % (now, a, 100.0 * a / amp0 if amp0 else float("nan")))
                    last_report = now
                if now >= SETTLE_MIN_S and amp0 > 0 and a <= target * amp0:
                    log("      settled: %.4f -> %.4f V in %.0f s" % (amp0, a, now))
                    return True
                if amp0 > 0 and a > SETTLE_ABORT * amp0 and now >= SETTLE_MIN_S:
                    log("      !! ABORT: %.4f -> %.4f V (%.1fx). PUMPING -- parked."
                        % (amp0, a, a / amp0))
                    return False
        log("      timeout at %.0f s: %.4f -> %.4f V"
            % (max_s, amp0 if amp0 else float("nan"),
               float(np.mean(win)) if win else float("nan")))
        return False
    finally:
        park(dac, damper.coils)


def phase_compare(res, log=print):
    """Score the raw and leftover-subtracted estimators against the SAME physics.

    Choosing between two estimators by which one fits the data better would be
    fitting. This is not that: "one mode has one phase, up to a sign" is a
    constraint the plant obeys whatever we measure, so an estimator that violates
    it is wrong independently of anything we wanted. That makes the comparison a
    test, and the number below is reported either way.
    """
    if not any("H_raw" in p for p in res):
        return None
    raw = [dict(p, H=p["H_raw"]) for p in res if "H_raw" in p]
    log("\n  --- leftover subtraction: does it help? ---")
    log("  Raw:")
    a = phase_consistency(raw, log=lambda m: log("  " + m))
    log("  Leftover subtracted:")
    b = phase_consistency(res, log=lambda m: log("  " + m))
    log("\n  worst per-coil phase spread   raw %.1f deg   subtracted %.1f deg"
        % (a, b))
    if b < a - PHASE_DIFF_DEG:
        log("  The subtraction HELPS. It is exported.")
    elif b > a + PHASE_DIFF_DEG:
        log("  The subtraction HURTS -- as it did at PRE_S = 6 s on the 23:12")
        log("  pass. Export the RAW estimator and say so; do not tune this.")
    else:
        log("  No difference beyond %.0f deg: either the leftover was small, or"
            % PHASE_DIFF_DEG)
        log("  the pre window still does not measure it well enough.")
    return a, b


def phase_consistency(res, log=print):
    """For ONE mode, every coil must respond at the SAME PHASE up to a sign.

    A coil applies a force. Force-to-modal-response phase is a property of the
    MODE, not of the coil, so a coil can only set magnitude and sign -- 0 or 180
    degrees. Anything in between is not a plant property and cannot be one.

    This is the check that caught the 23:12 pass, and nothing else did: the
    rank-1 fraction was 0.9968-0.9994 and the colocation check passed 4 of 4,
    while the per-coil modal phases scattered by 73-89 degrees:

        mode A  +129.4  -28.5  -82.4  -33.5
        mode B  +142.5  -68.1 +153.5  +91.2
        mode C  +150.7 +141.9 +120.9  -28.0

    Scored by folding each phase into [0, 180) -- which removes the legitimate
    sign freedom and nothing else -- and taking the circular spread of what is
    left. A clean pass is a few degrees. This REPORTS; it does not refuse, because
    the threshold on this rig has never been measured on a pass known to be good.
    """
    log("\n  === PHASE CONSISTENCY (one mode, one phase, up to a sign) ===")
    log("  A coil sets magnitude and sign; the MODE sets phase. Scatter here is")
    log("  contamination, not physics -- most likely a leftover ring, which stays")
    log("  EXACTLY rank 1 and is therefore invisible to the rank-1 check.")
    worst = 0.0
    for mi, (name, _) in enumerate(MODES):
        pts = sorted([p for p in res if p["mode"] == name], key=lambda p: p["coil"])
        if len(pts) < 2:
            continue
        g = GEO_PHI[:, mi]
        ang = []
        for p in pts:
            if np.abs(g).sum() > 0:
                z = complex(np.dot(g[:4], p["H"][:4]))
            else:
                k = int(np.argmax(np.abs(p["H"])))
                z = p["H"][k]
            if abs(z) > 0:
                ang.append(math.degrees(np.angle(z)) % 180.0)
        if len(ang) < 2:
            continue
        # Circular spread on a 180-degree circle: double the angles, average the
        # unit vectors, and read the resultant length. A plain std would call
        # 179 and 1 degrees "89 apart" when they are 2 apart.
        a2 = np.radians(np.array(ang) * 2.0)
        R = abs(np.mean(np.exp(1j * a2)))
        spread = math.degrees(math.sqrt(max(-2.0 * math.log(max(R, 1e-12)), 0.0))) / 2.0
        worst = max(worst, spread)
        log("   mode %s  per-coil phase (mod 180) %s   spread %5.1f deg  %s"
            % (name, " ".join("%6.1f" % a for a in ang), spread,
               "OK" if spread < PHASE_SPREAD_OK_DEG else
               "<-- SCATTERED; A's phase is not trustworthy"))
    log("\n   worst spread %.1f deg. On the 2026-08-17 23:12 pass this was 73-89"
        % worst)
    log("   deg and the off-quadrature fraction was 0.65 / 0.83 / 1.23.")
    return worst


def _dc_point(dac, rdr, rec, coil, log):
    """Static +/-DC_STEP_V. Cheap, and it is the only check on DAC_MAP.

    A bias step is a force step into an undamped plant, so it rings and the
    ringing does NOT decay inside any affordable dwell. It does not have to:
    the ring is zero-mean about the NEW equilibrium, so averaging over many
    periods returns that equilibrium. 20 s is 14 periods of the slowest mode.
    Same argument, and the same 0.8%-of-swing residual, as
    `analysis/bias_sweep.py`.
    """
    out = []
    for sgn in (+1.0, -1.0):
        v = bias_of(coil) + sgn * DC_STEP_V
        rec.mark("dc", coil, -1.0, sgn * DC_STEP_V)
        dac.set_voltage(DAC_MAP[coil], clamp(v))
        _, c, _, _ = acquire(dac, rdr, rec, DC_STEP_S, None, coil, hold_v=v)
        out.append(c.mean(axis=0) if len(c) else np.full(NCOLS, np.nan))
    park(dac, [coil])
    d = (out[0] - out[1]) / (2.0 * DC_STEP_V)      # counts per volt
    log("      DC  %s counts/V" % " ".join("%+6.1f" % x for x in d))
    return d


def coils_run(dac, coils, rec, log=print, dc=True, multi=False, pause=None,
               damp=False, reopen=None):
    """`reopen()` -> a fresh dac, or None. Required only when `damp` is set: the
    controller needs the port, so we give it up and take it back."""
    rdr = Reader(dac)
    park(dac)
    res, dcm = [], np.full((NCOLS, NCOLS), np.nan)
    n, total = 0, len(coils) * (1 if multi else MODES_N)
    state = {"dac": dac, "rdr": rdr}

    damper = ModalDamper(log=log) if damp else None

    def handover(coil):
        """Give the port to a controller, let it kill the ring, take it back.

        AFTER EVERY POINT, not just between coils. With `damp` set the unwind is
        skipped, so nothing else removes the ring -- and the modes are only
        0.27 Hz apart against a 30 s window, which is exactly how the 23:12 pass
        contaminated every one of its twelve points (|pre|/|drive| 3.5% to 417%).
        Settling per coil but skipping the unwind per point would be WORSE than
        the pass it replaces.
        """
        park(state["dac"])
        rec.mark("settle", coil, -1.0, 0.0)
        # EACH TEARDOWN STEP GETS ITS OWN GUARD. Wrapping both in one `try`
        # means a raise in stop_stream() skips close(), the port stays open, and
        # nothing else can have it -- measured 2026-08-18: the controller could
        # not start (rc=1) and the reopen then failed four times in a row against
        # a board that was perfectly healthy. `close()` is the step that must not
        # be skippable. Same shape as the defect in osem.eta.py's `finally`.
        try:
            state["dac"].stop_stream()
        except Exception as exc:
            log("      stop_stream raised (%s) -- closing anyway" % type(exc).__name__)
        try:
            state["dac"].close()
        except Exception as exc:
            log("      close raised (%s) -- the port may still be held"
                % type(exc).__name__)
        settle_controller(log=log)
        state["dac"] = reopen()
        state["rdr"] = Reader(state["dac"])
        # RAMPED, not stepped: the board reset left every coil at 0 V and a step
        # back to bias is a force step that undoes the settle. See park_ramped.
        log("      restoring bias over %.0f s (a step here re-excites the plate)"
            % BIAS_RAMP_S)
        park_ramped(state["dac"])
    for ci, coil in enumerate(coils):
        log("\n  --- coil %d (DAC ch%d) ---" % (coil, DAC_MAP[coil]))
        # Between coils the plate is still ringing from the last one and tau is
        # over 138 s, so the only affordable way to start from rest is a hand on
        # the table. This is the whole reason the multisine is worth having: it
        # cuts the transitions that need it from eleven to three.
        if pause and ci > 0:
            pause(coil)
        if dc:
            dcm[:, coil] = _dc_point(state["dac"], state["rdr"], rec, coil, log)
        if multi:
            n += 1
            amps = [choose_amp(coil, f, DWELL_S)[0] for _, f in MODES]
            seed = choose_amp(coil, MODES[0][1], DWELL_S)[1]
            log("    [%d/%d] all %d modes at once, amps %s V  (sum %.4f V peak,"
                " seed %.0f counts/V)"
                % (n, total, MODES_N, " ".join("%.4f" % a for a in amps),
                   sum(amps), seed))
            res.extend(_point_multi(state["dac"], state["rdr"], rec, coil, amps, log))
            if damp and damper is not None and damper.ok:
                settle_modal(state["dac"], state["rdr"], rec, log=log,
                             damper=damper)
            elif damp and reopen is not None:
                handover(coil)
            else:
                acquire(state["dac"], state["rdr"], rec,
                        SETTLE_BETWEEN_COILS_S)
            continue
        for name, f in MODES:
            n += 1
            amp, seed = choose_amp(coil, f, DWELL_S)
            pred = amp * seed * ramp_gain(f, DWELL_S)
            log("    [%d/%d] mode %s @ %.4f Hz  amp %.4f V  (seed %.0f counts/V,"
                " ramp x%.0f, predict %.0f counts)"
                % (n, total, name, f, amp, seed, ramp_gain(f, DWELL_S), pred))
            for _ in range(1 + AMP_RETRIES):
                p = _point(state["dac"], state["rdr"], rec, coil, name, f, amp,
                           log, skip_unwind=damp)
                if p is not None:
                    res.append(p)
                    break
                amp *= AMP_RETRY_SCALE
                if amp < AMP_MIN:
                    break
                log("      retry at amp %.4f V" % amp)
            if damp and damper is not None and damper.ok:
                settle_modal(state["dac"], state["rdr"], rec, log=log,
                             damper=damper)
            elif damp and reopen is not None:
                handover(coil)
        if not (damp and reopen is not None):
            acquire(state["dac"], state["rdr"], rec, SETTLE_BETWEEN_COILS_S)
    state["rdr"].report()
    return res, dcm


def coils_report(res, dcm=None, log=print):
    log("\n  === COILS: %d points ===" % len(res))
    if dcm is not None and np.isfinite(dcm).any():
        log("\n  d(counts_i)/d(bias_j), re-measured  [row = sensor, col = coil]")
        log("        " + " ".join("coil%d " % j for j in range(NCOLS)))
        for i in range(NCOLS):
            log("   a%d  %s" % (i, " ".join(
                "%+6.1f" % dcm[i, j] if np.isfinite(dcm[i, j]) else "     ."
                for j in range(NCOLS))))
        fin = np.isfinite(dcm)
        blk = dcm[:4, :4][fin[:4, :4]]
        oth = dcm[4:, 4:][fin[4:, 4:]]
        if blk.size:
            log("\n  mean |response| a0-a3 x coil0-3: %.1f counts/V" % np.abs(blk).mean())
        if oth.size:
            log("  mean |response| a4-a7 x coil4-7: %.1f counts/V"
                " (%.1f was this measurement's noise floor in 2026-08-04)"
                % (np.abs(oth).mean(), DC_SEED_FLOOR))
        log("\n  DAC_MAP check: coil j should dominate its own sensor j.")
        for j in range(NCOLS):
            col = dcm[:, j]
            if not np.isfinite(col).any():
                continue
            k = int(np.nanargmax(np.abs(col)))
            log("   coil %d (DAC ch%d) -> loudest sensor a%d (%+.1f counts/V)%s"
                % (j, DAC_MAP[j], k, col[k], "" if k == j else "   <-- NOT a%d" % j))

    for name, _ in MODES:
        pts = [p for p in res if p["mode"] == name]
        if not pts:
            continue
        log("\n  --- mode %s, |H| in V per V of coil drive (sigma below) ---" % name)
        log("        " + " ".join("coil%d " % p["coil"] for p in pts))
        for i in range(NCOLS):
            log("   a%d  %s" % (i, " ".join(
                ("%6.2f" % abs(p["H"][i])) if p["ok"][i] else "  rail" for p in pts)))
        log("\n   per-sensor SNR (|H| / sigma):")
        for i in range(NCOLS):
            log("   a%d  %s" % (i, " ".join(
                "%6.1f" % (abs(p["H"][i]) / p["sigma"][i] if p["sigma"][i] > 0 else 0.0)
                for p in pts)))
    if res:
        census_coils(res, dcm, log=log)


# ===========================================================================
# pass 3 -- the gate
# ===========================================================================
def phi_from_passive(t, counts, window_s=PHI_WINDOW_S, log=print):
    """Recover Phi from AMBIENT motion. No drive, no bench time, no risk.

    The rig has been treating Phi as something only a driven sweep can give.
    It is not. A mode is ONE coordinate q_m(t) and every sensor sees the same
    one, so at that frequency

        z_i  =  Phi_im * q_m   + noise_i

    and the ratios BETWEEN sensors are Phi_m up to a single complex scale.
    Q > 433 means the modes ring continuously on ambient excitation -- the
    passive record of 2026-08-06 has mode B at 0.52 V on a0 -- so q_m is
    already there, for free, in every quiet log on disk.

    THE ESTIMATOR is the cross-spectral density at f, averaged over K short
    windows: S = <z z^H>. Rank one if a single coordinate is responsible, and
    its leading eigenvector is Phi_m. `sim/server.py` already builds SHAPE_8
    this way ("cross-spectrum referred to a0"), for two modes, without error
    bars and without ever being used as a gate.

    WHY AVERAGE WINDOWS RATHER THAN TAKE ONE LONG LOCK-IN. One long lock-in
    gives one 8-vector, from which no covariance can be formed and no
    eigenvalue split can be measured -- a single vector is rank one by
    construction and would prove nothing. K windows give K draws of the same
    Phi against independent noise, so the rank-one check becomes real and the
    jackknife gives the error bar the gate needs.

    WHAT THIS CANNOT GIVE, and it matters: only Phi, never A. Ambient
    excitation is not a known input, so nothing here says which coil pushes
    which mode. The driven pass is still required for A -- but A was never the
    blocker. The measured allocator is 3x4 at cond 1.41 (`CLAUDE.md`).
    """
    t = np.asarray(t, float)
    span = t[-1] - t[0]
    K = int(span // window_s)
    if K < PHI_MIN_WINDOWS:
        log("  passive record is %.0f s; need at least %.0f s for %d windows."
            % (span, PHI_MIN_WINDOWS * window_s, PHI_MIN_WINDOWS))
        return None
    edges = [np.searchsorted(t, t[0] + k * window_s) for k in range(K + 1)]
    v = to_volts(counts)

    Phi = np.zeros((NCOLS, MODES_N), complex)
    Sig = np.full((NCOLS, MODES_N), np.inf)
    ok = np.zeros((NCOLS, MODES_N), bool)

    log("\n  === PHI FROM AMBIENT MOTION ===")
    log("  %d windows of %.1f s over %.0f s, no drive" % (K, window_s, span))

    for mi, (name, f) in enumerate(MODES):
        Z = np.zeros((K, NCOLS), complex)
        for k in range(K):
            a, b = edges[k], edges[k + 1]
            for i in range(NCOLS):
                Z[k, i] = lockin(t[a:b], v[a:b, i], f)
        S = (Z.conj().T @ Z) / K
        w, V = np.linalg.eigh(S)
        order = np.argsort(w)[::-1]
        w, V = w[order].real, V[:, order]
        frac = float(w[0] / max(w.sum(), 1e-30))
        phi = V[:, 0] * math.sqrt(max(w[0], 0.0))
        kmax = int(np.argmax(np.abs(phi)))
        phi = phi * np.exp(-1j * np.angle(phi[kmax]))

        # Jackknife over windows: the spread of Phi when each window is left
        # out in turn. Non-parametric, and it prices the fact that ambient
        # excitation is not stationary -- which a formula would not.
        js = np.zeros((K, NCOLS), complex)
        for k in range(K):
            Zk = np.delete(Z, k, axis=0)
            Sk = (Zk.conj().T @ Zk) / (K - 1)
            wk, Vk = np.linalg.eigh(Sk)
            o = np.argsort(wk)[::-1]
            pk = Vk[:, o[0]] * math.sqrt(max(wk[o[0]].real, 0.0))
            js[k] = pk * np.exp(-1j * np.angle(pk[kmax]))
        sig = np.sqrt((K - 1) / K * np.sum(np.abs(js - js.mean(axis=0)) ** 2, axis=0))

        Phi[:, mi], Sig[:, mi] = phi, sig
        ok[:, mi] = np.abs(phi) > ROW_SNR * sig

        log("\n  mode %s @ %.4f Hz" % (name, f))
        log("    eigenvalues %s" % " ".join("%.4f" % (x / w[0]) for x in w[:4]))
        log("    single-coordinate fraction %.4f   %s"
            % (frac, "OK" if frac >= RANK1_MIN else
               "<-- more than one thing at this frequency"))
        log("    sensor    |Phi|     sigma     SNR   phase deg   determined")
        for i in range(NCOLS):
            snr = abs(phi[i]) / sig[i] if sig[i] > 0 else 0.0
            log("      a%d    %8.5f  %8.5f  %6.1f   %+8.1f    %s"
                % (i, abs(phi[i]), sig[i], snr, np.degrees(np.angle(phi[i])),
                   "yes" if ok[i, mi] else "no"))
    return dict(Phi=Phi, sigma=Sig, ok=ok)


def gate_verdict(Phi, Sig, ok, log=print, source="driven"):
    """Count the determined rows and say what that permits. The gate itself.

    Three modes, so:

        rows determined >= 4   ->  Phi is TALL. A demoted sensor still leaves
                                   rank 3, and the null space is non-empty so
                                   the out-of-mode residual is a real check.
        rows determined == 3   ->  Phi is SQUARE. This is where the 2026-08-06
                                   measurement landed and why MIMO was closed:
                                   no degradation, and a residual identically
                                   zero (7.4e-16 over 137528 samples).
        rows determined <  3   ->  the modes are not observable at all.
    """
    rows = [i for i in range(NCOLS) if ok[i].all()]
    log("\n  rows determined at ALL THREE modes above %.0f sigma: %s"
        % (ROW_SNR, rows if rows else "none"))
    partial = [i for i in range(NCOLS) if ok[i].any() and not ok[i].all()]
    if partial:
        log("  determined at SOME modes only (unusable, and exactly a1's"
            " 2026-08-06 failure mode): %s" % partial)

    if len(rows) < MODES_N:
        log("\n  VERDICT: only %d rows determined against %d modes. The modes"
            " are not observable. MIMO stays closed." % (len(rows), MODES_N))
        _gate_next_steps(log)
        return rows

    Ps = Phi[rows, :]
    sv = np.linalg.svd(Ps, compute_uv=False)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else np.inf
    log("\n  Phi is %d x %d, singular values %s, cond %.2f"
        % (len(rows), MODES_N, " ".join("%.4f" % x for x in sv), cond))

    if len(rows) == MODES_N:
        log("\n  VERDICT: Phi is SQUARE (%d rows, %d modes). This is the"
            " 2026-08-06 result." % (len(rows), MODES_N))
        log("  A square Phi reproduces any reading exactly, so the out-of-mode")
        log("  residual is identically zero and cannot catch a bad sensor; and")
        log("  losing any one sensor drops the rank below the mode count.")
        log("  MIMO stays closed. CLAUDE.md is unchanged.")
        _gate_next_steps(log)
        return rows

    # The out-of-mode residual: what is left of a reading after the modal fit.
    # Its size is the ONLY check that catches a sensor which is neither railed
    # nor silent, and a square Phi makes it identically zero.
    P = np.eye(len(rows)) - Ps @ np.linalg.pinv(Ps)
    log("  null space dimension %d, residual projector norm %.4f"
        % (len(rows) - MODES_N, float(np.linalg.norm(P, 2))))
    log("\n  VERDICT: Phi is TALL. Graceful degradation is available:")
    for i in range(len(rows)):
        keep = [k for k in range(len(rows)) if k != i]
        r = np.linalg.matrix_rank(Ps[keep, :], tol=1e-6)
        sv2 = np.linalg.svd(Ps[keep, :], compute_uv=False)
        c2 = float(sv2[0] / sv2[-1]) if sv2[-1] > 1e-12 else np.inf
        log("    drop a%d -> rank %d of %d, cond %.2f%s"
            % (rows[i], r, MODES_N, c2,
               "" if r == MODES_N else "   <-- LOSES A MODE"))
    if source == "passive":
        log("\n  Phi passes the gate ON AMBIENT MOTION ALONE. What is NOT")
        log("  answered here is A -- which coil pushes which mode -- because")
        log("  ambient excitation is not a known input. Run `status.py coils`")
        log("  for that. A was never the blocker: the measured allocator is")
        log("  3x4 at cond 1.41 (CLAUDE.md).")
    else:
        log("\n  MIMO IS BUILDABLE on this measurement. Next: re-read")
        log("  analysis/mimo_closed.md Secs 3-5 -- the control law and the")
        log("  per-channel interlocks under a mixed command are designed")
        log("  there and were never the blocker.")
    _gate_next_steps(log)
    return rows


def _gate_next_steps(log):
    log("\n  What would change the answer, in order of cost:")
    log("    - raise TARGET_SWING_COUNTS (now %.0f) or DWELL_S (now %.0f s);"
        % (TARGET_SWING_COUNTS, DWELL_S))
    log("      SNR is linear in both and neither needs hardware.")
    log("    - for the passive path, a longer quiet record: sigma falls as")
    log("      sqrt(windows) and quiet records are free.")
    log("    - a sensor determined at two modes and not the third needs DRIVE")
    log("      at the third, not a better sensor.")
    log("    - a sensor determined at none is a hardware job, and not ours.")


def mimo_gate(res, log=print):
    """Assemble Phi and A from the DRIVEN pass, then apply the gate.

    Each mode's response matrix is Psi_m[i,j] = (sensor i) / (coil j drive) at
    that mode, and physics says it factorises as Phi_m (x) A_m -- one mode is
    one modal coordinate, so Psi_m is rank 1. That is not an assumption to be
    hoped for, it is a CHECK: a rank-1 fraction below RANK1_MIN means the point
    is not measuring one mode, and the right response is to fix the measurement
    rather than to build a controller on it. `gains.json` got 0.9418 on mode A
    from off-resonance data; on resonance it should be better.
    """
    log("\n  === MIMO GATE (driven) ===")
    Phi = np.zeros((NCOLS, MODES_N), complex)
    Sig = np.full((NCOLS, MODES_N), np.inf)
    ok = np.zeros((NCOLS, MODES_N), bool)
    A = np.zeros((MODES_N, NCOLS), complex)
    # The geometric factorisation, computed alongside and exported by default.
    Ag = np.zeros((MODES_N, NCOLS), complex)
    Sg = np.full((NCOLS, MODES_N), np.inf)
    okg = np.zeros((NCOLS, MODES_N), bool)
    geo_note = []

    for mi, (name, f) in enumerate(MODES):
        pts = sorted([p for p in res if p["mode"] == name], key=lambda p: p["coil"])
        if len(pts) < 2:
            log("\n  mode %s: %d points -- cannot factorise. Need >= 2 coils."
                % (name, len(pts)))
            continue
        cols = [p["coil"] for p in pts]
        Psi = np.column_stack([p["H"] for p in pts])
        Ssd = np.column_stack([p["sigma"] for p in pts])
        good = np.column_stack([p["ok"] for p in pts])
        Psi = np.where(good, Psi, 0.0)

        U, s, Vh = np.linalg.svd(Psi, full_matrices=False)
        r1 = float(s[0] ** 2 / np.sum(s ** 2)) if s.size else 0.0
        phi, a = U[:, 0] * s[0], Vh[0, :]
        # Fix the phase gauge so Phi is comparable between runs and against a
        # stored signature: rotate so the loudest sensor is real and positive.
        k = int(np.argmax(np.abs(phi)))
        rot = np.exp(-1j * np.angle(phi[k]))
        phi, a = phi * rot, a * np.conj(rot)

        # sigma on phi. Propagate it, do not guess at it.
        #
        # Psi ~ phi a^H with ||a|| = 1, so phi = Psi a and therefore
        #     var(phi_i) = sum_j sigma_ij^2 |a_j|^2
        # exactly, treating a as known. Because ||a|| = 1 that is a WEIGHTED RMS
        # of the per-coil sigmas -- it sits between the smallest and the largest
        # of them and does NOT shrink with the coil count.
        #
        # This replaces `sqrt(sum sigma^2) / ncoils`, which was wrong by a factor
        # sqrt(ncoils) = 2.83 here and wrong in the dangerous direction. The
        # selftest caught it: a5, planted at EXACTLY zero at mode C, came back
        # "determined" at 4.1 sigma. A row that is determined by nothing but its
        # own noise is precisely how MIMO gets reopened on a sensor that cannot
        # see. The second-order term from the error in a itself is dropped, which
        # is safe only while the rank-1 fraction is high -- and that is checked
        # and printed on the line above.
        sig = np.sqrt((Ssd ** 2) @ (np.abs(a) ** 2))

        Phi[:, mi], Sig[:, mi] = phi, sig
        A[mi, cols] = a
        ok[:, mi] = np.abs(phi) > ROW_SNR * sig

        log("\n  mode %s @ %.4f Hz, %d coils: %s" % (name, f, len(pts), cols))
        log("    singular values %s" % " ".join("%.4f" % (x / s[0]) for x in s[:4]))
        log("    rank-1 fraction %.4f   %s"
            % (r1, "OK" if r1 >= RANK1_MIN else
               "<-- NOT one mode; measurement suspect"))
        log("    sensor    |Phi|     sigma     SNR   determined")
        for i in range(NCOLS):
            snr = abs(phi[i]) / sig[i] if sig[i] > 0 else 0.0
            log("      a%d    %8.4f  %8.4f  %6.1f   %s"
                % (i, abs(phi[i]), sig[i], snr, "yes" if ok[i, mi] else "no"))

        # ---- the same data against the GEOMETRIC column -------------------
        # No SVD and no gauge: Psi = g (x) a with g KNOWN, so a is a plain
        # least-squares projection, a[j] = (g . Psi[:,j]) / (g . g). Every sign
        # in the answer is then a property of the plant rather than of a
        # factorisation, which is the whole point (see GEO_PHI).
        g = GEO_PHI[:, mi]
        use = good.all(axis=1) & (g != 0)
        if use.sum() >= 3:
            ag = (g[use] @ Psi[use, :]) / float(g[use] @ g[use])
            Ag[mi, cols] = ag
            nag = np.linalg.norm(ag)
            Sg[:, mi] = np.sqrt((Ssd ** 2) @ (np.abs(ag / nag) ** 2)) if nag > 0 else np.inf
            okg[:, mi] = use
            res_m = Psi[use, :] - np.outer(g[use], ag)
            rfrac = (float(np.linalg.norm(res_m) / np.linalg.norm(Psi[use, :]))
                     if np.linalg.norm(Psi[use, :]) > 0 else np.nan)
            # |cos| of the measured shape against the geometric one. This is the
            # check the geometry earns by being independent of the measurement.
            pv, gv = phi[use], g[use].astype(complex)
            cosg = float(abs(np.vdot(gv, pv)) / (np.linalg.norm(gv) * np.linalg.norm(pv))) \
                if np.linalg.norm(pv) > 0 else np.nan
            # And how much of the DRIVEN response lands in warp, which no rigid
            # body produces. On ambient it ran 0.139 before the 2026-08-20
            # re-seating and 0.262 after, and SENSOR_H accounts for it: the same
            # warp-null fit takes the static residual down 19.3x.
            wfrac = (float(np.linalg.norm(GEO_WARP @ Psi[:4, :] / 4.0)
                           / max(np.linalg.norm(g[:4] @ Psi[:4, :] / 4.0), 1e-30))
                     if use[:4].all() else np.nan)
            log("    geometry: %s   |cos| vs measured %.3f %s" % (
                GEO_DOF[mi], cosg,
                "OK" if cosg >= GEO_COS_WARN else "<-- DISAGREES with geometry"))
            log("      unexplained by the rigid column %.3f, warp/this-mode %.3f"
                % (rfrac, wfrac))
            log("      A row from the projection, counts-scale V/V: %s"
                % " ".join("%+8.4f" % x for x in (-ag).imag))
            geo_note.append((name, cosg, rfrac, use.sum()))
        else:
            log("    geometry: only %d of the 4 in-plane rows are usable at this"
                " mode -- no geometric projection." % int(use.sum()))

    if geo_note:
        log("\n  --- GEOMETRIC Phi, the default export ---")
        log("   mode   dof         |cos| vs measured   unexplained   rows")
        for name, cosg, rfrac, nr in geo_note:
            log("     %s    %-10s %13.3f %13.3f %6d"
                % (name, GEO_DOF["ABC".index(name)], cosg, rfrac, nr))
        log("   Phi is EXACT here, so it carries no sigma of its own and there is")
        log("   no gauge left for A's sign to disagree with. The sigma exported")
        log("   with it is the per-sensor measurement noise, which is what the")
        log("   controller's weighted least squares actually wants.")
        if len(geo_note) < MODES_N:
            log("   %d of %d modes projected; the rest keep no geometric A."
                % (len(geo_note), MODES_N))

    rows = gate_verdict(Phi, Sig, ok, log=log, source="driven")
    return dict(Phi=Phi, sigma=Sig, ok=ok, rows=rows, A=A,
                Phi_geo=GEO_PHI.astype(complex), sigma_geo=Sg, ok_geo=okg, A_geo=Ag,
                geo_modes=[n for n, _, _, _ in geo_note])


# ===========================================================================
# the modal export -- the only way Phi and A reach a controller
# ===========================================================================
# `dcm` is carried too: `a_from_dc(Phi)` with dc=None silently falls back to
# DC_SEED, the HARDCODED 2026-08-04 matrix, even on a run that just measured its
# own. That is two dates and two rigs, and it is one of the three reasons
# `a_from_dc` labels its result PROVISIONAL. Added 2026-08-20.
_EXPORT = {"phi": None, "source": "", "dcm": None}
MODAL_PATH = os.path.join("data", "modal.json")
MODAL_SCHEMA = "osem-modal-1"


def realify(Z):
    """Complex Phi column -> real vector, plus how much did not fit.

    A lightly, proportionally damped mode has a REAL mode shape: every sensor
    is in phase or exactly out of phase with every other. The measurement
    returns complex numbers, so the question is how nearly real they are once
    the arbitrary global phase is removed.

    The principal axis is the eigenvector of the 2x2 real scatter of
    (Re, Im), i.e. the rotation that puts the most energy into Re. What is
    left in Im is the part no real mode shape can explain: a non-proportional
    damping term, a sensor with a genuine phase lag, or a bad measurement.
    It is REPORTED, never silently dropped, because a controller multiplying a
    real Phi against a shape that is 40% imaginary is using a number that does
    not mean what it says.

    RETURNS THE ROTATION IT APPLIED, and the caller must use it. A mode's Phi
    column and A row are determined only up to a SHARED unit-modulus factor:
    (phi*z)(a/z) reproduces the same response for any |z| = 1. Realifying the two
    halves independently therefore picks two unrelated gauges and destroys the
    pairing -- measured 2026-08-17, that is exactly what happened: all four coils
    came back with A/Phi sign ratios like (+0.98, +0.24, -0.88), i.e. mode C's
    sign flipped on every coil, and osem.zeta.py's colocation check refused the
    file. The check was right and the export was wrong. `save_modal` now applies
    conj(rot) to A's row so that phi (x) a is preserved exactly.
    """
    Z = np.asarray(Z, complex)
    M = np.array([[float(np.sum(Z.real ** 2)), float(np.sum(Z.real * Z.imag))],
                  [float(np.sum(Z.real * Z.imag)), float(np.sum(Z.imag ** 2))]])
    _, V = np.linalg.eigh(M)
    th = math.atan2(V[1, -1], V[0, -1])
    rot = np.exp(-1j * th)
    R = Z * rot
    if np.sum(R.real) < 0:                 # gauge: the sum is positive
        R, rot = -R, -rot
    den = float(np.linalg.norm(R.real))
    return (R.real,
            (float(np.linalg.norm(R.imag) / den) if den > 0 else float("inf")),
            rot)


def a_from_dc(Phi, dc=None, log=print):
    """A PROVISIONAL A, decomposed out of the measured DC matrix onto Phi.

    This is not a measurement of A and is labelled `provisional` in the file so
    that `osem.zeta.py` refuses it unless somebody deliberately overrides. It
    exists so the modal loop can be exercised in the simulator before the bench
    run that measures A properly.

    IT IS A DERIVATION, NOT A GUESS. The static response of the sensors to a
    step on coil j is the sum over modes of each mode's static deflection:

        d[:, j]  =  SUM_m  Phi[:, m] * A[m, j] / w_m^2

    so projecting the measured column onto each mode shape recovers A:

        A[m, j]  =  w_m^2 * (Phi[:, m] . d[:, j]) / (Phi[:, m] . Phi[:, m])

    Both inputs are measured -- `d` is `bench/20260804/dcmatrix.log`, one coil
    stepped at a time, and Phi is this tool's own.

    WHY IT IS STILL PROVISIONAL, in three parts, none of them small:
      * `d` is a STATIC response and the modes are at 0.7-1.6 Hz. Any frequency
        dependence in the coil driver or the flag geometry between DC and there
        is unmodelled and lands directly in A.
      * the a4-a7 block of `d` is 4 counts/V against its own 4-count noise
        floor, so those columns are consistent with zero.
      * `d` was measured 2026-08-04 and Phi 2026-08-06. Two dates is two rigs
        unless something says otherwise.
    """
    d = DC_SEED if dc is None else np.asarray(dc, float)
    A = np.zeros((MODES_N, NCOLS))
    for mi, (_, f) in enumerate(MODES):
        p = np.real(Phi[:, mi])
        den = float(p @ p)
        if den <= 0:
            continue
        w2 = (2.0 * math.pi * f) ** 2
        A[mi] = w2 * (p @ d) / den
    log("\n  A DERIVED from the DC matrix and Phi -- PROVISIONAL, not measured.")
    log("  osem.zeta.py refuses it unless OSEM_MODAL_PROVISIONAL=1.")
    for mi, (n, _) in enumerate(MODES):
        log("    mode %s  %s" % (n, " ".join("%+9.1f" % x for x in A[mi])))
    # HANDED BACK AS PURE IMAGINARY, and that is not a cosmetic choice.
    # `save_modal` writes `-Im(A * conj(rot_phi))`, because on resonance the
    # response lags the drive by 90 degrees and so a DRIVEN A sits entirely in the
    # quadrature part with a determined sign. A static A has no quadrature at all:
    # it is real. Handing this real array straight to `save_modal` therefore wrote
    # `-Im(real) == 0` and produced a modal.json whose `a.value` was ALL ZEROS,
    # with `imag_frac` inf. That was silent -- the file wrote, parsed, and carried
    # a perfectly good Phi -- and it means the `--derive-a-dc` path never once
    # produced a usable A. Found 2026-08-20.
    # Multiplying by -1j puts the magnitude where the writer looks for it and
    # preserves the sign exactly: with rot = 1 (which is what a geometric Phi
    # gives), z = -1j*A, z.imag = -A, and -z.imag = A. The real part is then
    # exactly zero, so `imag_frac` correctly reports 0.0 -- a static response has
    # no off-quadrature residual to report, which is the whole reason a DC pass is
    # immune to the leftover-ring contamination that ruins a driven one.
    # ALL EIGHT COILS, not the first four -- this said `list(range(4))` until
    # 2026-08-20 and silently threw away half the actuator set. THE DISCARDED
    # COILS ARE NOT NEGLIGIBLE: on the 2026-08-20 DC matrix projected onto the
    # geometric Phi, mode C reads coil 3 +331.7, coil 4 -337.1, coil 5 +304.8,
    # coil 7 +183.4, so coil 4 is LARGER there than the coil that was kept.
    # Low SNR is a weighting question, not a reason to drop a column:
    # `Modal._build_tables` keeps `sum(sv >= sv[0]/MODAL_COND_MAX)` singular
    # directions downstream and refuses only what is genuinely unreachable.
    return dict(A=(-1j) * A, coils=list(range(NCOLS)), provisional=True,
                provenance=("DC matrix projected onto Phi; real by construction, "
                            "carried as -1j*A so save_modal's quadrature "
                            "convention recovers it exactly"))


def pick_export(g, geo=True):
    """Choose which factorisation of the same driven data goes to the controller.

    `geo=True` is the default and takes Phi from the geometry, exact and signed,
    with A projected onto it; `geo=False` takes both from the rank-1 SVD, which is
    what the rig ran until 2026-08-17 and which carries a sign per mode that
    nothing offline can check. THEY ARE NOT INTERCHANGEABLE HALVES: a geometric
    Phi paired with an SVD A is precisely the mismatch that pumped, so this
    returns both halves together or neither.
    """
    if g is None:
        return None, None
    if geo and g.get("A_geo") is not None and g.get("geo_modes"):
        phi = dict(Phi=g["Phi_geo"], sigma=g["sigma_geo"], ok=g["ok_geo"])
        a = dict(A=g["A_geo"], coils=g.get("coils", []))
        return phi, a
    phi = dict(Phi=g["Phi"], sigma=g["sigma"], ok=g["ok"])
    a = (dict(A=g["A"], coils=g.get("coils", []))
         if g.get("A") is not None else None)
    return phi, a


def do_export(path, want_geo=True, derive_a_dc=False, log=print):
    """The one place `--save-modal` decides what to write. Both call sites use it."""
    g = _EXPORT["phi"]
    if g is None:
        log("\n  --save-modal: nothing to write. `phi`, `coils` or `mimo` has to"
            " have run.")
        return None
    geo = bool(want_geo) and bool(g.get("geo_modes"))
    if want_geo and not geo:
        log("\n  --save-modal: asked for the geometric basis and this run has no")
        log("  geometric projection (that needs a DRIVEN pass with 3+ usable")
        log("  in-plane rows). Falling back to the measured factorisation, which")
        log("  carries a sign per mode that nothing offline can check.")
    ph, ex = pick_export(g, geo=geo)
    if ex is None and derive_a_dc:
        ex = a_from_dc(ph["Phi"], _EXPORT.get("dcm"))
    return save_modal(ph, ex, path, _EXPORT["source"], log=log, geo=geo)


def save_modal(phi=None, a=None, path=MODAL_PATH, source="", log=print, geo=False):
    """Write Phi and A where a controller can find them, or refuse to.

    THE POINT OF THIS FILE IS THAT IT CAN BE ABSENT. `osem.zeta.py` runs the
    modal law only when this exists, parses, and passes its own checks; with no
    file it runs the diagonal law and says so. That is deliberate and it is the
    lesson of v13, which was runnable from a file whose own docstring said it
    must not be (`versions.md`): the refusal has to be the DEFAULT path, not a
    branch somebody has to remember to take.

    So nothing is invented here. A column of Phi that was never determined is
    written with ok=false and the controller drops that sensor; if A is absent
    the file still writes, with a=null, and the controller stays diagonal.
    """
    out = dict(schema=MODAL_SCHEMA,
               created=datetime.now().isoformat(timespec="seconds"),
               git_rev=_git_rev(), source=source,
               modes=[dict(name=n, f_hz=f) for n, f in MODES],
               row_snr=ROW_SNR, n_sensors=NCOLS, n_modes=MODES_N,
               basis="geometric" if geo else "svd")
    # `gauge` ties the two halves together and a controller refuses a pair whose
    # gauges differ. Under a geometric Phi there IS no gauge, so say so in the
    # string too -- a file whose phi says geometric and whose a says svd is the
    # exact mismatch that pumped on 2026-08-17.
    if geo:
        source = "geometric Phi; A " + source
        out["source"] = source
        out["dof"] = list(GEO_DOF)

    rots = None
    if phi is not None:
        P, S, K = phi["Phi"], phi["sigma"], phi["ok"]
        cols, imag, rots = [], [], []
        for mi in range(MODES_N):
            if geo:
                # Already real, already signed. Rotating it would be picking a
                # gauge back up after deliberately throwing it away -- and note
                # `realify`'s principal axis is defined up to a sign, so on an
                # exactly real column it can hand back rot = -1 and silently
                # relabel Z as -Z.
                r, im, rot = P[:, mi].real.astype(float), 0.0, 1.0 + 0.0j
            else:
                r, im, rot = realify(P[:, mi])
            cols.append(r)
            imag.append(im)
            rots.append(rot)
        R = np.column_stack(cols)
        out["phi"] = dict(
            value=[[float(x) for x in row] for row in R],
            sigma=[[float(x) for x in row] for row in np.abs(S)],
            ok=[[bool(x) for x in row] for row in K],
            imag_frac=[float(x) for x in imag],
            rows=[i for i in range(NCOLS) if K[i].all()])
        log("\n  Phi -> real, per mode. `imag_frac` is the part no real mode")
        log("  shape explains; a controller multiplies the real part only.")
        for mi, (n, _) in enumerate(MODES):
            log("    mode %s  imag_frac %.3f   %s"
                % (n, imag[mi], "OK" if imag[mi] < IMAG_FRAC_MAX else
                   "<-- large; treat Phi at this mode as provisional"))

    # `gauge` ties Phi and A to ONE factorisation. Each mode's Phi and A are
    # determined only up to a shared scale and sign, so a Phi from the passive
    # run paired with an A from a driven run has an INDEPENDENT sign per mode --
    # and a wrong sign pumps. osem.zeta.py refuses a pair whose gauges differ.
    if "phi" in out:
        out["phi"]["gauge"] = source

    if a is not None:
        A, coils = a["A"], a["coils"]
        # THE SAME GAUGE AS Phi, not an independently chosen one. `realify` above
        # multiplied Phi's column m by rots[m]; phi (x) a is invariant only if a's
        # row is multiplied by conj(rots[m]). Realifying A on its own picks a
        # second, unrelated gauge and the two halves disagree by a sign --
        # measured 2026-08-17 on all four coils at mode C.
        #
        # THE SIGN IS PHYSICS, NOT A GAUGE CHOICE. On resonance the response lags
        # the drive by exactly 90 degrees, so once Phi is rotated real the whole of
        # A sits in the IMAGINARY part with a determined sign:
        #
        #     A = -Im(a * conj(rot_phi))
        #
        # Verified against this file's own lockin, undamped oscillator driven on
        # resonance from rest with Phi*A known:
        #
        #     Phi*A > 0  ->  H = -1.2024i        Phi*A < 0  ->  H = +1.2024i
        #
        # The previous version chose the per-mode sign by maximising colocation
        # consistency, which is wrong twice over: colocation does not hold on this
        # rig at all (`CLAUDE.md`), and even where it does it constrains lambda_j
        # across modes and says nothing about which sign each MODE gets, so four
        # patterns pass and one damps. Measured on hardware 2026-08-17: median
        # channel ratio 1.5 diagonal -> 2.3 modal, i.e. it added energy. Nothing
        # offline catches it -- the allocator inverts a wrong-signed A exactly, so
        # the force is right in the model and backwards in the plant.
        rows, a_imag = [], []
        for mi in range(MODES_N):
            z = A[mi, :] * (np.conj(rots[mi]) if rots is not None else 1.0)
            rows.append(-z.imag)
            den = float(np.linalg.norm(z.imag))
            # Here the REAL part is the leftover: on resonance there should not be
            # one, so this is the honest residual of the 90-degree assumption.
            a_imag.append(float(np.linalg.norm(z.real) / den)
                          if den > 0 else float("inf"))
        rows = np.array(rows, float)

        if rots is not None:
            keep = [j for j in range(rows.shape[1])
                    if j in coils and abs(R[j, :]).min() > 0]
            good = sum(1 for j in keep
                       if np.all((rows[:, j] / R[j, :]) > 0)
                       or np.all((rows[:, j] / R[j, :]) < 0))
            log("\n  A sign set by the resonant phase (Im, lag = -90 deg), not by")
            log("  a search. Colocation is now a CHECK rather than the criterion:")
            log("  %d of %d measured coils have a consistent-sign lambda." % (good, len(keep)))
            if good < len(keep):
                log("  The rest disagree across modes; osem.zeta.py will drop them")
                log("  and refuse if fewer than 3 survive. That is the check doing")
                log("  its job -- do not tune the sign to make it pass.")
        log("\n  A off-quadrature fraction per mode (should be small on resonance):")
        for mi, (n, _) in enumerate(MODES):
            log("    mode %s  %.3f   %s" % (n, a_imag[mi],
                "OK" if a_imag[mi] < IMAG_FRAC_MAX else
                "<-- large; A at this mode is suspect"))
        # `basis` lives in the A block because that is where a controller reads
        # it: stdlib.Modal refuses an A whose basis differs from the Phi it is
        # holding, in both directions. A geometric Phi paired with an A measured
        # against an SVD Phi is the exact mismatch that pumped the optic on
        # 2026-08-17, and the two halves are indistinguishable once written.
        out["a"] = dict(value=[[float(x) for x in row] for row in rows],
                        imag_frac=[float(x) for x in a_imag],
                        coils=[int(c) for c in coils], gauge=source,
                        basis="geometric" if geo else "measured",
                        provisional=bool(a.get("provisional")),
                        provenance=a.get("provenance", ""))
    else:
        out["a"] = None
        log("\n  A is NOT in this file. `status.py coils` measures it. Until it")
        log("  does, osem.zeta.py will boot, read this, and run DIAGONAL.")

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as fh:
        json.dump(out, fh, indent=1)
    log("\n  modal data -> %s" % path)
    return out


# ===========================================================================
# pass 4 -- the calibration pulse and drift
# ===========================================================================
def _hann_pulse(t, w, amp):
    if t < 0 or t > w:
        return 0.0
    return amp * 0.5 * (1.0 - math.cos(2.0 * math.pi * t / w))


def pulse_run(dac, rec, coil=PULSE_COIL, repeats=PULSE_REPEATS, log=print):
    """One Hann impulse, then the free ring. Repeat, subtracting the residual.

    ONE time origin per pulse event covers pre, pulse and ring, so the ring's
    lock-in divided by the drive's lock-in has a meaningful PHASE, and the
    pre-window's phasor is directly subtractable from the ring's.
    """
    rdr = Reader(dac)
    park(dac)
    shots = []
    for k in range(repeats):
        t0 = time.time()
        log("\n  pulse %d/%d on coil %d (DAC ch%d), %.2f V x %.2f s Hann"
            % (k + 1, repeats, coil, DAC_MAP[coil], PULSE_AMP_V, PULSE_W_S))
        rec.mark("pre", coil, -1.0, PULSE_AMP_V)
        tp, cp, _, _ = acquire(dac, rdr, rec, PRE_S, None, coil, t0)
        pre0 = tp[0] if len(tp) else 0.0
        watch = guard_mask(cp, log)

        # The envelope is measured from the START OF THIS WINDOW, the phasors
        # from the shared origin `t0`. `t_ref=None` gives `drive` the former
        # while `acquire` keeps timestamping in the latter, which is the whole
        # reason the two arguments are separate.
        rec.mark("pulse", coil, -1.0, PULSE_AMP_V)
        tpu, _, upu, _ = acquire(
            dac, rdr, rec, PULSE_W_S + 0.2,
            lambda t: _hann_pulse(t, PULSE_W_S, PULSE_AMP_V),
            coil, t0, guard=watch)
        park(dac, [coil])
        rec.mark("ring", coil, -1.0, PULSE_AMP_V)
        tr, cr, _, _ = acquire(dac, rdr, rec, PULSE_RING_S, None, coil, t0)
        if len(tr) < 64 or len(tpu) < 8:
            log("    !! too few samples -- discarded")
            continue

        shot = dict(coil=coil, t_pulse=float(tpu[0]), n=len(tr), modes={})
        for name, f in MODES:
            D = lockin(tpu, upu, f)                       # the recorded drive
            if abs(D) < 1e-9:
                continue
            g = MODE_GAMMA[name]
            for i in range(NCOLS):
                vp = (cp[:, i] - cp[:, i].mean()) * COUNTS_TO_V if len(cp) > 8 else None
                vr = (cr[:, i] - cr[:, i].mean()) * COUNTS_TO_V
                Zr, sd = lockin_sigma(tr, vr, f)
                Zp = lockin(tp, vp, f) if vp is not None else 0j
                # The residual is free ringing on the SAME origin, so it is the
                # same phasor decayed across the gap. Undamped-plant arithmetic,
                # corrected by the measured gamma; the pre-level is reported so
                # this is checkable rather than trusted.
                dec = math.exp(-g * max(float(tr[0] - pre0), 0.0))
                Zc = Zr - Zp * dec
                shot["modes"].setdefault(name, {})[i] = dict(
                    H=complex(Zc / D), sigma=float(sd / abs(D)),
                    pre=float(abs(Zp)), ring=float(abs(Zr)))
        shots.append(shot)
        worst = max((m[i]["pre"] / max(m[i]["ring"], 1e-12)
                     for m in shot["modes"].values() for i in m), default=0.0)
        log("    residual before this pulse was %.0f%% of the ring after it"
            % (100.0 * worst))
        if worst > PULSE_PRE_MAX_FRAC and k + 1 < repeats:
            log("    !! above PULSE_PRE_MAX_FRAC = %.0f%%; the optic has not"
                " settled. Later pulses in this run are less trustworthy."
                % (100.0 * PULSE_PRE_MAX_FRAC))
    rdr.report()
    park(dac)
    return shots


def pulse_fingerprint(shots, resting=None):
    """Average the shots into one fingerprint: complex gain per (sensor, mode)."""
    fp = dict(created=datetime.now().isoformat(timespec="seconds"),
              git_rev=_git_rev(), modes={n: f for n, f in MODES},
              coil=shots[0]["coil"] if shots else None,
              n_shots=len(shots), resting=resting, gain={})
    for name, _ in MODES:
        rows = {}
        for i in range(NCOLS):
            zs = [s["modes"][name][i]["H"] for s in shots
                  if name in s["modes"] and i in s["modes"][name]]
            if not zs:
                continue
            z = np.mean(zs)
            spread = (np.std(np.abs(zs), ddof=1) / max(abs(z), 1e-12)
                      if len(zs) > 1 else float("nan"))
            rows[str(i)] = dict(mag=float(abs(z)), phase_deg=float(np.degrees(np.angle(z))),
                                rel_spread=float(spread))
        fp["gain"][name] = rows
    return fp


def pulse_compare(fp, ref, log=print):
    log("\n  === DRIFT vs %s (%s, %d shots) ==="
        % (ref.get("created", "?"), ref.get("git_rev", "?"), ref.get("n_shots", 0)))
    if ref.get("coil") != fp.get("coil"):
        log("  !! signature was taken on coil %s, this run on coil %s --"
            " not comparable." % (ref.get("coil"), fp.get("coil")))
        return
    log("\n  gain drift, dB (+ = louder than the signature); repeat spread in")
    log("  brackets is THIS run's shot-to-shot scatter -- a drift smaller than")
    log("  that is not a result.")
    log("   ch      mode A            mode B            mode C")
    for i in range(NCOLS):
        cells = []
        for name, _ in MODES:
            a = fp["gain"].get(name, {}).get(str(i))
            b = ref["gain"].get(name, {}).get(str(i))
            if not a or not b or b["mag"] <= 0 or a["mag"] <= 0:
                cells.append("      .          ")
                continue
            db = 20.0 * math.log10(a["mag"] / b["mag"])
            sp = a.get("rel_spread", float("nan"))
            sp_db = 20.0 * math.log10(1.0 + sp) if np.isfinite(sp) else float("nan")
            cells.append("%+6.2f [%s]  " % (db, ("%.2f" % sp_db) if np.isfinite(sp_db) else " . "))
        log("   a%d  %s" % (i, "".join(cells)))

    log("\n  phase drift, degrees:")
    log("   ch      mode A     mode B     mode C")
    for i in range(NCOLS):
        cells = []
        for name, _ in MODES:
            a = fp["gain"].get(name, {}).get(str(i))
            b = ref["gain"].get(name, {}).get(str(i))
            if not a or not b:
                cells.append("      .    ")
                continue
            d = (a["phase_deg"] - b["phase_deg"] + 180.0) % 360.0 - 180.0
            cells.append("%+9.1f  " % d)
        log("   a%d  %s" % (i, "".join(cells)))

    if fp.get("resting") and ref.get("resting"):
        log("\n  resting counts drift (a flag or an LED moving shows up here first):")
        log("   ch    then     now    delta")
        for i in range(NCOLS):
            try:
                b, a = float(ref["resting"][i]), float(fp["resting"][i])
            except Exception:
                continue
            log("   a%d  %7.1f %7.1f  %+7.1f" % (i, b, a, a - b))

    log("\n  Interpretation. A per-channel gain change with NO phase change and")
    log("  NO resting-count change is sensor or driver gain. A resting-count")
    log("  change with the gain intact is the flag having moved. A phase change")
    log("  at one mode only is that mode having moved in frequency -- re-run")
    log("  `status.py sensors` and read the peak-location table.")
    log("\n  A drift COMMON to drive and sensor cannot appear here: the")
    log("  fingerprint is a ratio. Nothing on this rig measures absolute")
    log("  displacement, so that is a property of the bench, not of this tool.")


# ===========================================================================
# replay
# ===========================================================================
def load_csv(path):
    """This tool's own schema, or any bare `time_s,a0..a7` log.

    The second form matters: every quiet record and every controller CSV in
    `data/` is that shape, and the passive-Phi result came out of one of them
    (`data/20260806_192723_quiet_openloop.csv`). Refusing to read them would
    have made the cheapest measurement on this rig the one thing the tool
    could not do.
    """
    import csv
    rows = []
    if not os.path.isfile(path):
        near = []
        d = os.path.dirname(path) or "data"
        if os.path.isdir(d):
            near = sorted(f for f in os.listdir(d) if f.endswith(".csv"))[-5:]
        sys.exit("  no such file: %s%s"
                 % (path, ("\n  most recent .csv in %s: %s" % (d, ", ".join(near)))
                    if near else ""))
    try:
        fh = open(path)
    except OSError as e:
        sys.exit("  cannot read %s: %s" % (path, e))
    with fh:
        r = csv.reader(fh)
        try:
            hdr = next(r)
        except StopIteration:
            sys.exit("  %s is empty." % path)
        idx = {k.strip(): i for i, k in enumerate(hdr)}
        if "time_s" not in idx or "a0" not in idx:
            sys.exit("  %s has no `time_s`/`a0` columns -- header is:\n    %s"
                     % (path, ",".join(hdr[:12])))
        rich = all(k in idx for k in ("seg", "phase", "coil", "freq_hz",
                                      "amp_v", "u_v"))
        if not rich:
            print("  bare log (no phase column): treating all of it as one"
                  " undriven `passive` segment.")
        ncols = sum(1 for i in range(NCOLS) if ("a%d" % i) in idx)
        if ncols < NCOLS:
            print("  only %d sensor columns in this file; the rest read zero."
                  % ncols)
        for line in r:
            if len(line) < len(hdr):
                continue
            try:
                counts = [int(float(line[idx["a%d" % i]])) if ("a%d" % i) in idx
                          else 0 for i in range(NCOLS)]
                t = float(line[idx["time_s"]])
            except ValueError:
                continue
            if any(c < 0 or c > ADC_MAX_COUNTS for c in counts[:ncols]):
                continue                              # sample-guard, offline too
            if rich:
                rows.append((t, int(line[idx["seg"]]), line[idx["phase"]],
                             int(line[idx["coil"]]), float(line[idx["freq_hz"]]),
                             float(line[idx["amp_v"]]), float(line[idx["u_v"]]),
                             counts))
            else:
                rows.append((t, 0, "passive", -1, -1.0, 0.0, BIAS_V, counts))
    return rows


def replay(path, what, log=print):
    rows = load_csv(path)
    if not rows:
        sys.exit("  no usable rows in %s" % path)
    log("  replay: %d rows from %s" % (len(rows), path))
    segs = {}
    for t, seg, phase, coil, f, amp, u, c in rows:
        segs.setdefault(seg, dict(phase=phase, coil=coil, f=f, amp=amp,
                                  t=[], u=[], c=[]))
        s = segs[seg]
        s["t"].append(t)
        s["u"].append(u - bias_of(coil if coil >= 0 else None))
        s["c"].append(c)
    for s in segs.values():
        s["t"] = np.asarray(s["t"])
        s["u"] = np.asarray(s["u"])
        s["c"] = np.asarray(s["c"], float)

    if what in ("sensors", "phi", "all"):
        ps = [s for s in segs.values() if s["phase"] == "passive"]
        if ps:
            t = np.concatenate([s["t"] for s in ps])
            c = np.concatenate([s["c"] for s in ps])
            if what in ("sensors", "all"):
                sensors_analyse(t, c, log=log)
            if what in ("phi", "all"):
                g = phi_from_passive(t, c, log=log)
                if g:
                    gate_verdict(g["Phi"], g["sigma"], g["ok"], log=log,
                                 source="passive")
                    _EXPORT["phi"] = g
                    _EXPORT["source"] = "passive: " + os.path.basename(path)
        else:
            log("  no `passive` segments in this file.")

    if what in ("coils", "mimo", "all"):
        res = []
        # The `pre` segment before each drive is the rest reference railed_frac
        # needs, so an offline replay excludes a dead pin the same way the live
        # pass does. Segment numbers are monotonic, so "the nearest earlier pre".
        pre_of = {}
        last_pre = None
        for k in sorted(segs):
            if segs[k]["phase"] == "pre":
                last_pre = segs[k]["c"]
            pre_of[k] = last_pre
        for k in sorted(segs):
            s = segs[k]
            if s["phase"] != "drive" or len(s["t"]) < 64:
                continue
            U = lockin(s["t"], s["u"], s["f"])
            if abs(U) < 1e-7:
                continue
            H, S = response(s["t"], s["c"], U, s["f"])
            name = min(MODES, key=lambda m: abs(m[1] - s["f"]))[0]
            res.append(dict(coil=s["coil"], mode=name, f=s["f"], amp=s["amp"],
                            H=H, sigma=S,
                            ok=railed_frac(s["c"], rest=pre_of.get(k))
                            <= RAIL_FRAC_REJECT,
                            swing=s["c"].max(axis=0) - s["c"].min(axis=0),
                            n=len(s["t"]), resid=0.0, peak=0.0))
        if not res:
            log("  no `drive` segments in this file -- nothing for `coils`/`mimo`.")
            return
        # Re-derive the DC matrix from the dc segments, if present.
        dcm = np.full((NCOLS, NCOLS), np.nan)
        for coil in range(NCOLS):
            ss = [s for s in segs.values() if s["phase"] == "dc" and s["coil"] == coil]
            if len(ss) == 2:
                hi = max(ss, key=lambda s: s["amp"])
                lo = min(ss, key=lambda s: s["amp"])
                if hi["amp"] != lo["amp"]:
                    dcm[:, coil] = (hi["c"].mean(axis=0) - lo["c"].mean(axis=0)) \
                        / (hi["amp"] - lo["amp"])
        if what in ("coils", "all"):
            coils_report(res, dcm, log=log)
        phase_compare(res, log=log)
        phase_consistency(res, log=log)
        g = mimo_gate(res, log=log)
        g["coils"] = sorted({p["coil"] for p in res})
        _EXPORT["phi"] = g
        _EXPORT["source"] = "driven: " + os.path.basename(path)
        _EXPORT["dcm"] = dcm


# ===========================================================================
# selftest -- the whole analysis chain against a plant with a KNOWN answer
# ===========================================================================
def selftest(log=print):
    """Synthetic three-mode plant, pushed through the real analysis functions.

    This validates the estimator BEFORE the bench, which is the only part of
    this tool that can be validated without the optic. It does NOT validate the
    drive, the guard or the unwind -- those are hardware behaviour and the
    simulator cannot reach them, the same limitation `README` records for the
    gain and the coil map.
    """
    rng = np.random.default_rng(20260815)
    log("\n  selftest: synthetic plant, 8 sensors x 8 coils x 3 modes")
    # Entries bounded away from zero, so "determined" is a statement about the
    # ESTIMATOR and not about a planted value that happened to land near zero.
    Phi = np.sign(rng.normal(size=(NCOLS, MODES_N))) * (0.5 + rng.random((NCOLS, MODES_N)))
    # The blind row is planted on a3 -- a HORIZONTAL channel -- and not on a6 as
    # it once was. a6/a7 are now known to be VERTICAL (see VERTICAL_CH), so this
    # census cannot grade them and a "blind" plant there would only exercise the
    # out-of-scope branch, losing the blind-detection test entirely.
    Phi[3] = 0.0                        # horizontal, sees nothing  -> DEAD
    Phi[6] = 0.0                        # vertical, sees nothing    -> ungradeable
    Phi[5, 2] = 0.0                     # a1's problem: determined at SOME modes
    # The coil directions are CONSTRUCTED, not drawn. A random 3x8 puts
    # near-parallel columns in by accident -- the first draw of this test had two
    # at cosine +0.997 -- and then the census is right and the test is wrong.
    # So: three orthogonal directions at full gain, three distinct combinations
    # at 1/20 gain, and two deliberate duplicates at 1/20 gain. The 1/20 is
    # coils 4-7's measured weakness; the duplicates are their real problem
    # (+0.966 pairwise on the rig, +1.000 here).
    Q = np.linalg.qr(rng.normal(size=(MODES_N, MODES_N)))[0]
    A = np.column_stack([
        Q[:, 0], Q[:, 1], Q[:, 2],                                # 0,1,2 unique
        0.05 * (Q[:, 0] + Q[:, 1] + Q[:, 2]) / math.sqrt(3.0),    # 3 unique, weak
        0.05 * (Q[:, 0] - Q[:, 1]) / math.sqrt(2.0),              # 4 unique, weak
        0.05 * (Q[:, 1] + Q[:, 2]) / math.sqrt(2.0),              # 5 unique, weak
        0.05 * Q[:, 0],                                           # 6 twins coil 0
        0.05 * Q[:, 1],                                           # 7 twins coil 1
    ])
    # The measured per-channel floors, `gains.json` measurement.sensor_noise_floor_v
    noise = np.array([0.00467, 0.02414, 0.00592, 0.00784,
                      0.00133, 0.01553, 0.02779, 0.02647])
    # Measured resting counts, 2026-08-06 (CLAUDE.md Sec 6 and Sec 8). a5 was not
    # in that table; a7 is moved to 900 on purpose, which is the only planted
    # value here that is not measured, to exercise the headroom branch.
    resting = np.array([601.2, 630.3, 686.5, 681.8, 567.5, 620.0, 860.6, 900.0])

    res = []
    fs = 300.0
    for coil in range(NCOLS):
        for mi, (name, f) in enumerate(MODES):
            amp, _ = choose_amp(coil, f, DWELL_S)
            t = np.arange(0, DWELL_S, 1.0 / fs)
            u = amp * np.sin(2 * math.pi * f * t)
            # on-resonance ramp: response grows linearly, 90 deg behind
            env = math.pi * f * t
            c = np.zeros((len(t), NCOLS))
            for i in range(NCOLS):
                c[:, i] = (Phi[i, mi] * A[mi, coil] * amp * env
                           * -np.cos(2 * math.pi * f * t)
                           + rng.normal(0, noise[i], len(t)))
            H = np.zeros(NCOLS, complex)
            S = np.zeros(NCOLS)
            U = lockin(t, u, f)
            for i in range(NCOLS):
                z, sd = lockin_sigma(t, c[:, i], f, ramp=True)
                H[i], S[i] = z / U, sd / abs(U)
            res.append(dict(coil=coil, mode=name, f=f, amp=amp, H=H, sigma=S,
                            ok=np.ones(NCOLS, bool), swing=np.zeros(NCOLS),
                            n=len(t), resid=0.0, peak=0.0))
    g = mimo_gate(res, log=log)
    cg = {r["coil"]: (r, gr) for r, gr, _ in census_coils(res, log=log)}

    # ---- the coil grades, against what was planted -------------------------
    # NOT "whatever came out". Each of these is a planted property: coil 7 was
    # built as a scaled copy of coil 3, and coils 4-6 at 1/20 gain. Note what is
    # deliberately NOT asserted: that a weak coil grades badly. A coil at 1/20
    # gain that still drives all three modes independently is a good coil that
    # needs more volts, and grading it DEAD would be wrong.
    want_c = {0: "OK", 1: "OK", 2: "GOOD", 3: "GOOD",
              4: "GOOD", 5: "GOOD", 6: "OK", 7: "OK"}
    twin_c = {0: 6, 6: 0, 1: 7, 7: 1}
    log("\n  planted coil directions vs recovered. Coils 3-5 are at 1/20 gain and")
    log("  should still be GOOD -- weak is not the same as redundant. Coils 6,7")
    log("  duplicate coils 0,1, so all four are flagged and none is singled out.")
    okcoil = True
    for j in range(NCOLS):
        row, got = cg[j][0], cg[j][1]
        bad = got != want_c[j] or (j in twin_c and row["twin"] != twin_c[j])
        okcoil = okcoil and not bad
        log("    coil %d  want %-4s  got %s  auth %.2f  nearest coil %d at %+.4f  %s"
            % (j, want_c[j], INK.pad(got, GRADE_COLOUR[got], 4),
               row["authority"], row["twin"], row["cos"],
               "" if not bad else "<-- WRONG"))
    log("  %s" % ("PASS -- duplicates caught, weak-but-unique coils not punished."
                  if okcoil else "FAIL -- see above."))

    # ---- the sensor grades, on an undriven record --------------------------
    log("\n  passive record from the same Phi: 240 s at 200 Hz, ambient only.")
    fsp, span = 200.0, 240.0
    tp = np.arange(0, span, 1.0 / fsp)
    q = np.column_stack([20.0 * np.sin(2 * math.pi * f * tp + rng.uniform(0, 2 * math.pi))
                         for _, f in MODES])          # counts of modal coordinate
    cp = resting + q @ Phi.T + rng.normal(0, 1.0, (len(tp), NCOLS)) \
        * (noise / COUNTS_TO_V)
    sres = sensors_analyse(tp, np.clip(np.round(cp), 0, ADC_MAX_COUNTS), log=log)
    sg = {r["ch"]: grade_sensor(r)[0] for r in sres}
    # One planted case per branch of grade_sensor, so no branch is untested.
    want = {0: "GOOD", 1: "GOOD", 2: "GOOD",
            3: "DEAD",      # Phi[3] zero, HORIZONTAL   -> blind on the optic
            4: "GOOD",
            5: "OK",        # Phi[5,2] zero             -> 2 of 3 modes
            6: "?",         # Phi[6] zero, VERTICAL     -> out of scope, not dead
            7: "OK"}        # resting 900 counts        -> no headroom to drive
    log("\n  planted grade vs recovered:")
    for i in range(NCOLS):
        log("    a%d  want %-4s  got %s   %s"
            % (i, want[i], INK.pad(sg[i], GRADE_COLOUR[sg[i]], 4),
               "" if sg[i] == want[i] else "<-- WRONG"))
    oksens = sg == want
    log("  %s" % ("PASS -- every grade is the planted one."
                  if oksens else "FAIL -- see above."))

    log("\n  --- selftest verdict ---")
    truth = [i for i in range(NCOLS) if i not in (3, 5, 6)]
    log("  planted determined rows: %s  (a3,a6 zero at all modes, a5 zero at C)"
        % truth)
    log("  recovered:               %s" % g["rows"])
    okrows = set(g["rows"]) == set(truth)
    # Direction check: recovered Phi should be parallel to the planted one.
    cos = []
    for mi in range(MODES_N):
        a = np.real(g["Phi"][:, mi])
        b = Phi[:, mi]
        if np.linalg.norm(a) > 0:
            cos.append(abs(float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))))
    log("  |cos(recovered Phi, planted Phi)| per mode: %s"
        % " ".join("%.4f" % x for x in cos))
    okdir = all(x > 0.98 for x in cos)

    # ---- the GEOMETRIC export, exactly ------------------------------------
    # This is the path a bench run now takes by default, so it is checked
    # numerically rather than trusted. Plant Phi = the geometry and an A with
    # mixed signs, put the response 90 degrees behind the drive as resonance
    # requires, and the projection must return that A to the last digit -- no
    # gauge, no sign search. A sign error here is invisible on the rig until the
    # loop pumps, which is what happened on 2026-08-17.
    Ap = np.array([[+2.0, -1.5, +0.8, +1.1],
                   [-0.9, +2.2, +1.3, -0.4],
                   [+1.7, +0.6, -2.1, +0.9]])
    gres = [dict(mode=nm, coil=j,
                 H=(GEO_PHI[:, mi] * Ap[mi, j] * (-1j)).astype(complex),
                 sigma=np.full(NCOLS, 0.01), ok=np.ones(NCOLS, bool))
            for mi, (nm, _) in enumerate(MODES) for j in range(4)]
    gg = mimo_gate(gres, log=lambda *_a: None)
    gg["coils"] = [0, 1, 2, 3]
    rec_a = -gg["A_geo"][:, :4].imag
    err = float(np.abs(rec_a - Ap).max())
    gph, gex = pick_export(gg, geo=True)
    with tempfile.TemporaryDirectory() as td:
        gout = save_modal(gph, gex, os.path.join(td, "modal_geo.json"),
                          "driven: selftest", log=lambda *_a: None, geo=True)
    okgeo = (err < 1e-9 and gout["basis"] == "geometric"
             and gout["phi"]["rows"] == [0, 1, 2, 3]
             and gout["phi"]["gauge"] == gout["a"]["gauge"]
             and max(gout["a"]["imag_frac"]) < 1e-9
             and np.allclose(gout["a"]["value"], np.pad(Ap, ((0, 0), (0, 4)))))
    log("\n  geometric export: max |A recovered - A planted| = %.2e over %d coils"
        % (err, 4))
    log("  basis %s, rows %s, one gauge %s, off-quadrature %.1e"
        % (gout["basis"], gout["phi"]["rows"],
           gout["phi"]["gauge"] == gout["a"]["gauge"],
           max(gout["a"]["imag_frac"])))
    log("  %s" % ("PASS -- a known A comes back exactly, with its signs, from a"
                  " geometric Phi." if okgeo else
                  "FAIL -- the geometric projection does not invert its own plant."))

    allok = okrows and okdir and okcoil and oksens and okgeo
    log("\n  %s" % ("PASS -- the gate recovers a planted Phi, rejects the blind"
                    " sensor, and the census grades match what was planted."
                    if allok else "FAIL -- see above."))
    return 0 if allok else 1


# ===========================================================================
def resolve_port(explicit):
    """Autodetect by USB VID, exactly the way `bench.py` does for the controllers.

    REUSED, not reimplemented: two copies of one hardware fact drift, which is
    how `bench.py` and `harness.py` ended up with version regexes that stopped
    agreeing. Imported lazily, so `--replay`, `--dry-run` and `--selftest` never
    need pyserial and never touch a port.
    """
    if explicit:
        return explicit
    try:
        import bench
    except Exception as e:                 # SystemExit deliberately propagates:
        print("  cannot autodetect (%s) -- falling back to %s" % (e, PORT))
        return PORT                        # its pyserial message is the right one
    return bench.choose_port(None)


def probe_baud(port):
    """The board's actual rate. Delegates to pyDAC2, which owns the candidate
    list and the probe loop for every tool in the tree -- a second copy here
    would be the same drift as the two version regexes `ladder.py` exists to
    prevent. Never assume the rate: at the wrong one the probe scans 200 lines
    at a 2 s timeout, which is up to 400 s of total silence.
    """
    import pyDAC2
    return pyDAC2.resolve_baud(port)


def open_dac(port, baud=None):
    from pyDAC2 import FastDAC
    dac = FastDAC(port=port, baud=baud) if baud else FastDAC(port=port)
    dac.start_stream()
    return dac


def preflight_stream(dac, seconds=PREFLIGHT_S):
    """Confirm samples are ARRIVING before occupying the optic.

    `bench.py` has had a preflight since it was written, and its docstring says
    why: opening the port successfully proves nothing, and the failure that
    matters -- board not flashed, sketch at another baud, wrong tty, stream not
    started -- looks exactly like a quiet rig. This file had no equivalent, so
    on 2026-08-17 three runs went to disk with a header and no rows: 60 s of
    `sensors` and the first 51 s of a 36-minute `coils` pass, all spent reading
    a stream that was not there.

    Cheap, and it happens before any coil is touched.
    """
    rdr = Reader(dac)
    t0 = time.time()
    n = 0
    while time.time() - t0 < seconds:
        if rdr.read() is not None:
            n += 1
    el = time.time() - t0
    hz = n / el if el > 0 else 0.0
    if hz < PREFLIGHT_MIN_HZ:
        sys.exit(
            "\n  THE BOARD IS NOT STREAMING: %d samples in %.1f s (%.1f Hz,"
            " need %.0f).\n"
            "  The port opened and the sketch answered READY, so this is the"
            " stream itself.\n"
            "  In order of likelihood:\n"
            "    - the sketch is not the one in arduino.ino -- reflash with"
            " `make arduino`\n"
            "    - it is streaming a different channel count than the %d this"
            " tool reads\n"
            "    - `STREAM` was refused; check `make ports` picked the right"
            " device\n"
            "  Nothing was driven and no coil was moved."
            % (n, el, hz, PREFLIGHT_MIN_HZ, NCOLS))
    print("  stream: %d samples in %.1f s (%.0f Hz)" % (n, el, hz))
    return hz


def plan(coils, multi=False, damp=False, dc=True):
    npt = 1 if multi else MODES_N
    pre = MULTI_PRE_S if multi else PRE_S
    unw = 0.0 if damp else UNWIND_S + QUIET_S
    # the settle is PER POINT when `damp` is set, plus ~6 s of port handover
    # (the board resets on open) -- not per coil
    stl = (SETTLE_CTL_S + 6.0) if damp else 0.0
    per = (DC_STEP_S * 2 if dc else 0.0) + npt * (pre + DWELL_S + unw + stl) \
        + (0.0 if damp else SETTLE_BETWEEN_COILS_S)
    print("\n  plan: %d coils x %s" % (
        len(coils), "1 multisine point carrying all %d modes" % MODES_N
        if multi else "%d modes, one at a time" % MODES_N))
    print("    per point : pre %.0fs + drive %.0fs%s"
          % (pre, DWELL_S, "" if damp else
             " + unwind %.0fs + quiet %.0fs" % (UNWIND_S, QUIET_S)))
    if damp:
        print("    settle    : %.0fs of `%s` between POINTS, not just coils --"
              " the controller replaces the unwind" % (SETTLE_CTL_S, SETTLE_VERSION))
    print("    per coil  : %s%d point(s)%s = %.1f min"
          % ("DC 2x%.0fs + " % DC_STEP_S if dc else "", npt,
             "" if damp else " + settle %.0fs" % SETTLE_BETWEEN_COILS_S,
             per / 60.0))
    print("    total     : %.1f min\n" % (len(coils) * per / 60.0))
    print("    coil  mode        f Hz    seed cts/V   ramp   amp V   predict cts")
    for c in coils:
        for name, f in MODES:
            amp, seed = choose_amp(c, f, DWELL_S)
            g = ramp_gain(f, DWELL_S)
            print("     %d     %s      %.4f    %8.0f   %5.0f  %.4f   %8.0f%s"
                  % (c, name, f, seed, g, amp, amp * seed * g,
                     "  <-- at AMP_MAX" if amp >= AMP_MAX else ""))
    print("\n  AMP_MAX is %.2f V against a %.2f V worst-case half-window (the"
          % (AMP_MAX, float(np.min(np.minimum(BIAS_CH - VMIN, VMAX - BIAS_CH)))))
    print("  bias is per-channel now, so the binding coil is the one nearest a")
    print("  rail: %s V). The rail guard unwinds at %d/%d counts and each point"
          % (" ".join("%.2f" % v for v in BIAS_CH), RAIL_GUARD_LOW,
             RAIL_GUARD_HIGH))
    print("  is retried at half amplitude.")


def main():
    ap = argparse.ArgumentParser(
        description="per-sensor and per-coil capability census, the MIMO gate, "
                    "and a drift-tracking calibration pulse")
    ap.add_argument("what", nargs="?", default="sensors",
                    choices=["sensors", "coils", "mimo", "phi", "pulse", "all"])
    ap.add_argument("--port", default=os.environ.get("PORT") or None,
                    help="serial port; autodetected by USB vendor ID if omitted")
    ap.add_argument("--coils", default="0,1,2,3,4,5,6,7",
                    help="which coils the active pass drives")
    ap.add_argument("--seconds", type=float, default=60.0,
                    help="passive census length; `phi` wants 300 s or more")
    ap.add_argument("--window", type=float, default=PHI_WINDOW_S,
                    help="`phi` averaging window; the record is split into these")
    ap.add_argument("--dwell", type=float, default=None,
                    help="override DWELL_S (SNR is linear in it)")
    ap.add_argument("--target", type=float, default=None,
                    help="override TARGET_SWING_COUNTS")
    ap.add_argument("--no-dc", action="store_true", help="skip the DC step pass")
    ap.add_argument("--multisine", action="store_true",
                    help="drive all three modes at once, one point per coil "
                         "instead of three. NEGATIVE RESULT ON RESONANCE, "
                         "2026-08-18: it improved the per-coil phase spread "
                         "(73-89 deg -> 31.6/36.7/7.8) and BROKE the signs, 7 of "
                         "12 against the one-at-a-time pass's 12 of 12, because a "
                         "ramping response has 1/f^2 skirts that leak the loudest "
                         "mode into the quietest mode's bin. Kept, but do not "
                         "spend 18 minutes rediscovering that")
    ap.add_argument("--settle", action="store_true",
                    help="damp between POINTS rather than waiting tau > 138 s for "
                         "the ring: in-process with `ModalDamper` when "
                         "data/modal.json loads, otherwise by handing the port to "
                         "`bench.py %s`. The in-process damper is UNPROVEN -- see "
                         "its docstring" % SETTLE_VERSION)
    ap.add_argument("--hand-damp", action="store_true",
                    help="stop between coils and ask a human to still the plate "
                         "by hand. tau > 138 s, so this is the only affordable "
                         "way to start a point from rest")
    ap.add_argument("--pulse-coil", type=int, default=PULSE_COIL)
    ap.add_argument("--repeats", type=int, default=PULSE_REPEATS)
    ap.add_argument("--save-signature", action="store_true",
                    help="store this pulse as the reference to drift against")
    ap.add_argument("--derive-a-dc", action="store_true",
                    help="derive a PROVISIONAL A from the DC matrix and Phi, so "
                         "the modal loop can be exercised off the bench. Not a "
                         "measurement of A; osem.zeta.py refuses it by default")
    ap.add_argument("--phi", default="geo", choices=["geo", "svd"],
                    help="which Phi goes in the export. `geo` (default) takes it "
                         "from the rig geometry -- exact, signed, no gauge, and A "
                         "is projected onto it. `svd` takes both halves from the "
                         "rank-1 fit, which is what ran until 2026-08-17 and "
                         "carries a per-mode sign nothing offline can check")
    ap.add_argument("--save-modal", nargs="?", const=MODAL_PATH, default=None,
                    metavar="PATH",
                    help="write Phi (and A, if `coils` ran) where osem.zeta.py "
                         "can load it; default data/modal.json")
    ap.add_argument("--signature", default=SIGNATURE_PATH)
    ap.add_argument("--freqs", default=None, metavar="fA,fB,fC",
                    help="drive/analyse at THESE mode frequencies instead of "
                         "MODES. The `sensors` pass prints the ones to use; the "
                         "modes drift, by up to 0.0170 Hz over the 11 days to "
                         "2026-08-17 and 0.0068 Hz again by 2026-08-20")
    ap.add_argument("--baud", type=int, default=None,
                    help="serial baud; probed via pyDAC2 if omitted")
    ap.add_argument("--color", default="auto", choices=["auto", "always", "never"],
                    help="colour the census tables; auto = only to a terminal")
    ap.add_argument("--replay", default=None, help="analyse a recorded CSV, no bench")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and every amplitude, open no port")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    global DWELL_S, UNWIND_S, TARGET_SWING_COUNTS, INK, MODES
    INK = Ink(a.color)
    if a.freqs:
        # Rebind MODES rather than threading a frequency list through twelve
        # call sites. Every function here reads MODES at call time, and the names
        # A/B/C are positional -- they identify which mode, not which frequency.
        fs = [float(x) for x in a.freqs.split(",") if x.strip()]
        if len(fs) != MODES_N:
            sys.exit("  --freqs needs %d comma-separated values, got %d"
                     % (MODES_N, len(fs)))
        was = MODES
        MODES = tuple((was[i][0], fs[i]) for i in range(MODES_N))
        print("  MODES overridden: %s"
              % ", ".join("%s=%.5f Hz (was %.5f)" % (n, fs[i], was[i][1])
                          for i, (n, _) in enumerate(MODES)))
    if a.dwell:
        DWELL_S = UNWIND_S = a.dwell
    if a.target:
        TARGET_SWING_COUNTS = a.target
    coils = [int(x) for x in a.coils.split(",") if x.strip() != ""]

    if a.selftest:
        sys.exit(selftest())
    if a.replay:
        replay(a.replay, a.what)
        if a.save_modal:
            do_export(a.save_modal, a.phi == "geo", a.derive_a_dc)
        return
    if a.dry_run:
        plan(coils, a.multisine, a.settle, not a.no_dc)
        return

    # Line-buffer stdout. Python block-buffers when it is not writing to a
    # terminal, so every print here is invisible for 8 kB at a time the moment
    # the output is piped, redirected or captured -- which is how a bench log
    # gets kept. On 2026-08-17 that turned a diagnosable failure into "no
    # output, idek if it's doing anything".
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    # Make a plain `kill` behave like Ctrl+C. Python's default SIGTERM handler
    # terminates the interpreter WITHOUT unwinding, so the `finally` below never
    # runs and every coil stays at whatever voltage the drive last set it to.
    # That happened on 2026-08-17: `pkill -f "status.py coils"` left a coil
    # mid-sine and the eight coils had to be parked by hand afterwards. Raising
    # KeyboardInterrupt routes SIGTERM through the same unwind Ctrl+C uses, so
    # the `finally` parks them either way.
    def _term(signum, frame):
        raise KeyboardInterrupt("SIGTERM")
    for sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _term)
        except (ValueError, OSError, AttributeError):
            pass                       # not on the main thread, or no SIGHUP

    print("\n  status.py -- %s   (git %s)" % (a.what, _git_rev()))
    print("  NOTHING IS DAMPING while this runs. Ctrl+C returns all coils to bias.")
    # Port FIRST, recorder second: a failed autodetect used to leave an empty
    # CSV in data/ for every attempt, and the one thing this directory must not
    # accumulate is files that look like records and are not.
    port = resolve_port(a.port)
    baud = a.baud or probe_baud(port)
    rec = Recorder(a.what)
    dac = None
    try:
        dac = open_dac(port, baud)
        preflight_stream(dac)
        if a.what in ("sensors", "all"):
            sensors_run(dac, a.seconds, rec)
        if a.what in ("coils", "all"):
            plan(coils, a.multisine, a.settle, not a.no_dc)
            def _pause(coil):
                print("\n  >>> STILL THE PLATE BY HAND, then press Enter for coil"
                      " %d." % coil)
                print("      A point that starts on the last one's ring is what"
                      " ruined the 23:12 pass.")
                try:
                    input()
                except EOFError:
                    print("      (no terminal -- continuing without the pause)")
            def _reopen():
                # A failed reopen must not end the pass: everything measured so
                # far is already on disk, but the remaining points are not.
                last = None
                for k in range(4):
                    try:
                        d = open_dac(port, baud)
                        preflight_stream(d)
                        return d
                    except Exception as exc:
                        last = exc
                        print("      reopen attempt %d failed (%s); waiting %.0fs"
                              % (k + 1, type(exc).__name__, PORT_SETTLE_S * (k + 1)))
                        time.sleep(PORT_SETTLE_S * (k + 1))
                raise last
            res, dcm = coils_run(dac, coils, rec, dc=not a.no_dc,
                                 multi=a.multisine, damp=a.settle,
                                 pause=_pause if a.hand_damp else None,
                                 reopen=_reopen)
            coils_report(res, dcm)
            phase_compare(res)
            phase_consistency(res)
            g = mimo_gate(res)
            g["coils"] = sorted({p["coil"] for p in res})
            _EXPORT["phi"] = g
            _EXPORT["source"] = "driven: " + os.path.basename(rec.path)
            _EXPORT["dcm"] = dcm
        if a.what == "phi":
            rdr = Reader(dac)
            park(dac)
            rec.mark("passive")
            print("\n  quiet record for Phi: %.0f s, nothing driven." % a.seconds)
            t, c, _, _ = acquire(dac, rdr, rec, a.seconds)
            rdr.report()
            g = phi_from_passive(t, c, a.window)
            if g:
                gate_verdict(g["Phi"], g["sigma"], g["ok"], source="passive")
                _EXPORT["phi"] = g
                _EXPORT["source"] = "passive: " + os.path.basename(rec.path)
        if a.what == "mimo":
            print("\n  `mimo` needs drive data. Run `status.py coils`, or")
            print("  `status.py mimo --replay data/<file>_status_coils.csv`.")
        if a.what == "pulse":
            rdr = Reader(dac)
            rec.mark("passive")
            t, c, _, _ = acquire(dac, rdr, rec, 10.0)
            resting = [float(x) for x in c.mean(axis=0)] if len(c) else None
            shots = pulse_run(dac, rec, a.pulse_coil, a.repeats)
            if not shots:
                sys.exit("  no usable shots.")
            fp = pulse_fingerprint(shots, resting)
            if a.save_signature:
                os.makedirs("data", exist_ok=True)
                with open(a.signature, "w") as fh:
                    json.dump(fp, fh, indent=1)
                print("\n  signature -> %s" % a.signature)
                print("  Re-run `status.py pulse` any time to diff against it.")
            elif os.path.exists(a.signature):
                with open(a.signature) as fh:
                    pulse_compare(fp, json.load(fh))
            else:
                print("\n  no signature at %s -- run once with --save-signature."
                      % a.signature)
        if a.save_modal:
            do_export(a.save_modal, a.phi == "geo", a.derive_a_dc)
    except KeyboardInterrupt:
        print("\n  stopped by user.")
    finally:
        rec.close()
        if dac is not None:
            park(dac)
            time.sleep(0.2)
            try:
                dac.stop_stream()
                dac.close()
            except Exception:
                pass
        print("  all coils returned to bias.")


if __name__ == "__main__":
    main()
