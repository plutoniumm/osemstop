#!/usr/bin/env python3
"""iota -- theta's law under ONE DECLARED ACTUATOR BUDGET, divided by NEED. Numerically theta by default.

    x     = [q_m, v_m] per mode + one DC state per sensor
    y_i   = sum_m Phi[i,m] q_m + d_i + n_i
    f_m   = -w_m K_m * (g_m * Proj qdot)_m             per-mode WEIGHT, then kernel
    u_C   = A_C+ f                                     min-norm, row-norm balanced
    u_j   = w_d * c_j * Kp * (-vel_j)                  coils A does not cover
    |u - bias| <= BUDGET_V on every coil               ONE number, solved on the TOTAL

THE SHIPPED DEFAULT IS theta, BIT FOR BIT, AND THAT IS THE POINT OF THE FILE.
BUDGET_TILT is 0.0, at which every claimant's weight is EXACTLY 1.0 -- for every
possible need vector, including zeros, nans and infinities, by construction and
not by rounding (see BudgetAllocator._weights) -- so f_m = -K_m (Proj qdot)_m,
the hybrid gains are HYBRID_GAIN unchanged, and every number this file produces
is theta's, which is in turn eta's at KERNEL_PHASE = 0. `--selftest` asserts that
against `osem.theta.py` itself, both controllers driven with the same synthetic
sensor history and their commanded voltages compared sample by sample through
CALIBRATING -> FAULT -> CALIBRATING -> DAMPING. WHY it matters that the default is
not merely close: theta and eta both have fixes pending and bench runs planned, so
those fixes must PORT into this file as a merge and not a reconciliation -- and
an allocator has more free parameters than a fixed split, while NOTHING OFFLINE
CATCHES A WRONG ONE. That is measured, not a caution: on 2026-08-17 a wrong
per-mode sign of A passed the selftest, the modal gate and the colocation check
and took the median channel ratio from 1.5 (diagonal) to 2.3 (modal) -- it PUMPED.

WHAT THE BUDGET IS. One number, BUDGET_V = MODAL_TOTAL_HEADROOM * BIAS_SWING =
0.225 V, the demand about bias any single coil may carry. That is theta's cap
unchanged; what is new is that it is DECLARED, that every other cap in the file
is quoted against it in one ledger, and that the split of it between control
terms is a computed allocation instead of an accident. The split being implicit
is not a stylistic complaint: it is how coil 3 reached 0.4229 V against a
0.250 V half-window -- 169 % -- and clipped, while the modal allocation stayed
inside its own 0.20 V cap for the entire run (CLAUDE.md Sec 2). Once a coil clips
the realised force is no longer A_C+ f and the dissipation guarantee is void.

WHO THE CLAIMANTS ARE, AND WHY THEY ARE THE MODES. Four: the three modes, and the
per-channel (hybrid diagonal) block as one. Per-mode is the granularity the data
argues for. Against a control, modal damping wins 5.9x on mode B (3.39 counts
against 19.86) and 6.7x on mode C (2.37 against 15.80), but on mode A it is 6.31
against a best diagonal of 6.86 -- INSIDE the run-to-run scatter of a factor 2-3,
so its advantage there is not established (CLAUDE.md, per-mode residual). With the
loop closed A/T1 is the LOUDEST mode. MODAL_KP is flat [0.035, 0.035, 0.035], so
today nothing can express "this mode needs more than that one". This is the object
that can.

WHAT NEED IS, AND WHY IT IS NOT `ratio`. THIS IS THE CRUX OF THE RUNG. `ratio` is
amplitude over the CALIBRATION baseline and calibration runs at ZERO GAIN, so it
measures the loop's own success and not the room: measured 2026-08-18, the same
physical quiet reads 0.117 with the modal law engaged and 1.449-1.832 with the
diagonal law -- a factor of 13 for the SAME plate (data/20260818_002207_jerk_eta.log,
data/20260818_001841_fast_lock.csv). An allocator keyed on it would hand budget to
whichever claimant is already winning, purely because it is winning: a positive
feedback loop closed through the measurement. It can also FREEZE -- 3.59 identical
for 1085 s on 2026-08-17 -- so a ramp keyed on it can latch.

Need is instead the loop's OWN DISSIPATION RATE per claimant,

    need_m = K_m qdot_m^2        need_diag = sum_j |g_j| vel_j^2

which is the power that claimant is removing. In a steady state the power removed
equals the power injected, and the injected power is a property of the ROOM, not
of the gain: doubling K halves <qdot^2> and the product stands still. So need
answers "how hard is this part of the system being driven", which is the question,
rather than "how quiet is it", which is the one that feeds back.

THE RESIDUAL DEPENDENCE IS BOUNDED, and the bound comes off measured decay rates.
The cancellation is exact only if the loop is the only dissipation. The plant's own
is 0.0072 /s (tau > 138 s, analysis/ringdown.md) against the modal loop's 0.1393 /s
(CLAUDE.md), so the loop retains 0.1321/0.1393 = 94.8 % of the injected power at
w = 1, 82.1 % at the w = 0.25 floor and 97.9 % at w = 2.5 -- the statistic moves at
most 1.19x from the WEIGHT itself across the whole clip range, against `ratio`'s
measured 13x between laws. Simulated end to end in the selftest: over a 10x range of
loop gain, rms motion moves 2.90x while need moves 1.24x. THAT SIMULATION IS A
MODEL, and the 1.19x is an argument from two measured rates, not a measurement.

WHAT BREAKS IT, and none of these is hypothetical.
  * A NARROWBAND DISTURBANCE ON A MODE. The invariance needs a disturbance whose
    spectrum is smooth across the mode's linewidth. At Q > 433 that half-width is
    under 0.00083 Hz, so a machinery or mains line would have to sit within a
    milliHertz of a mode -- but if one did, the injected power itself would depend
    on the gain and the statistic would follow the loop again.
  * TRANSIENTS. Dissipation lags injection, so a kick biases the allocation toward
    whichever mode it excited. Over BUDGET_TAU_S = 20 s against a measured re-quiet
    of 5.3 s (modal) / 17.4 s (diagonal) that is partly averaged and not removed.
  * ZERO GAIN. In CALIBRATING and FAULT every need is 0 by construction. That is
    the freeze failure in a new dress, and it is handled by not latching: see below.
  * PER-MODE INTRINSIC DAMPING is unmeasured -- analysis/ringdown.md gives ONE
    number for the plant -- so the retained fraction differs per mode by an amount
    nothing here knows, and that biases the shares.
  * qdot IS AN ESTIMATE. A wrong Phi row leaks one mode's motion into another's
    need. chi2/dof is the detector and it is logged and acted on by nothing.

IT CANNOT LATCH, and that is a design constraint rather than a hope. When the
statistic is unavailable -- not DAMPING, modal refused, gains at zero, or motion at
or under the actuator's own floor -- the allocator does not freeze where it was: it
RAMPS BACK TO FLAT at the same time constant. Flat is theta's split, the only split
that has ever been on this rig. A statistic that stops arriving therefore returns
the loop to the configuration with measurements behind it, and the held time is
counted and printed.

SLOW, AND HOW SLOW IS DERIVED. The slowest mode is 0.72194 Hz, a 1.385 s period, so
CLAUDE.md Sec 17 asks for a reallocation time constant of order 10 s or more.
BUDGET_TAU_S = 20 s is 14.4 periods of that mode and twice that floor, and the
weights are additionally slew-limited to 1/BUDGET_TAU_S = 0.05 per second, so a
weight moves at most 0.069 -- 6.9 % of nominal -- in one mode-A period. The
modulation corner 1/(2 pi tau) = 0.0080 Hz is 1.1 % of the slowest mode frequency
and about 9.5 half-widths off resonance at Q > 433, so the sidebands a time-varying
gain produces sit outside the resonance rather than inside it. THAT LAST NUMBER IS
ARITHMETIC ON A MEASURED Q, not a measurement of anything this loop does.

DISSIPATIVITY OF A TIME-VARYING BLEND: PARTLY SETTLED, AND THE REST IS OPEN.
CLAUDE.md Sec 17 lists it as an open question and it stays open, but part of it can
be closed by writing it down. The modal energy sum_m (qdot_m^2 + w_m^2 q_m^2)/2
does not contain the weights, and each term's power is -w_k K_k qdot^2 <= 0
POINTWISE for any w_k >= 0, so a non-negative time-varying reweighting of two
pointwise-dissipative laws is pointwise dissipative -- no rate bound needed, and no
cross term appears. WHAT IS STILL OPEN is the part that argument assumes away: the
loop acts on an ESTIMATED velocity, and modulating the gain moves the product to
frequencies where the estimator's phase error is different. Nothing here bounds
that, which is the real reason for the slow ramp and the reason the default is
pinned. Every weight is clipped non-negative and floored at BUDGET_SHARE_FLOOR, so
the sign half of the argument holds unconditionally.

WHAT THE TALMUD ANALOGY GIVES AND WHAT IT DOES NOT. The shape is taken from it --
one declared pot, division by need, nobody hoarding what they are not using, so a
quiet mode does not sit on authority a ringing one needs. The FORMULA is not:
Aumann and Maschler's contested-garment rule divides a FIXED estate among claims
that are known and static, and here the estate's usable size depends on what is
already being demanded, the claims are noisy real-time estimates, and paying one
claimant changes every other claimant's future claim. That is a feedback loop, not
a division problem. PROPORTIONAL DIVISION WITH A FLOOR IS THEREFORE A CHOICE, and
it is justified only by two properties: at equal need it reduces EXACTLY to today's
flat split, which the equivalence constraint requires anyway, and it is closed form
with no iteration inside a 10 ms control step.

THE OLD GAIN SCHEDULE IS INERT, AND IOTA DEPENDS ON THAT. CAPTURE_GAIN and
STEADY_GAIN are identical vectors, so the capture/steady interpolation runs every
step between two equal endpoints and changes nothing -- SCHEDULE_INERT asserts it.
That schedule was the previous attempt at allocation and it had three defects this
one must not reproduce: it REDUCED gain as the plate quietened, so the loop
withdrew authority exactly as it started succeeding; it was keyed on `ratio`, the
zero-gain-referenced statistic above; and it was a per-channel scalar, which cannot
express "mode B needs more than mode A". Two allocators keyed on different
references would fight, so there is only one live allocator here. If anyone makes
the endpoints differ again, arming the budget is REFUSED at construction with the
reason -- proof by refusal rather than by comment.

WHAT IS MEASURED AND WHAT IS NOT.
  * every number the allocator would act on is UNMEASURED on this rig: the
    distribution of need across the three modes has never been recorded, and
    whether the ramp should point toward modal or away from it at low disturbance
    is the RIG OWNER'S CLAIM and not a result. The per-mode residual data compares
    two laws at ONE ambient level; nothing compares them at two levels. So
    BUDGET_TILT ships at 0 and this file is theta until somebody measures that.
  * the per-channel block has NEVER RUN: |gain| was exactly 0.0000 on ch4, ch6 and
    ch7 for 100 % of DAMPING samples across both 2026-08-18 runs, demoted at engage
    by `inband-floor`. Its need is therefore 0 today whatever the room is doing,
    and it can only give budget up, never take it (BUDGET_W_DIAG_MAX).
  * modes 0.72194 / 0.99193 / 1.65607 Hz, empty room, 840 s, spread 0.00000 Hz.
  * Q > 433 (1 sigma), tau > 138 s (analysis/ringdown.md).
  * chi2 per dof is LOGGED and NOT acted on. It was considered as the need
    statistic and refused: its median is 0.14 with the modal law engaged and fitting
    against 18-21 with the gains off (n = 9914 and 35848, data/20260818_002229_fast_lock.csv)
    -- law-dependent by 130x, worse than `ratio` -- and it spikes to 888.74 at the
    instant of a hand kick, so it is a model-misfit statistic and not a disturbance
    one.
  * the kernel bank is present and ZEROED, exactly as in theta.
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

VERSION_TAG, BENCH_STATUS = "iota", "untested"
KIND = "controller"
# It closes its own loop and declares its own gain vectors, so bench.py prints the
# numbers that will be applied out of THIS file -- the check `stdlib.py` was
# allowed to survive by holding no constants. Named explicitly all the same,
# because the LAW here is theta's -- which is eta's at KERNEL_PHASE = 0 -- and a
# reader is entitled to know that the gains below are copied, not new ones. NOT ONE
# GAIN IN THIS FILE DIFFERS FROM theta's; what differs is who gets to spend them.
GAIN_SOURCE = "osem.iota.py"
LAW_SOURCE = "osem.theta.py"

PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 8

# ADC ai -> DAC channel. Measured pairing, bench/20260804/dcmatrix.log.
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]

FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend",
         "bias-trim", "soft-saturation", "baseline-floor",
         "sat-window", "fast-transport",
         "sample-guard", "decimate", "persist-baseline", "baseline-sanity",
         "kalman-velocity", "mains-null", "lock-quorum",
         "runaway-quorum", "runaway-peak", "trim-quiet",
         "modal-law", "modal-refuse", "modal-colocation", "modal-residual",
         "modal-kalman", "modal-chi2", "dead-pin",
         "inband-floor", "hybrid-diagonal", "fault-clear-live", "per-mode-gain",
         "modal-kernel", "budget-ramp")

# ---------------------------------------------------------------------------
# channels and gains  (bench.py reads these five vectors out of this module)
# ---------------------------------------------------------------------------
# a5 is a disconnected pin: exactly one distinct value, 0.0 counts, variance
# exactly zero, across 236387 samples.
ENABLE_CHANNEL = [True, True, True, True, True, False, True, True]

BIAS = np.full(N, 0.25)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 0.5, 2.0     # nominal; the trim moves the window

# Signs are per-channel residue signs from the measured response (tune.py's
# fit_modal). -0.035 is the ceiling: -0.040 is the documented rail onset and the
# least-evidenced number in the repo (analysis/kp040.md).
STEADY_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                        +0.000, +0.000, +0.000, +0.000])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                         +0.000, +0.000, +0.000, +0.000])
# Ki dissipates nothing and ate the headroom: peak 0.1135 -> 0.0056 V when it
# went to zero, which took total peak demand 0.2998 -> 0.200 V.
KI_GAIN = np.array([+0.0000, +0.0000, +0.0000, +0.0000,
                    +0.0000, +0.0000, +0.0000, +0.0000])
KD_GAIN = np.array([-0.00045, -0.00045, +0.00045, -0.00045,
                    +0.00000, +0.00000, +0.00000, +0.00000])

CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
# THE CAPTURE/STEADY SCHEDULE IS INERT, AND THE BUDGET ALLOCATOR DEPENDS ON IT.
# The two vectors above are IDENTICAL, so `steady + frac * (capture - steady)` in
# `_damping` interpolates between two equal endpoints: the machinery runs on every
# control step and changes nothing. It is left in place rather than deleted for the
# reason every rung on this ladder keeps its predecessor's code -- the default must
# stay bit-for-bit theta, and removing a live code path is not the way to prove that
# -- but it is NOT a second allocator, and two allocators keyed on different
# references would fight.
#
# WHAT IT WAS AND WHY IT IS NOT THE MODEL FOR THIS ONE. When the endpoints differed
# it was capture-aggressive -> steady-gentle keyed on `ratio`. Three defects:
#   1. SELF-DEFEATING. It reduced gain as the plate quietened, so the loop withdrew
#      authority exactly as it began to succeed.
#   2. WRONG ANCHOR. `ratio` is amplitude over a baseline measured at ZERO GAIN.
#      Measured 2026-08-18, the same physical quiet reads 0.117 under the modal law
#      and 1.449-1.832 under the diagonal law -- a factor of 13.
#   3. WRONG GRANULARITY. A per-channel scalar cannot express "mode B needs more
#      than mode A", which is what the per-mode residual data says.
# Arming the budget allocator while this schedule is live is REFUSED in
# `Controller.__init__`, with that reason. The refusal is the proof; a comment is
# not one.
SCHEDULE_INERT = bool(np.array_equal(STEADY_GAIN, CAPTURE_GAIN))
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
F_MODE_HZ = np.array([0.72194, 0.99193, 1.65607])
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
# STAYS 4.0, and the margin under it now comes from the mechanism instead of the
# clock. Reconstructing the breaker's internals from the two records of
# 2026-08-18, the longest continuous `high & growing` latch produced by a single
# hand kick was 4.00 s (modal ch2, diagonal ch3) against this 4.0 s sustain, so
# both runs faulted on the first kick, under both laws, with ZERO margin.
# stdlib.Breaker now also requires the envelope not to have RECEDED from its own
# recent peak, which cuts that same measured latch to 1.87-2.23 s -- a 1.8x margin
# on all eight channel-instances of both records, at the same 4.0 s.
# RAISING THIS TO 8.0 WAS TRIED AND REVERTED. It bought margin against an
# idealised smooth ring-up, and it cost two measured detections the simulator
# holds: the pumped-resonance scenario (peak ch2 ratio 2.58 against the 1.8 line)
# went uncaught, and the runaway-recovery scenario never tripped at all. A
# disturbance that grows for 4 s and does not come off its peak is what this
# breaker is for; the fix belonged in what "growing" means, not in how long the
# rig waits.
RUNAWAY_SUSTAIN_S = 4.0
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 10.0, 1.02

SAT_SUSTAIN_S, SAT_FRACTION = 1.30, 0.80
SAT_DECAY_FRAC = 0.98

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
# runs faulted on the first hand kick and sat there to the end of the run:
# diagonal FAULT 40.9 -> 159.6 s = 118.7 s (data/20260818_001841_fast_lock.csv),
# modal FAULT 38.5 -> 133.0 s = 94.5 s (data/20260818_002229_fast_lock.csv). The
# diagonal run MISSED THE CEILING BY 1.3 s. A ceiling longer than a whole jerk.py
# run is not a ceiling.
# The re-derivation, from measurements in this repo:
#   - the LOOP is what brings motion down, not the ringdown: measured re-quiet
#     after a matched kick is 5.3 s modal / 17.4 s diagonal (CLAUDE.md).
#   - with the gain at zero the plant decays at 0.0072 /s, tau > 138 s
#     (analysis/ringdown.md), so ringing down from a trip at ratio 3.8 to
#     FAULT_CLEAR_RATIO takes ln(3.8/1.4)/0.0072 = 139 s. ANY ceiling under that
#     is the operative path; the amplitude gate will not open first.
# 30.0 s is longer than the slowest measured re-quiet (17.4 s), so the loop gets a
# full chance to be the thing that fixes it before the ceiling is used, and far
# under the 138 s intrinsic. THE REMAINING FACTOR IS A CHOICE, NOT A MEASUREMENT:
# 30 s is 1.7x the 17.4 s re-quiet and 6x the FAULT_CLEAR_SUSTAIN_S a re-trip
# costs. The conservative direction is LONGER -- re-engaging into a disturbance
# that is still there costs one 5 s re-trip -- and 30 s is the shortest value that
# clears the measured re-quiet with margin.
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
# cycles of the slowest mode (0.72194 Hz, 1.385 s) so the envelope is a level and
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
BIAS_QUANTUM = 0.25            # coarse: 2 DOF, 4 knobs, so fine steps would
BIAS_MIN, BIAS_MAX = 0.25, 1.25          # just chase each other
BIAS_SWING = 0.25              # +-this around each channel's own bias
MID_COUNTS = 511.5             # (ADC_MAX_COUNTS - 1) / 2
TRIM_PERIOD_S = 15.0           # >= one ringdown at Q~50, f0~1 Hz (~16 s)
TRIM_DEADBAND_COUNTS = 40.0    # inside this, leave it alone
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
# Sign of d(counts)/d(bias volts) per channel.
#   a0-a3  from the per-channel decay fits (CLAUDE.md); a2 is mounted the other
#          way round, which is also why STEADY_GAIN[2] is positive.
#   a6,a7  MEASURED 2026-08-17 by DC step, +10.07 +-2.02 and +8.05 +-1.24
#          counts/V -- 5.0 and 6.5 sigma (slopesign.py, data/20260817_215348).
#   a4     +4.83 +-1.66, only 2.9 sigma: same sign, not yet resolved. Its hybrid
#          gain is live, so re-measure with a longer dwell before trusting it.
#   a5     no signal at all; its pin is at ground and it is disabled.
# The sigma is on the MEAN of the step, not the per-sample dither -- a 2-count
# shift is resolvable because the mean of ~3800 samples is ~60x quieter than one.
SLOPE_SIGN = np.array([-1.0, -1.0, +1.0, -1.0,
                       +1.0, -1.0, +1.0, +1.0])

# ---------------------------------------------------------------------------
# the hybrid law: modal on A's coils, per-channel velocity feedback on the rest
# ---------------------------------------------------------------------------
# One switch, so the bench can kill the whole feature without editing logic.
HYBRID_DIAGONAL = True
# COIL AUTHORITY, measured static |rigid| force: coils 4/6/7 give 3.10-3.60
# counts/V against coils 0-3's 9.92-34.28, i.e. 1/3 to 1/10 and NOT the ~1/100
# this repo used to claim (A_DC_COUNTS_PER_V). But their output is almost entirely
# T1 = mode A, already the best-damped -- modal velocity rms A 0.64 against B 2.61
# and C 3.15 -- so arming the hybrid adds authority where it is least needed and is
# NOT the fix for B and C. Per-mode gain is, and MODAL_KP ships flat because
# retuning it needs the bench.
# WHY A NOISY CHANNEL IS STILL WORTH DRIVING. Dissipation is LINEAR in gain, noise
# injection is QUADRATIC: energy removed goes as g x (the part of the signal
# correlated with true velocity), energy added as g^2 x (uncorrelated power). For
# small enough g the useful term always wins, so a noisy channel is not
# disqualified -- its GAIN IS CAPPED. And the cap is not a fudge: the
# minimum-variance velocity estimate from a noisy measurement is the measurement
# scaled by its coherent fraction, so gain proportional to coherent fraction is
# Wiener-optimal.
#
# MEASURED multiple coherence of each channel against a0-a3 as a block over the
# 0.6-1.8 Hz mode band, data/20260817_205211_status_sensors.csv (113418 samples,
# 300 s, 378 Hz, 26 Hann segments of 8192, multiple-coherence bias floor
# p/K = 3/26 = 0.12) -- raw -> bias-corrected fraction of in-band variance that IS
# the optic:  a0 1.000 -> 1.00,  a4 0.519 -> 0.46,  a6 0.347 -> 0.26,
# a7 0.559 -> 0.50.  a5 is 0.00: its pin is at hard ground.
# ONE 300 s ambient record. Re-measure it; these are not constants of the rig.
HYBRID_COHERENT = np.array([1.00, 1.00, 1.00, 1.00, 0.46, 0.00, 0.26, 0.50])
HYBRID_CHANNEL = [True, True, True, True, True, False, True, True]
# Magnitude is the shipped diagonal law's own |Kp|, not a new number. Sign is the
# measured SLOPE_SIGN, which equals the STEADY_GAIN sign on all four determined
# channels (a0-a3: -,-,+,-), so the same rule sets a4/a6/a7.
HYBRID_KP = 0.035
HYBRID_GAIN = HYBRID_KP * SLOPE_SIGN * HYBRID_COHERENT
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
#
# THE HYBRID CHANNELS HAVE NEVER RUN ON HARDWARE, and that is measured rather than
# assumed: across both runs of the 2026-08-17 evening the |gain| on ch4, ch6 and
# ch7 was exactly 0.0000 for 100 % of DAMPING samples, because the in-band floor
# check demotes them at engage. That check scores a channel against the three
# HORIZONTAL mode frequencies, and a4 senses X while a6/a7 sense Y -- orthogonal
# axes -- so they fail it BY CONSTRUCTION and not because they are blind (measured
# coherence with the optic 0.46 / 0.26 / 0.50). The fix belongs in the floor check
# and is being made in eta. NOTHING IN THIS FILE ASSUMES THOSE THREE ARE DEAD OR
# ABSENT: the kernel acts on the MODAL path, which is coils 0-3 through the
# allocator, and the hybrid diagonal term is summed after it exactly as in eta, so
# whatever the floor fix does to their gain it does the same thing here.

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
# true one, and it won by 2x -- warp/rigid 0.139 against 0.277 and 1.566 (dof.py
# on data/20260817_205211_status_sensors.csv).
#
# MODE IDENTIFICATION, 840 s empty room at 0.0116 Hz bins
# (data/20260817_215634_status_sensors.csv): 0.7283 Hz is 90.3 % T1, 0.9941 Hz is
# 88.2 % Z, 1.6530 Hz is 89.4 % T2. INDEPENDENT CONFIRMATION: the Phi measured
# from ambient motion by cross-spectral eigendecomposition, with no geometry as
# input, matches these vectors at |cos| 0.971 (A/T1), 0.978 (B/Z), 0.963 (C/T2).
#
# ROWS a4-a7 ARE ZERO ON PURPOSE -- DO NOT "FIX" THEM. They read axes orthogonal
# to Z/T1/T2; their measured coherence with the optic (0.46 / 0.26 / 0.50,
# bias-corrected) is cross-coupling from imperfect alignment, not their own view
# of these modes, and the hybrid diagonal path above already drives them.
PHI_BASIS = "geometric"
PHI_GEOM = np.zeros((N, NMODE))
PHI_GEOM[:4, 0] = [+1.0, +1.0, -1.0, -1.0]      # T1 tilt  -> mode A 0.72194 Hz
PHI_GEOM[:4, 1] = [+1.0, +1.0, +1.0, +1.0]      # Z normal -> mode B 0.99193 Hz
PHI_GEOM[:4, 2] = [+1.0, -1.0, +1.0, -1.0]      # T2 tilt  -> mode C 1.65607 Hz
# The leftover direction. No rigid-body motion produces it, so it is what the
# out-of-mode residual measures; it is real at 0.14-0.17 of rigid and its cause
# is NOT established (CLAUDE.md).
PHI_WARP = np.zeros(N)
PHI_WARP[:4] = [+1.0, -1.0, -1.0, +1.0]
# A channel with no Phi row watches an axis the in-band is not defined on, so its
# in-band amplitude is small BY CONSTRUCTION and the cross-channel in-band floor
# grades it blind whatever its health. Derived from the geometry rather than
# listed, so giving a4 a Phi row later automatically puts it back under the
# ordinary test. See Health.floor: these are judged on their own broadband motion.
INBAND_EXEMPT = ~PHI_GEOM.any(axis=1)
# Below this per-mode |cos| between a measured Phi and the geometric one, the
# geometry or the corner assignment is wrong. Warned, never a gate.
PHI_COS_WARN = 0.90

# STATIC rigid-body force per volt, counts/V, projected onto (T1, Z, T2) so the
# rows are in F_MODE_HZ order. Coils 0-3 from data/20260817_194726_status_coils.csv,
# coils 4/6/7 from data/20260817_215348_status_slopesign.csv. Coil 5 is a dead pin.
A_DC_COUNTS_PER_V = np.array([
    [-7.97, +28.62, +29.35, +22.43, -3.30, 0.0, +2.93, -3.01],      # A  T1
    [-1.22,  +8.03, -17.35, -16.18, +0.91, 0.0, -0.99, +0.42],      # B  Z
    [-5.78,  -2.12,  -3.60,  +8.52, -1.14, 0.0, -0.75, -0.63]])     # C  T2
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
A_DC_PROVENANCE = ("DC static, geometry basis, data/20260817_194726 + "
                   "data/20260817_215348; never closed-loop validated")

# ---------------------------------------------------------------------------
# modal data and law
# ---------------------------------------------------------------------------
MODAL_PATH = os.path.join(HERE, "data", "modal.json")
MODAL_SCHEMA = "osem-modal-1"

# PER MODE, and that is the point of a modal law. Ships FLAT: measured modal
# velocity rms A 0.64 / B 2.61 / C 3.15 over 28775 samples says B and C are 4-5x
# worse damped than A, but retuning needs the bench.
MODAL_KP = np.array([0.035, 0.035, 0.035])
MODAL_GAIN_SCALE = 0.5         # start at half the diagonal ceiling
MODAL_DEMAND_CAP_V = 0.20      # per-vector, uniform: direction preserved
MODAL_TOTAL_HEADROOM = 0.90    # of BIAS_SWING, on the TOTAL output not the modal
                               # part: the old cap let coil 3 reach 0.4229 V
                               # against a 0.250 V half-window
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
# the convolution kernel bank -- the only thing in this file that is not eta
# ---------------------------------------------------------------------------
# PER-MODE PHASE, degrees, POSITIVE IS A LAG (the loop acts on that mode later).
# ZERO IS eta, exactly, and zero is what ships.
#
# THIS DEFAULT IS NOT CAUTION, IT IS THE ONLY DEFENSIBLE VALUE. A phase here
# cancels a phase in the plant, and no measurement of the plant's phase exists:
# A's magnitudes are NOT established (CLAUDE.md Sec 3 -- the driven pass in
# data/modal.json is off-quadrature 0.624 / 0.749 / 1.196 where resonance wants
# ~0, and the two DC passes reproduce individual entries only to a factor
# 0.23-1.8). A's SIGNS are settled, 12 of 12 across three independent
# determinations, and that is the half of A this file's DEFAULT depends on --
# because at zero phase the kernel is the identity and the law is eta's, which
# needs only what eta needs. THE MOMENT THIS IS NON-ZERO IT DEPENDS ON THE
# MAGNITUDES TOO, and they are the thing Sec 16 says blocks this work.
#
# The knob is here so that when a clean A does exist, the change is a number in
# this vector and not a new control law written under bench pressure.
KERNEL_PHASE_DEG = np.zeros(NMODE)

# Derived, not chosen. On a tone at f_m a lag psi is identically
# cos(psi) qdot + sin(psi) omega q, and <q qdot> = 0 over a cycle, so the
# cycle-averaged modal power is -K cos(psi) <qdot^2>: at |psi| = 90 deg the modal
# term dissipates NOTHING and past it the sign flips and it PUMPS. Asserted
# numerically in the selftest, both the cos(psi) law and the refusal.
KERNEL_PHASE_HARD_CAP_DEG = 90.0

# 140 taps = 1.40 s at CONTROL_HZ. Set by the SLOWEST mode: 0.72194 Hz is a
# 1.385 s period = 138.5 control samples, and a causal filter reaches a phase
# LEAD only as a lag of one period minus it, so any phase in [0, 360) on mode A
# needs a full period of history plus the interpolation tap. Only two taps per
# mode are ever non-zero; the length is what the ring has to hold.
KERNEL_TAPS = int(np.ceil(1.0 / (float(F_MODE_HZ.min()) * CONTROL_PERIOD_S))) + 1
KERNEL_LEN_S = KERNEL_TAPS * CONTROL_PERIOD_S


def _parse_kernel_phase(spec):
    """OSEM_KERNEL_PHASE_DEG=a,b,c -> an array of NMODE degrees, or None.

    ATTENDED ONLY, and refused past the cap. Same pattern as OSEM_GAIN_RAMP: the
    bench can reach it without a diff, and it cannot reach a value the derivation
    above says pumps.
    """
    if not spec:
        return None
    parts = [p for p in spec.replace(",", " ").split() if p]
    if len(parts) == 1:
        parts = parts * NMODE
    if len(parts) != NMODE:
        raise ValueError("OSEM_KERNEL_PHASE_DEG wants %d values (or one for all "
                         "modes), got %r" % (NMODE, spec))
    psi = np.array([float(p) for p in parts])
    worst = float(np.max(np.abs(psi)))
    if worst >= KERNEL_PHASE_HARD_CAP_DEG:
        raise ValueError(
            "OSEM_KERNEL_PHASE_DEG %r asks for %.1f deg. The cycle-averaged modal "
            "dissipation goes as cos(psi), so at %.0f deg it is zero and past it the "
            "term PUMPS. This cap is a derivation, not a preference: if you mean to "
            "cross it, change the law, not the number."
            % (spec, worst, KERNEL_PHASE_HARD_CAP_DEG))
    return psi


class ModalKernel:
    """One causal FIR per mode, on the PROJECTED modal velocity.

        f_m = -K_m * sum_k g_m[k] * (Proj qdot)_m(t - k dt)

    g_m is the two-tap kernel with EXACTLY unit magnitude and EXACTLY phase
    -psi_m at f_m: with omega = 2 pi f_m dt, D = psi/omega, k1 = floor(D) and
    k2 = k1 + 1,

        g[k1] = sin(omega k2 - psi) / sin(omega)
        g[k2] = sin(psi - omega k1) / sin(omega)

    Two taps are enough because the target is a phase at ONE frequency, and the
    result is exact there rather than approximate: the design is solved, not
    windowed. At psi = 0 the first tap is sin(omega)/sin(omega) -- the same
    floating-point value divided by itself, so exactly 1.0 -- and the second is
    sin(0)/sin(omega), exactly 0.0. THAT IS WHY THE DEFAULT IS BIT-FOR-BIT eta
    and not merely close, and the selftest asserts it as an identity rather than
    with a tolerance.

    WHY THE KERNEL SEES THE PROJECTED VELOCITY. `Modal.project` returns
    Proj qdot, and `Modal.allocate` applies Proj again; at the shipped rank 3 Proj
    is the identity and the order is moot, but under rank reduction filtering
    inside the reachable subspace and re-projecting after is the conservative
    order -- it cannot move demand into a direction the coils cannot reach.

    WHAT THIS DOES NOT INHERIT. eta's dissipation argument is that
    -Proj K Proj qdot is symmetric PSD. A NON-ZERO PHASE VOIDS THAT: the kernel
    is not a positive scalar, the retained dissipation is cos(psi) of it, and the
    remainder is a stiffness term that moves the mode instead of damping it. At
    psi = 0 the argument is untouched because the kernel is the identity.
    """

    def __init__(self, f_hz, phase_deg, taps, dt):
        self.f = np.asarray(f_hz, float)
        self.nm = len(self.f)
        self.dt = float(dt)
        self.taps = int(taps)
        self.psi = np.asarray(phase_deg, float) * (np.pi / 180.0)
        self.w = 2.0 * np.pi * self.f * self.dt
        self.g = np.zeros((self.nm, self.taps))
        for m in range(self.nm):
            self.g[m] = self.build(self.w[m], self.psi[m], self.taps)
        self.hist = np.zeros((self.nm, self.taps))
        self.n_nonfinite = 0

    @staticmethod
    def build(w, psi, taps):
        """The two-tap kernel, exact at omega = w. psi wraps into [0, 2 pi)."""
        g = np.zeros(int(taps))
        psi = float(np.mod(psi, 2.0 * np.pi))
        k1 = int(np.floor(psi / w))
        k2 = k1 + 1
        if k2 >= taps:
            raise ValueError("a phase of %.4f rad at omega %.6f rad/sample needs "
                             "%d taps and there are %d. KERNEL_TAPS is set by the "
                             "SLOWEST mode; this one is faster and should fit."
                             % (psi, w, k2 + 1, taps))
        s = np.sin(w)
        g[k1] = np.sin(w * k2 - psi) / s
        g[k2] = np.sin(psi - w * k1) / s
        return g

    @property
    def identity(self):
        """True when every kernel is the unit impulse, i.e. this IS eta."""
        return bool(np.all(self.g[:, 0] == 1.0) and not np.any(self.g[:, 1:]))

    def response(self, m, f_hz=None):
        """The kernel's complex response at f_hz (its own mode by default)."""
        f = self.f[m] if f_hz is None else float(f_hz)
        k = np.arange(self.taps)
        return complex(np.sum(self.g[m] * np.exp(-1j * 2.0 * np.pi * f * self.dt * k)))

    def reset(self):
        self.hist[:] = 0.0

    def push(self, qd):
        """Advance one control step and return the filtered modal velocity.

        A non-finite input resets the ring and passes through: a nan in the modal
        velocity is already terminal for the estimator upstream, and there is no
        reason for the kernel to remember it 1.4 s longer than eta would.
        """
        qd = np.asarray(qd, float)
        if not np.isfinite(qd).all():
            self.n_nonfinite += 1
            self.reset()
            return qd
        self.hist[:, 1:] = self.hist[:, :-1]
        self.hist[:, 0] = qd
        return np.einsum("mk,mk->m", self.g, self.hist)

    def report(self):
        deg = self.psi * (180.0 / np.pi)
        out = ["[kernel] %d taps = %.2f s of modal-velocity history at %.0f Hz; "
               "2 taps non-zero per mode"
               % (self.taps, self.taps * self.dt, 1.0 / self.dt)]
        if self.identity:
            out.append("[kernel] per-mode phase %s deg -- IDENTITY. Every g_m is the "
                       "unit impulse, so this run is %s's law, numerically."
                       % (" / ".join("%.1f" % d for d in deg), LAW_SOURCE))
            out.append("[kernel] the extra freedom is present and ZEROED: there is no "
                       "measurement of the plant phase to cancel (CLAUDE.md Sec 3, "
                       "A's magnitudes).")
            return out
        out.append("[kernel] !! PER-MODE PHASE IS NON-ZERO: %s deg. THIS IS NOT eta'S "
                   "LAW and nothing offline catches a wrong kernel -- the 2026-08-17 "
                   "sign-of-A failure passed every offline check and PUMPED. ATTENDED "
                   "ONLY." % " / ".join("%+.1f" % d for d in deg))
        out.append("[kernel] cycle-averaged dissipation retained per mode: %s of the "
                   "psi = 0 value. The remainder is a STIFFNESS term -- it moves the "
                   "mode, it does not damp it."
                   % " / ".join("%.3f" % c for c in np.cos(self.psi)))
        for m in range(self.nm):
            k = np.nonzero(self.g[m])[0]
            h = self.response(m)
            out.append("[kernel]   mode %d  %.5f Hz  psi %+7.2f deg  taps %s  "
                       "|H| %.6f  arg H %+7.2f deg  group delay %.3f s"
                       % (m, self.f[m], deg[m],
                          " ".join("%d:%+.4f" % (int(i), self.g[m][i]) for i in k),
                          abs(h), np.degrees(np.angle(h)),
                          float(k.max()) * self.dt))
        out.append("[kernel] a LAG is a real time delay, so a transient on that mode "
                   "is acted on that late. A lead is realised as one period minus it, "
                   "which is the longest delay of all.")
        return out


# Resolved once, at import, so `bench.py` and the banner see the same vector the
# loop will use. Unset -> KERNEL_PHASE_DEG -> zeros -> eta.
KERNEL_PHASE = _parse_kernel_phase(os.environ.get("OSEM_KERNEL_PHASE_DEG", "").strip())
if KERNEL_PHASE is None:
    KERNEL_PHASE = KERNEL_PHASE_DEG


# ---------------------------------------------------------------------------
# THE ACTUATOR BUDGET -- one declared number, divided among claimants by NEED
# ---------------------------------------------------------------------------
# ONE NUMBER. Every cap in this file is quoted against it and nothing else here
# invents one. It is the demand about bias a single coil may carry, and it is
# theta's existing cap under a name:
#
#     BUDGET_V = MODAL_TOTAL_HEADROOM * BIAS_SWING = 0.900 * 0.250 = 0.225 V
#
# THE LEDGER, so the whole set is in one place:
#     BIAS_SWING             0.2500 V  the half-window (VMIN/VMAX). There is NO
#                                      recorded justification for it anywhere and
#                                      the coil driver is not in this repo -- do
#                                      not raise it here (CLAUDE.md Sec 11).
#     BUDGET_V               0.2250 V  90 % of it, on the TOTAL output, SOLVED by
#                                      `sl.cap_scale` rather than estimated.
#     MODAL_DEMAND_CAP_V     0.2000 V  a per-vector cap inside `Modal.allocate`,
#                                      i.e. 89 % of the budget, uniform so the
#                                      allocation direction is preserved.
#     BUDGET_MOTION_FLOOR_V  0.0089 V  the bottom of the ramp: the actuator's own
#                                      residual, 0.5 mV deadband plus the 10 ms
#                                      throttle (0.0089 / 0.0087 / 0.0094 V at
#                                      20 / 5 / 0 mV of injected sensor noise --
#                                      SIMULATOR numbers, CLAUDE.md). Motion under
#                                      it is not something any allocation reaches,
#                                      so the allocator holds there.
# MEASURED against that ledger and NOT respected before theta: coil 3 ran max
# 0.4229 V -- 169 % of the 0.250 V half-window -- because the derivative term was
# added after the modal cap (CLAUDE.md Sec 2).
BUDGET_V = MODAL_TOTAL_HEADROOM * BIAS_SWING

# Claimants: one per mode, plus the per-channel (hybrid diagonal) block as one.
BUDGET_CLAIMS = NMODE + 1

# HOW FAR THE DIVISION IS ALLOWED TO DEPART FROM FLAT. 0 = flat, i.e. theta.
# 1 = fully need-proportional.  ZERO IS WHAT SHIPS, AND IT IS NOT CAUTION.
# Nothing on this rig has ever measured the distribution of need across the three
# modes, and WHICH WAY the allocation should move at low disturbance is the rig
# owner's claim rather than a result: the per-mode residual data compares two laws
# at ONE ambient level (CLAUDE.md, per-mode residual) and nothing compares either
# law at two levels. At tilt = 0 every weight is EXACTLY 1.0 for every possible
# need vector -- nans and infinities included -- so this file is theta until
# somebody measures that, and the knob is here so the change is a number rather
# than a new control law written under bench pressure.
BUDGET_TILT = 0.0
BUDGET_TILT_HARD_CAP = 1.0     # past this a claimant's weight could go negative,
                               # which is not a reallocation, it is a sign flip

# THE RAMP TIME CONSTANT, and it is derived. The slowest mode is 0.72194 Hz, a
# 1.385 s period, and CLAUDE.md Sec 17 asks for "order 10 s or more" because
# anything faster modulates the loop gain inside the control band and is itself a
# disturbance source. 20 s is 14.4 periods of that mode and 2x that floor. The
# modulation corner 1/(2 pi tau) = 0.0080 Hz is 1.1 % of the slowest mode
# frequency, and at Q > 433 the mode half-width is 0.72194/(2 x 433) = 0.00083 Hz,
# so the sidebands a time-varying gain produces sit about 9.5 half-widths OFF
# resonance rather than on it. That is arithmetic on a measured Q, not a
# measurement of this loop.
BUDGET_TAU_S = 20.0
# Belt and braces on top of the lag: no weight may move faster than this. A unit
# change therefore takes at least BUDGET_TAU_S, and a weight moves at most
# 0.05 x 1.385 = 0.069 -- 6.9 % of nominal -- in one period of the slowest mode.
BUDGET_W_SLEW_PER_S = 1.0 / BUDGET_TAU_S
# Nobody is starved. Shares are mixed with the flat share before they become
# weights, so a claimant keeps at least this fraction of an equal share whatever
# the need says. A mode at zero gain is a mode back at the plant's own 0.0072 /s,
# tau > 138 s (analysis/ringdown.md), and no measurement says that is ever right.
# At tilt = 1 and 4 claimants this bounds the weights to [0.25, 3.25].
BUDGET_SHARE_FLOOR = 0.25
# The per-channel block may GIVE budget up and may not take any. Its gain is
# HYBRID_KP x SLOPE_SIGN x the MEASURED coherent fraction, which is the
# Wiener-optimal cap on a noisy channel (see HYBRID_COHERENT); scaling it past 1
# would spend budget on a number nothing justifies. The surplus goes back into the
# pot and is redivided among the modes, so the pot is still conserved.
BUDGET_W_DIAG_MAX = 1.0
# Below this the allocation cannot matter: it is the actuator's own floor, not the
# sensing floor. SIMULATOR number.
BUDGET_MOTION_FLOOR_V = 0.0089
# Held this long with no usable statistic and the run says so once. Held time is
# not a fault -- flat is a valid configuration, it is theta's -- but a run that
# spent most of itself held measured nothing about the allocator.
BUDGET_HOLD_SAY_S = 60.0


def _parse_budget_tilt(spec):
    """OSEM_BUDGET_TILT=x -> a float in [0, cap], or None.

    ATTENDED ONLY, same pattern as OSEM_KERNEL_PHASE_DEG and OSEM_GAIN_RAMP: the
    bench can reach it without a diff, and it cannot reach a value that would take
    a claimant's weight negative.
    """
    if not spec:
        return None
    tilt = float(spec)
    if not 0.0 <= tilt <= BUDGET_TILT_HARD_CAP:
        raise ValueError(
            "OSEM_BUDGET_TILT %r wants 0 <= tilt <= %.1f. Past the cap a claimant's "
            "weight can go negative, which is not a reallocation -- it is a sign "
            "flip, and a wrong-signed term PUMPS (CLAUDE.md, the sign of A)."
            % (spec, BUDGET_TILT_HARD_CAP))
    return tilt


class BudgetAllocator:
    """One declared budget, divided among claimants by NEED, ramped slowly.

        need_k   the power claimant k is REMOVING:  K qdot^2 per mode,
                 sum_j |g_j| vel_j^2 for the per-channel block
        share_k  need_k / sum(need), one-poled at BUDGET_TAU_S
        w_k      1 + tilt * (K share'_k - 1),  sum_k w_k = K  (the pot is conserved)

    AT tilt = 0 EVERY WEIGHT IS EXACTLY 1.0. Not approximately: the tilt term is
    forced finite before it is multiplied, so `1.0 + 0.0 * finite` is the identity
    for every need vector this can be handed, including all-zero, all-nan and one
    infinity. That is why the shipped file is theta bit for bit, and the selftest
    asserts it with `==` over pathological inputs rather than with a tolerance.

    WHY NEED IS A DISSIPATION RATE AND NOT AN AMPLITUDE -- the whole reason this
    rung is not dangerous. `ratio` is amplitude over a ZERO-GAIN baseline, so it
    measures the loop's own success: measured 2026-08-18 the same physical quiet
    reads 0.117 under the modal law and 1.449-1.832 under the diagonal one, 13x for
    the same plate. An allocator keyed on it hands budget to whoever is already
    winning, which is positive feedback through the measurement. A dissipation rate
    is the power leaving the mode, and in a steady state that equals the power the
    ROOM is injecting, which no gain of ours changes. The cancellation is not exact
    -- the plant takes its own 0.0072 /s share -- and the residual is bounded:
    across the full [0.25, 2.5] weight range the retained fraction runs 82.1 % to
    97.9 %, so the statistic moves at most 1.19x from the weight itself against
    `ratio`'s 13x. Simulated end to end in the selftest: 2.90x in rms motion
    against 1.24x in need, over a 10x range of loop gain.

    IT CANNOT LATCH. When the statistic is not available -- not DAMPING, the modal
    law not engaged, every gain at zero, need not finite, or motion at or under
    BUDGET_MOTION_FLOOR_V -- the shares are ramped BACK TOWARD FLAT at the same
    time constant instead of frozen where they were. Flat is theta's split, the only
    one with any measurement behind it, so a statistic that stops arriving returns
    the loop to the known configuration rather than pinning it wherever the last
    good sample left it. CLAUDE.md Sec 1 is the reason that is a requirement and not
    a nicety: `ratio` froze at 3.59 for 1085 s on 2026-08-17, and a ramp keyed on a
    statistic that can freeze is a ramp that can latch.

    DIVISION BY PROPORTION IS A CHOICE. The shape is the rig owner's Talmud
    reading -- one declared pot, division by need, nobody sitting on authority they
    are not using. The formula is not: the contested-garment rule divides a FIXED
    estate among STATIC known claims, and here the estate's usable size depends on
    what is already being demanded, the claims are noisy real-time estimates, and
    paying one claimant changes every other claim. Proportional-with-a-floor is
    justified only by reducing EXACTLY to the flat split at equal need and by being
    closed form inside a 10 ms step.
    """

    def __init__(self, k, tilt, tau_s, floor, w_diag_max, slew_per_s,
                 motion_floor_v):
        self.k = int(k)
        self.tilt = float(tilt)
        self.tau = float(tau_s)
        self.floor = float(floor)
        self.w_diag_max = float(w_diag_max)
        self.slew = float(slew_per_s)
        self.motion_floor = float(motion_floor_v)
        self.flat_share = np.full(self.k, 1.0 / self.k)
        self.share = self.flat_share.copy()
        self.w = np.ones(self.k)
        # The division BEFORE the slew limiter. sum(w_target) == k exactly; the
        # slew limiter is what makes the applied `w` lag it, so the pot is
        # conserved in the division and only transiently mis-served on the way
        # there -- bounded by the slew cap and asserted in the selftest.
        self.w_target = np.ones(self.k)
        self.held_s = self.live_s = 0.0
        self.hold_why = "not started"
        self.holding = True
        self.wsum, self.wn = np.zeros(self.k), 0
        self.wmin, self.wmax = np.ones(self.k), np.ones(self.k)
        self.spent_peak = 0.0          # realised |out - bias|, V, over driven coils
        self.spent_over = 0            # control steps at or past the budget

    # -- what the loop reads ------------------------------------------------
    @property
    def armed(self):
        return self.tilt != 0.0

    @property
    def flat(self):
        """True when every weight is EXACTLY 1.0 -- i.e. this IS theta's split."""
        return bool(np.all(self.w == 1.0))

    def weights(self):
        """(per-mode weights, per-channel-block weight)."""
        return self.w[:self.k - 1], float(self.w[self.k - 1])

    # -- one control step ---------------------------------------------------
    def update(self, dt, need, motion_v, usable, why=""):
        """Advance the allocation one control step. Never raises, never nans.

        `usable` is the caller's statement that the statistic means something this
        step. False does NOT freeze: it ramps back toward flat.
        """
        dt = float(dt)
        need = np.asarray(need, float)
        good = (bool(usable) and need.shape == (self.k,) and np.isfinite(need).all()
                and float(need.min()) >= 0.0 and float(need.sum()) > 0.0
                and np.isfinite(motion_v) and float(motion_v) > self.motion_floor)
        if good:
            target = need / float(need.sum())
            self.holding, self.hold_why = False, ""
            self.live_s += dt
        else:
            target = self.flat_share
            self.holding = True
            self.held_s += dt
            if not why:
                why = ("no usable need: motion %.4f V against the %.4f V actuator "
                       "floor" % (motion_v, self.motion_floor))
            self.hold_why = why
        # ONE POLE at BUDGET_TAU_S. This is the ramp; everything else is a bound.
        a = dt / (self.tau + dt) if self.tau > 0.0 else 1.0
        self.share = self.share + a * (target - self.share)
        self._weights(dt)
        self.wsum += self.w
        self.wn += 1
        self.wmin = np.minimum(self.wmin, self.w)
        self.wmax = np.maximum(self.wmax, self.w)
        return self.holding

    def _weights(self, dt):
        # Floor first, so no claimant can be starved and no weight can go negative.
        s = (1.0 - self.floor) * self.share + self.floor / self.k
        tilt_term = self.k * s - 1.0
        # FORCED FINITE BEFORE THE MULTIPLY. `0.0 * nan` is nan, and one nan in a
        # gain vector is the end of the run; `0.0 * finite` is exactly 0.0, which is
        # what makes tilt = 0 the identity rather than nearly it.
        tilt_term = np.where(np.isfinite(tilt_term), tilt_term, 0.0)
        w = 1.0 + self.tilt * tilt_term
        # The per-channel block gives and does not take (BUDGET_W_DIAG_MAX). The
        # surplus is redivided among the modes in proportion, so sum(w) = k stands.
        if w[self.k - 1] > self.w_diag_max:
            surplus = w[self.k - 1] - self.w_diag_max
            w[self.k - 1] = self.w_diag_max
            tot = float(w[:self.k - 1].sum())
            if tot > 0.0:
                w[:self.k - 1] = w[:self.k - 1] + surplus * w[:self.k - 1] / tot
        self.w_target = w
        cap = self.slew * dt
        self.w = self.w + np.clip(w - self.w, -cap, cap)

    def spend(self, demand_v):
        """Record what was actually spent. Observation only; changes nothing."""
        if demand_v.size:
            peak = float(np.abs(demand_v).max())
            self.spent_peak = max(self.spent_peak, peak)
            if peak >= BUDGET_V:
                self.spent_over += 1

    # -- reporting ----------------------------------------------------------
    def report(self):
        out = ["[budget] ONE declared budget %.4f V per coil about bias (%.0f %% of "
               "the %.3f V half-window), solved on the TOTAL output"
               % (BUDGET_V, 100.0 * MODAL_TOTAL_HEADROOM, BIAS_SWING),
               "[budget] %d claimants: %d modes + the per-channel block; division is "
               "need-proportional with a %.0f %% floor, one-poled at %.0f s"
               % (self.k, self.k - 1, 100.0 * self.floor, self.tau)]
        if not self.armed:
            out.append("[budget] tilt %.2f -- FLAT. Every weight is exactly 1.0 for "
                       "every possible need, so this run is %s's split, numerically."
                       % (self.tilt, LAW_SOURCE))
            out.append("[budget] the allocator is present, in the path on every "
                       "control step, and NOT ARMED: nothing on this rig has "
                       "measured the distribution of need across the modes, and "
                       "which way the ramp should point is a claim, not a result.")
            return out
        out.append("[budget] !! TILT IS NON-ZERO: %.2f. THIS IS NOT %s'S SPLIT. "
                   "Weights are bounded to [%.2f, %.2f], slew %.3f /s (a unit move "
                   "takes >= %.0f s), and nothing offline catches a wrong "
                   "allocation. ATTENDED ONLY."
                   % (self.tilt, LAW_SOURCE,
                      1.0 - self.tilt * (1.0 - self.floor),
                      1.0 + self.tilt * (self.k - 1) * (1.0 - self.floor),
                      self.slew, 1.0 / self.slew))
        out.append("[budget] need is the loop's own DISSIPATION RATE per claimant, "
                   "NOT `ratio`: `ratio` is referenced to a zero-gain baseline and "
                   "reads 13x apart for the same plate under the two laws "
                   "(2026-08-18). With no usable need the shares ramp BACK TO FLAT "
                   "rather than freezing, so this cannot latch.")
        return out

    def summary(self):
        if self.wn == 0:
            return "\n  BUDGET: no control step ran."
        mean = self.wsum / self.wn
        out = ["", "=" * 68, "  ACTUATOR BUDGET -- %.4f V per coil, divided by need"
               % BUDGET_V, "=" * 68,
               "  claimant      mean w    min w    max w",
               ]
        for m in range(self.k - 1):
            out.append("  mode %d %7.5f Hz  %6.3f   %6.3f   %6.3f"
                       % (m, F_MODE_HZ[m], mean[m], self.wmin[m], self.wmax[m]))
        j = self.k - 1
        out.append("  per-channel        %6.3f   %6.3f   %6.3f"
                   % (mean[j], self.wmin[j], self.wmax[j]))
        out += ["",
                "  allocation live %.1f s, held at flat %.1f s (%s)"
                % (self.live_s, self.held_s, self.hold_why or "-"),
                "  peak realised demand about bias %.4f V against the %.4f V budget"
                % (self.spent_peak, BUDGET_V)
                + ("" if self.spent_over == 0 else
                   "  <-- AT OR OVER on %d control step(s)" % self.spent_over)]
        if not self.armed:
            out.append("  tilt 0: this is %s's flat split and the table above is a "
                       "constant by construction." % LAW_SOURCE)
        out.append("")
        return "\n".join(out)


BUDGET_TILT_LIVE = _parse_budget_tilt(os.environ.get("OSEM_BUDGET_TILT", "").strip())
if BUDGET_TILT_LIVE is None:
    BUDGET_TILT_LIVE = BUDGET_TILT


# ---------------------------------------------------------------------------
# the stored floor
# ---------------------------------------------------------------------------
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0
BASELINE_PATH = os.path.join(HERE, "data", "baseline.json")
BASELINE_FILE_SCHEMA = 1
BASELINE_FILE_MAX_AGE_S = 1800.0
WIRE_RATE_TOL = 1.5
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


GAIN_RAMP_HARD_CAP = 0.060      # refuses to parse past this, whatever is asked
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
    # THE ALLOCATION IS LOGGED AND THE KERNEL IS NOT, and the difference is not
    # taste: the kernel is constant for a run and goes in the banner, while the
    # weights MOVE during a run and are the only record of what was actually
    # applied. "Always stream the raw data to disk" is a standing practice here,
    # and a ramp nobody can reconstruct afterwards measured nothing. Appended at
    # the END of the modal block, so every column theta and eta write keeps its
    # name and its meaning and analysis/ reads a run of any of the three the same
    # way. At the shipped tilt these five are constants: bw* = 1, bhold = 1.
    cols += [(f"bw{m}", ".5f") for m in range(NMODE)]
    cols += [("bwd", ".5f"), ("bhold", None)]
    for i in range(N):
        cols += [(f"ch{i}_counts", None), (f"ch{i}_V", ".4f")]
        cols += [(f"ch{i}_{k}", f) for k, f in _LOG]
        cols += [(f"ch{i}_{k}", None) for k in ("rail", "locked", "healthy")]
    return cols


CSV_COLS = csv_cols()
CSV_HEADER = ",".join(name for name, _ in CSV_COLS)

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
                 modal_path=None):
        # modal_path defaults to None so the suite is reproducible; only main()
        # passes the real path.
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
        # The kernel bank. At KERNEL_PHASE = 0 every g_m is the unit impulse and
        # this object returns its input unchanged, exactly -- it is still in the
        # path on every control step, so the machinery is exercised by every run
        # and by every check in the suite rather than only when it is armed.
        self.kernel = ModalKernel(F_MODE_HZ, KERNEL_PHASE, KERNEL_TAPS,
                                  CONTROL_PERIOD_S)
        # The budget allocator. At BUDGET_TILT_LIVE = 0 every weight it returns is
        # exactly 1.0, so the gain lines below multiply by 1.0 and this file is
        # theta -- and it is still called on every control step in every state, so
        # the machinery is exercised by every run and by every check in the suite
        # rather than only when it is armed. Same rule as the kernel bank.
        self.budget = BudgetAllocator(BUDGET_CLAIMS, BUDGET_TILT_LIVE, BUDGET_TAU_S,
                                      BUDGET_SHARE_FLOOR, BUDGET_W_DIAG_MAX,
                                      BUDGET_W_SLEW_PER_S, BUDGET_MOTION_FLOOR_V)
        self._budget_hold_said = False
        # PROOF BY REFUSAL. The capture/steady schedule is the older allocator and
        # it is keyed on `ratio`, the zero-gain-referenced statistic this rung
        # exists to stop using. It is inert only because its two endpoints are
        # equal. If anyone makes them differ, two allocators are live at once on
        # two different references and they will fight -- so arming the budget is
        # refused rather than allowed to be discovered on the bench.
        if self.budget.armed and not np.array_equal(self.steady, self.capture):
            raise ValueError(
                "OSEM_BUDGET_TILT is armed (%.3f) while CAPTURE_GAIN and "
                "STEADY_GAIN differ, so the capture/steady schedule is LIVE. That "
                "schedule reduces gain as `ratio` falls -- it withdraws authority "
                "as the loop succeeds, and `ratio` is referenced to a ZERO-GAIN "
                "baseline (13x apart between the two laws, 2026-08-18). Two "
                "allocators on two references fight. Retire one before arming the "
                "other.\n  steady  %s\n  capture %s"
                % (self.budget.tilt, np.round(self.steady, 5).tolist(),
                   np.round(self.capture, 5).tolist()))

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

        self.trim_steps = np.zeros(N, int)
        self.trim_frozen = np.zeros(N, bool)
        self.trim_last = self.trim_step_t = self.trim_pending = self.trim_ref = None
        self.trim_budget_said = False

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

    @property
    def trusted_baseline(self):
        return self.base.trusted

    @property
    def refusals(self):
        return self.base.refusals

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
        return self.baseline if q is None else np.maximum(self.baseline, q)

    # -- faults -------------------------------------------------------------
    def _fault(self, t, msg, railed=False, amp=False):
        self.say("!! " + msg)
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

        was = self.state
        if self.state == "CALIBRATING":
            self._calibrating(t, volts)
        elif self.state == "DAMPING":
            self._damping(t, dt)
        elif self.state == "FAULT":
            self._faulted(t)
        if was != "DAMPING":
            # NOT a freeze. Every gain is zero here, so every need is zero by
            # construction -- the same shape as the `ratio` that sat at 3.59 for
            # 1085 s (CLAUDE.md Sec 1). The allocator ramps BACK TO FLAT instead of
            # latching wherever the last good sample left it, and flat is the split
            # that has actually been on this rig.
            self.budget.update(dt, np.zeros(self.budget.k), 0.0, usable=False,
                               why="state is %s -- the gains are at zero, so the "
                                   "loop dissipates nothing and there is nothing to "
                                   "measure a need with" % was)
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
                 # "0.72 in log", which is 2.05x and reads as small. It is not:
                 # the breaker, the fault-clear gate and the lock detector all
                 # scale off this number, so a 2x error here is a simultaneous 2x
                 # error in all three. NOT TIGHTENED, and not overridden by the
                 # warm-up either -- the warm-up is 2.0s with the first 40 %
                 # dropped, about 1.2s of data, and the 20s calibration length
                 # exists precisely because a short window cannot be trusted
                 # (measured worst-channel skew from one 6 V/s kick: 74.3 % at 5s
                 # against 20.1 % at 20s, CLAUDE.md). The 1.2s number is the LESS
                 # reliable of the two, so it stays a check and not a replacement.
                 # Which of the two was right on 2026-08-18 is NOT ESTABLISHED.
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
        # sample. Their bias-corrected coherence with the optic is 0.46/0.26/0.50.
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
        # THE BUDGET, sampled before anything spends it. Need is the power each
        # claimant is REMOVING, not the motion it has left: `ratio` reads 13x apart
        # for the same plate under the two laws because its reference is a
        # zero-gain window, so an allocation keyed on it would feed back through
        # its own success. See BudgetAllocator.
        # `modal_on` and `diag_on` are last step's -- `_actuate` sets them after
        # this -- which is one control step of lag on a 20 s ramp.
        need = np.zeros(self.budget.k)
        need[:self.nmode] = self.mode_gain * self.qdot ** 2
        dmask = self.diag_on & live
        need[self.nmode] = (float(np.sum(np.abs(self.gain[dmask])
                                         * self.vel[dmask] ** 2))
                            if dmask.any() else 0.0)
        seen = live & self._driven()
        motion = (float(np.sqrt(np.mean(self.bp[seen] ** 2))) if seen.any() else 0.0)
        self.budget.update(
            dt, need, motion,
            usable=bool(self.modal_on and float(need[:self.nmode].sum()) > 0.0),
            why=("" if self.modal_on else
                 "the modal law is not engaged, so there is one term and nothing "
                 "to divide"))
        if (self.budget.armed and not self._budget_hold_said
                and self.budget.held_s >= BUDGET_HOLD_SAY_S):
            self._budget_hold_said = True
            self.say(f"[budget] held at the FLAT split for "
                     f"{self.budget.held_s:.0f}s -- {self.budget.hold_why}. Flat is "
                     f"{LAW_SOURCE}'s split, so this is safe; it is also the "
                     f"allocator measuring nothing.")
        w_mode, w_diag = self.budget.weights()

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
        # The per-channel block's share of the budget. At w_diag = 1.0 this is
        # HYBRID_GAIN unchanged, exactly.
        want = np.where(self._hyb_arm(), self.hybrid_gain * w_diag, want)
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)
        # Per mode, slewed like the diagonal gains so nothing steps.
        # PER MODE, weighted by that mode's share of the one budget, then slewed
        # like the diagonal gains so nothing steps. At w_mode = 1.0 this is
        # MODAL_KP * MODAL_GAIN_SCALE unchanged, exactly -- x * 1.0 is x.
        self.mode_gain = (self.mode_gain + np.clip(
            MODAL_KP * MODAL_GAIN_SCALE * w_mode - self.mode_gain,
            -GAIN_SLEW_PER_S * dt, GAIN_SLEW_PER_S * dt)
            if self.modal.ok else np.zeros(self.nmode))

        # The breaker watches everything the loop DRIVES, hybrid channels included:
        # a wrong sign there shows up as their own amplitude growing. A spurious
        # trip on their incoherent motion now costs one FAULT_CLEAR_SUSTAIN_S, not a
        # deadlock.
        ref = self._amp_ref()
        # `judge` is NOT narrowed to the lock quorum here, and that was tried.
        # Restricting the trip to coherent channels looks right -- a `ratio` on a
        # channel that is half not-the-optic is weak evidence -- but `_driven`'s
        # docstring already records the measurement that kills it: the channel
        # which crosses first in the pumped-resonance case is not the obvious one,
        # and narrowing the evidence set loses that detection outright (simulator,
        # peak ch2 ratio 2.58 against the 1.8 line, 0 faults). The breaker keeps
        # every driven channel as evidence.
        got = self.brk.sample(t, self.bp, self.healthy, self._driven(),
                              ref, self.sat)
        # Fed ONLY here, so the window holds closed-loop time and a FAULT freezes
        # it rather than filling it with the open-loop plant.
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

        # AGAINST THE ZERO-GAIN BASELINE, ON PURPOSE, and it is the one place in
        # this file where that is the right reference. LOCKED is the deliverable
        # (README 4) and its definition is a statement about the loop's EFFECT:
        # the motion is under LOCK_RMS_FACTOR of what it was with the gain off.
        # Re-pointing it at the running quiet would make it self-referential --
        # that quiet IS a low percentile of the loop's own envelope, so "near its
        # own floor" is true by construction about a fifth of the time whatever
        # the loop is doing. A measurement would become a tautology.
        # NOT A FALSE POSITIVE TODAY, measured: under modal the driven channels
        # read 1.98/1.52/1.07/1.30 at t=5 s and 0.26/0.12/0.37/0.33 at t=15 s, and
        # LOCKED was announced at 20.7 s against a settled floor of 0.117 -- the
        # 0.35 line sat 3x above the floor and the announced time tracked real
        # settling (data/20260818_002207_jerk_eta.log).
        # NOT ESTABLISHED: whether 0.35 stays a real bar if the loop improves
        # further. It is a fixed fraction of a fixed reference, so a loop that
        # holds 0.01 would clear it instantly and the lock TIME would stop meaning
        # anything. The LOCKED line therefore reports the margin against the
        # running quiet when one exists -- logged, acted on by nothing, which is
        # what this repo does with a number it has not yet measured a threshold for.
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
        self.bias[i] = new
        self.vmin[i], self.vmax[i] = new - BIAS_SWING, new + BIAS_SWING
        self.trim_step_t = t
        self.brk.hist.clear()
        self.excess_since[:] = np.inf
        self.health.sat_hist.clear()
        self.sat_sum[:] = 0.0

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
        step_all = -np.sign(err) * SLOPE_SIGN * BIAS_QUANTUM
        nxt = np.clip(self.bias + step_all, BIAS_MIN, BIAS_MAX)
        movable = np.abs(nxt - self.bias) > 1e-9
        affordable = (np.abs(self.bias - BIAS).sum() - np.abs(self.bias - BIAS)
                      + np.abs(nxt - BIAS)) <= TRIM_MAX_TOTAL_EXCURSION_V
        elig = (watch & ~self.trim_frozen & movable & affordable
                & (self.trim_steps < TRIM_MAX_STEPS)
                & (np.abs(err) > TRIM_DEADBAND_COUNTS))
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
        live = self.enabled & self.healthy & (self.state == "DAMPING")
        self.modal_on = False
        self.modal_u[:] = 0.0
        self.modal_cols[:] = False
        self.diag_on[:] = False

        if self.modal.ok and self.state == "DAMPING":
            # SENSE_OK is "this sensor's opinion counts"; DRIVE_OK is "this coil is
            # driven". They differ, and a5 -- live sensor, dead coil -- is the case
            # that proves one `live` flag cannot represent both.
            drive_ok = live & ~self.floor_bad
            enough = self.mode_seen >= MODAL_MIN_SENSORS
            qd, self.modal_rank = self.modal.project(self.qdot, drive_ok)
            # THE KERNEL. One control step of history per call, so it must be
            # called exactly once per step and only where `qd` is defined -- which
            # is here. At KERNEL_PHASE = 0 this returns `qd` itself, so the line
            # below is eta's `f = -mode_gain * qd` unchanged.
            f = np.where(enough, -self.mode_gain * self.kernel.push(qd), 0.0)
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
        else:
            # No modal step this control period -- refused data, or not DAMPING.
            # Drop the history rather than leave a gap in it: with a non-zero
            # phase the kernel reads a sample up to 1.4 s old, and a sample from
            # before a fault is not that sample. At zero phase only tap 0 is used
            # and this changes nothing.
            self.kernel.reset()

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
            # SOLVED, not estimated. The previous form measured the peak of the
            # total and then scaled the MODAL part by that ratio, which only
            # reaches the cap when the modal part is the whole demand. It is not:
            # the D term is added after, and it is what took coil 3 to 0.4229 V
            # against a 0.250 V half-window -- 169 % -- while the allocation stayed
            # inside its own 0.20 V cap for the entire run (CLAUDE.md 2). Once a
            # coil clips the realised force is no longer A_C^+ f and the
            # dissipation guarantee is void; the 5x was measured with that
            # happening. `cap_scale` returns the largest scale for which the TOTAL
            # is inside the cap, exactly.
            #
            # THE ALTERNATIVE IS TO RAISE THE WINDOW, AND IT IS NOT FREE: that is
            # CLAUDE.md 11. `VMIN, VMAX = 0.0, 0.5` and `BIAS = 0.25` have NO
            # recorded justification anywhere, the coil driver between the DAC and
            # the coil is not in this repo, and 0.25 V may encode a real current
            # limit. Capping the total costs authority; guessing the window costs
            # hardware.
            # THE DECLARED BUDGET. One number, quoted from one place.
            cap = BUDGET_V
            self.modal_scaled = sl.cap_scale(self.modal_u, self.pid.i + self.pid.d,
                                             cap, self.modal_cols)
            if self.modal_scaled < 1.0:
                self.modal_u = self.modal_u * self.modal_scaled
                self.pid.p = np.where(self.modal_cols, self.modal_u, self.pid.p)
                raw = self.bias + self.pid.p + self.pid.i + self.pid.d

        out = self.pid.drive(raw, self.bias, self.vmin, self.vmax, live, dt)
        # What was actually spent, against what was declared. Observation only --
        # it reads `out` after every decision is made and changes nothing.
        self.budget.spend((out - self.bias)[live] if live.any()
                          else np.zeros(0))
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
        if self.modal.ok:
            mode = ("MIMO" if self.modal_on else "diag") + (
                "" if self.kernel.identity else "+ker") + (
                "" if self.budget.flat else "+bud") + (
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
        row += list(self.budget.w[:self.nmode])
        row += [float(self.budget.w[self.budget.k - 1]),
                "1" if self.budget.holding else "0"]
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
    """Anything that changes what a baseline MEANS goes in here.

    KERNEL_PHASE and BUDGET_TILT_LIVE are deliberately NOT in it, and that is not
    an oversight: the baseline is measured with every gain at zero, so no control
    law -- kernel, allocation or neither -- can reach it. A floor written by eta is therefore valid for this file
    and the other way round, which is what the equivalence is worth in practice.
    """
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

    print("\n  === osem.iota.py selftest ===\n")

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
    check("...and ships FLAT, as measured modal rms A 0.64 / B 2.61 / C 3.15 says "
          "it should not stay",
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

    check("HYBRID_GAIN is HYBRID_KP x SLOPE_SIGN x the measured coherent fraction",
          np.allclose(HYBRID_GAIN, HYBRID_KP * SLOPE_SIGN * HYBRID_COHERENT)
          and abs(abs(HYBRID_GAIN[4]) / HYBRID_KP - 0.46) < 1e-12
          and abs(abs(HYBRID_GAIN[6]) / HYBRID_KP - 0.26) < 1e-12
          and abs(abs(HYBRID_GAIN[7]) / HYBRID_KP - 0.50) < 1e-12,
          "a4 %.5f a6 %.5f a7 %.5f against a full %.5f"
          % (HYBRID_GAIN[4], HYBRID_GAIN[6], HYBRID_GAIN[7], HYBRID_KP))
    check("a5 has coherent fraction 0 and can never actuate",
          HYBRID_COHERENT[5] == 0.0 and HYBRID_GAIN[5] == 0.0
          and not HYBRID_CHANNEL[5] and not ENABLE_CHANNEL[5])

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
    check("ENABLE_CHANNEL[5] is False -- a5 is a disconnected pin, 0.0 counts and "
          "variance exactly zero over 236387 samples",
          not ENABLE_CHANNEL[5] and not HYBRID_CHANNEL[5])

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

        def fn(t):
            c = base_c.copy()
            if t < 16.0:
                amp = 3.0                                     # quiet: calibrates
            elif t < 26.0:
                amp = 3.0 * np.exp(0.55 * (t - 16.0))         # grows: trips
            else:
                amp = (720.0 * np.exp(-0.7 * (t - 26.0)) if decay else 720.0)
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
    check("a DISABLED railed pin cannot veto the clear -- ch5 sat at 0.0 counts and "
          "held FAULT for 1085 s",
          bool(cd.rail[5]) and not ENABLE_CHANNEL[5] and cleared_d is not None,
          "ch5 railed %s, enabled %s" % (bool(cd.rail[5]), ENABLE_CHANNEL[5]))

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
        base_c = np.array([600.0, 620.0, 640.0, 660.0, 580.0, 0.0, 590.0, 570.0])
        c, dt = Controller(_NoDac()), 1.0 / wire_hz
        # The EXCURSION is what the breaker sees, and it is the measured quantity:
        # 0.117 -> 4.73 is 40.4x under the modal law, 1.240 -> 4.73 is 3.8x under
        # the diagonal. Stepping the amplitude at engage instead would model the
        # loop changing the motion instantly, which it does not, and the step
        # itself reads as sustained growth.
        a_cal, states, ex = 20.0, set(), peak_ratio / quiet_ratio
        for k in range(int(seconds * wire_hz)):
            t = k * dt
            if t < t_kick:
                amp = a_cal
            else:
                # Ring up over ONE PERIOD of the dominant mode -- what an impulse
                # does to a resonator -- then decay at the measured re-quiet rate,
                # with the beat ripple three modes produce
                # (0.99193 - 0.72194 = 0.26999 Hz).
                s, rise = t - t_kick, 1.0 / F_MODE_HZ[1]
                env = (min(1.0, s / rise) if s < rise
                       else np.exp(-decay * (s - rise)))
                amp = a_cal * (1.0 + (ex - 1.0) * env)
                amp *= 1.0 + 0.10 * np.sin(2 * np.pi * 0.26999 * s)
            cts, band = base_c.copy(), amp * np.sin(2.0 * np.pi * F_MODE_HZ[1] * t)
            cts[:4] += band + 1.0 * rg.normal(size=4)
            # The orthogonal axes see only their MEASURED coherent fraction of the
            # band (a4 0.46, a6 0.26, a7 0.50), on top of their own broadband
            # motion. Giving them the whole kick is what a4 does not do, and a
            # synthetic that does it trips the breaker on a channel the rig would
            # not have tripped.
            for j in (4, 6, 7):
                cts[j] += HYBRID_COHERENT[j] * band + 5.0 * rg.normal()
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
    # SUPPOSED to have; their bias-corrected coherence with the optic is
    # 0.46 / 0.26 / 0.50.
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
    dith_c = floor_run(2.0)       # dither only: over the pin line, far under a0-a3
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

    # ---- the kernel bank: the machinery, and that it is ZEROED ---------------
    print("\n  -- the convolution kernel bank --\n")
    dtc = CONTROL_PERIOD_S
    ker = ModalKernel(F_MODE_HZ, KERNEL_PHASE, KERNEL_TAPS, dtc)
    check("KERNEL_PHASE ships at zero -- the freedom exists and is not armed",
          bool(np.all(KERNEL_PHASE_DEG == 0.0)) and bool(np.all(KERNEL_PHASE == 0.0)),
          "%s deg" % KERNEL_PHASE.tolist())
    check("...so every g_m is EXACTLY the unit impulse, not approximately",
          bool(np.all(ker.g[:, 0] == 1.0)) and not bool(np.any(ker.g[:, 1:]))
          and ker.identity,
          "g[:,0] %s, |g[:,1:]| max %.1e"
          % (ker.g[:, 0].tolist(), float(np.abs(ker.g[:, 1:]).max())))
    rgk = np.random.default_rng(20260818)
    ident = True
    for _ in range(500):
        v = rgk.normal(size=NMODE) * 10.0 ** rgk.integers(-6, 3)
        ident = ident and bool(np.array_equal(ker.push(v), v))
    check("...and the kernel is the IDENTITY on the modal velocity, bit for bit",
          ident, "500 random vectors over 9 decades, `==` not `allclose`")
    per = 1.0 / float(F_MODE_HZ.min())
    check("KERNEL_TAPS holds one full period of the SLOWEST mode",
          (KERNEL_TAPS - 1) * dtc >= per and KERNEL_TAPS < per / dtc + 3,
          "%d taps = %.3f s against a %.3f s period at %.5f Hz"
          % (KERNEL_TAPS, KERNEL_LEN_S, per, F_MODE_HZ.min()))

    # The design is solved, not windowed, so it is exact rather than close.
    worst_mag, worst_ph = 0.0, 0.0
    for psi_deg in (-80.0, -45.0, -12.5, 0.0, 12.5, 45.0, 80.0):
        k2 = ModalKernel(F_MODE_HZ, np.full(NMODE, psi_deg), KERNEL_TAPS, dtc)
        for m in range(NMODE):
            h = k2.response(m)
            worst_mag = max(worst_mag, abs(abs(h) - 1.0))
            worst_ph = max(worst_ph, abs((np.degrees(np.angle(h)) + psi_deg + 180.0)
                                         % 360.0 - 180.0))
    check("a built kernel has unit magnitude and the REQUESTED phase at f_m",
          worst_mag < 1e-12 and worst_ph < 1e-9,
          "worst ||H|-1| %.1e, worst phase error %.1e deg" % (worst_mag, worst_ph))

    # The derivation the 90 deg cap rests on, measured on a tone.
    dis, want = [], []
    for psi_deg in (0.0, 30.0, 60.0, 85.0):
        k3 = ModalKernel(F_MODE_HZ, np.full(NMODE, psi_deg), KERNEL_TAPS, dtc)
        k3.reset()
        f0, cyc = float(F_MODE_HZ[0]), 12
        nstep = int(round(cyc / (f0 * dtc)))
        num = den = 0.0
        for kk in range(nstep + KERNEL_TAPS):
            qdot = np.zeros(NMODE)
            qdot[0] = np.cos(2.0 * np.pi * f0 * kk * dtc)
            fm = -k3.push(qdot)[0]                    # unit gain: -1 x kernel(qdot)
            if kk >= KERNEL_TAPS:                     # after the ring has filled
                num += fm * qdot[0]
                den += qdot[0] ** 2
        dis.append(num / den)
        want.append(-np.cos(np.radians(psi_deg)))
    check("cycle-averaged modal dissipation goes as -cos(psi): the cap's derivation",
          float(np.abs(np.array(dis) - np.array(want)).max()) < 5e-3,
          "psi 0/30/60/85 deg -> %s against -cos(psi) %s"
          % (" ".join("%+.4f" % d for d in dis), " ".join("%+.4f" % w for w in want)))
    try:
        _parse_kernel_phase("0,0,%.1f" % KERNEL_PHASE_HARD_CAP_DEG)
        capped = False
    except ValueError:
        capped = True
    check("a phase at or past the %.0f deg cap is REFUSED, not clipped"
          % KERNEL_PHASE_HARD_CAP_DEG,
          capped and _parse_kernel_phase("12") is not None
          and np.allclose(_parse_kernel_phase("12"), 12.0),
          "one value fills every mode; past the cap it raises")


    # ---- the actuator budget: the ledger, the division, and that it is FLAT --
    print("\n  -- the actuator budget --\n")
    dtb = CONTROL_PERIOD_S

    def alloc(tilt=BUDGET_TILT_LIVE):
        return BudgetAllocator(BUDGET_CLAIMS, tilt, BUDGET_TAU_S,
                               BUDGET_SHARE_FLOOR, BUDGET_W_DIAG_MAX,
                               BUDGET_W_SLEW_PER_S, BUDGET_MOTION_FLOOR_V)

    check("ONE declared budget, and it is theta's cap under a name",
          BUDGET_V == MODAL_TOTAL_HEADROOM * BIAS_SWING
          and BUDGET_V < BIAS_SWING and MODAL_DEMAND_CAP_V < BUDGET_V
          and BUDGET_MOTION_FLOOR_V < MODAL_DEMAND_CAP_V,
          "%.4f V, %.0f%% of the %.3f V half-window; allocator cap %.3f V is %.0f%% "
          "of it; floor %.4f V"
          % (BUDGET_V, 100 * BUDGET_V / BIAS_SWING, BIAS_SWING, MODAL_DEMAND_CAP_V,
             100 * MODAL_DEMAND_CAP_V / BUDGET_V, BUDGET_MOTION_FLOOR_V))
    check("the capture/steady schedule is INERT -- one live allocator, not two",
          SCHEDULE_INERT and bool(np.array_equal(STEADY_GAIN, CAPTURE_GAIN)),
          "endpoints equal, so steady + frac*(capture - steady) cannot move")

    # tilt = 0 is the identity for EVERY need vector, not merely for sensible ones.
    a0 = alloc(0.0)
    ident_w = True
    for need in ([0.0] * BUDGET_CLAIMS, [np.nan] * BUDGET_CLAIMS,
                 [np.inf] + [1.0] * (BUDGET_CLAIMS - 1),
                 [1e30] + [1e-30] * (BUDGET_CLAIMS - 1),
                 [-1.0] + [1.0] * (BUDGET_CLAIMS - 1),
                 list(np.random.default_rng(4).random(BUDGET_CLAIMS))):
        for _ in range(400):
            a0.update(dtb, np.array(need, float), 1.0, usable=True)
        ident_w = ident_w and bool(np.all(a0.w == 1.0)) and a0.flat
    check("BUDGET_TILT ships at zero -- the freedom exists and is not armed",
          BUDGET_TILT == 0.0 and BUDGET_TILT_LIVE == 0.0 and not a0.armed,
          "tilt %.3f" % BUDGET_TILT_LIVE)
    check("...so every weight is EXACTLY 1.0, for every possible need vector",
          ident_w,
          "zeros, nans, an infinity, 60 decades apart, a negative and a random "
          "vector, 2400 steps, `==` not `allclose`")

    # At equal need the division IS the flat split -- required by the equivalence
    # constraint, and the only property that justifies proportional division here.
    eq_exact, cons, lag, bounds = True, 0.0, 0.0, (np.inf, -np.inf)
    for tilt in (0.0, 0.25, 0.5, 1.0):
        ae = alloc(tilt)
        for _ in range(4000):
            ae.update(dtb, np.ones(BUDGET_CLAIMS), 1.0, usable=True)
        eq_exact = eq_exact and bool(np.all(ae.w == 1.0))
        askew = alloc(tilt)
        for _ in range(20000):
            askew.update(dtb, np.array([9.0, 1.0, 0.05] + [0.0] * (BUDGET_CLAIMS - 3)),
                         1.0, usable=True)
            cons = max(cons, abs(float(askew.w_target.sum()) - BUDGET_CLAIMS))
            lag = max(lag, abs(float(askew.w.sum()) - BUDGET_CLAIMS))
            bounds = (min(bounds[0], float(askew.w.min())),
                      max(bounds[1], float(askew.w.max())))
    check("at EQUAL need the division is the flat split, at any tilt",
          eq_exact, "tilt 0 / 0.25 / 0.5 / 1.0, all weights == 1.0 exactly")
    check("the pot is conserved: the division sums to the claimant count",
          cons < 1e-9,
          "worst |sum(w_target) - %d| = %.1e over 80000 steps at four tilts"
          % (BUDGET_CLAIMS, cons))
    # The slew limiter is the ONLY thing that breaks it, and only on the way to a
    # new division: it moves every weight at the same capped rate, so while a
    # reallocation is in flight the pot is transiently under- or over-served. That
    # is a bounded lag and not a leak -- it is zero in the steady state.
    check("...and the slew limiter's transient shortfall is bounded, not a leak",
          lag < BUDGET_CLAIMS * BUDGET_TILT_HARD_CAP * (1.0 - BUDGET_SHARE_FLOOR)
          and lag > 0.0,
          "worst |sum(w) - %d| = %.3f in flight, and 0 once the ramp has arrived"
          % (BUDGET_CLAIMS, lag))
    lo = 1.0 - BUDGET_TILT_HARD_CAP * (1.0 - BUDGET_SHARE_FLOOR)
    hi = 1.0 + BUDGET_TILT_HARD_CAP * (BUDGET_CLAIMS - 1) * (1.0 - BUDGET_SHARE_FLOOR)
    check("nobody is starved and nobody goes negative: the share floor bounds w",
          bounds[0] >= lo - 1e-12 and bounds[1] <= hi + 1e-12 and lo > 0.0,
          "w in [%.3f, %.3f] against the derived [%.2f, %.2f] at tilt 1"
          % (bounds[0], bounds[1], lo, hi))

    # The ramp is SLOW, and the bound is on the weight itself rather than on the
    # filter alone -- the filter sets the shape, the slew cap sets the worst case.
    asl = alloc(1.0)
    worst_step, t_half = 0.0, None
    hard = np.array([1.0] + [0.0] * (BUDGET_CLAIMS - 1))
    for kstep in range(int(120.0 / dtb)):
        prev = asl.w.copy()
        asl.update(dtb, hard, 1.0, usable=True)
        worst_step = max(worst_step, float(np.abs(asl.w - prev).max()))
        if t_half is None and asl.w[0] >= 1.0 + 0.5 * (hi - 1.0):
            t_half = kstep * dtb
    per_a = worst_step / dtb / float(F_MODE_HZ.min())
    check("the ramp is slow against the slowest mode -- %.0f s, %.5f Hz"
          % (BUDGET_TAU_S, F_MODE_HZ.min()),
          worst_step <= BUDGET_W_SLEW_PER_S * dtb + 1e-12
          and t_half is not None and t_half >= BUDGET_TAU_S
          and BUDGET_TAU_S >= 10.0,
          "<= %.4f of a weight per step, %.3f per %.3f s mode-A period; half the "
          "excursion took %.1f s" % (worst_step, per_a, 1.0 / F_MODE_HZ.min(),
                                     t_half if t_half is not None else float("nan")))

    # IT CANNOT LATCH. CLAUDE.md Sec 1: `ratio` froze at 3.59 for 1085 s. A ramp on
    # a statistic that can freeze is a ramp that can latch -- unless losing the
    # statistic drives it home rather than pinning it.
    ah = alloc(1.0)
    for _ in range(int(200.0 / dtb)):
        ah.update(dtb, hard, 1.0, usable=True)
    skewed = float(ah.w.max())
    for _ in range(int(200.0 / dtb)):
        ah.update(dtb, np.zeros(BUDGET_CLAIMS), 0.0, usable=False)
    check("losing the statistic ramps BACK TO FLAT -- it cannot latch",
          skewed > 1.5 and float(np.abs(ah.w - 1.0).max()) < 1e-3
          and ah.holding and ah.held_s >= 199.0,
          "w drifted to %.3f, then 200 s with no usable need returned it to %.5f"
          % (skewed, float(ah.w.max())))
    check("...and a motion at the actuator's own floor is not a need either",
          alloc(1.0).update(dtb, hard, BUDGET_MOTION_FLOOR_V * 0.5, usable=True),
          "under %.4f V nothing an allocation does can matter (SIMULATOR number)"
          % BUDGET_MOTION_FLOOR_V)

    # The per-channel block gives and does not take: its gain is already at a
    # measured Wiener cap.
    ad = alloc(1.0)
    for _ in range(int(300.0 / dtb)):
        ad.update(dtb, np.array([0.0] * (BUDGET_CLAIMS - 1) + [1.0]), 1.0,
                  usable=True)
    check("the per-channel block may GIVE budget up and may not take any",
          float(ad.w[BUDGET_CLAIMS - 1]) <= BUDGET_W_DIAG_MAX + 1e-12,
          "claiming the whole pot leaves it at w %.3f and the surplus went back to "
          "the modes (%s)" % (ad.w[BUDGET_CLAIMS - 1],
                              " ".join("%.3f" % x for x in ad.w[:BUDGET_CLAIMS - 1])))

    # ---- and the part that makes the statistic safe -------------------------
    # `ratio` is amplitude over a ZERO-GAIN baseline, so it reads the loop's own
    # success: 0.117 modal against 1.449-1.832 diagonal for the same plate, 13x
    # (2026-08-18). Need is a DISSIPATION RATE, and in a steady state that is the
    # power the ROOM injects. THIS IS A MODEL, not a measurement of this rig: one
    # mode, white force, the same forcing realisation at every gain.
    def onemode(gam_loop, gam0=0.0072, T=2000.0, dt=0.01, seed=7, scale=20.0):
        g0, gl = gam0 * scale, gam_loop * scale
        w0, c = 2.0 * np.pi * float(F_MODE_HZ[0]), 2.0 * (gam0 + gam_loop) * scale
        rng = np.random.default_rng(seed)
        nstep = int(T / dt)
        force = rng.normal(size=nstep) / np.sqrt(dt)
        q = v = sq = sv = 0.0
        m, burn = 0, int(nstep * 0.2)
        for kk in range(nstep):
            v += (-w0 * w0 * q - c * v + force[kk]) * dt
            q += v * dt
            if kk >= burn:
                sq += q * q
                sv += v * v
                m += 1
        return np.sqrt(sq / m), (2.0 * gl) * (sv / m)

    gl0 = 0.1393 - 0.0072            # the MEASURED modal decay less the plant's own
    (rq_lo, nd_lo) = onemode(gl0 * BUDGET_SHARE_FLOOR)
    (rq_hi, nd_hi) = onemode(gl0 * 2.5)
    mv_amp, mv_need = rq_lo / rq_hi, nd_hi / nd_lo
    check("need is a DISSIPATION rate, so the loop's own authority barely moves it",
          mv_amp > 2.3 and mv_need < 1.5,
          "over a 10x range of loop gain: rms motion moves %.2fx, need moves %.2fx "
          "(argued bound 1.19x from 0.0072 vs 0.1393 /s); `ratio` moves 13x between "
          "the two LAWS on hardware" % (mv_amp, mv_need))

    # Arming while the old schedule is live is REFUSED, not warned about.
    _saved_tilt = globals()["BUDGET_TILT_LIVE"]
    try:
        globals()["BUDGET_TILT_LIVE"] = 0.5
        try:
            Controller(_NoDac(), capture=(np.asarray(CAPTURE_GAIN) * 0.5).tolist())
            refused = False
        except ValueError:
            refused = True
        try:
            Controller(_NoDac())
            armed_ok = True
        except ValueError:
            armed_ok = False
    finally:
        globals()["BUDGET_TILT_LIVE"] = _saved_tilt
    check("arming the budget while capture/steady is LIVE is REFUSED",
          refused and armed_ok,
          "two allocators on two references would fight, and the older one is "
          "anchored to `ratio`")

    # ---- equivalence 1: against an explicit u = A_C+ (-K Proj qdot) ----------
    mref = _modal(None, allow=True)
    drv = np.zeros(N, bool)
    drv[A_DC_COILS] = True
    Kref = MODAL_KP * MODAL_GAIN_SCALE
    kref = ModalKernel(F_MODE_HZ, KERNEL_PHASE, KERNEL_TAPS, dtc)
    rgr = np.random.default_rng(902)
    duw = 0.0
    for _ in range(300):
        qdot = rgr.normal(size=NMODE)
        qd_r, _ = mref.project(qdot, drv)
        u_ref = mref.allocate(-Kref * qd_r, drv)[0]              # eta's law, written out
        u_ker = mref.allocate(-Kref * kref.push(qd_r), drv)[0]   # theta's path
        duw = max(duw, float(np.abs(u_ker - u_ref).max()))
    check("the kernel path IS u = A_C+ (-K Proj qdot), to the last bit",
          duw == 0.0 and mref.ok,
          "max |du| %.1e V over 300 allocations" % duw)

    # ---- equivalence 2: against osem.eta.py itself, on one sensor history ----
    def _load_law(fname):
        import importlib.util
        p = os.path.join(HERE, fname)
        if not os.path.isfile(p):
            return None, "%s is not next to this file" % fname
        try:
            spec = importlib.util.spec_from_file_location(
                fname[:-3].replace(".", "_") + "__ref", p)
            mod = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = mod
            spec.loader.exec_module(mod)
            return mod, ""
        except Exception as exc:                        # noqa: BLE001
            return None, "%s did not import: %r" % (fname, exc)

    twin = os.path.join(tempfile.gettempdir(), "iota_twin_modal.json")
    with open(twin, "w") as fh:
        json.dump(geodoc(PHI_GEOM.tolist()), fh)

    def twin_stream(seconds=75.0, wire_hz=400.0):
        """a0-a3 driven at all three modes in their geometric shapes, plus a rail.

        The rail is deliberate and it is what makes this worth running: it faults
        the rig during CALIBRATING, so the twin walks CALIBRATING -> FAULT ->
        CALIBRATING -> DAMPING and compares the two laws through the recovery as
        well as through the modal steady state. An equivalence that only ever saw
        DAMPING would not notice a divergence in the fault path, which is the part
        of eta being changed. It is railed AND MOVING -- std ~2.1 counts against
        the 1.30-count dead-pin line -- so it is a whole-rig fault and not a
        demotion.
        """
        rg = np.random.default_rng(20260818)
        dt = 1.0 / wire_hz
        rest = np.array([680.3, 585.4, 636.2, 699.6, 558.6, 0.0, 591.3, 566.0])
        for k in range(int(seconds * wire_hz)):
            t = k * dt
            c = rest.copy()
            for m, fm in enumerate(F_MODE_HZ):
                c[:4] += ((14.0 - 3.0 * m) * PHI_GEOM[:4, m]
                          * np.sin(2.0 * np.pi * fm * t + 0.4 * m))
            c[:4] += rg.normal(size=4)
            c[4] += 6.0 * np.sin(2.0 * np.pi * F_MODE_HZ[0] * t + 1.3) + rg.normal()
            c[6] += rg.normal()
            c[7] += rg.normal()
            if 5.0 <= t < 8.0:
                c[0] = 1018.0 + 3.0 * np.sin(2.0 * np.pi * 1.3 * t)   # over RAIL_HIGH
            yield c, c * (A_VCC / ADC_MAX_COUNTS), t, dt

    ref_mod, why_ref = _load_law(LAW_SOURCE)
    if ref_mod is None:
        check("%s is here to be compared against" % LAW_SOURCE, False, why_ref)
    else:
        ct = Controller(_NoDac(), modal_path=twin)
        ce = ref_mod.Controller(_NoDac(), modal_path=twin)
        dv, nstep, nmodal, states = 0.0, 0, 0, set()
        lockstep = True
        for c, v, tt, dd in twin_stream():
            ct.step(c, v, tt, dd)
            ce.step(c, v, tt, dd)
            ct.drain_events()
            ce.drain_events()
            dv = max(dv, float(np.abs(ct.out - ce.out).max()))
            lockstep = lockstep and (ct.state == ce.state
                                     and ct.modal_on == ce.modal_on
                                     and ct.stepped == ce.stepped)
            nstep += 1
            nmodal += int(ct.modal_on)
            states.add(ct.state)
        check("the twin exercised the modal law AND a fault -- else it proves nothing",
              nmodal > 0 and states >= {"CALIBRATING", "DAMPING", "FAULT"}
              and ct.modal.ok,
              "%d of %d samples with the modal law engaged, states %s"
              % (nmodal, nstep, sorted(states)))
        check("iota and %s command IDENTICAL voltages on one sensor history"
              % LAW_SOURCE, dv == 0.0 and nmodal > 0,
              "max |du| %.3e V over %d samples on 8 channels" % (dv, nstep)
              + ("" if dv == 0.0 else
                 "  <-- THE LAWS HAVE DIVERGED. If %s was fixed, PORT THE FIX "
                 "FORWARD; do not relax this check." % LAW_SOURCE))
        check("...and they agree on state and on modal ON/OFF at every sample",
              lockstep,
              "final state %s/%s, modal %s/%s over %d samples"
              % (ct.state, ce.state, ct.modal_on, ce.modal_on, nstep))
        check("...and the allocation stayed EXACTLY flat for all of it",
              ct.budget.flat and bool(np.all(ct.budget.wmin == 1.0))
              and bool(np.all(ct.budget.wmax == 1.0)),
              "min/max weight over %d control steps: %s / %s"
              % (ct.budget.wn, ct.budget.wmin.tolist(), ct.budget.wmax.tolist()))
        # osem.eta.py is INFORMATIONAL here and not a check, deliberately. theta's
        # own selftest asserts theta == eta, so iota == theta carries it; and eta
        # is the file with fixes in flight, so a failure there is not this file's.
        eta_mod, why_eta = _load_law("osem.eta.py")
        if eta_mod is None:
            print("   %-52s %s   %s" % ("note: osem.eta.py not compared", "----",
                                        why_eta))
        else:
            ci = Controller(_NoDac(), modal_path=twin)
            cn = eta_mod.Controller(_NoDac(), modal_path=twin)
            de = 0.0
            for c, v, tt, dd in twin_stream():
                ci.step(c, v, tt, dd)
                cn.step(c, v, tt, dd)
                ci.drain_events()
                cn.drain_events()
                de = max(de, float(np.abs(ci.out - cn.out).max()))
            print("   %-52s %s   max |du| %.3e V%s"
                  % ("note: against osem.eta.py, the far end of the ladder",
                     "same" if de == 0.0 else "DIFF", de,
                     "" if de == 0.0 else "  <-- eta has moved; theta's own "
                     "selftest is where that is owned, not this one."))

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
        "  AND THE ACTUATOR BUDGET IS ONE DECLARED NUMBER, 0.225 V per coil about\n"
        "  bias, divided among four claimants -- three modes and the per-channel\n"
        "  block -- in proportion to each one's own DISSIPATION RATE, one-poled at\n"
        "  20 s and slew-capped so a weight moves under 7 % of nominal in a period of\n"
        "  the slowest mode. Need is not `ratio`: `ratio` is referenced to a zero-gain\n"
        "  baseline and reads 13x apart for the same plate under the two laws, while a\n"
        "  dissipation rate is the power the ROOM injects and moves 1.2x over a 10x\n"
        "  range of loop gain. Losing the statistic ramps the split BACK TO FLAT\n"
        "  instead of latching, the pot is conserved, no claimant is starved, and the\n"
        "  older capture/steady schedule is proved inert rather than left to fight it.\n"
        "  IT IS NOT ARMED: BUDGET_TILT is 0, at which every weight is exactly 1.0 for\n"
        "  every possible need vector, so this file is osem.theta.py numerically.\n"
        "  AND THE KERNEL BANK IS PRESENT, EXERCISED AND ZEROED: every g_m is exactly\n"
        "  the unit impulse, the kernel is the identity on the modal velocity bit for\n"
        "  bit, the path IS u = A_C+ (-K Proj qdot) to the last bit, and driven with\n"
        "  the same synthetic sensor history this file and osem.theta.py command the\n"
        "  SAME VOLTAGES on all eight channels at every sample. The freedom is real --\n"
        "  a built kernel hits the requested phase exactly at f_m and its dissipation\n"
        "  follows -cos(psi) -- and it is not armed.\n"
        "  NOT tested here, and only the bench can: whether it damps the optic, what\n"
        "  chi2/dof distributes as on this rig, which sign pattern A needs, and\n"
        "  whether the hybrid improves the measured DECAY RATE -- jerk.py is that\n"
        "  test, and `ratio` is not. NOR IS ANY NON-ZERO TILT tested anywhere: the\n"
        "  checks above prove the allocator divides what it is given and cannot latch,\n"
        "  NOT that reallocating helps. Whether a time-varying blend of two dissipative\n"
        "  laws is dissipative through the ESTIMATOR is OPEN, and which way the ramp\n"
        "  should point at low disturbance is the rig owner's claim, not a result.\n"
        "  NOR IS ANY NON-ZERO PHASE tested anywhere: the\n"
        "  checks above prove the kernel realises the phase it is asked for, not that\n"
        "  any phase is the right one. That needs A's magnitudes (CLAUDE.md Sec 3)."
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

    pre = _modal(MODAL_PATH)
    for line in pre.report(MODAL_KP, MODAL_GAIN_SCALE):
        print(line)
    # Before any coil is energised, the same rule the [modal] banner follows: the
    # kernel is constant for a run and is not in the CSV, so this line is the only
    # record of which law actually ran.
    for line in ModalKernel(F_MODE_HZ, KERNEL_PHASE, KERNEL_TAPS,
                            CONTROL_PERIOD_S).report():
        print(line)
    # Same rule: the budget is declared before any coil is energised, so the log
    # records which allocation actually ran.
    for line in BudgetAllocator(BUDGET_CLAIMS, BUDGET_TILT_LIVE, BUDGET_TAU_S,
                                BUDGET_SHARE_FLOOR, BUDGET_W_DIAG_MAX,
                                BUDGET_W_SLEW_PER_S,
                                BUDGET_MOTION_FLOOR_V).report():
        print(line)
    if not SCHEDULE_INERT:
        print("[budget] !! CAPTURE_GAIN and STEADY_GAIN DIFFER, so the old "
              "capture/steady schedule is LIVE alongside this allocator and they "
              "are keyed on different references. Arming the budget is refused "
              "while that is true.")
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

    dac = sl.open_dac(PORT)
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
    if GAIN_RAMP:
        lv, dw = GAIN_RAMP
        print(f"[gain-ramp] {len(lv)} levels, |Kp| {lv[0]:.4f} -> {lv[-1]:.4f}, "
              f"{dw:.0f}s each, about {len(lv) * dw / 60:.0f} min of DAMPING.\n"
              f"[gain-ramp] ATTENDED ONLY. This walks the loop toward instability "
              f"deliberately. Keep a scope on the coil drive and be ready to "
              f"Ctrl+C.\n")
    try:
        while True:
            sample = ctl.guard.read(dac.ser)
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
        # raise there skipped the park entirely and left every coil at whatever
        # the loop last commanded. That is CLAUDE.md 15's outcome reached without
        # a signal at all -- a plain Ctrl+C could do it. The coils were parked by
        # hand three times on 2026-08-17/18. `stop_stream` does not merely raise on
        # its deadline either: measured 2026-08-18, a controller survived 90 s of
        # SIGINT delivered to both the process and its process group while blocked
        # in it waiting for an ACK a busy board never sent, holding the port
        # throughout.
        # Parking needs the port OPEN, so it has to come before stop_stream and
        # close: `Actuator.send` writes the same `SET` commands mid-stream on every
        # control step of every run, so this is the proven path, not a new one.
        # EACH STEP IN ITS OWN try/except, not one try around all of them -- a
        # single wrapper has the identical defect, because the first raise still
        # skips everything after it.
        # The last-resort fallback, and it is NOT an excuse for a broken teardown:
        # arduino.ino:363 zeroes all eight coils in setup(), and setup() runs when
        # the port is opened, so whoever opens it next finds them at 0 V.
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
        print(ctl.budget.summary())
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
