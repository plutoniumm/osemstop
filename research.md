# Research queue

Ranked by impact × confidence × cost, **in dependency order** — each item gated by the ones
above. Item 1 is **done**; nothing below it is started. Every number came from `harness.py`.


## The measurement everything else follows from

The amplitude ratio settles near **0.23, not zero**: the steady state of a *continuously
forced* oscillator, not a failure to converge. Drive removed mid-run:

| | damped (Kp −0.030) | gain off |
|---|---|---|
| decay time constant | **3.4 s** | 15.9 s |
| predicted | 2.76 s | 15.92 s |

Undamped matches theory to 0.01 s — model and loop both sound. A gain sweep confirms the
floor tracks `intrinsic / (intrinsic + added)` damping:

| Kp | predicted floor | measured |
|---|---|---|
| −0.005 | 0.557 | 0.545 |
| −0.030 | 0.173 | 0.239 |
| −0.100 | 0.059 | 0.088 |
| −0.300 | 0.021 | 0.031 |

So the floor falls with gain, and gain is capped by an instability at −0.04 that **has no
recorded evidence anywhere**. Hence item 3 gates items 4 and 5.

Negative result: the 0.0089 V residual after the drive stops is **not** sensor-noise
injection — 0.0089 / 0.0087 / 0.0094 at 20 / 5 / 0 mV of sensor noise. It is
`RateLimitedActuator`'s 0.5 mV deadband and 10 ms throttle: firmware, not sensing.


## 1. Fix the suite for the rigid-body plant — **DONE**

Was 13 failures, 7 of them *stale assertions* encoding the old four-independent-oscillator
plant; nothing else could be validated while the test bed lied. Now 88 checks, **83 pass,
5 fail, ~120 s**; v0–v3 green (18 / 16 / 18 / 18). All 5 failures were the since-deleted
modal v4, all real: no damping (ratios 0.24–0.77), never locks, one fault in a quiet lab,
I term never accumulates so P-dominance has nothing to compare — item 2. Three assertions
rewritten in `harness.py`; no controller touched.

| Stale assertion | Replaced by |
|---|---|
| *disabled channels stay at their baseline* | Split: a disabled channel never actuates (peak `\|out − bias\|` = 0), but its ratio must **fall** — one rigid body, so damping ch0 drains the whole mass (`provenance.md` §3). v0: driven ch0 0.197, undriven 0.265 / 0.323 / 0.355, driven still flattest. |
| *a runaway trip auto-recovers* | Four on with ch2 inverted no longer runs away: the other three out-damp it (net `gain_i · COIL_GAIN_i` −0.1415 → −0.0419, a third of the damping surviving), so the optic sits near full amplitude asking more than its ±0.25 V clip and trips on **ch3 saturation**, t = 10.74 s, `sat_streak` = 31 — defect #1, which latches. Asserted broken-side for v0/v1/v2, fixed-side for v3. Runaway gets its own scenario (ch2 alone, wrong-signed at half magnitude: slow enough to trip on ratio, actuator peaking 0.055–0.112 V of its 0.25 V clip) that every version must recover from — for un-fixed ones that recovery *is* proof nothing saturated. |
| *KNOWN GAP / FIXED runaway-baseline* | The 18% line was calibrated on the old plant and separates nothing. Swept one shock across the calibration window: worst case 45.3% (v0/v1/v2) vs 25.0% (v3), line now 35% (`versions.md` § v3). One fixed shock time ranks them backwards for late shocks, so the check sweeps. |

All simulator, rigid-body plant, **never hardware**.


## 2. Modal (MIMO) damping — what killed the last attempt

The main **beyond-PID** direction, tried once — by the *original* `osem.v4.py`, the modal
controller **deleted 2026-08-03**, in git history only (today's `osem.v4.py` is a different
file: v3 plus `auto-disable`). It never damped. It learned the modal basis with no geometry
input ("2 of 4 modes carry real motion, 98.8% / 1.2%", from the calibration covariance) and
failed at one point: recovering the actuation matrix. Truth (LONG row, normalised)
`[−0.59, −0.45, +2.45, −0.51]` — ch2 4× the others, opposite sign.

| scheme | result |
|---|---|
| velocity × PRBS (what it did) | `[2.0, 0.2, −0.53, −1.27]` |
| acceleration × PRBS | `[−0.71, −0.47, −0.86, −1.96]` |

Three compounding causes:

1. **Wrong quadrature — the fundamental one.** Force acts instantaneously on *acceleration*;
   velocity is its integral, so a zero-lag correlation on velocity has no coherent term to
   find. (The earlier displacement → velocity fix was right *for a sinusoid at resonance*,
   and broadband PRBS silently invalidated it.)
2. **Simultaneous excitation, sequences too short.** ~40 bits gives ~16% cross-talk between
   channels, comparable to the differences measured; the four LFSR seeds are phase shifts of
   one m-sequence.
3. **SNR.** Dither force ≈ 1.0 against ambient drive 0.8 and seismic 0.8.

**Fix.** Fit a *state-space model* to the dither data rather than correlating at one lag —
lag structure is what states are for; `scipy` / `python-control` are mature. Or the bench
standard, **stepped sine, one coil at a time, both quadratures**: magnitude and phase both
fall out, so the sign never depends on guessing a quadrature. It need not be online at all —
on the bench that is one afternoon's measurement with no convergence risk, and it is what
the online system ID kept getting wrong.

**Done when:** the recovered matrix matches ground truth in sign and to ~20% in magnitude,
and the modal controller damps on the bench.


## 3. Characterise the −0.04 instability on the bench

**The only item that needs hardware, and it gates 4 and 5.**

Every gain in the repo rests on one sentence in `osem.v0.py`'s docstring: *"onset of
instability/rail at gain=-0.04."* No data records it; `provenance.md`, two days later, never
mentions it — no sweep, no −0.04, no rail-onset trace. The simulator provably cannot
reproduce it, the plant being linear: more negative Kp is monotonically more damping to
−0.6. Items 4 and 5 both raise effective gain, so pushing on an undocumented stability limit
first is the wrong order.

**How:** re-run the sweep on the bench; keep the 12-column CSV *and* the scope trace; record
where clipping starts versus where oscillation grows — different failures. For the mechanism
this is the one place **nonlinear system ID** (NARX, or a small network) beats a physics
model: something nonlinear the linear model cannot express — actuator saturation, magnetic
nonlinearity, or the OSEM shadow response leaving its linear range.

**Done when:** a logged run in `data/` shows the onset, with a stated mechanism.


## 4. Resonant / internal-model control

**The biggest single win on the actual objective.** The floor is set by a *persistent
narrowband* disturbance near 1 Hz; the internal model principle says asymptotic rejection of
a sinusoid needs a loop pole there. A lightly-damped complex pole pair at f₀ — a "boost"
stage, standard on real suspensions — puts enormous loop gain where the disturbance lives
and almost none elsewhere: the one way at the 0.23 floor **without** the broadband gain
increase that runs into −0.04.

**Watch for:** added phase, so it interacts with item 5, and it needs f₀. No frequency
estimator exists — `provenance.md`'s block diagram specifies `zero-crossing → f̂` in
Calibration, never implemented. This item is the reason to build it.


## 5. Kalman velocity estimator

Today: finite difference plus a 5 Hz lowpass — pure phase lag at 1 Hz, and phase lag
consumes stability margin. A Kalman filter on the known second-order model is the optimal
estimator for noisy position in, velocity out. Payoffs: gain headroom for item 4, and the
**leading hypothesis for item 3** — why −0.04 destabilises on hardware when the linear model
says it should not.


## 6. Stop paying 20 s of zero gain for a baseline

**Cheap, self-contained, does not wait on the bench.** All measured in the simulator on
2026-08-04.

### What the window buys, and what it costs

`CALIBRATION_S` holds gain at **zero** while measuring `baseline_rms`, the undamped noise
floor. Three things scale off it: the runaway breaker (`> 1.8x baseline`, sustained), the
gain scheduler (`ratio = local_rms / baseline`), the lock detector (`< 0.35x baseline`).
Cost: 20 s of no damping per entry, and `FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S` = 25 s after
a fault. v5's `fast-refault` already elides the *repeat* case; this item is the rest.

### Why it is 20 s, which is not arbitrary

In a quiet lab almost any window works — 3 s and 20 s give the same answer, 0.8%
run-to-run scatter over 8 noise seeds. The length exists for the `runaway-baseline` defect:
one shock inside the window. Worst-channel skew from a single 6 V/s kick:

| CALIBRATION_S | sub-window | worst skew |
|---|---|---|
| 5 s | 1.0 s | 74.3% |
| 8 s | 1.6 s | 86.5% |
| 10 s | 2.0 s | 41.0% |
| 15 s | 3.0 s | 57.9% |
| **20 s** | **4.0 s** | **20.1%** |

The harness asserts < 35% and v0 un-fixed reaches 39.3%, so every shorter window fails, some
worse than the defect being fixed.

Cause: **ringdown**. At Q = 50, f0 ~ 1 Hz, tau = Q/(pi*f0) ~ **16 s**. Median-of-sub-windows
helps only when *some* sub-windows are clean, i.e. window comparable to tau; at 10 s the
whole window is contaminated and the median picks the middle of a uniformly bad set. Hence
the non-monotonicity too (12 s worse than 10 s): what dominates is where the shock falls
relative to sub-window boundaries, not a smooth statistical effect.

**Derive the ceiling from Q, do not hardcode it.** If the real Q is not 50 the window scales
with it, and ringdown falls out of the item-2 sweep for free.

### 6a. Adaptive window — do this first

Accumulate sub-windows, stop when the last few agree to a tolerance, 20 s a **ceiling not a
target**. Quiet lab: exits in seconds — the 0.8% scatter says the information is already
there. After a shock: keeps going until the ringdown passes. Strictly better than either
constant, at the cost of one convergence test.

**Done when:** it exits early in a quiet lab AND still bounds the shock skew under 35% on
the swept-shock check.

### 6b. Always-on baseline — the right end state, with one trap

Never stop measuring; just *do not use* a fresh estimate unless something needs it.

**The trap** (the naive version is worse than useless): `baseline_rms` is the **undamped**
floor, but integrating the live bandpassed signal closed-loop measures the **damped** level
— ~0.03x of it at shipped gains. That leaves the lock detector demanding 0.35x of an
already-damped signal and the runaway breaker tripping on ordinary motion, so it cannot just
be the existing accumulator left running. Three that work:

| | How | Catch |
|---|---|---|
| **(a) Invert the known ratio** | floor tracks `gamma_int / (gamma_int + gamma_added)` (top of file), so open-loop ~ closed-loop / floor(Kp) | predicted vs measured is 0.173 vs 0.239 at Kp = -0.030, ~40% error — good enough to *seed*, not to be the reference |
| **(b) A witness channel, held open loop** — **the one to build** | one rigid body, so a channel parked at bias measures the undamped floor continuously and free | costs that channel's damping authority; revisit when `v5.5` brings eight channels and four DOF online — eight sensors for four DOF is real redundancy, and the witness can be rotated so no axis stays unactuated |
| **(c) Out-of-band estimation** | the loop only has authority in 0.4-3 Hz; outside it the plant is open loop at any gain, so if the disturbance is spectrally stationary, track the out-of-band level and scale | needs the scaling characterised once, then free forever and costs no channel |

**Dependency:** 6a is independent, do it now. 6b(b) is gated on the 8-channel bring-up;
6b(c) is gated on nothing but wants item 2's plant model to justify the scaling.


## Deliberately cut, with reasons

| Candidate | Why not |
|---|---|
| **Actuator deadband / rate-limit fix** | Sets the current floor (0.009 V above; identical at 0 / 5 / 20 mV sensor noise, so not the sensor). Cheap, firmware-only — but it only matters near 0.009 and you are at 0.23. Premature. |
| **Safe Bayesian optimisation for gain tuning** | Good fit: expensive trials, few parameters, a hard safety constraint (SafeOpt-style methods explore without violating one). The right tool *after* item 3 says where the constraint is. |
| **CUSUM / change-point interlocks** | Principled fix for both threshold defects — #2 "3 counts, 250 consecutive samples", #3 "2 s RMS > 1.8 × an 8 s baseline" — which test an instantaneous threshold instead of accumulating evidence. v3's are better *thresholds*; this is the better *kind of thing*. Not urgent. |
| **Feedforward / Wiener filtering, LQG / H∞, adaptive notch (LMS), ICA** | Wait on a validated plant model = item 2. Wiener feedforward from a seismometer is the detector standard, and strongest of these once a witness sensor exists. |
| **Reinforcement learning for the damping law** | Sample-hungry on hardware, no stability guarantee, hard safety constraint. Sim-to-real is the usual escape, but this simulator's one documented blind spot is the −0.04 instability — an agent trained on it would learn that more gain is always better, the lesson you least want it to learn. |

**`ENABLE_CHANNEL` vs the report, and `auto-disable`.** Only the first is written down —
`versions.md` § v1 and `README.md` § "Channel mapping": v0's "only ch0 is validated"
predates `provenance.md` §4 by two days, and v1–v3 ship all four channels on.
`auto-disable` appears once in the report's block-diagram Safety Checks box next to `lock
detect`, and in no controller (`grep auto-disable osem.v*.py` is empty) — drawn and never
implemented, like `zero-crossing → f̂` (item 4). Correction: this entry once listed "the
report's 0.085 Hz error" and credited all three to `versions.md`; both wrong. `versions.md`
contains neither `0.085` nor `auto-disable`, and `0.085` occurs nowhere in the repo or in
`provenance.md`'s text layer — that error has left no record, so re-derive it from the report
before acting. No action needed until wanted.

## Where learning belongs

Learn the **model**, the **estimator**, and the **anomaly detector**; keep the control law
and the interlocks classical and certifiable. That split is what real detectors do, and it
is what lets you argue the machine cannot destroy the optic — the argument that matters when
the other end is a suspended mirror.
