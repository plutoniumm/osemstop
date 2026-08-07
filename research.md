# Research queue

Dependency order — each item gated by the ones above. Simulator numbers (`harness.py`) unless
marked bench. In the tree: `osem.v3.py`, `osem.v6.py` (sysid), `osem.v7.py`, `osem.v9.py`,
`osem.v10.py`, `osem.v11.py`; v7, v9 and v10 confirmed on hardware. `v0`–`v2`, both files called
`v4`, `v5`, `v5.5`, `v6.1`, `v6.5`, `v6.6`, `v8` are git history only; their numbers stand.

**Active as of 2026-08-06** — everything else here is background:

| | item | why now |
|---|---|---|
| **settle faster** | 6a — retune `CALIB_AGREE_TOL` | built, fails on one constant, fixable offline |
| **settle faster** | 6b(c) — out-of-band baseline | kills the calibration window entirely, not just shortens it |
| **closer to zero** | 4 — resonant / internal-model control | **unblocked**: f₀ measured 2026-08-06 at 1.046 and 1.657 Hz |
| **closer to zero** | 3 — is −0.04 real? | the floor is gain-limited, and the cap has never been reached — `analysis/kp040.md` |
| both | 5 — Kalman velocity estimator | v11 measured the finite-difference noise directly: 3.5 → 12.8 vel RMS at 49× loop rate |

Item 2 is blocked on a clean actuation matrix; leave it until the above move.

## The floor

Amplitude ratio settles at **0.23** — steady state of a continuously forced oscillator, not a
failure to converge. Drive removed mid-run: decay τ **3.4 s** damped (Kp −0.030) against **15.9 s**
gain-off, predicted 2.76 / 15.92 s. The floor tracks `intrinsic / (intrinsic + added)` damping:

| Kp | −0.005 | −0.030 | −0.100 | −0.300 |
|---|---|---|---|---|
| predicted floor | 0.557 | 0.173 | 0.059 | 0.021 |
| measured | 0.545 | 0.239 | 0.088 | 0.031 |

**The floor falls with gain; gain is capped by the −0.04 instability — item 3.** The 0.0089 V
residual is the actuator's 0.5 mV deadband and 10 ms throttle, not sensing: 0.0089 / 0.0087 /
0.0094 V at 20 / 5 / 0 mV injected sensor noise.

## 2. Modal (MIMO) damping — blocked on a clean actuation matrix

Acceptance target — ground truth (LONG row, normalised) `[−0.59, −0.45, +2.45, −0.51]`: ch2 4× the
others, opposite sign. The online attempt (git history) recovered `[2.0, 0.2, −0.53, −1.27]` from
velocity × PRBS and `[−0.71, −0.47, −0.86, −1.96]` from acceleration × PRBS; its mode estimator was
fine (98.8% / 1.2% mode content from the calibration covariance). Still binding: dither 1.0 barely
beat ambient drive 0.8 + seismic 0.8, so size any new dither above that. Superseded: zero-lag
correlation on velocity has no coherent term (force acts on **acceleration**) and ~40-bit PRBS
gives **~16%** inter-channel cross-talk, the size of the differences measured. Stepped sine +
lock-in (`sysid.py`, `osem.v6.py`) fixes both — magnitude and phase both fall out, no quadrature
guessed, divided by the **commanded** drive.

**Bench 2026-08-04, a quarter of a matrix.** Rank check **passed**: singular values
1.000 / 0.154 / 0.058 / 0.043, two directions above 10% against two expected — four coils drive two
DOF. But only **4 of 12 frequencies were usable** (ch2 railed in 17 of 48 steps), and two of those
four, 0.3797 and 0.4805 Hz, are near-identical across coils — ambient-, not drive-dominated. The
multisine variant (git history) **failed**: ch2 railed in all four passes, and its pass 1 ran
14.8 Hz effective against ~252 Hz, hence ~4700 V/V against ~70 — aliasing, not physics.

One cause: **open loop, nothing damping, so the optic rings up and ch2 clips.** No OSEM rests at
mid-scale (bench 600 / 631 / 708 / 677 counts against 511.5), so every channel clips its top rail
first, and bias cannot centre them — least squares wants −1.93 … +3.77 V against 2.5 V of DAC.
v7's trim gets total offset 565 → 455 counts.

**Next: closed-loop dither**, the loop holding the optic in range while the known signal rides on
top. **Done when** the matrix matches ground truth in sign and to ~20% in magnitude, and the modal
controller damps on the bench.

## 3. Characterise the −0.04 instability — bench, gates 4 and 5

Every gain in the repo rests on one sentence in a deleted docstring, *"onset of instability/rail at
gain=-0.04"*. No data records it, `provenance.md` confirms the report never mentions it, and the
simulator cannot reproduce it — the plant is linear and damping rises monotonically to −0.6.

Sweep on the bench keeping the 12-column CSV *and* the scope trace, separating where clipping starts
from where oscillation grows. For the mechanism this is the one place nonlinear system ID (NARX, or
a small network) beats a physics model: saturation, magnetic nonlinearity, or the OSEM shadow
response leaving its linear range. **Done when** a logged run in `data/` shows the onset with a
stated mechanism.

## 4. Resonant / internal-model control

The floor is a persistent narrowband disturbance near 1 Hz, so asymptotic rejection needs a loop
pole there. A lightly-damped complex pole pair at f₀ — a "boost", standard on real suspensions —
puts gain only where the disturbance lives: the one route below 0.23 without the broadband gain
that hits −0.04. Costs phase, so it interacts with item 5, and needs f₀, which nothing estimates:
`zero-crossing → f̂` is drawn in the report's block diagram and was never built.

## 5. Kalman velocity estimator

Finite difference + 5 Hz lowpass is pure phase lag at 1 Hz, and phase lag is stability margin. A
Kalman filter on the known second-order model is the optimal noisy-position → velocity estimator:
gain headroom for item 4, and the **leading hypothesis for item 3** — why −0.04 destabilises
hardware when the linear model says it should not.

## 6. Stop paying 20 s of zero gain for a baseline

`CALIBRATION_S` holds gain at zero to measure `baseline_rms`, the undamped floor; the runaway
breaker (`>1.8x`), the gain scheduler and the lock detector (`<0.35x`) all scale off it. Bench,
four channels: 20 s + ~11 s to lock = **65% of wall clock at zero gain**, plus 25 s open loop after
a fault — ~40 s of not damping per kick.

**Why not simply shorter.** A quiet lab needs 3 s (3 s and 20 s agree, 0.8% scatter over 8 seeds);
the length exists for one shock inside the window. Worst-channel skew from a 6 V/s kick:

| CALIBRATION_S | 5 s | 8 s | 10 s | 15 s | **20 s** |
|---|---|---|---|---|---|
| worst skew | 74.3% | 86.5% | 41.0% | 57.9% | **20.1%** |

Against a **35%** assertion line and **39.3%** un-fixed, every shorter window fails, some worse than
the defect. Cause is **ringdown** — Q = 50, f0 ~ 1 Hz, tau = Q/(pi·f0) ~ **16 s** — so
median-of-sub-windows helps only when the window is comparable to tau; at 10 s every sub-window is
dirty, which is why 12 s is also worse than 10 s. **Scale the ceiling off Q, do not hardcode it.**

### 6a. Adaptive window — built, bench-failed on one constant

`fast-calib` (v9). Simulator: calibration 20.00 → **6.01 s**, time-to-lock 30.6 → **16.5 s**, and
swept-shock skew *improves* 19.7% → 17.6%, because a shock inside the window refuses to converge
early and is still measured the old way. Swept over 16 lab conditions, 12 exit at 6.01 s within
7.0% of the full-window floor and 4 run the ceiling, returning the old number to 0.0%.

**Bench 2026-08-04: never fired.** The agreement tolerance (max/min ratio **1.20**) was tuned on
simulator noise, one RNG seed; the bench floor does not satisfy it, so it burns the ceiling and
falls back to the old estimator — safe, but the saving is unconfirmed on hardware. **Retune it from
the nine bench logs already recorded; done when** it exits early on the bench and still holds shock
skew under 35%.

`warm-restart` (same version) did transfer: two 15 V/s kicks into a locked loop, re-engages 5 s
after the all-clear and re-locks 10.7 s later; without it, no LOCKED inside 115 s.

### 6b. Always-on baseline

**The trap:** `baseline_rms` is the *undamped* floor, but the live bandpassed signal closed-loop is
the *damped* level, ~0.03x of it — the lock detector would demand 0.35x of an already-damped signal
and the breaker would trip on ordinary motion.

| | how | catch |
|---|---|---|
| (a) invert the ratio | open-loop ~ closed-loop / floor(Kp) | 0.173 predicted vs 0.239 measured at Kp −0.030, ~40% error: seed only |
| (b) witness channel at bias | one rigid body, so it reads the undamped floor free | costs a quarter of the damping authority — blocked, below |
| **(c) out-of-band — build this** | loop has authority only in 0.4–3 Hz; outside it the plant is open loop at any gain, so track the out-of-band level and scale | scaling characterised once, then free and costs no channel |

(b) waited on eight channels. **All eight OSEMs are connected** (scope-confirmed 2026-08-06; the
earlier "a4–a7 not wired" call was wrong). The split is in-band vs out-of-band: a0–a3 carry 77–99%
of their power in 0.4–3 Hz and a5 22–71%, but a4/a6/a7 carry only 0.1–9% — 91–99% of their signal
sits at 3–20 Hz, dominated by a narrow **6.19 Hz** line (12.38 Hz harmonic on a7) absent from
a0–a3. Their flags are outside the linear partial-shadow region: **alignment, not wiring.** a5 is
the same story one step milder — in band, at ~1/12 of a0's gain.

The bandpass already removes the 6.19 Hz line, which is why those channels calibrated ~0.002 V
baselines and blew up `env/baseline` in the v5.5 run. Method note: the original call used a DC
threshold and raw correlation, both blind to a small coherent signal. Test for a peak at the known
resonance. See `analysis/where_is_power.py`.

`auto-disable` is built (v4 on) but **still never exercised on hardware** — `CLAUDE.md` item 4.
Do not chase "the report's 0.085 Hz error"; `0.085` occurs nowhere in the repo or in
`provenance.md`'s text layer.

## Where learning belongs

Learn the model, the estimator and the anomaly detector; keep the control law and the interlocks
classical and certifiable. That is what lets you argue the machine cannot destroy a suspended
mirror.
