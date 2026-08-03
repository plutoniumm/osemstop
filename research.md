# Research queue

Ranked by impact × confidence × cost, **in dependency order** — each one is
partly gated by the ones above it. Item 1 is **done**; nothing below it is
started.

Everything below rests on measurements in this repo, not on estimates. Where a
number appears, `harness.py` produced it.

---

## The measurement everything else follows from

The amplitude ratio settles near **0.23 and does not go to zero**. That is not a
failure to converge — it is the steady state of a *continuously forced*
oscillator. Removing the drive mid-run:

| | damped (Kp −0.030) | gain off |
|---|---|---|
| decay time constant | **3.4 s** | 15.9 s |
| predicted | 2.76 s | 15.92 s |

The undamped case matches theory to 0.01 s, so the model and the loop are both
sound. A gain sweep confirms the floor tracks the ratio of damping coefficients,
`intrinsic / (intrinsic + added)`:

| Kp | predicted floor | measured |
|---|---|---|
| −0.005 | 0.557 | 0.545 |
| −0.030 | 0.173 | 0.239 |
| −0.100 | 0.059 | 0.088 |
| −0.300 | 0.021 | 0.031 |

So the floor falls with gain, and gain is capped by an instability at −0.04 that
**has no recorded evidence anywhere**. That is why item 3 gates items 4 and 5.

One negative result worth keeping: the residual after the drive stops
(0.0089 V) is **not** sensor-noise injection — it is identical at 20 mV, 5 mV and
0 mV of sensor noise (0.0089 / 0.0087 / 0.0094). It is `RateLimitedActuator`'s
0.5 mV deadband and 10 ms throttle. That is a firmware limit, not a sensing one.

---

## 1. Fix the suite for the rigid-body plant — **DONE**

**Why it was first:** `make check` was red with 13 failures, 7 of them *stale
assertions* rather than regressions — they encoded the old
four-independent-oscillator plant. Until the test bed stopped lying, nothing
else on this list could be validated.

**Now:** 88 checks, **83 pass, 5 fail, ~120 s**. v0–v3 are green (18 / 16 / 18 /
18). The 5 remaining failures were all in v4, and all real: it did not damp (ratios
0.24–0.77), never locks, faults once in a quiet lab, and its I term never
accumulates so the P-dominance check has nothing to compare. That is item 2.

Three assertions were rewritten in `harness.py`; no controller was touched.

- *"disabled channels stay at their baseline"* → split in two. A disabled
  channel must never actuate (peak `|out − bias|` is exactly 0), but its
  amplitude ratio must **fall** — one rigid body, so damping ch0 takes energy
  out of the whole mass and all four OSEMs see it, which is `provenance.md` §3.
  Measured on v0: driven ch0 = 0.197, undriven 0.265 / 0.323 / 0.355, and the
  driven channel is still the flattest, which is the report's other claim.
- *"a runaway trip auto-recovers"* → the scenario it used (all four on, ch2's
  sign inverted) no longer produces a runaway at all. Inverting ch2 leaves the
  other three out-damping it — the net of `gain_i · COIL_GAIN_i` goes from
  −0.1415 to −0.0419, so about a third of the damping survives — and the optic
  simply stays near full amplitude, where the velocity feedback asks for more
  than the ±0.25 V it is clipped to. It trips on **ch3 saturation** at t = 10.74 s
  (`sat_streak` = 31), which is defect #1 and latches. That is now asserted as
  such, from the broken side for v0/v1/v2 and the fixed side for v3. The
  runaway path gets its own scenario — ch2 alone, wrong-signed at half
  magnitude, so the growth is slow enough to trip on amplitude ratio while the
  actuator peaks at 0.055–0.112 V of its 0.25 V clip — and every version must
  recover from it. For the un-fixed versions recovery *is* the proof that
  nothing saturated, since a saturation trip would latch forever.
- *"KNOWN GAP / FIXED runaway-baseline"* → the 18% line was calibrated on the
  old plant and no longer separates anything. Re-measured: see the sweep table
  in `versions.md` § v3. One shock, swept across the calibration window, worst
  case 45.3% (v0/v1/v2) against 25.0% (v3); the line is now 35%. A single
  fixed shock time ranks the two backwards for late shocks, so the check sweeps.

All of that is simulator, on the rigid-body plant, **never hardware**.

---

## 2. Modal (MIMO) damping — what killed the last attempt

This is the main **beyond-PID** direction, and it has been tried once. `osem.v4.py`
implemented it and was deleted on 2026-08-03 without ever damping; it survives in
git history. Read this before starting again, because the failure was specific and
is worth not repeating.

**The diagnosis, already made.** v4 learned the modal basis correctly with no
geometry input — it reported "2 of 4 modes carry real motion, 98.8% / 1.2%" from
the calibration covariance alone. It failed at exactly one point: recovering the
actuation matrix.

Truth (LONG row, normalised): `[−0.59, −0.45, +2.45, −0.51]` — ch2 stands out at
4× with the opposite sign. Measured:

| scheme | result |
|---|---|
| velocity × PRBS (what v4 did) | `[2.0, 0.2, −0.53, −1.27]` |
| acceleration × PRBS | `[−0.71, −0.47, −0.86, −1.96]` |

Three compounding causes:

1. **Wrong quadrature — the fundamental one.** For a second-order plant, force
   acts instantaneously on *acceleration*; velocity is its integral. A zero-lag
   correlation on velocity has no coherent term to find. (The earlier fix from
   displacement to velocity was right *for a sinusoid at resonance* and was
   silently invalidated when the excitation became broadband PRBS.)
2. **Simultaneous excitation, sequences too short.** ~40 bits gives ~16%
   cross-talk between channels — comparable to the differences being measured.
   The four LFSR seeds are also phase shifts of one m-sequence.
3. **SNR.** Dither force ≈ 1.0 against ambient drive 0.8 and seismic 0.8.

**The fix.** Fit a *state-space model* to the dither data instead of correlating
at one lag — lag structure is what the states are for. Mature and available
(`scipy`, `python-control`). Alternatively the bench-standard **stepped sine,
one coil at a time, both quadratures**, so magnitude and phase both fall out and
the sign never depends on guessing a quadrature in advance.

**Done when:** the recovered matrix matches ground truth in sign and to ~20% in
magnitude, and the modal controller damps on the bench.

**Note the cheaper option.** The actuation matrix does not have to be learned
online at all. With the bench in front of you, drive one coil at a time with a
stepped sine and record the response — that is a one-afternoon measurement that
yields the same matrix with no convergence risk, and it is what v4's online
system ID kept getting wrong.

---

## 3. Characterise the −0.04 instability on the bench

**The only item that needs hardware, and it gates 4 and 5.**

Every gain in the repo rests on one sentence in `osem.v0.py`'s docstring:
*"onset of instability/rail at gain=-0.04."* That number appears in no data
anywhere. `provenance.md` — written two days later — never mentions it: no sweep,
no −0.04, no rail-onset trace. And the simulator provably cannot reproduce it,
because the plant is linear: here, more negative Kp is monotonically more
damping all the way to −0.6.

Items 4 and 5 both work by **raising effective gain**. Pushing on an
undocumented stability limit is the wrong order of operations.

**How:** re-run the sweep on the bench, keep the 12-column CSV *and* the scope
trace, and record where clipping starts versus where oscillation grows — they
are different failures. If you want the mechanism, this is the one place where
**nonlinear system ID** (NARX, or a small network) genuinely beats a physics
model: something nonlinear is happening that the linear model cannot express —
actuator saturation, magnetic nonlinearity, or the OSEM shadow response leaving
its linear range.

**Done when:** there is a logged run in `data/` showing the onset, and a stated
mechanism.

---

## 4. Resonant / internal-model control (v5)

**The biggest single win on the actual objective.**

The floor is set by a *persistent narrowband* disturbance near 1 Hz. The
internal model principle: to asymptotically reject a sinusoid, the loop must
contain a pole at that frequency. A lightly-damped complex pole pair at f₀ — a
"boost" or resonant gain stage, standard on real suspensions — gives enormous
loop gain exactly where the disturbance lives and almost none elsewhere.

That is the one way to attack the 0.23 floor **without** the broadband gain
increase that runs into −0.04.

**Watch for:** a resonant stage adds phase, so it interacts with item 5, and it
needs f₀ known. Note the repo has no frequency estimator — `provenance.md`'s block
diagram specifies `zero-crossing → f̂` in Calibration and it was never
implemented. This item is the reason to build it.

---

## 5. Kalman velocity estimator

The current chain is a finite difference plus a 5 Hz lowpass. That lowpass is
pure phase lag at 1 Hz, and phase lag is what consumes stability margin. A
Kalman filter on the known second-order model is the statistically optimal
estimator for exactly this problem — noisy position in, velocity out.

Two payoffs: real gain headroom for item 4, and it is the **leading hypothesis
for item 3** — why −0.04 destabilises on hardware when the linear model says it
should not.

---

## Deliberately cut, with reasons

- **Actuator deadband / rate-limit fix.** Measured to set the current floor
  (0.009 V, identical at 0 / 5 / 20 mV sensor noise, so not the sensor). Real,
  cheap, firmware-only — but it only matters once you are near 0.009 and you are
  at 0.23. Premature.
- **Safe Bayesian optimisation for gain tuning.** Genuinely good fit: expensive
  trials, few parameters, a hard safety constraint (SafeOpt-style methods
  explore without violating one). Becomes the right tool *after* item 3 says
  where the constraint actually is.
- **CUSUM / change-point interlocks.** The principled fix for the two
  threshold-based defects — bug #2 is "3 counts, 250 consecutive samples", bug
  #3 is "2 s RMS > 1.8 × an 8 s baseline". Both fail by testing an instantaneous
  threshold instead of accumulating evidence. v3's fixes are better *thresholds*;
  this is the better *kind of thing*. Worth doing, not urgent.
- **Feedforward / Wiener filtering, LQG / H∞, adaptive notch (LMS), ICA.** All
  wait on a validated plant model, which is item 2. Wiener feedforward from a
  seismometer is the standard technique in real detectors and is the strongest
  of these once a witness sensor exists.
- **Reinforcement learning for the damping law.** Argued against despite the
  appeal: sample-hungry on hardware, no stability guarantee, hard safety
  constraint. Sim-to-real is the usual escape, but this simulator's one
  documented blind spot is the −0.04 instability — so an agent trained on it
  would learn confidently that more gain is always better, which is the single
  lesson you least want it to learn.
- **`ENABLE_CHANNEL` vs the report, and `auto-disable`.** Only the first is
  written down: `versions.md` § v1 and `README.md` § "Channel mapping" both spell
  out that v0's "only ch0 is validated" predates `provenance.md` §4 by two days and
  that v1–v3 ship all four channels on. `auto-disable` is documented nowhere in
  this repo — it appears once, in the report's block-diagram Safety Checks box
  next to `lock detect`, and in no controller (`grep auto-disable osem.v*.py` is
  empty). Like the `zero-crossing → f̂` estimator in item 4, it was drawn and
  never implemented. This bullet used to also list "the report's 0.085 Hz error"
  and to credit all three to `versions.md`; both were wrong. `versions.md`
  contains neither `0.085` nor any mention of `auto-disable`, and `0.085` occurs
  nowhere in the repo or in `provenance.md`'s text layer, so whatever that error was
  has left no record — do not act on it without re-deriving it from the report.
  No action needed until wanted.

## Where learning belongs

Learn the **model**, the **estimator**, and the **anomaly detector**. Keep the
control law and the interlocks classical and certifiable. That split is what is
actually done on real detectors, and it is what lets you argue the machine
cannot destroy the optic — which is the argument that matters when the thing on
the other end is a suspended mirror.
