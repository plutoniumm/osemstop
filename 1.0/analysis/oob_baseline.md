# Out-of-band baseline — does it work?

*(The proposal was `research.md` item 6b option (c); that file was folded into
`CLAUDE.md` and deleted 2026-08-17.)*

**No. Do not build it.** Regenerate every number with
`.venv/bin/python analysis/oob_baseline.py` (25 runs, 3 days, 20.0 min of logged
open loop and 6.0 min of closed loop; runs in ~40 s).

Two independent failures, either one fatal.

1. **No band is both loop-free and informative.** The bands that carry motion
   (3–8 Hz) are *inside* the loop's authority — they drop to **0.28–0.55×** when
   the gain engages, because `BP_HIGH_HZ = 3.0` is a one-pole −3 dB corner, not a
   wall. The bands that are genuinely loop-free (>8 Hz, and only at the 900 Hz
   loop rate) contain nothing but **50 Hz mains and a flat 0.16 counts/√Hz ADC
   floor**. They are invariant because they are instrument, not lab.
2. **The scale factor does not hold still.** Leave-one-run-out, the proposal as
   written (`baseline = α × out-of-band`) predicts the in-band floor to
   **20–67% median / 58–920% at p90** depending on channel and band — **no better
   than a per-channel constant** (39–60% / 85–161%), which uses no out-of-band
   term at all. α itself scatters **43–181% (robust 1σ) across runs** and moves by
   up to **5.4×** across the 357 → 1111 Hz sample-rate change.

Against the ~40% bar that sank option (a): the *median* clears 40% for 10 of the
36 (channel, band) pairs but for **no band on all four channels at once**, the
**p90 clears it for none of the 36**, and α does not transfer between days.
**It does not clear the bar.**

And the third failure, specific to the hardware that flew on 08-03/08-04: at the
18 Hz loop rate the >8 Hz bands are not loop-free either — they *rise* **3.6×
(8–20 Hz)** and **8.2× (20–60 Hz)** when the gain engages, from the throttled DAC.

**What to do instead.** The 20 s calibration is not statistically necessary.
Shortening the existing in-band measurement gives 11% error at 2 s and **4% at
4 s** against its own 18 s answer — better than any out-of-band estimator at any
window length, with no new physics and no new characterisation. The 20 s exists
so `CALIB_SUBWINDOWS = 5` can outvote a transient (`osem.v3.py`
`accumulate_calibration`), not for convergence. If the goal is the "lock within
10 s including calibration" target, that is where the time is.

---

## Method

Reference quantity is `baseline_rms` as the controllers define it: RMS of the
0.4–3.0 Hz bandpassed sensor signal over a zero-gain window. Measured here with a
brick-wall FFT band on raw counts; §0 of the script checks that against the logged
`ch*_bp` column and gets a fixed **1.15–1.17×** on every healthy channel, a pure
scale any α absorbs (a4/a6 differ — they have no in-band signal to scale).

Open loop = `CALIBRATING` **or** `FAULT`; in both the actuator is pinned at
`BIAS` (measured output std 0.000 V in `CALIBRATING`, ≤0.002 V in `FAULT`).
Closed loop = `DAMPING` with
the scheduled gain slewed in. Windows are 18 s, not 20 — the logged `CALIBRATING`
segments run 19.3–19.9 s, so a 20 s window keeps 2 runs and an 18 s window keeps
22.

Scatter is quoted as a robust 1σ: 68th percentile of |log x − median log x|,
reported as a percentage. It is insensitive to the one or two runs that were
kicked hard.

## Q1 — where could the band live?

Open-loop ASD, counts/√Hz, median over all zero-gain windows:

| ch | 0.05–0.2 | 0.2–0.4 | **0.4–1** | **1–3** | 3–5 | 5–8 | 8–20 | 20–45 | 45–55 | 55–150 |
|---|---|---|---|---|---|---|---|---|---|---|
| a0 (357 Hz) | 4.56 | 8.89 | **82.4** | **34.8** | 2.71 | 1.14 | 0.62 | 0.57 | 1.25 | 0.73 |
| a3 (357 Hz) | 1.03 | 1.02 | **30.1** | **15.2** | 0.99 | 0.49 | 0.18 | 0.08 | 0.08 | 0.07 |
| a0 (1111 Hz) | 1.88 | 2.57 | **34.8** | **13.1** | 0.27 | 0.23 | 0.17 | 0.16 | 9.13 | 0.16 |
| a3 (1111 Hz) | 0.61 | 0.34 | **15.9** | **4.94** | 0.20 | 0.22 | 0.17 | 0.17 | 0.33 | 0.18 |

The signal is 2–3 decades down by 8 Hz and then **flat at 0.16–0.17 counts/√Hz on
every channel** — that is the ADC/electronics floor, not the mirror. The only
structure above it is a **49.91/50.05 Hz mains pair** present on all eight channels
(ASD 56 counts/√Hz on a0, 31 on a5, 1.1–1.8 on a2/a3). Below 0.4 Hz is drift and
TIA wander, uncorrelated or anti-correlated with the in-band level.

So the candidate set is: 3–8 Hz (real motion, questionable loop-freedom),
>8 Hz (mains + ADC floor), <0.4 Hz (drift).

## Q2 — does the out-of-band level survive the gain? *(load-bearing)*

Paired windows across one open-loop → `DAMPING` transition in the same run:
4 s immediately before, 4 s starting 4 s after. Ratio closed/open, median [p16, p84].

| band | 08-03 (loop 18 Hz) | 08-04 (loop 18 Hz) | 08-06 (loop 900 Hz) |
|---|---|---|---|
| **in-band 0.4–3** | **0.03** [0.02, 0.10] | **0.08** [0.05, 0.10] | **0.26** [0.17, 0.47] |
| 0.05–0.2 | 1.14 [0.65, 1.60] | 0.39 [0.24, 1.26] | 0.69 [0.45, 0.97] |
| 0.2–0.4 | 0.91 [0.32, 1.08] | 0.63 [0.34, 1.39] | 1.08 [0.60, 1.32] |
| 3–5 | **0.28** [0.16, 0.57] | **0.36** [0.18, 0.84] | 1.07 [0.77, 1.17] |
| 4–8 | **0.40** [0.13, 0.82] | **0.44** [0.21, 0.71] | 1.07 [0.69, 1.17] |
| 6.5–8 | 0.41 [0.13, 0.82] | 0.55 [0.38, 1.14] | 1.10 [0.75, 1.26] |
| 8–20 | 0.74 [0.46, 1.42] | **3.58** [2.79, 4.87] | 1.11 [0.85, 1.29] |
| 20–60 | 1.11 [0.75, 1.32] | **8.19** [4.62, 10.19] | 1.00 [0.94, 1.19] |
| 60–150 | 1.15 [0.94, 1.24] | **6.14** [2.20, 17.03] | 1.11 [0.95, 1.36] |

Read it as three separate findings.

- **3–8 Hz is not out of band.** It falls by 2–4× with the gain on both 18 Hz-loop
  days. The one-pole `BP_HIGH_HZ` corner leaves real authority out to ~8 Hz. On
  08-06 it does not fall — but by then the in-band suppression is only 0.26 (not
  0.03), so there is much less authority to leak.
- **>8 Hz was not loop-free at the 18 Hz loop rate.** On 08-04 the loop injects
  3.6–8.2×. Control: in the same runs, channels whose own coil was *not* driven
  show 8–20 Hz at **1.03** and 20–60 Hz at **1.32** against the driven channels'
  3.58 and 8.19 — the injection is each loop putting broadband energy into its own
  coil, exactly as expected from a throttled DAC stepping at 18 Hz.
- **>8 Hz is loop-free at 900 Hz** — 1.00–1.11, tightest of anything measured
  (20–60 Hz sits in [0.94, 1.19]). This is the one clean pass in the table, and
  §Q3 shows why it is worthless: that band is the mains line and the ADC floor.

So Q2 alone kills the idea on v1–v9-era hardware and passes it only on v10/v11-era
hardware, in a band that then fails Q3.

## Q3 — the scale factor and its scatter

α = in-band RMS / out-of-band RMS, one point per (run, channel), 22 runs.

| band | a0 α (scatter) | a1 | a2 | a3 |
|---|---|---|---|---|
| 0.05–0.2 | 32.0 (122%) | 30.8 (136%) | 32.5 (150%) | 42.1 (126%) |
| 0.2–0.4 | 17.0 (**43%**) | 24.0 (73%) | 23.3 (62%) | 31.4 (84%) |
| 3–5 | 17.6 (138%) | 30.8 (134%) | 15.9 (118%) | 20.3 (181%) |
| 4–8 | 22.5 (94%) | 34.4 (132%) | 21.0 (101%) | 21.6 (180%) |
| 8–20 | 29.9 (68%) | 45.8 (90%) | 60.9 (154%) | 40.0 (154%) |
| 20–60 | 13.3 (120%) | 27.4 (88%) | 117.1 (92%) | 47.0 (84%) |

Best case is 0.2–0.4 Hz at 43–84% — and that band moved 0.63–1.08 with the gain
in Q2, so it is not out of band either.

**Across days and sample rates** (α normalised per channel to its all-run median):

| band | 08-03 | 08-04 | 08-06 | 357 Hz | 1111 Hz | all-run scatter |
|---|---|---|---|---|---|---|
| 0.2–0.4 | 0.81 | 1.43 | 0.92 | 1.04 | 0.92 | 63% |
| 3–5 | 0.90 | 0.97 | 1.12 | 1.00 | 1.49 | 137% |
| 8–20 | 1.01 | 1.04 | 0.77 | 1.03 | 0.71 | 121% |
| 20–60 | 1.38 | 0.99 | **0.34** | 1.02 | **0.19** | 101% |

The one band that passed Q2 (20–60 Hz at 900 Hz loop rate) has an α that changes
by **5.4× between the 357 Hz and 1111 Hz runs**. "Characterise once, free forever"
does not survive a sample-rate change, let alone a re-alignment.

**Leave-one-run-out prediction of the in-band floor**, |error| median / p90:

| predictor | a0 | a1 | a2 | a3 |
|---|---|---|---|---|
| **constant, no out-of-band term** | 60% / 161% | 48% / 92% | 39% / 85% | 56% / 99% |
| α × (0.2–0.4 Hz) | **20% / 58%** | 47% / 72% | 33% / 67% | 39% / 91% |
| α × (3–5 Hz) | 54% / 322% | 56% / 696% | 52% / 220% | 63% / 388% |
| α × (8–20 Hz) | 35% / 281% | 40% / 280% | 48% / 461% | 67% / 324% |
| α × (20–60 Hz) | 53% / 246% | 49% / 403% | 32% / 198% | 37% / 199% |

A fitted power law `a·oob^b` does better (23–43% median) but its exponent comes
out **b ≈ +0.45 to +0.56** on every motion-carrying band — the fit is shrinking
hard toward the mean, i.e. telling you the out-of-band level explains about half
of a decade per decade. It is also a different model from the one proposed, and
its p90 is still 78–136%.

### The control group settles it

a4/a6/a7 carry no motion (lock-in SNR 1.1–1.5; 91–99% of their power in the
6.19 Hz line). Pooled 10 s zero-gain windows, correlation of log(in-band) against
log(out-of-band), and the ratio's scatter:

| band | a0 | a1 | a2 | a3 | a4 | a6 | a7 |
|---|---|---|---|---|---|---|---|
| 3–5 | +0.57 (264%) | +0.57 (246%) | +0.76 (205%) | +0.63 (195%) | **+0.89 (39%)** | **+0.97 (33%)** | **+0.97 (34%)** |
| 8–20 | +0.38 (151%) | +0.53 (150%) | +0.67 (140%) | +0.61 (141%) | +0.58 (79%) | +0.68 (83%) | +0.93 (102%) |

The method works **6× better on the channels that see nothing**. On a dead channel
both "in-band" and "out-of-band" are the same electronics noise, so of course they
track. That is the signature of a measurement of the instrument rather than the
mirror, and it is what the real channels' 195–264% is telling you.

## Q4 — how fast does it converge?

Reference is the in-band RMS over a full 18 s zero-gain window. Each estimator
gets the **first T seconds** of that same window. Median |error| over 88
(run, channel) windows:

| estimator | 0.5 s | 1 s | 2 s | 4 s | 8 s | 12 s | 18 s |
|---|---|---|---|---|---|---|---|
| **in-band(T)** — just shorten the existing calibration | 23% | 22% | **11%** | **4%** | 3% | 2% | 0% |
| α × (0.2–0.4 Hz) | 72% | 64% | 60% | 38% | 28% | 29% | **27%** |
| α × (3–5 Hz) | 155% | 88% | 65% | 47% | 35% | 42% | 39% |
| α × (8–20 Hz) | 125% | 71% | 57% | 41% | 35% | 32% | 32% |
| α × (20–60 Hz) | 57% | 50% | 46% | 43% | 40% | 41% | 38% |

Every out-of-band estimator **plateaus at 27–50% and stops improving** — that
plateau is the α-transfer error from Q3, not measurement noise, so more window
length cannot fix it. Meanwhile the in-band estimate is at 4% after 4 s.

(in-band(T) is nested inside its own reference, so this measures convergence
within a window, not against a future floor. That is the right comparison here —
both estimators are scored against the same reference — but see the next
paragraph for what the floor itself does.)

**The target is not stationary anyway.** Between one 18 s open-loop window and
the next in the same run, the in-band floor moves by a robust 1σ of
**25 / 99 / 66 / 117%** on a0/a1/a2/a3. No baseline estimator, in-band or out, is
better than ~50% accurate about the *next* 18 s. This is worth knowing
independently: it bounds what any of `RUNAWAY_MULTIPLE = 1.8` or
`LOCK_RMS_FACTOR = 0.35` can mean, since both are ratios against a number that
itself wanders by that much.

## What would have to be true for this to work

For the record, if someone wants to revisit:

1. A band that is genuinely outside loop authority **and** carries seismic drive.
   That means a real roll-off at `BP_HIGH_HZ`, not a one-pole corner — a
   2nd/4th-order bandpass would put the 3–8 Hz motion outside the loop, where it
   is currently 0.28–0.55× suppressed. This is the only change that could revive
   the idea, and it is a change to the controller, not to the estimator.
2. Mains and ADC floor excluded by construction, or the "invariant" band is
   invariant for the wrong reason.
3. α re-characterised after any change to sample rate, loop rate, or alignment —
   measured drifts of 5.4×, 1.4×, and (from the a4–a7 story) whatever a flag
   moving out of the linear region does.
4. Even granting all three, the answer arrives at 27–50% against a target that
   itself wanders 25–117% per window. The 20 s it saves is better bought by
   shortening the in-band calibration to 4 s.

## Files

- `analysis/oob_baseline.py` — regenerates every number above.
- Related: `analysis/where_is_power.py` (in-band vs out-of-band power fractions),
  `analysis/kp040.md` (per-run context).
