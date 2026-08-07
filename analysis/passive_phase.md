# Is the ambient 0.4–3 Hz disturbance phase-stable enough to cancel?

**Verdict: no — not as an addition to the damping loop, which is the only way it
would ever run.** Regenerate every number with
`.venv/bin/python analysis/passive_phase.py` (31 logs, 3 days, 972 s of logged
open loop in 47 windows and 1206 s of long closed-loop segments; ~2 min).

**The number that kills it.** The same canceller that removes **43–62 %** of the
in-band power from open-loop data removes **−11 % to +13 %** when it runs on top
of the existing velocity damper — i.e. nothing, and *negative on a2/a3*. Push it
to three tones and a 4 s application window and it reaches **−68 %**: it pumps
harder than it damps. (§4, `ch*_bp`, K=1, `Test`=8 s, τ=0.5 s.) The damper has
already taken the predictable part; what it leaves is not predictable.

**The headline phase number.** The RMS wrapped phase increment of the dominant
in-band line reaches **1 radian at 5.7–6.5 s** on a0/a2 (0.70 Hz) and is
**0.8–1.0 rad by 4 s** on every channel and every line. It does not go to
1.81 rad (fully random) — the motion is not white — but it is at
**0.33–0.48 rad after only 0.5 s**, and 0.5 rad of phase error caps an otherwise
perfect anti-phase injection at **6 dB**. Read against the numbers above: the
ceiling is not what stops this, the damper already being there is.

**Timescale, stated plainly.** Useful coherence (phase error < 0.5 rad) lasts
**≈ 0.5–1 s**. Partial coherence (< 1 rad) lasts **≈ 4–7 s**. Beyond ~4 s the
phase error saturates at 0.8–1.0 rad and never recovers. Any canceller would
have to re-estimate at ≥ 1 Hz, which is not "feedforward" in any useful sense —
it is a narrowband feedback loop with the same authority and the same sign risk.

---

## Why it fails, in one line

The in-band floor is **not a narrowband disturbance**. It is broadband seismic
drive filtered by the suspension's own **Q ≈ 20 resonances at 0.70 / 1.00 /
1.65 Hz**, which is why it *looks* narrowband in a spectrum and behaves like
noise in time. Its line frequency wanders by **218 mHz = 6.4 half-power widths**
between segments (§5), its amplitude changes by a factor of ~2 between adjacent
4 s windows (IQR/median 0.87–1.81), and across that frequency wander the plant
phase swings by **162°** (§6). Anti-phase and in-phase are separated by less
than the measurement's own frequency uncertainty.

---

## Method, and the two things that changed the answer

Reference quantity is the 0.4–3.0 Hz motion as the controller defines it. Two
routes are used and they agree:

- **phase tracking** — a non-uniform lock-in run straight on the raw irregular
  samples, 4 s Hann window, 0.5 s hop, with each window's phase referred to a
  **common** time origin;
- **canceller simulation** — the proposal itself, run on `ch*_bp`, the
  controller's own *causal* one-pole 0.4→3.0 Hz cascade as logged
  (`osem.v11.py` `OnePole`, `BP_LOW_HZ`/`BP_HIGH_HZ`). At each step it locks in
  over the previous `Test` seconds, takes the K strongest tones in band,
  extrapolates them, and subtracts over `[t+τ, t+τ+Tapp]`. No sample from the
  application interval enters the estimate.

Both were wrong the first time, in ways worth recording because they both
manufacture a *positive* result:

1. **Referring each lock-in window's phase to its own start** adds a spurious
   rotation of `2π·f0·hop` per hop. At f0 = 1.00 Hz with a 0.5 s hop that is
   exactly π, the unwrap then fails, and the measured phase increment came out
   **1.9–2.1 rad at 0.5 s** — a fabricated total decorrelation. Corrected, the
   same data gives 0.33–0.38 rad.
2. **A brick-wall FFT bandpass over a whole window is non-causal.** It leaks the
   application interval back into the estimation window. With that filter the
   open-loop canceller scored 0.47/0.56/0.64/0.64; on the causal `ch*_bp` it
   scores 0.43/0.55/0.62/0.62. Small here, but it is a look-ahead and it does not
   stay small — it was worth ~+0.10 in the closed-loop case.

Open loop = `CALIBRATING` or `FAULT` (actuator pinned at `BIAS`; verified in
`analysis/oob_baseline.md`). Closed loop = `DAMPING` with the first 8 s dropped,
because the gain is still slewing (`ch0_gain` reaches its final −0.0350 at
t = 147.8 s in a segment starting at 146.1 s).

**Hard limit of the dataset:** the longest contiguous open-loop window anywhere
in `data/` is **48.8 s**, and 45 of the 47 usable ones are 15–20 s. Open-loop
lags beyond ~10 s cannot be measured from what exists. Lags out to 60 s come
only from closed-loop segments, where the loop has already broadened the lines.

---

## Q1 — where the in-band power actually is

Open-loop ASD, counts/√Hz, median over 47 windows:

| ch | 0.70 Hz | 1.00 Hz | 1.65 Hz | 2.30 Hz | 6.09 Hz |
|---|---|---|---|---|---|
| a0 | 95.9 | **127.4** | 61.9 | 6.8 | 0.7 |
| a1 | **58.3** | 48.5 | 48.2 | 0.7 | 0.3 |
| a2 | 151.8 | **193.0** | 100.1 | 4.1 | 0.9 |
| a3 | 47.2 | 43.0 | 44.1 | 2.1 | 0.3 |
| a4 | 0.4 | 0.6 | 0.3 | 0.2 | **1.7** |
| a5 | 4.4 | **89.0** | 1.8 | 1.1 | 0.3 |
| a6 | 0.3 | 0.5 | 0.3 | 0.4 | **6.4** |
| a7 | 0.2 | 0.6 | 0.3 | 0.2 | 0.5 |

Fraction of the 0.4–3.0 Hz power inside each line ±0.08 Hz:

| ch | 0.70 | 1.00 | 1.65 | rest |
|---|---|---|---|---|
| a0 | 0.32 | 0.49 | 0.14 | 0.05 |
| a1 | 0.43 | 0.29 | 0.27 | 0.02 |
| a2 | 0.32 | 0.52 | 0.14 | 0.02 |
| a3 | 0.39 | 0.28 | 0.31 | 0.02 |
| a5 | 0.00 | 0.94 | 0.00 | 0.06 |

**"A persistent narrowband disturbance near 1 Hz" (`research.md` item 4) is half
right.** There is not one line, there are **three**, at 0.70 / 1.00 / 1.65 Hz,
and 95–98 % of the in-band power sits in them. But which one dominates changes
between windows — 0.70 Hz leads on a1/a3, 1.00 Hz on a0/a2, and the ranking
flips inside a single 240 s run. A single-tone canceller therefore addresses
28–52 % of the band at best. Two of the three match the modal fit in
`gains.json` (0.7149 and 0.9941 Hz); 1.65 Hz matches the 1.657 Hz differential
mode in `analysis/out/modes.csv`.

a4/a6/a7 carry 0.2–0.6 counts/√Hz in band against 0.5–6.4 at 6.09 Hz. They are
low-gain sensors, not dead ones — they simply put almost nothing in the control
band, so they are irrelevant to this question either way.

---

## Q2 — how fast the phase wanders *(headline)*

RMS **wrapped** phase increment in radians, open loop, pooled over 47 windows.
A best-fit mean frequency is removed per window first, i.e. the canceller is
*given* perfect knowledge of the line's average frequency over that window.
1.81 rad = completely random. τ₁ = lag at which the increment reaches 1 rad.

| line | ch | 0.5 s | 1 s | 2 s | 4 s | 6 s | 8 s | 10 s | **τ₁** |
|---|---|---|---|---|---|---|---|---|---|
| 0.70 | a0 | 0.48 | 0.70 | 0.90 | 0.83 | 0.98 | 1.05 | 0.98 | **6.5 s** |
| 0.70 | a1 | 0.34 | 0.55 | 0.73 | 0.73 | 0.87 | 0.93 | 0.80 | > 10 s |
| 0.70 | a2 | 0.47 | 0.71 | 0.93 | 0.77 | 1.04 | 1.07 | 0.97 | **5.7 s** |
| 0.70 | a3 | 0.37 | 0.60 | 0.80 | 0.70 | 0.96 | 0.99 | 0.97 | > 10 s |
| 1.00 | a0 | 0.33 | 0.54 | 0.76 | 0.85 | 0.88 | 0.90 | 0.89 | > 10 s |
| 1.00 | a2 | 0.33 | 0.53 | 0.72 | 0.71 | 0.85 | 0.91 | 0.80 | > 10 s |
| 1.65 | a1 | 0.12 | 0.17 | 0.24 | 0.26 | 0.27 | 0.25 | 0.26 | > 10 s |
| 1.65 | a2 | 0.27 | 0.38 | 0.52 | 0.66 | 0.73 | 0.65 | 0.55 | > 10 s |

What the numbers cost a canceller — an otherwise perfect anti-phase injection
with RMS phase error δφ leaves `|1−e^{iδφ}|² ≈ δφ²` of the power:

| δφ (rad) | 0.2 | 0.3 | 0.5 | 0.7 | 1.0 | 1.4 |
|---|---|---|---|---|---|---|
| best suppression | 14 dB | 10.5 dB | **6 dB** | 3.1 dB | **0 dB** | 0 dB |

So at a realistic 0.5 s update latency the ceiling is 6–10 dB; at 4 s it is
0–3 dB; beyond 4 s the increment saturates at 0.8–1.0 rad and the ceiling is
zero. Two caveats, both stated honestly:

- the saturation below 1.81 rad is partly an artefact of the 15–20 s record
  length — removing a linear phase ramp from a 20 s window suppresses increments
  at lags comparable to that window. The short-lag numbers (≤ 2 s) are clean.
- the tabulated τ₁ is an *upper* bound on the useful horizon, because it grants
  the canceller a frequency estimate it would not have. §5 shows the frequency
  it would actually have to guess wanders by 7–31 %.

---

## Q3 — controls, so the estimator is not just measuring itself

Same estimator, same window structure, same lags:

| signal | 0.5 s | 1 s | 2 s | 4 s | 6 s | 8 s | 10 s | τ₁ |
|---|---|---|---|---|---|---|---|---|
| **6.09 Hz on a6** (positive: known coherent) | 0.23 | 0.40 | 0.57 | 0.70 | 0.75 | 0.74 | 0.59 | > 10 s |
| 6.09 Hz on a4 | 0.40 | 0.67 | 0.97 | 1.17 | 1.18 | 1.19 | 0.97 | 2.3 s |
| 6.09 Hz on a7 | 0.41 | 0.69 | 1.01 | 1.23 | 1.22 | 1.24 | 1.02 | 2.0 s |
| **2.30 Hz** (negative: in band, no line) | 0.63–1.02 | 0.95–1.33 | 1.10–1.37 | 1.20–1.56 | — | — | — | **0.5–1.3 s** |
| **white noise** (synthetic, same windows) | 0.66 | 1.09 | 1.57 | 1.77 | 1.73 | 1.71 | 1.68 | 0.8 s |

The estimator orders the signals correctly: a6's 6.09 Hz line (ASD 6.4, the
strongest instance of the known-coherent line) is the most coherent thing
measured; 2.30 Hz, where there is no line, decorrelates in ~1 s; white noise
reaches the 1.81 rad asymptote by 4 s. **The method is not broken, and it is not
generous** — it puts the in-band lines *between* a real coherent line and
nothing.

It also puts a limit on the 6.19 Hz line's usefulness as a control: it is only
coherent in proportion to its SNR (a6, ASD 6.4 → τ₁ > 10 s; a4/a7, ASD 1.7/0.5 →
τ₁ ≈ 2 s). It is a line, not a clock.

---

## Q4 — what the canceller would actually remove *(load-bearing)*

Fraction of 0.4–3.0 Hz power removed from `ch*_bp`. **Positive = cancels,
negative = pumps.**

**Open loop** (47 windows, 972 s):

| K | Test | τ | Tapp | a0 | a1 | a2 | a3 |
|---|---|---|---|---|---|---|---|
| 1 | 4 s | 0.5 | 1 s | −0.026 | +0.228 | −0.031 | +0.347 |
| 1 | 8 s | 0.5 | 1 s | **+0.427** | **+0.551** | **+0.621** | **+0.622** |
| 1 | 12 s | 0.5 | 1 s | +0.396 | +0.552 | +0.585 | +0.622 |
| 3 | 8 s | 0.5 | 1 s | +0.607 | +0.700 | +0.820 | +0.774 |
| 3 | 8 s | 0.5 | 2 s | +0.512 | +0.652 | +0.775 | +0.716 |

**Closed loop, on top of the existing damper** (8 segments, 1206 s):

| K | Test | τ | Tapp | set | a0 | a1 | a2 | a3 |
|---|---|---|---|---|---|---|---|---|
| 1 | 8 s | 0.5 | 1 s | full | +0.096 | +0.027 | +0.049 | **−0.005** |
| 1 | 8 s | 0.5 | 1 s | 20 s | +0.102 | +0.130 | +0.043 | **−0.113** |
| 1 | 8 s | 0.5 | 4 s | 20 s | −0.057 | +0.000 | **−0.134** | **−0.136** |
| 3 | 8 s | 0.5 | 1 s | 20 s | −0.109 | −0.072 | **−0.271** | **−0.313** |
| 3 | 8 s | 0.5 | 4 s | 20 s | −0.266 | −0.189 | **−0.675** | **−0.448** |
| 3 | 16 s | 0.5 | 4 s | full | +0.114 | −0.034 | −0.040 | **−0.167** |

Three readings.

- **Open loop it works, and that is not the question.** 43–62 % of the power
  (2.4–4.2 dB) with one tone, 61–84 % (4.1–8 dB) with three. But the loop it
  would sit next to already achieves an in-band power ratio of **0.03–0.26**
  closed/open, i.e. **6–15 dB** (`analysis/oob_baseline.md`, Q2). Feedforward is
  strictly worse than the damper it would have to replace, using the same
  actuator authority and the same `VMAX`.
- **Added to the damper it does nothing**, and on a2/a3 it does harm. The
  residual after 6–15 dB of velocity damping is what is *left* when the
  predictable resonant response has been removed; there is nothing further to
  predict. `research.md` item 4's framing — asymptotic rejection of a persistent
  narrowband disturbance — assumes a disturbance that is still there after the
  loop. It is not.
- **The estimation window is the binding constraint, not the latency.** Going
  from `Test` = 4 s to 8 s is worth +0.30 to +0.45; going from τ = 0 to 0.5 s
  costs only 0.01–0.03. Feedforward's advertised advantage (no latency penalty)
  is not the thing that limits it. What limits it is needing 8 s of history to
  measure a line whose phase is already 0.8 rad wrong 4 s later.

**Quiet vs loud** (open loop, application intervals sorted by their own power,
bottom vs top quartile):

| ch | quiet 25 % | mid 50 % | loud 25 % |
|---|---|---|---|
| a0 | +0.475 | +0.501 | +0.398 |
| a1 | +0.403 | +0.549 | +0.555 |
| a2 | +0.528 | +0.626 | +0.622 |
| a3 | +0.415 | +0.536 | +0.645 |

Post-kick ringdowns are *slightly* more predictable than quiet ambient on a1–a3
(+0.14 to +0.23) and slightly less on a0. This is the one place the idea has a
future: a coherent ringdown is by construction a decaying sinusoid, and a
canceller sees it. It is also the case the existing damper handles best, so the
margin is the smallest where it matters least.

---

## Q5 — the line is not a line

Per-window lock-in amplitude, 4 s windows, open loop, counts:

| line | ch | 08-03 | 08-04 | 08-06 | IQR/median |
|---|---|---|---|---|---|
| 0.70 | a0 | 107.0 | 51.2 | 56.0 | 1.40 |
| 0.70 | a2 | 126.3 | 70.6 | 75.8 | 1.01 |
| 1.00 | a0 | 106.3 | 74.7 | 47.5 | 1.20 |
| 1.00 | a2 | 117.3 | 109.0 | 71.0 | 0.87 |
| 1.65 | a0 | 65.0 | 27.2 | 21.9 | 1.80 |
| 1.65 | a2 | 72.7 | 32.7 | 28.1 | 1.72 |

An IQR/median of 0.87–1.81 means the amplitude of "the line" changes by roughly
a factor of two between adjacent 4 s windows. Day to day it moves by 2–3×.

Peak frequency, one estimate per long segment (best available resolution,
0.004–0.017 Hz):

| segment | dur | 0.70 Hz | 1.00 Hz | 1.65 Hz |
|---|---|---|---|---|
| 20260803_160302 | 120 s | 0.5917 | 0.9500 | 1.7917 |
| 20260806_184216 | 67 s | 0.6582 | 0.9274 | 1.5408 |
| 20260806_184216 | 130 s | 0.6769 | 0.9923 | 1.6154 |
| 20260806_184723 | 175 s | 0.6636 | 0.9897 | 1.6018 |
| 20260806_185116 | 286 s | 0.7060 | 0.9577 | 1.6148 |
| 20260806_190046 | 61 s | 0.6667 | 0.9431 | 1.6748 |
| 20260806_190046 | 266 s | 0.6871 | 0.9949 | 1.6445 |
| 20260806_190046 | 100 s | 0.8096 | 0.9995 | 1.6392 |
| **spread** | | **0.218** | **0.072** | **0.251** |
| **as a fraction of f₀** | | **31 %** | **7.2 %** | **15 %** |

218 mHz of wander at 0.70 Hz is **6.4 half-power widths** of a Q = 21 resonance
(f₀/Q = 34 mHz). A fixed-frequency canceller is not merely detuned, it is
outside the feature. An adaptive one has to re-acquire a frequency that moves by
more than its own linewidth, from data whose phase is only good for ~1 s.

---

## Q6 — the plant, and why "anti-phase" is not well defined here

If someone builds it anyway, the canceller needs `H(f)` at the injection
frequency. `gains.json` (2026-08-06, `data/20260806_182231_tune_raw.csv`) has
15 driven tones — 0.3444, 0.4111, 0.4889, 0.5667, 0.6667, 0.7889, 0.9222,
1.0889, 1.2778, 1.5111, 1.7778, 2.0889, 2.4556, 2.8889, 3.4000 Hz — and a modal
fit `f₀ = 0.7149 Hz, Q = 288` and `f₀ = 0.9941 Hz, Q = 2334`.

**None of the tones is on a line.** The nearest is 0.6667 Hz, 33 mHz from the
0.70 Hz line — about one half-power width at Q = 21, three at Q = 288. This is
the same failure `analysis/mimo_design.md` §7 already documented: the tone
spacing near 1 Hz is ~167 mHz against a resonance width of 50 mHz, so the peak
was never sampled and the fitted Q is an interpolation, not a measurement.

Plant phase near the 0.7149 Hz mode, over the *measured* frequency wander of the
0.70 Hz line (0.5917 → 0.8096 Hz):

| assumed Q | phase at 0.5917 Hz | phase at 0.8096 Hz | swing |
|---|---|---|---|
| 21 (ringdown envelopes, `mimo_design.md`) | −7° | −169° | **162°** |
| 288 (multisine fit, `gains.json`) | −1° | −179° | **179°** |

**The plant phase swings through nearly 180° across the range the disturbance
frequency is observed to occupy.** Whichever Q is right, an injection computed
for one end of that range is in-phase — pumping — at the other. And Q itself is
contradicted 14× between the two measurements already on disk (20–22 from the
gain-off ringdown envelopes, 288/2334 from the multisine fit), which is a second,
independent sign ambiguity on top of the first.

---

## What would have to be true for this to work

For the record, if someone wants to revisit:

1. **A disturbance that survives the damper.** The whole case rests on the
   damper leaving something predictable behind, and measured on 1206 s of
   closed-loop data it does not (−0.11 to +0.13). Any revival has to start by
   showing a closed-loop residual with structure.
2. **f₀ and Q measured on the peak.** A 60 s gain-off ringdown fitted as a
   damped sinusoid gives both, costs no bench time
   (`analysis/out/decay_windows.csv` already holds 6 824 gain-off decay windows),
   and is `mimo_design.md` §7's own recommendation. Without it the injection sign
   is a coin flip.
3. **A phase error budget under 0.5 rad at the canceller's update period.**
   Measured: 0.33–0.48 rad at 0.5 s. That is the whole margin, before plant
   phase error, before actuator latency, before the 7–31 % frequency wander.
4. Even granting all three, the ceiling from Q4 is 2.4–4.2 dB against a damper
   already delivering 6–15 dB with the same actuator.

**What to do instead.** Nothing here argues against `research.md` item 4's
*other* half — a resonant / internal-model term inside the existing feedback
loop. That does not need the disturbance to be predictable, only the plant to be
known, and it degrades gracefully when f₀ drifts. The measurement it needs is the
same one item 2 above needs: a ringdown.

## Files

- `analysis/passive_phase.py` — regenerates every number above.
- Related: `analysis/mimo_design.md` §7 (why the multisine cannot give f₀/Q),
  `analysis/oob_baseline.md` Q2 (the 6–15 dB the damper already delivers),
  `analysis/where_is_power.py` (the 6.19 Hz line), `research.md` item 4.
