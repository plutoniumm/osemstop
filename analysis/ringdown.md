# f0 and Q from ringdowns

Two analyses, in order of authority:

- `analysis/ringdown_quiet.py` — the **320 s quiet open-loop record**,
  `data/20260806_192723_quiet_openloop.csv`, taken 2026-08-06 19:27. This is
  the measurement. Tables in `analysis/out/quiet_*.csv`.
- `analysis/ringdown.py` — everything that was on disk before it,
  `data/*_fast_lock.csv` and `analysis/out/decay_windows.csv`. Tables in
  `analysis/out/ringdown_*.csv`. Kept because it is what traced the wrong
  numbers to their sources.

Offline throughout. No serial port opened.

---

## Verdict

**f0 is settled to 0.0005 Hz. Q is still a bound, but the bound has moved by a
factor of four and it has changed which claims survive: Q = 100 is now excluded
too, and Q = 2334 is the only prior claim left standing.**

| | answer | source |
|---|---|---|
| **Modes** | **three** | `quiet_frequencies.csv` |
| **f_A** | **0.7155 ± 0.0005 Hz** | `quiet_frequencies.csv`, `quiet_freq_stability.csv` |
| **f_B** | **0.9949 ± 0.0005 Hz** | ditto |
| **f_C** | **1.6396 ± 0.0005 Hz** | ditto |
| **gamma** | **+0.00004 ± 0.00719 1/s** — consistent with zero | `quiet_verdict.csv` |
| **Q** | **> 433 (1σ), > 217 (2σ).** Still a bound, not a value | `quiet_verdict.csv`, `quiet_calibration.csv` |
| **tau** | **> 138 s** at 1σ | 1/(gamma+sd) |
| **largest Q this record can resolve** | **540** by coherence, **183** by linewidth | `quiet_calibration.csv`, `quiet_linewidth.csv` |

| prior claim | verdict | the number that decides it |
|---|---|---|
| Q = 20-22 (`mimo_design.md:677`) | **dead**, and never was an open-loop measurement | 21.7 sigma; all 73 windows behind it had the other 3 channels damping |
| Q ~ 50 (`research.md:105`, `sim/server.py:294`) | **dead** | 8.7 sigma |
| Q = 100 (the bound in the previous version of this file) | **now excluded too** | 4.3 sigma |
| Q = 288 at 0.7149 Hz (`tune.py fit`) | **disfavoured, not excluded** | 1.5 sigma; and `quiet_linewidth.csv` puts mode A at Q ≤ 451 |
| Q = 2334 at 0.9941 Hz (`tune.py fit`) | **survives.** Compatible, still not established | 0.2 sigma |
| f = 0.7149 / 0.9941 Hz (multisine) | **right, to 0.0005 / 0.0008 Hz** | `quiet_frequencies.csv` |
| f = 0.7049 / 0.9967 Hz (previous version of this file) | **mode A was wrong by 0.0101 Hz** — the same bin disease it accused others of | 18 s records on a 0.0139 Hz grid vs 276 s on 0.00083 Hz |
| 1.046 / 1.657 Hz (earlier run) | **bin artefacts of the right lines** | both sit on the 0.08717 Hz FFT grid of `modes.csv` (357.14 Hz / 4096) |

**What the 320 s record did and did not do.** It did not turn Q into a number.
It moved the ceiling of what is measurable from ~264 to ~540 and pushed the
floor from Q > 100 to Q > 433, which is enough to kill Q = 100 and to leave
Q = 2334 as the last claim standing. Reporting a value for Q from it would be
the same error this file was written to correct.

---

## 0. The 320 s quiet record

`data/20260806_192723_quiet_openloop.csv`, 355797 samples, 320.0 s, 1111.87 Hz
effective, coils held at BIAS = 0.25 V and written 8 times total. First 20.0 s
discarded as settle, leaving 300.0 s; the band-shaping filter costs 12 s at each
end, so **276.0 s** are actually fitted.

- **1 sample rejected**: a `72647` on ch7 (a serial glitch; the ADC is 10-bit).
  One such sample left in a least-squares fit would have dominated it.
- **0 samples on an ADC rail**, on any channel. Nothing clipped, which is the
  whole point of taking the record with no drive.
- dt median 0.8900 ms, sd 0.2393 ms, monotonic.
- Frequency resolution 1/T = **0.00333 Hz**, 4.2x finer than anything used
  before it.

### 0.1 The record is not stationary — and the decay is not the optic

Block RMS over 30 s blocks falls monotonically on every channel that carries
motion (`quiet_blockrms.csv`): ch0 87.8 → 54.3, ch1 34.0 → 20.7, ch2 117.4 →
68.1, ch3 29.7 → 16.1 counts over 270 s. Implied rates +0.00178 to +0.00226 1/s.
Read naively that is a ringdown with tau ≈ 470 s and Q ≈ 1500, and
`quiet_direct.csv` fits exactly that, with r² of 0.91-0.93.

It is not the optic. **The out-of-band control kills it**
(`quiet_controls.csv`): the 2.4-3.6 Hz band, which contains no mode, falls at
+0.00384, +0.00204, +0.00428, +0.00268 1/s on ch0-ch3 — the same rate as the
modes, or faster. The ambient level dropped by ~40% over the five minutes and
everything followed it down. The split-half control agrees: mode B's fitted
decay rate is 1.9-2.0x faster in the second half than the first, which no single
exponential does.

So the amplitude-based reading of Q from this record is void, and the
`quiet_direct.csv` numbers (Q = 1435-1671 for mode B) are a measurement of the
lab, not the suspension. They are kept in the output as the thing that had to be
ruled out.

---

## 0b. What replaces it: coherence, not amplitude

`analysis/ringdown_quiet.py` §4. Narrowband each mode to ±0.060 Hz (raised
cosine, **absolute** 0.030 Hz transitions — with proportional transitions a
band centred on mode A still passes mode B at 0.40 of amplitude and the modes
are not separated at all), take the analytic signal, and form

    rho(tau) = |<z*(t) z(t+tau)>| / rms(t) rms(t+tau)

Dividing by the two local RMS values cancels any smooth common amplitude trend,
so rho is blind to exactly the thing §0.1 showed is contaminating the envelope.
What is left decays as exp(-gamma·tau).

**Blindness check** (`quiet_blindness.csv`): the same statistic on a simulated
stationary resonance, with and without the record's own 0.0021 1/s trend
multiplied in — Q = 100 gives 0.0410 vs 0.0312, Q = 300 gives 0.0106 vs 0.0094.
The trend does not move it.

**Fit range is adaptive**: lag 0 to whichever comes first, 90 s or the lag at
which rho first falls below 0.35. rho does not decay to zero — with N_eff
independent samples it flattens near 1/sqrt(N_eff) — so a fixed range measures
the approach to that floor and reads every low Q back as a high one. That bug
was in the first version of this analysis and made Q = 30 come back as Q = 291.

### Result

`quiet_coherence.csv`, four sensors, per mode:

| mode | f | rho(90 s), ch0-ch3 | gamma |
|---|---|---|---|
| A | 0.7155 Hz | 0.958, 0.999, 0.989, 0.999 | -0.00001 |
| B | 0.9949 Hz | 0.997, 0.997, 0.997, 0.997 | +0.00004 |
| C | 1.6396 Hz | 0.956, 0.999, 0.994, 0.999 | -0.00000 |

**The dead channels are the control that makes this readable.** ch4, ch6 and ch7
carry 5-6 counts RMS, i.e. sensor noise. Their rho falls at gamma = 0.171-0.209
1/s — which is the 0.12 Hz analysis band's own decorrelation rate, exactly what
uncorrelated noise must give. ch0-ch3 do not. The coherence is in the signal,
not in the pipeline.

### Calibration, and therefore the uncertainty

`quiet_calibration.csv`: known Q in, recovered gamma out, 276 s records, 24
realisations each, identical pipeline, at the in-band SNR measured from the
record itself (median 109, from the mode band against an equally wide mode-free
band at 2.96 Hz).

| Q true | gamma true | gamma recovered | Q recovered |
|---|---|---|---|
| 30 | 0.10419 | 0.08659 ± 0.03583 | 36 |
| 60 | 0.05209 | 0.05026 ± 0.02577 | 62 |
| 100 | 0.03126 | 0.03175 ± 0.01902 | 98 |
| 200 | 0.01563 | 0.01641 ± 0.01529 | 190 |
| 300 | 0.01042 | 0.01180 ± 0.01269 | 265 |
| 500 | 0.00625 | 0.00871 ± 0.00540 | 359 |
| 1000 | 0.00313 | 0.00667 ± 0.00525 | 468 |
| 2334 | 0.00134 | 0.00587 ± 0.00582 | 532 |
| **infinite** | **0.00000** | **0.00579 ± 0.00719** | **540** |

Faithful to 5% up to Q = 100, to 20% at Q = 300, and saturating above that.

**New noise floor for a 300 s window: gamma = +0.00579 ± 0.00719 1/s at
gamma_true = 0.** (The old, 48 s figure was +0.0072 ± 0.0023.) Nothing below
0.0202 1/s is distinguishable from zero at 2σ, so **the largest Q this record
can resolve is 540**, and the measured gamma = +0.00004 ± 0.00719 sits below
the floor.

| mode | gamma | tau | Q |
|---|---|---|---|
| A | -0.00001 ± 0.00719 | — | **> 313 (1σ)** |
| B | +0.00004 ± 0.00719 | > 138 s | **> 433 (1σ), > 217 (2σ)** |
| C | -0.00000 ± 0.00719 | — | **> 717 (1σ)** |

Every prior claim as a sigma distance from mode B's measured gamma
(`quiet_verdict.csv`): Q = 20-22 → **21.7σ**; Q = 50 → **8.7σ**; Q = 100 →
**4.3σ**; Q = 288 → **1.5σ**; Q = 2334 → **0.2σ**.

### Independent confirmation: linewidth

`quiet_linewidth.csv`, §4b — no coherence machinery involved. Compare each
mode's −3 dB width against a pure tone through the identical window and
zero-padding (reference width 0.00543 Hz, Hann theory 1.44/T = 0.00522 Hz):

- **mode B: 0.00543 Hz on all four channels — identical to the pure tone.
  Instrument-limited. No excess width at all.**
- **mode C: 0.00543 Hz. Instrument-limited.**
- mode A: 0.00566 Hz, excess 0.00159 Hz → **Q ≤ 451**. One padded-grid element,
  so marginal, but it is the only mode showing any width at all and it is the
  mode `tune.py` assigned Q = 288 to.

This ceiling is f/(1.44/T) = **183**, so the linewidth test can only say Q > 183.
It agrees with the coherence result and rules out nothing the coherence result
does not already rule out — which is the point of running it.

### Frequencies, and the bin disease in my own previous numbers

`quiet_frequencies.csv`, 276 s Hann periodogram, zero-padded 16x, peak refined
by a 3-point parabolic fit so the answer is not itself a bin. All four channels
land on the same refined position:

| mode | f (this record) | previous value here | shift | multisine |
|---|---|---|---|---|
| A | **0.7155** | 0.7049 ± 0.0057 | **+0.0106** | 0.7149 |
| B | **0.9949** | 0.9967 ± 0.0031 | -0.0018 | 0.9941 |
| C | **1.6396** | 1.6404 ± 0.0006 | -0.0008 | — |

**Mode A's previous value was wrong by 0.0106 Hz, 1.9x its own quoted error.**
It came from 18 s records on a 0.0139 Hz zero-padded grid — the same bin
limitation this file diagnosed in `modes.csv`. The multisine's 0.7149 was right
and this file's 0.7049 was not. `quiet_freq_stability.csv`: first half vs second
half of the record agree to **±0.0001 Hz** on all three modes, so ±0.0005 Hz is
a conservative error bar.

### Beat periods, measured

`ringdown_quiet.py` §6, from the frequency differences:

| pair | df | beat period |
|---|---|---|
| A-B | 0.2795 Hz | 3.578 s |
| **B-C** | **0.6446 Hz** | **1.551 s** |
| A-C | 0.9241 Hz | 1.082 s |

The 1.56 s figure quoted earlier was the B-C beat and is confirmed at 1.551 s.
It is still shorter than the 8 s envelope window `kp040.py` used, which is why
those envelopes were not exponential.

---

## The rest of this file

Everything below predates the 320 s record and comes from
`analysis/ringdown.py`. Its frequencies are superseded by the table above. Its
exclusions (Q = 20-22, Q = 50) are confirmed and strengthened by it. Its
tracing of where the wrong numbers came from is unaffected and is the part
worth keeping.

---

## 1. What was actually available

`ringdown_inventory.csv`, from all 30 `data/*_fast_lock.csv`:

- **101 contiguous stretches with every channel's `chN_gain` identically 0.0**,
  **1360.3 s** in total.
- Longest single record: **48.75 s** (`20260803_155916_fast_lock.csv`, FAULT),
  then 47.19 s, 29.87 s, and 45 records ≥ 18 s.
- Gain-off was taken from the `chN_gain` column, not the state label. During
  these stretches `chN_out` is a constant (0.25 / 0.50 / 0.25 / 1.00 V observed
  at t = 95-100 s of `20260806_184216_fast_lock.csv`) — a static force, which
  cannot damp.
- Ambient RMS floor in the 0.25-4 Hz band, from each run's *opening*
  calibration: ch0 61.9, ch1 34.2, ch2 78.8, ch3 26.6, ch5 18.9 counts. ch4,
  ch6, ch7 sit at 0.63, 0.68, 0.52 counts — under one ADC count, i.e. no signal.

**The resolution ceiling this implies, before any fitting.** A record of length
T cannot resolve a linewidth narrower than ~1/T:

| claim | implied FWHM | record needed | have 48.75 s? |
|---|---|---|---|
| Q = 20 at 1.0 Hz | 0.0500 Hz | 20.0 s | yes |
| Q = 50 at 1.0 Hz | 0.0200 Hz | 50.0 s | no |
| Q = 288 at 0.7149 Hz | 0.00248 Hz | 402.9 s | no |
| Q = 2334 at 0.9941 Hz | 0.00043 Hz | 2347.9 s | no |

So a *positive* measurement of any Q above ~50 was never on the table here.
What is on the table is an exclusion, and that is what §3.2 delivers.

---

## 2. Frequencies — settled, and there are three of them

Coherently averaged periodogram over every gain-off record ≥ 18 s, Hann
windowed, zero-padded 4x (bin 0.0139 Hz). `ringdown_peaks.csv`:

| ch | n | peaks (relative power) |
|---|---|---|
| ch0 | 33 | 1.0004 (1.00), 0.7086 (0.90), 1.6395 (0.28) |
| ch1 | 43 | 1.6395 (1.00), 0.7086 (0.46), 0.9865 (0.39) |
| ch2 | 25 | 1.0004 (1.00), 0.7086 (0.77) |
| ch3 | 41 | 1.6395 (1.00), 1.0004 (0.41), 0.7086 (0.40) |
| ch5 | 17 | 1.0004 (1.00) |

The continuous estimates come from the K = 3 damped-sinusoid fit to the
autocorrelation (§3), taking each mode's frequency as seen through every
channel whose fit explains the ACF (r² > 0.95):

| mode | f | channels |
|---|---|---|
| A | **0.7049 ± 0.0057 Hz** | ch0, ch1, ch2, ch3 |
| B | **0.9967 ± 0.0031 Hz** | ch0, ch1, ch2, ch3, ch5 |
| C | **1.6404 ± 0.0006 Hz** | ch0, ch1, ch2, ch3 |

Three checks that these are mechanical and not instrumentation:

1. **They partition by channel.** 0.9967 Hz dominates ch0/ch2/ch5, 1.6404 Hz
   dominates ch1/ch3. A DAQ or ground-loop artefact appears with the same weight
   everywhere; a mode shape does not.
2. **Same frequencies on all three days**, 2026-08-03 / 04 / 06, to within one
   zero-pad bin.
3. **Same frequencies across a 3.11x change in raw sample rate**, 357.14 Hz on
   08-03/04 and 1111.11 Hz on 08-06. A line locked to the ADC cadence would move.

**Two components or one?** Two is not enough; three is right.
- Transient fits: BIC prefers K = 2 over K = 1 in **40 of 43** fits
  (`ringdown_transients.csv`), median r² 0.266 → 0.670.
- ACF fits: r²(K=1) → r²(K=2) → r²(K=3) is 0.592 → 0.958 → **0.998** on ch0,
  0.683 → 0.705 → **1.000** on ch2, 0.744 → 0.879 → **0.998** on ch3. Three
  damped sinusoids explain the open-loop autocorrelation essentially exactly.

The envelope fits in `decay_windows.csv` could not have found this: an envelope
discards the frequency, and a beat between 0.9967 and 1.6404 Hz (period 1.56 s)
lives inside an 8 s window, which is exactly what made those envelopes
non-exponential (median r² **0.046** on the gain-off windows).

---

## 3. Q — the honest answer is a bound

### 3.1 The model-free statement

Stack the unbiased autocorrelation of every gain-off record ≥ 24 s and read
R(12 s)/R(0). For a decay at rate gamma this ratio is exp(-12·gamma), no
fitting involved. `ringdown_longlag.csv`:

| ch | records | s | R(12 s)/R(0) |
|---|---|---|---|
| ch0 | 11 | 279.9 | **1.033 ± 0.082** |
| ch1 | 16 | 428.9 | **1.081 ± 0.163** |
| ch2 | 7 | 175.1 | **1.120 ± 0.080** |
| ch3 | 15 | 380.2 | **1.119 ± 0.144** |
| ch5 | 7 | 175.1 | 0.831 ± 0.121 |
| ch4 / ch6 / ch7 | 8 / 5 / 8 | | 0.108 / 0.434 / 0.361 |

Errors are leave-one-record-out jackknife. **The four original OSEMs show no
decay at all over 12 s of lag.** ch4/ch6/ch7 do decay, which is the expected
signature of a channel whose entire signal is sub-count noise (ambient RMS
0.5-0.7 counts): uncorrelated noise has an ACF that collapses immediately.

A true envelope ratio cannot exceed 1, so the 0.120 excess on ch2 is this
statistic's own positive bias. Everything below subtracts **all** of it from
every channel, which is the conservative choice.

### 3.2 Every claimed Q, turned into a prediction and tested

`ringdown_exclusion.csv`, at f = 0.9967 Hz, sigma after the 0.120 bias
subtraction:

| Q | implied gamma | predicted R(12 s)/R(0) | ch0 | ch1 | ch2 | ch3 |
|---|---|---|---|---|---|---|
| 20 | 0.1566 | 0.153 | 9.2 | 5.0 | 10.6 | 5.9 |
| 22 | 0.1423 | 0.181 | 8.9 | 4.8 | 10.2 | 5.7 |
| **50** | 0.0626 | 0.472 | **5.4** | **3.0** | **6.6** | **3.7** |
| **100** | 0.0313 | 0.687 | **2.8** | 1.7 | **3.9** | **2.2** |
| 200 | 0.0157 | 0.829 | 1.0 | 0.8 | 2.1 | 1.2 |
| 288 | 0.0109 | 0.878 | 0.4 | 0.5 | 1.5 | 0.8 |
| 500 | 0.0063 | 0.928 | -0.2 | 0.2 | 0.9 | 0.5 |
| 2334 | 0.0013 | 0.984 | -0.9 | -0.1 | 0.2 | 0.1 |

- **Q = 20-22 is excluded at 4.8-10.6 sigma.**
- **Q = 50 is excluded at 3.0-6.6 sigma.**
- **Q = 100 is disfavoured** at 2.2-3.9 sigma on three of four channels.
- Q ≥ 200 is compatible with the data, and **Q = 200, 288, 500 and 2334 cannot
  be told apart.**

One loophole, closed only partly: the long-lag ratio is dominated by the
slowest-decaying component, so a fast mode could in principle hide behind a slow
one. It cannot hide far. From the ch0 peak census the three modes carry 0.459 /
0.413 / 0.128 of the in-band power, so a Q = 50 mode among the two dominant ones
would drag the ratio down by ≥ 0.41 × (1 - 0.472) = 0.22, which is 2.6 sigma
against the 0.082 error. For the weakest of the three the same argument only
reaches ~1 sigma, so **mode A at 0.7049 Hz is the one whose Q is least
constrained.**

### 3.3 Why the direct transient fits do not settle it

43 post-kick transients were fitted directly (`ringdown_transients.csv`,
selection: leading amplitude > 3x the channel's ambient RMS, fit truncated when
the envelope falls below 2x ambient, any channel with > 0.1% of samples on an
ADC rail discarded). Pooled by frequency band:

| band | n | f | gamma | Q |
|---|---|---|---|---|
| 0.25-0.85 Hz | 10 | 0.7274 | +0.0467 [+0.0230, +0.1307] | 49 |
| 0.85-1.35 Hz | 11 | 0.9945 | +0.0475 [+0.0245, +0.1212] | 66 |
| 1.35-2.10 Hz | 22 | 1.6405 | +0.0290 [+0.0255, +0.0488] | 178 |

Those numbers look like a measurement and are not one. Two controls:

- **Null control** (`ringdown_null.csv`). Run the identical selection and fit on
  each run's *opening* calibration — stationary ambient, nothing has ever kicked
  the optic and no feedback has ever been applied, so the true gamma there is
  zero. Result over 178 windows: **gamma = +0.0168 [+0.0114, +0.0212] 1/s**.
  Selecting windows on a large starting amplitude manufactures most of the
  apparent decay. Subtracting it leaves 0.012-0.031 1/s, which no longer excludes
  zero.
- **Early vs late halves of the same record**: gamma = **+0.0125** in the first
  half, **-0.0001** in the second (`ringdown_amplitude.csv`, n = 20 each). The
  "decay" is confined to the part of the record the selection rule looked at.

### 3.4 The same mode, seen through different sensors, should give the same gamma

It does not. `ringdown_mode_summary.csv`, from the per-channel K = 3 ACF fits:

| mode | f | gamma per channel | spread |
|---|---|---|---|
| A | 0.7049 Hz | +0.0207, +0.1388, +0.0107, +0.0760 | **13x** |
| B | 0.9967 Hz | +0.0032, +0.0579, +0.0005, +0.0511, +0.0082 | **115x** |
| C | 1.6404 Hz | +0.0530, +0.0061, +0.0106, +0.0083 | **9x** |

Frequency agrees across sensors to 0.04-0.8%. gamma disagrees by an order of
magnitude. gamma is a property of the mode, so this is the estimator running out
of information, not physics.

### 3.5 What the estimator can and cannot see

`ringdown_recovery.csv`: a single complex pole driven by white noise, at
f0 = 0.9990 Hz, cut into the 45 record lengths actually on disk, 8 realisations
per point, pushed through the same ACF fit.

| gamma_true | Q_true | gamma recovered | bias |
|---|---|---|---|
| 0.0000 | inf | 0.0072 ± 0.0023 | — |
| 0.0050 | 628 | 0.0092 ± 0.0031 | +83% |
| 0.0100 | 314 | 0.0124 ± 0.0030 | +24% |
| 0.0200 | 157 | 0.0223 ± 0.0079 | +12% |
| 0.0400 | 78 | 0.0473 ± 0.0131 | +18% |
| 0.0630 | 50 | 0.0642 ± 0.0109 | +2% |
| 0.1000 | 31 | 0.0938 ± 0.0166 | -6% |
| 0.2500 | 13 | 0.2337 ± 0.0455 | -7% |

**With gamma_true = 0 the estimator still returns +0.0072 ± 0.0023.** Nothing
below ~0.012 1/s is distinguishable from zero with these record lengths, so
**Q above ~264 at 1 Hz is not measurable from this data at all** — which is
precisely why 288 and 2334 come out compatible above. Above gamma = 0.02 the
estimator is good to 2-24%, which is why the Q ≤ 100 exclusions are trustworthy.

### 3.6 Amplitude dependence — cannot be tested here

Section 5 of `ringdown.py`'s output splits the transients by starting
amplitude. Only two bins are populated at all — A0/ambient 3-6 with n = 43 and
gamma +0.0307 [+0.0255, +0.0475], and 6-12 with **n = 4** and gamma +0.0971
[+0.0103, +0.1851]. Two points, one of which rests on four fits, is not a trend,
and the first bin's excess over the null (0.0307 - 0.0168 = 0.014) is well
inside the null's own IQR of -0.0078 to +0.0826. **No statement about nonlinear
damping can be made from this data**, so nothing here bears on the `-0.040`
instability in `analysis/kp040.md` either way. Getting a
real answer needs a kick large enough to give the envelope a decade of range,
which the 1023-count ADC will not permit against a 62-count ambient (ch0 rests
at ~590 counts, so ~430 counts of swing, i.e. 7x ambient, before railing).

---

## 4. Where the wrong numbers came from

### 4.1 Q = 20-22 — measured with the loop closed

`mimo_design.md:677` reads "gamma = 0.149-0.166 /s at zero gain ⟹ Q = pi·f/gamma
= 20-22", from the `|gain| = 0.0` rows of `analysis/out/damping_vs_gain.csv`.
Traced in section 6 of the run:

- Those rows are **73 windows**, all in state DAMPING, selected by kp040's
  filters (r² > 0.80, envelope falls by > 10%, gain constant). Median gamma
  **+0.1575 1/s**, giving Q = 19.9. That reproduces the claim exactly.
- **All 73 of them had the other three channels driven**, at \|gain\| up to
  0.035. So did **all 813** gain-off DAMPING windows in the file. All eight
  OSEMs sit on **one rigid body** (`CLAUDE.md` §2) — zeroing one channel's gain
  does not free the body while the other three damp it. 0.1575 1/s is the
  *closed-loop* decay of the body seen through an undriven sensor.
- Applying the same filters to windows that genuinely are open loop —
  CALIBRATING and FAULT, where all four gains are zero — leaves **0 of 1188**
  and **0 of 236** survivors. The open-loop number was never computed from
  open-loop data.
- The selection is severe in its own right: of the 2237 gain-off windows in the
  file only 74 (3.3%) reach r² > 0.80, the median gain-off r² is 0.046, and
  40% of the CALIBRATING windows fit a *negative* gamma.

`kp040.py` itself prints "the open-loop tau (hence Q) IS NOT MEASURABLE from
these logs" a few lines before writing the file. The caveat did not travel with
the number.

### 4.2 Q ~ 50 — an assumption, now excluded

`research.md:105` states Q = 50, f0 ~ 1 Hz, tau = Q/(pi·f0) ~ 16 s.
`sim/server.py:294` says outright that it is an assumption: "with the
simulator's Q = 50 (the report measures no Q, so this part inherits that
assumption)". f0 ~ 1 Hz is right to 0.3%. Q = 50 is excluded at 3.0-6.6 sigma.

### 4.3 Q = 288 and Q = 2334 — right lines, unsupported widths

*(Superseded in part by the 320 s record; see the top of this file. The
frequencies turned out to be better than the values in this section, and mode
A's Q = 288 is now disfavoured at 1.5 sigma while Q = 2334 survives at 0.2.)*

The frequencies are real: 0.7149 vs the measured 0.7049 ± 0.0057 (1.8 sigma) and
0.9941 vs 0.9967 ± 0.0031 (0.8 sigma). The multisine found modes A and B and
missed C at 1.6404 Hz.

The Q values remain unsupported for the reason already given — 0.167 Hz tone
spacing against a claimed 0.0025 Hz linewidth — but **this analysis does not
refute them**: at 0.4-1.5 sigma they sit comfortably inside the data. Reporting
them as measurements is still wrong; reporting them as excluded would also be
wrong.

### 4.4 1.046 and 1.657 Hz — one FFT bin off

`analysis/out/modes.csv` lists peaks at 0.698, 0.785, 1.046, 1.657, 2.005,
2.093, 2.529, 2.790, 3.401, 3.749, 4.360, 4.447 Hz. Every one is an integer
multiple of **0.08717 Hz**, which is `kp040.py`'s Welch bin, 357.1428 Hz / 4096.
1.046 = bin 12 and 1.657 = bin 19. The true lines 0.9949 and 1.6396 fall at bins
11.4 and 18.8, so both were rounded up one bin. The 0.698 and 0.785 entries
straddle mode A at 0.7155 — bin 8 is 0.6975 and bin 9 is 0.7847.

---

## 5. What this changes

Updated for the 320 s record. Where a number moved, the old one is shown
struck through in words.

### 5.1 `CALIBRATION_S = 20 s`

The stated justification (`research.md:105-106`) is "Cause is **ringdown** —
Q = 50, f0 ~ 1 Hz, tau = Q/(pi·f0) ~ **16 s** — so median-of-sub-windows helps
only when the window is comparable to tau".

**tau > 138 s at 1σ** (was "> 32 s" before the quiet record). The 16 s figure is
wrong by **at least 8.6x**, and 20 s is at most **0.14 tau**, not the
"comparable to tau" the argument needs. The premise is false by an order of
magnitude, not by a factor of two.

This still does **not** license shortening the window — it removes the stated
reason for the length without supplying a replacement, and the direction of the
error is toward the optic staying dirty for *longer*, not shorter. What it does
mean concretely:

- The empirical table under that sentence (worst-channel skew 74.3% at 5 s down
  to 20.1% at 20 s) is a *simulator* result, and the simulator runs
  `MODE_F0 = [1.000, 0.940, 1.070]` at Q = 50: three modes inside 7% of each
  other, decaying **at least 8.6x too fast**. The real optic has modes at
  0.7155, 0.9949 and 1.6396 Hz, spread 2.3x, beating at 1.551 s.
- With tau > 138 s, a shock inside a 20 s window has decayed by at most 13.4%
  before the window ends. Median-of-sub-windows cannot average it away, which
  is consistent with `research.md` recording that `fast-calib`'s agreement
  ratio of 1.20 **never fired on the bench** — the sub-windows do not agree
  because the ringdown is still there in all of them.
- **Re-derive the window from the measured modes, and do not tune it in the
  simulator until `MODE_F0` and Q there are corrected.** "Scale the ceiling off
  Q" remains the right instinct and remains impossible: Q is still a bound.

### 5.2 Intrinsic damping used to predict added damping

`sim/server.py:294-303` derives `COIL_GAIN` by subtracting
`gamma_int = w0/2Q = 0.0628 1/s` from measured total damping rates. The measured
intrinsic gamma is **+0.00004 ± 0.00719 1/s** — consistent with zero, and
smaller than 0.0628 by at least a factor of 9 even at the 1σ upper edge. The
right number to subtract is **0.000**, with an uncertainty of 0.007.

| ch | \|K\| | total | added, gamma_int = 0.0628 | added, gamma_int = 0.000 | change |
|---|---|---|---|---|---|
| A0 | 0.030 | 0.2197 | 0.1569 | **0.2197** | +40% |
| A1 | 0.030 | 0.1821 | 0.1193 | **0.1821** | +53% |
| A2 | 0.010 | 0.2796 | 0.2168 | **0.2796** | +29% |
| A3 | 0.030 | 0.1982 | 0.1354 | **0.1982** | +46% |

So **every absolute added-damping prediction in the repo is 29-53% low** (the
previous version of this file said 23-43%, using the then-current bound rather
than a measurement).

The *ratios*, which is all `COIL_GAIN` actually encodes, move much less:

    COIL_GAIN = [1.00, 0.76, -4.15, 0.86]     as written
              → [1.00, 0.83, -3.82, 0.90]     with gamma_int = 0

a 5-9% shift. The sign on ch2 and its ~4x authority per volt both survive
unchanged. The `sim/server.py:294` comment "the report measures no Q, so this
part inherits that assumption" can now be replaced with a measurement, and the
`R^2 = 0.65..0.92` caveat under it is still the binding limit on precision.

### 5.3 Simulator mode frequencies

`MODE_F0 = [1.000, 0.940, 1.070]` (4-OSEM body) and `MODE_F0_8 = [1.046, 1.657]`
(8-OSEM body) should both be **[0.7155, 0.9949, 1.6396]**, and the simulator's
Q = 50 should be **at least 433**.

The frequency error is structural, not cosmetic. The simulator's three modes span
7%; the real three span 2.3x. Anything tuned against a single 1 Hz cluster —
the bandpass corners `BP_LOW_HZ, BP_HIGH_HZ = 0.4, 3.0`, the calibration
sub-window logic, the modal/MIMO work in `mimo_design.md` — has been validated
against a plant whose modal structure the bench does not have. `MODE_F0_8` is
additionally the bin-quantised pair from §4.4 and is missing mode A entirely.

### 5.4 What would settle Q, now that Q > 433

The 320 s record moved the bound from 100 to 433 and the measurable ceiling from
264 to 540. It did not produce a value. What remains:

- **Separating Q = 288 from Q = 2334** is the live question, and it is close.
  At Q = 300 the estimator's own single-record scatter is **±0.0127 1/s**
  against a signal of 0.0104 1/s (`quiet_calibration.csv`) — under 1σ per
  record. Averaging N independent 300 s records cuts that as sqrt(N), so
  **13 repeats of the same five-minute run, about 65 minutes total**, reaches
  2.95σ. That is the cheapest path and it needs no new code: the same file format,
  the same script.
- **Resolving Q = 2334 as a linewidth** needs FWHM 0.000426 Hz, i.e. a single
  coherent record of **T > 2350 s (39 minutes)**. Only worth it if the repeats
  above come back saying Q is genuinely in the thousands.
- **Still no kick, still no drive.** The 320 s record had 0 railed samples on 8
  channels, and the one thing that went wrong with it — a 40% ambient drop
  mid-record — was handled by using a statistic blind to amplitude. Repeats
  average that away as well.

Until those repeats exist, use **Q > 433, tau > 138 s** and treat any value as
unmeasured.

## Reproduce

```
.venv/bin/python analysis/ringdown_quiet.py     # ~30 s, the 320 s record
.venv/bin/python analysis/ringdown.py --cache   # ~2 min, 800 MB of CSV -> npz
.venv/bin/python analysis/ringdown.py           # ~18 s from the cache
```
