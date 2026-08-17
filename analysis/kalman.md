# A Kalman velocity estimator for the OSEM damping loop

> **CAVEAT ADDED 2026-08-17: every frequency in this file is stale, and this file
> is the one most exposed to that.** The estimator is three undamped oscillators
> at 0.7155 / 0.9949 / 1.6396 Hz (§ *The model*), and those frequencies moved
> between 2026-08-06 and 2026-08-17 by **+0.00750 / −0.00300 / +0.01700 Hz** —
> 9.1 / 2.6 / 9.0 half-widths at Q = 433. Re-measured from
> `data/20260817_172150_status_sensors.csv`; `versions.md`
> § *On the bench, 2026-08-17* § 2.
>
> **By this file's own arithmetic** (§ 7a: a fixed frequency error accrues
> `360 × Δf` °/s), offsets of 0.0030–0.0170 Hz reach 90° in **15–83 s**, against
> the **2146–4065 s** § 7a quotes for the within-record drift the design was sized
> against — a factor of 26–277. **What that does to this filter's steady-state
> error, as opposed to an open-loop phase-matcher's accumulating error, is not
> derived here and is not established.** The design is not invalidated; its
> central premise — "the frequencies are already known" — now needs re-measuring
> each session rather than reading off a constant, and `F_MODE_HZ` in
> `osem.epsilon.py:163` must be updated before the filter goes on the bench.

Design and offline validation. Nothing here was run on hardware, no serial port
was opened, and no controller was executed. Everything is replay of records
already on disk, in the way `sample-guard` and `decimate` were designed and
validated.

- Code and every table: `analysis/kalman.py` → `analysis/out/kalman_*.csv`.
  Re-runnable with `.venv/bin/python analysis/kalman.py`, about three minutes.
- The shipping controller is `osem.delta.py` (formerly `osem.v12.py`). Its
  control law and every filter constant quoted here are unchanged between the
  two names — checked, not assumed.

---

## Verdict

**Build it.** Three measured results, in descending order of how much they
matter.

| | delta, as shipped | this estimator | source |
|---|---|---|---|
| **Velocity error against a known truth**, steady state, at kick amplitude | **29.3 %** | **0.6 %** | `kalman_sweep_synthetic.csv` |
| …at ambient amplitude | 33.8 % | 7.0 % | ditto |
| **Dissipative fraction of Kp** at mode C | 0.6306 | 1.0000 | `kalman_dissipation.csv` |
| **Residual against a zero-phase reference**, quiet record, a1 | 0.401 | **0.055** | `kalman_replay_quiet.csv` |
| Cost, eight channels, one step | 11.8 µs | **10.8 µs** (21.7 µs if Φ is rebuilt) | `kalman_cost.csv` |

1. **delta's velocity estimate is wrong by 29–34 % even when nothing is noisy.**
   That is not noise, it is the gain and phase error of the bandpass and the
   difference quotient: `|H|/ω` is 0.826 / 0.842 / 0.772 and the phase error is
   +6.16° / −9.48° / **−35.21°** at 0.7155 / 0.9949 / 1.6396 Hz
   (`kalman_phase.csv`). The Kalman estimate is **exactly** unity gain and
   **exactly** zero phase error at those three frequencies.
2. **A phase error on a velocity estimate is not a small loss of damping, it is
   a spring.** Decomposing `v̂ = (|H|/ω)(cos e − j sin e) v`, at mode C
   **0.4451 of delta's Kp force is stiffness and only 0.6306 is a dashpot**. At
   the same `Kp` the Kalman estimator delivers **1.218× / 1.205× / 1.586×** the
   dissipation at modes A / B / C. Part of that is restored magnitude, which
   costs volts (1.21× / 1.19× / 1.30× the commanded amplitude in band); the rest
   is that **all** of the commanded force becomes dissipative, where delta wastes
   0.6 % / 1.4 % / **18.3 %** of it on a spring.
3. **Ki can be deleted**, which returns the 0.1135 V that `gains.json` spends on
   it — 38 % of a 0.2998 V peak against a 0.250 V half-window
   (`analysis/mimo_closed.md` §5.2). Kd can go too. Section 8.

**And one negative result, stated because it is the thing most likely to be
over-claimed:** the narrowbanding *by itself* returns roughly **no** actuator
headroom. Replayed on the bench run, demand rms goes 0.59× and 0.74× on ch0 and
ch2 but **1.17× and 1.08× on ch1 and ch3** — the restored in-band magnitude
cancels most of what is saved above 3 Hz (`kalman_headroom.csv`). The headroom
is Ki's, and Ki's alone.

**And one finding that is not about the Kalman filter at all and should ship
first, on its own:** delta's decimation aliases the mains line into the control
band. Section 4.

**The FFT proposal is the same idea and should be built as this filter, not as
an FFT.** Section 7.

---

## 1. The estimator

Per channel `i`, seven states, all in that channel's own sensor volts:

```
x = [q_A, v_A, q_B, v_B, q_C, v_C, b]

    q̇_m = v_m
    v̇_m = −ω_m² q_m + w_m          ω_m = 2π × (0.7155, 0.9949, 1.6396) Hz
    ḃ   = w_b

    y    = q_A + q_B + q_C + b + n        n ~ N(0, R_i)
    v̂    = v_A + v_B + v_C
```

**Why those states.**

- **Three oscillators, not two, not a generic filter.** The three modes are
  measured to ±0.0005 Hz (`analysis/ringdown.md`, `quiet_frequencies.csv`).
  Putting them in the model is what buys the exactness in §5.
- **Undamped.** τ > 138 s at 1σ against a closed loop that settles in
  2.9–4.7 s: over one filter time constant the intrinsic decay is under 1.5 %.
  The data cannot constrain a damping term, so there is no damping term.
  *This is an assumption and is labelled here as one: the plant is treated as
  exactly undamped, and the process noise carries the amplitude changes.*
- **One DC state, and it is the whole reason Ki dies.** `v̂` reads `Cv` which
  does not touch `b`, so the estimate is DC-free **by construction** rather than
  because a 0.4 Hz highpass removed the DC. The drift is estimated explicitly
  instead of being filtered away, which leaves an integrator nothing to reject.
- **`q_m` is a per-channel quantity**, the projection of mode `m` onto sensor
  `i` in sensor units — *not* a modal coordinate. No Φ is formed, nothing is
  inverted across channels, and dropping a channel changes nothing about any
  other channel's filter. **This is SISO, one filter per channel, and it stays
  that way.** The sensors and coils being colocated and coaxial is what makes it
  legitimate: one OSEM head per channel, so the per-channel residue sign is a
  single constant and the decomposition needs no geometry.

**Discretisation is exact, not Euler.** Each 2×2 block is the rotation
`[[cos θ, sin θ/ω], [−ω sin θ, cos θ]]`, `θ = ωΔ`, and the process-noise block
is the closed-form integral

```
Qd = q · [[ (Δ/2 − sin 2θ/(4ω))/ω² ,  sin²θ/(2ω²) ],
          [ sin²θ/(2ω²)            ,  Δ/2 + sin 2θ/(4ω) ]]
```

**The steady-state gain `K` is solved offline and shipped as a constant.** No
Riccati recursion runs on the bench. It converges from `P = I` in 4 000–12 000
iterations, 0.2–0.4 s per channel, and the tolerance must be **relative** to
`|K|` — an absolute tolerance below float precision never terminates, which is
how the first draft of this hung.

**Free, having paid for the states:** `displacement()` (`bp`'s replacement),
`modal_velocity()` (per-mode, which a bandpass cannot produce at any price), and
`acceleration()` in closed form as `a_m = −ω_m² q_m`, with no difference
quotient and therefore no noise cost.

---

## 2. R — measured, not chosen

`data/20260806_192723_quiet_openloop.csv`, 355 797 samples, 1111.87 Hz, coils
held at BIAS and never written again. One sample rejected outside 0..1023 — the
same `72647` on ch7 `ringdown.md` reports. First 20 s discarded as settle, then
delta's own decimation applied, because **the decimated mean is what the filter
sees**.

`R` is defined as *everything the model does not carry*: the broadband floor
plus every line that is not one of the three modes, integrated over 0.2–50 Hz
with the three mode bands bridged. That definition is the honest one for a
Kalman filter — anything the state model cannot represent is measurement noise
as far as the filter is concerned.

`analysis/out/kalman_R.csv`:

| | a0 | a1 | a2 | a3 | a4 | a5 | a6 | a7 |
|---|---|---|---|---|---|---|---|---|
| √R as shipped, V | 0.1148 | 0.0257 | 0.0608 | 0.0088 | 0.0108 | 0.0555 | 0.0137 | 0.0067 |
| √R with `mains-null`, V | **0.0684** | **0.0061** | 0.0604 | 0.0070 | 0.0058 | **0.0242** | 0.0110 | 0.0049 |
| counts | 13.9 | 1.2 | 12.3 | 1.4 | 1.2 | 4.9 | 2.2 | 1.0 |

The spread across channels is 14×, which is the point of measuring all eight
rather than assuming a uniform R. It is handled without any per-channel tuning:
see §3.

---

## 3. Q — 24 measurements and one constant

For an undamped oscillator driven by white acceleration of intensity `q`, the
position variance grows as `qT/(2ω²)`. Setting that equal to the mode's
**measured** variance in this channel at `T = T_AMP`:

```
q[i,m] = 2 ω_m² σ²[i,m] / T_AMP
```

The 24 `σ²[i,m]` come from the same quiet-record spectrum that gave R
(`kalman_Q.csv`), so R and Q are consistent by construction. **`T_AMP` is the
only tuning constant in the whole estimator**, and it means one thing: the time
in which a mode's amplitude is allowed to change by order itself.

**`T_AMP = 50 s`**, chosen from a sweep, not from taste
(`kalman_sweep_synthetic.csv`, `kalman_sweep_quiet.csv`,
`kalman_sweep_freqerr.csv`):

| T_AMP | settle after a ×10 kick | steady error, ambient | steady error, post-kick | phase err at f + 0.05 Hz |
|---|---|---|---|---|
| 0.5 | 0.54 s | 29.5 % | 2.5 % | −2.9° |
| 5 | 0.79 s | 14.2 % | 1.1 % | −3.8° |
| **20** | **1.42 s** | **9.2 %** | **0.7 %** | −5.6° |
| **50** | **1.48 s** | **7.0 %** | **0.6 %** | −7.3° |
| **100** | **1.57 s** | **5.8 %** | **0.5 %** | −9.2° |
| 300 | 3.64 s | 4.7 % | 0.6 % | −13.4° |
| 1000 | 7.02 s | 4.9 % | 1.3 % | — |
| delta | 19.90 s (never gets under 10 %) | 33.8 % | 29.3 % | n/a |

The settle time is **flat at 1.4–1.6 s over T_AMP 20–100** and only then
degrades. It saturates because telling mode A from mode B needs
`1/0.2794 = 3.58 s` of evidence no matter what estimator is used, and the filter
pays about half of it. 50 is the round number in the middle of that flat
optimum.

**Note what this parameterisation buys.** R varies 14× across channels and the
modal powers vary 300×, and neither needs a per-channel knob: both enter `q/R`,
which is what sets each filter's bandwidth, and both are measured. One constant
for all eight channels.

---

## 4. A prerequisite that is not part of the estimator: `mains-null`

**This was not known before and it should ship on its own, ahead of any
estimator work.**

The raw stream carries a **50.06 Hz mains line**: 0.1439 V rms on a0 (29.33
counts), 0.0778 V on a5, 0.0386 V on a1 (`kalman_R.csv`). delta's decimation
averages ~11 wire samples spanning 10 ms, so it attenuates 50 Hz only to
`sinc(0.5) = 0.64`, and the 100 Hz control rate then folds it to just under
Nyquist. It arrives in the control band's noise budget at **0.80 / 0.96 / 0.89
of √R on a0 / a1 / a5** — i.e. on three of the eight channels, *the dominant
measurement noise at the control rate is aliased mains*.

**The fix is one line: average two consecutive control means.** That is a 20 ms
boxcar, whose first null is at 50 Hz *exactly*. Measured effect on the
above-49 Hz content of the decimated record: a0 0.0920 → 0.00026 V, a1 0.0247 →
0.00009 V, a5 0.0497 → 0.00012 V. R falls to **0.354× / 0.057× / 0.190×** on
a0 / a1 / a5.

The cost is pure group delay of 5.0 ms, flat: **−1.29° / −1.79° / −2.95°** at
the three modes, magnitude 0.9997 / 0.9995 / 0.9987.

It helps `osem.delta.py` exactly as much as it helps this estimator, and it is
independent of everything else here. Everything below is measured with it
applied.

*Caveat, and it is the reason to check rather than assume: the null is at
1/(2Δ) where Δ is the control period. It lands on 50 Hz because `CONTROL_HZ` is
100. Change the control rate and the null moves off the mains.*

---

## 5. Phase, and why it is the whole argument

`analysis/out/kalman_phase.csv`, both estimators at 100 Hz, against an ideal
differentiator (`|H| = ω`, phase +90°):

| f (Hz) | | delta `\|H\|/ω` | delta phase − 90° | kalman `\|H\|/ω` | kalman phase − 90° |
|---|---|---|---|---|---|
| 0.4000 | BP corner | 0.6925 | +31.77° | 1.9147 | +76.62° |
| **0.7155** | **mode A** | 0.8257 | **+6.155°** | **1.0000** | **+0.000°** |
| **0.9949** | **mode B** | 0.8415 | **−9.480°** | **1.0000** | **+0.000°** |
| 1.2000 | | 0.8289 | −18.80° | 0.7398 | −29.27° |
| **1.6396** | **mode C** | 0.7719 | **−35.213°** | **1.0000** | **+0.000°** |
| 3.0000 | BP corner | 0.5452 | −69.63° | 0.2109 | −119.39° |

**The exactness at the three modes is structural, not tuned.** A pure sinusoid
at `f_m` is an exact zero-process-noise trajectory of the model; the error
dynamics `(I − KH)Φ` are stable; therefore the estimate converges to the true
state, and to the true velocity. It holds for **any** Q and **any** R —
confirmed numerically across the whole of the T_AMP sweep. This is why the
frequencies being measured to ±0.0005 Hz is what unblocked the design, and why
Q being unknown does not block it.

### What the phase error costs, in the only currency that matters

Kp acts on `v̂`, so it is a dashpot only to the extent `v̂` is in phase with
velocity. Write `v̂ = (|H|/ω)(cos e − j sin e) v`: the `cos` part is a dashpot,
the `sin` part is a **spring** — a stiffness, negative for a lagging estimate —
and it dissipates nothing. `analysis/out/kalman_dissipation.csv`:

| mode | f | delta dashpot | delta spring | kalman dashpot | ratio |
|---|---|---|---|---|---|
| A | 0.7155 | 0.8209 | +0.0885 | 1.0000 | **1.218×** |
| B | 0.9949 | 0.8300 | −0.1386 | 1.0000 | **1.205×** |
| C | 1.6396 | 0.6306 | **−0.4451** | 1.0000 | **1.586×** |

At mode C, **44.5 % of what Kp commands is a spring** — `sin(35.213°) × 0.7719`
of the ideal `ω`, which is **18.3 % of the force actually commanded**. This is
the same physics
the brief states about Ki and Kd, arriving through a different door: Ki
integrates velocity, which is position, which is a spring; Kd differentiates it,
which is acceleration, which is effective mass; only the in-phase part of Kp
removes energy. A bandpass-and-differentiate estimator quietly converts part of
your one dissipative term into one of the two that are not.

### Rejection of what is not a mode

`analysis/out/kalman_offmode.csv`, `|H|` in V/s per V:

| line | f (Hz) | delta | kalman | kalman/delta |
|---|---|---|---|---|
| C − B intermod | 0.6447 | 3.285 | 6.218 | 1.89 |
| A + B intermod | 1.7104 | 8.175 | 11.734 | 1.44 |
| 2B intermod | 1.9898 | 8.926 | 8.395 | 0.94 |
| 3B intermod | 2.9847 | 10.269 | 4.006 | 0.39 |
| 6.19 Hz interference | 6.19 | 8.998 | 1.625 | **0.18** |
| 50 Hz alias | 49.91 | 2.310 | 0.298 | **0.13** |

The estimator rejects the far-out lines hard and does **not** reject the
intermodulation lines that sit within ~0.3 Hz of a mode. That is honest and it
is the price of a 1.4–1.6 s settle time: a filter narrow enough to reject a line
0.07 Hz away is a filter that takes 14 s to acquire.

### The loop, and the one place this is worse

`analysis/out/kalman_loop.csv`, computed with `analysis/kp040.py`'s own transfer
functions so the rows are comparable to `phase_budget.csv`. `L_nonplant` is
sensor path × PID × ZOH at `Kp = −0.035`, Ki/Kd as shipped for delta and **zero**
for kalman.

| f (Hz) | delta `\|L\|` | delta phase | kalman `\|L\|` | kalman phase |
|---|---|---|---|---|
| 0.7155 | 0.1325 | +95.56° | 0.1573 | +87.42° |
| 0.9949 | 0.1905 | +80.69° | 0.2186 | +86.42° |
| 1.6396 | 0.2971 | +54.99° | 0.3599 | +84.10° |
| 2.79 | 0.3924 | +22.69° | 0.1545 | −34.73° |
| 6.19 | 0.3571 | −26.31° | 0.0554 | −76.27° |
| 49.9 | 0.0594 | −89.96° | 0.0000 | −179.80° |

Phase margin at the three modes (`phase − 90 + 180`, valid only *at* a
resonance): delta **185.6 / 170.7 / 145.0°**, kalman **177.4 / 176.4 / 174.1°**.
The kalman loop is within 6° of a pure dashpot at all three; delta is 35° off at
mode C.

**Above the band the kalman loop lags more, and that is the honest cost of a
narrowband estimator.** Three things bound it, all measured.

- Its `|L|` **above** the band is 2.5–6.5× *smaller* than delta's: 0.155 vs
  0.392 at 2.79 Hz, 0.055 vs 0.357 at 6.19 Hz, 0.0000 vs 0.0594 at Nyquist. Peak
  `|L|` is the same for both — 0.4099 at 1.71 Hz against delta's 0.4076 at
  3.66 Hz — but the kalman peak sits *on* the modes and delta's sits above them.
- **Neither loop reaches −180° below Nyquist**, so `kp040.md`'s "no sub-Nyquist
  −180 crossing" survives in form.
- The kalman non-plant phase does cross **−90°** — the crossing that would
  matter *if a resonance sat there* — at **0.31 Hz (`|L|` 0.1515)** and
  **9.14 Hz (`|L|` 0.0357)**. There is no measured mode at either; the modes are
  0.7155–1.6396 Hz, and the 0.31 Hz crossing is the DC state's own phase.

**This is a check to repeat on the bench, not a settled result: the plant above
the modes and below 0.7 Hz is not measured on this rig.**

**Standing caveat.** `analysis/kp040.md` established that phase margin does
*not* explain the −0.040 rail onset. Extra margin at the modes is therefore
**not** a licence to raise Kp. What the phase buys is the dashpot ratio above,
at the same Kp.

---

## 6. What the replay says

### 6a. The quiet record — the clean comparison

Scored against a **zero-phase reference**: keep the band, differentiate exactly
in the frequency domain, come back. *That is not ground truth and is labelled as
such; it is a definition of what the in-band velocity is, computed with no
causal filter, so two causal estimators can be scored against the same thing.*
Two references, because the two estimators do not have the same target:
`v_modal` (the three lines) and `v_band` (all of 0.4–3.0 Hz).
`analysis/out/kalman_replay_quiet.csv`:

| ch | estimator | resid vs `v_modal` | corr | resid vs `v_band` | corr |
|---|---|---|---|---|---|
| a0 | delta | 0.412 | 0.912 | 0.397 | 0.924 |
| a0 | **kalman** | **0.264** | **0.967** | 0.407 | 0.914 |
| a1 | delta | 0.401 | 0.923 | 0.400 | 0.923 |
| a1 | **kalman** | **0.055** | **0.998** | **0.041** | **0.999** |
| a2 | delta | 0.344 | 0.944 | 0.352 | 0.943 |
| a2 | **kalman** | **0.185** | **0.983** | **0.255** | **0.967** |
| a3 | delta | 0.406 | 0.920 | 0.405 | 0.920 |
| a3 | **kalman** | **0.079** | **0.997** | **0.071** | **0.998** |

Better on every channel against `v_modal` and on three of four against `v_band`
— which is delta's *own* target, not this estimator's. a0 and a2 are the two
channels carrying the strong intermodulation lines (§2 of `kalman.py`), which
`v_modal` excludes and the estimator partly passes; that is where their 0.264
and 0.185 come from.

### 6b. The bench run — reconstruction, then the same ordering

`data/20260806_200822_fast_lock.csv`. The control step is reconstructed
**exactly**: delta logs `ctl` and `n_avg`, and the `n_avg` rows ending at a
flagged row are the set that step averaged. The reconstruction is **checked**
by re-running delta's own filter chain against the logged `ch{i}_vel`
(`kalman_replay_check.csv`):

> median 1.1e-3, p99 1.5e-2, max 8.0e-2 V/s against a 35.87 V/s peak —
> which is exactly the CSV's own rounding, since `bp` is logged at five decimals
> and `vel = d(bp)/dt` multiplies that 1e-5 by `1/dt = 100`.

That check failed on the first attempt (62 V/s) and the cause is worth
recording: a low-pass lane in `OnePole` primes to **its own input**, and for
`lp` that input is the high-pass output, i.e. **0**, not the raw volts. A
reimplementation that primes `lp` to the raw signal is wrong for the first 27
control steps and right afterwards.

Over **clean DAMPING** (no kick window, nothing railed — 13 485 of 18 940
control steps), residual vs `v_modal` (`kalman_replay_run.csv`): delta 0.744 / 0.755 / 0.779 / 0.781,
kalman **0.632 / 0.588 / 0.675 / 0.614**, on ch0–ch3.
*Weaker evidence than 6a: the reference is built from a record containing
clipped kicks, so its band-limited form rings into the quiet stretches. Read the
ordering, which is the same on all four channels; read the absolute numbers off
6a, where nothing rails.*

### 6c. Inside the kicks, and `rail-blank`

**13.7 % of ch0's and 16.5 % of ch2's control samples are railed inside the kick
windows.** No offline reference is valid there, because the recorded signal is
clipped and any reference built from it is clipped too. What can be reported is
what each estimator does (`kalman_replay_kicks.csv`, peak |v̂| in V/s):

| ch | railed | delta | kalman | kalman + `rail-blank` |
|---|---|---|---|---|
| ch0 | 13.70 % | 35.36 | 21.85 | **15.95** |
| ch1 | 4.13 % | 27.43 | 35.58 | 27.95 |
| ch2 | 16.45 % | 37.23 | 29.92 | **20.75** |
| ch3 | 4.68 % | 27.90 | 30.93 | 27.45 |

`rail-blank` is a capability the bandpass structurally cannot have: when
`self.rail` says a sample is at an end stop, skip the *update* and predict only,
coasting on the model for that channel. It needs no new measurement —
`_rail_check` already produces `self.rail` at full wire rate — and it does not
make any interlock trust a filtered signal, because the interlock still reads
raw counts and the estimator merely *consumes* its verdict.

**This is not validated and must not ship enabled.** A permanently railed sensor
would coast forever, so it needs a bound (blank for at most N consecutive steps,
then fault) and that bound has no measurement behind it yet.

---

## 7. "FFT the data, phase-match, sum, play it back in negative"

Three claims. Two are wrong for measurable reasons; the third is right and is
§8.

**The framing as put to me was checked item by item and every number in it is
right.** Drift +6.150e−05 / −9.809e−05 / +1.165e−04 Hz — confirmed, those are
`quiet_freq_stability.csv` to the digit. A 1e−4 Hz error at 0.7155 Hz accrues
`360 × 1e−4 = 0.036 °/s`, so 90° in 2500 s — confirmed (mode B's own drift gives
2549 s). Separating A from B needs `1/0.2794 = 3.579 s` — confirmed. Half a
block stale is 1.79 s, which at 0.7155 Hz is 1.28 cycles — confirmed. **No
corrections.** What follows adds one thing the framing did not have: the same
argument applied to the *intermodulation* lines, where the block length required
is four times worse.

### 7a. The FFT is unnecessary, and it costs far more phase than it recovers

The frequencies are already known. Drift across the 320 s record, first half
against second (`quiet_freq_stability.csv`, `kalman_fft_drift.csv`):

| mode | drift | phase rate | 90° in |
|---|---|---|---|
| A | +6.15e−05 Hz | 0.0221 °/s | 4065 s (67.7 min) |
| B | −9.81e−05 Hz | 0.0353 °/s | 2549 s (42.5 min) |
| C | +1.16e−04 Hz | 0.0419 °/s | 2146 s (35.8 min) |

Against that, what a block costs (`kalman_fft_latency.csv`). Separating two
lines needs a block longer than `1/Δf`, and a block's answer describes its
**middle**, so it is stale by half a block:

| pair | Δf | min block | stale by | at 0.7155 Hz |
|---|---|---|---|---|
| **A–B** | 0.2794 Hz | **3.58 s** | **1.79 s** | **1.28 cycles = 461°** |
| B–C | 0.6447 Hz | 1.55 s | 0.78 s | 0.55 cyc = 200° |
| A–C | 0.9241 Hz | 1.08 s | 0.54 s | 0.39 cyc = 139° |
| A vs the 0.6447 Hz line | 0.0708 Hz | 14.12 s | 7.06 s | 5.05 cyc = 1819° |

**The binding pair is A–B: 461° of latency to correct an error that reaches 90°
once every 36–68 minutes.** The trade is not close. And it is worse than that:
separating mode A from the 0.6447 Hz intermodulation line needs a 14.1 s block,
so any block short enough to be useful hands that line's power to mode A.

**Use a recursive lock-in at the known frequencies, not a block transform.**
Multiply by `exp(−jω_m t)` and low-pass: no block, no block latency, a
first-order lag on the *amplitude* only, and the phase of an already-acquired
line is exact.

**And that is this filter.** A steady-state Kalman filter carrying an undamped
oscillator at `ω_m` **is** a quadrature lock-in at `ω_m`: `(q_m, v_m)` is the
in-phase/quadrature pair rotating at `ω_m`, `K` is the loop filter, and the
exactness in §5 is the lock-in's zero steady-state phase error. Three of them
with the velocities summed is the proposal, done recursively. **The two ideas
are one design.**

The information cost does not vanish, it changes currency. The FFT delays
**everything** by 1.79 s; the filter delays only the **amplitude** — measured
settle 1.4–1.6 s, §3 — and holds phase exactly throughout, including during
acquisition.

### 7b. "Play it back in negative" is a spring, not a damper

A force proportional to `−x` is stiffness. It moves the mode frequency and
**dissipates nothing**: over a cycle the work done by `−kx` against velocity
integrates to zero. Removing energy requires a force opposing **velocity**, 90°
from displacement — which is what `Kp·v̂` already is, and why Kp is the only
dissipative term `osem.delta.py` has.

```
Ki integrates velocity  →  displacement  →  a spring
Kp acts on velocity     →  a dashpot     →  the only term that damps
Kd differentiates it    →  acceleration  →  effective mass
```

A sign error here does not under-damp, it **pumps**. The standing proof in this
repo is ch2, whose gains are POSITIVE where the other three are negative because
that OSEM is mounted the other way round. The summed narrowband estimate goes to
`Kp` with the per-channel sign kept, and nothing about the sign handling
changes.

---

## 8. Narrowband (resonant) gain — the part of the proposal that is right

The three modes have a **measured** 3 dB width of 0.00566 Hz each
(`quiet_linewidth.csv`), and that is resolution-limited — the excess over a pure
tone through the same window is only 0.00159 Hz. At the Q > 433 bound the true
width is `f/Q` = 0.00165 / 0.00230 / 0.00379 Hz.

| | total 3 dB linewidth | fraction of the 2.6 Hz control band |
|---|---|---|
| as measured (instrument-limited) | 0.01698 Hz | **0.65 %** |
| at the Q > 433 bound | 0.00774 Hz | **0.30 %** |

Broadband velocity feedback spends authority and phase across 2.6 Hz to damp
0.3–0.65 % of it. **Concentrating gain at the lines is right, and it is already
in the estimator**: `Cv` reads only `v_A + v_B + v_C`, so `v̂` *is* the sum of
three resonant estimates and `Kp·v̂` *is* resonant gain. No extra stage, no
extra phase. `modal_velocity()` exposes the three separately if per-mode gains
are ever wanted — still one filter per channel, still no Φ, still not MIMO.

### Where the actuator volts actually go

`kalman_demand_spectrum.csv`, on the longest **contiguous** DAMPING window
(20.0–92.0 s, 7203 control samples, 0.0139 Hz resolution — a concatenation of
non-contiguous samples has no spectrum). Fraction of the commanded `p+i+d`
power:

| ch | demand rms | at the three lines | in band, off-line | **above 3 Hz** |
|---|---|---|---|---|
| ch0 | 0.1910 V | 2.95 % | 30.77 % | **66.27 %** |
| ch1 | 0.1185 V | 1.52 % | 42.38 % | **56.10 %** |
| ch2 | 0.1960 V | 2.38 % | 30.54 % | **67.08 %** |
| ch3 | 0.1314 V | 0.96 % | 49.08 % | **49.95 %** |

**Half to two-thirds of the actuator power delta commands is above 3 Hz**, where
there is no measured mode. That is not a surprise once §5's table is in hand:
delta's `|H|` peaks at 3.75 Hz (10.38) against 5.26 at mode B, because a
difference quotient rises with frequency faster than a one-pole low-pass falls.
Under 3 % is at the three lines the loop exists to damp.

### What the narrowbanding itself returns in headroom — roughly nothing

Replay the **same recorded motion** through both estimators and form the demand
each would command: delta's `p+i+d` as logged, kalman's `gain × (−v̂)` with
Ki = Kd = 0. *Open-loop replay — the closed loop would have taken a different
trajectory, so this compares two estimators on one recorded signal and is not a
prediction of closed-loop demand.* `kalman_headroom.csv`, DAMPING:

| ch | delta rms | kalman rms | ratio | delta peak | kalman peak |
|---|---|---|---|---|---|
| ch0 | 0.1610 V | **0.0947** | 0.59 | 1.3808 V | **0.7646** |
| ch1 | 0.1154 V | 0.1352 | 1.17 | 1.1292 V | 1.2454 |
| ch2 | 0.1619 V | **0.1204** | 0.74 | 1.4204 V | **1.0472** |
| ch3 | 0.1249 V | 0.1345 | 1.08 | 1.1283 V | 1.0827 |

Two effects pull opposite ways and **they roughly cancel**. Against: the
estimator restores the in-band magnitude the bandpass attenuated, so at the same
Kp it asks for 1.21× / 1.19× / 1.30× more at modes A / B / C. For: it commands
almost nothing above 3 Hz, where 50–67 % of delta's demand power sits, and Ki
and Kd are gone.

**So the headroom claim must be made carefully.** Demand falls on ch0 and ch2 —
the two channels carrying the mains alias and the strong intermodulation — and
rises 8–17 % on ch1 and ch3. **Narrowband gain by itself is roughly neutral on
actuator demand on this rig.** The headroom this work returns is Ki's, below,
and that one is measured.

### Ki, and what dropping it returns

`gains.json`'s design-point split (`analysis/mimo_closed.md` §5.2, derived from
drive amplitudes and filter gains, not from a mode fit): **P 0.1279 + I 0.1135 +
D 0.0584 = 0.2998 V against a 0.250 V half-window — 120 % of it, Ki 38 % of the
total.** On the bench run itself (`kalman_actuator_budget.csv`, DAMPING) the I
term peaks at 0.1339 / 0.0524 / 0.1450 / 0.0691 V on ch0–ch3.

**This design lets Ki be dropped outright, and it removes even the pretext.**
Today `i = −Ki·bp` exactly, because the integrator sits behind the 0.4 Hz
highpass and `bp` carries no DC — so Ki rejects no drift while consuming 38 % of
the peak. With the estimator, the drift is a **state** (`b`), estimated
explicitly, and `v̂` never contains it. There is nothing left for an integrator
to reject. **Ki → 0 returns 0.1135 V of the design-point peak, 38 %.**

Kd survives as a choice rather than a necessity: `acceleration()` is available
in closed form, so a D term would cost no differentiation and no noise. It is
still effective mass and still dissipates nothing. **Recommendation: Kd → 0 as
well.** With both at zero the back-calculation anti-windup in `_actuate` has
nothing to act on and can go with them.

### What narrowband gain costs

1. **A disturbance that is not at a mode gets no force.** On this rig there is
   no evidence of one: on a0–a3 the three modelled lines account for **95.5 /
   99.8 / 98.6 / 99.8 % of all 0.4–3.0 Hz power** in the quiet record
   (`kalman_Q.csv`, last column), and what they do not account for is
   intermodulation (`kalman_lines.csv`, `kalman_intermod.csv`) — lines at exact
   sums and differences of the measured three, which a mechanical mode has no
   reason to be at. Driving a coil at those would be feeding back on the
   sensor's own curvature. *The quadratic-nonlinearity explanation is consistent
   to within a factor of 3–4 across three independent product lines on a0 and
   a2 and is **not** established; a factor of 3 is not a fit. What does not
   depend on the mechanism is that they are inside 0.4–3.0 Hz and delta's
   bandpass passes them at full weight.*
2. **During a kick the response looks broadband.** Measured on the longest
   contiguous piece of each kick window, with the half-width widened to the
   window's own resolution (`kalman_kicks.csv`), fraction of ch0's power at the
   lines: **20.6 % (12.0 s) / 97.8 % (11.0 s) / 32.7 % (8.4 s) / 95.4 %
   (11.0 s)**; the fifth window is 5.5 s, whose resolution is coarser than the
   mode spacing, so it is reported and not used. **The spread is 20 % to 98 %.
   This does not settle the question — it is the weakest number in this work and
   is reported as such.** What widens a line in these windows is the finite
   window, the transient itself, and clipping; a rigid body kicked by hand
   responds at its resonances and nowhere else, so none of the three is motion a
   coil can usefully oppose. But that is an argument, not this measurement.
3. **Authority in the 2.9–4.7 s settling regime is not reduced.** The dashpot at
   the modes goes *up* by 1.218× / 1.205× / 1.586×, at the same Kp. Nothing is
   taken away in band.

### Recommendation, not a menu

**Build the Kalman estimator, delete Ki and Kd, and take the resonant gain as
what it already is — a property of that estimator — rather than as a separate
stage.**

The two ideas cannot usefully be separated. Narrowband gain *without* the Kalman
filter needs a resonant filter bank, which is the same computation with a worse
phase response and no DC state. The Kalman filter *without* the narrowband
property is not a thing that can be built, because `Cv` reading only the modal
velocities is exactly what makes it exact at the modes.

**What is separable, and should go first, on its own:** `mains-null` (§4). One
line, independently justified, and it improves `osem.delta.py` as it stands.

**Ranked by evidence, weakest last:**

1. `mains-null` — R falls 0.354× / 0.057× / 0.190× on a0 / a1 / a5, for 5 ms of
   flat group delay. Measured.
2. Ki → 0 — 0.1135 V, 38 % of the design-point peak, on a term that by
   construction rejects no drift. Measured, and the estimator removes the
   pretext for keeping it.
3. The estimator itself — 29.3 % → 0.6 % velocity error against known truth, and
   1.218× / 1.205× / 1.586× the dissipation per unit Kp. Measured against
   synthesis and confirmed in replay on two records.
4. `rail-blank` — plausible, unvalidated, needs a bound. Do not ship enabled.
5. Per-mode gains via `modal_velocity()` — possible, no evidence it is wanted.

---

## 9. Cost per step

`kalman_cost.csv`, all eight channels, one control step, numpy 2.5.1, measured
over 20 000 iterations:

| | µs/step | of the 10 ms budget |
|---|---|---|
| kalman, fixed Φ | **10.8** | 0.11 % |
| kalman, Φ rebuilt from the measured `dt` | **21.7** | 0.22 % |
| delta's HP/LP/diff/LP, for scale | 11.8 | 0.12 % |

*These are wall-clock on one laptop and repeat to about ±2 % (10.6 / 21.3 / 11.8
on a second run). The operation count below is what actually bounds it.*

**It is not more expensive than what it replaces.** Per step, eight channels:
one (8,7)×(7,7) matmul = 392 multiply-adds plus three length-7 contractions =
168. Rebuilding Φ costs **3 cos + 3 sin in total**, shared by every channel,
because Φ does not depend on the channel — so the jitter in the control period
can be absorbed exactly rather than ignored, for 10 µs. `K` is solved offline
and shipped as a constant.

---

## 10. Exactly what changes in `osem.delta.py`, and what does not

**No new controller file is proposed here** — the estimator is a class in
`analysis/kalman.py` that a controller imports or has pasted in.

### Changes

| where | change |
|---|---|
| `Controller.__init__` | `self.hp`, `self.lp`, `self.dsm` (and `self.dfilt` if Kd goes) are replaced by one `self.kf = KalmanVelocity(R, q, q_dc)`. `R` and `q` are constant arrays, computed by `analysis/kalman.py` and pasted in like `SLOPE_SIGN`. |
| `_control`, first three lines | `self.vel = self.kf.update(volts, dt, valid=~self.rail)` then `self.bp = self.kf.displacement()`. `prev_bp` and `filt_on` go away — the estimator has no priming special case. |
| `step()`, full-rate half | **one added line**: keep the previous control mean and average it with this one (`mains-null`, §4). |
| `_actuate` | `Ki = Kd = 0` ⇒ the `self.i` integrator, the `TRACK_TC_S` back-calculation and `self.dfilt` all become dead code. `self.p = self.gain * (-self.vel)` is the whole law. |
| `_health` demotion | add `self.kf.reset(dead)` alongside the existing `self.dfilt.reset(dead)`, for the same reason: a lane coming back must not resume on a stale state. |
| `_fingerprint` / `save_baseline` | `bp_low_hz`, `bp_high_hz` are no longer what defines the baseline signal. They must be replaced by the mode list and `T_AMP`, or **every stored baseline refused**. The baseline *is* an RMS of `bp`, and `displacement()` is a different signal from the bandpass output; a stored floor measured against the old one is a wrong denominator, which `persist-baseline`'s own docstring says desensitises the runaway breaker and makes `LOCKED` easier to declare. |
| `REARM_SUSTAIN_S`, `BASELINE_WARMUP_S` | both are 2.0 s and both are documented as "5 τ of the 0.4 Hz highpass". There is no highpass any more. **2.0 s survives on the new justification** — the estimator's measured acquisition is 1.48 s (§3) — but the comment must change, or the number becomes another unsourced constant. |

### Not changed

- **`step()`'s full-rate half, and `_rail_check` on RAW COUNTS.** The rail
  interlock keeps reading raw counts at wire rate and keeps never seeing a
  filtered signal. `rail-blank` (§6c) is the estimator *consuming* `self.rail`,
  not the interlock consuming the estimator.
- Every other interlock: `_sat_check`, the runaway breaker, `soft-saturation`,
  the lock detector. All read `self.bp` (still produced) and `self.out`, and all
  are ratios against the baseline, so they survive a change of what `bp` means
  **provided the baseline is re-measured with the same definition** — which is
  the fingerprint item above.
- `Actuator`, `read_sample`, `in_range`, `sample-guard`, `decimate`'s
  time-based accumulation, `bias-trim`, `baseline-floor`, `fast-calib`,
  `warm-restart`, the state machine, `main()`.
- **The per-channel sign handling.** `STEADY_GAIN[2]` stays positive. The
  estimator is per-channel and sign-blind; it estimates the velocity of whatever
  that sensor sees, and the sign that turns it into a force is exactly where it
  is now.
- **No MIMO.** No Φ, no cross-channel term, no shared state. Eight independent
  filters. Graceful degradation and the quorum are untouched, because dropping a
  channel drops one filter and nothing else.

---

## 11. What this does NOT fix

- **The clipping.** 2.9 % of ch0's and 3.6 % of ch2's samples over the whole
  run, 13.7 % and 16.5 % inside the kick windows. A better estimator cannot
  recover information the ADC never sampled. This remains the sensor-range
  problem in README §5.
- **`LOCKED` is still never announced.** That is the a5 quorum, unrelated.
- **The 10 s lock budget.** `LOCK_SUSTAIN_S = 5.0` and
  `FAULT_CLEAR_SUSTAIN_S = 5.0` are untouched and are larger items than this.
- **a1's noise floor, and therefore MIMO.** This is SISO and does not
  bear on it either way.
- **a4/a6/a7 at ~1/100 in-band gain.** Their `q` comes out ~1e−7 and their
  filters will correctly estimate almost nothing.
- **`Kp = −0.040`.** Still uncharacterised, and §5 explains why the extra phase
  margin is not a licence to go there.
- **The intermodulation lines.** The estimator rejects the far ones and passes
  the near ones; it does not remove them from the sensor.
- **Day-to-day frequency stability.** Measured drift *within* one 320 s record
  is ≤ 1.2e−4 Hz. Whether `f_m` is the same tomorrow is **not measured**, and
  that is the standing hazard of a model-based estimator, not the tuning. §3
  gives the degradation curve: even 0.05 Hz of error costs only 7.3° at
  `T_AMP = 50`, which is still better than delta at the exactly-right frequency,
  but there is no measurement bounding how far it can drift.

---

## 12. What the bench should do with this, in order

1. **`mains-null` alone**, on `osem.delta.py`, nothing else changed. One line,
   independently justified, and it makes every subsequent number cleaner.
   Confirm the >49 Hz content of the decimated stream drops by the factors in
   §4.
2. **Estimator in, Ki and Kd still at their shipped values**, so the change is
   one thing. Confirm the `vel` column's character and that the interlocks
   behave. **Refuse the stored baseline** (`--fresh`) — the fingerprint item
   above.
3. **Ki → 0, then Kd → 0**, separately, and measure the peak actuator demand
   each time against the 0.2998 V / 0.250 V arithmetic.
4. Only then `rail-blank`, with a bound on consecutive blanked steps, since
   nothing validates it yet.
5. Re-measure `f_m` at the start of any session that uses this. It is cheap —
   `analysis/ringdown_quiet.py` on a 320 s gain-off record — and it is the one
   input the whole design rests on.
