# Retuning `CALIB_AGREE_TOL` from the bench logs

**Recommendation: `CALIB_AGREE_TOL = 1.35`** — the loosest value whose worst
measured baseline skew on the recorded bench windows (**29.0 %**) stays under the
35 % line, against **37.3 %** at 1.45. It fires on 29 of 35 recorded calibration
windows and takes mean calibration **20.0 → 11.5 s**, median exit **8.0 s**.

**But the honest headline is a negative result.** No value of `CALIB_AGREE_TOL`
alone can both fire on this bench and reject a ringdown, because the statistic it
thresholds is smaller for a ringdown than for ordinary bench noise:

| trailing-3 max/min ratio, worst solid channel | value |
|---|---|
| a pure `Q=50, f0=1 Hz, tau=16 s` ringdown, 2 s sub-windows | **1.284** (`e^(4/16)`) |
| bench, at the 6 s floor: median / p90 / max | **1.43 / 2.72 / 8.04** |

More than half of real bench windows are *noisier* than a full ringdown is. Any
tolerance loose enough to fire on the bench is loose enough to accept a ringdown.
1.35 is therefore a **damage bound, not a discriminator**: it is chosen so that
when it does fire on a decaying window the resulting error stays inside 35 %.
The margin is 1.21x and it is thin. §7 gives the two-line structural change that
takes the same saving at half the skew.

`.venv/bin/python analysis/calib_tol.py` regenerates every number here.

---

## 1. Material and method

`data/*_fast_lock.csv`, 27 runs from 2026-08-03 / 08-04 / 08-06 → **42
`CALIBRATING` episodes**, **35** of which ran a full 20 s window (22 first-of-run
"cold", 13 post-fault re-calibrations, i.e. windows that follow a kick). The
`chN_bp` column is written straight from `Controller.bp`, which is the exact
array `_close_subwindow` folds into an RMS, so the replay consumes the
controller's own input.

`analysis/calib_tol.py` reimplements `_close_subwindow`, `_calib_stationary` and
`_set_baseline` including the two details that move sub-window boundaries by a
sample: the current sample is appended to `self.calib` *before* the boundary
test, and `sub_start` is reset to the boundary time rather than to `t0 + 2k`.
`calib_start` is taken from the row *preceding* the episode, because
`_start_calibration(t)` runs on the sample that enters the state.

**Verified against four printed bench baselines** (worst 0.18 %, which is the
5-decimal rounding of `chN_bp` in the CSV):

| bench log | printed | replayed |
|---|---|---|
| `20260804/v9_4ch_3kicks.log:30` ceiling | 0.2100/0.0993/0.3151/0.0794 | 0.2100/0.0993/0.3151/0.0795 |
| `20260804/v8_4ch_3kicks.log:30` ceiling | 0.2548/0.1146/0.4234/0.1040 | 0.2548/0.1146/0.4234/0.1040 |
| `20260806/v10_4ch_3kicks.log:30` ceiling | 0.4968/0.2144/0.6736/0.1789 | 0.4959/0.2144/0.6736/0.1789 |
| `20260806/v10_4ch_3kicks.log:15549` **early** | 0.7427/0.4404/0.8563/0.4656 | 0.7417/0.4398/0.8559/0.4652 |

**Skew** here is `|early baseline / that same window's 20 s ceiling baseline − 1|`,
worst over the solid channels ch0–3 (`harness.py`'s `solid`; a4/a6/a7 measure
0.1–9 % in-band and a5 ~1/12 of a0, `CLAUDE.md` § *Where the constants came from*). The ceiling baseline is
the number the bench stores today, so this is the measured cost of the change and
not a simulated one. It is *not* the same quantity `harness.py` asserts — that
one compares a shocked run to a separate clean run — but it is the only version
of it the bench data can supply, and it is the stricter framing on drifting
windows.

## 2. What the agreement ratio actually is on this bench

Worst over ch0–3, all 35 complete windows. `r1` = trailing-3 max/min; `r2` =
median(tail) vs median(all).

| k | t (s) | r1 min | med | p90 | max | r2 med | r2 max | both ≤ 1.20 |
|---|---|---|---|---|---|---|---|---|
| 3 | 6 | 1.11 | 1.43 | 2.72 | 8.04 | 1.00 | 1.00 | 5/35 |
| 4 | 8 | 1.07 | 1.44 | 2.36 | 5.58 | 1.08 | 1.55 | 4/35 |
| 5 | 10 | 1.11 | 1.50 | 1.95 | 2.75 | 1.11 | 2.29 | 3/35 |
| 6 | 12 | 1.11 | 1.35 | 1.71 | 2.91 | 1.09 | 1.68 | 5/35 |
| 7 | 14 | 1.12 | 1.37 | 1.59 | 2.91 | 1.11 | 1.96 | 2/35 |
| 8 | 16 | 1.12 | 1.36 | 1.65 | 2.39 | 1.08 | 1.64 | 4/35 |
| 9 | 18 | 1.06 | 1.33 | 1.65 | 2.33 | 1.12 | 1.63 | 5/35 |

Smallest `r1` a window ever reaches, over all k: min 1.06, **median 1.27**, p90
1.39, max 2.33. **25 of 35 windows never get below 1.20 at any point.** That is
the whole failure: 1.20 sits below the median of the distribution it is
thresholding.

Two structural facts fall out of this table and drive everything below.

1. **`r2 ≡ 1.000 at k = 3.** When the number of sub-windows equals
   `CALIB_AGREE_N`, "the tail" and "all of them" are the same set. With
   `CALIB_MIN_SUBWINDOWS = CALIB_AGREE_N = 3` the earliest exit is guarded by
   test (1) **alone**, and at k = 4 test (2) is still 3-of-4 (median 1.08). The
   ringdown guard that `_calib_stationary`'s docstring is built around does not
   exist at the two exit points where the risk is largest.
2. **`r1` does not separate ringdown from noise.** A `tau = 16 s` decay changes a
   2 s RMS by `e^(2/16) = 1.133` per window and gives `r1 = e^(4/16) = 1.284`.
   The bench's ambient `r1` is *larger* than that at the median. Any tolerance
   that fires here admits a ringdown by construction.

## 3. The sweep — shipped structure, one shared constant

35 complete windows. "mean cal" counts non-firing windows at the 20 s ceiling.

| tol | fired | mean cal | median exit | med skew | p90 | **max** | worst HIGH |
|---|---|---|---|---|---|---|---|
| **1.20** (today) | 10/35 | 16.98 s | 8.01 s | 5.7 % | 10.9 % | 12.7 % | 12.7 % |
| 1.25 | 14/35 | 15.83 s | 7.00 s | 6.6 % | 12.1 % | 27.2 % | 27.2 % |
| 1.30 | 20/35 | 13.83 s | 7.00 s | 7.7 % | 19.1 % | 27.2 % | 27.2 % |
| **1.35** | **29/35** | **11.49 s** | **8.01 s** | **8.0 %** | 19.4 % | **29.0 %** | 27.2 % |
| 1.40 | 32/35 | 10.52 s | 8.01 s | 7.8 % | 18.2 % | 29.0 % | 27.2 % |
| 1.45 | 34/35 | 9.21 s | 6.02 s | 8.9 % | 17.1 % | **37.3 %** | 37.3 % |
| 1.50 | 34/35 | 8.01 s | 6.01 s | 9.4 % | 26.5 % | **40.7 %** | 40.7 % |
| 1.80 | 34/35 | 7.21 s | 6.01 s | 9.6 % | 26.5 % | 40.7 % | 40.7 % |

**1.45 measurably crosses the line.** The recommendation is 1.35 rather than 1.40
because the two are indistinguishable on the bench (identical 29.0 % worst skew,
1 s of mean calibration between them) and 1.40 is the *exact* value of the second
test's statistic in the simulator's 6 V/s shock scenario — `_calib_stationary`'s
docstring records `r2 = 1.400`, and the comparison is `<=`. A shared constant of
1.40 disarms, by a coincidence of rounding, the guard that keeps
`runaway-baseline` at 17.6 %. 1.35 keeps 0.05 clear of it for 1 s.

The two worst cases at 1.35 are both real, and they fail in opposite directions:

| skew | run | exit | what the window did |
|---|---|---|---|
| −29.0 % | `20260804_140508` ep1 (8 ch) | 8.01 s | floor **rising** 0.0108 → 0.0200 V across the 20 s |
| +27.2 % | `20260806_160952` ep0 (4 ch) | 8.00 s | **ringdown**, ch3 0.758 → 0.52 V, decay 1.48 |
| +22.3 % | `20260803_171159` ep0 | 6.01 s | ringdown, decay 1.57 |

The high-side error is the dangerous one — the 1.8x runaway breaker goes slow and
the 0.35x lock detector goes permissive — and it is capped at **27.2 %** at 1.35.

## 4. Is the test buying anything at all? Yes

Null: take the median of *any* three adjacent sub-windows, no agreement test, and
compare to the same window's 20 s answer (245 positions).

| exit | median | p90 | max |
|---|---|---|---|
| k=3 (6 s) | 11.6 % | 39.4 % | **71.2 %** |
| k=4 (8 s) | 12.2 % | 28.2 % | 49.6 % |
| k=5 (10 s) | 10.8 % | 25.3 % | 54.1 % |
| all positions | 9.5 % | 27.5 % | 71.2 % |

4.5 % of arbitrary positions land past 35 %. **"Exit at 6 s, no test" gives 71.2 %
worst case on the bench** — against the simulator sweep giving 74.3 %
(`CLAUDE.md` § *Why the calibration window is 20 s*)
for `CALIBRATION_S = 5 s`. The two agree closely, which is independent evidence
that the bench replay and the simulator sweep are measuring the same thing.
Against that null, tol = 1.35 turning 71.2 % into 29.0 % is the test doing real
work; it is just not doing *enough* of it.

## 5. Sample rate — the briefing's worry is measurably a non-issue

Inside `CALIBRATING` the stream is **344–352 Hz on every 2026-08-03 and
2026-08-04 episode and on 4 of the 5 2026-08-06 runs**. Only
`20260806_171500` runs 1107–1113 Hz. (The 18 Hz figure is the closed-loop
`DAMPING` rate, not a calibration rate.)

Decimating that one high-rate run — `bp` is band-limited to 3 Hz, so this only
removes samples — moves the smallest tolerance it ever needs by ≤ 0.005 over a
**32x** rate change on three of four episodes:

| episode | 1113 Hz | 371 Hz | 139 Hz | 35 Hz |
|---|---|---|---|---|
| ep0 | 1.292 | 1.290 | 1.289 | 1.294 |
| ep1 | 1.308 | 1.307 | 1.305 | 1.299 |
| ep2 | 1.351 | 1.354 | 1.356 | 1.420 |
| ep3 | 2.326 | 2.325 | 2.325 | 2.099 |

The statistic is set by the **~2 cycles of 1 Hz motion** in a 2 s window, not by
the sample count. Nothing in the recommendation needs to change with baud rate.
Confirmed end-to-end by §7: the recommendation's numbers on the ~350 Hz subset
(n=32) and the ~1112 Hz subset (n=3) are the same within the scatter.

## 6. `.all()` over eight channels is a separate blocker

`_calib_stationary` tests `.all()` over `N` channels. On the eight-channel builds
that includes a4/a6/a7, whose 2 s RMS is their own out-of-band noise:

| run | ch0–3 needs | ch0–7 needs |
|---|---|---|
| `20260804_140508` ep0 | 1.20 | **2.26** |
| `20260804_111021` ep0 | 1.35 | **1.73** |
| `20260804_170145` ep1 | 1.16 | **1.68** |
| `20260806_171500` ep0 | 1.29 | **1.51** |

At the recommendation, restricting to solid channels fires 9/10 of the
eight-channel windows; testing all eight fires 3/10. **On v10/v11, retuning the
tolerance alone will not make `fast-calib` fire** — the test has to run over the
channels whose baseline is the optic. `harness.py` already knows this
(`KNOWN LIMIT fast-calib`, line 939).

## 7. If 1.21x of margin is not enough — and it probably is not

The 29.0 % worst case is not a single outlier: leave-one-out over the 35 windows
gives 27.2–29.0 %. It is a property of the bench, and 35 windows is thin
sampling of a tail. Two changes, both of which halve it, with test (2) pinned at
its present 1.20 (so the simulator's `r2 = 1.400` shock still runs the ceiling):

| variant | fired | mean cal | median exit | p90 skew | **max** | worst HIGH |
|---|---|---|---|---|---|---|
| shipped, tol 1.35 | 29/35 | 11.49 s | 8.01 s | 19.4 % | 29.0 % | 27.2 % |
| `MIN_SUBWINDOWS=4`, tol1 1.50 | 34/35 | 10.24 s | 8.01 s | 13.0 % | 29.0 % | 27.2 % |
| **`MIN_SUBWINDOWS=5`, tol1 1.50** | **33/35** | **12.47 s** | 10.01 s | **12.4 %** | **14.2 %** | **13.7 %** |
| `MIN_SUBWINDOWS=6`, tol1 1.50 | 33/35 | 13.79 s | 12.01 s | 12.5 % | 14.2 % | 13.6 % |
| `AGREE_N=4`, tol1 1.40 | 23/35 | 14.12 s | 8.04 s | 12.9 % | 14.0 % | 13.6 % |

**`CALIB_MIN_SUBWINDOWS = 5` is the change that matters**, and the reason is §2's
first structural fact: at a floor of 5 sub-windows the median-of-all in test (2)
finally contains windows the tail does not, so the ringdown guard is armed at the
earliest legal exit instead of being identically 1.000 there. Worst skew halves,
27.2 % → 13.7 % on the high side, margin 1.21x → 2.5x, for 1.0 s more mean
calibration than tol 1.35 alone. It is also exactly the recorded advice —
"scale the ceiling off Q" — since 10 s ≈ 0.63 tau is where a median over
sub-windows starts to bound a 16 s decay.

Two caveats, stated because they are the cost:

- The earliest exit becomes 10.01 s, and `harness.py:925` asserts
  `t_calib <= CALIBRATION_S * 0.5` = 10.00 s. **`FIXED fast-calib` would fail by
  14 ms.** That assertion's 0.5 factor is a design intent, not a measurement, and
  would have to be revisited (or `CALIB_SUBWINDOW_S` trimmed) — it is a judgement
  call, not something the data settles.
- If the second test is made **disjoint** (tail vs the windows *before* it, not
  vs all of them) the same protection arrives at a floor of 4 sub-windows:
  `MIN=4`, tol1 1.50, disjoint tol2 1.30 gives 33/35 fired, mean 11.67 s, max
  skew **13.7 %**, earliest exit 8.01 s — better than `MIN=5` on every axis and
  under the 10 s assertion. It needs three lines in `_calib_stationary`, not one
  constant.

## 8. What protects a calibration shorter than tau from a ringdown

At tol 1.35 the median exit is 8.0 s ≈ 0.5 tau, and the answer is: **structurally,
nothing.** §2 shows test (1) cannot see a ringdown (1.284 against an ambient
median of 1.43) and test (2) is 1.000 at 6 s and 1.08 at 8 s. What bounds the
damage is arithmetic rather than detection — a ringdown is a slow multiplicative
drift, so exiting at `t_e` instead of 20 s costs roughly
`e^((T_eff − t_e)/tau)`, ~1.3x for an 8 s exit, not an unbounded error. That is
the 27.2 % measured on `20260806_160952` ep0, and it is why the sweep's worst case
sits at 29 % rather than at the null's 71 %.

If that argument is uncomfortable — and it should be, at 1.21x — take
`CALIB_MIN_SUBWINDOWS = 5` from §7, which replaces the arithmetic bound with an
actual detector and measures 14.2 %.

## 9. Where the briefing and the data disagree

1. **"On hardware it has never once fired" is false.** `fast-calib` fired once, on
   v10, 2026-08-06: `bench/20260806/v10_4ch_3kicks.log:15549`, "baseline set in
   6.0s (sub-windows agreed)", storing 0.7427/0.4404/0.8563/0.4656 V. That is
   `data/20260806_155848_fast_lock.csv` episode 1, and the replay reproduces it
   to 0.15 %. The claim is true only of *first* calibrations.
2. **At 1.20 it would have fired on 10 of 35 recorded windows** — but **7 of 13
   post-fault** windows against **3 of 22** cold ones. Today's constant is not
   inert; it is biased towards exactly the wrong half, the windows that follow a
   kick and are most likely to be ringing. The one hardware firing is an instance
   of that bias.
3. **Rates:** calibration streams at ~350 Hz on 08-03 *and* 08-04, not just 08-04
   (§5); only one 08-06 run is at 1112 Hz. "A sub-window now contains ~3x more
   samples" is true and measurably irrelevant.
4. `20260806_160826` is 6.00 s of `CALIBRATING` and nothing else — an aborted run,
   not an early exit. It closed only 2 sub-windows.

## 10. What this does and does not buy

Mean calibration over the recorded windows **20.0 → 11.5 s** at tol 1.35, or
12.5 s with `MIN_SUBWINDOWS = 5`. Bench time-to-lock was 31.6 s (v10, 20 s of it
calibrating, 11.6 s after gain), so this takes it to roughly **23 s**. The
project's "lock within 10 s including calibration" target is **not** reached by
retuning this constant; calibration stops being the largest single item and the
11.6 s lock acquisition becomes it.

---

## Reproducing

```
.venv/bin/python analysis/calib_tol.py            # ~90 s cold, ~5 s cached
.venv/bin/python analysis/calib_tol.py --rebuild  # re-extract from the CSVs
```

Extracted `CALIBRATING` windows are cached in
`analysis/out/calib_tol_windows/*.npz` and every table above is streamed to
`analysis/out/calib_tol_*.csv` (`episodes`, `ratio_vs_time`, `decimation`,
`null`, `sweep`, `variants`, `recommended`).
