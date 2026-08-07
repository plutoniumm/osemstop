# Operational uptime over 24 h under Poisson shocks

Reproduce every table here with `python3 analysis/uptime.py` (standard library
only — it does **not** need the `ligo` conda env). Outputs land in
`analysis/out/uptime_*.csv`, including `uptime_trials.csv`, one row per
Monte-Carlo trial, per CLAUDE.md's standing practice.

---

## 1. Assumptions, stated before the answer

1. **Uptime = time in `DAMPING`.** `CALIBRATING` forces the gain to zero,
   `FAULT` freezes all gains and holds every output at bias. Neither removes
   energy, so both are downtime. *(Definition, from the brief.)*
2. **`LOCK_SUSTAIN_S = 5.0` counts as uptime.** *(Choice.)* The gain is applied
   and the loop is damping throughout that window; it delays only the
   announcement. It also never appears in the `state` column — the CSV says
   `DAMPING` — so counting it as downtime would put the model at odds with the
   ground truth it is calibrated against.
3. **λ counts *trip-capable* shocks, not all disturbances.** *(Assumption, and
   the load-bearing one.)* Every scripted kick that landed while the rig was
   `DAMPING` tripped the breaker — v4 2/2, v9 3/3, v10 3/3 (measured, from the
   CSVs). A per-shock trip probability is therefore **not identifiable** from
   this data, so it is set to 1 and λ is redefined as the rate of shocks of that
   magnitude. All the "not every door slam trips it" uncertainty lives in the λ
   sweep instead of in an invented `p_trip`.
4. **A shock arriving in `FAULT` or `CALIBRATING` is absorbed at zero marginal
   cost.** *(Measured, not assumed.)* v4's third kick at t = 155.0 s landed in
   `CALIBRATING` and cost nothing
   (`bench/20260804/v4_4ch_3kicks.log`, `data/20260804_111807_fast_lock.csv`).
5. **A shock is not measured to trip once.** It produces a *burst* of `m` fault
   entries separated by `g` seconds of `DAMPING`, both measured per version
   (§3). Burst length is modelled geometric with mean `m`. *(Assumption on the
   distribution; `m` and `g` are measurements.)*
6. **λ is swept 1–100/h and no single value is privileged.** The headline is
   λ = 10/h — one trip-capable shock every 6 minutes. *(Assumption. It is the
   geometric middle of the swept range and 4.5–7× below the bench provocation
   rate; there is no measurement of the true rate on this rig.)*
7. **No hardware, no controller run.** Everything here is offline arithmetic on
   logs already on disk.

---

## 2. Ground truth — recomputed, not taken on trust

Time-weighted state fractions, straight out of the raw CSVs
(`analysis/out/uptime_bench.csv`):

| run | span | DAMPING | FAULT | CALIB | faults | bursts | m | gap g |
|---|---|---|---|---|---|---|---|---|
| v4 `20260804_111807` | 197.8 s | 64.6% | 5.1% | 30.3% | 2 | 2 | 1.00 | — |
| v9 `20260804_135853` | 237.7 s | 83.1% | 8.5% | 8.4% | 4 | 3 | 1.33 | 4.04 s |
| v10 `20260806_155848` | 237.7 s | 74.1% | 15.0% | 10.9% | 7 | 3 | 2.33 | 4.08 s |
| v11 `20260806_171500` | 237.7 s | 42.3% | 25.3% | 32.4% | 12 | 2 | 6.00 | 4.01 s |
| v12 `20260806_200822` | 244.5 s | 77.5% | 14.4% | 8.2% | 7 | 4 | 1.75 | 12.03 s |
| v12 lag2 `20260806_200158` | 199.4 s | 54.1% | 25.2% | 20.7% | 10 | 1 | 10.00 | 4.03 s |

v9/v10/v11/v12 reproduce `versions.md` § *v12* exactly on the fractions. The
fault counts differ by one on v10 (7 CSV state transitions vs 8 in the table)
and v11 (12 vs 11); the CSV transition count is what this model uses, since it
is what the state column actually did.

**v4's 75% is a sample-count artefact, not a time fraction.** `CALIBRATING` is
21053 of 27932 samples = **75.4% of samples**, but only **30.3% of time**. The
stream rate is not constant across states: during `CALIBRATING` no `SET` traffic
shares the wire, so the run samples at 351 Hz, against 26 Hz while `DAMPING`
(`README` § *Serial protocol*). Anything sample-weighted over-counts
`CALIBRATING` by 2.5×.

**Burst structure.** A `DAMPING` interval shorter than `FAST_REFAULT_S = 15.0 s`
is the same disturbance still ringing — the controller's own definition — so the
fault ending it belongs to the *same* burst; longer is a genuine re-engagement.
The gaps are strikingly quantised: v9 4.04 s, v10 4.08–4.12 s, v12 12.03 s. v12's
loop gets three times as long back in the loop between re-trips as v9's does.

---

## 3. The model

An alternating renewal process over the state machine's own constants,
Monte-Carlo'd (2000 trials × 24 h × 7 λ × 5 versions, seed 20260807) because
`warm-restart` is path-dependent and a closed form would have to approximate it
away.

Per fault entry: `FAULT_CLEAR_SUSTAIN_S = 5.0 s`, then either a warm restart
(0 s) or a full `CALIBRATION_S`. Which one is decided by transcribing
`_why_reuse` (`osem.v9.py:707`, `osem.v12.py:2158` — identical):

- refuse if baseline age > `BASELINE_MAX_AGE_S = 300 s`;
- refuse if `reuse_count >= MAX_BASELINE_REUSE = 4`, where the counter resets on
  any engagement ≥ `FAST_REFAULT_S = 15 s`;
- otherwise reuse.

Version constants (`analysis/out/uptime_params.csv`):

| | channels | `CALIBRATION_S` | `warm-restart` | sat latch | m | g |
|---|---|---|---|---|---|---|
| **v0** | 1 | **8.0** | no | **yes** | 1.00 † | — † |
| **v4** | 4 | 20.0 | no | no | 1.00 | — |
| **v9** | 4 | 20.0 | yes | no | 1.33 | 4.04 s |
| **v12** | 8 | 20.0 | yes | no | 1.75 | 12.03 s |
| **v13** | 8 | 20.0 | yes | no | 1.75 ‡ | 12.03 s ‡ |

v0 and v4 are deleted; their constants come from git history and the commits are
named in the script so they can be checked: `git show b486cd0^:osem.v0.py`
(line 178), `git show 912a9c7^:osem.v4.py` (line 294).

† **v0 has no bench run of its own.** The 2026-08-03 session ran v1, v2 and v3
(`versions.md` § *On the bench, 2026-08-03*). v0's `m` and `g` are **substituted
from v4**, the nearest measured version that also lacks `warm-restart`. That
substitution is stated in `uptime_params.csv` in the `m_source` column, not
hidden.

‡ **v13 is v12.** `main()` raises `SystemExit`; the observer path
(`_run_as_v12`, opt-in only) runs v12's control law, v12's gains and v12's
interlocks, *imported not copied*, plus a three-mode observer that produces
**zero force** (`osem.v13.py:98`, `versions.md` § *v13*). There is no timing
constant it can change, so its uptime is v12's — and the script seeds it off
v12's random stream deliberately, so the row is byte-identical rather than
differing by Monte-Carlo noise that would read as a real effect. As shipped, its
uptime is **0%**: it will not start.

### Validation

The model run at each bench run's own kick rate over its own span
(`analysis/out/uptime_validation.csv`):

| run | λ | measured DAMPING | model | Δ |
|---|---|---|---|---|
| v4 | 54.6/h | 64.6% | 66.4% | +1.8 |
| v9 | 45.4/h | 83.1% | 84.4% | +1.4 |
| **v10** | 45.4/h | 74.1% | **79.3%** | **+5.2** |
| v12 | 73.6/h | 77.5% | 79.0% | +1.5 |

v10 is out-of-sample — it is scored on v9's machinery with v10's own `m` and `g`,
and no v10 row is fitted. The model is systematically **optimistic by 1.4–5.2 pp**
over a 200–240 s window, because the bench kicks were at fixed times while the
model's are Poisson. Read every number below as an upper bound of that order.

A closed-form alternating-renewal bracket is carried in
`uptime_sweep.csv` (`closed_form_lo_pct`/`hi_pct`). For the two versions without
`warm-restart` it is exact and agrees with the Monte-Carlo to 0.02 pp (v4 at
λ = 10: 93.47 vs 93.49).

---

## 4. Results — 24 h uptime vs λ

`analysis/out/uptime_sweep.csv`. Monte-Carlo s.d. across trials is 0.07–0.66 pp.

| λ (/h) | v0 † | v4 | v9 | v12 | v13 |
|---|---|---|---|---|---|
| 1 | 99.6% | 99.3% | 99.3% | 99.2% | 99.2% |
| 2 | 99.3% | 98.6% | 98.7% | 98.5% | 98.5% |
| 5 | 98.2% | 96.6% | 97.1% | 96.8% | 96.8% |
| **10** | **96.5%** | **93.5%** | **95.2%** | **94.7%** | **94.7%** |
| 20 | 93.2% | 87.8% | 92.5% | 91.5% | 91.5% |
| 50 | 84.6% | 74.2% | **86.7%** | 85.0% | 85.0% |
| 100 | 73.4% | 59.0% | **79.3%** | 77.6% | 77.6% |

† v0 assumes the saturation latch **never fires** — an upper bound, see §6.

Headline, λ = 10/h (`analysis/out/uptime_headline.csv`):

| | uptime | downtime | FAULT | CALIB | faults/24 h | recals/24 h |
|---|---|---|---|---|---|---|
| v0 † | 96.5% | 50 min | 1.4% | 2.2% | 232 | 232 |
| v4 | 93.5% | 94 min | 1.3% | 5.2% | 225 | 225 |
| **v9** | **95.2%** | **69 min** | 1.8% | 3.0% | 304 | 128 |
| v12 | 94.7% | 77 min | 2.2% | 3.1% | 387 | 133 |
| v13 | 94.7% | 77 min | 2.2% | 3.1% | 387 | 133 |

---

## 5. Sensitivity to λ, and three inconvenient results

**λ is worth more than the version.** Across 1→100/h, uptime moves 26 pp on v0,
40 pp on v4, 20 pp on v9 and 22 pp on v12. Across versions at fixed λ it moves
0.4 pp (λ = 1) to 20 pp (λ = 100). Below λ ≈ 5/h **the four live versions are within
0.6 pp of each other** and the choice does not matter on this axis at all.

**v9 beats v12, at every λ, by 0.1–1.7 pp.** Reported as measured, not tuned
away. The cause is `m`: 1.33 faults per shock on v9 against 1.75 on v12. But the
comparison is *not controlled* — `versions.md` § *v12* says so directly: v9 took
three scripted kicks, v12 five by hand at unrecorded strength, hard enough to
clip every channel. The controlled comparisons in that file are v12-vs-v11 and
v12-vs-itself across the lag change, and v12 wins both. **Do not choose v9 over
v12 on the strength of this table.**

**`warm-restart` only pays above ~12/h.** `_why_reuse` refuses reuse once the
floor is older than `BASELINE_MAX_AGE_S = 300 s`. Below λ = 3600/300 = **12/h**
the mean gap between faults exceeds that, so nearly every fault recalibrates
anyway and v9/v12 collapse to v4's per-fault cost:

| λ (/h) | v9 recals/fault | v12 recals/fault | v12 − v4 uptime |
|---|---|---|---|
| 1 | 0.69 | 0.54 | −0.1 pp |
| 5 | 0.54 | 0.43 | +0.2 pp |
| 10 | 0.42 | 0.34 | +1.2 pp |
| 20 | 0.30 | 0.25 | +3.7 pp |
| 100 | 0.11 | 0.12 | +18.7 pp |

The feature that most distinguishes the modern supervisor is worth nothing in a
quiet lab and 19 pp in a violent one.

---

## 6. v0 — stated plainly

v0 **cannot be estimated from a run of its own**; it has none. Two things are
substituted and both are labelled: its `m`/`g` come from v4 (§3), and the only
hardware evidence for its class is v1/v2/v3 on 2026-08-03.

More importantly, **v0's `FAULT` is an absorbing state**. `saturated_flag` is
recomputed only inside `check_runaway()`, which does not run in `FAULT`, so one
saturation trip ends the shift until a human restarts it — the bug is commented
in the source (`git show b486cd0^:osem.v0.py`, line 615) and was reproduced on
hardware: v2 latched ~50 s on a 31-sample pinned run, v1 cleared in 5.0 s from a
28-sample one (`versions.md` § *On the bench, 2026-08-03*).

The per-fault probability that a trip is a *saturation* trip is **not measured** —
one latch and one clean recovery, on two different control laws, is not a rate.
So it is swept (`analysis/out/uptime_v0_latch.csv`):

| p_latch | λ = 1/h | 10/h | 100/h |
|---|---|---|---|
| 0.00 | 99.6% | 96.5% | 73.4% |
| 0.10 | 38.5% | 4.2% | 0.4% |
| 0.25 | 16.7% | 1.7% | 0.2% |
| 0.50 | 8.2% | 0.8% | 0.1% |

**v0's headline 96.5% is therefore fiction unless the latch probability is
exactly zero.** At any p_latch ≥ 0.1 it is the worst version by an order of
magnitude. Two further caveats on the 96.5% itself: it is high only because
`CALIBRATION_S = 8.0` rather than 20.0, and v0 drives **one channel**
(`ENABLE_CHANNEL = [True, False, False, False]`), which damps the whole body in
≈70 s against ≈17 s for four (`provenance.md` §3/§4). Its uptime seconds are not
worth the same as the others'.

---

## 7. What would actually buy uptime

One constant changed at a time on v12, at λ = 10/h
(`analysis/out/uptime_counterfactual.csv`). **Timing-model counterfactuals only
— none has been run, and each has a safety argument this file does not make.**

| change | uptime | Δ |
|---|---|---|
| v12 as shipped | 94.7% | — |
| `FAULT_CLEAR_SUSTAIN_S` 5.0 → 2.0 | 96.0% | +1.3 |
| `CALIBRATION_S` 20 → 2 on the *recovery* path too | 97.3% | +2.7 |
| `BASELINE_MAX_AGE_S` 300 → 3600 | 97.1% | +2.4 |
| all three | 99.0% | +4.3 |

This matches `versions.md` § *Still open after v12* item 5 from the other
direction: at the bench's own λ the whole fault cost is the 5 s hold (7 × 5 = 35 s
= 14.3% against 14.4% measured), but over 24 h at a realistic λ the **larger**
share is `CALIBRATING`, not `FAULT` — 3.1% against 2.2% on v12 — and it is there
because the 300 s age limit expires between faults. `persist-baseline` already
solves the cold start (20 s → 2 s, worth 0.02% over 24 h and therefore
irrelevant); extending it to the *recovery* path is worth 2.7 pp, a hundred times
more.

---

## 8. What to run

At λ ≤ 5/h it does not matter: every live version is 96.6–97.2% up and the
spread (0.6 pp) is well inside the model's own 1.4–5.2 pp optimism. At λ ≥ 20/h the spread is
real and v4 is the one to avoid — it pays `FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S`
= 25 s on every single fault, 20 pp behind v9 by λ = 100/h.

**Run v12.** Not because it wins this table — it does not — but because
that gap (0.1–1.7 pp) is measured against an uncontrolled kick schedule, v12 carries
`sample-guard` (v11's log has ten out-of-range rows, one worth 42 V demanded into
a 0.5 V rail), and it is the only version that drives all eight OSEMs. Uptime is
not the axis this choice turns on.
