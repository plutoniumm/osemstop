# Full MIMO / modal damping — the design

`research.md` item 2. Written 2026-08-06, against `osem.v12.py` and `tune.py`,
and revised the same day when the 8-coil closed-loop measurement landed.

Reproduce every number with

    .venv/bin/python analysis/mimo_design.py

which reads only `bench/20260804/dcmatrix.log`, `sim/server.py`'s measured
constants, `analysis/out/*.csv` and `gains.json`. It opens no port and runs no
suite. Numbers that still need a measurement are marked **HOLE**.

---

## 0. The short version

**MIMO is not required to damp this rig, and the measurement now says so
directly.** The two modes' residue vectors are *aligned*, not anti-parallel —
`cos(P₀, P₁) = +0.672`, sign pattern `[−, −, +, −]` in both modes — so one
scalar gain per channel, with the sign of that channel's own residue, adds
damping to both modes at once. §2 shows this is not luck: it is forced by the
sensor and coil being the same physical unit, and the measurement confirms the
prediction on all four usable channels.

So the case for MIMO has to be made on something else. Here is that case,
honestly ranked, after the measurement:

| | argument | status |
|---|---|---|
| 1 | **graceful degradation** — a demoted channel is projected out and the survivors re-hit the same modal target, instead of the target drifting wherever the surviving gains happen to point | **stands.** The only argument that is structural rather than numerical, and the reason the rest of this document is mostly about interlocks |
| 2 | **headroom** — min-norm allocation is the ℓ₂-smallest coil command producing a given modal force | **stands, unquantified.** The motivation is now hard: v12's shipping gains peak at **0.2998 V against a 0.250 V half-window, 20% over budget**. But see the sting in §5.2 |
| 3 | exact mode decoupling, `L_mn = 0` | stands, second-order in the poles |
| 4 | ~~ratio control — diagonal can only reach `Δγ₀/Δγ₁` in a narrow window~~ | **DEAD.** Measured `\|P₀ⱼ\|/\|P₁ⱼ\|` spans **1.45 to 32.25**, a 22× window. Diagonal can already put the damping almost anywhere it likes. This was the strongest quantitative argument before the measurement and it did not survive it |

**The sting (§5.2).** v12's peak is 0.2998 V of which **Ki is 0.1135 V — 38%,
to dissipate nothing** (the integrator sits behind the 0.4 Hz highpass, so
`i = −Ki·bp` and `bp` is DC-free by construction; and its damping contribution
computes *negative* at both modes, §2.2). **Deleting Ki recovers more headroom
than any allocation scheme can, and needs no matrix at all.** MIMO's headroom
argument is what is left *after* that, and it has not been sized.

**The blocker (§6).** The fit returned `f = 0.7149 Hz (Q = 288)` and
`0.9941 Hz (Q = 2334)`. Q = 2334 is not physical for this pendulum, and the loop
was damping throughout, so any pole it moved should be *more* damped, not 50×
less. **Mode frequencies and damping ratios are now holes**, and every quantity
in /s in this design is a hole with them. §7 says how to fill them, and the
answer is a ringdown, not a sweep.

---

## 1. What is measured, what is inferred, what is a hole

### Measured on this rig, and trustworthy

| quantity | value | source |
|---|---|---|
| **per-mode residue diagonal `P_mj`** | table in §2.5 | `data/20260806_182231_tune_raw.csv` → `gains.json`, 717 420 samples, 880 Hz, 0 frames dropped |
| **actuator peak, shipping gains** | P 0.1279 + I 0.1135 + D 0.0584 = **0.2998 V** of a 0.250 V half-window | same; derived from drive amplitudes and filter gains, **not** from the mode fit, so unaffected by the frequency caveat |
| 8×8 DC actuation matrix | counts/V, one coil stepped at a time | `bench/20260804/dcmatrix.log` |
| per-coil DC diagonals | −105 / −68 / **+213** / −51 / +8 / −7 / +4 / +7 | same |
| mode shapes, amplitude and sign (passive) | `SHAPE_8` | cross-spectrum referred to a0 over four 8-channel logs, `sim/server.py:400-420` |
| mode frequencies (passive) | 1.046, 1.657 Hz | `analysis/out/modes.csv` |
| envelope decay γ, gain off | **0.149 – 0.166 /s** ⟹ **Q ≈ 20–22** | `analysis/out/damping_vs_gain.csv` |
| in-band baselines, 8 ch | 0.3104 / 0.1745 / 0.4436 / 0.1559 / 0.0052 / 0.0896 / 0.0090 / 0.0022 V | `data/20260806_161715_fast_lock.csv` |
| actuation rank, 4 coils | σ = 1.000 / 0.154 / 0.058 / 0.043 | `osem.v6.py` stepped-sine rank check |

### Computed, needing no measurement

`ρ_m ≡ −Im(c_p(iω_m))/(2ω_m)` — the controller's own half of the damping map,
from v12's corner frequencies and its rate alone:

| loop rate | ρ₀ (1.046 Hz) | ρ₁ (1.657 Hz) | Ki factor ₀ | Ki factor ₁ |
|---|---|---|---|---|
| **v12, 100 Hz** | **+0.410779** | **+0.311967** | **−0.011154** | **−0.020022** |
| v9/v10, 4 coils, 23.5 Hz | +0.366055 | +0.245110 | −0.009870 | −0.015740 |
| v10, 8 coils, 12.5 Hz | +0.317893 | +0.185941 | −0.008351 | −0.011736 |

Both `ρ_p > 0` — a gain with the sign of the residue damps. **Both Ki factors
are negative at both modes**: the integrator subtracts damping, structurally,
before anything is measured. Kd factors at 100 Hz are +1.583 and +2.932.

*(These are evaluated at the passive-spectra frequencies. They move with `f`;
`ρ₀` shifts 23% between 100 Hz and 12.5 Hz, so the frequency hole propagates
here too.)*

### Holes

| hole | what | who fills it |
|---|---|---|
| **H1** | `f_m`, `ζ_m` — **the fit's values are not usable** | a ringdown, §7 |
| **H2** | the **rows** of `R_m`, not only its diagonal — i.e. `Φ` and `A` | the same run, re-fitted after H1; `tune._bootstrap_P` currently keeps only `np.diag(...)` |
| **H3** | is a5 a dead sensor or a dead coil? | row a5 of `R_m`, off-diagonal, §4.1 |
| **H4** | every quantity in /s | H1 |
| **H5** | the actual headroom `‖u_MIMO‖_∞ / ‖u_diag‖_∞` | H2 |

---

## 2. Why diagonal already damps both modes

### 2.1 The colocation identity

Each OSEM is one unit — LED, photodiode and coil, colocated and coaxial, reading
and pushing the same flag — so sensing and actuation share a geometry
(`sim/server.py:322` states this for the four-OSEM body). With `ψ_mj` the
mechanical mode shape, `σ_j` the sensor gain and `g_j` the coil gain:

    s_m[j] = σ_j ψ_mj          a_m[j] = g_j ψ_mj      (SAME ψ, by colocation)

`tune.py` measures `R_m = s_m a_mᵀ` and every gain rests on its diagonal
(`tune.py:86-91` — the diagonal carries none of the rank-one split ambiguity):

    P_mj = R_m[j][j] = σ_j g_j ψ_mj²  =  λ_j · ψ̂_mj² · μ_m                (1)

`ψ̂_mj²` is a square. Therefore

    sign(P_0j) = sign(P_1j) = sign(λ_j)   for every channel j.            (2)

### 2.2 The consequence

Added damping is linear, one equation per mode (`tune.damping_map`):

    Δγ_m = ρ_m · Σ_j P_mj · Kp_j                                          (3)

With `ρ_m > 0` and (2), choosing `Kp_j` with the sign of `λ_j` makes **every
term of both sums positive**:

    Kp_j = sign(λ_j)·k_j , k_j ≥ 0   ⟹   Δγ₀ > 0 AND Δγ₁ > 0              (4)

That is the classical colocated-damping result. It answers the framing question
"could `(φ₀⊙b₀)` and `(φ₁⊙b₁)` be near anti-parallel?" with **no — not on a
colocated rig**: by (2) they lie in the same orthant and their inner product is
a sum of positive terms. **Feasibility was never the problem.**

### 2.3 The prediction was testable, and it passed

(2) is a falsifiable statement about a measurement that had not been taken when
it was written. The 2026-08-06 run took it:

| ch | mode 0 | mode 1 | signs agree? |
|---|---|---|---|
| 0 | −7.851 ± 0.238 | −0.463 ± 0.124 | ✓ |
| 1 | −8.867 ± 0.622 | −0.275 ± **2.549** | ✓ |
| 2 | **+13.573** ± 0.308 | **+2.055** ± 0.440 | ✓ |
| 3 | −5.282 ± 0.169 | −3.641 ± 0.393 | ✓ |

`[−, −, +, −]` in **both** modes — the same pattern the DC diagonal gives
(−105 / −68 / **+213** / −51) and the same pattern v12 ships
(−0.030 / −0.030 / **+0.010** / −0.030). Three independent routes, one answer.

**This is also the coil-map verification.** If `DAC_CHANNELS = [1,3,5,7,0,2,4,6]`
were wrong for channel `j`, then `P_mj = s_m[j]·a_m[j]` would pair a sensor at
one location with a coil at another, `ψ_mj²` would no longer be a square, and
(2) would break. It does not break on ch0–ch3. That closes `CLAUDE.md` item 2
step 2 for those four — a check that is invisible to the simulator and to every
interlock, answered for free by a measurement taken for another reason.

### 2.4 The argument that died

Before the measurement, the case for MIMO rested on the *ratio*: with
sign-correct gains, `Δγ₀/Δγ₁` is a weighted mean of `(ρ₀/ρ₁)|P_0j|/|P_1j|` and
so is confined to that set's range. On the passive mode shapes that range was
`[0.250, 1.000]`, a 4× window closed at the top — narrow enough that diagonal
might not reach a wanted ratio without a wrong-sign gain, which by (4) anti-damps.

Measured, the range is:

| ch | ch0 | ch1 | ch2 | ch3 |
|---|---|---|---|---|
| `\|P₀\|/\|P₁\|` | 16.97 | 32.25 | 6.61 | **1.45** |

    reachable Δγ₀/Δγ₁ ∈ [1.45, 32.25] × (ρ₀/ρ₁)     — a 22× window

**Not binding.** Diagonal can put the damping ratio essentially anywhere. The
argument is dead and this document will not resurrect it.

### 2.5 Where the residues do and do not agree with the passive shapes

Colocation says `(P_1j/P_0j) / (S_j1/S_j0)²` must be the same constant `μ₁/μ₀`
for every channel. Measured (§9 of the script):

| ch | `P₁/P₀` | `(S₁/S₀)²` | ⟹ `μ₁/μ₀` |
|---|---|---|---|
| 0 | 0.0589 | 1.0000 | 0.0589 |
| 1 | 0.0310 | 3.2600 | 0.0095 ← ch1 mode 1 is unmeasured |
| 2 | 0.1514 | 1.1366 | 0.1332 |
| 3 | 0.6894 | 4.0000 | 0.1724 |

Spread 18× over all four, **2.9× excluding ch1**, `μ₁/μ₀ ≈ 0.13`. A perfect
agreement would be 1.0×. The residual is dominated by the fit's frequency error,
which mis-splits power between the two modes — one more reason H1 has to be
filled before any of this becomes a gain.

---

## 3. The control law

### 3.1 Where the projection goes, and why it is free

v12 computes per channel, at `CONTROL_HZ`: `bp = lp(hp(volts))`, then
`vel = dsm((bp − bp_prev)/dt)`. `Φ⁺` is a constant real matrix and **every lane
runs the identical filter**, so `Φ⁺` commutes with the whole chain: projecting
raw volts then filtering is bit-identical to filtering then projecting.

v13 therefore **does not touch the filter bank**. It computes v12's per-channel
`bp` and `vel` exactly as v12 does and projects afterwards. That is not
tidiness — every per-channel interlock reads `bp` (gain schedule, runaway
breaker, lock detector, `baseline-floor`) or raw counts (`_rail_check`). Project
first and all of them would be reading a modal coordinate with thresholds
measured against something else. **Projecting after the filters is what lets §4
keep every interlock unchanged.**

The commuting argument needs the lanes in lock-step. v12 resets `dfilt` per lane
but deliberately never resets `hp`, `lp` or `dsm` ("filters and baseline are
deliberately carried across the gap", `_health`). **v13 must not add a per-lane
reset to `hp`/`lp`/`dsm`.**

### 3.2 The law

With `S` the surviving **sensor** set and `C` the surviving **coil** set (§4.1 —
different sets), `Φ_S` the `|S|×2` mode-shape matrix and `A_C` the `2×|C|`
coil→mode matrix (both **H2**):

    q̇̂   = Φ_S⁺ · vel_S                 modal velocity, 2-vector
    f    = −K_m ⊙ q̇̂                    2 modal forces, K_m ≥ 0
    u_C  = A_C⁺ · f                     min-norm coil command
    out  = bias + u,  then v12's clip, slew and anti-windup, unchanged

Under colocation these are not independent: `A_Cᵀ = diag(λ) Φ_C`, so the
measurement produces `Φ` and one scalar `λ_j` per channel, not two matrices.

Loop gain seen by the modes: `L = A_C A_C⁺ diag(K_m) Φ_S⁺ Φ_S · c_p(iω)`. With
the true `Φ` and `A` both products are the identity:

    L = c_p(iω) · diag(K_m)     — exactly diagonal, L_mn = 0
    Δγ_m = ρ_m · K_m                                                      (5)

Equivalently, v13 is v12 with `diag(Kp)` replaced by the full matrix

    K_matrix = A_C⁺ diag(K_m) Φ_S⁺        (|C| × |S|)                     (6)

which is the whole change and the whole hazard: **every row is dense, so every
sensor reaches every coil.**

### 3.3 Robustness to a wrong Φ — the margin, stated

With estimates, write `Φ̂⁺Φ = I + E` and `A Â⁺ = I + F`. Then
`L_mm = c_p K_m (1 + E_mm + F_mm + …)`, so mode `m` stays **dissipative** while

    |E_mm + F_mm| < 1                                                     (7)

Cross-leakage `E_mn` moves poles at second order (the same argument
`tune.py:92-97` makes for the split ambiguity) but cannot flip the sign of the
first-order damping. §5.3 measures the error at 14% median / 24% p90 — a factor
of ~4 inside (7). **A badly-conditioned modal estimator on this rig mis-shares
damping between the modes; it does not pump one.** That is what makes the
fallback in §4.4 a degradation rather than a fault.

It stops being true if `λ` has a wrong sign on a channel with real authority,
because then `F` is not a perturbation. §2.3 is the test, and it passes on
ch0–ch3.

### 3.4 Headroom: what min-norm promises

`A_C⁺ f` is by definition the **ℓ₂-smallest** `u` with `A_C u = f`. For any modal
force trajectory — including the one a diagonal law happens to produce — MIMO's
command has `‖u‖₂` no larger. Equality only if the diagonal command already lies
in the row space of `A_C`; the orthogonal part produces **no modal force at
all** and is spent driving the directions the 0.058 / 0.043 singular values live
in, i.e. spent on nothing the loop can see. With 4 coils and 2 modes that row
space is 2-dimensional in ℝ⁴, so a generic diagonal command wastes a substantial
fraction — **HOLE H5** for the actual number.

**The honest caveat:** the rail is per-channel and ℓ∞, and ℓ₂-min does not
minimise ℓ∞. Handled, not hidden:

- v13 uses a **weighted** pseudo-inverse `A_W⁺ = W A_Cᵀ (A_C W A_Cᵀ)⁻¹` with
  `W = diag(w_j)` **fixed for the run**, set from each channel's own headroom
  `min(bias_j − vmin_j, vmax_j − bias_j)`. Still one constant linear map,
  computed once, printed at preflight. Making `W` track live headroom would put
  a nonlinearity inside the loop and is refused on `research.md`'s rule that the
  control law stays classical and certifiable.
- the residual is left to v12's existing per-channel clip, slew limit and
  back-calculation anti-windup, all unchanged.

### 3.5 Regularisation, and what ch1 forces

Three decisions; only the third is free.

**(a) The number of modes is the truncation.** Fitting `nmode = 2` rank-one
residues *is* the rank-2 projection; there is no second SVD threshold anywhere.
The discarded directions of the measured 4×4 — **0.058 and 0.043** — sit at or
below that measurement's own noise: `dcmatrix.log` quotes mean |response| 66
counts in the a0–a3 block against 4 counts in the a4–a7 block, so the relative
noise floor is **4/66 = 0.061**, above both. **Truncation by measured noise, not
by a chosen threshold.** The DC 4×4 separately resolves *three* directions
(σ = 1.000 / 0.443 / 0.084 / 0.021 against a noise-equivalent 0.027) — 2 modes
plus `D_static`, which is exactly `tune.py`'s model `H = Σ R_m/(…) + D`. Three
independent methods, one structure.

**(b) `D_static` is kept, not truncated.** It is real (σ = 0.084 vs 0.027) and
does not resonate, so it contributes no damping and is excluded from `Φ` and
`A`. Its role in v13 is as a *check*: `SENSE_8`'s third row is that direction,
"orthogonal to both to 5e-4", and "any reading in it is sensors disagreeing"
(`sim/server.py:424`). §4.5 uses it.

**(c) The inverse is weighted per (mode, channel), not per channel.** This is
what the measurement forced:

> ch1 mode 0 is **−8.867 ± 0.622** — 14.3 σ, excellent.
> ch1 mode 1 is **−0.275 ± 2.549** — the error bar is **9× the value**.

ch1 is well measured at one mode and unmeasured at the other. `tune._boot_gate`
is `.any(axis=0)`, a per-**channel** test, and ch1 passes it on the strength of
mode 0 alone — after which its unmeasured mode-1 entry is free to set the
allocation at mode 1. `gains.json`'s own bootstrap shows exactly this failure:
`proposal.kp` p16 has ch1 at **−0.035** and p84 has it at **+0.035**. The sign
flips inside one standard interval.

So `Φ_S⁺` and `A_C⁺` are **inverse-variance weighted least squares**, not plain
pseudo-inverses:

    Φ_S⁺ = (Φ_Sᵀ Ω Φ_S + ε I)⁻¹ Φ_Sᵀ Ω ,   Ω = diag(1/σ²) per (sensor, mode)
    ε    = (2 σ̄)²                            matching tune._boot_gate's 2σ rule

This down-weights ch1's mode-1 row without discarding ch1, which a per-channel
gate cannot express. It is still closed-form, still computed once, still
auditable. **HOLE:** `σ` per row needs H2.

The ridge is deliberately weak. A subset needing a strong ridge should be
running diagonal instead (§4.4) — biasing an estimator until an ill-posed subset
stops complaining is the failure `baseline-floor` was written to avoid one
interlock over.

### 3.6 Ki and Kd

**Ki ships at zero, and the structure said so before the measurement agreed.**
The integrator sits behind the 0.4 Hz highpass, so `i = −Ki·bp` exactly and `bp`
is DC-free by construction: **Ki rejects no drift.** Its damping contribution
computes at `−0.011154` and `−0.020022` per unit `Σ_j P_mj Ki_j` — negative at
both modes (§1). And it now has a price tag: **0.1135 V of peak, 38% of a
0.2998 V total that is already 20% over the window.** v13 ships
`KI_MODAL = [0.0, 0.0]`.

**Kd stays sized by measured noise**, as `tune._size_kd` does it. Under MIMO the
D path runs on the modal coordinate, so the noise entering it is `‖Φ_S⁺‖ ×` the
per-channel floor — one more reason the conditioning gate is not cosmetic.

---

## 4. The hard problem: per-channel interlocks under a mixed command

### 4.1 Two sets, not one

v12 has a single `live = enabled & healthy`, used both for "this sensor's
opinion counts" and "this coil is driven". MIMO cannot share them, and **a5 is
the case that proves it.**

a5 was **rejected** by the residue gate on 2026-08-06 — but the residue diagonal
is a **product**, `P_mj = s_m[j]·a_m[j]`, and a product at noise does not say
*which* factor is at noise. The rest of the evidence points opposite ways:

- **sensor alive**: in-band baseline 0.0896 V, 51% of the median, kept by
  `baseline-floor`; visibly damped on the bench, 0.0860 → 0.0420 V over 40 s
  (v10, 2026-08-06); tracks the 1.046 Hz mode to within one FFT bin in all
  three 8-channel logs.
- **coil dead**: DC diagonal −7 counts/V against a 4-count block noise — "under
  2× the noise of the measurement it comes from".

The reading that fits both is **a live sensor with a dead coil**, which is
exactly the case a single `live` flag cannot represent. So:

    SENSE_OK[j] = enabled & healthy & ~floor_bad & sensor_row_ok[j]
    DRIVE_OK[j] = enabled & healthy & ~floor_bad & residue_ok[j]

**HOLE H3** resolves it: the **off-diagonal** entries of row a5 of `R_m` — a5's
response to coils 0–3. If a5 responds to *other* coils, its sensor is alive and
it belongs in `SENSE_OK`. That is one row of a matrix already recorded; it needs
no new bench time, only a refit that keeps the rows.

On today's numbers, pending H3: `DRIVE_OK = {a0, a1, a2, a3}`, `SENSE_OK` is the
same four plus a5 *if* H3 says so. a4/a6/a7 are in neither — their baselines are
3%, 5% and 1% of the median, so `baseline-floor` removes them with no new policy.

### 4.2 Projecting a channel out — a table, not a zeroed row

**Zeroing row `j` of `Φ⁺` is wrong.** `Φ⁺` is the least-squares inverse computed
*assuming* sensor `j` was present; deleting its row keeps a normalisation that
no longer applies and returns a biased modal estimate. The correct operation is
to re-derive the inverse on the survivors: `Φ_S⁺` with the dead rows **removed**,
and likewise `A_C⁺`.

Doing that inside the loop would mean a pseudo-inverse per fault event with
data-dependent timing. v13 does not:

> **All 2⁸ = 256 sensor masks and 256 coil masks are inverted once, at start-up,
> and cached. The control step does one array lookup and two small matrix–vector
> products. There is no matrix arithmetic in the loop and the worst-case step
> cost is a constant.**

Each is a 2×2 solve; the whole table is ~33 kB per side. The §4.4 gate is
evaluated at the same time and cached as a boolean per mask, so **the decision
to fall back is also a lookup**. Preflight prints which masks run MIMO, which
fall back, and the conditioning of each — the file about to run stays the thing
you can read.

Conditioning as sensors drop, computed on the passive `SHAPE_8` because that is
the only mode-shape matrix that exists today (**H2 replaces this table**):

| surviving sensors | cond | ‖Φ_S⁺‖ | shape-spread error, med / p90 |
|---|---|---|---|
| a0,a1,a3 | 1.59 | 1.08 | 8.9% / 12.7% |
| a1,a2,a3 | 1.96 | 1.08 | 13.5% / 23.1% |
| a0,a1 | 2.04 | 1.42 | 2.3% / 4.4% |
| a0,a3 | 2.28 | 1.59 | 12.5% / 18.1% |
| **a0,a1,a2,a3** | **2.44** | **1.06** | **14.1% / 24.1%** |
| a1,a2 | 2.55 | 1.43 | 10.1% / 19.0% |
| a2,a3 | 2.87 | 1.60 | 16.0% / 28.2% |
| a0,a1,a2 | 3.22 | 1.41 | 14.2% / 28.0% |
| a0,a2,a3 | 3.61 | 1.59 | 19.8% / 34.4% |
| **a1,a3** | **47.7** | 47.7 | 80.8% / 87.9% |
| **a0,a2** | **64.1** | 28.3 | 85.4% / 238.8% |
| **a0,a5** | **singular** | ∞ | — |

### 4.3 Why the table looks like that: two clusters

Angles between sensor rows in the 2-D mode space:

```
   a0 vs a2   1.83 deg      a1 vs a3   2.41 deg
   a0 vs a5   0.00 deg      a0 vs a1  73.98 deg
   a2 vs a5   1.83 deg      a2 vs a3  69.73 deg
```

**The sensing channels are two clusters, each internally degenerate to under
2.5°, separated by 70–74°.** {a0, a2, a5} see the COMMON 1.046 Hz mode; {a1, a3}
see the DIFFERENTIAL 1.657 Hz mode. (a5's 0.00° is the `1/12` assumption making
it exactly parallel to a0; a real measurement will not be exact, but a5 carries
1/12 of the amplitude either way — removing it from the full set changes cond by
nothing.)

> **MIMO needs at least one healthy sensor from {a0, a2, a5} AND at least one
> from {a1, a3}.** Two sensors from the same cluster average, they do not
> resolve. `a0,a2` (cond 64) and `a1,a3` (cond 48) are exactly the two ways to
> have two healthy sensors and no modal estimate.

This answers "the minimum sensor count below which you must fall back":
**there is no such count.** Two can be excellent (`a0,a1`: cond 2.04, 2.3%
error) or unusable (`a0,a5`: singular). Four can be worse than three (`a0,a2,a3`
at 3.61 is worse than `a0,a1,a3` at 1.59). A count is the wrong statistic and
v13 does not use one — it uses the conditioning of the actual surviving subset,
from the table.

### 4.4 The gate, and the fallback

    MIMO_OK[mask] ⟺  rank(Φ_S) = 2  and  rank(A_C) = 2
                 and cond(Φ_S) ≤ COND_MAX  and cond(A_C) ≤ COND_MAX
                 and propagated modal error ≤ ERR_MAX

`cond` is a backstop, not the criterion, and the table shows why it cannot be
the criterion: `a1,a5` has cond 6.57 but only 1.7% propagated error, because a1
is the best-measured row on the rig. **The criterion is the propagated error** —
the measurement's own covariance pushed through `Φ_S⁺`, per mask, at start-up.
`COND_MAX` catches the singular cases the covariance push cannot see.

**HOLE:** `ERR_MAX`. It is a policy number, set against (7): safe while
`|E_mm| < 1`, so **0.25** sits a factor of 4 inside the sign-flip boundary while
admitting the full sensing set (24.1% p90) today. That is the recommended
starting value, to be restated once H2 supplies a real covariance in place of
the four-log spread used here.

**When the gate fails, v13 runs v12.** Not a fault — v12's law, v12's gains
restricted to the survivors, unchanged to the last digit. That path has 27 bench
logs behind it and damps with a single healthy channel
(`MIN_HEALTHY_CHANNELS = 1`, `provenance.md` §3), strictly below anything MIMO
can do.

| healthy set | v13 runs |
|---|---|
| gate passes | MIMO |
| gate fails, ≥ 1 healthy channel | **diagonal — exactly v12** |
| 0 healthy channels | FAULT, exactly as v12 |

`MIN_HEALTHY_CHANNELS = 1` is **not** raised. Raising it converts a degradation
into a fault, and v12's fault history says that trade goes the wrong way.

**Bumpless switching.** Both laws write the same `self.out` through the same
clip and the same `MAX_SLEW_PER_S = 2.0` V/s limiter, so the command cannot
jump. On a switch v13 zeroes the modal integrator and resets the modal D filter,
and — unlike `_moved`'s handling of a bias step — leaves `env_hist` and
`excess_since` **alone**, because a law switch injects no force transient.
Logged with the mask, the conditioning and the reason.

**Anti-chatter.** Demotion is immediate (safety); re-arm already waits
`REARM_SUSTAIN_S = 2.0 s` in `_health`, so the mask cannot oscillate faster than
v12's own clock and no new hysteresis is needed. A mask change is acted on only
at a control step, never mid-sample.

### 4.5 Every interlock, one at a time

| interlock | input | under MIMO |
|---|---|---|
| **`_rail_check`** | raw ADC counts, full wire rate | **No change.** A rail is a converter hitting an end stop, upstream of the projection. |
| **`baseline-floor`** | per-channel calibrated `bp` RMS | **No change**, and more load-bearing: it removes a4/a6/a7 before they reach `Φ⁺`. Its safety property — the largest baseline ≥ the median > the threshold, so at least one enabled channel always survives — is untouched. |
| **runaway breaker** | per-channel `env` vs own baseline, plus growth | **No change to the test.** It reads `bp`, pre-projection. |
| **lock detector** | per-channel `bp` RMS vs baseline | **No change.** |
| **gain schedule** | per-channel `ratio = env/baseline` | **Target changes.** Under MIMO the scheduled quantity is `K_m`, one scalar per mode, driven from the **worst** `ratio` over `SENSE_OK` and slew-limited by the same `GAIN_SLEW_PER_S`. Worst is the conservative direction and keeps the soft-start. |
| **`_sat_check` / `soft-saturation`** | per-channel `pinned` at that channel's own window | **Test unchanged, meaning changed — below.** |
| **`auto-disable` / `_health`** | per-channel rail, re-arm timer | **Unchanged**; its effect is now a mask change, i.e. a table lookup, instead of zeroing a gain. |
| **quorum** | count of healthy | **Unchanged at 1.** The MIMO/diagonal gate sits above it. |

**Saturation is the one that needs care.** `pinned` is still a hardware fact
about coil `j` and the trip still fires per channel. What changes is the
diagnosis: under diagonal, coil `j` pinned means channel `j`'s own loop ran out
of authority; under MIMO it means **the allocation asked coil `j` for more than
it has** — an allocation problem, not necessarily a fault.

v13 keeps `soft-saturation` exactly as v12 has it — pinned for `SAT_FRACTION` of
`SAT_SUSTAIN_S` **and** the envelope not falling — and adds one step *before*
the trip: if a single coil is pinned while the surviving set still spans two
modes without it, **drop that coil from `DRIVE_OK`, re-look-up `A_C⁺`, and carry
on**, delivering the same modal force from the remaining coils. If the gate
fails without it, or the envelope is not falling on the reduced set either, the
v12 trip fires unchanged.

This is the one place MIMO makes an interlock *better* rather than harder, and
it is worth having: `soft-saturation` has never fired on hardware in any
version, and `sat-window` exists because the previous form could not be reached
at all.

**One genuinely new check, computed but not acted on.** `SENSE_8`'s third row is
the direction orthogonal to both modes. No in-band motion of a rigid body can
produce a reading there, so a sustained non-zero is **sensors disagreeing** — a
bad sensor that is not railed, not signal-less, and therefore invisible to every
existing interlock. Exactly the failure MIMO is most exposed to. v13 computes it
and logs it every control step and **does not act on it**: there is no measured
threshold, and adding an unmeasured fault source to a rig whose fault history is
the main thing wrong with it is a bad trade. First bench run records the
distribution; the threshold comes after.

---

## 5. Predicted damping, and where the headroom actually is

### 5.1 The comparison

    Δγ_m^diag = ρ_m · Σ_j P_mj · Kp_j            (3)
    Δγ_m^mimo = ρ_m · K_m                        (5)

with `ρ₀ = 0.410779`, `ρ₁ = 0.311967` at v12's 100 Hz — **evaluated at the
passive frequencies, and therefore a hole until H1.** `P` is measured (§2.3).

**Every number in /s is a hole.** `gains.json` reports the shipping gains adding
`[0.3445, 0.0628] /s` and the proposal reaching `0.0739 /s` worst-mode, but
those are computed at `f = 0.7149 / 0.9941 Hz` with `Q = 288 / 2334`, and the
bench measures the gain-off envelope decaying at 0.149–0.166 /s, implying
**Q ≈ 20–22**. The fitted `Q = 2334` is 100× that, is not physical for this
pendulum, and points the wrong way — the loop was damping throughout, so a pole
it moved should be *more* damped, not 50× less. Do not quote any /s figure from
that fit, including in this document's favour.

### 5.2 The headroom, which is measured, and the sting

`gains.json`'s peak-actuator block is derived from drive amplitudes and filter
gains rather than from the mode fit, so it survives the caveat:

| term | peak, V | share |
|---|---|---|
| P | 0.1279 | 43% |
| **I** | **0.1135** | **38%** |
| D | 0.0584 | 19% |
| **total** | **0.2998** | **120% of the 0.250 V half-window** |

**The clipping is arithmetic.** v12's shipping gains ask for 20% more than the
actuator has, before any disturbance. That is the same clipping `CLAUDE.md`
item 1 set out to explain with a bias sweep, and it now has a second, simpler
explanation that needs no hardware at all.

**And the largest single item is Ki, which dissipates nothing.** 0.1135 V, 38%
of the peak, on a term that by construction rejects no drift (§3.6) and whose
damping contribution computes negative at both modes. Deleting it recovers more
headroom than any allocation scheme can. `gains.json`'s proposal — Ki cut to
15%, Kp raised to the 0.035 ceiling — moves the total to **0.2001 V** and the
energy-removing share of authority from **43% to 88%**, and it is a diagonal
proposal.

**So the honest ranking of what buys headroom on this rig is:**

1. delete Ki — **0.1135 V, measured, needs no matrix**;
2. re-balance Kp across channels — diagonal, already proposed;
3. min-norm allocation — **HOLE H5**, and it is third.

MIMO's headroom argument is what is left after (1) and (2). It is real
(§3.4 is a theorem, not an estimate) but it has not been sized, and it would be
dishonest to lead with it.

### 5.3 The risk

The mode shapes are known to tens of percent. Measured run-to-run half-spreads
over the four 8-channel logs: a2 mode-0 **±33.5%**, a3 mode-0 **±36.7%**, a2
mode-1 ±17.1%, a3 mode-1 ±20.8%, a1 both ±6-7%. Pushed through `Φ_S⁺` for the
full sensing set that is **14.1% median / 24.1% p90** modal-estimate error.

By (7) that is safe — a factor of ~4 inside the sign-flip margin — but it is not
accurate, and an inaccurate `Φ` spends authority mis-sharing damping between the
modes. **MIMO's advantage over diagonal is smaller than that error until the
shapes are re-measured.**

---

## 6. What is needed from the measurement

**M1 — the mode frequencies and damping ratios, honestly.** §7. Everything in
/s waits on this, and so does `ρ_m`.

**M2 — the rows of `R_m`, not only its diagonal.** `tune._bootstrap_P` keeps
`np.diag(...)` of each perturbed residue and discards the rest. v13's estimator
is `Φ = s_m`, the *columns* of `R_m`, and its gate needs their covariance. This
is a one-line change to what `_bootstrap_P` retains and it is the single most
important thing the tool is not already saving.

**M3 — row a5, off-diagonal.** Does a5 respond to coils 0–3? That separates a
live sensor with a dead coil from a dead sensor, decides `SENSE_OK`, and is
already in the recorded file. No bench time.

**M4 — per-(mode, channel) sigmas**, which the same bootstrap gives, for the
weighted inverse of §3.5c. ch1 mode 1 at ±9× its value is the reason.

**M5 — the off-tone noise floor per channel**, which sizes Kd and the ridge.

**M6 — confirm `CONTROL_HZ = 100` on the bench with eight coils.** Every `ρ_m`
is rate-dependent (`ρ₀` moves 0.411 → 0.318 between 100 Hz and 12.5 Hz, 23%).
v12 owns its rate by construction, but it must be *observed*, because it is the
assumption under every damping number here.

### Already answered, and worth recording as answered

- **A1, colocation / coil map** — `sign(P_0j) = sign(P_1j)` on all four usable
  channels. `DAC_CHANNELS` confirmed for ch0–ch3. §2.3.
- **A2, sign agreement** — the measured residue signs `[−, −, +, −]` match the
  DC diagonal and match v12's shipping `Kp`, ch2's positive gain included.

---

## 7. Getting f₀ and Q — the recommendation

**Why the multisine could not.** A Q ≈ 21 pole at 1.046 Hz has a half-power
width of `f/Q = 0.050 Hz`. The tones were placed at 0.3444 / 0.4111 / 0.4889 /
0.5667 / 0.6667 / 0.7889 / 0.9222 / 1.0889 / … — spacing ~0.167 Hz near 1 Hz,
**3× the resonance width**, and *both* fitted frequencies land in gaps. The peak
was never sampled. The fit interpolated a sharp feature from data that contains
no evidence of where it is or how sharp it is, which is exactly how you get
Q = 2334.

The off-resonance placement was **correct** for its purpose: a Q ≈ 50 mode
amplifies an on-peak drive ~50× and that is what railed ch2 in v6
(17 of 48 steps). The run measured what it was designed to measure — residue
ratios and signs, which are trustworthy. It was never going to give f₀.

**Recommended: a ringdown, not a sweep.**

Kick the optic, drop the gain to zero, and fit a **damped sinusoid** to the raw
bandpassed signal — not an envelope. `f` and `γ` fall out of the same fit, from
one decay, with no drive to rail anything.

- the two modes separate by **channel**: `analysis/out/modes.csv` has 1.046 Hz
  dominant on ch0/ch2 and 1.657 Hz dominant on ch1/ch3, so one ringdown gives
  both;
- fit each channel separately and cross-check — the mode a channel is dominant
  in should return the same `f` from every channel dominant in it;
- **it can be done offline first.** `analysis/out/decay_windows.csv` already
  holds 6 824 gain-off decay windows extracted from the existing logs. They were
  fitted as *envelopes*, which throws the frequency away. Re-fitting the same
  windows as damped sinusoids costs **no bench time at all** and may settle H1
  outright.
- the envelope fits already constrain the answer: `γ = 0.149–0.166 /s` at zero
  gain ⟹ **Q = πf/γ = 20–22**. Both Q = 50 (assumed everywhere in the repo) and
  Q = 2334 (fitted) are already contradicted by data on disk.

**If a driven confirmation is wanted**, a narrow stepped sweep bracketing each
resonance is the standard cross-check, and it is the expensive option — near
resonance the settling time *is* the ringdown, `τ = Q/(πf) ≈ 6.4 s`, so ≥ 5τ ≈
32 s of dwell per point, ~5 points per mode, plus the drive backed off by ~Q to
keep the same sensor swing. Call it 10 minutes of optic time for a number a
60-second ringdown gives directly. Do the ringdown first.

---

## 9. Two supervisor findings from the 2026-08-06 run, and what v13 does with them

### 9.1 `baseline-sanity` — a fresh baseline is never checked against a good one

**The bug.** `BASELINE_SANITY_RATIO = 3.0` is applied on **one** path only: when
a *stored* baseline is loaded from disk, `_warm_from_file` compares it against
`BASELINE_WARMUP_S` of fresh samples and refuses it outside 3.0x. A baseline
**measured fresh** by `_set_baseline` is compared against nothing at all.

**What it cost, on the bench.** A quiet calibration measured ch0 = 0.0530 V.
After a kick and four re-faults the code forced a recalibration, `fast-calib`
exited early at 10 s claiming the sub-windows agreed, and stored:

| | ch0 | ch1 | ch2 | ch3 | ch5 |
|---|---|---|---|---|---|
| inflation vs the quiet floor | **11.5x** | 13.4x | 8.3x | 15.5x | 10.5x |

ch0 went 0.0530 -> 0.6112 V. Nothing checked it. Every test downstream of
calibration is a ratio against that number, so:

- the **runaway breaker** now trips at `1.8 x` an 11.5x-inflated floor, i.e. it
  is desensitised by an order of magnitude — the interlock that stops a pumping
  loop;
- the **lock detector** claims LOCKED at `0.35 x` the same number, i.e. on
  motion ~4x the true floor. A false LOCKED on a rig whose whole target is
  "locked within 10 s".

Both directions are unsafe, which is why v12's own comment on the *load* path —
"a wrong denominator desensitises the runaway breaker AND makes LOCKED easier to
declare" — applies word for word to the path that has no check.

**The fix, and it is small.** Apply the same ratio test to a **fresh** baseline
whenever a prior one exists — the stored file, or the pre-fault baseline
`warm-restart` is already holding. Same constant, same direction of bias
(refuse, and pay the 20 s), same message. v13 carries it as a named fix,
`baseline-sanity`, and it is worth having in v12 independently of anything
modal: it is a supervisor defect, not a control one.

### 9.2 `fast-calib` measures stationarity, and stationarity is not quietness

**The root cause**, and it is a real one. `_calib_stationary` asks whether the
trailing `CALIB_AGREE_N = 3` sub-window RMSs agree within
`CALIB_AGREE_TOL = 1.20`. On a ringdown of time constant `tau`, the trailing
three 2 s sub-windows differ by `exp(4/tau)` — so the test passes whenever
`tau >= 4/ln(1.20) = 22 s`, **no matter how far above the floor the whole
ringdown sits.**

That is the flaw stated exactly: **a stationarity test bounds the derivative,
not the level.** The ratio it measures grows only logarithmically in how wrong
the answer is, while the error in the stored baseline grows linearly. A smooth
exponential decay is *locally stationary everywhere*, so `fast-calib` fires most
confidently precisely when it is most wrong — and 2026-08-06 was the first time
it has ever fired on hardware, 11.5x wrong.

§9.1's ratio check is the right fix because it tests the **level**, which is the
thing that was wrong, against an independent measurement of the same level.
Tightening `CALIB_AGREE_TOL` is not: `analysis/calib_tol.md` already calls it
"a damage bound, not a discriminator", and no tolerance on a derivative can
bound a level.

### 9.3 What this means for the modal estimator — the answer is "it does not touch it"

The trap generalises: **any quantity estimated from a calibration window
inherits it**, because the window can be a ringdown and look perfectly settled.
A modal estimator built from the calibration covariance would have inferred mode
content from an 11.5x-inflated, ringdown-dominated window and reported a
confident answer. That is not hypothetical — the deleted modal v4 did exactly
that, taking "98.8% / 1.2% mode content from the calibration covariance"
(`research.md` item 2).

**v13 does not estimate anything modal from live data.** `Phi`, `A`, their
covariances and the whole 256-mask table are **constants in the file**,
transcribed by a human from `gains.json` after a driven measurement, exactly as
`gains.json` demands ("Nothing loads this file... A human transcribes"). The
only live inputs to the modal path are `vel`, which is already filtered, and the
health mask, which is a set of booleans. There is no adaptive identification in
the loop and there will not be, on the `research.md` rule: learn the model
offline, keep the control law and the interlocks classical and certifiable.

The calibration baseline still enters v13 — through the gain schedule, the
runaway breaker and the lock detector, all of which stay per-channel and
unchanged (§4.5). So v13 inherits the §9.1 defect and therefore inherits the
fix, which is why `baseline-sanity` is on its FIXES list rather than left to a
later version.

### 9.4 The bar v13 has to beat

The tuned diagonal gains ran on hardware immediately after the measurement and
worked, pre-kick:

| | ch0 | ch1 | ch2 | ch3 | lock line |
|---|---|---|---|---|---|
| ratio | 0.23 | 0.17 | **0.05** | 0.10 | 0.35 |

All four inside the lock threshold, and **ch2 — the channel that used to rail in
17 of 48 stepped-sine steps — is now the strongest.** That is the number v13 has
to beat, it is diagonal, and it is real. Anything this design claims is measured
against it.


---

## 10. Files

- `analysis/mimo_design.py` — reproduces every number above.
- `osem.v13.py` — the skeleton. Marks every hole and refuses to run MIMO until
  they are filled.
