# Provenance

Where the constants in the controllers came from. Extracted from two files that
were removed from the working tree on 2026-08-03 and survive only in git history:

- **`report.pdf`** — *Weekly Report-11, "Python-Based Closed-Loop Suspended Mirror
  Stabilization Using Active Damping"*, Siddhartha Mithiya, Project Associate,
  CQuICC, Indian Institute of Technology Madras, 17 July 2026. 4 pages.
- **`ref.py`** — an 8-channel variant of the controller, used here only as the
  record of the coil wiring.

Nothing in the repo reads this file. It exists so that "why is this number what it
is?" has an answer that is not folklore.

---

## Table 1 — per-channel decay fits

Damping coefficient γ, decay time τ = 1/γ, and fit quality R², measured with all
four channels damping simultaneously.

| Channel | γ (1/s) | τ (s) | R² |
|---|---|---|---|
| A0 | 0.2197 | 4.5511 | 0.6477 |
| A1 | 0.1821 | 5.4901 | 0.7960 |
| A2 | **0.2796** | **3.5767** | 0.7526 |
| A3 | 0.1982 | 5.0452 | 0.9185 |

**This table is the sole justification for `STEADY_GAIN[2]` being positive.**
A2 damps *fastest* of the four — and Figure 2's applied-gain panel shows it running
at **K = +0.010** while A0/A1/A3 ran at **−0.030**. A positive gain producing the
best damping only makes sense if that channel's coil or OSEM is mounted the other
way round, so the sign convention differs for ch2 alone. Do not "correct" it
without re-deriving it on the bench.

R² spans 0.65–0.92, so these are **one significant figure** numbers. The report is
explicit that the decay is not cleanly single-exponential: the four OSEM channels
are coupled projections of a shared set of rigid-body normal modes, so a real decay
curve departs from the idealised single-mode model, worst at the tail.

`sim/server.py`'s `COIL_GAIN = [1.00, 0.76, -4.15, 0.86]` is *inferred* from this
table, not measured directly. The negative entry for ch2 is the same sign fact.

## §3 — one channel damps the whole mass

Driving a single OSEM/coil pair damps **all four** channels, not just the driven one.
The four OSEMs are mounted on one rigid body, so they are different projections of
the same rigid-body normal modes; removing energy from one projection removes it
from the coupled system. The undriven channels decay more slowly, since they can
only lose energy indirectly through their coupling to the driven one.

Whole-system damping time this way: **≈70 s** (Figure 1).

This is the rigid-body behaviour `sim/server.py` models. It is also why a railed
sensor on any one channel freezes all four — one bad projection makes the optic's
position untrustworthy globally.

## §4 — four channels together

All four damping simultaneously, each running its own independent velocity-feedback
loop, settles the optic in **≈17 s** against ≈70 s one at a time (Figure 2). That
4× improvement is the entire justification for v1 existing.

The report is careful about *why*, and the distinction matters: this is still **four
independent SISO loops**, not a MIMO controller. Each OSEM only ever drives its own
coil. The gain comes from parallel coverage of the coupled modes — the four loops
act in parallel rather than in series, so the overall damping rate is closer to the
*sum* of their contributions than to the single slowest coupling path. There is no
explicit decoupling between modes anywhere in v0–v3.

## The documented algorithm

```
bp(t) = LP_3Hz[ HP_0.4Hz[ A(t) ] ]                       (1)
v(t)  = LP_5Hz[ (bp(t) - bp(t-Δt)) / Δt ]                (2)
u(t)  = -K(t) · v(t)                                     (3)
RMS_base = sqrt( (1/N) Σ bp(t_i)² )   over an 8 s open-loop calibration   (4)
ratio(t) = RMS_1s[bp(t)] / RMS_base                      (5)
K(t) = K_steady + clip( (ratio - 0.2)/(0.6 - 0.2), 0, 1 ) · (K_cap - K_steady)   (6)
```

State machine: `CALIBRATING → DAMPING → FAULT → CALIBRATING`, with FAULT forcing
K → 0 and holding until the channel has been clear for several seconds.

## What the report does NOT contain

Equally load-bearing, because these are things the repo asserts that the report
does not support:

- **No mention of the −0.040 instability.** `osem.v0.py`'s docstring is the only
  record of it (2026-07-15 run 3: damping at −0.030, instability/rail onset at
  −0.040). The report was written two days later and says nothing about it. The
  simulator is linear and provably cannot reproduce it. Treat that limit as real
  and uncharacterised.
- **`zero-crossing → f̂`** appears in the block diagram's calibration block. No
  frequency estimator was ever implemented in any controller.
- **`auto-disable`** appears in the safety-checks block. Never implemented.

## §5 — what the report said was next

1. Improve the stabilisation algorithm for more accurate and stable OSEM signals.
2. **Extend the microcontroller board to support an additional four OSEMs**, to
   control a second suspended mirror.

Item 2 is why the firmware streams eight ADC columns and why the coil map below has
eight entries.

---

## Coil wiring (from `ref.py`)

The rewiring done on 2026-08-03. `ref.py:121` carried the full eight-pair map:

```python
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]   # sensor index 0..7 -> DAC channel
```

The controllers here take the **first four** entries, `[1, 3, 5, 7]`, because only
A0–A3 have OSEMs on them. Measured on the bench 2026-08-03: A0–A3 carry coherent
~1 Hz pendulum motion (2.0–4.2 mean-crossings/s), while A4–A7 sit at std 0.4–12.4
counts with 60–113 crossings/s and means decaying 560 → 535 → 514 → 507 — the
sample-and-hold residue signature of unconnected AVR analog pins.

The previous map was `[0, 2, 4, 6]`. **It now points at the coils for sensors 4–7**,
so any controller still holding it closes every loop onto the wrong actuator. The
simulator cannot catch this: `sim/server.py` builds its held-voltage dict from
`enumerate(DAC_CHANNELS)` and reads it back with the same index, so it round-trips
through whatever map the controller declares. Only the bench can catch a wrong map.

Two other things `ref.py` carried that were **not** adopted, recorded so nobody
re-imports them by accident:

| | `ref.py` | v0–v3 |
|---|---|---|
| `PORT` | `/dev/ttyUSB0`, commented `board-2 ; DAC-6` | `COM7` (overridden by `bench.py`) |
| `BIAS` | `0.25`×4 then `0.15`×4 | `0.25`×4 |
| `VMIN, VMAX` | `0.0, 0.4` | `0.0, 0.5` |
| `STEADY_GAIN[0]` | `-0.200` | `-0.030` |
| `CAPTURE_GAIN[0]` | `-0.035` | `-0.035` |

`ref.py`'s `-0.200` is 6.7× the only gain ever validated on hardware and 5× past the
documented −0.040 rail onset. It is also *larger* than its own `CAPTURE_GAIN`, which
inverts the capture-aggressive / steady-gentle convention every other version uses.
The `board-2 ; DAC-6` comment suggests it described a different rig. Only the
`DAC_CHANNELS` line was taken from it.
