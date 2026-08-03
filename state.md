# State reference — OSEM fast-lock damping controller

Every piece of state in the loop, one section per part, with its initial value,
update rule, and reset behaviour. Nothing reimplements this — `sim/server.py` runs
the actual classes described here — so if this document and the code ever
disagree, the code is right and this file is stale.

Read it in layers — state lives in four places, and only the top layer is a real
state *machine*. The rest is held state that the machine reads and clears.

| Layer | Holder | Kind of state |
|---|---|---|
| 0 | `thing.c` (firmware) | streaming flag, 8 DAC hold registers |
| 1 | `pyDAC.py` `DACController` | serial port, protocol sync |
| 2 | `Controller` | the controller state machine + run-level bookkeeping |
| 3 | `Channel` ×4 | filters, PID, calibration, health latches |

---

## Layer 2 — the controller state machine

Three states. One variable, `Controller.state`, a string. The machine lives in
`Controller.step()`, which `main()` drives from the serial stream and
`sim/server.py` drives from a simulated plant — one implementation, two callers,
so the simulator exercises the shipping code rather than a copy of it. It gates actuation
(`Channel.actuate` reads it) and it is written to every CSV row.

### `CALIBRATING`

Entered at startup and after every fault recovery. Measures each channel's
undamped noise floor with **all gains forced to zero**.

- **Actuation:** all four outputs held at `BIAS` (0.25 V). No feedback.
- **Accumulates:** `calib_sq_sum`, `calib_n` per channel, over `bp_out²`.
- **Duration:** `CALIBRATION_S` = 8.0 s (~8 cycles of the ~1 Hz resonance).
- **Exit → `DAMPING`:** on timeout. Calls `finish_calibration()` on every channel,
  which sets `baseline_rms` — the reference *both* the runaway breaker and the lock
  detector compare against for the rest of the run.
- **Exit → `FAULT`:** any channel rails. A rail here means the optic is misaligned
  before the loop ever closed, so the baseline would be garbage.

> The baseline is re-measured on every entry, never carried over. This is the
> deliberate fix for the old sweep script, which measured one 45 s window at the
> start of a ~9 minute run and compared everything after against it.

### `DAMPING`

The only state that actuates. Runs indefinitely.

Per sample, in order: schedule gains → check runaway → update lock → actuate.

- **Actuation:** `out = clip(bias + P + I + D)`, then slew-limited. Disabled
  channels get `active_gain = 0.0` explicitly and still hold at bias.
- **Exit → `FAULT`:** a sustained rail on any channel, a runaway on any channel, or
  a pinned actuator on any channel. Any one of the four trips **all** of them.

### `FAULT`

Everything frozen, waiting to see if the world comes back.

- **Actuation:** all outputs to `BIAS`. `active_gain` forced to 0.0 every sample.
- **Not running here:** `update_schedule`, `check_runaway`, `update_lock`. This
  matters — see the hazard note below.
- **Exit → `CALIBRATING`:** all channels clear for `FAULT_CLEAR_SUSTAIN_S` = 5.0 s.
  Calls `reset_for_recalibration()` on every channel and starts over from scratch.

### Transitions

| From | To | Condition | Side effect |
|---|---|---|---|
| — | `CALIBRATING` | process start | `calib_start = now` |
| `CALIBRATING` | `DAMPING` | `now - calib_start ≥ CALIBRATION_S` | `finish_calibration()` all; `damping_start = now`; `locked_announced = False` |
| `CALIBRATING` | `FAULT` | `any_rail` | `fault_clear_since = None` |
| `DAMPING` | `FAULT` | `any_rail` **or** any `check_runaway()` true | `fault_clear_since = None` |
| `FAULT` | `CALIBRATING` | `all_clear` held `FAULT_CLEAR_SUSTAIN_S` | `reset_for_recalibration()` all; `calib_start = now` |

There is no transition out of `DAMPING` other than into `FAULT`, and no terminal
state — `Ctrl+C` leaves the loop through the `finally` block, which returns all
four DAC channels to bias and closes the log.

---

## Layer 3 — per-channel state (`Channel`, ×4)

### Identity & configuration (never mutates after construction)

| Field | Value | Note |
|---|---|---|
| `idx` | 0–3 | sensor index → analog pin `A<idx>` |
| `enabled` | `ENABLE_CHANNEL[idx]` | only ch0 is `True`; only validated axis |
| `steady_gain` | Kp at lock | `[-0.030, -0.030, +0.010, -0.030]` |
| `capture_gain` | Kp at large amplitude | `[-0.035, -0.035, +0.012, -0.035]` |
| `ki`, `kd` | `0.0` all channels | see PID state below |
| `bias` | 0.25 V | the actuator's zero-force operating point |

> **ch2's sign is inverted** relative to the other three (`+0.010` vs `-0.030`).
> Since all four coils are wired the same way, that is anti-damping — it pumps
> energy into the resonance rather than removing it. This is exactly what the
> simulator demonstrates when you enable ch2.

### Filter chain state

Reset only by constructing a new `Channel` — **not** by `reset_for_recalibration()`.

| Field | Init | Update |
|---|---|---|
| `hp` | `OnePoleFilter(0.4 Hz, "high")` | removes DC / suspension drift |
| `lp` | `OnePoleFilter(3.0 Hz, "low")` | rejects sensor noise above the resonance |
| `deriv_smooth` | `OnePoleFilter(5.0 Hz, "low")` | smooths the finite-difference velocity |
| `prev_filt` | `0.0` | previous `bp_out`, for the difference |
| `filt_initialized` | `False` | first sample seeds `prev_filt`, emits `vel = 0` |
| `bp_out` | `0.0` | bandpassed position, **the signal everything else measures** |
| `vel` | `0.0` | smoothed velocity estimate, the PID process variable |

### PID state (velocity loop, setpoint 0)

Error is `e = -vel`. All three terms are in DAC volts and sum into the output.

| Field | Init | Update | Cleared when |
|---|---|---|---|
| `active_gain` | `0.0` | Kp; scheduled + slew-limited, see below | recalibration; forced 0 in `FAULT` and on disabled channels |
| `p_term` | `0.0` | `active_gain * e` | not actuating |
| `i_term` | `0.0` | `clip(i_term + ki*e*dt, ±I_CLAMP_V)` | not actuating, **and** recalibration |
| `d_term` | `0.0` | `kd * lowpass(Δe/dt, D_SMOOTH_HZ)` | not actuating |
| `prev_err`, `err_initialized` | `0.0`, `False` | previous `e` for the D difference | not actuating |

With `ki = kd = 0` the sum collapses to `p_term = active_gain * (-vel)`, which is
identical to the original velocity-feedback law. That is the shipped default.

**Anti-windup is two layers:** a hard clamp at `±I_CLAMP_V` (0.15 V), plus
conditional integration — the accumulator stops growing when the output is already
against `VMIN`/`VMAX` and this error would push further into it.

**What I and D physically are, in this loop:** integrating velocity gives
displacement, so I is a **spring** — it moves the resonant frequency, it does not
damp. Differentiating velocity gives acceleration, so D is an **added mass** — it
changes effective inertia, also without damping, while double-differentiating the
sensor. Kp is the only term that removes energy.

### Gain scheduling state

| Field | Init | Update |
|---|---|---|
| `schedule_rms` | `SlidingRMS(1.0 s)` | short rolling RMS of `bp_out` |
| `last_ratio` | `0.0` | `schedule_rms / baseline_rms` |

`frac = clip((ratio - 0.2) / (0.6 - 0.2), 0, 1)`, then
`target = steady + frac·(capture - steady)`, then `active_gain` moves toward
`target` at no more than `GAIN_SLEW_PER_S` (0.02 /s).

Because `active_gain` starts at `0.0`, this slew limiter **is** the soft-start —
there is no separate startup ramp. Running only when `baseline_rms` is set means
scheduling cannot act before calibration completes.

### Calibration state

| Field | Init | Update | Cleared when |
|---|---|---|---|
| `calib_sq_sum` | `0.0` | `+= bp_out²`, only in `CALIBRATING` | `finish_calibration()`, recalibration |
| `calib_n` | `0` | `+= 1` | same |
| `baseline_rms` | `None` | `max(sqrt(sum/n), 1e-6)` at exit | set to `None` on recalibration |

`baseline_rms is None` is the guard used by `update_schedule`, `check_runaway`, and
`update_lock` to no-op before the first calibration finishes.

### Health & latch state

| Field | Init | Set when | Cleared when |
|---|---|---|---|
| `rail_since` | `None` | counts leave `[3, 1020]` | counts back in range |
| `rail_fault` | `False` | railed ≥ `RAIL_SUSTAIN_S` (0.5 s) | counts back in range — **self-clearing, every sample** |
| `excess_since` | `None` | RMS > `1.8×baseline` | RMS back under |
| `sat_streak` | `0` | `out` within 1e-6 of `VMIN`/`VMAX` | any non-saturated sample |
| `saturated_flag` | `False` | `sat_streak > 30` | `sat_streak` breaks; on entry to `FAULT` |
| `locked` | `False` | RMS < `0.35×baseline` for 5 s | RMS rises above |
| `locked_since` | `None` | first sample under threshold | RMS rises above |

Rail detection reads **raw ADC counts, never volts** — a fully occluded OSEM pins
the ADC at 0, and the bandpassed signal then flatlines, which an RMS-only check
reads as perfect stability while the sensor is actually blind.

### Actuator state

| Field | Init | Note |
|---|---|---|
| `prev_out` | `bias` | input to `slew_limit`, capped at `MAX_SLEW_PER_S` = 2.0 V/s |
| `out` | `bias` | the commanded voltage this sample |
| `actuator.last_sent_t` | `0.0` | DAC writes throttled to ≥ 10 ms apart |
| `actuator.last_sent_v` | `None` | writes suppressed under a 0.5 mV deadband |

`out` is **not** what the coil sees. `RateLimitedActuator` turns the per-sample
value into a ~100 Hz zero-order hold, and the DAC quantizes to 12 bits over 0–2.5 V
(0.61 mV/LSB). The simulator models both, because the hold introduces real phase lag.

---

## Helper state

**`OnePoleFilter`** — `y`, `x_prev`, `initialized`, `tau = 1/(2π·f_c)`. First sample
seeds `y = x` (lowpass) or `y = 0` (highpass) and returns immediately. Lowpass:
`y += (dt/(τ+dt))·(x-y)`. Highpass: `y = (τ/(τ+dt))·(y + x - x_prev)`.

**`SlidingRMS`** — `buf` (deque of `(t, value)`), `sq_sum`. Time-based window, not
sample-count: evicts while `t - buf[0][0] > window_s`. Three instances per channel
with different windows (lock 5.0 s, runaway 2.0 s, schedule 1.0 s).

**`RateLimitedActuator`** — swallows `RuntimeError` from a missed DAC ack on
purpose; the next sample resends.

---

## Layer 2 — run-level state in `Controller`

| Field | Purpose |
|---|---|
| `calib_start` | start of the current calibration window |
| `damping_start` | when gain was first applied — the denominator of the lock-time report |
| `fault_clear_since` | start of the current all-clear run in `FAULT` |
| `locked_announced` | one-shot latch so `*** LOCKED ***` is emitted once per damping entry |
| `lock_time`, `fault_count` | reported to the caller; `lock_time` clears on re-calibration |
| `events` | console strings, drained by the caller — `main()` prints them, the server ships them to the browser |

`main()` still owns `start_time`, `prev_t`, the CSV file and the status cadence.
`dt` is clamped there to `[1e-4, 0.05]` s, so a stalled serial link cannot inject
a huge timestep into the filters or the integrator.

**One clock, supplied by the caller — with one exception.**
`Controller.step(counts, volts, t, dt)` takes its time from the caller, so every
timer in the machine is driven by whatever clock the caller supplies. The single
exception is `RateLimitedActuator.send()`, which calls `time.time()` directly for
its 10 ms throttle. That is correct on hardware, where DAC writes really should
be spaced in real milliseconds, and wrong for a simulation stepped faster than
real time — sim samples advance while the throttle holds the DAC on a stale
value, so the damping degrades in proportion to how fast the host happens to run.

There is **no `CLOCK` attribute in any controller**; the throttle is a plain
`time.time()` call. The simulator solves it from the outside instead, by swapping
the module object the controller resolves `time` through. `sim/server.py` defines
`_SimTimeModule`, whose `time()` returns `Sim.t` and whose `__getattr__`
delegates everything else to the real `time` module, and `Sim.reset()` installs
it with

```python
osem.time = _SimTimeModule(self)
```

so `RateLimitedActuator.send`'s `time.time()` resolves to simulated seconds while
the controller file on disk stays byte-for-byte what runs on the bench. `main()`
is never called from the simulator, so its own `time.time()` calls are unaffected.

---

## Layers 0 and 1 — firmware and transport

**`thing.c`** holds `streaming` (bool, set by `STREAM`/`STOP`) and the DAC's eight
internal hold registers. `setup()` enables the internal 2.5 V reference and zeroes
channels 0/2/4/6 before anything else. Channel state persists in the chip until
overwritten — nothing on the host side re-reads it.

**`DACController`** holds the `serial.Serial` handle and, implicitly, protocol sync.
`set_voltage` scans up to 50 lines for an `OK`/`ERR` because acks arrive interleaved
with the sample stream; failing that scan raises `RuntimeError`, which is where the
actuator's swallowed exception comes from.

---

## State hazards

Things that are easy to get wrong when editing, and what the simulator checks.

1. **`saturated_flag` is a permanent latch.** It is only recomputed inside
   `check_runaway()`, which does not run in `FAULT`. A saturation trip therefore
   pins `all_clear` to `False` forever and makes `FAULT` unrecoverable without a
   manual restart — contradicting the auto-recovery the module docstring promises.

   A *runaway* trip is different and recovers normally, because `saturated_flag`
   was never set; only saturation deadlocks. The simulator reproduces both, and
   `harness.py` asserts the broken behaviour for v0/v1/v2 and the fixed behaviour
   for v3/v4 — the `KNOWN BUG` checks flip to a recovery assertion, so the fix is
   verified rather than assumed.

   Which scenario produces which trip is **not** what it was before the plant
   became one rigid body. Inverting ch2's sign with all four channels enabled no
   longer runs away: the other three still out-damp it (the net of
   `gain_i · COIL_GAIN_i` is −0.0419 against −0.1415 with the correct sign, so
   about a third of the damping survives), the optic just stays near full
   amplitude, and the velocity feedback then asks for more than the ±0.25 V it is
   clipped to. It trips on **ch3 saturation** at t = 10.74 s with `sat_streak` = 31
   and latches. To exercise the runaway path on its own the harness runs ch2
   alone at half the shipped magnitude, wrong-signed: the growth is then slow
   enough for the 2 s `RUNAWAY_SUSTAIN_S` to elapse while the actuator is still
   at 0.055–0.112 V of its 0.25 V clip, and every version recovers from it.

   **Deliberately left in place.** Do not fix this without being asked.

2. **Filter state survives recalibration.** `reset_for_recalibration()` clears the
   PID, the RMS windows, and the baseline, but *not* `hp`/`lp`/`deriv_smooth`. That
   is intentional — the filters are tracking a continuous physical signal and
   re-seeding them would inject a transient at the exact moment the loop re-arms.

3. **One rail freezes all four channels.** Not a per-axis response. All four OSEMs
   sit on the same rigid body, so one blind sensor means the optic's position is
   untrustworthy globally.

4. **The rail trip point sits very close to the noise floor.** `RAIL_LOW_COUNTS`
   is 3 counts of 1023 (~14.7 mV), and `check_rail` requires the signal to stay
   under it *continuously* for `RAIL_SUSTAIN_S` — 250 consecutive samples at the
   ~500 Hz stream rate. Any sample above the line resets `rail_since` to `None`.
   Simulated, with one OSEM occluded and measurement noise swept: ≤5 mV arms in
   0.51 s every time; 7 mV takes 6.8 s; **≥10 mV never fires at all.** So if the
   dark noise on a blind channel is more than roughly one ADC count RMS, this
   interlock silently fails to arm — in exactly the scenario it exists for, and
   the one an RMS-only check cannot catch, since a blind channel's bandpassed
   signal flatlines and reads as perfect stability.

   The dark noise on these OSEMs has not been measured, so whether this bites on
   the bench is unknown. Either fix is cheap and neither risks false trips: raise
   the threshold (the optic at rest sits near 510 counts and swings ±180, so even
   30 counts is nowhere near the operating range), or score "railed for most of a
   window" rather than "railed continuously". **Not changed in the code** — it is
   a safety threshold, so the call is yours.

4. **`baseline_rms` is measured with the drive still running.** It is a noise floor
   *under whatever excitation exists at calibration time*, not an intrinsic
   property. Calibrating during an unusually quiet minute raises the effective
   trip sensitivity for the rest of that run.

   This gets worse when the excitation is broadband rather than a coherent tone.
   A high-Q resonance driven by noise is a narrowband random process whose
   amplitude wanders on the ringdown timescale — `Q/f0`, tens of seconds for this
   suspension. The runaway breaker compares a 2 s RMS (`ENVELOPE_WINDOW_S`)
   against a baseline taken from one 8 s window, and neither is long enough to
   characterize that wander, so an ordinary excursion reads as a runaway. The
   simulator reproduces this: set the drive to zero, raise seismic noise, and the
   controller trips several times in a few minutes with nothing actually wrong.
   Lengthening `CALIBRATION_S` and `ENVELOPE_WINDOW_S`, or taking the baseline as
   a median over several windows, would all reduce it.

5. **Disabled channels still hold full state.** They filter, rail-check, calibrate,
   and log — they just never actuate, and their `active_gain` is pinned to 0.0.
   They can still trip the global interlock.
