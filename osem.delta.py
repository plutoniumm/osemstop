"""
v12 -- the loop gets its own clock, and the wire stops being trusted.
====================================================================
v11 made the wire 49x faster (`fast-transport` + 500000 baud, 18 Hz -> ~900 Hz)
and then damped WORSE than every version before it. Time-weighted over the same
240 s / three-kick schedule on the bench, 2026-08-06:

      |                    | DAMPING | FAULT | CALIB | faults | locks |
      | v9  (4ch, 23 Hz)   |  83.1%  |  8.5% |  8.4% |    4   |   4   |
      | v10 (8ch, 18 Hz)   |  74.1%  | 15.0% | 10.9% |    8   |   3   |
      | v11 (8ch, 1111 Hz) |  42.3%  | 25.3% | 32.4% |   11   | 1, degraded |

Two changes, and a third that is bookkeeping. NOTHING in the control law moves:
STEADY_GAIN, KI_GAIN and KD_GAIN on a0-a3 are still v3's to the last digit,
ch2's +0.010 included, and no interlock threshold is retuned.

  1. `sample-guard`. FIRST, because it is the larger of the two effects and
     because change 2 makes it MANDATORY. `read_sample()` accepts any row whose
     first character is a digit and which splits into >= N fields -- so a TORN
     serial line passes. That was harmless while the ack was being drained;
     `fast-transport` stopped draining it, so the board's `OK ch=.. v=..`
     replies now arrive interleaved with data rows and a buffer boundary can
     splice two fields' digits together. Measured in v11's own 240 s log
     (`data/20260806_171500_fast_lock.csv`): TEN rows carry a count outside
     0..1023, values like 5659, 65690, 522676 -- and ZERO such rows appear in
     any of the four slower logs from the same session. It is a defect the
     speed-up created.
     What one of them costs: counts = 522676 is +45.6 V into the bandpass,
     which then rings down over the 0.4 Hz highpass (tau = 0.398 s) with the
     velocity estimate peaking at 1401 V/s. P = 0.030 x 1401 = 42 V demanded
     into a 0.5 V rail. Four of v11's twelve entries to FAULT follow a torn line
     within 2 s, and the first one starts the 4 s-DAMPING / 5 s-FAULT thrash
     that eats the middle of the run.
     The fix is a range check: a count outside 0..ADC_MAX_COUNTS is a FRAMING
     error, not a sample, and is dropped exactly the way an `OK` line is.
     Replayed offline over the same log, ch0's velocity estimate goes
     rms 13.109 -> 5.413 V/s and peak 1441 -> 41 V/s.

  2. `decimate`. Keep reading the wire at full rate; run the CONTROL STEP at a
     fixed CONTROL_HZ = 100 Hz on the MEAN of whatever arrived since the last
     one. See THE CONTROL RATE below for the whole argument -- the short form is
     that the arrival rate has changed three times in three days (24 -> 12.5 ->
     18 -> 1111 Hz) and every gain, filter and interlock in this file was tuned
     against one of those numbers. A control rate the file OWNS is a rate that
     stops moving underneath them.
     Replayed offline on the same log, WITH the guard: rms 5.413 -> 4.732 V/s,
     p99 27.0 -> 23.7, peak 41.1 -> 36.3. WITHOUT the guard it makes things
     WORSE -- 13.1 -> 23.4 V/s -- because a boxcar mean SPREADS one bad sample
     across a whole control step instead of confining it to one. That is why
     change 1 is not optional and why the two ship together.

  3. `persist-baseline`. The noise floor is a property of the room, not of the
     run, so it is written to a JSON file and reused across process restarts --
     with a config fingerprint, an age limit and a start-up sanity check, any
     of which refuses it. Cold-start calibration goes from 20 s to
     BASELINE_WARMUP_S = 2.0 s. See THE PERSISTED BASELINE below.

WHAT IS DELIBERATELY NOT DONE. `LOCK_SUSTAIN_S = 5.0` is five seconds of the
"lock within 10 s" budget spent confirming a lock that has already happened, and
it is the obvious next cut. It is NOT cut here. Change 2 alters the noise the
lock detector sees; cutting the confirmation window in the same version makes
the two changes inseparable, and the first bench run of v12 has to be able to
answer "did the decimation help?" without also asking "or did the shorter lock
window just make it look that way?". Next version, on its own, with the v12
numbers in hand.

--------------------------------------------------------------------------
THE CONTROL RATE, AND WHAT AVERAGING COSTS
--------------------------------------------------------------------------
WHY A FIXED RATE AT ALL. This file's arrival rate has been 73 Hz (one coil),
23.5 Hz (four), 12.5 Hz (eight), 18 Hz (eight, v10 today) and ~1111 Hz (v11
today, `analysis/out/loop_rate.csv` and the logs). It was never a constant: it
is a function of how many coils are being driven and which transport is
compiled in, i.e. of things that are not control decisions. Everything in this
file that is expressed per SECOND -- GAIN_SLEW_PER_S, MAX_SLEW_PER_S, every
filter corner in Hz -- adapts to that, and everything expressed per SAMPLE does
not. v11 removed the last per-sample threshold (`sat-window`), which is what
makes this change safe to make now. Decoupling the loop's clock from the wire's
finishes the job: the control step happens at CONTROL_HZ whatever the wire does.

ACCUMULATE BY TIME, NOT BY COUNT. `_due()` fires on elapsed time and the step
consumes HOWEVER MANY samples arrived -- 11 at 1111 Hz, 3 at the simulator's
347 Hz, 1 if the wire is slower than CONTROL_HZ. A fixed "average every 11
samples" would silently become a 50 Hz loop the moment somebody changed the baud
again, which is the exact failure this change exists to prevent. When the wire
is SLOWER than CONTROL_HZ every arrival runs a step, which degrades to v11's
behaviour at v11's rate -- correct, because you cannot control faster than you
can measure and stalling would only add delay.

WHY 100 Hz. Bracketed from both sides, and there is a lot of room between them:
  * ABOVE. It must be fast against the plant. The modes are 1.01 / 1.66 /
    2.79 Hz and the loop's own authority peaks near 3 Hz (kp040.py section 6),
    so 100 Hz is 27x the fastest mode and 33x the authority peak.
  * BELOW. It must be slow enough to average something. At the measured
    1111 Hz wire it takes 9.7 samples per step (measured, from replaying the
    log's own timestamps), which is the sqrt(N) this change is for.
  * AND it must not cost phase. See the budget below -- 100 Hz gives back
    13 deg at 3.75 Hz against v11's ~900 Hz, and still holds 18 deg MORE than
    the four-coil v9/v10 configuration every gain here was validated at.
50 Hz would average twice as much and cost 14 deg at 3.75 Hz; 200 Hz costs 6 deg
and averages half as much. 100 Hz is the round number in the middle of a flat
optimum, not a tuned value.

THE GROUP DELAY, WHICH IS THE THING THE SPEED-UP WAS FOR. A boxcar over M wire
samples is an FIR with group delay exactly (M-1)/2 wire samples, flat with
frequency: at M = 11 and 0.90 ms that is 4.50 ms, and its magnitude response is
0.9998 at 1.05 Hz and 0.9978 at 3.75 Hz -- i.e. the averaging attenuates NOTHING
in band and costs only delay. Phase margin, computed with
`analysis/kp040.py`'s own transfer functions so it is directly comparable to
`analysis/out/phase_budget.csv`:

      configuration                      dt_ctl   0.70Hz  1.05Hz  1.66Hz  2.79Hz  3.75Hz
      v10, 8 coils, DACController        79.95ms   160.7   141.0   110.8    69.8    45.5
      v9/v10, 4 coils, DACController     42.60ms   171.0   155.4   130.4    94.7    73.5
      v11, fast-transport, no decimation  0.90ms   182.7   172.2   153.9   124.3   105.0
      v12, 100 Hz step, boxcar of 11      9.90ms   179.0   166.8   145.9   113.1    91.9

The first three rows are the shipped `phase_budget.csv` to one decimal; the
fourth is the same computation with the boxcar folded in. So v12 gives back
13.1 deg at 3.75 Hz against v11, and keeps 18.4 deg MORE than the four-coil
configuration in which every gain in this file was measured -- and 46.4 deg more
than the eight-coil configuration v10 actually ran. The speed-up is not undone;
about a quarter of it is spent, deliberately, on averaging.

THE FILTERS DO NOT NEED RETUNING, AND THAT WAS CHECKED RATHER THAN ASSUMED.
BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ and D_SMOOTH_HZ are corner frequencies in
Hz, and `OnePole.update` recomputes its coefficient from `dt` every sample, so
the corner is invariant to the rate by construction. The one thing that is NOT
invariant is how much quantisation noise survives them, and the direction is the
opposite of the obvious one: differencing a signal quantised at 4.907 mV/count
gives noise proportional to 1/dt, but the FIXED 5 Hz DERIV_SMOOTH_HZ then
averages it, and the two combine to noise proportional to sqrt(dt). Measured, by
re-quantising the bench signal itself so the staircase keeps its real
correlation structure (at 1111 Hz a 1 Hz pendulum moves far less than one count
between samples, so the error is a staircase and a white-noise model is
optimistic): 0.0028 V/s of velocity noise at 0.90 ms, 0.0023 V/s decimated to
100 Hz, and 0.0086 V/s at v10's 44 ms. So the ADC contributes ~0.05% of the
velocity signal at any of these rates -- it is NOT what made v11 worse, the torn
lines were -- and decimation leaves the loop marginally cleaner than v11's and
3.7x cleaner than v9/v10's, while running at a rate the file chose.

WHAT RUNS AT WHICH RATE, and each exception is deliberate:

  full wire rate, every step() call
      the accumulator; `counts_mean` (the trim's rest-position EMA, which is a
      per-second time constant and only gets better with more samples);
      `_rail_check`, ON RAW COUNTS. A rail is a fact about a SAMPLE -- the ADC
      hit an end stop -- and averaging eleven samples of which nine are pinned
      produces a mean that is not pinned. RAIL_SUSTAIN_S / RAIL_FRACTION are a
      fraction of a window in seconds, so feeding them the full stream changes
      nothing about what they mean and makes them strictly better evidenced.

  CONTROL_HZ, on the mean
      the bandpass, the velocity estimate, the PID, the gain schedule, the
      runaway breaker, the lock detector, `_sat_check`, the trim, `_actuate`,
      and every state transition.

NO INTERLOCK IS COUNTED IN CONTROL STEPS, which was checked one at a time
because changing the rate is exactly what breaks that kind of threshold, and
this file has been bitten by it once already (`sat-window`, v11):
  RAIL_SUSTAIN_S / RAIL_FRACTION          seconds + fraction   (full rate now)
  SAT_SUSTAIN_S / SAT_FRACTION            seconds + fraction
  ENVELOPE_WINDOW_S, RUNAWAY_SUSTAIN_S,
    RUNAWAY_TREND_LAG_S                   seconds, deques keyed on t
  LOCK_WINDOW_S, LOCK_SUSTAIN_S           seconds
  CALIBRATION_S, CALIB_SUBWINDOW_S        seconds
  REARM_SUSTAIN_S, FAULT_CLEAR_SUSTAIN_S,
    TRIM_PERIOD_S, BASELINE_MAX_AGE_S     seconds
  GAIN_SLEW_PER_S, MAX_SLEW_PER_S         per second, x the CONTROL dt
  MAX_BASELINE_REUSE, TRIM_MAX_STEPS      counts of EVENTS, not of samples
One thing improves for free: `_set_baseline`'s ceiling branch splits the
calibration samples into CALIB_SUBWINDOWS equal-COUNT pieces, which is only
equal-TIME if the sample rate is constant. On v11 it was not -- the rate depends
on the state and on how many coils are being written -- and now it is.

LOGGING STAYS AT THE FULL RATE. CLAUDE.md's standing practice is that every raw
sample goes to disk, and decimation is not a licence to throw 90% of them away.
Every accepted sample writes a CSV row with its own raw counts; two new columns,
`ctl` and `n_avg`, mark the row on which a control step ran and how many samples
it consumed. The n_avg rows ENDING at a flagged row are exactly the set that
step averaged, so the decimation is reconstructible offline to the sample -- and
the derived columns (bp, vel, out, ...) simply repeat between control steps,
which is what the coil sees too.

--------------------------------------------------------------------------
THE PERSISTED BASELINE
--------------------------------------------------------------------------
Calibration is the largest single item in "lock within 10 s including
calibration". The floor is CALIB_MIN_SUBWINDOWS x CALIB_SUBWINDOW_S = 6.0 s and
in practice it runs the full 20 s ceiling, because `fast-calib`'s
CALIB_AGREE_TOL = 1.20 was tuned on simulator noise and has never converged on
the bench (v8, 2026-08-04: "ran the full 20 s ceiling -- the floor never
settled"; v10, 2026-08-06: the same). So the bench pays 20 s, every start, to
re-measure a property of the ROOM.

`warm-restart` already established that reusing a baseline is sound WITHIN a
run. This extends it across process restarts: the vector is written to
BASELINE_PATH at the end of every real calibration and read back at start-up.

WHAT IS STORED WITH IT, and any mismatch REFUSES the file rather than adapting
to it -- a floor measured under a different setup is worse than no floor,
because every downstream test is a ratio against it and a wrong denominator
desensitises the runaway breaker AND makes LOCKED easier to declare:
  * `schema`         -- format version. Unknown: refuse.
  * `measured_unix`  -- age. Over BASELINE_FILE_MAX_AGE_S: refuse.
  * `n_channels`     -- refuse on any change.
  * `enable`, `steady`, `capture`, `ki`, `kd`, `bias` -- the whole gain and bias
    configuration. Bias especially: it is a DC force, it moves where the optic
    hangs, and `bias-trim` moves it DURING a run, so the saved vector is the
    trimmed one and not the constant.
  * `bp_low_hz`, `bp_high_hz` -- the baseline IS a bandpassed RMS. Change the
    band and the number means something else.
  * `control_hz`     -- exact. It is a constant in this file, and it sets what
    the loop averages over.
  * `wire_hz`        -- measured, so compared within WIRE_RATE_TOL rather than
    exactly. A wire rate that has moved by more than 1.5x means a different
    transport or a different baud, which is the case that produced this whole
    version.
  * `a_vcc`, `adc_max_counts`, `dac_channels` -- the volts-per-count scale and
    the coil map.

AND ONE CHECK THAT IS NOT A FINGERPRINT: BASELINE_WARMUP_S = 2.0 s of measuring
before the loop closes, and the loaded floor must agree with what those 2 s see
to within BASELINE_SANITY_RATIO. 2.0 s is 5 tau of the 0.4 Hz highpass -- the
same number REARM_SUSTAIN_S uses, and for the same reason: the filters have to
be primed before anything computed from them means anything. It also lets the
rail interlock arm (RAIL_SUSTAIN_S = 0.5 s) before any coil is driven.

The numbers behind BASELINE_SANITY_RATIO = 3.0 and BASELINE_FILE_MAX_AGE_S,
which are measured rather than chosen, from the calibration banners in `bench/`:
      v9,  2026-08-04   ch0 0.2100  ch1 0.0993  ch2 0.3151  ch3 0.0794 V
      v8,  2026-08-04   ch0 0.2547  ch1 0.1146  ch2 0.4234  ch3 0.1040 V
      v10, 2026-08-06   ch0 0.4968  ch1 0.2144  ch2 0.6736  ch3 0.1789 V
Minutes apart in one session the floor reproduces to 1.21-1.35x. Two days apart
it has moved 2.0-2.4x. So 3.0x sits above the same-session spread and far below
what this check must catch, which is order-of-magnitude: a5 reads the same
motion at 1/12 of a0's counts-per-metre and a4/a6/a7 at ~1/200, so a flag that
has been moved shows up as a factor of ten, not a factor of two.
BASELINE_FILE_MAX_AGE_S = 1800 s is the same kind of policy choice as
BASELINE_MAX_AGE_S = 300 s and deliberately 6x it, because the two answer
different questions: 300 s is "may a loop that has just FAULTED keep leaning on
this floor", where the fault itself is evidence something moved; 1800 s is "is
this still the same lab session", and it covers an edit-restart cycle while
being 96x shorter than the two-day gap over which the floor was measured to
double.

AND IT CAN ALWAYS BE REFUSED BY HAND: `--fresh` on the command line, or
OSEM_FRESH_CALIB=1 in the environment. Preflight PRINTS what it loaded, from
where, how old it is, every channel's value and the verdict -- the repo's rule
is that the file about to run is the thing you can read, and a silently loaded
constant breaks that.

--------------------------------------------------------------------------
INHERITED FROM v11: eight OSEMs, and the transport stops throttling the loop
--------------------------------------------------------------------------
REQUIRES A REFLASH. `make arduino`. The board must be at the baud `pyDAC2` uses
before this file is run, and a host at the wrong baud reads garbage and fails
preflight with "never sent READY". That failure is the intended one; there is no
version negotiation on this link and there should not be.

  `sat-window`. The saturation interlock was `MAX_CONSECUTIVE_SATURATED = 30`
     -- thirty CONSECUTIVE iterations. Both halves of that are wrong. An
     iteration is not a fixed amount of time (it is one stream sample the
     controller managed to see, and DAC writes eat those, so 30 iterations is
     0.41 s on one coil, 1.28 s on four and 2.40 s on eight), and an unbroken run
     throws away all its evidence on one clean sample. Combined, that is why
     `soft-saturation` NEVER FIRED in v10's 240 s bench run on 2026-08-06: all
     eight faults were the runaway breaker. It is now a FRACTION of a window in
     SECONDS -- the shape the rail interlock has always used, `_sat_check` being
     `_rail_check` line for line -- and SAT_SUSTAIN_S = 1.30 s is chosen to be
     behaviour-preserving at the four-coil rate, so this is a bug fix, not a
     retune.

  `fast-transport`. `pyDAC.DACController.set_voltage()` writes SET and then
     reads up to 50 lines waiting for the board's `OK`; every one of those is a
     stream sample the controller never sees. Measured
     (`analysis/out/loop_rate.csv`): 73.0 Hz driving one coil, 23.5 Hz driving
     four, 12.5 Hz driving eight. The loop rate is a function of how many coils
     you drive, which is a strange thing for a control system to be true of.
     `pyDAC2.FastDAC` writes and returns. See THE TRANSPORT below.

  and then: ENABLE_CHANNEL is all eight, which `baseline-floor` is what makes
  survivable. See that array.

--------------------------------------------------------------------------
THE TRANSPORT, AND WHY THE ACK WAS WORTH GIVING UP
--------------------------------------------------------------------------
This is the riskiest of the three, because it is the first time a CONTROLLER
ships against anything but `pyDAC.DACController`, and because what it gives up
is an error-detection path. Taken anyway, and here is the whole argument.

WHAT THE ACK IS WORTH. `ERR` comes back for exactly two things: a malformed
command, and a channel outside 0..7. `FastDAC.set_voltage` rejects both locally,
before writing, and raises rather than continuing. The firmware clamps voltage to
0..2.5 V in `setChannel()` itself, so even a value that got past both checks
cannot ask the hardware for something it will not do. And the shipping
`Actuator.send()` has ALWAYS treated the ack as optional -- it catches the
RuntimeError raised when none arrives and moves on, because the next sample
resends. Nothing in this file has ever branched on an `OK`.

WHAT IT COSTS. Measured, not argued: at 12.5 Hz the phase margin at 3.75 Hz is
45.5 degrees, against 73.5 degrees at the four-coil 23.5 Hz
(`analysis/out/phase_budget.csv`). Sampling delay is pure phase lag and it is
the cheapest kind of instability to buy by accident -- and note the -0.040 rail
onset is still uncharacterised, so there is no measured margin to spend. An
eight-coil loop at 12.5 Hz is not a loop anybody has evidence is safe.

WHAT YOU MUST HANDLE, and it is real: the board still SENDS `OK ch=.. v=..` for
every write, and with nobody reading them they now arrive interleaved with the
sample rows. `read_sample()` filters them on the first character. Anything else
reading this stream must too. **v11 stopped there and it was not enough** -- the
interleaving also produces TORN rows that start with a digit and parse, ten of
them in v11's 240 s log against zero in every slower log. That is `sample-guard`
above, and it is the first thing this version fixes.

THE FALLBACK, since it should be one line. Change the import to
`from pyDAC import DACController` and `main()`'s `FastDAC(port=PORT)` to
`DACController(port=PORT)`. Everything else -- `Actuator`, `read_sample`, the
whole state machine -- works unmodified against either, which is itself the
evidence that the ack was never load-bearing. `bench.py`'s preflight still opens
the port through `DACController` whichever one this file uses, so the "is the
sketch alive, at this baud" check is unchanged.

--------------------------------------------------------------------------
THE CHANNEL SET AND THE SUPERVISOR, inherited from v10, unchanged below here
--------------------------------------------------------------------------
v10 is the eight-channel controller this is built on: same supervisor, same
state machine, and the same control law on a0-a3 to the last digit, ch2's
+0.010 included. It ran on the bench 2026-08-06 -- 60 s, zero faults, with
`baseline-floor` demoting a4/a6/a7 and a5 damping 0.0860 -> 0.0420 V. v11 adds
nothing to that; it changes the CLOCK the supervisor runs on and turns the
remaining three channels on as sensors.

WHAT CHANGED, AND THE EVIDENCE FOR IT (all 2026-08-06, write-ups in `analysis/`).
The eight OSEMs are all electrically connected -- confirmed on a scope, which
overturned the 2026-08-04 call that a4-a7 were "not wired". The real split is
in-band / out-of-band, and it is a spectral result, not a correlation one:

  | channel | senses suspension motion            | here                    |
  |---------|-------------------------------------|-------------------------|
  | a0-a3   | yes, 77-99% of power in 0.4-3 Hz    | enabled, gains as v10   |
  | a5      | yes, 22-71% in band, at ~1/12 gain  | ENABLED, new, -0.015    |
  | a4,a6,a7| no, 0.1-9% in band                  | sensors only, no coil   |

a5 tracks the 1.046 Hz suspension mode to within one FFT bin in all three
8-channel logs (offsets +0.0000 / +0.0000 / +0.0116 Hz at a 0.0116 Hz bin), with
mode amplitude 13.2 / 17.7 / 11.2 counts, and lock-in SNR 7.3 and 7.5 over two
long runs. A floating pin cannot peak at another channel's mechanical resonance
three times running. a4/a6/a7 score SNR 1.1-1.5 -- their own noise floor -- and
what they do carry is a 6.19 Hz interference line (with a 12.38 Hz harmonic on
a7), most likely ~350.9 Hz folded by the 357.1 Hz sample rate, i.e. the 7th
harmonic of ~50.1 Hz mains. That is an ALIGNMENT problem, not a wiring one: the
flag sits outside the partial shadow where a shadow sensor is linear. a5 is the
same story one step less severe -- inside the linear region but near its edge.

A5 IS WORTH MORE AS A SENSOR THAN AS A LOOP, TODAY. Two independent measurements
put its counts-per-metre at ~1/12 of a0's: the per-coil DC matrix gives a5 <=8
counts/V against a0's 105 (~13x), and the passive mode amplitudes give 13 counts
against 138 (~10.5x). Sensor gain and coil authority are different quantities and
only the first is measured, but they enter the loop as a product, so a twelfth of
the sensor gain is a twelfth of the loop gain unless a5's coil is unusually
strong -- and its own diagonal in that matrix, -7 counts/V, says it is not. So
a5 at -0.015 contributes a few percent of the rig's damping. What it does
contribute in full is a FIFTH INDEPENDENT VIEW of the same rigid body, at full
weight in the rail interlock, the runaway breaker and the lock claim. Treat this
build as "a5 joins the sensor set, and dips a toe in the loop".

THE SIMULATOR CANNOT TEST THIS FILE, and that is by design, not an omission.
`sim/server.py` models one rigid body with exactly four OSEMs -- GEOM is four
corners and SENSE is a 4x4 modal projection. There is no measured geometry or
coil gain for channels 4-7, and inventing them would mean validating this
against fabricated physics. `harness.py` skips it loudly, with the reason
printed. The `baseline-floor` guard it inherits IS covered, on four channels, in
v10's suite -- which is the point of having built it there first.

BRING-UP ORDER (CLAUDE.md item 2). Do not skip to the end:
  1. Run it as shipped and confirm a5's column carries coherent ~1 Hz motion,
     2-4 mean-crossings/s, not the 60-113 crossings/s of a floating pin. Also
     confirm `baseline-floor` demotes a4/a6/a7 at the end of calibration rather
     than the rig faulting -- that is the v5.5 failure, and this is the build
     where the fix meets it.
  2. Verify the coil map, one coil at a time. A wrong map is invisible to the
     simulator AND to every interlock in this file.
  3. THEN the sign test on a5, described at STEADY_GAIN.
  4. Only after that, KI/KD on a5, and only after a4/a6/a7 have been realigned
     and re-measured should their gains stop being zero.

--------------------------------------------------------------------------
THE SUPERVISOR ITSELF, unchanged since the four-channel v10
--------------------------------------------------------------------------
Same control law as every version back to v3 -- identical STEADY_GAIN, KI_GAIN
and KD_GAIN, including ch2's deliberate +0.010. Everything that changes is in
the SUPERVISOR. v10 is v9 (fast-calib, warm-restart, runaway-trend) plus v7's
`bias-trim`, which were parallel branches off v5, plus two new fixes.

  `bias-trim` (from v7, unchanged in behaviour). No OSEM rests at mid-scale --
  600 / 631 / 708 / 677 counts against 511.5 at bias 0.25 V -- so every channel
  clips its TOP rail first. Coil bias is a DC force, so stepping it moves where
  the optic hangs. One quantum at a time, kept only if the TOTAL offset improved.
  On hardware 2026-08-04 it took total offset 565 -> 455 counts at no cost in
  lock time. It cannot finish the job: centring wants -1.93..+3.77 V against a
  2.5 V DAC, and the rest is mechanical or TIA offset.

  `soft-saturation` (NEW). v9 faulted the whole rig after 30 consecutive samples
  with an output pinned against its rail. But clipping removes authority in ONE
  direction only -- an output pinned at vmax still pulls down at full strength --
  so a clipped loop is a weakened loop, not a broken one, and it is usually the
  thing bringing the optic back. Faulting replaces a half-strength actuator with
  a frozen one, which is strictly worse. The fault now needs two things at
  once: pinned for the same 30 samples v9 counted, AND not winning -- the
  envelope not falling. A clipped output whose envelope is coming down keeps
  damping. This is the same level-vs-trend correction `runaway-trend` made to
  the runaway breaker, applied one interlock over.

  `baseline-floor` (NEW). Every downstream test in this file is a RATIO against
  the calibrated baseline: the gain schedule, the lock detector, and the runaway
  breaker (env > RUNAWAY_MULTIPLE x baseline). A channel that senses no motion
  calibrates a near-ZERO baseline, and all three then divide by it -- so noise
  on a dead channel reads as a runaway, and the runaway breaker faults the WHOLE
  rig. Measured on the bench 2026-08-04 with all eight OSEMs streaming
  (`bench/20260804/v55_all8.log`, v5.5): a4 / a6 / a7 calibrated
  0.0014 / 0.0050 / 0.0020 V against ch0's 0.3109 V, and the rig faulted TEN
  times in 145 s -- nine of them naming a4, a6 or a7 -- while ch0-ch3 were
  damping happily at ratio 0.31-0.39. At the first of those faults a4 reported
  ratio 2.73 on a bandpassed signal of +0.000 V. Four working loops were frozen,
  repeatedly, by three channels that were not measuring anything.
  `auto-disable` does NOT catch this, and cannot: it keys on a channel RAILING,
  and a signal-less OSEM does not rail -- it sits mid-scale and flat, which is
  the one failure an RMS interlock reads as perfect stability.
  So: at the end of every calibration, a channel whose baseline is under
  BASELINE_FLOOR_FRAC of the MEDIAN baseline across the ENABLED channels is
  demoted, exactly as auto-disable demotes a railed one -- held at bias, out of
  the interlocks, everyone else keeps damping. See `_baseline_floor` for why the
  test is relative, why the statistic is the median, and what it deliberately
  does NOT do when most of the rig is signal-less.

WHY THE MERGE NEEDED A THIRD CHANGE. A bias step is a force step: it rings the
pendulum at ~1 Hz, inside the band both trend tests read. Rather than blind the
interlocks for a ringdown -- a 16 s hole in the breaker every TRIM_PERIOD_S,
i.e. open nearly always -- a step INVALIDATES the history it would have
corrupted: env_hist is cleared, and excess_since and sat_streak are reset. The
level halves keep working throughout; the trend halves resume one lag later.
See `_moved`. The BASELINE is deliberately not invalidated -- it is a bandpassed
floor and a bias step is DC; the first draft got that wrong, see `_why_reuse`.

Not verified on hardware. BENCH_STATUS is `untested`; every number quoted above
for `bias-trim` is v7's bench result, and `soft-saturation` has only ever run in
the simulator -- which models clipping but not the coil driver behind it.

--------------------------------------------------------------------------
INHERITED FROM v8: stop paying 20 s of zero gain for a baseline
--------------------------------------------------------------------------
v5 with two changes to the SUPERVISOR and nothing else. Every gain, filter,
threshold and state transition v5 ships is here unchanged: the PID, the
bandpass, the gain schedule, the rail interlock, the runaway breaker and the
lock detector are untouched, and the four new constants below are the only
additions to SETTINGS. Both changes are about WHEN the loop is allowed to be
closed, not about what it does once it is -- which is why the damping numbers
are v5's to three decimals (ch0 = 0.029, lock 10.5 s).

WHY. Time-to-lock is `calibration + lock-after-gain`, and on the bench
(2026-08-03/04, four channels) that is 20 s + ~11 s: **65% of the wall clock is
spent with the gain forced to zero and nothing damping.** Worse on recovery --
FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S = 25 s of open loop before the loop
re-engages, so one kick costs ~40 s of not damping. v5's `fast-refault` already
elides the case where the fault repeats immediately; this is the rest of it.
research.md item 6 is the write-up of the problem and ranks these two first.

  1. `fast-calib`. CALIBRATION_S becomes a CEILING rather than a target.
     Sub-windows of CALIB_SUBWINDOW_S accumulate as they complete and
     calibration ends as soon as the trailing ones say the noise floor has
     stopped moving. Quiet lab: out in ~6 s. Anything still ringing down: runs
     to the ceiling and falls back to EXACTLY v5's estimator, median of
     CALIB_SUBWINDOWS sub-windows over the whole window. Measured, in the
     simulator: calibration 20.00 s -> 6.01 s and total time-to-lock 30.6 s ->
     16.5 s, while the swept-shock baseline skew that `runaway-baseline` is
     scored on goes 19.7% -> 17.6% against a 35% line -- it does not degrade,
     because a shock inside the window is exactly the case that refuses to
     converge early and so is still measured v5's way.
     Swept over 16 lab conditions (sensor noise 0-40 mV, seismic 0.2-2.5, drive
     0-2.0 and 0.5-2.5 Hz, Q 10-200, 60 Hz hum, HVAC, shocks up to 60/min), 12
     exit at 6.01 s and land within 7.0% of the floor the full window would have
     measured -- worst case a lab so quiet the drive is off the resonance. The
     other 4 run the ceiling and return v5's number EXACTLY (0.0% difference,
     same code on the same samples): drive off with seismic only, the two
     off-resonance drives, and 60 shocks/min. Those are the cases where the
     floor genuinely is still moving, which is the answer you want.

  2. `warm-restart`. A fault that follows a healthy engagement re-engages on
     the baseline already in hand instead of measuring a new one. v5 does this
     only when the previous engagement lasted under FAST_REFAULT_S (the
     disturbance is evidently still there); v8 does it in the opposite case too
     (the baseline demonstrably supported a working loop, and the thing that
     faulted was a transient). Both are bounded -- see THE INVARIANT below.
     Measured on two 15 V/s kicks into a locked loop, at t = 45 s and t = 90 s:
     v4 and v5 pay FAULT_CLEAR_SUSTAIN_S + CALIBRATION_S = 25 s of open loop for
     each and never get back to LOCKED inside 115 s; v8 re-engages 5 s after the
     all-clear both times and re-locks 10.7 s later.

Everything else is v5 and is documented there and in versions.md: the gain
signs, ch2's positive Kp, the quorum of one, `auto-disable`, `fast-refault`,
and the three v3 defect fixes. This file is the mechanism, not the argument.

  CALIBRATING  gain 0. Sub-windows of CALIB_SUBWINDOW_S; ends EARLY once the
               trailing CALIB_AGREE_N of them agree with each other AND with
               every sub-window measured so far, else at the CALIBRATION_S
               ceiling on the median of CALIB_SUBWINDOWS windows. Any rail
               faults the rig -- you cannot calibrate a blind sensor, which is
               what guarantees a demoted channel has a baseline to come back to.
               On exit, any channel whose baseline came out under
               BASELINE_FLOOR_FRAC of the median is demoted: it measured
               nothing, so nothing may be divided by it.
  DAMPING      u = PID(-vel), gain scheduled by amplitude, slew-limited. A
               railed channel is DEMOTED (held at bias, the rest keep damping)
               and re-arms REARM_SUSTAIN_S after its rail clears. A
               baseline-floor demotion does NOT re-arm on a timer -- only a new
               calibration can overturn it. The rig faults only on a runaway, a
               pinned actuator, or losing the last channel.
  FAULT        all outputs at bias until clear for FAULT_CLEAR_SUSTAIN_S, then
               either straight back to DAMPING on the baseline already in hand,
               or a full recalibration (which also clears every demotion).

--------------------------------------------------------------------------
THE INVARIANT THIS BREAKS, AND WHERE THE LINE IS NOW
--------------------------------------------------------------------------
The standing rule is **"baseline RMS is re-measured, never borrowed"** (README,
versions.md). v4 broke it once, per channel, for the re-arm path; v5 broke it
once, for the whole rig, when a fault repeats inside FAST_REFAULT_S. v8 makes
borrowing the DEFAULT on the fault path. That is deliberate and it is the
riskiest thing in this file, so the reasoning is here rather than in a doc:

  * `baseline_rms` is the UNDAMPED noise floor of the lab. It is a property of
    the lab, not of the event that just faulted the loop.
  * Re-measuring after a transient measures the transient. At Q = 50 and
    f0 ~ 1 Hz the ringdown is tau = Q/(pi*f0) ~ 16 s, comparable to the whole
    window, so a post-kick recalibration returns an INFLATED floor. That is not
    merely wasteful, it is unsafe in both directions: the runaway breaker
    (> RUNAWAY_MULTIPLE x baseline) is desensitised, and the lock detector
    (< LOCK_RMS_FACTOR x baseline) becomes EASIER to satisfy -- a false LOCKED
    claim, which is the run's actual deliverable.
  * `fast-calib` narrows but does not close this: after a kick the sub-windows
    do not agree, so the recalibration correctly runs the full ceiling. Correct
    and still 20 s of not damping.

It must STILL re-measure, and does, in all four of these cases:

  1. **First entry at startup.** `self.baseline.all()` is False until a
     calibration has completed, so there is nothing to borrow. No exception.
  2. **After a RAIL-caused fault.** A rail means the SENSOR was suspect, and a
     floor measured through a suspect sensor is suspect with it -- including
     the possibility that the OSEM's DC operating point moved, which changes
     the floor without changing anything about the lab. `fault_railed` is set
     at the two places a rail can fault the rig (a rail during CALIBRATING, and
     losing the quorum in DAMPING) and forces a full recalibration.
  3. **After MAX_BASELINE_REUSE reuses** without an intervening engagement of
     at least FAST_REFAULT_S. This is v5's bound, unchanged, and it is what
     stops a stale baseline surviving a supervisor loop that is faulting on
     contact. A long engagement resets the budget because the baseline has just
     demonstrated that it supports a closed loop.
  4. **Once the baseline is older than BASELINE_MAX_AGE_S.** Rule 3 alone is
     not a bound in wall-clock time -- alternating long and short engagements
     could refresh the budget forever -- and an hour-long run must not close on
     a floor measured once at t = 0. A fault is the natural moment to re-measure.

Ran on the bench 2026-08-04 (`bench/20260804/v9_4ch_3kicks.log`): re-locked after
all three kicks, 4 faults against v8's 10, 26.6% of samples DAMPING. `runaway-trend`
is therefore confirmed on hardware. `fast-calib`, inherited from v8, is NOT --
`CALIB_AGREE_TOL` was tuned on simulator noise and the bench floor never satisfied
it, so it burns the full ceiling and degrades to v5's estimator. The numbers above
are still simulator numbers; what the simulator can and cannot tell you is in
versions.md.

Two conventions inherited from v5 and worth knowing before editing this:
`enabled` is static config, `healthy` is runtime, and `enabled & healthy` gates
the output. Timers that may be "not running" are np.inf, so `t - timer >= X` is
False without a branch; `baseline` is 0.0 rather than None while uncalibrated,
so it stays an array and still reads falsy where the old code tested it.
"""

import json
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime, timezone

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
# `fast-transport`. Every controller from v0 to v10.5 imports pyDAC.DACController,
# whose set_voltage() writes SET and then READS UP TO 50 LINES waiting for the
# board's `OK` -- and every one of those lines is a stream sample the controller
# never sees. That is not a small tax and it is not a constant one: it scales with
# the number of coils being driven, which is the one thing v11 changes.
# `pyDAC2.FastDAC` writes and returns. It is the same wire protocol, the same
# 0..2.5 V clamp and the same local argument validation -- the only thing given up
# is the acknowledgement, which is returned solely for malformed commands and
# channels > 7, both of which FastDAC already rejects itself before writing.
# See the docstring, THE TRANSPORT below, for why this was not optional here.
from pyDAC2 import FastDAC

# ===================== SETTINGS =====================
PORT, A_VCC, ADC_MAX_COUNTS, N = "COM7", 5.02, 1023, 8

# The FULL eight-pair map, sensor index 0..7 -> DAC channel, from `ref.py`
# (provenance.md). The four-channel controllers take only its first four entries.
# Nothing in software can check this: `sim/server.py` builds its held-voltage dict
# from `enumerate(DAC_CHANNELS)` and reads it back with the same index, so a wrong
# map round-trips cleanly, and the interlocks see a plausible loop that is simply
# closed onto somebody else's coil. Verify it on the bench, one coil at a time --
# CLAUDE.md item 2 step 2.
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]

# BENCH 2026-08-06. Ran with the MEASURED gains (see STEADY_GAIN). All four
# sensing channels reached LOCK at ratios 0.05-0.15 against a 0.35 threshold,
# the best this rig has produced; in absolute terms ~0.012-0.018 V of residual
# against a 0.0530 V quiet floor, i.e. 0.23-0.34x, which is the documented floor.
# Wire 1018-1043 Hz decimated to 100 Hz control, 0 torn frames across three runs
# and a 14-minute dither. Survived a hard kick that railed 7 of 8 ADCs: one
# runaway trip, ch0 peak ratio 3.79 back under threshold in ~15 s.
#
# NOT yet exercised on hardware: the `baseline-sanity` refusal path. It needs a
# forced post-fault re-calibration (4 consecutive re-faults), and the loop has so
# far always recovered by 1/4. The load-side check HAS fired, both on a config
# fingerprint mismatch and on ch3 measuring 3.25x its stored floor.
VERSION_TAG, BENCH_STATUS = "delta", "validated"
# NAME, not a number. See ladder.py. delta is the one to run: eight channels,
# 500000 baud, a 100 Hz control clock decoupled from the wire, and the fixes
# that made that safe -- sample-guard, decimate, persist-baseline,
# baseline-sanity, and the runaway-breaker lag.
FIXES = ("saturation-latch", "rail-threshold", "runaway-baseline", "auto-disable",
         "fast-refault", "fast-calib", "warm-restart", "runaway-trend",
         "bias-trim", "soft-saturation", "baseline-floor",
         "sat-window", "fast-transport",
         "sample-guard", "decimate", "persist-baseline", "baseline-sanity")

# ALL EIGHT. Not because all eight can damp -- a4/a6/a7 carry no in-band signal
# and ship at zero gain, see STEADY_GAIN -- but because "enabled" and "useful"
# are different questions and only the first one belongs in a constant. What
# `enabled` actually buys a channel is a place in the quorum denominator, in the
# lock claim and in the trim; every channel is filtered, rail-checked, calibrated,
# logged and watched by the runaway breaker whatever this array says.
#
# The reason this is now safe to write is `baseline-floor`. Enabling a channel
# that reads nothing used to be the one thing that could take the rig down --
# ch4/ch6/ch7 calibrate ~0.002 V, every ratio against that explodes, and the
# runaway breaker (which reads `healthy`, never `enabled`, so this was never
# about this array) faults everything. v5.5 did exactly that, ten times in 145 s.
# The guard demotes them at the end of each calibration instead, by name and with
# the numbers printed, and re-decides every time a new baseline is measured. So
# the channel set stops being a constant somebody has to maintain and becomes a
# MEASUREMENT the rig makes each time it calibrates: realign a4's flag and it
# joins on the next calibration, with no edit to this file.
ENABLE_CHANNEL = [True, True, True, True, True, True, True, True]

BIAS = np.full(N, 0.25)
VMIN, VMAX, MAX_SLEW_PER_S = 0.0, 0.5, 2.0     # nominal; per-channel copies in
                                               # __init__, which the trim moves

# Kp. STEADY is the bench-validated -0.030 on ch0; CAPTURE is stronger, used only
# at large amplitude, and is NOT independently validated.
#   STEADY_GAIN[2] is POSITIVE -- ch2's coil or OSEM is mounted the other way
#   round (provenance.md Table 1). Not a typo, do not "correct" it.
#   Kp = -0.040 is the documented instability / rail onset. Do not exceed -0.035
#   unattended, and note the simulator provably cannot reproduce that limit.
#
# ch4, ch6 and ch7 are ENABLED WITH ZERO GAIN, which is the whole of what
# "enable all eight" means here, and the distinction is worth being clear about
# because it is the difference between a commissioning build and a hazard.
# They join as SENSORS -- rail interlock, runaway breaker, quorum, lock claim,
# CSV -- and contribute no force at all. Two reasons, and the second is the one
# that matters:
#   * they carry no in-band signal to close a loop on (0.1-9% of power in
#     0.4-3 Hz, lock-in SNR 1.1-1.5, i.e. their own noise floor), so a loop there
#     would be feeding back on the 6.19 Hz interference line and nothing else;
#   * their coil sign is NOT KNOWN. Their own diagonals in the per-coil DC matrix
#     are +8 / +4 / +7 counts/V, and the mean |response| ANYWHERE in that block
#     of the matrix -- signal and cross-talk alike -- is 4 counts. Those signs
#     are coin flips dressed as measurements, and a wrong-signed channel does not
#     under-damp, it PUMPS. ch2 is the standing proof that a coil's sign cannot
#     be assumed; a5's -7 is at least the same order as its own noise, which is
#     why a5 gets a hedged gain and these three get none.
# `osem.v6.5.py` / `osem.v6.6.py` (stepped sine, all eight coils, both
# quadratures) are the measurement that turns these zeros into numbers --
# CLAUDE.md item 2b. Until it is run, leave them at zero.
#
# STEADY_GAIN[5] = -0.015 IS A PREDICTION, NOT A MEASUREMENT. Read this before
# running it. The per-coil DC matrix (2026-08-04, one coil stepped at a time,
# `bench/20260804/dcmatrix.log`) gives a5's own diagonal -- row a5, coil5 -- as
# -7 counts/V: the same sign as a0/a1/a3 (-105 / -68 / -51) and opposite to ch2
# (+213). Hence NEGATIVE. But -7 counts/V is a weak number -- the mean |response|
# anywhere in the a4..a7 block of that matrix is 4 counts, so -7 is under 2x the
# noise of the measurement it comes from -- and a wrong sign does not merely fail
# to damp, it pumps. So it ships at HALF the validated magnitude, and the first
# run is a sign test, not a damping run:
#
#     watch a5's OWN `bp` amplitude with only a5 newly enabled.
#     falling -> the sign is right, raise it toward -0.030.
#     rising  -> flip it to +0.015 and try again.
#
# CAPTURE_GAIN[5] is deliberately EQUAL to STEADY_GAIN[5] rather than 1.17x it.
# The schedule blends capture -> steady on amplitude, so during a sign test a
# changing gain and a changing amplitude move together and neither one explains
# the other. One constant gain makes the `bp` trend mean what it looks like.
# MEASURED 2026-08-06, tune.py fit of data/20260806_182231_tune_raw.csv --
# 717420 samples, 880 Hz, 0 frames dropped. Replaces hand-tuned values.
#
# ch2 goes +0.010 -> +0.035, a 3.5x rise, and that is the biggest single change
# here: ch2 carries the LARGEST residue of the four (+13.573 against -7.851 /
# -8.867 / -5.282 at mode 0), so the most effective channel was the most
# under-driven. Its positive sign is now confirmed a third time, and in BOTH
# modes rather than one.
#
# a5 drops -0.015 -> 0. Its residue is not distinguishable from its own noise in
# this run, and a gain on a channel that measures noise injects actuator noise
# while damping nothing. This contradicts the earlier "in band at ~1/12 of a0"
# read; the two disagree and the newer one is the one with error bars.
STEADY_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                        +0.000, +0.000, +0.000, +0.000])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.035, -0.035,
                         +0.000, +0.000, +0.000, +0.000])

# Ki/Kd on the velocity loop: the integral of velocity is DISPLACEMENT (an added
# spring, which moves the pole rather than damping it) and its derivative is
# ACCELERATION (added negative mass, and the noisiest term). Same sign per
# channel as Kp, because they share one loop.
#
# ZERO on a5, and that is the point of "same sign per channel": Kp, Ki and Kd
# share one sign, so shipping a5 with all three live would put the same guessed
# sign in three places at once and a sign test would be measuring their sum.
# CLAUDE.md item 2 puts switching these on AFTER the sign is confirmed, and this
# is that order written into the file.
#
# Ki cut to 15% of what shipped, and this is the headroom fix. MEASURED: the
# shipping Ki spends 0.1135 V of peak actuator -- 38% of the whole budget -- to
# dissipate nothing, because the integrator sits behind the 0.4 Hz highpass so
# i = -Ki*bp exactly and bp carries no DC for it to reject. Total peak demand
# was 0.2998 V against a 0.250 V half-window: the shipping gains were 20% OVER
# budget, which is the clipping, stated as arithmetic rather than as a symptom.
# Proposed set lands at 0.2001 V. Share of authority spent on terms that remove
# energy: 43% -> 88%.
KI_GAIN = np.array([-0.0060, -0.0060, +0.0060, -0.0060,
                    +0.0000, +0.0000, +0.0000, +0.0000])
KD_GAIN = np.array([-0.00045, -0.00045, +0.00045, -0.00045,
                    +0.00000, +0.00000, +0.00000, +0.00000])
D_SMOOTH_HZ, I_CLAMP_V, TRACK_TC_S = 2.0, 0.15, 0.5   # I_CLAMP is only a backstop

BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ = 0.4, 3.0, 5.0
CAPTURE_HIGH_FRAC, CAPTURE_LOW_FRAC = 0.6, 0.2
SCHEDULE_WINDOW_S, GAIN_SLEW_PER_S = 1.0, 0.02

# --- decimate: the loop's own clock ------------------------------------------
# THE one number this version exists to introduce. The full argument is in the
# docstring (THE CONTROL RATE); the short version is that the ARRIVAL rate has
# been 12.5, 18, 23.5, 73 and ~1111 Hz in this repo, depending on how many coils
# are driven and which transport is compiled in, and none of those are control
# decisions. This is.
#
# 100 Hz is bracketed, not tuned:
#   27x the fastest mode (2.79 Hz) and 33x the loop's own authority peak (~3 Hz)
#   9.7 wire samples per step at the measured 1111 Hz -- enough to average
#   costs 13.1 deg of phase at 3.75 Hz against v11, and still holds 18.4 deg MORE
#     than the four-coil configuration every gain in this file was measured at
# 50 Hz averages twice as much for 14 deg; 200 Hz costs 6 deg and averages half.
# The optimum is flat and this is the round number in the middle of it.
#
# The step accumulates by TIME and consumes however many samples arrived, so a
# baud change moves N and not the control rate. If the wire is SLOWER than this,
# every arrival runs a step -- you cannot control faster than you can measure,
# and waiting would only add delay. That is the case in the simulator with the
# ack drain on, and on every four-coil DACController build.
CONTROL_HZ = 100.0
CONTROL_PERIOD_S = 1.0 / CONTROL_HZ

# --- calibration: a ceiling and a convergence test, not a fixed duration -----
# CALIBRATION_S is now the LONGEST calibration may take, and CALIB_SUBWINDOWS is
# how the ceiling path divides it -- both unchanged from v5, so a calibration
# that runs to the ceiling produces exactly v5's number from exactly v5's code.
#
# CALIB_SUBWINDOW_S is the granularity of the convergence test. 2.0 s is two
# cycles of the ~1 Hz resonance the loop closes on, which is the shortest window
# whose RMS is not dominated by where in the cycle it started; it also divides
# CALIBRATION_S into 10, so the ceiling path regroups cleanly into the 5 windows
# v5 uses. CALIB_MIN_SUBWINDOWS = 3 keeps v5's "at least three, so one bad one
# can be outvoted" and sets the floor on calibration at 6 s.
#
# CALIB_AGREE_TOL is a RATIO (max/min) and 1.20 comes from a measured gap, not
# from taste: in a quiet lab the trailing three sub-windows agree to 1.09 at the
# earliest point the test can fire and to ~1.03 after that, while under a 6 V/s
# kick inside the window the same statistic runs 1.19-1.66 for the whole 20 s.
# 1.19 is the narrow side of that gap, which is exactly why _calib_stationary
# needs its SECOND test as well. Read that docstring before moving this number.
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5      # ceiling; subwindows >= 3
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20

# LOCK_SUSTAIN_S IS THE NEXT CANDIDATE AND IS DELIBERATELY NOT TOUCHED HERE.
# Five seconds of a ten-second budget spent confirming a lock that has already
# happened is the largest remaining item after `persist-baseline`, and cutting it
# is a one-character change. It is deferred because `decimate` changes the noise
# the lock detector sees -- the envelope now comes from a boxcar mean over ~10 ms
# rather than a single sample -- and two changes to the same claim in one version
# cannot be told apart on the bench. Run v12, get LOCK_WINDOW_S's false-positive
# behaviour at the new rate on paper, THEN cut it.
LOCK_RMS_FACTOR, LOCK_SUSTAIN_S, LOCK_WINDOW_S = 0.35, 5.0, 5.0
ENVELOPE_WINDOW_S, RUNAWAY_MULTIPLE, RUNAWAY_SUSTAIN_S = 2.0, 1.8, 2.0
# v9: the breaker also requires GROWTH. Compare the envelope against itself
# RUNAWAY_TREND_LAG_S ago; a real runaway grows on that timescale, a ringdown
# at Q~50 falls. 1.02 gives 2% headroom so envelope noise alone cannot read as
# growth. Lag is one ENVELOPE_WINDOW_S so the two estimates barely overlap.
# 2.0 s was the v9 value and it is TOO SHORT, measured 2026-08-06. The optic has
# three modes -- 0.7155 / 0.9949 / 1.6396 Hz -- which BEAT against each other with
# periods of 1.082 / 1.551 / 3.578 s (analysis/ringdown.md). A 2.0 s lag sits in
# the middle of that range, so `growing` was reading the beat rather than the
# trend: the envelope genuinely rises and falls every couple of seconds, and
# clearing 1.02 over 2 s happens constantly on a perfectly healthy ringdown.
#
# The signal it is supposed to see is far smaller than the beat it is competing
# against. At the measured tau of 56-130 s a real decay falls only 1.5-3.6% in
# 2.0 s -- at or under the 2% threshold. So the test could not separate decay
# from beating even in principle.
#
# 10.0 s is 2.8x the longest beat, so the beat averages out, and still 5.6-13x
# shorter than tau, so a genuine decay falls 7.4-16% across the lag and clears
# the threshold by a wide margin. Cost: a real runaway now needs
# 10.0 + RUNAWAY_SUSTAIN_S to trip. The rail interlock and `_sat_check` are the
# fast backstops for anything that diverges quicker than that.
RUNAWAY_TREND_LAG_S, RUNAWAY_GROWTH_FRAC = 10.0, 1.02

# --- soft-saturation ---------------------------------------------------------
# A pinned actuator is not the same thing as a broken one. Clipping at VMAX
# removes authority in ONE direction; the coil can still pull the other way at
# full strength, and the loop keeps working with the half it has. v9 faulted the
# whole rig on 30 consecutive pinned samples regardless -- the same level-vs-trend
# error `runaway-trend` fixed in the runaway breaker, one interlock over.
#
# The fault now needs both, together:
#   1. pinned for SAT_FRACTION of a SAT_SUSTAIN_S window -- see `sat-window`;
#   2. the envelope NOT falling -- if it is falling, the clipped output is
#      winning and taking it away would be perverse.
# A clipped output whose envelope is coming down keeps damping.
#
# SAT_DECAY_FRAC mirrors RUNAWAY_GROWTH_FRAC and is compared over the same lag
# against the same env_hist deque, so "falling" means one thing to both
# interlocks. 0.98 gives the same 2% of headroom against envelope noise.
#
# A rejected third condition, recorded because it looks right and is not: gating
# on clip_excess as "still demanding more than the rail can give". Back-
# calculation anti-windup drives that residual toward zero under sustained
# clipping BY DESIGN, so it vanishes exactly when saturation is worst. See
# actuate().
#
# --- sat-window (NEW in v11) -------------------------------------------------
# v10 counted `MAX_CONSECUTIVE_SATURATED = 30` CONSECUTIVE samples. Two defects,
# and on the bench 2026-08-06 the combination meant `soft-saturation` NEVER
# FIRED in a 240 s run with three kicks -- all eight faults were the runaway
# breaker. An interlock that cannot be reached is not an interlock.
#
#   (a) SAMPLES, not seconds, and one sample is not a fixed amount of time.
#       The controller only iterates on stream samples it actually SEES, and how
#       many it sees depends on how many coils it is driving, because every DAC
#       write costs it stream lines. Measured (`analysis/out/loop_rate.csv`):
#           1 coil  13.7 ms/iteration   73.0 Hz    30 samples = 0.41 s
#           4 coils 42.6 ms/iteration   23.5 Hz    30 samples = 1.28 s
#           8 coils 80.0 ms/iteration   12.5 Hz    30 samples = 2.40 s
#       So the threshold silently DOUBLED the moment channels were added, and
#       v11 changes the rate twice -- eight coils, then FastDAC, then 230400
#       baud. CLAUDE.md item 2c says to make it time-based BEFORE touching the
#       rate, and this is that.
#   (b) An UNBROKEN run. One sample off the rail -- one count of noise, one
#       slew-limited step that lands a microvolt short -- erases every bit of
#       accumulated evidence.
#
# Both defects were solved in this file years of versions ago, for the rail
# interlock: RAIL_SUSTAIN_S / RAIL_FRACTION, "a FRACTION of a window rather than
# an unbroken run, so one noise sample dilutes the evidence instead of erasing
# it". `_sat_check` is `_rail_check` with a different input, deliberately, down
# to the 0.9 x window `spanned` guard that stops the first samples of a run
# tripping it on a sample size of one.
#
# SAT_SUSTAIN_S = 1.30 s is chosen to be BEHAVIOUR-PRESERVING on four channels,
# so this is a bug fix and not a retune: 30 iterations x 42.6 ms = 1.278 s at the
# measured four-coil rate. On eight coils it now means 1.30 s as well, where the
# sample count would have meant 2.40 s. SAT_FRACTION = 0.80 is the rail check's
# own number, for the same reason it has it.
SAT_SUSTAIN_S, SAT_FRACTION = 1.30, 0.80
SAT_DECAY_FRAC = 0.98

# --- baseline-floor ----------------------------------------------------------
# The smallest baseline a channel may calibrate and still be trusted, as a
# FRACTION of the median baseline across the enabled channels.
#
# Relative, not an absolute number of volts, for two reasons that both bite.
# The floor scales with the LAB (seismic background, time of day, whether the
# HVAC is on) and with the SENSOR (an OSEM is a shadow sensor, so counts-per-
# metre is set by where the flag sits in its shadow -- a5 reads the same motion
# at ~1/12 of a0's gain, measured two independent ways on 2026-08-06). An
# absolute threshold in volts would have to be re-tuned for every lab and every
# realignment, and would be wrong on the first day either could change.
#
# 0.10 -- an order of magnitude below typical -- because all the OSEMs are
# bolted to ONE rigid body and are therefore projections of the same three
# modes. Geometry and sensor gain can spread their in-band RMS by a factor of a
# few; they cannot spread it by 10x unless a channel is not watching the body at
# all. The bench numbers bracket it from both sides (`v55_all8.log`, first
# calibration, as fractions of the median over ch0-ch3+a5):
#     a4 0.010, a7 0.014, a6 0.036   -- the signal-less three, all demoted,
#                                       worst case 2.8x under the line
#     a5 0.579                       -- real but weak, kept, 5.8x over it
# So the line sits inside a 16x-wide empty gap. It is deliberately biased toward
# KEEPING a channel: demoting a good sensor costs one loop, keeping a dead one
# costs the whole rig.
BASELINE_FLOOR_FRAC = 0.10

# --- bias trim (from v7) -----------------------------------------------------
# No OSEM rests at mid-scale: 600 / 631 / 708 / 677 counts against 511.5 at bias
# 0.25 V, so every channel clips its TOP rail first and ch2 has the least room.
# Coil bias is a DC force, so it moves where the optic hangs, and the per-coil DC
# matrix (2026-08-04) says by how much.
#
# It is deliberately NOT a one-shot least-squares solve. That solve wants
# -1.93..+3.77 V against a 2.5 V DAC, so most of it is unreachable, and it
# assumes a matrix measured on a different day still holds. This trims
# ITERATIVELY: one quantum at a time, kept only if the TOTAL offset across all
# channels improved. Total, not per-channel, because four coils drive two DOF
# (v6 rank check: 2 directions above 10%) -- the channels are coupled and helping
# a0 can hurt a2. Reverting on a worse total is also what makes a wrong per-coil
# sign cost one step instead of walking a sensor into its rail.
BIAS_QUANTUM = 0.25            # coarse on purpose: 2 DOF, 4 knobs -- fine steps
BIAS_MIN, BIAS_MAX = 0.25, 1.25          # would just chase each other
BIAS_SWING = 0.25              # +-this around each channel's own bias
MID_COUNTS = 511.5             # (ADC_MAX_COUNTS - 1) / 2
TRIM_PERIOD_S = 15.0           # >= one ringdown at Q~50, f0~1 Hz (~16 s)
TRIM_DEADBAND_COUNTS = 40.0    # inside this, leave it alone
TRIM_MAX_STEPS = 4             # per channel, per run
# Folding v7 into v9 is not a paste, and this is why. A bias step is a force
# step: it rings the pendulum at ~1 Hz, inside BP_LOW_HZ..BP_HIGH_HZ -- the exact
# band `runaway-trend` reads for growth and soft-saturation reads for decay. So a
# step corrupts both TREND tests, which compare the envelope against its own past.
#
# The fix is NOT to suspend the interlocks for a ringdown. That would be a ~16 s
# hole in the only breaker that stops a pumping loop, opened every TRIM_PERIOD_S
# -- i.e. open nearly always, which is worse than not trimming at all. Instead a
# step INVALIDATES the history: env_hist is cleared and excess_since reset, so no
# comparison straddles the step. The LEVEL half of each test keeps working
# through it untouched, and the trend half resumes from post-step data within
# RUNAWAY_TREND_LAG_S. A real runaway is still caught, one lag later.
#
# `warm-restart` gets the same treatment: a baseline measured before the last
# accepted step describes a rest position that no longer exists, so it is refused.
# Direction to move THIS channel's bias to reduce its counts:
#   step = -sign(error) * SLOPE_SIGN * BIAS_QUANTUM
# Signs are the DIAGONAL of the per-coil DC matrix measured 2026-08-04 (one coil
# stepped at a time). ch2 is +1 because it is mounted the other way round, the
# same reason its gains are positive. Do NOT take these from a common-mode sweep:
# all four coils together read a3 as +43 counts/V while coil3 -> a3 alone is -51.
#
# The eight diagonals, in counts/V (`bench/20260804/dcmatrix.log`):
#     a0 -105   a1  -68   a2 +213   a3  -51      <- solidly measured
#     a4   +8   a5   -7   a6   +4   a7   +7      <- at the noise of that block,
#                                                   whose mean |response| is 4
# The last four are signs of numbers that are barely there. They are used anyway,
# because the trim is the one place a wrong sign is CHEAP: a step that does not
# improve the TOTAL offset is put back and that channel is frozen for the run, so
# a coin-flip sign costs one quantum and one period, not a sensor walked into its
# rail. That accept/revert is also what already caught a3, whose common-mode slope
# has the opposite sign to its per-coil one.
SLOPE_SIGN = np.array([-1.0, -1.0, +1.0, -1.0,
                       +1.0, -1.0, +1.0, +1.0])

# Rail: raw counts, so there is no float-rounding ambiguity, and a FRACTION of a
# window rather than an unbroken run, so one noise sample dilutes the evidence
# instead of erasing it.
RAIL_LOW_COUNTS, RAIL_HIGH_COUNTS = 12, 1011
RAIL_SUSTAIN_S, RAIL_FRACTION = 0.5, 0.80

FAULT_CLEAR_SUSTAIN_S = 5.0
REARM_SUSTAIN_S = 2.0        # 5 tau of the 0.4 Hz highpass; timed from rail-clear
MIN_HEALTHY_CHANNELS = 1     # one channel damps the whole mass (provenance.md §3)

# --- when the fault path may borrow the baseline it already has --------------
# The full argument, and the four cases where it must NOT, is in the docstring.
# FAST_REFAULT_S keeps v5's meaning and gains a second one: an engagement at
# least this long is what RESETS the reuse budget, because a baseline that
# carried a closed loop for 15 s has just been validated by the loop itself.
# MAX_BASELINE_REUSE is v5's bound, unchanged. BASELINE_MAX_AGE_S is the
# wall-clock backstop the budget alone does not give -- a policy choice, not a
# measurement: 5 minutes is long against the 16 s ringdown and short against a
# shift in the lab.
FAST_REFAULT_S, MAX_BASELINE_REUSE = 15.0, 4
BASELINE_MAX_AGE_S = 300.0

# --- persist-baseline --------------------------------------------------------
# The floor across process restarts. Written at the end of every REAL calibration
# and read at start-up; the full argument, and the measured run-to-run spread the
# two tolerances come from, is in the docstring.
#
# Anchored to THIS FILE's directory rather than the cwd, because the whole point
# is that a restart finds it again and `bench.py`, `make run` and a bare
# `python osem.v12.py` do not all run from the same place. `data/` is gitignored,
# which is correct: this is a measurement of one lab on one day, not source.
BASELINE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "data", "baseline.json")
BASELINE_FILE_SCHEMA = 1
# 6x the in-run BASELINE_MAX_AGE_S, deliberately -- they answer different
# questions. 300 s bounds how stale a floor may be when a loop that has just
# FAULTED wants to lean on it, and the fault is itself evidence something moved.
# 1800 s bounds "is this still the same lab session": long enough for an
# edit-restart cycle, 96x shorter than the two days over which the bench floor
# was measured to move 2.0-2.4x.
BASELINE_FILE_MAX_AGE_S = 1800.0
# A measured quantity, so compared with a tolerance rather than for equality.
# 1.5x is far wider than the jitter of a stable link and far narrower than any
# of the real changes -- 12.5 -> 18 -> 23.5 -> 1111 Hz are all >= 1.4x apart and
# the one that matters is 49x.
WIRE_RATE_TOL = 1.5
# Not a fingerprint but a MEASUREMENT: the loaded floor must agree with what the
# first BASELINE_WARMUP_S actually sees, per channel, to within this ratio.
# Same-session reproducibility on the bench is 1.21-1.35x; what this must catch
# is a flag that has moved, which is 12x (a5 vs a0) to 200x (a4/a6/a7 vs a0).
# 3.0 sits in that empty gap and is biased toward REFUSING the file, which costs
# 20 s and never costs safety.
BASELINE_SANITY_RATIO = 3.0
# How many consecutive fresh-baseline refusals before surrendering and taking the
# fresh number anyway. See `_set_baseline`: refusing keeps a floor that may be too
# LOW, which only makes the runaway breaker over-cautious, so this bound exists to
# stop a refuse/fault/recalibrate livelock -- it is not arbitrating correctness.
MAX_BASELINE_REFUSALS = 3
# 5 tau of the 0.4 Hz highpass, the same number REARM_SUSTAIN_S uses and for the
# same reason: nothing computed from the filters means anything until they are
# primed. It also covers RAIL_SUSTAIN_S = 0.5 s, so the rail interlock is armed
# before the first coil is driven.
BASELINE_WARMUP_S = 2.0

STATUS_PERIOD_S, CSV_FLUSH_EVERY_N = 5.0, 200

# ===================== kick-cue: the run tells you when to kick =====================
# A kick has to be hand-delivered and the operator is at the optic, not at the
# terminal, so the schedule was being kept by watching a scrolling log. Speak it
# instead. `say` is macOS-only and absent everywhere else, which is handled by
# not caring: if it is missing the cue is silently skipped and the run is
# identical.
#
# OFF BY DEFAULT, and that is not timidity: `make check` steps this file through
# 200-odd scenarios, and a suite that talks for twenty minutes is a suite nobody
# runs. Set the interval to enable it.
#
#     OSEM_KICK_CUE=25 make run V=v12        # cue every 25 s once damping
#
# NON-BLOCKING, which is the only part that could hurt anything. Popen, never
# run() or system(): `say` takes ~700 ms to speak this phrase, and 700 ms of
# blocked control loop at 100 Hz is 70 missed steps with the coils held at
# whatever they last commanded. Fired and forgotten, stderr discarded.
KICK_CUE_S = float(os.environ.get("OSEM_KICK_CUE", "0") or 0)
KICK_CUE_PHRASE = "jerk it"
_KICK_CUE_LEAD_S = 3.0          # first cue this long after DAMPING, not instantly


def kick_cue(phrase=KICK_CUE_PHRASE):
    """Speak `phrase` without blocking. Any failure is silent and harmless."""
    try:
        subprocess.Popen(["say", phrase],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, ValueError):
        pass                    # no `say` on this platform, or it vanished

_LOG = (("bp", ".5f"), ("vel", ".5f"), ("out", ".4f"), ("gain", ".5f"),
        ("ratio", ".4f"), ("p", ".5f"), ("i", ".5f"), ("d", ".5f"))
# `ctl` and `n_avg` are new in v12 and are what makes `decimate` auditable
# offline: `ctl` is 1 on the row whose sample completed a control step, and
# `n_avg` is how many raw samples that step averaged. The n_avg rows ENDING at a
# flagged row are exactly the set it consumed, so the mean it used is
# reconstructible to the sample from the `counts` columns. Between control steps
# every derived column simply repeats -- which is what the coil sees too, since
# the output is held.
CSV_HEADER = "time_s,state,ctl,n_avg," + ",".join(
    f"ch{i}_{c}" for i in range(N)
    for c in ("counts", "V") + tuple(k for k, _ in _LOG) + ("rail", "locked", "healthy"))
# ====================================================


class OnePole:
    """One-pole low/high pass over N independent lanes. `on` is per lane because
    the D-term filter is reset for whichever channels stopped actuating this
    sample, and a lane coming back must re-prime from its first sample rather
    than ramp up from zero."""

    def __init__(self, hz, kind="low", n=N):
        self.tau, self.low = 1.0 / (2.0 * np.pi * hz), kind == "low"
        self.y, self.xp, self.on = np.zeros(n), np.zeros(n), np.zeros(n, bool)

    def update(self, x, dt):
        new = ~self.on
        y0 = np.where(new, x if self.low else 0.0, self.y)
        a = dt / (self.tau + dt) if self.low else self.tau / (self.tau + dt)
        nxt = (y0 + a * (x - y0) if self.low
               else a * (y0 + x - np.where(new, x, self.xp)))
        self.y, self.xp = np.where(new, y0, nxt), x.copy()
        self.on[:] = True
        return self.y

    def reset(self, m=slice(None)):
        self.y[m], self.xp[m], self.on[m] = 0.0, 0.0, False


class SlidingRMS:
    """RMS over a time window. Scalar or array -- sim/server.py uses it scalar for
    its own diagnostic; the controller keeps one per lane so a single channel's
    window can be cleared without resizing everyone else's."""

    def __init__(self, window_s):
        self.window_s, self.buf, self.sq = window_s, deque(), 0.0

    def update(self, t, value):
        v2 = np.square(value)
        self.buf.append((t, v2))
        self.sq = self.sq + v2
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.sq = self.sq - self.buf.popleft()[1]
        # np.maximum is not defensive padding. The sum is incremental, so when a
        # large transient ages out of the window it leaves a rounding residue of
        # its own magnitude -- measured at -4.5e-14 after a 6 V/s kick in a quiet
        # lab, i.e. NEGATIVE. sqrt() of that is nan, and every comparison against
        # nan is False, so the runaway breaker would stop tripping and the lock
        # detector would stop firing, both silently and permanently. v0..v4 have
        # the same accumulator and the same latent bug.
        return (np.sqrt(np.maximum(self.sq, 0.0) / len(self.buf)) if self.buf
                else self.sq * 0.0)

    def reset(self):
        self.buf.clear()
        self.sq = 0.0


def _bank(window_s):
    return [SlidingRMS(window_s) for _ in range(N)]


def _rms(bank, t, x, m):
    """A lane outside `m` is not fed at all, so a blind channel's flatlined
    bandpass never enters its window."""
    return np.array([bank[i].update(t, x[i]) if m[i] else 0.0 for i in range(N)])


class Actuator:
    """DAC writes for all N coils: 0.5 mV deadband, and the CONTROL STEP is the
    throttle. Otherwise unchanged from v10 -- it never depended on the
    acknowledgement, which is most of why `fast-transport` is a drop-in.

    CHANGED IN v12. v10/v11 threw away writes closer together than 10 ms, which
    was a rate limiter bolted onto an actuator because the loop itself had no
    rate. It does now, and CONTROL_PERIOD_S is 10 ms, so keeping the old throttle
    would have raced it: two limits at the same nominal period, one on `t` and
    one on `time.time()`, dropping a semi-random half of the writes. The
    accounting that made 10 ms the right number is unchanged and still fits --
    8 coils x 100 Hz x ~15 bytes = 12 kB/s against 50 kB/s at 500000 baud
    (CLAUDE.md item 2c, which was written against the 11.5 kB/s of 115200).
    The deadband stays: it is not a rate limit, it is "do not spend wire on a
    command that changes nothing".

    The RuntimeError arm is now dead code with FastDAC, since nothing waits for a
    reply that could fail to arrive, and it is kept because this class is shared
    with the DACController versions and because a transport swap is not a reason
    to delete a guard. ValueError is deliberately NOT caught: FastDAC raises it
    for a channel outside 0..7 or a voltage outside 0..2.5 V, and neither is
    reachable -- `self.out` is a slew-limited walk between per-channel windows
    that themselves live inside [BIAS_MIN - BIAS_SWING, BIAS_MAX + BIAS_SWING] =
    [0.0, 1.5] V. If one ever does happen it is a bug in the clip, and a bug in
    the clip is the last thing that should be swallowed once per sample."""

    def __init__(self, dac, channels):
        self.dac, self.ch = dac, list(channels)
        self.t, self.v = np.zeros(len(self.ch)), np.full(len(self.ch), np.nan)

    def send(self, volts):
        now = time.time()
        due = np.isnan(self.v) | (abs(volts - self.v) >= 0.0005)
        for i in np.nonzero(due)[0]:
            try:
                self.dac.set_voltage(channel=self.ch[i], voltage=float(volts[i]))
                self.t[i], self.v[i] = now, volts[i]
            except RuntimeError:
                pass


def in_range(counts):
    """`sample-guard`, the test itself. A 10-bit converter cannot produce a count
    outside 0..1023, so a row carrying one is not a sample -- it is a framing
    error, and the only honest thing to do with it is drop it.

    Kept as a free function because the check is needed in TWO places and must
    mean the same thing in both: `read_sample()` rejects the row at the wire, and
    `Controller.step()` rejects it again because `sim/server.py` and `harness.py`
    build counts arrays themselves and never go through `read_sample`. Two lines,
    one definition.
    """
    return bool(np.all((counts >= 0) & (counts <= ADC_MAX_COUNTS)))


def read_sample(ser):
    """(counts, volts) as two length-N arrays, or None. arduino.ino streams eight
    columns and, as of 2026-08-06, all eight carry a real OSEM -- so N = 8 takes
    the whole row. A four-column firmware would simply never satisfy `len(parts)
    >= N` and this returns None forever, which is the loud failure you want.

    FROM v11, and required by `fast-transport`: the stream is no longer pure
    samples. Nobody is consuming the board's `OK ch=.. v=..` replies any more, so
    they arrive INTERLEAVED with the data rows. Rejecting on the first character
    is how pyDAC2.read_sample does it and is cheaper and less ambiguous than
    letting int() fail: `OK`, `ERR`, `STREAMING` and a torn line all start with
    something that is not a digit or a minus sign.

    FIXED (v12), `sample-guard`. v11 stopped there, and v11's own bench log says
    that was not enough: TEN rows in 241,813 carry a count outside 0..1023 --
    5659, 7575, 65690, 522676, 730608 -- against ZERO in each of the four slower
    logs from the same session. Those are two fields' digits spliced by a buffer
    boundary, and they start with a digit, split into eight fields and parse
    cleanly, so every guard above lets them through.
    One of them costs a fault. counts = 522676 is +45.6 V into a bandpass whose
    highpass has tau = 0.398 s, so it rings for most of a second with the
    velocity estimate peaking at 1401 V/s and P demanding 42 V into a 0.5 V rail.
    Four of v11's twelve entries to FAULT follow one within 2 s.
    The RANGE CHECK is the fix, and it also has to exist before `decimate` can:
    averaging is LESS robust to an outlier than passing it straight through,
    because one bad sample contaminates a whole control step instead of one
    sample. Measured offline on that log: with the guard, ch0's replayed velocity
    goes rms 13.109 -> 5.413 V/s and peak 1441 -> 41 V/s; decimating WITHOUT the
    guard takes it to 23.4 V/s, i.e. worse than v11.
    """
    if not ser.in_waiting:
        return None
    try:
        raw = ser.readline().decode("utf-8").strip()
        if not raw or raw[0] not in "0123456789-":
            return None                    # OK / ERR / STREAMING / junk
        parts = raw.split(",")
        if len(parts) < N:
            return None
        counts = np.array([int(p) for p in parts[:N]], dtype=float)
        if not in_range(counts):
            read_sample.rejected += 1      # torn row: a framing error, not data
            return None
        return counts, counts * (A_VCC / ADC_MAX_COUNTS)
    except (ValueError, IndexError):
        return None


# Counted on the function rather than in the Controller because this is the only
# place that sees the wire: by the time a row reaches step() it has already been
# dropped. Ten in 241,813 rows on v11's log, and the number is worth watching --
# if it climbs, the link is losing framing and no amount of control tuning is the
# answer. `Controller.bad_samples` is the same test one layer in, for the callers
# that build their own arrays.
read_sample.rejected = 0


# Channel attribute -> Controller array.
_VIEW = dict(zip("enabled healthy bias steady_gain capture_gain ki kd bp_out vel out "
                 "active_gain last_ratio p_term i_term d_term rail_fault locked "
                 "saturated_flag baseline_rms".split(),
                 "enabled healthy bias steady capture ki kd bp vel out "
                 "gain ratio p i d rail locked sat baseline".split()))


class Channel:
    """A view of one lane of the controller's arrays. v0..v4 kept per-channel
    state in per-channel objects; here it lives in length-N arrays, but the
    logger, the status line and sim/server.py all read and write
    `ctl.channels[i].<field>` -- including live gain edits from the browser UI --
    so this keeps that surface without a second copy of the state."""

    def __init__(self, ctl, idx):
        object.__setattr__(self, "ctl", ctl)
        object.__setattr__(self, "idx", idx)

    def __getattr__(self, k):
        if k in _VIEW:
            return getattr(self.ctl, _VIEW[k])[self.idx]
        raise AttributeError(k)

    def __setattr__(self, k, v):
        if k in _VIEW:
            getattr(self.ctl, _VIEW[k])[self.idx] = v
        else:
            object.__setattr__(self, k, v)


class Controller:
    """The fast-lock state machine. main() drives it from the serial stream and
    sim/server.py drives the same class from a simulated plant, so there is no
    second implementation of the loop."""

    def __init__(self, dac, dac_channels=None, enable=None, steady=None,
                 capture=None, ki=None, kd=None, bias=None, baseline_file=None):
        f = lambda v, d: np.array(d if v is None else v, dtype=float)
        self.enabled = np.array(ENABLE_CHANNEL if enable is None else enable, bool)
        self.steady, self.capture = f(steady, STEADY_GAIN), f(capture, CAPTURE_GAIN)
        self.ki, self.kd, self.bias = f(ki, KI_GAIN), f(kd, KD_GAIN), f(bias, BIAS)
        self.act = Actuator(dac, DAC_CHANNELS if dac_channels is None else dac_channels)

        self.hp, self.lp = OnePole(BP_LOW_HZ, "high"), OnePole(BP_HIGH_HZ)
        self.dsm, self.dfilt = OnePole(DERIV_SMOOTH_HZ), OnePole(D_SMOOTH_HZ)
        self.rms_sch, self.rms_run = _bank(SCHEDULE_WINDOW_S), _bank(ENVELOPE_WINDOW_S)
        self.env_hist = deque()          # (t, envelope) for the trend test
        self.rms_lock = _bank(LOCK_WINDOW_S)

        for k in ("bp prev_bp vel p i d prev_vel clip_excess gain ratio baseline "
                  "sat_sum rail_sum").split():
            setattr(self, k, np.zeros(N))
        for k in "primed sat locked rail".split():
            setattr(self, k, np.zeros(N, bool))
        self.healthy = np.ones(N, bool)
        # Per-channel output window. v9 had module-level VMIN/VMAX; the trim moves
        # each channel's bias independently, so the window has to move with it.
        self.vmin, self.vmax = self.bias - BIAS_SWING, self.bias + BIAS_SWING
        self.clear_t, self.excess_since = np.full(N, np.inf), np.full(N, np.inf)
        self.locked_since = np.full(N, np.inf)
        self.out, self.prev_out = self.bias.copy(), self.bias.copy()
        # `sat_hist` is the saturation counterpart of `rail_hist`: (t, pinned)
        # over the last SAT_SUSTAIN_S, so the interlock is a fraction of a window
        # in SECONDS rather than a run of consecutive iterations. See sat-window.
        self.rail_hist, self.sat_hist = deque(), deque()
        self.calib, self.events = [], []
        self.channels = [Channel(self, i) for i in range(N)]
        self.state, self.calib_start, self.damping_start = "CALIBRATING", 0.0, None
        self.clear_since = self.lock_time = None
        self.filt_on = self.locked_announced = False
        self.fault_count = self.demote_count = self.reuse_count = 0
        self.damped_for = None       # how long DAMPING lasted before the fault
        # --- fast-calib bookkeeping ---
        self.sub_rms = []            # one RMS vector per COMPLETED sub-window
        self.sub_start, self.sub_n0 = 0.0, 0
        self.calib_took = None       # how long the last calibration actually ran
        self.calib_early = False     # ...and whether it converged or hit the ceiling
        # Last baseline this run accepted, kept so a LATER calibration can be
        # checked against it. See `_set_baseline`; measured on the bench
        # 2026-08-06.
        self.trusted_baseline = None
        self.baseline_refused = False
        self.refusals = 0
        # --- warm-restart bookkeeping ---
        self.baseline_t = None       # when the live baseline was measured
        self.fault_railed = False    # was the fault a SENSOR failure?
        # --- baseline-floor bookkeeping ---
        # Kept SEPARATE from the rail demotion rather than folded into `healthy`
        # alone, because the two have different lifetimes: a rail clears and the
        # channel re-arms on a timer, a signal-less sensor does not get better on
        # its own. Only a new baseline can overturn this one. `floor_count` is
        # likewise separate from `demote_count`, which means "railed" everywhere
        # it is read.
        self.floor_bad = np.zeros(N, bool)
        self.floor_count = 0
        # --- bias-trim bookkeeping (v7) ---
        # `trim_ref` is the total offset the accepted step is judged against, so
        # a step is only kept if it beat the rig it was measured on.
        self.trim_steps = np.zeros(N, int)
        self.trim_frozen = np.zeros(N, bool)
        self.trim_last = None          # t of the last trim decision
        self.trim_pending = None       # (channel, previous_bias) awaiting judgement
        self.trim_ref = None           # total |counts - MID| before that step
        self.counts_mean = np.full(N, MID_COUNTS)
        self.trim_step_t = None        # t of the last accepted or reverted step
        # --- decimate bookkeeping (v12) ---
        # A running SUM and a count, not a list. This is touched at the WIRE
        # rate -- ~1111 Hz today -- and appending to a list there would allocate
        # a thousand times a second to compute one mean.
        self.acc_c, self.acc_v, self.acc_n = np.zeros(N), np.zeros(N), 0
        self.ctl_t = None            # NOMINAL deadline; None = no step run yet
        self.ctl_prev = None         # ...and when one actually last ran
        self.ctl_dt = CONTROL_PERIOD_S
        self.stepped = False         # did THIS step() call run a control step?
        self.n_avg = 0               # ...and how many raw samples did it average
        self.ctl_steps = 0
        self.bad_samples = 0         # sample-guard rejections seen here
        self.wire_n, self.wire_t0 = 0, None      # for the measured wire rate
        # --- persist-baseline bookkeeping (v12) ---
        # A loaded floor is held HERE and not written into self.baseline, because
        # it is not adopted until BASELINE_WARMUP_S has measured something to
        # check it against. Until then this is an ordinary CALIBRATING run with
        # baseline 0.0, which is what lets every other path in the file stay
        # exactly as it was.
        self.file_baseline = self.file_floor = None
        self.file_note = ""
        if baseline_file is not None:
            self.file_baseline = np.asarray(baseline_file[0], float)
            self.file_floor = np.asarray(baseline_file[1], bool)
        self.warm_used = False       # did this run engage on a file baseline?
        self.baseline_saveable = False   # main() reads and clears this

    # ---- helpers ----------------------------------------------------------
    def drain_events(self):
        out, self.events = self.events, []
        return out

    def _say(self, msg):
        self.events.append("\n" + msg + "\n")

    def _fault(self, t, msg, railed=False):
        self._say("!! " + msg)
        # Only an engagement that actually happened counts. v5 read damping_start
        # unconditionally, which after a rail during CALIBRATING reports the
        # PREVIOUS engagement's length -- harmless there, load-bearing here,
        # since damped_for now decides whether the reuse budget resets.
        self.damped_for = (t - self.damping_start
                           if self.state == "DAMPING" and self.damping_start is not None
                           else None)
        self.fault_railed = bool(railed)
        self.state, self.clear_since = "FAULT", None
        self.fault_count += 1

    @staticmethod
    def _who(m):
        return ", ".join(f"ch{i}" for i in np.nonzero(m)[0])

    @staticmethod
    def _hold(cond, since, t):
        """Per-lane stopwatch: start where `cond` just became true, clear where it
        is false. np.inf is 'not running'."""
        return np.where(cond, np.where(np.isinf(since), t, since), np.inf)

    def _rail_check(self, counts, t):
        railed = (counts <= RAIL_LOW_COUNTS) | (counts >= RAIL_HIGH_COUNTS)
        self.rail_hist.append((t, railed))
        self.rail_sum = self.rail_sum + railed
        while self.rail_hist and t - self.rail_hist[0][0] > RAIL_SUSTAIN_S:
            self.rail_sum = self.rail_sum - self.rail_hist.popleft()[1]
        # A full window must have elapsed, or the first samples of a run trip it
        # on a sample size of one.
        spanned = bool(self.rail_hist) and t - self.rail_hist[0][0] >= RAIL_SUSTAIN_S * 0.9
        self.rail = np.logical_and(
            spanned, self.rail_sum / max(len(self.rail_hist), 1) >= RAIL_FRACTION)

    def _sat_check(self, pinned, t):
        """FIXED (v11), `sat-window`. Was `sat_streak = where(pinned, streak+1, 0)`
        against MAX_CONSECUTIVE_SATURATED = 30 -- a count of consecutive
        ITERATIONS. This is `_rail_check` with `pinned` in place of `railed`, and
        deliberately line-for-line the same shape, because the two interlocks are
        answering the same kind of question and the rail one already got it right.

        What that buys, in order of how much it cost:
          * a threshold in SECONDS. An iteration is not a fixed amount of time --
            it is one stream sample the controller managed to see, and coil
            writes eat those, so the old constant meant 0.41 s on one coil,
            1.28 s on four and 2.40 s on eight. The interlock got less sensitive
            precisely as the rig got harder to control.
          * evidence that survives one clean sample. A pinned output that lets go
            for a single iteration -- one count of ADC noise, one slew step that
            lands a microvolt inside the rail -- used to reset the whole count to
            zero. Combined with the first defect that is why `soft-saturation`
            never fired once in the 240 s v10 bench run of 2026-08-06.

        Maintained in EVERY state, like the rail check and for the same reason:
        in FAULT the outputs sit at bias, `pinned` goes false, and the window
        drains on its own within SAT_SUSTAIN_S. Nothing latches.
        """
        self.sat_hist.append((t, pinned))
        self.sat_sum = self.sat_sum + pinned
        while self.sat_hist and t - self.sat_hist[0][0] > SAT_SUSTAIN_S:
            self.sat_sum = self.sat_sum - self.sat_hist.popleft()[1]
        spanned = bool(self.sat_hist) and t - self.sat_hist[0][0] >= SAT_SUSTAIN_S * 0.9
        self.sat = np.logical_and(
            spanned, self.sat_sum / max(len(self.sat_hist), 1) >= SAT_FRACTION)

    # ---- calibration ------------------------------------------------------
    def _close_subwindow(self, t):
        """Fold the samples since the last boundary into one RMS vector. The raw
        samples are KEPT as well: the ceiling path re-splits the whole window the
        way v5 does, and must give v5's answer."""
        a = np.asarray(self.calib[self.sub_n0:])
        if len(a) >= 2:
            self.sub_rms.append(np.sqrt((a ** 2).mean(0)))
        self.sub_n0, self.sub_start = len(self.calib), t

    def _calib_stationary(self):
        """Has the noise floor stopped moving? TWO tests, and both are needed.

        (1) the trailing CALIB_AGREE_N sub-windows agree with each other. Catches
            a floor that is still obviously jumping around.
        (2) their median agrees with the median of EVERY sub-window so far.
            Catches the case (1) cannot: a ringdown is slow -- tau = Q/(pi*f0) is
            ~16 s, comparable to the whole window -- so three consecutive
            sub-windows part-way down it can look perfectly stationary while
            sitting well away from the true floor. NOT hypothetical: with (1)
            alone at this tolerance a 6 V/s kick at t = 2 s reads as settled at
            t = 16.01 s (its trailing three agree to 1.187) and stores a baseline
            skewed 37.0% -- past the 35% line the suite asserts, so `fast-calib`
            would have broken `runaway-baseline`. Test (2) sees the same three
            windows sitting 1.400 away from the median of the window as a whole
            and runs to the ceiling instead, for 17.6%.

        Deliberately not a variance or an F-test: this compares the same
        statistic the baseline is made of, so a pass means "the number I am about
        to store has stopped changing" rather than "some other number agrees".
        """
        w = self.sub_rms
        if len(w) < max(CALIB_MIN_SUBWINDOWS, CALIB_AGREE_N):
            return False
        a = np.asarray(w)
        tail = a[-CALIB_AGREE_N:]
        if not bool((tail.max(0) / np.maximum(tail.min(0), 1e-9)
                     <= CALIB_AGREE_TOL).all()):
            return False
        m_tail, m_all = np.median(tail, 0), np.median(a, 0)
        spread = (np.maximum(m_tail, m_all)
                  / np.maximum(np.minimum(m_tail, m_all), 1e-9))
        return bool((spread <= CALIB_AGREE_TOL).all())

    def _set_baseline(self, early):
        """`early` picks the estimator, and the ceiling branch is v5's, unchanged:
        median of CALIB_SUBWINDOWS sub-window RMSs over the whole window, so a
        calibration that had to run the full CALIBRATION_S produces exactly the
        number v5 would have produced from the same samples. The early branch is
        the median of the trailing windows that were just shown to agree --
        deliberately not the median of everything, because on an early exit the
        first sub-window still carries the bandpass filter's own start-up
        transient (measured ~8% high) and there is no reason to average it in.

        NOTE (v12): the ceiling branch splits by SAMPLE COUNT, which is only a
        split by TIME if the sample rate is constant across the window. Before
        `decimate` it was not -- the rate depended on the state and on how many
        coils were being written -- so this quietly weighted the sub-windows
        unevenly. Now the calibration samples arrive at CONTROL_HZ and equal
        counts really are equal times. The code is unchanged; it simply became
        true."""
        if early and len(self.sub_rms) >= CALIB_AGREE_N:
            fresh = np.maximum(
                np.median(np.asarray(self.sub_rms[-CALIB_AGREE_N:]), 0), 1e-6)
        else:
            a = np.asarray(self.calib)
            parts = ([p for p in np.array_split(a, CALIB_SUBWINDOWS) if len(p)]
                     if len(a) else [])
            fresh = (
                np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0), 1e-6)
                if parts else np.full(N, 1e-6))

        # Is this a FLOOR, or is it a ringdown being mistaken for one?
        #
        # BENCH 2026-08-06, and this is why the check exists. A kick faulted the
        # loop, it re-faulted four times, the 4/4 counter forced a fresh
        # calibration, and `fast-calib` exited EARLY at 10.0 s reporting
        # "sub-windows agreed" -- while the optic was still ringing down. It
        # measured ch0 = 0.6112 V against a quiet 0.0530 V: 11.5x (ch1 13.4x,
        # ch2 8.3x, ch3 15.5x, ch5 10.5x). Nothing checked it.
        #
        # The root cause is that CALIB_AGREE_TOL tests STATIONARITY, not
        # QUIETNESS. At tau ~ 16 s a ringdown decays only ~12% across a 2 s
        # sub-window, far inside the 1.20 tolerance, so a smooth exponential
        # decay reads as perfectly settled. `fast-calib` is therefore most
        # confident exactly when it is most wrong.
        #
        # The cost of accepting one is what the loaded-baseline docstring above
        # already spells out: the runaway breaker trips at 1.8x of a number that
        # is 11x too big, and a false LOCKED claim gets easier. The identical
        # ratio test was ALREADY being applied to baselines loaded from disk and
        # simply was not applied to freshly measured ones.
        #
        # Only the HIGH side is refused. A fresh floor far BELOW the trusted one
        # is either a genuinely quieter room or the trusted one having been
        # inflated, and refusing that direction would lock a bad high baseline in
        # permanently -- the failure this is meant to prevent.
        ref = self.trusted_baseline
        self.baseline_refused = False
        if ref is not None:
            # Which channels can this test even be applied to?
            #
            # NOT `~self.floor_bad`: `_baseline_floor` runs AFTER this method, so
            # those flags belong to the previous calibration and are cleared on a
            # forced one. Measured 2026-08-06: reading them here refused a whole
            # calibration because ch4 (0.0163V vs 0.0028V, 5.9x) and ch6 (0.0664V
            # vs 0.0031V, 21.3x) tripped it -- both DEMOTED channels, where the
            # trusted value is sensor noise and any real motion is a huge ratio.
            #
            # So derive the mask from `ref` itself, with the same rule
            # `_baseline_floor` uses: a channel sitting under BASELINE_FLOOR_FRAC
            # of the median was not watching the optic when `ref` was measured,
            # and a ratio against noise is a ratio against nothing.
            live = ref > 0
            med = np.median(ref[live]) if live.any() else 0.0
            m = live & (ref >= med * BASELINE_FLOOR_FRAC)
            bad = m & (fresh > BASELINE_SANITY_RATIO * ref)
            # Refusing keeps a floor that may be genuinely too LOW for the room
            # it is now in, which makes the runaway breaker hypersensitive -- so
            # refuse, fault, recalibrate, refuse is a real livelock, and it ends
            # with nothing damping at all. Bound it. Keeping a low floor is the
            # SAFE direction (over-cautious, not blind), so the bound is generous
            # and the surrender is loud.
            if bad.any() and self.refusals >= MAX_BASELINE_REFUSALS:
                self._say(
                    f"!! fresh baseline refused {self.refusals} times running and "
                    "accepted anyway -- the floor really has moved, this is no "
                    "longer a ringdown. The runaway breaker is now scaled off it.")
                bad = np.zeros(N, bool)
            if bad.any():
                self.refusals += 1
                self.baseline_refused = True
                self._say(
                    "!! fresh baseline REFUSED -- "
                    + " / ".join(f"ch{i} measured {fresh[i]:.4f}V against a trusted "
                                 f"{ref[i]:.4f}V ({fresh[i] / ref[i]:.1f}x)"
                                 for i in np.nonzero(bad)[0])
                    + f", over {BASELINE_SANITY_RATIO:.1f}x. That is a ringdown, not "
                      "a floor -- a calibration window can be perfectly STATIONARY "
                      "and still sit far above the floor. Keeping the baseline "
                      "already in hand.")
                # Restore EXPLICITLY from `ref`. `self.baseline` is not still
                # holding the old numbers at this point -- entering CALIBRATING
                # clears it, which is why the banner reports "nothing measured
                # yet". Measured 2026-08-06: returning without this left the
                # baseline all zeros, every ratio read 0.00, and the runaway
                # breaker lost its denominator entirely -- strictly worse than
                # the inflated floor this check exists to reject.
                self.baseline = np.maximum(ref, 1e-6)
                self.calib, self.sub_rms, self.sub_n0 = [], [], 0
                return

        self.baseline = fresh
        self.trusted_baseline = fresh.copy()
        self.refusals = 0
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0

    def _start_calibration(self, t):
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_start = self.sub_start = t

    def _warm_from_file(self, t):
        """`persist-baseline`, the run-time half. Preflight has already decided
        the FILE is admissible -- schema, age, and a fingerprint of every gain,
        bias, filter corner, rate and scale factor that could change what a
        baseline means. What is left is the one question a fingerprint cannot
        answer: is this still the same OPTIC, in the same alignment, in the same
        room?

        So the loop measures for BASELINE_WARMUP_S before it closes, and the
        loaded floor has to agree with what those seconds see, per channel, to
        within BASELINE_SANITY_RATIO. On the bench the floor reproduces to
        1.21-1.35x minutes apart and moves 2.0-2.4x over two days; what this has
        to catch is a flag that has been moved, which shows up as 12x (a5 against
        a0) or 200x (a4/a6/a7). 3.0 sits in that gap, biased toward REFUSING --
        refusing costs 20 s, accepting a wrong floor costs the runaway breaker
        its sensitivity AND makes a false LOCKED claim easier.

        On refusal nothing is thrown away: the warm-up samples stay in
        `self.calib` and `calib_start` is untouched, so the next control step
        takes the ordinary `fast-calib` path with a two-second head start.

        Channels the FILE already demoted are excluded from the comparison and
        stay demoted. Their stored baseline is a number measured from a sensor
        that was not watching the optic, so there is nothing for it to agree
        with, and re-arming them here would hand the runaway breaker back the
        zero denominator `baseline-floor` exists to take away.
        """
        if t - self.calib_start < BASELINE_WARMUP_S:
            return
        a = np.asarray(self.calib)
        # The trailing 60% only. The first ~0.8 s is the highpass priming itself
        # (tau = 0.398 s), which reads high -- `_set_baseline` measures the same
        # effect at ~8% over a 2 s sub-window and drops the first one for it.
        a = a[int(len(a) * 0.4):]
        seen = np.sqrt((a ** 2).mean(0)) if len(a) >= 2 else np.zeros(N)
        loaded, floor = self.file_baseline, self.file_floor
        self.file_baseline = None                 # decided either way, once
        m = ~floor & (loaded > 0)
        ratio = np.where(m, np.maximum(seen, 1e-9) / np.maximum(loaded, 1e-9), 1.0)
        bad = m & ((ratio > BASELINE_SANITY_RATIO)
                   | (ratio < 1.0 / BASELINE_SANITY_RATIO))
        if not m.any() or bad.any():
            self._say("!! stored baseline REFUSED at the warm-up check -- "
                      + (f"{self._who(bad)} measured "
                         + " / ".join(f"{seen[i]:.4f}V against a stored "
                                      f"{loaded[i]:.4f}V ({ratio[i]:.2f}x)"
                                      for i in np.nonzero(bad)[0])
                         if bad.any() else
                         "the file demoted every channel, so there is nothing "
                         "left to check it against")
                      + f", outside {BASELINE_SANITY_RATIO:.1f}x. Something in "
                      f"the rig or the room has moved. Measuring a fresh floor "
                      f"the long way (up to {CALIBRATION_S:.0f}s).")
            return
        self.baseline = np.maximum(loaded, 1e-6)
        # A floor that just passed the sanity check against live warm-up samples
        # is exactly what a later calibration should be measured against, so a
        # warm-started run gets the same protection as a cold one.
        self.trusted_baseline = self.baseline.copy()
        self.baseline_t = t
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        self.calib_took, self.calib_early = t - self.calib_start, True
        self.reuse_count = 0
        self.floor_bad = floor.copy()
        self.healthy[floor], self.gain[floor] = False, 0.0
        self.clear_t[floor] = np.inf
        self.warm_used = True
        self.state, self.damping_start = "DAMPING", t
        self.locked_announced = False
        self._say(f"[DAMPING] engaged on the STORED baseline after {self.calib_took:.1f}s "
                  f"of warm-up (a full calibration is {CALIBRATION_S:.0f}s): "
                  + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                  + ". Measured over the warm-up: "
                  + ", ".join(f"ch{i}={seen[i]:.4f}V" for i in np.nonzero(m)[0])
                  + f" -- worst disagreement {np.abs(np.log(ratio[m])).max():.2f} in log, "
                    f"inside the {BASELINE_SANITY_RATIO:.1f}x window."
                  + (f" {self._who(floor)} stays demoted -- the file says they "
                     f"were not measuring the optic." if floor.any() else "")
                  + f" Gain schedules {CAPTURE_GAIN[0]:+.3f} -> {STEADY_GAIN[0]:+.3f}, "
                    f"ramping from zero.")

    def _baseline_floor(self, t):
        """Demote any channel whose freshly-measured baseline is implausibly
        small NEXT TO THE OTHERS. Runs once, at the end of every calibration.

        The failure it exists for is a sensor that is powered, reading, not
        railed and not measuring the optic -- an OSEM flag sitting outside the
        partial shadow where a shadow sensor is linear. Its bandpassed RMS is
        its own electronics noise, so it calibrates a baseline near zero, and
        every test downstream of calibration is a RATIO against that number:

            gain schedule    ratio = env / baseline    -> pinned to CAPTURE_GAIN
            lock detector    env < LOCK_RMS_FACTOR x baseline  -> never satisfied
            runaway breaker  env > RUNAWAY_MULTIPLE x baseline -> FAULTS THE RIG

        The third one is the expensive one and it is not hypothetical: see the
        file docstring for the ten faults in 145 s it cost on the bench.

        WHY THE MEDIAN, and why a fraction of it rather than a level in volts.
        The reference has to come from the rig itself, because the honest
        baseline depends on the lab (seismic background) and on the sensor
        (counts-per-metre is an ALIGNMENT property -- a5 sees the same motion at
        ~1/12 of a0's gain). The median over the enabled channels is that
        reference, and it buys the safety property outright:

            the largest enabled baseline is >= the median > the threshold,
            so AT LEAST ONE enabled channel always survives this check.

        No quorum arithmetic, no cap on how many may be dropped -- the statistic
        cannot demote more than half the enabled channels, ever.

        AND WHAT IT DOES WHEN MOST OF THE RIG IS SIGNAL-LESS: nothing at all.
        With five of eight channels dead the median IS a dead channel, the
        threshold collapses to a tenth of nothing, and no channel is under it.
        That is the intended answer, not a gap. A guard whose reference has
        itself been captured cannot tell which half is broken, and the safe
        failure is to leave the rig exactly as v9 had it -- visibly faulting,
        with every baseline printed in the DAMPING banner -- rather than to
        demote six channels on the word of two. Same reasoning as the quorum:
        this file never silently reconfigures the rig around a majority failure.

        Enabled channels only, on both sides of the comparison? No -- the
        reference is the enabled set, but EVERY channel is tested against it. A
        config-disabled channel is still filtered, rail-checked and, critically,
        still watched by the runaway breaker (`_damping` gates that on `healthy`,
        not on `live`), so a disabled dead channel faults the rig just as hard as
        an enabled one. On the bench it was disabled channels that did it.
        """
        ref = self.baseline[self.enabled]
        if ref.size == 0:
            return
        med = float(np.median(ref))
        bad = self.baseline < med * BASELINE_FLOOR_FRAC
        self.floor_bad = bad
        if not bad.any():
            return
        # Demoted the same way a rail demotes: unhealthy, gain zeroed so it ramps
        # from zero if a later calibration lets it back in, output parked at bias
        # by _actuate. clear_t is pinned to inf as well -- belt and braces, since
        # _health also refuses to start a re-arm clock on these.
        self.healthy[bad], self.gain[bad], self.clear_t[bad] = False, 0.0, np.inf
        self.floor_count += int(bad.sum())
        self._say("!! " + self._who(bad) + " demoted -- baseline "
                  + " / ".join(f"{self.baseline[i]:.4f}" for i in np.nonzero(bad)[0])
                  + f"V, under {BASELINE_FLOOR_FRAC:.0%} of the {med:.4f}V median "
                  f"across enabled channels. That sensor is not measuring the "
                  f"optic, so a ratio against it is a ratio against nothing -- "
                  f"held at bias and out of the interlocks. "
                  f"{int((self.enabled & self.healthy).sum())}/"
                  f"{int(self.enabled.sum())} still damping.")

    def _health(self, t):
        """Demote a railed channel; re-arm it REARM_SUSTAIN_S after its rail
        clears. Only RAIL demotes -- a saturated channel's measurement is fine and
        only its output is clipped, so parking it would remove damping authority
        at peak amplitude. Filters and baseline are deliberately carried across
        the gap; the short RMS windows are not. versions.md section v4."""
        drop = self.healthy & self.rail
        self.healthy[drop], self.gain[drop] = False, 0.0   # so gain ramps from zero
        self.clear_t[self.rail] = np.inf
        # `~floor_bad` is what makes `baseline-floor` stick. This re-arm path is
        # written for a rail, which is an event that ENDS: the flag comes back,
        # the rail clears, and REARM_SUSTAIN_S later the channel is fine. A
        # signal-less sensor has no such event -- there is nothing to clear,
        # because it never railed in the first place -- so without this the guard
        # would demote the channel and then hand it straight back 2 s later,
        # every 2 s, forever. Only a fresh calibration overturns a floor demotion.
        idle = ~self.healthy & ~self.rail & ~self.floor_bad
        self.clear_t[idle & np.isinf(self.clear_t)] = t
        back = idle & (t - self.clear_t >= REARM_SUSTAIN_S)
        self.healthy[back], self.clear_t[back] = True, np.inf
        for i in np.nonzero(back)[0]:
            self.rms_run[i].reset()
            self.rms_sch[i].reset()
        self.excess_since[back], self.ratio[back] = np.inf, 0.0
        return drop, back

    # ---- the loop ---------------------------------------------------------
    @property
    def wire_hz(self):
        """The MEASURED arrival rate. Not a constant anywhere in this repo -- it
        has been 12.5, 18, 23.5, 73 and ~1111 Hz depending on coil count and
        transport -- which is the whole reason `decimate` exists. Reported in the
        status line and fingerprinted into the baseline file."""
        span = (self.wire_t - self.wire_t0) if self.wire_t0 is not None else 0.0
        return (self.wire_n / span) if span > 0.5 else float("nan")

    def step(self, counts, volts, t, dt):
        """FIXED (v12), `decimate`. Called once per arriving sample, as before --
        the signature and the contract are unchanged, and `sim/server.py` and
        `harness.py` need no edit -- but only the FULL-RATE half runs every time.
        The control step fires on elapsed time and consumes the MEAN of whatever
        arrived since the last one.

        Three things live in the full-rate half, and each is there deliberately:
          * the accumulator, obviously;
          * `counts_mean`, the trim's rest-position EMA. Its time constant is in
            seconds, so more samples is strictly more evidence;
          * `_rail_check`, ON RAW COUNTS. A rail is a fact about a SAMPLE -- the
            converter hit an end stop -- and the mean of eleven samples of which
            nine are pinned is not pinned. RAIL_SUSTAIN_S / RAIL_FRACTION are a
            fraction of a window in SECONDS, so feeding them the whole stream
            does not change what they mean, only how well evidenced they are.

        `dt` (the WIRE interval, from the caller) is used only by `counts_mean`.
        Everything downstream gets `ctl_dt`, measured between control steps from
        `t`, because a controller that owns its rate must own its own clock too.
        """
        counts, volts = np.asarray(counts, float), np.asarray(volts, float)
        self.stepped = False
        # `sample-guard`, second of the two places (see in_range). read_sample()
        # already dropped this at the wire, but sim/server.py and harness.py
        # build their own arrays, and a guard that only exists in the parser is a
        # guard the tests cannot reach.
        if not in_range(counts):
            self.bad_samples += 1
            return self.state
        if self.wire_t0 is None:
            self.wire_t0 = t
        self.wire_t, self.wire_n = t, self.wire_n + 1
        self.acc_c += counts
        self.acc_v += volts
        self.acc_n += 1
        # Where each OSEM is RESTING, which is what the trim steers. A 2 s time
        # constant is two cycles of the ~1 Hz resonance, so the swing averages out
        # and what is left is the rest position. Maintained in every state,
        # including FAULT, so a trim decision never reads a stale mean.
        self.counts_mean += (counts - self.counts_mean) * min(1.0, dt / 2.0)
        self._rail_check(counts, t)

        # Accumulate by TIME and take however many arrived -- never a fixed
        # number of samples, which would turn a baud change into a control-rate
        # change, i.e. exactly the failure this version exists to remove.
        #
        # `ctl_t` is a DEADLINE that advances by whole periods, not a timestamp
        # of the last step, and the difference matters. Setting it to `t` would
        # make the period "the first arrival at or after CONTROL_PERIOD_S", i.e.
        # it would round UP to the wire's own grid: measured, that is 92.6 Hz on
        # a 1111 Hz wire, 86.7 Hz on the simulator's 347 Hz and 83 Hz on 500 Hz
        # -- a control rate that still moves when the baud does, which is a mild
        # version of the problem being fixed. Advancing the deadline keeps the
        # long-run rate at exactly CONTROL_HZ and puts the wire's granularity
        # into JITTER instead, where the measured `ctl_dt` below absorbs it.
        #
        # ...but never accumulate a backlog. If the wire stalls or the loop is
        # descheduled, several deadlines pass at once and firing one step per
        # missed deadline would be a burst of steps on stale data. Re-sync
        # instead: a missed deadline is missed.
        #
        # If the wire is SLOWER than CONTROL_HZ, the re-sync branch is taken on
        # every arrival and the loop degrades to one step per sample -- correct,
        # since you cannot control faster than you can measure, and it is exactly
        # what every DACController build already does.
        if self.ctl_t is None:
            self.ctl_t, self.ctl_prev = t - CONTROL_PERIOD_S, t - CONTROL_PERIOD_S
        if t < self.ctl_t + CONTROL_PERIOD_S:
            return self.state
        self.ctl_t += CONTROL_PERIOD_S
        if t - self.ctl_t >= CONTROL_PERIOD_S:
            self.ctl_t = t                     # missed deadline(s): re-sync
        n = self.acc_n
        mean_counts, mean_volts = self.acc_c / n, self.acc_v / n
        self.acc_c, self.acc_v, self.acc_n = np.zeros(N), np.zeros(N), 0
        # The REAL interval between control steps, not the nominal period. Same
        # clamp main() applied per sample; its upper bound matters more than it
        # used to, because a stall on the wire must not be handed to the
        # differentiator as though it were a real dt.
        self.ctl_dt = min(max(t - self.ctl_prev, 1e-4), 0.05)
        self.ctl_prev, self.stepped, self.n_avg = t, True, n
        self.ctl_steps += 1
        return self._control(mean_counts, mean_volts, t, self.ctl_dt)

    def _control(self, counts, volts, t, dt):
        """One control step, on the mean of the samples since the last one. This
        is v11's `step()` body verbatim apart from the rail check moving out to
        the full-rate half -- same filters, same state machine, same interlocks,
        same order. `counts` and `volts` are FRACTIONAL here, being means; every
        consumer of them either scales linearly (`volts`) or compares against a
        threshold (`counts_mean` in the trim), so nothing needed rounding."""
        self.bp = self.lp.update(self.hp.update(volts, dt), dt).copy()
        if self.filt_on:
            dv = (self.bp - self.prev_bp) / dt if dt > 0 else np.zeros(N)
            self.vel = self.dsm.update(dv, dt).copy()
        self.prev_bp, self.filt_on = self.bp, True

        if self.state == "CALIBRATING":
            self.calib.append(self.bp)
            if self.rail.any():
                self._fault(t, f"{self._who(self.rail)} railed during calibration -- "
                                f"check alignment.", railed=True)
            elif self.file_baseline is not None:
                self._warm_from_file(t)
            else:
                if t - self.sub_start >= CALIB_SUBWINDOW_S:
                    self._close_subwindow(t)
                ceiling = t - self.calib_start >= CALIBRATION_S
                early = (not ceiling) and self._calib_stationary()
                if early or ceiling:
                    if ceiling and len(self.calib) > self.sub_n0:
                        self._close_subwindow(t)     # flush the partial tail
                    self.calib_took, self.calib_early = t - self.calib_start, early
                    self._set_baseline(early)
                    self.baseline_t = t
                    self.reuse_count = 0
                    self.state, self.damping_start = "DAMPING", t
                    self.locked_announced = False
                    self._say(
                        (f"[DAMPING] baseline KEPT after a refused {self.calib_took:.1f}s "
                         "calibration -- these are the numbers already in hand, not "
                         "freshly measured"
                         if self.baseline_refused else
                         f"[DAMPING] baseline set in {self.calib_took:.1f}s "
                         + ("(sub-windows agreed -- stopped early, ceiling is "
                            f"{CALIBRATION_S:.0f}s)" if early
                            else f"(ran the full {CALIBRATION_S:.0f}s ceiling -- the "
                                 "floor never settled, median of sub-windows used)"))
                        + ": "
                        + ", ".join(f"ch{i}={b:.4f}V" for i, b in enumerate(self.baseline))
                        + f". Gain schedules {CAPTURE_GAIN[0]:+.3f} -> "
                          f"{STEADY_GAIN[0]:+.3f}...")
                    # AFTER the banner, deliberately: the operator reads every
                    # baseline first and then the verdict on them, which is the
                    # order in which the demotion makes sense.
                    self._baseline_floor(t)
                    # This one is worth persisting: it is a floor MEASURED at
                    # zero gain over a full window. main() picks the flag up and
                    # does the file I/O, so the Controller stays free of it --
                    # otherwise every simulator run in `harness.py` would write
                    # over the bench's baseline file.
                    self.baseline_saveable = True

        elif self.state == "DAMPING":
            self._damping(t, dt)

        elif self.state == "FAULT":
            self.gain[:] = 0.0
            # `sat` is refreshed in _actuate() on every control step in every
            # state, so this is a live reading and not a latch: in FAULT the
            # outputs sit at bias and it clears on its own.
            if self.rail.any() or self.sat.any():
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = t
            elif t - self.clear_since >= FAULT_CLEAR_SUSTAIN_S:
                self._recover(t)

        self._actuate(t, dt)
        return self.state

    def _moved(self, t, i, new):
        """Apply a bias change to channel `i` and invalidate everything that was
        measured against the old operating point.

        Both trend interlocks compare the envelope against its own past, and a
        bias step injects a ~1 Hz transient right into the band they read. Rather
        than blind them for a ringdown, drop the history so no comparison spans
        the step: the level tests are untouched and the trend tests resume from
        post-step data one RUNAWAY_TREND_LAG_S later."""
        self.bias[i] = new
        self.vmin[i], self.vmax[i] = new - BIAS_SWING, new + BIAS_SWING
        self.trim_step_t = t
        self.env_hist.clear()
        self.excess_since[:] = np.inf
        # The saturation evidence goes with it. A bias step moves this channel's
        # whole output WINDOW (vmin/vmax move with the bias), so pinned samples
        # recorded against the old window are answers to a question that is no
        # longer being asked. v10 zeroed `sat_streak` here for the same reason.
        self.sat_hist.clear()
        self.sat_sum[:] = 0.0

    def _trim(self, t):
        """Nudge one channel's bias a quantum toward mid-scale, then judge it.

        Runs only in DAMPING and no faster than TRIM_PERIOD_S, because a bias
        step is a DC force step and the pendulum needs a ringdown before the new
        rest position means anything.

        One channel at a time, and the verdict is on the TOTAL offset across all
        four -- four coils drive two DOF, so the channels are coupled and a step
        that centres a0 can push a2 further out. A step that does not improve the
        total is put back and that channel is frozen for the rest of the run,
        which is also what makes a wrong SLOPE_SIGN cost one step instead of
        walking a sensor into its rail.
        """
        if self.trim_last is None:
            self.trim_last = t
            return
        if t - self.trim_last < TRIM_PERIOD_S:
            return
        self.trim_last = t
        total = float(np.abs(self.counts_mean - MID_COUNTS).sum())

        if self.trim_pending is not None:          # judge last period's step
            i, was = self.trim_pending
            self.trim_pending = None
            if total >= self.trim_ref:
                self._moved(t, i, was)
                self.trim_frozen[i] = True
                self._say(f"[trim] ch{i} reverted to {was:.2f}V and frozen -- total "
                          f"offset {self.trim_ref:.0f} -> {total:.0f} counts.")
                return
            self._say(f"[trim] ch{i} kept at {self.bias[i]:.2f}V -- total offset "
                      f"{self.trim_ref:.0f} -> {total:.0f} counts.")

        err = self.counts_mean - MID_COUNTS
        elig = (self.enabled & self.healthy & ~self.trim_frozen
                & (self.trim_steps < TRIM_MAX_STEPS)
                & (np.abs(err) > TRIM_DEADBAND_COUNTS))
        if not elig.any():
            return
        i = int(np.argmax(np.where(elig, np.abs(err), -1.0)))
        step = -np.sign(err[i]) * SLOPE_SIGN[i] * BIAS_QUANTUM
        new = float(np.clip(self.bias[i] + step, BIAS_MIN, BIAS_MAX))
        if abs(new - float(self.bias[i])) < 1e-9:   # already against a clamp
            self.trim_frozen[i] = True
            return
        self.trim_pending, self.trim_ref = (i, float(self.bias[i])), total
        self.trim_steps[i] += 1
        was_counts = self.counts_mean[i]
        self._moved(t, i, new)
        self._say(f"[trim] ch{i} rests at {was_counts:.0f} counts "
                  f"({err[i]:+.0f} off mid-scale) -- bias -> {new:.2f}V")

    def _damping(self, t, dt):
        drop, back = self._health(t)
        live, n_conf = self.enabled & self.healthy, int(self.enabled.sum())
        n = int(live.sum())
        for i in np.nonzero(drop)[0]:
            self.demote_count += 1
            self._say(f"!! ch{i} railed -- held at bias, {n}/{n_conf} still damping.")
        for i in np.nonzero(back)[0]:
            self._say(f"[ch{i} back] rail clear {REARM_SUSTAIN_S:.0f}s -- re-engaging on its "
                      f"pre-event baseline {self.baseline[i]:.4f}V, {n}/{n_conf} damping.")
        if n_conf and n < MIN_HEALTHY_CHANNELS:
            return self._fault(t, f"quorum lost -- {n}/{n_conf} healthy, need "
                                  f"{MIN_HEALTHY_CHANNELS}. Freezing everything.",
                               railed=True)

        # Gain schedule: blend capture -> steady on amplitude over this channel's
        # own baseline, slew-limited. Doubles as the startup soft-start.
        self.ratio = np.where(live, _rms(self.rms_sch, t, self.bp, live) / self.baseline, self.ratio)
        frac = np.clip((self.ratio - CAPTURE_LOW_FRAC) / (CAPTURE_HIGH_FRAC - CAPTURE_LOW_FRAC), 0, 1)
        want, cap = self.steady + frac * (self.capture - self.steady), GAIN_SLEW_PER_S * dt
        self.gain = np.where(live, self.gain + np.clip(want - self.gain, -cap, cap), 0.0)

        # Runaway breaker, gated on `healthy` not `live`: a config-disabled channel
        # still watches its own amplitude, but a blind one measures nothing.
        env = _rms(self.rms_run, t, self.bp, self.healthy)
        # FIXED (v9), "runaway-trend". The old test was env > RUNAWAY_MULTIPLE x
        # baseline sustained RUNAWAY_SUSTAIN_S -- a LEVEL test. A runaway is the
        # loop pumping energy IN, which means amplitude GROWING; a large but
        # decaying envelope is the loop succeeding. The level test cannot tell
        # them apart, so after a kick it trips on a timer while the optic rings
        # down, every RUNAWAY_SUSTAIN_S, until the amplitude happens to fall
        # under the line. Measured on hardware 2026-08-04 (v8, 240 s, 3 kicks):
        # ten faults, and the envelope was falling through every one of them --
        # 5.97 -> 4.61, 3.00 -> 3.14, 2.72 -> 2.07. The only re-engagement that
        # survived started at 1.64, just under the 1.8 line. That is the whole
        # fault thrash, and it was in every version back to v0.
        #
        # So: still require the envelope to be high (a decaying transient is not
        # interesting no matter what it does), but ALSO require it to be growing
        # relative to where it was RUNAWAY_TREND_LAG_S ago. Both, sustained.
        self.env_hist.append((t, env.copy()))
        while self.env_hist and t - self.env_hist[0][0] > RUNAWAY_TREND_LAG_S * 2:
            self.env_hist.popleft()
        past = env
        for ts, e in self.env_hist:                    # oldest sample >= lag old
            if t - ts >= RUNAWAY_TREND_LAG_S:
                past = e
                break
        growing = env > past * RUNAWAY_GROWTH_FRAC
        high = env > self.baseline * RUNAWAY_MULTIPLE
        self.excess_since = self._hold(self.healthy & high & growing, self.excess_since, t)
        run_trip = self.healthy & (t - self.excess_since >= RUNAWAY_SUSTAIN_S)
        # "soft-saturation", second half. `sat` is now v11's `sat-window` reading
        # -- pinned for SAT_FRACTION of the last SAT_SUSTAIN_S rather than v10's
        # run of consecutive iterations; what v10 added is the
        # question v9 never asked -- is the clipped output WINNING? A falling
        # envelope says yes, and freezing the loop would throw away the only
        # thing working. A flat or rising one says no, and that is a real
        # saturation fault: a pumped or over-driven loop that has run out of
        # authority. Same lag and the same env_hist as the runaway trend, so
        # "falling" means exactly one thing across both interlocks.
        sat_trip = self.healthy & self.sat & ~(env < past * SAT_DECAY_FRAC)
        if run_trip.any():
            return self._fault(t, f"{self._who(run_trip)} runaway -- freezing all "
                                  f"channels, entering FAULT.")
        if sat_trip.any():
            return self._fault(t, f"{self._who(sat_trip)} pinned against the rail, "
                                  f"still demanding more, and not winning -- freezing "
                                  f"all channels, entering FAULT.")

        # Lock detector. A demoted channel sits at bias with a flatlined bandpass,
        # which an RMS-only test reads as the steadiest axis on the rack, so it may
        # neither lock nor block a lock.
        quiet = live & (_rms(self.rms_lock, t, self.bp, live) < self.baseline * LOCK_RMS_FACTOR)
        self.locked_since = self._hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= LOCK_SUSTAIN_S)
        self._trim(t)
        if n and self.locked[live].all() and not self.locked_announced:
            self.lock_time = t - self.damping_start
            self._say(f"*** LOCKED -- {self.lock_time:.1f}s after gain was applied "
                      f"(t={t:.1f}s total)"
                      + ("" if n == n_conf else f" -- DEGRADED, {n}/{n_conf} channels") + " ***")
            self.locked_announced = True

    def _actuate(self, t, dt):
        # Takes `t` as of v11: the saturation interlock is a time window now, not
        # a count of iterations, so this needs the clock the rest of the file uses.
        live = self.enabled & self.healthy & (self.state == "DAMPING")
        err = -self.vel                                # setpoint is zero velocity
        self.p = self.gain * err
        self.prev_vel[~self.primed] = self.vel[~self.primed]
        self.primed[:] = True
        # Derivative on the MEASUREMENT: identical while the setpoint is a constant
        # zero, but it cannot kick if that ever changes.
        dv = (self.vel - self.prev_vel) / dt if dt > 0 else np.zeros(N)
        self.prev_vel = self.vel.copy()
        self.d = -self.kd * self.dfilt.update(dv, dt)
        # Back-calculation anti-windup: the integrator unwinds at TRACK_TC_S from
        # last sample's clip rather than freezing at a limit.
        self.i = np.clip(self.i + (self.ki * err - self.clip_excess / TRACK_TC_S) * dt,
                         -I_CLAMP_V, I_CLAMP_V)

        raw = self.bias + self.p + self.i + self.d
        clipped = np.clip(raw, self.vmin, self.vmax)
        self.clip_excess = np.where(live, raw - clipped, 0.0)
        # Bumpless re-entry for every lane not actuating -- FAULT, config-disabled
        # and demoted are all the same case, so none can resume with a stale
        # integral or a derivative step across the gap.
        dead = ~live
        if dead.any():
            self.p[dead] = self.i[dead] = self.d[dead] = self.prev_vel[dead] = 0.0
            self.primed[dead] = False
            self.dfilt.reset(dead)
        cap = MAX_SLEW_PER_S * dt
        target = np.where(live, clipped, self.bias)
        self.out = self.prev_out + np.clip(target - self.prev_out, -cap, cap)
        self.prev_out = self.out.copy()
        # Maintained HERE, where the output is actually known, so it refreshes
        # every sample in every state including FAULT -- which is what stops the
        # v0/v1/v2 saturation latch.
        pinned = (self.out <= self.vmin + 1e-6) | (self.out >= self.vmax - 1e-6)
        # FIXED (v11), "sat-window": a fraction of a window in SECONDS, not a run
        # of consecutive iterations. See _sat_check.
        self._sat_check(pinned, t)
        # FIXED (v10), "soft-saturation". v9 faulted the rig on the streak alone.
        # But clipping only removes authority in ONE direction -- an output pinned
        # at vmax can still pull all the way down -- so a clipped loop is a
        # weakened loop, not a broken one, and it is usually still the thing
        # bringing the optic back. Faulting hands the optic a frozen actuator
        # instead of a half-strength one, which is strictly worse.
        #
        # The missing question is whether the clipped output is ACHIEVING
        # anything, and that is answered by the envelope at the trip below, where
        # it is in hand. Not by clip_excess: the first draft of v10 gated on
        # `|clip_excess| > SAT_EXCESS_V` as "still demanding more than the rail
        # can give", which is wrong for a reason worth recording. Back-calculation
        # anti-windup exists precisely to drive the integrator down until `raw`
        # stops exceeding the rail, so under SUSTAINED clipping clip_excess decays
        # toward zero by design. The test therefore went false exactly when
        # saturation was worst, and the suite caught it: "over-gain plus a
        # disturbance trips a fault -- 0 fault(s)", i.e. a pumped loop that
        # nothing stopped. An anti-windup residual cannot measure unmet demand.
        self.act.send(self.out)

    def _why_reuse(self, t):
        """Whether the fault path may re-engage on the baseline in hand, and the
        one-line reason either way. The four re-measure cases are the file
        docstring's; this is where each of them is enforced."""
        if not bool(self.baseline.all()):
            return False, "nothing measured yet"
        if self.fault_railed:
            return False, ("the fault was a RAIL -- a floor measured through a "
                           "suspect sensor is suspect with it")
        # A trim step deliberately does NOT invalidate the baseline here, though
        # the first draft of v10 had it do so. The baseline is a BANDPASSED RMS
        # (BP_LOW_HZ..BP_HIGH_HZ); a bias step moves the DC rest position, which
        # the bandpass removes. It is not a floor measured at an operating point.
        # The second-order effect is real -- an OSEM is a shadow sensor, so moving
        # the flag changes counts-per-metre and therefore the floor in volts --
        # but a quantum is ~25-50 counts of 1023 and the floor feeds thresholds
        # like RUNAWAY_MULTIPLE = 1.8, so a few percent of sensor-gain error is
        # nowhere near any of them. Refusing reuse on a trim step instead disabled
        # warm-restart outright, since the trim steps every TRIM_PERIOD_S and the
        # baseline is measured once.
        age = None if self.baseline_t is None else t - self.baseline_t
        if age is not None and age > BASELINE_MAX_AGE_S:
            return False, (f"the baseline is {age:.0f}s old, over the "
                           f"{BASELINE_MAX_AGE_S:.0f}s limit")
        if self.reuse_count >= MAX_BASELINE_REUSE:
            return False, (f"already reused {self.reuse_count} times without a "
                           f"{FAST_REFAULT_S:.0f}s engagement in between")
        if self.damped_for is not None and self.damped_for < FAST_REFAULT_S:
            return True, (f"re-faulted after only {self.damped_for:.1f}s, so the "
                          f"disturbance is still there and a fresh window would "
                          f"measure it rather than the floor")
        return True, ("the previous engagement was healthy, so this baseline is "
                      "known good and the fault was a transient -- measuring a "
                      "new floor now would measure the ringdown")

    def _recover(self, t):
        # Leaving FAULT. Either straight back to DAMPING on the baseline already
        # in hand, or a full recalibration. CALIBRATION_S of zero gain is not free
        # and is not always even correct -- see _why_reuse and the file docstring.
        if self.damped_for is not None and self.damped_for >= FAST_REFAULT_S:
            self.reuse_count = 0        # the loop itself validated this baseline
        reuse, why = self._why_reuse(t)
        keep, keep_t = self.baseline.copy(), self.baseline_t
        keep_floor = self.floor_bad.copy()
        self.calib, self.sub_rms, self.sub_n0 = [], [], 0
        for k in "baseline gain ratio sat_sum clip_excess p i d prev_vel".split():
            getattr(self, k)[:] = 0.0
        # `sat_sum` above is only half of it -- the window it sums over has to go
        # with it, or the next SAT_SUSTAIN_S of drain would carry pinned samples
        # from before the fault into the re-engaged loop's evidence.
        self.sat_hist.clear()
        self.primed[:] = self.sat[:] = self.locked[:] = False
        self.locked_since[:] = self.excess_since[:] = self.clear_t[:] = np.inf
        self.healthy[:] = True          # a recovery forgets every demotion...
        self.floor_bad[:] = False
        self.dfilt.reset()
        for b in self.rms_sch + self.rms_run + self.rms_lock:
            b.reset()
        if reuse:
            self.reuse_count += 1
            self.baseline, self.baseline_t = keep, keep_t
            # ...except a floor demotion, which travels WITH the baseline it was
            # derived from. `warm-restart` re-engages on a baseline it did not
            # re-measure, so the verdict on that baseline has not been re-measured
            # either and still holds. Re-arming a signal-less channel here would
            # hand the runaway breaker back the same zero denominator and re-fault
            # the rig inside RUNAWAY_SUSTAIN_S -- the v5.5 thrash, one layer down.
            self.floor_bad = keep_floor
            self.healthy[keep_floor] = False
            self.state, self.damping_start = "DAMPING", t
            self.locked_announced = False
            self._say(f"[recovered] re-engaging on the baseline already in hand -- "
                      f"{why} ({self.reuse_count}/{MAX_BASELINE_REUSE} before a "
                      f"forced re-calibration). Gain ramps from zero."
                      + (f" {self._who(keep_floor)} stays demoted -- same "
                         f"baseline, same verdict." if keep_floor.any() else ""))
        else:
            # A full re-calibration re-decides it on new evidence, which is the
            # only thing that can put a floor-demoted channel back in the loop.
            self.reuse_count = 0
            self.baseline_t, self.damping_start = None, None
            self._start_calibration(t)
            self.state = "CALIBRATING"
            self._say(f"[recovered] all channels clear for {FAULT_CLEAR_SUSTAIN_S:.0f}s "
                      f"-- re-calibrating ({why}) for up to {CALIBRATION_S:.0f}s.")
        self.fault_railed = False
        self.clear_since = self.lock_time = None

    # ---- reporting --------------------------------------------------------
    def status_line(self, t):
        def one(i):
            if self.floor_bad[i]:
                # Distinct from DOWN on purpose: DOWN is a rail and will re-arm
                # on its own, NOSIG will not and wants a person with a torch.
                return f"ch{i}:NOSIG bp={self.bp[i]:+.3f}V base={self.baseline[i]:.4f}V"
            if not self.enabled[i]:
                return f"ch{i}:off  bp={self.bp[i]:+.3f}V"
            if not self.healthy[i]:
                return f"ch{i}:DOWN bp={self.bp[i]:+.3f}V"
            flags = ("LOCK" if self.locked[i] else "....") + ("!RAIL" if self.rail[i] else "")
            return (f"ch{i}:{flags} g={self.gain[i]:+.4f} "
                    f"bp={self.bp[i]:+.3f}V ratio={self.ratio[i]:.2f}")
        # `wire/ctl/avg` is here because the whole of v12 is about that ratio,
        # and because a wire rate that has quietly changed is exactly the thing
        # that has invalidated a threshold in this file twice already. If `avg`
        # reads 1.0 the wire is slower than CONTROL_HZ and nothing is being
        # averaged -- true and worth seeing, not a fault.
        rate = (f"{self.wire_hz:.0f}/{CONTROL_HZ:.0f}Hz avg{self.n_avg:d}"
                if self.wire_hz == self.wire_hz else f"--/{CONTROL_HZ:.0f}Hz")
        bad = f" bad{self.bad_samples}" if self.bad_samples else ""
        return (f"[{t:7.1f}s] {self.state:11s} {rate}{bad} "
                + "  ".join(one(i) for i in range(N)))

    def csv_row(self, t, counts):
        """`healthy` is the 13th column and `rail` cannot replace it: a demotion
        outlasts the rail that caused it by REARM_SUSTAIN_S.

        Written for EVERY accepted sample, not every control step -- CLAUDE.md's
        standing practice is that the raw stream goes to disk and `decimate` is
        not a licence to drop nine rows in ten. `ctl` marks the row whose sample
        completed a control step and `n_avg` says how many it averaged, so the
        `n_avg` rows ending at a flagged row are exactly what that step consumed
        and the decimation can be replayed offline from the counts columns. The
        derived columns repeat in between, which is also what the coil sees."""
        row = [f"{t:.4f}", self.state,
               "1" if self.stepped else "0",
               str(self.n_avg if self.stepped else 0)]
        for i in range(N):
            row += [str(int(counts[i])), f"{counts[i] * (A_VCC / ADC_MAX_COUNTS):.4f}"]
            row += [format(getattr(self, k)[i], f) for k, f in _LOG]
            row += [str(int(v[i])) for v in (self.rail, self.locked, self.healthy)]
        return ",".join(row)


# ===================== persist-baseline: the file =====================
# Kept at module level, out of Controller, for one reason: `harness.py` and
# `sim/server.py` step the Controller thousands of times per suite run, and a
# Controller that did its own file I/O would overwrite the bench's measured floor
# with a simulated one. main() is the only caller.
def _fingerprint(bias, enable=None, steady=None, capture=None, ki=None, kd=None):
    """Everything that changes what a baseline MEANS. A mismatch on any of it
    refuses the file -- a floor measured under a different setup is worse than no
    floor, because every downstream test divides by it, and a wrong denominator
    both desensitises the runaway breaker and makes LOCKED easier to declare.

    `bias` is passed in rather than read from the constant because `bias-trim`
    moves it during a run: what has to be stored is the bias the floor was
    MEASURED at, which is the live vector at the end of calibration, and what has
    to match at start-up is the vector the next run will actually begin with.
    """
    g = lambda v, d: [round(float(x), 9) for x in (d if v is None else v)]
    return dict(
        n_channels=int(N),
        enable=[bool(v) for v in (ENABLE_CHANNEL if enable is None else enable)],
        steady=g(steady, STEADY_GAIN), capture=g(capture, CAPTURE_GAIN),
        ki=g(ki, KI_GAIN), kd=g(kd, KD_GAIN), bias=g(bias, BIAS),
        dac_channels=[int(c) for c in DAC_CHANNELS],
        # The band the RMS is taken over. Move a corner and the number is an RMS
        # of something else.
        bp_low_hz=BP_LOW_HZ, bp_high_hz=BP_HIGH_HZ,
        deriv_smooth_hz=DERIV_SMOOTH_HZ, d_smooth_hz=D_SMOOTH_HZ,
        # The rate the loop averages at, and the counts->volts scale.
        control_hz=CONTROL_HZ, a_vcc=A_VCC, adc_max_counts=int(ADC_MAX_COUNTS))


def _same(a, b, tol=1e-9):
    if isinstance(a, list) != isinstance(b, list):
        return False
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def save_baseline(ctl, path=BASELINE_PATH):
    """Write the floor just measured, with the fingerprint it was measured under.

    Written through a temp file and os.replace, which is atomic on every platform
    this runs on: a floor half-written by a Ctrl+C must never be loadable, and a
    JSON file truncated mid-array would either fail to parse (fine) or, worse,
    parse short and hand the loop a baseline of the wrong length.
    """
    payload = dict(
        schema=BASELINE_FILE_SCHEMA,
        written_by=VERSION_TAG,
        measured_unix=time.time(),
        measured_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        baseline_v=[float(v) for v in ctl.baseline],
        # Travels WITH the baseline, for the same reason `warm-restart` carries
        # it across a reuse: the verdict was derived from these numbers, so it is
        # exactly as fresh as they are. Re-arming a signal-less channel on load
        # would hand the runaway breaker back the zero denominator.
        floor_bad=[bool(v) for v in ctl.floor_bad],
        calib_took_s=(None if ctl.calib_took is None else float(ctl.calib_took)),
        calib_early=bool(ctl.calib_early),
        # MEASURED, not configured, so it is compared with a tolerance on load.
        wire_hz=(float(ctl.wire_hz) if ctl.wire_hz == ctl.wire_hz else None),
        config=_fingerprint(ctl.bias, ctl.enabled, ctl.steady, ctl.capture,
                            ctl.ki, ctl.kd))
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
    os.replace(tmp, path)
    return payload


def load_baseline(path=BASELINE_PATH, fresh=False, now=None):
    """-> ((baseline, floor_bad) or None, [lines to print]).

    Every rejection returns a REASON, and main() prints all of them before the
    first sample. The repo's rule is that the file about to run is the thing you
    can read; a constant that arrives silently from somewhere else breaks that,
    so `persist-baseline` is only acceptable if it is loud.
    """
    say = [f"stored baseline: {path}"]
    if fresh:
        say.append("  IGNORED -- fresh calibration forced (--fresh / "
                   "OSEM_FRESH_CALIB). Measuring the floor the long way.")
        return None, say
    try:
        with open(path) as fh:
            d = json.load(fh)
    except FileNotFoundError:
        say.append(f"  none yet. Calibrating for up to {CALIBRATION_S:.0f}s and "
                   f"writing one at the end.")
        return None, say
    except (OSError, ValueError) as e:
        say.append(f"  UNREADABLE ({e}) -- ignored, calibrating from scratch.")
        return None, say

    def no(why):
        say.append(f"  REFUSED -- {why}. Calibrating from scratch "
                   f"(up to {CALIBRATION_S:.0f}s).")
        return None, say

    if d.get("schema") != BASELINE_FILE_SCHEMA:
        return no(f"schema {d.get('schema')!r}, this build reads "
                  f"{BASELINE_FILE_SCHEMA}")
    age = (time.time() if now is None else now) - float(d.get("measured_unix", 0.0))
    say.append(f"  written by {d.get('written_by', '?')} at "
               f"{d.get('measured_utc', '?')}, {age:.0f}s ago")
    if not (0 <= age <= BASELINE_FILE_MAX_AGE_S):
        return no(f"{age:.0f}s old against a {BASELINE_FILE_MAX_AGE_S:.0f}s limit"
                  if age >= 0 else
                  f"measured {-age:.0f}s in the FUTURE -- the clock moved")
    have, want = d.get("config", {}), _fingerprint(BIAS)
    for k in sorted(want):
        if k not in have:
            return no(f"the file does not record `{k}`")
        if not _same(have[k], want[k]):
            return no(f"`{k}` differs: file {have[k]!r}, this build {want[k]!r}")
    # The wire rate is measured, not configured, so it gets a ratio rather than
    # equality. A change bigger than this means a different transport or a
    # different baud -- which is the change that produced this whole version.
    fw = d.get("wire_hz")
    b = d.get("baseline_v") or []
    fl = d.get("floor_bad") or [False] * len(b)
    if len(b) != N or len(fl) != N:
        return no(f"{len(b)} baselines and {len(fl)} floor flags, expected {N}")
    if not all(isinstance(x, (int, float)) and np.isfinite(x) and x > 0 for x in b):
        return no("a baseline is zero, negative or not finite")
    say.append("  " + ", ".join(f"ch{i}={v:.4f}V" + ("(NOSIG)" if fl[i] else "")
                                for i, v in enumerate(b)))
    say.append(f"  measured at {('%.0f Hz' % fw) if fw else 'an unrecorded rate'} "
               f"on the wire, {CONTROL_HZ:.0f} Hz control step, calibration took "
               f"{d.get('calib_took_s') or float('nan'):.1f}s")
    say.append(f"  ACCEPTED provisionally. It is adopted only if the first "
               f"{BASELINE_WARMUP_S:.1f}s of live signal agree with it to within "
               f"{BASELINE_SANITY_RATIO:.1f}x, per channel; otherwise this run "
               f"calibrates normally and says so.")
    return (np.array(b, float), np.array(fl, bool)), say


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    fresh = ("--fresh" in argv
             or os.environ.get("OSEM_FRESH_CALIB", "0") not in ("", "0"))
    loaded, report = load_baseline(BASELINE_PATH, fresh=fresh)
    print()
    for line in report:
        print(line)
    print()

    dac = FastDAC(port=PORT)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))
    # The prompt is a guard against starting a coil-driving run by accident, and
    # that guard is meaningless when there is no terminal attached: `input()` then
    # raises EOFError and kills the run AFTER the biases are already applied, so
    # the failure mode is a board sitting at bias with nothing driving it. If
    # stdin is not a tty there is no human to prompt, so do not pretend there is.
    if sys.stdin.isatty():
        input("DAC biases set. Press Enter to start fast-lock damping (Ctrl+C to stop)... ")
    else:
        print("DAC biases set. stdin is not a tty -- starting without the prompt.")
    dac.start_stream()
    ctl = Controller(dac, baseline_file=loaded)

    os.makedirs("data", exist_ok=True)
    path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_fast_lock.csv")
    log = open(path, "w", buffering=1)
    log.write(CSV_HEADER + "\n")
    print(f"Logging to {path}, every raw sample, control step at {CONTROL_HZ:.0f} Hz "
          f"(`ctl`/`n_avg` mark which samples each step averaged).\n\n"
          + (f"[{ctl.state}] warming up for {BASELINE_WARMUP_S:.1f}s and then "
             f"checking the stored floor against it...\n" if loaded is not None else
             f"[{ctl.state}] measuring baseline noise (outputs held at bias, no "
             f"damping yet). This stops as soon as the floor settles, and after "
             f"{CALIBRATION_S:.0f}s at the latest...\n"))

    start = prev = time.time()
    last_status, rows = 0.0, 0
    next_cue = None                 # armed on the first DAMPING sample, not here
    if KICK_CUE_S > 0:
        print(f"[kick-cue] speaking \"{KICK_CUE_PHRASE}\" every {KICK_CUE_S:.0f}s "
              f"once damping starts. Unset OSEM_KICK_CUE to silence it.\n")
    try:
        while True:
            sample = read_sample(dac.ser)
            if sample is None:
                continue
            counts, volts = sample
            now = time.time()
            dt, prev = min(max(now - prev, 1e-4), 0.05), now
            t = now - start
            ctl.step(counts, volts, t, dt)
            # Only ever True on the step that finished a REAL calibration --
            # never on a `warm-restart` reuse and never on a floor loaded from
            # this same file, so a stale number cannot refresh its own timestamp
            # and live forever.
            if ctl.baseline_saveable:
                ctl.baseline_saveable = False
                try:
                    save_baseline(ctl)
                    print(f"\n[baseline] written to {BASELINE_PATH} -- the next "
                          f"start-up engages in ~{BASELINE_WARMUP_S:.0f}s instead of "
                          f"{ctl.calib_took:.0f}s, if nothing in the fingerprint or "
                          f"the room has moved.\n")
                except OSError as e:
                    print(f"\n[baseline] could NOT be written ({e}) -- the run is "
                          f"unaffected, the next one just calibrates.\n")
            for msg in ctl.drain_events():
                print(msg)
            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                print(ctl.status_line(t))
            # Cue only while DAMPING. Kicking during CALIBRATING poisons the
            # noise floor being measured -- that is the baseline-inflation bug,
            # and asking the operator to kick into it would be manufacturing it
            # on purpose. Kicking during FAULT tells you nothing either: the
            # gains are frozen, so nothing is damping the result. The clock
            # re-arms on each entry to DAMPING rather than free-running, so a
            # fault does not leave a cue queued up to fire mid-recovery.
            # The clock is ABSOLUTE, not re-armed per DAMPING entry. The first
            # version re-armed with a 3 s lead every time the state came back,
            # so a run with 12 faults cued 8 times in 240 s instead of the 5 the
            # interval asks for: every recovery scheduled a fresh cue 3 s later.
            # A cue rate that rises with the fault rate is backwards, because a
            # faulting rig is the one that least needs another kick.
            if KICK_CUE_S > 0:
                if next_cue is None:
                    if ctl.state == "DAMPING":
                        next_cue = t + _KICK_CUE_LEAD_S     # arm once, on first damping
                elif t >= next_cue:
                    next_cue += KICK_CUE_S
                    # Skip a cue that came due while faulted rather than firing
                    # it late: kicking into a recovery measures the recovery.
                    if ctl.state == "DAMPING":
                        kick_cue()
                        print(f"[kick-cue] t={t:.0f}s -- KICK NOW")
            log.write(ctl.csv_row(t, counts) + "\n")
            rows += 1
            if rows % CSV_FLUSH_EVERY_N == 0:
                log.flush()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        log.close()
        dac.stop_stream()
        print("Returning DAC outputs to bias voltages...")
        for c, b in zip(DAC_CHANNELS, BIAS):
            try:
                dac.set_voltage(channel=c, voltage=float(b))
            except RuntimeError:
                pass                      # best effort; the board resets on reconnect
        time.sleep(0.1)
        dac.close()
        print(f"Log saved to {path}")
        print(f"{rows} raw samples, {ctl.ctl_steps} control steps "
              f"({rows / max(ctl.ctl_steps, 1):.1f} averaged per step), "
              f"wire {ctl.wire_hz:.0f} Hz.")
        # Loud, because a rising count is a LINK problem and no amount of control
        # tuning addresses it. v11's own 240 s log had ten.
        print(f"sample-guard: {read_sample.rejected} torn row(s) rejected at the "
              f"wire, {ctl.bad_samples} at the controller."
              + (" v11 would have fed every one of those straight into the "
                 "bandpass." if read_sample.rejected or ctl.bad_samples else ""))


if __name__ == "__main__":
    main()
