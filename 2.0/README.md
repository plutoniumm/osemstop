# 2.0

Modal (MIMO) damping on a modal Kalman filter, PID only on the coils the modal
law leaves out, one actuator budget over both. `../1.0/` is the frozen previous
tree and still runs from its own directory; `../data/` is shared.

    python run.py preflight            # no port. read it.
    python run.py sim                  # the loop against a synthetic plate
    python run.py replay ../data/X.csv # the estimator, open loop, on a real record
    python run.py identify CENSUS DC   # rebuild rig.json from two 1.0 records
    python run.py bench                # ON HARDWARE
    python harness.py [word ...]       # every scenario, or the ones matching

| file | what it is |
|---|---|
| `MIMO.md` | the whole process written out: sign test, every matrix, the damping, the link, the ADC. Start here. |
| `rig.json` / `rig.py` | everything MEASURED: frequencies, mode labels, Φ, A, slopes, DC matrix, noise model, Kalman gain, PID weights, wiring, bias. `identify` rebuilds all of it from a quiet census and a DC step pass. Nothing else in 2.0 hardcodes a measured number. |
| `control.py` | `Config` (every threshold), `ModalLaw`, `Budget`, `Controller`, `preflight`. The controller takes counts and a time and returns volts; it never sees a port. |
| `transport.py` | `Serial` (the board, firmware 2 binary frames), `Sim` (synthetic plate), `Replay` (a CSV). One protocol. |
| `firmware/firmware.ino` | firmware 2: 16× oversampled binary sample frames, checksummed binary command frames. Still speaks 1.0's ASCII. `run.py flash`. |
| `run.py` | the CLI and the one run loop, which parks the coils on every exit path. Also `link` (exercise the link at bias) and `flash`. |
| `staircase.py` | ON HARDWARE: raise the bias in steps with no feedback and stop on supply or front-end trouble. |
| `harness.py` / `harness_rig.json` | scenarios against `Sim` on a frozen rig, plus one on the live `rig.json`. |
| `stdlib.py`, `pyDAC2.py` | copied unchanged from 1.0: the interlocks, the estimators, the serial handshake. |

## How a channel ends up modal or PID

It is decided by the data in `rig.json`, not by a list in the code.

- A **sensor** is in the Kalman filter if its Φ row is resolved (4 σ over ten
  time segments). The four corners always are.
- A **coil** is in the modal law if its A column is resolved (3 σ).
- A coil that is not gets PID from its own sensor, **only if** that own-sensor
  slope is resolved (3 σ). The gain carries the slope's sign. Unresolved: no gain.
- PID on the modal coils is a fallback only, and only where it is safe: if
  per-channel feedback would pump a mode (`sym(A diag(g) Φ)` not positive
  definite) it is switched off on those coils and a modal drop-out leaves them at
  bias. That is the case on the 2026-10-05 plate.

## Swapping parts

    Controller(rig, cfg)                       # everything from rig.json + defaults
    Controller(rig, replace(cfg, kp=0.02))     # any threshold
    Controller(rig, cfg, modal=None)           # PID only
    Controller(rig, cfg, modal=MyLaw(...))     # anything with .command/.phi/.rowok/.coils
    Controller(rig, cfg, budget=MyBudget(...)) # anything with .update/.modes/.pid/.w

## Not here yet

The measurement passes themselves (`1.0/status.py sensors`, `1.0/slopesign.py`;
they still record over ASCII), the kick test (`1.0/jerk.py`), the stored-baseline
warm start, bias trim, self-kick.
