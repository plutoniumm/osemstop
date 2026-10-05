"""ON HARDWARE: one long gain-off record, so Q can finally be measured.

WHY THIS EXISTS. `analysis/ringdown.md` bounds the open-loop decay at
tau > 32 s (Q > 100) but cannot pin it, because nothing in `data/` holds a
contiguous open-loop window longer than 48.8 s -- 45 of 47 are 15-20 s. The
estimator itself returns +0.0072 +- 0.0023 1/s when the truth is exactly zero,
so anything above Q ~ 264 is unmeasurable from windows that short. A single
uninterrupted record several time constants long is the whole fix.

NO KICK, AND THE KICK IS THE POINT. A ringdown wants a large starting amplitude,
but this optic cannot afford one: the ADC allows about 7x headroom over ambient
and a 300 s decay needs roughly 20x, so a kick rails the sensor and the tail --
the only part that carries the decay rate -- is the part that gets clipped. What
is measured here instead is the AMBIENT-DRIVEN motion of an undamped optic, and
its autocorrelation decays at the same gamma as a ringdown would. That is the
same statistic `ringdown.py` already uses, just given a window long enough to
resolve it.

Coils are held at BIAS and never written again, so this drives nothing. The only
way to spoil the measurement is to touch the bench while it runs.
"""
import os
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pyDAC2 import FastDAC                                   # noqa: E402

N = 8
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]
BIAS = [0.25] * N
SECONDS = 320.0            # 10 tau at the tau > 32 s lower bound
SETTLE_S = 20.0            # discarded: the optic is still moving from whatever
                           # last ran, and tau > 32 s means that takes a while


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-1120"
    dac = FastDAC(port=port)
    for c, b in zip(DAC_CHANNELS, BIAS):
        dac.set_voltage(channel=c, voltage=float(b))
    dac.start_stream()

    os.makedirs(os.path.join(os.path.dirname(HERE), "data"), exist_ok=True)
    path = os.path.join(os.path.dirname(HERE), "data",
                        datetime.now().strftime("%Y%m%d_%H%M%S") + "_quiet_openloop.csv")
    t0 = time.time()
    n = 0
    # Line-buffered and closed in a `finally`, per the standing practice in
    # CLAUDE.md: a Ctrl+C or a crash keeps every sample up to that point.
    f = open(path, "w", buffering=1)
    try:
        f.write("time_s," + ",".join(f"a{i}" for i in range(N)) + "\n")
        print(f"recording {SECONDS:.0f}s of GAIN-OFF data to {path}")
        print(f"  first {SETTLE_S:.0f}s are settle and are marked as such")
        print("  DO NOT TOUCH THE BENCH -- no kick, no bump, nothing.")
        last = t0
        while True:
            t = time.time() - t0
            if t >= SECONDS:
                break
            row = dac.read_sample(N)
            if row is None:
                continue
            f.write(f"{t:.5f}," + ",".join(str(v) for v in row) + "\n")
            n += 1
            if time.time() - last >= 20.0:
                last = time.time()
                tag = "settle" if t < SETTLE_S else "RECORDING"
                print(f"  [{t:6.1f}s] {tag}  {n} samples  "
                      + " ".join(f"a{i}={row[i]:4d}" for i in range(N)))
    finally:
        f.close()
        try:
            dac.stop_stream()
            dac.close()
        except Exception:
            pass
        dt = time.time() - t0
        print(f"\ndone: {n} samples in {dt:.1f}s ({n / max(dt, 1e-9):.1f} Hz) -> {path}")
        print(f"usable window: {max(0.0, dt - SETTLE_S):.1f}s after the {SETTLE_S:.0f}s settle")


if __name__ == "__main__":
    main()
