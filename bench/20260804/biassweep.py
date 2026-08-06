"""Does coil bias move the optic? -- the electrical-centering question.

Holds all four coils at a DC bias, waits for the pendulum to settle, then
measures each OSEM's resting ADC counts. Steps the bias and repeats.

If resting counts track bias, the coil exerts a DC force and bias is a
positioning knob: it can be trimmed to pull each OSEM toward mid-scale (511)
without touching hardware. If they do not move, the offset is electrical (TIA)
and bias is inert for centering.

Q~50 at ~1 Hz rings for ~16 s, so each step needs a real settle before the mean
means anything. Nothing is damping, so this reads the FREE optic.

All values stay inside the DAC's 0..2.5 V and the firmware's own clamp. Coils
are returned to 0.25 V on the way out, including on Ctrl+C.
"""
import statistics
import sys
import time

sys.path.insert(0, "/Users/gojira/Documents/GitHub.nosync/ligo")
from pyDAC import DACController

PORT = "/dev/cu.usbserial-1120"
DAC_CHANNELS = [1, 3, 5, 7]          # sensor index 0..3 -> coil, rewired 2026-08-03
BIASES = [0.10, 0.25, 0.40, 0.55, 0.70, 0.85]
SETTLE_S = 18.0                      # > one ringdown at Q~50, f0~1 Hz
MEASURE_S = 6.0
MID = 511.5

dac = DACController(port=PORT)
ser = dac.ser


def read_block(seconds):
    ser.reset_input_buffer()
    rows, t0 = [], time.time()
    while time.time() - t0 < seconds:
        parts = ser.readline().decode(errors="replace").strip().split(",")
        if len(parts) < 4:
            continue
        try:
            rows.append([int(p) for p in parts[:4]])
        except ValueError:
            pass
    return rows


try:
    dac.start_stream()
    print("\n  bias    a0             a1             a2             a3          (mean +- sd)")
    print("  " + "-" * 74)
    results = []
    for b in BIASES:
        for ch in DAC_CHANNELS:
            dac.set_voltage(channel=ch, voltage=b)
        time.sleep(SETTLE_S)
        rows = read_block(MEASURE_S)
        if not rows:
            print("  %.2f    no samples" % b)
            continue
        means, sds = [], []
        for i in range(4):
            col = [r[i] for r in rows]
            means.append(statistics.fmean(col))
            sds.append(statistics.pstdev(col))
        results.append((b, means))
        print("  %.2f  " % b + "  ".join("%6.1f+-%4.1f" % (means[i], sds[i]) for i in range(4)))

    print("\n  SLOPE (counts per volt of bias), from first to last point:")
    if len(results) >= 2:
        b0, m0 = results[0]
        b1, m1 = results[-1]
        db = b1 - b0
        for i in range(4):
            slope = (m1[i] - m0[i]) / db
            # counts of bias needed to bring this channel to mid-scale
            trim = ("%.3f V" % ((MID - m0[i]) / slope + b0)) if abs(slope) > 5 else "n/a"
            verdict = "MOVES -- bias is a positioning knob" if abs(slope) > 20 else \
                      "weak" if abs(slope) > 5 else "INERT -- offset is electrical"
            print("   a%d  %+8.1f counts/V   bias for mid-scale: %-9s %s"
                  % (i, slope, trim, verdict))
finally:
    print("\n  restoring bias 0.25 V ...")
    for ch in DAC_CHANNELS:
        try:
            dac.set_voltage(channel=ch, voltage=0.25)
        except RuntimeError:
            pass
    time.sleep(0.2)
    dac.stop_stream()
    dac.close()
