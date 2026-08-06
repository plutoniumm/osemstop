"""Read-only 8-column probe: which analog pins actually have an OSEM on them?

Streams for a fixed window and reports, per column, the DC level and how much
it moves. A live OSEM on a swinging optic wanders by tens of counts; a floating
pin sits at a near-constant value with only ADC noise on it. Never sends SET,
so nothing is driven.
"""
import statistics
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-1120"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

ser = serial.Serial(PORT, 115200, timeout=2.0)
time.sleep(2)
print("handshake: %r" % ser.readline().decode(errors="replace").strip())

ser.reset_input_buffer()
ser.write(b"STREAM\n")
print("STREAM  -> %r" % ser.readline().decode(errors="replace").strip())

rows, ncols, t0 = [], None, time.time()
while time.time() - t0 < SECONDS:
    raw = ser.readline().decode(errors="replace").strip()
    parts = raw.split(",")
    if len(parts) < 2:
        continue
    try:
        vals = [int(p) for p in parts]
    except ValueError:
        continue
    if ncols is None:
        ncols = len(vals)
    if len(vals) == ncols:
        rows.append(vals)
elapsed = time.time() - t0

ser.write(b"STOP\n")
ser.close()

rate = len(rows) / elapsed
print("\n%d rows x %d cols in %.1fs  ->  %.0f Hz  (%.2f ms/sample)\n"
      % (len(rows), ncols, elapsed, rate, 1000.0 / rate))

print("  col |   min   max   mean |  std  p-p |  mean V | verdict")
for i in range(ncols):
    col = [r[i] for r in rows]
    lo, hi = min(col), max(col)
    mean = statistics.fmean(col)
    std = statistics.pstdev(col)
    pp = hi - lo
    if hi <= 3:
        verdict = "RAILED LOW (blind)"
    elif lo >= 1020:
        verdict = "RAILED HIGH"
    elif std < 1.0:
        verdict = "flat -- floating or unconnected"
    elif std < 3.0:
        verdict = "quiet -- noise only?"
    else:
        verdict = "LIVE -- real signal"
    print("  a%d  | %5d %5d %6.1f | %4.1f %4d | %6.3f  | %s"
          % (i, lo, hi, mean, std, pp, mean * 5.02 / 1023, verdict))

# A swinging optic is coherent at ~1 Hz; noise is not. Count mean-crossings as a
# crude frequency estimate to separate "moving" from "noisy".
print("\n  col | mean-crossings/s (a ~1 Hz pendulum gives ~2)")
for i in range(ncols):
    col = [r[i] for r in rows]
    m = statistics.fmean(col)
    xs = sum(1 for a, b in zip(col, col[1:]) if (a - m) * (b - m) < 0)
    print("  a%d  | %6.1f" % (i, xs / elapsed))
