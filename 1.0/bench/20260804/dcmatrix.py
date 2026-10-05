"""Per-coil DC actuation matrix: how far does each coil move each OSEM?

Steps ONE coil's bias at a time and averages every sensor over a window long
enough that the ringdown averages out -- the mean IS the new rest position even
while the pendulum still rings, so this need not wait 3 tau.

Transport is done directly rather than through DACController: `set_voltage`
scans only 50 lines for its ack, and at ~350 Hz the OS buffer holds far more
stale samples than that, so a SET issued right after STOP loses the race. Here
STOP is followed by a drain-to-quiet before anything is written.

DC only, nothing resonant, every value inside the DAC's 0..2.5 V. All coils are
returned to BIAS on the way out, including on Ctrl+C.
"""
import statistics
import sys
import time

import numpy as np
import serial

PORT = "/dev/cu.usbserial-1120"
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]      # sensor index -> coil DAC channel
BIAS, STEP = 0.25, 0.20
PRE_S, MEAS_S = 5.0, 10.0
NCH = 8

ser = serial.Serial(PORT, 115200, timeout=1.0)
time.sleep(2)


def drain_quiet(idle=0.3, cap=8.0):
    """Read until nothing arrives for `idle` seconds. This is what makes a
    following command's reply findable."""
    t0, last = time.time(), time.time()
    while time.time() - t0 < cap:
        n = ser.in_waiting
        if n:
            ser.read(n)
            last = time.time()
        elif time.time() - last > idle:
            return
        else:
            time.sleep(0.02)


def cmd(text, want, tries=4):
    for _ in range(tries):
        drain_quiet()
        ser.write((text + "\n").encode())
        t0 = time.time()
        while time.time() - t0 < 2.0:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith(want):
                return line
    raise RuntimeError("no %r reply to %r" % (want, text))


def setv(ch, v):
    cmd("STOP", "STOPPED")
    cmd("SET %d %.4f" % (ch, v), "OK")
    cmd("STREAM", "STREAMING")


def block(seconds):
    ser.reset_input_buffer()
    rows, t0 = [], time.time()
    while time.time() - t0 < seconds:
        p = ser.readline().decode(errors="replace").strip().split(",")
        if len(p) < NCH:
            continue
        try:
            rows.append([int(x) for x in p[:NCH]])
        except ValueError:
            pass
    return [statistics.fmean([r[i] for r in rows]) for i in range(NCH)] if rows else None


try:
    for _ in range(30):
        if ser.readline().decode(errors="replace").strip() == "READY":
            break
    print("  connected", flush=True)
    for ch in DAC_MAP:
        setv(ch, BIAS)
    time.sleep(8)
    base = block(MEAS_S)
    print("\n  baseline: " + "  ".join("%6.1f" % v for v in base), flush=True)

    M = np.zeros((NCH, NCH))                       # M[sensor, coil]
    for j in range(NCH):
        setv(DAC_MAP[j], BIAS + STEP)
        time.sleep(PRE_S)
        up = block(MEAS_S)
        setv(DAC_MAP[j], BIAS)
        time.sleep(PRE_S)
        if up is None:
            print("  coil %d: no samples" % j, flush=True)
            continue
        M[:, j] = [(up[i] - base[i]) / STEP for i in range(NCH)]
        print("  coil %d (ch%d): " % (j, DAC_MAP[j])
              + " ".join("%+6.0f" % M[i, j] for i in range(NCH)), flush=True)

    print("\n  d(counts_i)/d(bias_j)  [row = sensor, col = coil]")
    print("        " + "".join(" coil%d " % j for j in range(NCH)))
    for i in range(NCH):
        print("   a%d   " % i + "".join("%+6.0f" % M[i, j] for j in range(NCH)))

    s = np.linalg.svd(M, compute_uv=False)
    s = s / s[0] if s[0] else s
    print("\n  singular values: " + "  ".join("%.3f" % v for v in s))
    print("  above 10%%: %d    above 3%%: %d    (8 coils on one body -> expect ~4)"
          % (int((s > 0.10).sum()), int((s > 0.03).sum())))
    print("  mean |resp| within 0-3: %.0f   within 4-7: %.0f   across: %.0f / %.0f"
          % (np.abs(M[:4, :4]).mean(), np.abs(M[4:, 4:]).mean(),
             np.abs(M[:4, 4:]).mean(), np.abs(M[4:, :4]).mean()))
    np.save("/tmp/dcmatrix.npy", M)
finally:
    print("\n  restoring coils to %.2f V ..." % BIAS)
    try:
        cmd("STOP", "STOPPED")
        for ch in DAC_MAP:
            cmd("SET %d %.4f" % (ch, BIAS), "OK")
    except Exception as e:
        print("  restore issue: %s" % e)
    ser.close()
