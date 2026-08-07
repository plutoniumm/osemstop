#!/usr/bin/env python3
"""ADC prescaler vs per-channel noise floor, on the real board.

The AVR datasheet specifies 50-200 kHz ADC clock for full 10-bit accuracy;
prescaler 64 is already 250 kHz and 32 is 500 kHz. Whether that costs bits HERE
is an empirical question about this board, these cables and these OSEMs, so
measure it.

Method
------
* Binary framing, so every sample is checksummed and sequence-numbered and a
  dropped frame is counted rather than silently interpolated.
* Prescalers are visited round-robin (128, 64, 32, 16, then again) rather than
  one long block each, so a slow drift in the optic or the room cannot be
  mistaken for a prescaler effect.
* The noise figure is the SECOND-DIFFERENCE estimator,
      sigma = std(x[n] - 2x[n-1] + x[n-2]) / sqrt(6),
  which for white noise is unbiased and which suppresses the pendulum almost
  perfectly: a sinusoid of amplitude A at frequency f contributes
  A*(2*sin(pi*f*dt))^2, i.e. ~1e-4 counts for A=100, f=1 Hz, dt=1 ms. Plain std
  is reported too, but plain std is dominated by real optic motion and is NOT a
  noise floor.
"""
import os, statistics, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wirebench as W

PORT = "/dev/cu.usbserial-1120"
BAUD = 500000
PRESCALERS = [128, 64, 32, 16]
CYCLES = 3
SECS = 8.0


def block(ser, presc, secs):
    W.cmd(ser, "PRESC %d" % presc, "OK presc=", deadline=2.0)
    W.cmd(ser, "STREAM", "STREAMING", deadline=2.0)
    time.sleep(0.4)
    W.flush_in(ser, 1.0)
    dec = W.BinDecoder()
    rows = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < secs:
        k = ser.in_waiting
        if k:
            dec.feed(ser.read(k), rows)
    dt = time.perf_counter() - t0
    W.stop_stream(ser)
    time.sleep(0.1)
    W.flush_in(ser, 1.0)
    return rows, dt, dec


def main():
    ser = W.open_port(PORT, BAUD)
    W.cmd(ser, "MODE BIN", "OK mode=", deadline=2.0)
    acc = {p: {i: [] for i in range(8)} for p in PRESCALERS}
    rate = {p: [] for p in PRESCALERS}
    integ = {p: [0, 0] for p in PRESCALERS}
    try:
        for c in range(CYCLES):
            for p in PRESCALERS:
                rows, dt, dec = block(ser, p, SECS)
                if len(rows) < 200:
                    print("  cycle %d presc %3d: only %d rows" % (c, p, len(rows)))
                    continue
                rate[p].append(len(rows) / dt)
                integ[p][0] += dec.bad
                integ[p][1] += dec.dropped
                cols = list(zip(*rows))
                for i, col in enumerate(cols):
                    d2 = [col[k] - 2 * col[k - 1] + col[k - 2]
                          for k in range(2, len(col))]
                    acc[p][i].append((statistics.fmean(col),
                                      statistics.pstdev(col),
                                      statistics.pstdev(d2) / 6 ** 0.5))
                print("  cycle %d presc %3d: %5d rows  %7.1f Hz  bad=%d dropped=%d"
                      % (c, p, len(rows), len(rows) / dt, dec.bad, dec.dropped))
                sys.stdout.flush()
    finally:
        try:
            ser.write(b"STOP\n"); time.sleep(0.2)
        finally:
            ser.close()

    print("\n  ADC clock = 16 MHz / prescaler.  nf = second-difference noise floor, LSB")
    print("\n  %-6s %8s %6s %6s %6s %6s %6s %6s %6s %6s  %6s" %
          ("presc", "rate/Hz", *["a%d" % i for i in range(8)], "bad/drop"))
    for p in PRESCALERS:
        if not rate[p]:
            continue
        nf = [statistics.fmean([t[2] for t in acc[p][i]]) for i in range(8)]
        print("  %-6d %8.1f " % (p, statistics.fmean(rate[p])) +
              " ".join("%6.3f" % v for v in nf) +
              "  %d/%d" % tuple(integ[p]))
    print("\n  raw std (dominated by real optic motion, NOT a noise floor)")
    for p in PRESCALERS:
        if not rate[p]:
            continue
        sd = [statistics.fmean([t[1] for t in acc[p][i]]) for i in range(8)]
        print("  %-6d %8s " % (p, "") + " ".join("%6.2f" % v for v in sd))
    print("\n  mean counts")
    for p in PRESCALERS:
        if not rate[p]:
            continue
        mn = [statistics.fmean([t[0] for t in acc[p][i]]) for i in range(8)]
        print("  %-6d %8s " % (p, "") + " ".join("%6.1f" % v for v in mn))


if __name__ == "__main__":
    main()
