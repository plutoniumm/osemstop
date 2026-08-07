#!/usr/bin/env python3
"""Prescaler 64 vs 32, head to head, with per-cycle scatter.

The broad sweep said 32 costs a5 nothing. That is the decision that ships, so
it gets a dedicated A/B: prescalers alternate block by block, and every block's
own a5 figure is printed, so the run-to-run scatter is visible next to the
difference being claimed. A difference smaller than the scatter is not a result.
"""
import os, statistics, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wirebench as W
from presc_sweep import block

PORT, BAUD = "/dev/cu.usbserial-1120", 500000
PAIR = [64, 32]
CYCLES = 6
SECS = 8.0


def main():
    ser = W.open_port(PORT, BAUD)
    W.cmd(ser, "MODE BIN", "OK mode=", deadline=2.0)
    per = {p: [] for p in PAIR}          # list of per-channel nf lists
    rates = {p: [] for p in PAIR}
    try:
        for c in range(CYCLES):
            for p in PAIR:
                rows, dt, dec = block(ser, p, SECS)
                if len(rows) < 200:
                    print("  cycle %d presc %3d: short (%d rows)" % (c, p, len(rows)))
                    continue
                cols = list(zip(*rows))
                nf = []
                for col in cols:
                    d2 = [col[k] - 2 * col[k - 1] + col[k - 2]
                          for k in range(2, len(col))]
                    nf.append(statistics.pstdev(d2) / 6 ** 0.5)
                per[p].append(nf)
                rates[p].append(len(rows) / dt)
                print("  cycle %d presc %3d  %7.1f Hz   a5 nf = %.4f   "
                      "(a0 %.3f a7 %.3f)  bad=%d drop=%d"
                      % (c, p, len(rows) / dt, nf[5], nf[0], nf[7],
                         dec.bad, dec.dropped))
                sys.stdout.flush()
    finally:
        try:
            ser.write(b"STOP\n"); time.sleep(0.2)
        finally:
            ser.close()

    print()
    for i in range(8):
        line = "  a%d  " % i
        for p in PAIR:
            v = [n[i] for n in per[p]]
            line += "presc%-4d %.4f +/- %.4f   " % (p, statistics.fmean(v),
                                                    statistics.pstdev(v))
        a = [n[i] for n in per[PAIR[0]]]
        b = [n[i] for n in per[PAIR[1]]]
        d = statistics.fmean(b) - statistics.fmean(a)
        pooled = (statistics.pstdev(a) ** 2 + statistics.pstdev(b) ** 2) ** 0.5
        line += "delta %+.4f (%+.1f%%, %.1f sigma)" % (
            d, 100 * d / statistics.fmean(a), abs(d) / pooled if pooled else 0.0)
        print(line)
    print()
    for p in PAIR:
        print("  presc %3d mean rate %.1f Hz" % (p, statistics.fmean(rates[p])))


if __name__ == "__main__":
    main()
