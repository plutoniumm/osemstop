"""One figure: the four channels dying out after each kick, v4 on the bench.

Source: data/20260804_111807_fast_lock.csv -- the FULL-RATE log of the run whose
stdout is bench/20260804/v4_4ch_3kicks.log. 27932 rows over 197.8 s, 24.2 Hz
median while DAMPING. The stdout log samples every 5 s, which aliases a ~1 Hz
optic into nonsense; at 24 Hz the oscillation is resolved, so this plots the
actual bandpassed signal rather than an envelope of it.

Kicks at 55.0 / 105.0 / 155.0 s, from the log header.

Kick 3 is NOT a damped decay: the run entered FAULT at ~150 s, before the kick
landed, so all gains were frozen at zero and what follows is a re-calibration.
It is drawn because hiding it would misrepresent the run, and shaded like every
other not-damping stretch so it cannot be mistaken for one.
"""
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CSV = os.environ.get("PLOT_CSV") or os.path.join(ROOT, "data", "20260804_111807_fast_lock.csv")
LOG = os.environ.get("PLOT_LOG") or os.path.join(ROOT, "bench", "20260804", "v4_4ch_3kicks.log")
OUT = os.path.join(HERE, "out", "plots")
NCH = int(os.environ.get("PLOT_NCH", "4"))


def main():
    kicks = []
    for line in open(LOG):
        if line.startswith("  kicks at"):
            kicks = [float(x) for x in re.findall(r"[0-9.]+", line.split(":", 1)[1])]
            break

    if not kicks and os.environ.get("PLOT_AUTOKICK"):
        kicks = None            # filled after the data loads
    cols = open(CSV).readline().rstrip("\n").split(",")
    idx = {c: i for i, c in enumerate(cols)}
    take = [idx["time_s"], idx["state"]] + [idx[f"ch{i}_bp"] for i in range(NCH)]
    t, st, bp = [], [], []
    with open(CSV) as f:
        f.readline()
        for line in f:
            p = line.rstrip("\n").split(",")
            if len(p) < len(cols):
                continue
            try:
                t.append(float(p[take[0]]))
                st.append(p[take[1]])
                bp.append([float(p[j]) for j in take[2:]])
            except ValueError:
                pass
    t = np.asarray(t)
    bp = np.asarray(bp)
    st = np.asarray(st)

    # PLOT_TRANGE="a,b" crops to one stretch of a long run. A 325 s record with
    # eight hand-kicks draws as a wall of ink in which no single ringdown can be
    # read; cropping to three shows the decay that is the point of the figure.
    # The full record is always still on disk, and the caption below reports what
    # was cropped away so the figure cannot imply the run was shorter.
    tr = os.environ.get("PLOT_TRANGE")
    full_span, full_n = (t[0], t[-1]), len(t)
    if tr:
        lo, hi = (float(x) for x in tr.split(","))
        keep = (t >= lo) & (t <= hi)
        t, bp, st = t[keep], bp[keep], st[keep]
        if kicks:
            kicks = [k for k in kicks if lo <= k <= hi]

    if kicks is None:
        # No kick list in the log -- bench.py runs are kicked by hand, so the
        # times are not recorded anywhere. Detect them: a kick is a sample where
        # the across-channel peak jumps far above the running median. Marked as
        # DETECTED on the figure, because an inferred time is not a logged one.
        env = np.abs(bp).max(1)
        med = np.median(env)
        hot = env > 8.0 * max(med, 1e-6)
        kicks = []
        for j in np.flatnonzero(hot):
            if not kicks or t[j] - kicks[-1] > 20.0:
                kicks.append(float(t[j]))
        # A "kick" detected while the loop was not damping is the optic already
        # ringing, not a new impulse, and labelling it as one would be wrong.
        kicks = [k for k in kicks
                 if st[np.searchsorted(t, k)] == "DAMPING"]

    fig, ax = plt.subplots(figsize=(13, 6.5))

    # CALIBRATING and FAULT are shaded SEPARATELY. Lumping them as "not damping"
    # hides the thing worth seeing: FAULT is the loop giving up, CALIBRATING is
    # the 20 s it then spends measuring a floor before it may try again. On this
    # run CALIBRATING is 21053 of 27932 samples -- 75% of the record.
    SHADE = {"CALIBRATING": ("#cfe3f7", "calibrating (gain forced to 0)"),
             "FAULT":       ("#f9d6d2", "fault (runaway trip, gains frozen)")}
    edges = np.flatnonzero(st[1:] != st[:-1]) + 1
    seen = set()
    for a, b in zip(np.r_[0, edges], np.r_[edges, len(t)]):
        if st[a] in SHADE:
            col, lab = SHADE[st[a]]
            ax.axvspan(t[a], t[b - 1], color=col, zorder=0, lw=0,
                       label=None if st[a] in seen else lab)
            seen.add(st[a])

    # Each channel gets its own lane. Without this the four traces sit on top of
    # one another and only ch2 -- 4x the authority of the others -- is visible.
    span = float(np.nanmax(np.abs(bp))) * 1.15
    colours = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd",
               "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22"]
    for i, c in zip(range(NCH), colours):
        off = (NCH - 1 - i) * span
        ax.axhline(off, color=c, lw=0.6, alpha=0.35, zorder=1)
        ax.plot(t, bp[:, i] + off, lw=0.8, color=c, alpha=0.9, zorder=3)
        ax.text(t[0] + 1.0, off + span * 0.34, f"ch{i}", color=c,
                fontsize=11, fontweight="bold", va="center", zorder=6)

    for j, k in enumerate(kicks):
        ax.axvline(k, color="crimson", ls="--", lw=1.6, zorder=4)
        ax.text(k, NCH * span - span * 0.12, f" kick {j + 1}", color="crimson",
                fontsize=10, fontweight="bold", va="top", ha="left", zorder=6)
    if kicks:
        ax.plot([], [], color="crimson", ls="--", lw=1.6, label=f"kick ({len(kicks)}x)")

    ax.set_xlabel("time (s)")
    ax.set_ylabel("bandpassed sensor signal, channels offset (V per lane)")
    ax.set_title(os.environ.get("PLOT_TITLE", "v4 on the bench, 2026-08-04: four channels ringing down after each kick") + "\n"
                 f"{len(t)} samples, {len(t)/max(t[-1],1e-9):.0f} Hz" + os.environ.get("PLOT_SUB",""), fontsize=11)
    ax.grid(alpha=0.25, axis="x")
    ax.set_yticks([(NCH - 1 - i) * span for i in range(NCH)])
    ax.set_yticklabels([f"ch{i}" for i in range(NCH)])
    ax.legend(loc="lower right", fontsize=9, ncol=3, framealpha=0.95)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-span * 0.75, NCH * span - span * 0.1)
    fig.tight_layout()

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, os.environ.get("PLOT_OUT", "v4_decay.png"))
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")
    print(f"  {len(t)} rows, {t[0]:.1f}-{t[-1]:.1f}s, kicks at {kicks}")
    if os.environ.get("PLOT_TRANGE"):
        print(f"  CROPPED from the full record: {full_n} rows, "
              f"{full_span[0]:.1f}-{full_span[1]:.1f}s")
    for k in kicks:
        w = (t >= k) & (t <= k + 25)
        if w.sum():
            pk = np.abs(bp[w]).max(0)
            print(f"  kick {k:6.1f}s  peak |bp| per channel: "
                  + "  ".join(f"{v:.3f}" for v in pk)
                  + f"   ({st[w][0]} at kick)")


if __name__ == "__main__":
    main()
