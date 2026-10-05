"""Operational uptime over 24 h, under Poisson-distributed mechanical shocks.

Damping speed is not the deliverable on its own. A controller in CALIBRATING has
its gain forced to zero; a controller in FAULT has every gain frozen and every
output held at bias. Neither is removing energy from the optic, so both are
DOWNTIME. This asks: what fraction of a 24 h shift is each version actually
damping?

WHAT IS MEASURED AND WHAT IS ASSUMED. Everything in `BENCH` below is read out of
the bench CSVs at run time -- state fractions, fault counts, burst structure,
FAULT and CALIBRATING durations. The only assumed input is lambda, the shock
rate, which has never been measured on this rig; it is swept over two orders of
magnitude for exactly that reason.

THE MODEL. An alternating renewal process over the state machine's own timing
constants, Monte-Carlo'd rather than solved, because `warm-restart`'s reuse rules
are path-dependent (they depend on how long the PREVIOUS engagement lasted and on
how old the baseline is) and a closed form would have to approximate them away.

  * Shocks arrive Poisson(lambda). lambda counts TRIP-CAPABLE shocks -- ones of
    the magnitude the bench kicks had. Every scripted kick that landed while the
    rig was DAMPING tripped the breaker (v4 2/2, v9 3/3, v10 3/3), so a
    trip probability is not identifiable from this data and is set to 1 by
    definition of lambda. The "not every door slam trips it" uncertainty lives in
    the lambda sweep, not in a fabricated p_trip.
  * A shock landing in FAULT or CALIBRATING is ABSORBED at no marginal cost --
    the rig is already down. This is not an assumption, it was observed: v4's
    third kick (t = 155.0 s) landed in CALIBRATING and cost nothing.
  * A trip produces a BURST of `m` fault entries separated by `g` seconds of
    DAMPING, both measured per version. Burst length is geometric with mean m.
  * Each fault entry costs FAULT_CLEAR_SUSTAIN_S, then either a warm restart
    (0 s) or a full CALIBRATION_S, decided by the version's actual
    `_why_reuse` rules: baseline age vs BASELINE_MAX_AGE_S, reuse_count vs
    MAX_BASELINE_REUSE, and the FAST_REFAULT_S reset.

LOCK_SUSTAIN_S IS COUNTED AS UPTIME. Those 5 s are spent confirming a lock that
has already happened, but the gain is applied and the loop is removing energy
throughout, so by this file's own definition of downtime it is uptime. It also
never appears in the state column -- the CSV says DAMPING -- so counting it as
downtime would mean disagreeing with the ground truth this model is calibrated
against.

Standing practice (CLAUDE.md): every trial is streamed to disk, not just the
summary, so the sweep can be re-analysed without re-running it.

Run:  python3 analysis/uptime.py        (standard library only -- no numpy)
Out:  analysis/out/uptime_*.csv
"""
import csv
import os
import random
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")
DATA = os.path.join(REPO, "data")

SEED = 20260807
TRIALS = 2000
HORIZON_S = 24 * 3600.0

# Lambda, trip-capable shocks per hour. Swept, not chosen: it is the least known
# input in the whole calculation. The top of the range is where the bench
# actually sat -- v9/v10/v11 were kicked 3x in 237.7 s (45.4/h) and v12 5x in
# 244.5 s (73.6/h) -- and that is deliberate provocation, not an operating
# condition.
LAMBDAS_PER_H = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0]
HEADLINE_LAMBDA_PER_H = 10.0

# ---------------------------------------------------------------------------
# The bench runs this is calibrated against. Paths are the raw logs in data/;
# kick times come from the console logs in bench/. `None` means hand kicks at
# unrecorded times (v12, 2026-08-06).
# ---------------------------------------------------------------------------
BENCH = {
    "v4":  dict(csv="20260804_111807_fast_lock.csv", kicks=[55.0, 105.0, 155.0],
                log="bench/20260804/v4_4ch_3kicks.log"),
    "v9":  dict(csv="20260804_135853_fast_lock.csv", kicks=[60.0, 120.0, 180.0],
                log="bench/20260804/v9_4ch_3kicks.log"),
    "v10": dict(csv="20260806_155848_fast_lock.csv", kicks=[60.0, 120.0, 180.0],
                log="bench/20260806/v10_4ch_3kicks.log"),
    "v11": dict(csv="20260806_171500_fast_lock.csv", kicks=[60.0, 120.0, 180.0],
                log="versions.md sec v12 (no console log kept)"),
    "v12": dict(csv="20260806_200822_fast_lock.csv", kicks=None, n_kicks=5,
                log="bench/20260806/v12_lag10_5kicks.log"),
    "v12lag2": dict(csv="20260806_200158_fast_lock.csv", kicks=None, n_kicks=5,
                    log="bench/20260806/v12_lag2_prefix_5kicks.log"),
}

# ---------------------------------------------------------------------------
# Timing constants, read from source. v0 and v4 are deleted -- the values below
# are transcribed from git history and the commit is named so they can be
# checked:
#     git show b486cd0^:osem.v0.py   (CALIBRATION_S = 8.0, line 178)
#     git show 912a9c7^:osem.v4.py   (CALIBRATION_S = 20.0, line 294)
# v9/v12 are read from the files in the tree.
# ---------------------------------------------------------------------------
VERSIONS = {
    "v0": dict(
        calib_s=8.0,            # git show b486cd0^:osem.v0.py:178
        cold_calib_s=8.0,
        fault_hold_s=5.0,       # FAULT_CLEAR_SUSTAIN_S, same in every version
        warm_restart=False,     # no `warm-restart` -- every fault recalibrates
        sat_latch=True,         # FAULT is ABSORBING after a saturation trip
        channels=1,             # ENABLE_CHANNEL = [True, False, False, False]
        m_source="v4 (nearest measured version without warm-restart)",
        g_source="v4",
    ),
    "v4": dict(
        calib_s=20.0,           # git show 912a9c7^:osem.v4.py:294
        cold_calib_s=20.0,
        fault_hold_s=5.0,
        warm_restart=False,
        sat_latch=False,        # FIXES carries `saturation-latch`
        channels=4,
        m_source="measured", g_source="measured",
    ),
    "v9": dict(
        calib_s=20.0,           # osem.v9.py:192, ceiling; ran full on the bench
        cold_calib_s=20.0,
        fault_hold_s=5.0,
        warm_restart=True,
        sat_latch=False,
        channels=4,
        m_source="measured", g_source="measured",
    ),
    "v12": dict(
        calib_s=20.0,           # osem.v12.py:789
        cold_calib_s=20.0,      # the MEASURED run cold-started: baseline.json
                                # did not exist yet ("none yet" in the console
                                # log). BASELINE_WARMUP_S below is what it would
                                # be on a warm start, and is worth 0.02%.
        fault_hold_s=5.0,
        warm_restart=True,
        sat_latch=False,
        channels=8,
        m_source="measured", g_source="measured",
    ),
    "v13": dict(
        calib_s=20.0, cold_calib_s=20.0, fault_hold_s=5.0,
        warm_restart=True, sat_latch=False, channels=8,
        m_source="v12 (imported, not copied)", g_source="v12",
    ),
}

# `warm-restart` reuse rules. Identical in v9 (osem.v9.py:225-227, 707-731) and
# v12 (osem.v12.py:986-987, 2158-2191).
FAST_REFAULT_S = 15.0        # an engagement this long RESETS the reuse budget
MAX_BASELINE_REUSE = 4       # consecutive warm restarts before a forced recal
BASELINE_MAX_AGE_S = 300.0   # wall-clock backstop on the stored floor

# v12 only: `persist-baseline` cuts a COLD start to BASELINE_WARMUP_S if
# data/baseline.json exists and passes the fingerprint + sanity check. The 08-06
# run did not have one. One 18 s difference in 86400 s is 0.02%, so this is
# reported and then ignored.
BASELINE_WARMUP_S = 2.0      # osem.v12.py:1029


# ===========================================================================
# 1. Read the ground truth out of the bench CSVs.
# ===========================================================================
def state_segments(path):
    """(state, t_start, t_end) for every contiguous run of the `state` column.

    Parses only the first two fields of each line. The 08-06 logs are 140-340 MB
    and a csv.DictReader over them is minutes; this is seconds.
    """
    segs, prev, start, last = [], None, None, None
    with open(path) as fh:
        fh.readline()
        for line in fh:
            i = line.find(",")
            j = line.find(",", i + 1)
            if i < 0 or j < 0:
                continue
            try:
                t = float(line[:i])
            except ValueError:
                continue
            s = line[i + 1:j]
            if s != prev:
                if prev is not None:
                    segs.append((prev, start, t))
                prev, start = s, t
            last = t
    if prev is not None:
        segs.append((prev, start, last))
    return segs


def measure(name, spec):
    """Time-weighted state fractions and burst structure for one bench run."""
    segs = state_segments(os.path.join(DATA, spec["csv"]))
    span = segs[-1][2] - segs[0][1]
    tw = {"DAMPING": 0.0, "FAULT": 0.0, "CALIBRATING": 0.0}
    for s, a, b in segs:
        tw[s] = tw.get(s, 0.0) + (b - a)

    damp = [(a, b) for s, a, b in segs if s == "DAMPING"]
    faults = [(a, b) for s, a, b in segs if s == "FAULT"]
    calibs = [(a, b) for s, a, b in segs if s == "CALIBRATING"]

    # A DAMPING interval shorter than FAST_REFAULT_S is the same disturbance
    # still ringing (the controller's own definition of "re-faulted early"), so
    # the fault that ends it belongs to the SAME burst. One longer than that is a
    # genuine re-engagement, so the next fault starts a NEW burst.
    bursts, gaps = 0, []
    for a, _b in faults:
        prev = [d for d in damp if abs(d[1] - a) < 0.05]
        if not prev:
            bursts += 1                      # faulted out of CALIBRATING
            continue
        length = prev[0][1] - prev[0][0]
        if length >= FAST_REFAULT_S:
            bursts += 1
        else:
            gaps.append(length)

    n_kicks = len(spec["kicks"]) if spec["kicks"] else spec.get("n_kicks")
    kicks_while_damping = None
    if spec["kicks"]:
        kicks_while_damping = 0
        for k in spec["kicks"]:
            for s, a, b in segs:
                if a <= k < b:
                    if s == "DAMPING":
                        kicks_while_damping += 1
                    break

    return dict(
        name=name, span_s=span, n_rows=None,
        damping_s=tw["DAMPING"], fault_s=tw["FAULT"], calib_s=tw["CALIBRATING"],
        damping_frac=tw["DAMPING"] / span,
        fault_frac=tw["FAULT"] / span,
        calib_frac=tw["CALIBRATING"] / span,
        n_faults=len(faults), n_bursts=bursts,
        m=len(faults) / bursts if bursts else float("nan"),
        g_s=statistics.median(gaps) if gaps else 0.0,
        fault_hold_s=statistics.median(b - a for a, b in faults) if faults else 0.0,
        calib_durations=[round(b - a, 2) for a, b in calibs],
        n_kicks=n_kicks, kicks_while_damping=kicks_while_damping,
        lambda_per_h=3600.0 * n_kicks / span if n_kicks else float("nan"),
    )


# ===========================================================================
# 2. The model.
# ===========================================================================
def simulate(cfg, lam_per_s, horizon=HORIZON_S, rng=None, cold_calib_s=None):
    """One 24 h shift. Returns seconds in each state plus event counts.

    Everything here is the state machine's own arithmetic; there is no plant, no
    optic and no control law. It is a timing model, deliberately.
    """
    rng = rng or random
    t = 0.0
    calib = cfg["cold_calib_s"] if cold_calib_s is None else cold_calib_s
    damping_s, fault_s = 0.0, 0.0
    calib_s = min(calib, horizon)
    t += calib_s
    baseline_t = t                      # floor measured at end of calibration
    reuse_count = 0
    n_faults, n_recals, n_bursts = 0, 0, 0
    latched = False

    p_continue = 0.0 if cfg["m"] <= 1.0 else 1.0 - 1.0 / cfg["m"]

    while t < horizon:
        # --- DAMPING: wait for the next trip-capable shock -------------------
        wait = rng.expovariate(lam_per_s)
        engaged = min(wait, horizon - t)
        damping_s += engaged
        t += engaged
        if t >= horizon:
            break
        n_bursts += 1

        # --- the burst -------------------------------------------------------
        while True:
            n_faults += 1
            hold = min(cfg["fault_hold_s"], horizon - t)
            fault_s += hold
            t += hold
            if t >= horizon:
                break

            if cfg["sat_latch"] and rng.random() < cfg["p_sat_latch"]:
                # v0/v1/v2 hazard 1: `saturated_flag` is only recomputed inside
                # check_runaway(), which does not run in FAULT. Once set, FAULT
                # is absorbing until a human restarts it.
                fault_s += horizon - t
                t = horizon
                latched = True
                break

            # --- _why_reuse ---------------------------------------------------
            if engaged >= FAST_REFAULT_S:
                reuse_count = 0
            reuse = (cfg["warm_restart"]
                     and (t - baseline_t) <= cfg.get("max_age_s",
                                                     BASELINE_MAX_AGE_S)
                     and reuse_count < MAX_BASELINE_REUSE)
            if reuse:
                reuse_count += 1
            else:
                n_recals += 1
                took = min(cfg["calib_s"], horizon - t)
                calib_s += took
                t += took
                baseline_t = t
                reuse_count = 0
            if t >= horizon:
                break

            if rng.random() >= p_continue:
                break                       # burst over
            engaged = min(cfg["g_s"], horizon - t)
            damping_s += engaged            # the burst gap IS damping
            t += engaged
            if t >= horizon:
                break

        if latched:
            break

    return dict(damping_s=damping_s, fault_s=fault_s, calib_s=calib_s,
                n_faults=n_faults, n_bursts=n_bursts, n_recals=n_recals,
                latched=int(latched))


def closed_form(cfg, lam_per_s):
    """Alternating-renewal cross-check, with the recalibration probability at
    its two bounds. The truth is between them; the Monte-Carlo picks it out."""
    m = cfg["m"]
    up = 1.0 / lam_per_s + (m - 1.0) * cfg["g_s"]
    lo = m * cfg["fault_hold_s"]                       # every fault warm-restarts
    hi = m * (cfg["fault_hold_s"] + cfg["calib_s"])    # every fault recalibrates
    return up / (up + hi), up / (up + lo)


# ===========================================================================
# 3. Run it.
# ===========================================================================
def main():
    os.makedirs(OUT, exist_ok=True)
    rng = random.Random(SEED)

    print("=" * 78)
    print("1. GROUND TRUTH -- time-weighted state fractions from the raw CSVs")
    print("=" * 78)
    bench = {}
    for name, spec in BENCH.items():
        bench[name] = measure(name, spec)

    with open(os.path.join(OUT, "uptime_bench.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run", "csv", "span_s", "damping_pct", "fault_pct",
                    "calib_pct", "n_faults", "n_bursts", "m_faults_per_burst",
                    "burst_gap_s", "fault_hold_s", "calib_durations_s",
                    "n_kicks", "kicks_while_damping", "kick_lambda_per_h"])
        for name, b in bench.items():
            w.writerow([name, BENCH[name]["csv"], f"{b['span_s']:.1f}",
                        f"{100 * b['damping_frac']:.1f}",
                        f"{100 * b['fault_frac']:.1f}",
                        f"{100 * b['calib_frac']:.1f}",
                        b["n_faults"], b["n_bursts"], f"{b['m']:.2f}",
                        f"{b['g_s']:.2f}", f"{b['fault_hold_s']:.3f}",
                        " ".join(str(x) for x in b["calib_durations"]),
                        b["n_kicks"], b["kicks_while_damping"],
                        f"{b['lambda_per_h']:.1f}"])

    hdr = (f"{'run':9s} {'span':>7s} {'DAMP':>6s} {'FAULT':>6s} {'CALIB':>6s} "
           f"{'flt':>4s} {'brst':>5s} {'m':>5s} {'gap':>6s} {'kick/h':>7s}")
    print(hdr)
    for name, b in bench.items():
        print(f"{name:9s} {b['span_s']:7.1f} {100 * b['damping_frac']:5.1f}% "
              f"{100 * b['fault_frac']:5.1f}% {100 * b['calib_frac']:5.1f}% "
              f"{b['n_faults']:4d} {b['n_bursts']:5d} {b['m']:5.2f} "
              f"{b['g_s']:6.2f} {b['lambda_per_h']:7.1f}")
    print("\nNOTE: these are TIME-weighted. Sample-weighted is a different and")
    print("misleading number, because the stream rate is not constant across")
    print("states -- during CALIBRATING no SET traffic shares the wire. v4's")
    n4 = 27932
    print(f"      CALIBRATING is 21053/{n4} = 75.4% of SAMPLES but only "
          f"{100 * bench['v4']['calib_frac']:.1f}% of TIME.")

    # ---- parameters, per version ------------------------------------------
    cfgs = {}
    for v, base in VERSIONS.items():
        cfg = dict(base)
        src = {"v0": "v4", "v4": "v4", "v9": "v9", "v12": "v12", "v13": "v12"}[v]
        cfg["m"] = bench[src]["m"]
        cfg["g_s"] = bench[src]["g_s"]
        cfg["fault_hold_s"] = bench[src]["fault_hold_s"]
        cfg["p_sat_latch"] = 0.0
        cfgs[v] = cfg
    # v0's saturation latch. There is no measurement of how often a trip is a
    # saturation trip rather than a runaway trip: the 08-03 session gives one
    # latch (v2, 31 pinned samples) against one clean recovery (v1, 28 samples),
    # on two different control laws. So it is swept, not asserted.
    P_LATCH_SWEEP = [0.0, 0.1, 0.25, 0.5]
    cfgs["v0"]["p_sat_latch"] = 0.0

    with open(os.path.join(OUT, "uptime_params.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["version", "channels", "cold_calib_s", "calib_s",
                    "fault_hold_s", "warm_restart", "sat_latch",
                    "m_faults_per_burst", "burst_gap_s", "m_source", "g_source"])
        for v, c in cfgs.items():
            w.writerow([v, c["channels"], c["cold_calib_s"], c["calib_s"],
                        f"{c['fault_hold_s']:.3f}", int(c["warm_restart"]),
                        int(c["sat_latch"]), f"{c['m']:.2f}", f"{c['g_s']:.2f}",
                        c["m_source"], c["g_source"]])

    # ---- validation: replay each bench run's own lambda over its own span ---
    print()
    print("=" * 78)
    print("2. VALIDATION -- model run at each bench run's own kick rate and span")
    print("=" * 78)
    print(f"{'run':6s} {'span':>7s} {'lambda/h':>9s} {'measured':>9s} "
          f"{'model':>9s} {'delta':>7s}")
    val_rows = []
    for run, ver in (("v4", "v4"), ("v9", "v9"), ("v10", "v9"), ("v12", "v12")):
        b = bench[run]
        cfg = dict(cfgs[ver])
        cfg["m"], cfg["g_s"] = b["m"], b["g_s"]
        lam = b["n_kicks"] / b["span_s"]
        r = random.Random(SEED + 1)
        got = [simulate(cfg, lam, horizon=b["span_s"], rng=r)["damping_s"] / b["span_s"]
               for _ in range(TRIALS)]
        mu = statistics.mean(got)
        print(f"{run:6s} {b['span_s']:7.1f} {3600 * lam:9.1f} "
              f"{100 * b['damping_frac']:8.1f}% {100 * mu:8.1f}% "
              f"{100 * (mu - b['damping_frac']):+6.1f}")
        val_rows.append([run, ver, f"{b['span_s']:.1f}", f"{3600 * lam:.1f}",
                         f"{100 * b['damping_frac']:.1f}", f"{100 * mu:.1f}",
                         f"{100 * (mu - b['damping_frac']):+.1f}"])
    print("\nv10 is modelled on v9's machinery with v10's own m and gap; it is")
    print("here as an out-of-sample check, since no v10 row is fitted.")
    with open(os.path.join(OUT, "uptime_validation.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run", "modelled_as", "span_s", "lambda_per_h",
                    "measured_damping_pct", "model_damping_pct", "delta_pct"])
        w.writerows(val_rows)

    # ---- the sweep ---------------------------------------------------------
    print()
    print("=" * 78)
    print(f"3. 24 h UPTIME vs LAMBDA  ({TRIALS} trials/cell, seed {SEED})")
    print("=" * 78)
    trials_fh = open(os.path.join(OUT, "uptime_trials.csv"), "w",
                     newline="", buffering=1)
    tw = csv.writer(trials_fh)
    tw.writerow(["version", "lambda_per_h", "trial", "damping_s", "fault_s",
                 "calib_s", "n_faults", "n_bursts", "n_recals", "latched"])

    sweep = {}
    try:
        for v, cfg in cfgs.items():
            for lph in LAMBDAS_PER_H:
                lam = lph / 3600.0
                # v13 is seeded off v12 ON PURPOSE. Its control law, gains and
                # interlocks are v12's, imported not copied, so a different
                # random stream would print a Monte-Carlo difference where
                # there is no behavioural one. Identical seed -> identical row.
                seed_key = "v12" if v == "v13" else v
                r = random.Random(SEED + sum(map(ord, seed_key)) + int(lph))
                res = [simulate(cfg, lam, rng=r) for _ in range(TRIALS)]
                for k, x in enumerate(res):
                    tw.writerow([v, lph, k, f"{x['damping_s']:.1f}",
                                 f"{x['fault_s']:.1f}", f"{x['calib_s']:.1f}",
                                 x["n_faults"], x["n_bursts"], x["n_recals"],
                                 x["latched"]])
                up = [x["damping_s"] / HORIZON_S for x in res]
                sweep[(v, lph)] = dict(
                    up=statistics.mean(up),
                    up_sd=statistics.pstdev(up),
                    fault=statistics.mean(x["fault_s"] for x in res) / HORIZON_S,
                    calib=statistics.mean(x["calib_s"] for x in res) / HORIZON_S,
                    faults=statistics.mean(x["n_faults"] for x in res),
                    recals=statistics.mean(x["n_recals"] for x in res),
                )
    finally:
        trials_fh.close()

    order = ["v0", "v4", "v9", "v12", "v13"]
    print(f"{'lambda/h':>9s} " + " ".join(f"{v:>8s}" for v in order))
    for lph in LAMBDAS_PER_H:
        print(f"{lph:9.0f} " + " ".join(
            f"{100 * sweep[(v, lph)]['up']:7.1f}%" for v in order))
    print("\n(v0 above assumes the saturation latch NEVER fires -- an upper")
    print(" bound. The latch sweep is below.)")

    with open(os.path.join(OUT, "uptime_sweep.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["version", "lambda_per_h", "uptime_pct", "uptime_sd_pct",
                    "fault_pct", "calib_pct", "mean_faults_24h",
                    "mean_recalibrations_24h", "downtime_min_per_24h",
                    "closed_form_lo_pct", "closed_form_hi_pct"])
        for v in order:
            for lph in LAMBDAS_PER_H:
                s = sweep[(v, lph)]
                lo, hi = closed_form(cfgs[v], lph / 3600.0)
                w.writerow([v, lph, f"{100 * s['up']:.2f}",
                            f"{100 * s['up_sd']:.2f}",
                            f"{100 * s['fault']:.2f}", f"{100 * s['calib']:.2f}",
                            f"{s['faults']:.1f}", f"{s['recals']:.1f}",
                            f"{(1 - s['up']) * 1440:.1f}",
                            f"{100 * lo:.2f}", f"{100 * hi:.2f}"])

    # ---- headline ----------------------------------------------------------
    print()
    print("=" * 78)
    print(f"4. HEADLINE at lambda = {HEADLINE_LAMBDA_PER_H:.0f}/h "
          f"(one trip-capable shock every "
          f"{3600 / HEADLINE_LAMBDA_PER_H:.0f} s)")
    print("=" * 78)
    print(f"{'version':8s} {'uptime':>8s} {'downtime':>10s} {'FAULT':>7s} "
          f"{'CALIB':>7s} {'faults':>7s} {'recals':>7s}")
    head_rows = []
    for v in order:
        s = sweep[(v, HEADLINE_LAMBDA_PER_H)]
        print(f"{v:8s} {100 * s['up']:7.1f}% {(1 - s['up']) * 1440:8.0f} min "
              f"{100 * s['fault']:6.1f}% {100 * s['calib']:6.1f}% "
              f"{s['faults']:7.0f} {s['recals']:7.0f}")
        head_rows.append([v, HEADLINE_LAMBDA_PER_H, f"{100 * s['up']:.1f}",
                          f"{(1 - s['up']) * 1440:.0f}",
                          f"{100 * s['fault']:.1f}", f"{100 * s['calib']:.1f}",
                          f"{s['faults']:.0f}", f"{s['recals']:.0f}"])
    with open(os.path.join(OUT, "uptime_headline.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["version", "lambda_per_h", "uptime_pct",
                    "downtime_min_per_24h", "fault_pct", "calib_pct",
                    "mean_faults_24h", "mean_recalibrations_24h"])
        w.writerows(head_rows)

    # ---- v0's absorbing state ---------------------------------------------
    print()
    print("=" * 78)
    print("5. v0 ONLY -- the saturation latch is an ABSORBING state")
    print("=" * 78)
    print("Hazard 1: saturated_flag is recomputed only inside check_runaway(),")
    print("which does not run in FAULT, so one saturation trip ends the shift.")
    print("p_latch is the per-fault probability that the trip is a SATURATION")
    print("trip. It is NOT measured -- 08-03 gives one latch (v2, 31 pinned")
    print("samples) against one clean recovery (v1, 28) on two control laws.")
    print()
    print(f"{'p_latch':>8s} " + " ".join(f"{l:>7.0f}/h" for l in LAMBDAS_PER_H))
    lat_rows = []
    for pl in P_LATCH_SWEEP:
        cfg = dict(cfgs["v0"])
        cfg["p_sat_latch"] = pl
        row = []
        for lph in LAMBDAS_PER_H:
            r = random.Random(SEED + 7 + int(100 * pl) + int(lph))
            res = [simulate(cfg, lph / 3600.0, rng=r) for _ in range(TRIALS)]
            up = statistics.mean(x["damping_s"] for x in res) / HORIZON_S
            lat = statistics.mean(x["latched"] for x in res)
            row.append(up)
            lat_rows.append([pl, lph, f"{100 * up:.2f}", f"{100 * lat:.1f}"])
        print(f"{pl:8.2f} " + " ".join(f"{100 * u:7.1f}%" for u in row))
    with open(os.path.join(OUT, "uptime_v0_latch.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["p_latch_per_fault", "lambda_per_h", "uptime_pct",
                    "pct_of_shifts_latched"])
        w.writerows(lat_rows)

    # ---- where warm-restart stops paying ------------------------------------
    print()
    print("=" * 78)
    print("6. WHERE warm-restart ACTUALLY PAYS")
    print("=" * 78)
    print("`_why_reuse` refuses a warm restart once the floor is older than")
    print(f"BASELINE_MAX_AGE_S = {BASELINE_MAX_AGE_S:.0f} s. Below")
    print(f"lambda = {3600 / BASELINE_MAX_AGE_S:.0f}/h the mean gap between")
    print("faults exceeds that, so nearly every fault recalibrates anyway and")
    print("v9/v12 collapse toward v4's per-fault cost.")
    print()
    print(f"{'lambda/h':>9s} {'v9 recal/fault':>15s} {'v12 recal/fault':>16s} "
          f"{'v12-v4 uptime':>14s}")
    for lph in LAMBDAS_PER_H:
        s9, s12, s4 = (sweep[(v, lph)] for v in ("v9", "v12", "v4"))
        print(f"{lph:9.0f} {s9['recals'] / max(s9['faults'], 1e-9):15.2f} "
              f"{s12['recals'] / max(s12['faults'], 1e-9):16.2f} "
              f"{100 * (s12['up'] - s4['up']):+13.1f}%")

    # ---- counterfactuals ----------------------------------------------------
    print()
    print("=" * 78)
    print(f"7. WHAT WOULD ACTUALLY BUY UPTIME, at lambda = "
          f"{HEADLINE_LAMBDA_PER_H:.0f}/h")
    print("=" * 78)
    print("Each row changes exactly one constant on v12 and nothing else. These")
    print("are counterfactuals on the TIMING MODEL only -- none has been run, and")
    print("each has a safety argument attached to it that this file does not make.")
    print()
    lam = HEADLINE_LAMBDA_PER_H / 3600.0
    base_age = BASELINE_MAX_AGE_S
    cf_rows, cf = [], []
    cf.append(("v12 as shipped", dict(), base_age))
    cf.append(("FAULT_CLEAR_SUSTAIN_S 5.0 -> 2.0",
               dict(fault_hold_s=2.0), base_age))
    cf.append(("CALIBRATION_S 20 -> 2 on the recovery path too "
               "(persist-baseline)", dict(calib_s=BASELINE_WARMUP_S), base_age))
    cf.append(("BASELINE_MAX_AGE_S 300 -> 3600", dict(), 3600.0))
    cf.append(("all three together",
               dict(fault_hold_s=2.0, calib_s=BASELINE_WARMUP_S), 3600.0))
    print(f"{'change':62s} {'uptime':>7s} {'delta':>7s}")
    ref = None
    for label, over, age in cf:
        cfg = dict(cfgs["v12"])
        cfg["max_age_s"] = age
        cfg.update(over)
        r = random.Random(SEED + 99)
        res = [simulate(cfg, lam, rng=r) for _ in range(TRIALS)]
        up = statistics.mean(x["damping_s"] for x in res) / HORIZON_S
        ref = up if ref is None else ref
        print(f"{label:62s} {100 * up:6.1f}% {100 * (up - ref):+6.1f}")
        cf_rows.append([label, f"{100 * up:.2f}", f"{100 * (up - ref):+.2f}",
                        f"{(1 - up) * 1440:.0f}"])
    with open(os.path.join(OUT, "uptime_counterfactual.csv"), "w",
              newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["change", "uptime_pct", "delta_pct_vs_shipped",
                    "downtime_min_per_24h"])
        w.writerows(cf_rows)

    print()
    print("Wrote analysis/out/uptime_{bench,params,validation,sweep,trials,"
          "headline,v0_latch,counterfactual}.csv")


if __name__ == "__main__":
    main()
