"""control -- the 2.0 loop.

    sensors (8) -> ModalKalman -> q_dot (3 modes)
    f   = -w_m K_m Proj q_dot                 modal law
    u_C = A_C+ f                              on every coil whose A column is resolved
    u_j = -w_p g_j vel_j                      PID, ONLY on coils the modal law leaves out
    |u - bias| <= budget_v                    one budget, on the total, both paths

Everything measured comes in as a `Rig`; every threshold as a `Config`; the two
laws and the budget are objects handed to the Controller, so a test can swap any
of them. The Controller never touches a serial port: `step` takes counts and a
time and returns volts.

The interlocks (Health, Breaker, QuietLevel, Baseline) are stdlib's, unchanged --
they are the hardware-validated part of 1.0 and their docstrings carry the
measurements behind every rule.
"""

from dataclasses import dataclass

import numpy as np

import stdlib as sl

STATES = ("CALIBRATING", "DAMPING", "FAULT")


@dataclass(frozen=True)
class Config:
    # front end and clock
    vcc: float = 5.02
    adc_max: int = 1023
    control_hz: float = 100.0
    mains_null: bool = True
    disable: tuple = ()                # channel indices to switch off entirely
    # actuator window. bias itself is the rig's. 0.25 V of swing about 0.50 V is
    # the only window run without the supply complaining; see CLAUDE.md 11.
    bias_swing: float = 0.30
    vmin: float = 0.0
    vmax: float = 2.5
    slew_per_s: float = 2.0
    budget_frac: float = 0.90          # of bias_swing, on the TOTAL per coil
    # THE SUPPLY. Measured 2026-10-05: the coils stop following their commands
    # once the commands SUM to about 4.1 V (two staircases broke at 4.0-4.4 and
    # 4.05-4.10 V; with the total at 2.65 V the same coil stayed linear to
    # 0.85 V). In that state one coil's command moves the others' currents, so
    # nothing the model says about force is true. Stay well under it.
    sum_v_limit: float = 4.1
    sum_v_max: float = 3.6             # 88 % of it; the loop never commands more
    # gains. 0.035 is the ceiling: -0.040 is the documented rail onset.
    kp: float = 0.035
    kd: float = 0.00045
    d_hz: float = 2.0
    modal_kp: float = 0.035
    modal_scale: float = 0.5           # modal starts at half the per-channel ceiling
    gain_slew_per_s: float = 0.02
    modal: bool = True
    modal_cond_max: float = 12.0
    modal_min_sensors: int = 2
    modal_min_coils: int = 3
    # budget ramp. tilt 0 = flat (every weight exactly 1); 1 = fully by need.
    budget_tilt: float = 0.0
    budget_tau_s: float = 20.0         # >= 10 s: slower than the slowest mode
    budget_floor: float = 0.25
    budget_motion_floor_v: float = 0.0089
    # estimator
    t_amp_s: float = 50.0
    t_dc_s: float = 20.0
    # calibration (zero gain; every amplitude threshold scales off it)
    calibration_s: float = 20.0
    calib_subwindow_s: float = 2.0
    calib_subwindows: int = 5
    calib_min_subwindows: int = 3
    calib_agree_n: int = 3
    calib_agree_tol: float = 1.20
    baseline_sanity: float = 3.0
    baseline_max_refusals: int = 3
    baseline_max_age_s: float = 300.0
    baseline_max_reuse: int = 4
    fast_refault_s: float = 15.0
    # lock
    lock_factor: float = 0.35
    lock_sustain_s: float = 5.0
    lock_window_s: float = 5.0
    ratio_window_s: float = 1.0
    # runaway breaker
    envelope_window_s: float = 2.0
    runaway_multiple: float = 1.8
    runaway_sustain_s: float = 5.0
    runaway_lag_s: float = 10.0
    runaway_growth: float = 1.02
    sat_sustain_s: float = 1.30
    sat_fraction: float = 0.80
    sat_decay: float = 0.98
    ref_floor_frac: float = 0.10
    # sensor health
    rail_lo: int = 12
    rail_hi: int = 1011
    rail_sustain_s: float = 0.5
    rail_fraction: float = 0.80
    dead_pin_std: float = 1.3
    rearm_s: float = 2.0
    floor_frac: float = 0.10
    min_healthy: int = 1
    # fault recovery
    fault_clear_s: float = 5.0
    fault_clear_ratio: float = 1.4
    fault_max_hold_s: float = 30.0
    max_failed_engagements: int = 2    # faults out of DAMPING with no LOCK between
    # the running loop's own floor
    quiet_window_s: float = 30.0
    quiet_pct: float = 20.0
    quiet_keep_hz: float = 10.0
    quiet_min_fill: float = 0.80
    # housekeeping
    status_period_s: float = 5.0
    rig_max_age_days: float = 7.0

    @property
    def budget_v(self):
        return self.budget_frac * self.bias_swing


class ModalLaw:
    """q_dot -> volts about bias on the coils A resolves. Dissipative by
    construction IF A is right: the velocity is projected onto the reachable
    modes BEFORE the gain, and an over-budget vector is scaled as a whole so its
    direction survives. Nothing offline can check that A is right."""

    def __init__(self, rig, cfg):
        self.m = sl.Modal(None, rig.n, rig.f_hz, "rig", phi=rig.phi, basis="rig",
                          a=rig.a, a_coils=rig.a_coils, a_provisional=False,
                          min_sensors=cfg.modal_min_sensors,
                          min_coils=cfg.modal_min_coils, cond_max=cfg.modal_cond_max,
                          demand_cap_v=cfg.budget_v)
        if not self.m.ok:
            raise ValueError(self.m.why)
        self.min_sensors = cfg.modal_min_sensors
        self.phi, self.rowok = self.m.phi, self.m.rowok
        self.coils = np.zeros(rig.n, bool)
        self.coils[rig.a_coils] = True

    def command(self, qdot, gain, coil_ok, seen):
        """(u, coils commanded). Both empty if no mode is both seen and reachable."""
        enough = seen >= self.min_sensors
        qd, _ = self.m.project(qdot, coil_ok)
        u, _, cols, ok = self.m.allocate(np.where(enough, -gain * qd, 0.0), coil_ok)
        if not (ok and enough.any()):
            return np.zeros(len(coil_ok)), np.zeros(len(coil_ok), bool)
        return u, cols

    def report(self):
        return self.m.report()


class Budget:
    """Who gets how much of the gain: one weight per mode plus one for the PID
    block, summing to their count, ramped at `tau_s`.

    Need is the power each claimant is REMOVING (K q_dot^2), not `ratio`, which
    is referenced to a zero-gain baseline. With no usable need the shares ramp
    back to flat instead of freezing, so it cannot latch. At tilt 0 every weight
    is exactly 1.0 whatever the need. The PID block may give budget up but never
    take more than its nominal gain; its surplus goes to the modes.

    A time-varying blend of dissipative laws is NOT known to be dissipative.
    That is why tilt ships at 0 and is a bench decision, not a default.
    """

    def __init__(self, nm, tilt=0.0, tau_s=20.0, floor=0.25, motion_floor_v=0.0089):
        if not 0.0 <= tilt <= 1.0:
            raise ValueError("budget tilt %r outside 0..1" % tilt)
        self.k, self.tilt, self.tau = nm + 1, float(tilt), float(tau_s)
        self.floor, self.motion_floor = float(floor), float(motion_floor_v)
        self.share = np.full(self.k, 1.0 / self.k)
        self.w = np.ones(self.k)

    modes = property(lambda self: self.w[:-1])
    pid = property(lambda self: float(self.w[-1]))

    def update(self, dt, need, motion_v, usable):
        need = np.asarray(need, float)
        good = (usable and np.isfinite(need).all() and need.min() >= 0.0
                and need.sum() > 0.0 and motion_v > self.motion_floor)
        target = need / need.sum() if good else np.full(self.k, 1.0 / self.k)
        self.share += dt / (self.tau + dt) * (target - self.share)
        s = (1.0 - self.floor) * self.share + self.floor / self.k
        w = 1.0 + self.tilt * np.nan_to_num(self.k * s - 1.0)
        if w[-1] > 1.0:                           # PID gives, never takes
            w[:-1] += (w[-1] - 1.0) * w[:-1] / w[:-1].sum()
            w[-1] = 1.0
        self.w += np.clip(w - self.w, -dt / self.tau, dt / self.tau)


class Controller(sl.Loop):
    """CALIBRATING -> DAMPING (LOCKED is an annotation) -> FAULT."""

    def __init__(self, rig, cfg=Config(), modal="auto", budget=None):
        sl.Loop.__init__(self)
        self.rig, self.cfg, n = rig, cfg, rig.n
        self.n, self.dt0 = n, 1.0 / cfg.control_hz
        self.enabled = np.ones(n, bool)
        self.enabled[list(cfg.disable)] = False
        self.bias = rig.bias.copy()
        self.vmin = np.maximum(self.bias - cfg.bias_swing, cfg.vmin)
        self.vmax = np.minimum(self.bias + cfg.bias_swing, cfg.vmax)

        if modal == "auto":
            try:
                modal = ModalLaw(rig, cfg) if cfg.modal else None
            except ValueError as e:
                modal = None
                self.say("[modal] REFUSED -- %s. Only the PID path is left." % e)
        self.modal = modal
        self.budget = budget or Budget(rig.nm, cfg.budget_tilt, cfg.budget_tau_s,
                                       cfg.budget_floor, cfg.budget_motion_floor_v)
        # The sign comes from the measured slope: power = -slope x gain x vel^2,
        # so the gain must CARRY the slope's sign. Unresolved slope -> no gain.
        self.pid_gain = cfg.kp * rig.slope_sign * rig.pid_weight
        self.pid_gain[~self.enabled] = 0.0
        # PER-CHANNEL FEEDBACK IS ONLY SAFE IF IT DAMPS EVERY MODE, and on a
        # plate where a coil pushes other corners harder than its own that has
        # to be computed: S = A diag(g) Phi, symmetric part positive definite.
        # It failed on 2026-10-05 (eigenvalue -0.26; coil 3 moves a0 and a2 more
        # than a3). When it fails, PID is taken off every coil the modal law
        # covers, so a modal drop-out leaves those coils at bias rather than
        # pumping. Coils outside the modal law keep theirs: they barely reach
        # the modes, which is why they are outside.
        S = rig.a_dc @ np.diag(self.pid_gain) @ rig.phi
        self.pid_eig = np.linalg.eigvalsh(0.5 * (S + S.T))
        self.pid_fallback = bool((self.pid_eig > 0).all())
        if not self.pid_fallback:
            self.pid_gain[rig.a_coils] = 0.0
            self.say("[pid] per-channel feedback would PUMP a mode on this plate "
                     "(eigenvalues %s) -- it is OFF on the modal coils %s. If the "
                     "modal law drops out, those coils hold bias."
                     % (np.round(self.pid_eig, 3), rig.a_coils))
        kd = cfg.kd * np.sign(self.pid_gain) * np.abs(self.pid_gain) / cfg.kp

        # Sensors whose amplitude RATIO means something: the ones that carry the
        # modes at full strength. Only they vote on a runaway or a lock.
        rn = np.linalg.norm(rig.phi, axis=1)
        self.phi_rows = rn > 0
        self.voters = rn >= 0.5 * rn.max()

        self.guard = sl.SampleGuard(n, cfg.vcc, cfg.adc_max)
        self.dec = sl.Decimator(cfg.control_hz, n, cfg.mains_null)
        self.health = sl.Health(n, cfg.rail_lo, cfg.rail_hi, cfg.rail_sustain_s,
                                cfg.rail_fraction, cfg.sat_sustain_s,
                                cfg.sat_fraction, cfg.dead_pin_std, cfg.rearm_s,
                                cfg.floor_frac)
        self.brk = sl.Breaker(n, cfg.envelope_window_s, cfg.runaway_multiple,
                              cfg.runaway_sustain_s, cfg.runaway_lag_s,
                              cfg.runaway_growth, cfg.sat_decay, cfg.min_healthy)
        self.pid = sl.PID(n, np.zeros(n), kd, cfg.d_hz, 0.0, 1.0, cfg.slew_per_s)
        self.pid.bias(self.bias)
        self.base = sl.Baseline(n, cfg.calib_subwindow_s, cfg.calib_subwindows,
                                cfg.calib_min_subwindows, cfg.calib_agree_n,
                                cfg.calib_agree_tol, cfg.baseline_sanity,
                                cfg.baseline_max_refusals, cfg.floor_frac)
        self.inband = sl.InBand(rig.f_hz, n)
        self.rms_ratio = sl.RMSBank(cfg.ratio_window_s, n)
        self.rms_lock = sl.RMSBank(cfg.lock_window_s, n)
        self.quiet = sl.QuietLevel(n, cfg.quiet_window_s, cfg.quiet_pct,
                                   cfg.quiet_keep_hz, cfg.quiet_min_fill)
        self.kf = sl.KalmanVelocity(rig.kalman_k, n, rig.f_hz, self.dt0)
        self.mkf = (sl.ModalKalman(modal.phi, modal.rowok, rig.kalman_r,
                                   rig.kalman_q, rig.kalman_r / cfg.t_dc_s,
                                   rig.f_hz, self.dt0, n, cfg.t_amp_s)
                    if modal is not None else None)
        self.chi2_log = {s: sl.Chi2Log((1.0, 2.0, 3.0, 5.0, 10.0)) for s in STATES}

        for k in "bp vel gain ratio baseline out".split():
            setattr(self, k, np.zeros(n))
        self.out = self.bias.copy()
        self.qdot, self.mode_gain = np.zeros(rig.nm), np.zeros(rig.nm)
        self.chi2 = float("nan")
        self.modal_cols = np.zeros(n, bool)       # coils the modal law drove
        self.pid_on = np.zeros(n, bool)           # coils the PID path drove
        self.locked = np.zeros(n, bool)
        self.locked_since = np.full(n, np.inf)
        self.sense_ok = self.enabled.copy()
        self.calib_start, self.damping_start, self.baseline_t = 0.0, None, None
        self.damped_for, self.fault_t, self.fault_railed = None, None, False
        self.fault_amp, self.reuse_count, self.bad_samples = False, 0, 0
        self.stepped, self.n_avg, self.peak_demand = False, 0, 0.0
        self.failed = 0
        self.sum_limited, self.peak_sum = 0, 0.0
        self._said = set()
        self.base.start(0.0)

    # -- small helpers ------------------------------------------------------
    @property
    def latched(self):
        return self.failed >= self.cfg.max_failed_engagements

    @staticmethod
    def _who(m):
        return ", ".join("ch%d" % i for i in np.nonzero(m)[0])

    def _once(self, key, msg):
        if key not in self._said:
            self._said.add(key)
            self.say(msg)

    def _driven(self):
        """Every channel something commands. The breaker's evidence set."""
        d = self.pid_gain != 0.0
        return d | self.modal.coils if self.modal is not None else d

    def _vote(self):
        v = self._driven() & self.voters
        return v if v.any() else self._driven()

    def _amp_ref(self):
        """What the amplitude interlocks measure against: the larger of the
        zero-gain baseline and the running loop's own floor, floored across
        channels so a reference that is an artefact cannot vote on a ratio."""
        q = self.quiet.level()
        ref = self.baseline if q is None else np.maximum(self.baseline, q)
        d = self._driven()
        if d.any() and np.median(ref[d]) > 0:
            ref = np.maximum(ref, self.cfg.ref_floor_frac * np.median(ref[d]))
        return ref

    # -- one wire sample ----------------------------------------------------
    def step(self, counts, t):
        """Volts for all coils when a control step ran, else None."""
        counts = np.asarray(counts, float)
        self.stepped = False
        if not self.guard.in_range(counts):
            self.bad_samples += 1
            return None
        self.health.rails(counts, t)                   # wire rate
        got = self.dec.feed(counts, counts * self.guard.scale, t)
        if got is None:
            return None
        _, volts, dt, self.n_avg = got
        self.stepped = True
        h = self.health
        self.vel = self.kf.update(volts, dt)
        self.bp = self.kf.displacement()
        self.sense_ok = self.enabled & h.healthy & ~h.floor_bad
        if self.mkf is not None:
            self.qdot = self.mkf.update(
                volts, self.sense_ok & ~h.rail & self.phi_rows, dt)
            self.chi2 = self.mkf.chi2_per_dof()
            self.chi2_log[self.state].add(self.chi2)
        if self.state == "CALIBRATING":
            self._calibrating(t, volts)
        elif self.state == "DAMPING":
            self._damping(t, dt)
        else:
            self._faulted(t)
        return self._actuate(t, dt)

    def _fault(self, t, msg, railed=False, amp=False):
        self.say("!! " + msg)
        self.damped_for = (t - self.damping_start
                           if self.state == "DAMPING" else None)
        self.fault_railed, self.fault_amp, self.fault_t = railed, amp, t
        self.failed += self.state == "DAMPING"
        self.state, self.clear_since = "FAULT", None
        self.fault_count += 1

    # -- CALIBRATING: zero gain, measure the floor --------------------------
    def _calibrating(self, t, volts):
        cfg, h = self.cfg, self.health
        self.base.add(self.bp)
        self.inband.add(t, volts)
        dead, witness = h.dead_pin(self.enabled)
        railed = h.rail & self.enabled & ~dead
        if railed.any():
            return self._fault(t, "%s railed during calibration -- check "
                               "alignment." % self._who(railed), railed=True)
        if t - self.base.sub_start >= cfg.calib_subwindow_s:
            self.base.close_subwindow(t)
        ceiling = t - self.calib_start >= cfg.calibration_s
        early = not ceiling and self.base.stationary()
        if not (early or ceiling):
            return
        if ceiling and len(self.base.acc) > self.base.sub_n0:
            self.base.close_subwindow(t)
        took = t - self.calib_start
        self.baseline, note = self.base.settle(early)
        if note:
            self.say(note)
        self.baseline_t, self.reuse_count = t, 0
        self._engage(t)
        self.say("[DAMPING] baseline %s in %.1fs: %s"
                 % ("KEPT (fresh one refused)" if self.base.refused else "set",
                    took, ", ".join("ch%d=%.4fV" % (i, b)
                                    for i, b in enumerate(self.baseline))))
        self._floor()

    def _engage(self, t):
        self.state, self.damping_start = "DAMPING", t
        self.brk.arm(t)
        self.quiet.reset()
        self.locked_announced = False
        self._said.discard("quiet")

    def _floor(self):
        """Demote sensors that are not measuring the optic. A sensor with no
        full-strength Phi row is judged on whether it MOVES, not on an in-band
        amplitude its axis cannot carry (the error made four times in 1.0)."""
        h = self.health
        x = self.inband.rms()
        x = x if (x > 0).any() else self.baseline
        bad, med, witness = h.floor(x, self.enabled, exempt=~self.voters,
                                    motion=h.rail_std)
        h.floor_bad = bad
        if not bad.any():
            return
        h.healthy[bad], h.clear_t[bad], self.gain[bad] = False, np.inf, 0.0
        self.say("!! demoted as SENSORS: %s (in-band %s V against a %.4f V median; "
                 "raw std %s counts). Their COILS are unaffected."
                 % (self._who(bad),
                    "/".join("%.4f" % v for v in x[bad]), med,
                    "/".join("%.2f" % v for v in h.rail_std[bad])))

    # -- DAMPING ------------------------------------------------------------
    def _damping(self, t, dt):
        cfg, h = self.cfg, self.health
        drop, _, back = h.rearm(t, kf_reset=self.kf.reset)
        self.gain[drop] = 0.0
        if back.any():
            self.brk.reset(back)
            self.rms_ratio.reset(back)
            self.brk.excess_since[back], self.ratio[back] = np.inf, 0.0
        live = self.enabled & h.healthy
        for i in np.nonzero(drop)[0]:
            self.say("!! ch%d railed -- sensor out, its PID held at bias." % i)
        for i in np.nonzero(back)[0]:
            self.say("[ch%d back] rail clear %.0fs." % (i, cfg.rearm_s))
        if int(live.sum()) < cfg.min_healthy:
            return self._fault(t, "quorum lost -- %d healthy, need %d."
                               % (live.sum(), cfg.min_healthy), railed=True)

        self.ratio = np.where(live, self.rms_ratio.update(t, self.bp, live)
                              / np.maximum(self.baseline, 1e-12), self.ratio)

        # Gains: slewed from zero, scaled by the budget's weights.
        cap = cfg.gain_slew_per_s * dt
        want = self.pid_gain * self.budget.pid
        self.gain = np.where(self.sense_ok,
                             self.gain + np.clip(want - self.gain, -cap, cap), 0.0)
        want_m = (cfg.modal_kp * cfg.modal_scale * self.budget.modes
                  if self.modal is not None else np.zeros(self.rig.nm))
        self.mode_gain = self.mode_gain + np.clip(want_m - self.mode_gain, -cap, cap)

        vote = self._vote()
        got = self.brk.sample(t, self.bp, h.healthy, vote, self._amp_ref(), h.sat)
        self.quiet.add(t, got["env"], live)             # closed-loop time only
        if self.quiet.ready():
            q = self.quiet.level()
            m = live & (self.baseline > 0)
            self._once("quiet", "[quiet] the running loop's own floor: "
                       + ", ".join("ch%d %.2fx baseline" % (i, q[i] / self.baseline[i])
                                   for i in np.nonzero(m)[0])
                       + ". The breaker and the fault-clear gate now scale off "
                         "max(baseline, this).")
        if got["run_trip"].any():
            return self._fault(t, "%s runaway -- all coils to bias."
                               % self._who(got["run_trip"]), amp=True)
        if got["sat_trip"].any():
            return self._fault(t, "%s pinned against the window, still demanding "
                               "more, and not winning." % self._who(got["sat_trip"]))

        # The budget ramp, keyed on dissipation rate per claimant.
        self.budget.update(
            dt, np.append(self.mode_gain * self.qdot ** 2,
                          float((np.abs(self.gain) * self.vel ** 2)[self.pid_on].sum())),
            float(got["env"][vote].max()) if vote.any() else 0.0,
            usable=bool(self.modal_cols.any()))

        # LOCKED is judged against the ZERO-GAIN baseline on purpose: it is a
        # statement about the loop's effect on the undamped plate.
        quiet = live & (self.rms_lock.update(t, self.bp, live)
                        < self.baseline * cfg.lock_factor)
        self.locked_since = sl.Health.hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= cfg.lock_sustain_s)
        claim = live & vote
        if claim.any() and self.locked[claim].all() and not self.locked_announced:
            self.locked_announced = True
            self.failed = 0
            self.lock_time = t - self.damping_start
            self.say("*** LOCKED -- %.1fs after gain was applied (t=%.1fs), %d "
                     "voting channels quiet, worst ratio %.3f against the %.2f "
                     "line; modal on %d coils, PID on %d ***"
                     % (self.lock_time, t, claim.sum(), self.ratio[claim].max(),
                        cfg.lock_factor, self.modal_cols.sum(), self.pid_on.sum()))

    # -- FAULT: coils frozen at bias, measurement LIVE ----------------------
    def _faulted(self, t):
        cfg, h = self.cfg, self.health
        self.gain[:] = 0.0
        self.mode_gain[:] = 0.0
        m = self.enabled & ~h.floor_bad & (self.baseline > 0)
        if m.any():
            self.ratio = np.where(m, self.rms_ratio.update(t, self.bp, m)
                                  / np.maximum(self.baseline, 1e-12), self.ratio)
            self.brk.sample(t, self.bp, m, np.zeros(self.n, bool), self._amp_ref(),
                            np.zeros(self.n, bool))
        if self.latched:
            # A loop that faults every time it engages and never locks is most
            # likely PUMPING, and each re-engagement adds energy to a plate that
            # takes 138 s to lose it. A wrong coil polarity looks exactly like
            # this and nothing offline catches one, so this is the last line:
            # stop, and say what to check. 1.0 re-engaged forever.
            return self._once("latch", "!! LATCHED OFF -- %d engagements in a row "
                              "ended in a fault without ever locking. Coils stay at "
                              "bias until restarted. Re-measure the slopes and A "
                              "(`run.py identify`) before trying again."
                              % self.failed)
        dead, _ = h.dead_pin(self.enabled)
        watch = self.enabled & ~h.floor_bad
        stuck = h.rail & watch & ~dead
        blocked = bool(stuck.any() or (h.sat & watch).any() or self._still_hot(t, m))
        if self.clear_gate(t, blocked, cfg.fault_clear_s):
            self._recover(t)

    def _still_hot(self, t, m):
        """An amplitude fault re-engages on amplitude, with a ceiling on the wait."""
        cfg = self.cfg
        if not self.fault_amp:
            return False
        if t - self.fault_t >= cfg.fault_max_hold_s:
            self._once(("hold", self.fault_count),
                       "!! held out for %.0fs by amplitude alone -- dropping the "
                       "amplitude gate and re-engaging." % cfg.fault_max_hold_s)
            return False
        m = m & self._vote()
        line = cfg.fault_clear_ratio * np.maximum(
            1.0, self._amp_ref() / np.maximum(self.baseline, 1e-12))
        return bool(m.any() and (self.ratio[m] > line[m]).any())

    def _recover(self, t):
        cfg, h = self.cfg, self.health
        if self.damped_for is not None and self.damped_for >= cfg.fast_refault_s:
            self.reuse_count = 0           # the loop itself validated this baseline
        age = None if self.baseline_t is None else t - self.baseline_t
        reuse = (bool(self.baseline.all()) and not self.fault_railed
                 and age is not None and age <= cfg.baseline_max_age_s
                 and self.reuse_count < cfg.baseline_max_reuse)
        keep_floor = h.floor_bad.copy()
        self.base.start(t)
        self.inband.reset()
        self.gain[:], self.ratio[:] = 0.0, 0.0
        h.sat_hist.clear()
        h.sat_sum[:], h.sat[:], h.healthy[:], h.floor_bad[:] = 0.0, False, True, False
        h.clear_t[:] = self.brk.excess_since[:] = self.locked_since[:] = np.inf
        self.locked[:] = False
        for part in (self.pid, self.kf, self.brk, self.rms_ratio, self.rms_lock):
            part.reset()
        if self.mkf is not None:
            self.mkf.reset()
        if reuse:
            self.reuse_count += 1
            h.floor_bad, h.healthy[keep_floor] = keep_floor, False
            self._engage(t)
            self.say("[recovered] re-engaging on the baseline in hand (%d/%d before "
                     "a forced re-calibration). Gain ramps from zero."
                     % (self.reuse_count, cfg.baseline_max_reuse))
        else:
            self.reuse_count, self.baseline_t, self.calib_start = 0, None, t
            self.baseline[:] = 0.0
            self.state = "CALIBRATING"
            self.say("[recovered] clear for %.0fs -- re-calibrating."
                     % cfg.fault_clear_s)
        self.fault_railed = self.fault_amp = False
        self.fault_t = self.clear_since = self.lock_time = None

    # -- output -------------------------------------------------------------
    def _actuate(self, t, dt):
        cfg, h = self.cfg, self.health
        on = self.state == "DAMPING"
        # A COIL is usable unless its own sensor is railed. A sensor demoted for
        # being quiet says nothing about its coil -- that distinction is what
        # lets the modal law keep coils whose sensors see nothing.
        coil_ok = self.enabled & (h.healthy | h.floor_bad) & on
        u, cols = np.zeros(self.n), np.zeros(self.n, bool)
        if self.modal is not None and on:
            u, cols = self.modal.command(self.qdot, self.mode_gain, coil_ok,
                                         self.mkf.seen)
            if not cols.any() and self.mkf.seen.any():
                self._once("nomodal", "[modal] no reachable mode with these coils "
                           "and sensors -- each coil falls to PID if it has a PID "
                           "gain, and otherwise holds bias.")
        p, _, d = self.pid.terms(self.vel, self.gain, dt)
        self.pid_on = coil_ok & self.sense_ok & ~cols & (self.pid_gain != 0.0)
        # PID is colocated, so it stays dissipative under its own clip.
        cmd = np.where(cols, u, np.where(
            self.pid_on, np.clip(p + d, -cfg.budget_v, cfg.budget_v), 0.0))
        # The total. Scaling EVERY coil's demand by one factor keeps the force
        # direction, so the law stays dissipative; letting the supply clip it
        # would not.
        over = float((self.bias + cmd)[self.enabled].sum()) - cfg.sum_v_max
        if over > 0 and cmd[self.enabled].sum() > 0:
            cmd = cmd * max(1.0 - over / float(cmd[self.enabled].sum()), 0.0)
            self.sum_limited += 1
        self.modal_cols = cols
        self.peak_demand = max(self.peak_demand, float(np.abs(cmd).max()))
        self.peak_sum = max(self.peak_sum, float((self.bias + cmd).sum()))
        self.out = self.pid.drive(self.bias + cmd, self.bias, self.vmin, self.vmax,
                                  coil_ok, dt)
        h.sats((self.out <= self.vmin + 1e-6) | (self.out >= self.vmax - 1e-6), t)
        return self.out

    # -- reporting ----------------------------------------------------------
    def banner(self):
        r, cfg = self.rig, self.cfg
        pid = np.nonzero(self.pid_gain)[0]
        out = ["[rig] measured %s (%.1f days old); modes %s Hz = %s"
               % (r.measured, r.age_days(), np.round(r.f_hz, 5), "/".join(r.dof)),
               "[law] modal on coils %s; PID available on %s (used only where "
               "the modal law is not)"
               % (self.rig.a_coils if self.modal else "NONE",
                  ", ".join("ch%d %+.4f" % (i, self.pid_gain[i]) for i in pid)),
               "[budget] %.3f V per coil about bias, tilt %.2f%s"
               % (cfg.budget_v, self.budget.tilt,
                  "" if self.budget.tilt else " (flat: every weight exactly 1)")]
        return out + ["[rig] NOTE: " + s for s in r.notes]

    def status_line(self, t):
        h = self.health

        def one(i):
            tag = ("off" if not self.enabled[i] else
                   "NOSIG" if h.floor_bad[i] else
                   "DOWN" if not h.healthy[i] else
                   ("LOCK" if self.locked[i] else "....")
                   + ("!RAIL" if h.rail[i] else ""))
            law = "M" if self.modal_cols[i] else "P" if self.pid_on[i] else "-"
            return "ch%d:%s%s r=%.2f" % (i, law, tag, self.ratio[i])

        hz = self.dec.wire_hz
        mode = ("q=" + "/".join("%+.3f" % x for x in self.qdot)
                + " chi2/dof=%.2f" % self.chi2
                + " w=" + "/".join("%.2f" % x for x in self.budget.w)
                if self.mkf is not None else "PID only")
        return sl.Console.line(
            t, self.state, "%s/%.0fHz" % ("--" if hz != hz else "%.0f" % hz,
                                          self.cfg.control_hz),
            mode, [one(i) for i in range(self.n)])

    def csv_cols(self):
        nm = self.rig.nm
        cols = [("t", ".5f"), ("state", None), ("ctl", None), ("n_avg", None),
                ("chi2", ".4f")]
        cols += [("qdot%d" % m, ".6f") for m in range(nm)]
        cols += [("kmode%d" % m, ".5f") for m in range(nm)]
        cols += [("w%d" % k, ".4f") for k in range(nm + 1)]
        for i in range(self.n):
            cols += [("a%d" % i, None), ("bp%d" % i, ".6f"), ("vel%d" % i, ".6f"),
                     ("out%d" % i, ".5f"), ("gain%d" % i, ".5f"),
                     ("ratio%d" % i, ".4f"), ("law%d" % i, None)]
        return cols

    def csv_row(self, t, counts):
        row = [t, self.state, "1" if self.stepped else "0", str(self.n_avg),
               self.chi2, *self.qdot, *self.mode_gain, *self.budget.w]
        for i in range(self.n):
            row += ["%.2f" % counts[i], self.bp[i], self.vel[i], self.out[i],
                    self.gain[i], self.ratio[i],
                    "M" if self.modal_cols[i] else "P" if self.pid_on[i] else "-"]
        return row


# ---------------------------------------------------------------------------
# preflight: everything checkable before a coil is energised. No port.
# ---------------------------------------------------------------------------
def preflight(rig, cfg, now=None):
    """[(name, ok, detail)]. Algebra and bookkeeping only -- it cannot tell you
    the rig still matches rig.json, which is why the first check is its age."""
    out = []

    def check(name, ok, detail=""):
        out.append((name, bool(ok), detail))

    age = rig.age_days(now)
    check("rig.json is fresh", age <= cfg.rig_max_age_days,
          "%.1f days old, limit %.0f -- frequencies drift and a re-seating "
          "flips slopes; re-run `run.py identify`" % (age, cfg.rig_max_age_days))
    ctl = Controller(rig, cfg)
    g = ctl.pid_gain
    check("slope x gain > 0 on every PID channel",
          (rig.slope * g)[g != 0].min(initial=1.0) > 0
          and not (g[rig.slope_sign == 0] != 0).any(),
          "gain %s" % np.round(g, 4))
    check("A is today's DC matrix on today's Phi",
          np.allclose(np.linalg.pinv(rig.phi[:4]) @ rig.dc[:4], rig.a_dc, atol=0.05))
    check("PID fallback is safe, or switched off",
          ctl.pid_fallback or ctl.modal is not None,
          "eigenvalues %s -- %s" % (np.round(ctl.pid_eig, 3),
                                    "damps every mode" if ctl.pid_fallback else
                                    "would pump, so it is OFF on the modal coils; "
                                    "there is NO fallback law on this plate"))
    if ctl.modal is None:
        check("modal law builds", not cfg.modal, "see the [modal] REFUSED line")
    else:
        rng, worst = np.random.default_rng(0), -np.inf
        ok = ctl.modal.coils.copy()
        for _ in range(2000):
            qd = rng.normal(size=rig.nm)
            u, _ = ctl.modal.command(qd, np.full(rig.nm, cfg.modal_kp), ok,
                                     np.full(rig.nm, rig.n))
            worst = max(worst, float(qd @ (ctl.modal.m.a @ u)))
        check("modal law dissipates in its OWN model, 2000 random velocities",
              worst <= 1e-12, "worst power %+.2e (says nothing about the real A)"
              % worst)
    check("window inside the DAC range",
          (rig.bias - cfg.bias_swing >= cfg.vmin - 1e-9).all()
          and (rig.bias + cfg.bias_swing <= cfg.vmax + 1e-9).all())
    total = float(rig.bias.sum())
    check("bias leaves the supply room to work",
          total + rig.n * cfg.budget_v / 2 <= cfg.sum_v_max,
          "bias sums to %.2f V; the loop is capped at %.2f V and the supply folds "
          "back near %.2f V (2026-10-05)" % (total, cfg.sum_v_max, cfg.sum_v_limit))
    return out
