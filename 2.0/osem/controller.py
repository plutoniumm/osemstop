import numpy as np
from .config import Config
from .filters import Decimator, InBand, RMSBank
from .kalman import Chi2Log, Estimator
from .regulators import Modal
from .term import STATE, c
from .safety import Baseline, Breaker, Health, QuietLevel

STATES = ("CALIBRATING", "DAMPING", "FAULT")


class Controller:

    def __init__(self, rig, cfg=Config(), regulators=None):
        self.rig, self.cfg, self.n = (rig, cfg, rig.n)
        n = rig.n
        self.regulators = [Modal(rig, cfg)] if regulators is None else list(regulators)
        self.est = Estimator(rig, cfg, modes=any((r.needs_modes for r in self.regulators)))
        self.enabled = np.ones(n, bool)
        self.enabled[list(cfg.disable)] = False
        self.bias = rig.bias.copy()
        self.vmin = np.maximum(self.bias - cfg.bias_swing, cfg.vmin)
        self.vmax = np.minimum(self.bias + cfg.bias_swing, cfg.vmax)
        self.out = self.bias.copy()
        strength = np.linalg.norm(rig.phi, axis=1)
        self.phi_rows = strength > 0
        self.voters = strength >= 0.5 * strength.max()
        self.driven = (
            np.any([r.coils for r in self.regulators], axis=0)
            if self.regulators
            else np.zeros(n, bool)
        )
        self.dec = Decimator(cfg.control_hz, n, cfg.mains_null)
        self.health = Health(
            n,
            cfg.rail_lo,
            cfg.rail_hi,
            cfg.rail_sustain_s,
            cfg.rail_fraction,
            cfg.sat_sustain_s,
            cfg.sat_fraction,
            cfg.dead_pin_std,
            cfg.rearm_s,
            cfg.floor_frac,
        )
        self.brk = Breaker(
            n,
            cfg.envelope_window_s,
            cfg.runaway_multiple,
            cfg.runaway_sustain_s,
            cfg.runaway_lag_s,
            cfg.runaway_growth,
            cfg.sat_decay,
            cfg.min_healthy,
        )
        self.base = Baseline(
            n,
            cfg.calib_subwindow_s,
            cfg.calib_subwindows,
            cfg.calib_min_subwindows,
            cfg.calib_agree_n,
            cfg.calib_agree_tol,
            cfg.baseline_sanity,
            cfg.baseline_max_refusals,
            cfg.floor_frac,
        )
        self.inband = InBand(rig.f_hz, n)
        self.rms_ratio = RMSBank(cfg.ratio_window_s, n)
        self.rms_lock = RMSBank(cfg.lock_window_s, n)
        self.quiet = QuietLevel(
            n, cfg.quiet_window_s, cfg.quiet_pct, cfg.quiet_keep_hz, cfg.quiet_min_fill
        )
        self.chi2_log = {s: Chi2Log((1.0, 2.0, 3.0, 5.0, 10.0)) for s in STATES}
        self.state, self.events = ("CALIBRATING", [])
        self.ratio, self.baseline = (np.zeros(n), np.zeros(n))
        self.locked = np.zeros(n, bool)
        self.locked_since = np.full(n, np.inf)
        self.locked_announced, self.lock_time = (False, None)
        self.sense_ok = self.enabled.copy()
        self.calib_start, self.damping_start, self.baseline_t = (0.0, None, None)
        self.damped_for, self.fault_t, self.clear_since = (None, None, None)
        self.fault_railed = self.fault_amp = False
        self.fault_count = self.failed = self.reuse_count = self.bad_samples = 0
        self.stepped, self.n_avg = (False, 0)
        self.peak_demand, self.peak_sum, self.sum_limited = (0.0, 0.0, 0)
        self._said = set()
        self.base.start(0.0)

    def find(self, kind):
        return next((r for r in self.regulators if isinstance(r, kind)), None)

    @property
    def latched(self):
        return self.failed >= self.cfg.max_failed_engagements

    def say(self, msg):
        self.events.append(msg)

    def drain_events(self):
        out, self.events = (self.events, [])
        return out

    def _once(self, key, msg):
        if key not in self._said:
            self._said.add(key)
            self.say(msg)

    @staticmethod
    def _who(mask):
        return ", ".join(("a%d" % i for i in np.nonzero(mask)[0]))

    def _vote(self):
        v = self.driven & self.voters
        return v if v.any() else self.driven

    def _amp_ref(self):
        q = self.quiet.level()
        ref = self.baseline if q is None else np.maximum(self.baseline, q)
        d = self.driven
        if d.any() and np.median(ref[d]) > 0:
            ref = np.maximum(ref, self.cfg.ref_floor_frac * np.median(ref[d]))
        return ref

    def step(self, counts, t):
        counts = np.asarray(counts, float)
        self.stepped = False
        if not np.all((counts >= 0) & (counts <= self.cfg.adc_max)):
            self.bad_samples += 1
            return None
        self.health.rails(counts, t)
        got = self.dec.feed(counts, counts * self.cfg.volts_per_count, t)
        if got is None:
            return None
        _, volts, dt, self.n_avg = got
        self.stepped = True
        h = self.health
        self.sense_ok = self.enabled & h.healthy & ~h.floor_bad
        self.est.update(volts, dt, self.sense_ok & ~h.rail & self.phi_rows)
        if self.est.mkf is not None:
            self.chi2_log[self.state].add(self.est.chi2)
        if self.state == "CALIBRATING":
            self._calibrating(t, volts)
        elif self.state == "DAMPING":
            self._damping(t, dt)
        else:
            self._faulted(t)
        return self._actuate(t, dt)

    def _fault(self, t, msg, railed=False, amp=False):
        self.say("!! " + msg)
        self.damped_for = t - self.damping_start if self.state == "DAMPING" else None
        self.fault_railed, self.fault_amp, self.fault_t = (railed, amp, t)
        self.failed += self.state == "DAMPING"
        self.state, self.clear_since = ("FAULT", None)
        self.fault_count += 1

    def _calibrating(self, t, volts):
        cfg, h = (self.cfg, self.health)
        self.base.add(self.est.bp)
        self.inband.add(t, volts)
        dead, _ = h.dead_pin(self.enabled)
        railed = h.rail & self.enabled & ~dead
        if railed.any():
            return self._fault(
                t,
                "%s railed during calibration -- check alignment." % self._who(railed),
                railed=True,
            )
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
        self.baseline_t, self.reuse_count = (t, 0)
        self._engage(t)
        self.say(
            "[gain on] baseline %s in %.1f s, counts rms: %s"
            % (
                "KEPT (fresh one refused)" if self.base.refused else "set",
                took,
                " ".join(("%.1f" % (b / cfg.volts_per_count) for b in self.baseline)),
            )
        )
        self._demote_quiet_sensors()

    def _engage(self, t):
        self.state, self.damping_start = ("DAMPING", t)
        self.brk.arm(t)
        self.quiet.reset()
        self.locked_announced = False
        self._said.discard("quiet")

    def _demote_quiet_sensors(self):
        h = self.health
        x = self.inband.rms()
        x = x if (x > 0).any() else self.baseline
        bad, med, _ = h.floor(x, self.enabled, exempt=~self.voters, motion=h.rail_std)
        h.floor_bad = bad
        if not bad.any():
            return
        h.healthy[bad], h.clear_t[bad] = (False, np.inf)
        for r in self.regulators:
            r.zero(bad)
        self.say(
            "[sensors] too quiet to vote: %s (%s counts std). Their coils still drive."
            % (self._who(bad), "/".join(("%.1f" % v for v in h.rail_std[bad])))
        )

    def _damping(self, t, dt):
        cfg, h, bp = (self.cfg, self.health, self.est.bp)
        drop, _, back = h.rearm(t, kf_reset=self.est.kf.reset)
        for r in self.regulators:
            r.zero(drop)
        if back.any():
            self.brk.reset(back)
            self.rms_ratio.reset(back)
            self.brk.excess_since[back], self.ratio[back] = (np.inf, 0.0)
        live = self.enabled & h.healthy
        for i in np.nonzero(drop)[0]:
            self.say("!! ch%d railed -- sensor out, its PID held at bias." % i)
        for i in np.nonzero(back)[0]:
            self.say("[ch%d back] rail clear %.0fs." % (i, cfg.rearm_s))
        if int(live.sum()) < cfg.min_healthy:
            return self._fault(
                t,
                "quorum lost -- %d healthy, need %d." % (live.sum(), cfg.min_healthy),
                railed=True,
            )
        self.ratio = np.where(
            live, self.rms_ratio.update(t, bp, live) / np.maximum(self.baseline, 1e-12), self.ratio
        )
        for r in self.regulators:
            r.ramp(dt, self.sense_ok)
        vote = self._vote()
        got = self.brk.sample(t, bp, h.healthy, vote, self._amp_ref(), h.sat)
        self.quiet.add(t, got["env"], live)
        if self.quiet.ready():
            q = self.quiet.level()
            m = live & (self.baseline > 0)
            self._once(
                "quiet",
                "[floor] loop's own floor, x baseline: "
                + " ".join(("a%d %.2f" % (i, q[i] / self.baseline[i]) for i in np.nonzero(m)[0])),
            )
        if got["run_trip"].any():
            return self._fault(
                t, "%s runaway -- all coils to bias." % self._who(got["run_trip"]), amp=True
            )
        if got["sat_trip"].any():
            return self._fault(
                t,
                "%s pinned against the window, still demanding more, and not winning."
                % self._who(got["sat_trip"]),
            )
        quiet = live & (self.rms_lock.update(t, bp, live) < self.baseline * cfg.lock_factor)
        self.locked_since = Health.hold(quiet, self.locked_since, t)
        self.locked = quiet & (t - self.locked_since >= cfg.lock_sustain_s)
        claim = live & vote
        if claim.any() and self.locked[claim].all() and (not self.locked_announced):
            self.locked_announced = True
            self.failed = 0
            self.lock_time = t - self.damping_start
            self.say(
                "LOCKED %.1f s after gain (t=%.1f s): %d channels quiet, worst ratio %.2f of %.2f, %s"
                % (
                    self.lock_time,
                    t,
                    claim.sum(),
                    self.ratio[claim].max(),
                    cfg.lock_factor,
                    ", ".join(
                        (
                            "%s on %d coils" % (type(r).__name__, r.claimed.sum())
                            for r in self.regulators
                        )
                    ),
                )
            )

    def _faulted(self, t):
        cfg, h, bp = (self.cfg, self.health, self.est.bp)
        for r in self.regulators:
            r.zero()
        m = self.enabled & ~h.floor_bad & (self.baseline > 0)
        if m.any():
            self.ratio = np.where(
                m, self.rms_ratio.update(t, bp, m) / np.maximum(self.baseline, 1e-12), self.ratio
            )
            self.brk.sample(
                t, bp, m, np.zeros(self.n, bool), self._amp_ref(), np.zeros(self.n, bool)
            )
        if self.latched:
            return self._once(
                "latch",
                "!! LATCHED OFF -- %d engagements in a row ended in a fault without ever locking. Coils stay at bias until restarted. Re-measure (`run.py measure`) before trying again."
                % self.failed,
            )
        dead, _ = h.dead_pin(self.enabled)
        watch = self.enabled & ~h.floor_bad
        stuck = h.rail & watch & ~dead
        blocked = bool(stuck.any() or (h.sat & watch).any() or self._still_hot(t, m))
        if blocked:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = t
        elif t - self.clear_since >= cfg.fault_clear_s:
            self._recover(t)

    def _still_hot(self, t, m):
        cfg = self.cfg
        if not self.fault_amp:
            return False
        if t - self.fault_t >= cfg.fault_max_hold_s:
            self._once(
                ("hold", self.fault_count),
                "!! held out for %.0fs by amplitude alone -- dropping the amplitude gate and re-engaging."
                % cfg.fault_max_hold_s,
            )
            return False
        m = m & self._vote()
        line = cfg.fault_clear_ratio * np.maximum(
            1.0, self._amp_ref() / np.maximum(self.baseline, 1e-12)
        )
        return bool(m.any() and (self.ratio[m] > line[m]).any())

    def _recover(self, t):
        cfg, h = (self.cfg, self.health)
        if self.damped_for is not None and self.damped_for >= cfg.fast_refault_s:
            self.reuse_count = 0
        age = None if self.baseline_t is None else t - self.baseline_t
        reuse = (
            bool(self.baseline.all())
            and (not self.fault_railed)
            and (age is not None)
            and (age <= cfg.baseline_max_age_s)
            and (self.reuse_count < cfg.baseline_max_reuse)
        )
        keep_floor = h.floor_bad.copy()
        self.base.start(t)
        self.inband.reset()
        self.ratio[:] = 0.0
        h.sat_hist.clear()
        h.sat_sum[:], h.sat[:], h.healthy[:], h.floor_bad[:] = (0.0, False, True, False)
        h.clear_t[:] = self.brk.excess_since[:] = self.locked_since[:] = np.inf
        self.locked[:] = False
        for r in self.regulators:
            r.zero()
            r.reset()
        for part in (self.est, self.brk, self.rms_ratio, self.rms_lock):
            part.reset()
        if reuse:
            self.reuse_count += 1
            h.floor_bad, h.healthy[keep_floor] = (keep_floor, False)
            self._engage(t)
            self.say(
                "[recovered] re-engaging on the baseline in hand (%d/%d before a forced re-calibration). Gain ramps from zero."
                % (self.reuse_count, cfg.baseline_max_reuse)
            )
        else:
            self.reuse_count, self.baseline_t, self.calib_start = (0, None, t)
            self.baseline[:] = 0.0
            self.state = "CALIBRATING"
            self.say("[recovered] clear for %.0fs -- re-calibrating." % cfg.fault_clear_s)
        self.fault_railed = self.fault_amp = False
        self.fault_t = self.clear_since = self.lock_time = None

    def _actuate(self, t, dt):
        cfg, h = (self.cfg, self.health)
        on = self.state == "DAMPING"
        # a quiet sensor says nothing about its coil; a railed one does
        coil_ok = self.enabled & (h.healthy | h.floor_bad) & on
        cmd, taken = (np.zeros(self.n), np.zeros(self.n, bool))
        for r in self.regulators:
            u = r.command(self.est, coil_ok & ~taken, self.sense_ok, dt, on)
            cmd = np.where(r.claimed, u, cmd)
            taken |= r.claimed
            if r.needs_modes and on and (not r.claimed.any()) and self.est.seen.any():
                self._once("nomodal", "[modal] no reachable mode with these coils and sensors.")
        # one factor on every coil keeps the force direction
        over = float((self.bias + cmd)[self.enabled].sum()) - cfg.sum_v_max
        if over > 0 and cmd[self.enabled].sum() > 0:
            cmd = cmd * max(1.0 - over / float(cmd[self.enabled].sum()), 0.0)
            self.sum_limited += 1
        self.peak_demand = max(self.peak_demand, float(np.abs(cmd).max()))
        self.peak_sum = max(self.peak_sum, float((self.bias + cmd).sum()))
        target = np.where(coil_ok, np.clip(self.bias + cmd, self.vmin, self.vmax), self.bias)
        for r in self.regulators:
            r.release(~coil_ok)
        step = cfg.slew_per_s * dt
        self.out = self.out + np.clip(target - self.out, -step, step)
        h.sats((self.out <= self.vmin + 1e-06) | (self.out >= self.vmax - 1e-06), t)
        return self.out

    def law(self, i):
        return next((r.letter for r in self.regulators if r.claimed[i]), "")

    def banner(self):
        r, cfg = (self.rig, self.cfg)
        out = [
            "[rig] measured %s (%.1f days old); modes %s Hz = %s"
            % (r.measured, r.age_days(), np.round(r.f_hz, 5), "/".join(r.dof))
        ]
        for reg in self.regulators:
            out.append(
                "[%s] coils %s" % (type(reg).__name__, " ".join(map(str, np.nonzero(reg.coils)[0])))
            )
        out.append(
            "[limits] %.3f V per coil about bias, %.2f V summed" % (cfg.budget_v, cfg.sum_v_max)
        )
        return out + ["[rig] NOTE: " + s for s in r.notes]

    def status_header(self):
        cols = "".join(("     a%d" % i for i in range(self.n)))
        return c(
            "    time  state           Hz    chi2 " + cols + "   ratio to baseline, * locked", "dim"
        )

    def _cell(self, i):
        h = self.health
        if not self.enabled[i]:
            return c("    off", "dim")
        if h.floor_bad[i]:
            return c("  nosig", "dim")
        if not h.healthy[i]:
            return c("   DOWN", "red")
        text = "%2s%4.2f" % (self.law(i), self.ratio[i])
        if h.rail[i]:
            return c(text + "!", "red")
        return c(text + "*", "green") if self.locked[i] else c(text + " ", "yellow")

    def status_line(self, t):
        hz = self.dec.wire_hz
        return "%8.1f  %s  %5s  %6.2f " % (
            t,
            c("%-11s" % self.state, STATE[self.state]),
            "--" if hz != hz else "%.0f" % hz,
            self.est.chi2,
        ) + "".join((self._cell(i) for i in range(self.n)))

    def csv_header(self):
        cols = ["t", "state", "ctl", "n_avg", "chi2"] + ["qdot%d" % m for m in range(self.rig.nm)]
        for i in range(self.n):
            cols += ["%s%d" % (k, i) for k in ("a", "bp", "vel", "out", "ratio", "law")]
        return cols

    def csv_row(self, t, counts):
        e = self.est
        row = [
            "%.6f" % t,
            self.state,
            "1" if self.stepped else "0",
            str(self.n_avg),
            "%.4f" % e.chi2,
        ]
        row += ["%.6f" % x for x in e.qdot]
        for i in range(self.n):
            row += [
                "%.4f" % counts[i],
                "%.6f" % e.bp[i],
                "%.6f" % e.vel[i],
                "%.5f" % self.out[i],
                "%.4f" % self.ratio[i],
                self.law(i) or "-",
            ]
        return row
