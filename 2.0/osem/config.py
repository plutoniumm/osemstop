from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    vcc: float = 5.02
    adc_max: int = 1023
    control_hz: float = 100.0
    mains_null: bool = True
    disable: tuple = ()
    bias_swing: float = 0.3
    vmin: float = 0.0
    vmax: float = 2.5
    slew_per_s: float = 2.0
    budget_frac: float = 0.9
    # the supply folds back near this many volts summed over all coils
    sum_v_limit: float = 4.1
    sum_v_max: float = 3.6
    kp: float = 0.035
    kd: float = 0.00045
    d_hz: float = 2.0
    modal_kp: float = 0.035
    modal_scale: float = 2.0
    gain_slew_per_s: float = 0.1
    modal_cond_max: float = 12.0
    modal_min_sensors: int = 2
    modal_min_coils: int = 3
    t_amp_s: float = 50.0
    t_dc_s: float = 20.0
    calibration_s: float = 20.0
    calib_subwindow_s: float = 2.0
    calib_subwindows: int = 5
    calib_min_subwindows: int = 3
    calib_agree_n: int = 3
    calib_agree_tol: float = 1.2
    baseline_sanity: float = 3.0
    baseline_max_refusals: int = 3
    baseline_max_age_s: float = 300.0
    baseline_max_reuse: int = 4
    fast_refault_s: float = 15.0
    lock_factor: float = 0.35
    lock_sustain_s: float = 2.0
    lock_window_s: float = 2.0
    ratio_window_s: float = 1.0
    envelope_window_s: float = 2.0
    runaway_multiple: float = 1.8
    runaway_sustain_s: float = 5.0
    runaway_lag_s: float = 10.0
    runaway_growth: float = 1.02
    sat_sustain_s: float = 1.3
    sat_fraction: float = 0.8
    sat_decay: float = 0.98
    ref_floor_frac: float = 0.1
    rail_lo: int = 12
    rail_hi: int = 1011
    rail_sustain_s: float = 0.5
    rail_fraction: float = 0.8
    dead_pin_std: float = 1.3
    rearm_s: float = 2.0
    floor_frac: float = 0.1
    min_healthy: int = 1
    fault_clear_s: float = 5.0
    fault_clear_ratio: float = 1.4
    fault_max_hold_s: float = 30.0
    max_failed_engagements: int = 2
    quiet_window_s: float = 30.0
    quiet_pct: float = 20.0
    quiet_keep_hz: float = 10.0
    quiet_min_fill: float = 0.8
    status_period_s: float = 5.0
    rig_max_age_days: float = 7.0

    @property
    def budget_v(self):
        return self.budget_frac * self.bias_swing

    @property
    def volts_per_count(self):
        return self.vcc / self.adc_max
