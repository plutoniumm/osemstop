⏺ v9 techniques

┌──────────────────┬───────────────────────────────────────────────────────────────────────────────────┐
│                  │                                                                                   │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ PID on velocity  │ err = -vel, setpoint zero — P damps, I acts as a spring, D as added mass          │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ gain schedule    │ blends capture (−0.035) → steady (−0.030) on amplitude, slew-limited              │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ bandpass         │ 0.4–3.0 Hz, so DC and high-frequency noise never reach the loop                   │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ anti-windup      │ back-calculation, integrator unwinds from the clip instead of freezing            │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ saturation-latch │ a saturation trip clears instead of latching FAULT forever                        │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ rail-threshold   │ rail detected on raw counts over a fraction of a window, not an unbroken run      │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ runaway-baseline │ runaway measured against the median of calibration sub-windows, not a skewed mean │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ auto-disable     │ one railed OSEM is demoted, the rest keep damping; re-arms 2 s after it clears    │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ fast-refault     │ a fault repeating within 15 s reuses the baseline instead of recalibrating        │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ fast-calib       │ 20 s calibration becomes a ceiling; exits early once the floor settles            │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ warm-restart     │ reuses a known-good baseline across a fault, bounded by 4 reuses / 300 s          │
├──────────────────┼───────────────────────────────────────────────────────────────────────────────────┤
│ runaway-trend    │ runaway needs the envelope growing, not just large — stops faulting on ringdowns  │
└──────────────────┴───────────────────────────────────────────────────────────────────────────────────┘

v10 — everything above, plus:

┌─────────────────┬───────────────────────────────────────────────────────────────────────────────────────┐
│                 │                                                                                       │
├─────────────────┼───────────────────────────────────────────────────────────────────────────────────────┤
│ bias-trim       │ steps each coil's bias toward mid-scale, keeps the step only if total offset improved │
├─────────────────┼───────────────────────────────────────────────────────────────────────────────────────┤
│ soft-saturation │ a pinned actuator only faults if the envelope also stops falling                      │
└─────────────────┴───────────────────────────────────────────────────────────────────────────────────────┘

Same control law in both — identical Kp/Ki/Kd.