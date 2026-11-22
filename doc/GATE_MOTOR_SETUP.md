# Nectar separator motor

| Device | Connection | Purpose |
| --- | --- | --- |
| `intake` | Expansion Hub motor port 0 | Collect/feed nectar |
| `gateMotor` | Expansion Hub motor port 1 | 435-RPM separator before Shoot1 |
| `Shoot1` | Expansion Hub motor port 2 | Shooter wheel |

The name `gateMotor` is case-sensitive. Source settings are
`RobotConfig.Hardware.GATE_MOTOR` and `RobotConfig.GateMotor.MAX_RPM` (435).
`EXPANSION_HUB_PORT` (1) documents wiring; actual port assignment is in the active
Robot Controller configuration. The reference `.run/RobotConfig.xml` replaces
the obsolete `yOdo` device on port 1; odometry wheels remain connected to Pinpoint.

The code applies a goBILDA 435-RPM encoder profile with 384.5 ticks per output
revolution. The reference XML uses the generic `Motor` entry; the subsystem sets
the encoder profile at runtime. This rating does not calibrate one-ball travel.

## Controller test

Run **Test: Gate Motor** under Diagnostics. It initializes `gateMotor`, `intake`, and `Shoot1`.
The gate starts **OFF/FORWARD** and waits for Play and a B press. The intake starts
running **FORWARD continuously at 100% power** on Play (`RobotConfig.Mechanisms.INTAKE_POWER`).
Shoot1 starts ON at Play at its selected RPM; X toggles it. No drive, servo or
camera is initialized. Keep hands clear before Play.

- **A** swaps FORWARD/REVERSE without changing whether the motor is running.
- **B** toggles gate start/stop; releasing it does not stop the gate or intake.
- **D-pad left/right** decreases/increases gate RPM by 10; it starts selected at 60 RPM.
- **X** toggles Shoot1; **D-pad up/down** increases/decreases shooter RPM by 50.
- Either controller works. Held/overlapping buttons toggle only once until released.
- Driver Station STOP or exiting the test stops all three motors.

The test uses closed-loop encoder velocity control, with BRAKE while stopped.
Telemetry shows ON/OFF, direction, selected and measured RPM for gate and shooter.
The gate's configured base motor direction is FORWARD. Stop and allow coast-down before
reversing; keep hands clear. Check the separator unloaded first.
Telemetry also shows the continuously running intake and configured odometry
ticks-to-inches coefficient; this test does not initialize or read Pinpoint.

The indexed separator helper's settings live in `RobotConfig.GateMotor`.
Automatic feeding remains disabled and uncalibrated (`AUTO_FEED_ENABLED = false`,
`TICKS_PER_INDEX = 0`); these settings do not change this manual test's A/B controls.

Autonomous remains driving only, and TeleTourney controls are unchanged. This test
does not implement automatic nectar separation. Before adding automatic separator
logic, verify the gate's feed direction, mechanism travel/cycle, required interval
between nectar, and whether a sensor is available. Motor RPM alone cannot guarantee
that two nectar pieces will not arrive together. Prefer nonblocking feed timing so
waiting for a separation interval does not pause drivetrain updates.

Rebuild/deploy source and separately update/activate the hardware configuration
on the Robot Controller; repository XML edits are not automatically uploaded.
The reference XML now matches the current assigned devices, including Pinpoint.
Verify hub/module identifiers and physical wiring before importing it.
