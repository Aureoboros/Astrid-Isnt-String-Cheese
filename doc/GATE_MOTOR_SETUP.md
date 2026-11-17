# Nectar separator motor

| Device | Connection | Purpose |
| --- | --- | --- |
| `intake` | Expansion Hub motor port 0 | Collect/feed nectar |
| `gateMotor` | Expansion Hub motor port 1 | 1500-RPM separator before Shoot1 |
| `Shoot1` | Expansion Hub motor port 2 | Shooter wheel |

The name `gateMotor` is case-sensitive. Source settings are
`RobotConfig.Hardware.GATE_MOTOR` and `RobotConfig.GateMotor.MAX_RPM` (1500).
`EXPANSION_HUB_PORT` (1) documents wiring; actual port assignment is in the active
Robot Controller configuration. The reference `.run/RobotConfig.xml` replaces
the obsolete `yOdo` device on port 1; odometry wheels remain connected to Pinpoint.

The motor brand/model, gear ratio and encoder ticks per output revolution have
not been confirmed. The XML therefore uses the unspecified `Motor` profile,
not Shoot1's 6000-RPM profile. Select the correct motor type on the hub before
closed-loop RPM control. The 1500-RPM rating is not a startup command or a
calibrated feed speed.

## Controller test

Run **Test: Gate Motor** under Diagnostics. It initializes `gateMotor` and `intake`.
The gate starts **OFF/FORWARD** and waits for Play and a B press. The intake starts
running **FORWARD continuously at 100% power** on Play (`RobotConfig.Mechanisms.INTAKE_POWER`).
No drive, shooter, servo or camera is initialized. Keep hands clear before Play.

- **A** swaps FORWARD/REVERSE without changing whether the motor is running.
- **B** toggles gate start/stop; releasing it does not stop the gate or intake.
- Either controller works. Held/overlapping buttons toggle only once until released.
- Driver Station STOP or exiting the test commands zero power to both motors.

The test uses **20% open-loop power** (`RobotConfig.GateMotor.TEST_POWER = 0.2`)
with BRAKE at zero power. Telemetry shows ON/OFF, direction, signed commanded
power, and the 1500-RPM **rating**, not a measured RPM. No encoder profile is
assumed, so actual speed is not regulated. Stop and allow coast-down before
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
Verify hub/module identifiers and physical wiring before importing it; gate motor
encoder metadata still needs confirmation.
