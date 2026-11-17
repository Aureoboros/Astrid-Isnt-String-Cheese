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

Run **Test: Gate Motor** under Diagnostics. It initializes `gateMotor`, `intake`
and `Shoot1`. From Play, the intake runs FORWARD continuously and Shoot1 spins at
its selected RPM continuously (direction from `RobotConfig.TeleTourney.SHOOTER_REVERSED`).
The gate starts **OFF/FORWARD** and only moves after a B press.

- **A** swaps gate FORWARD/REVERSE without changing whether it is running.
- **B** toggles gate start/stop; releasing it does not stop the motor.
- **D-pad up/down** changes Shoot1 RPM by 50 (starts at 4500, range 0-6000).
- **D-pad left/right** changes gate RPM by 25 (starts at 200, range 0-1500).
- Either controller works. Held/overlapping buttons act only once until released.
- Driver Station STOP or exiting the test stops all three motors.

Shoot1 uses closed-loop encoder velocity, and telemetry shows target and actual RPM.
The gate is **open-loop**: power = target RPM / 1500 (200 RPM is about 13% power),
with BRAKE at zero power. No gate encoder profile is assumed, so its actual speed
is an estimate that is not regulated or measured. Starting values are
`RobotConfig.GateMotor.TEST_INITIAL_RPM` and `RobotConfig.Diagnostics.GATE_TEST_SHOOTER_RPM`.
Stop and allow coast-down before reversing; keep hands clear of the intake,
separator and shooter wheel.

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
