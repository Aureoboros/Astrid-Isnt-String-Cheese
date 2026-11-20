# Nectar separator motor

| Device | Connection | Purpose |
| --- | --- | --- |
| `intake` | Expansion Hub motor port 0 | Collect/feed nectar |
| `gateMotor` | Expansion Hub motor port 1 | goBILDA 435 RPM separator before Shoot1 |
| `Shoot1` | Expansion Hub motor port 2 | Shooter wheel |

The name `gateMotor` is case-sensitive. Source settings are
`RobotConfig.Hardware.GATE_MOTOR`, `RobotConfig.GateMotor.MAX_RPM` (435) and
`RobotConfig.GateMotor.TICKS_PER_REVOLUTION` (384.5).
`EXPANSION_HUB_PORT` (1) documents wiring; actual port assignment is in the active
Robot Controller configuration. The reference `.run/RobotConfig.xml` replaces
the obsolete `yOdo` device on port 1; odometry wheels remain connected to Pinpoint.

The gate is a goBILDA 5203-series 435 RPM motor (13.7:1, 384.5 encoder counts per
output revolution). The reference XML still lists the generic `Motor` type; the code
applies the 435 RPM profile to this motor at startup, so no hub type change is needed.
The encoder cable must be connected: closed-loop speed control without encoder
feedback will drive the motor toward full speed.

## Controller test

Run **Test: Gate Motor** under Diagnostics. It initializes `gateMotor`, `intake`
and `Shoot1`. From Play, the intake runs FORWARD continuously and Shoot1 spins at
its selected RPM until X turns it off (direction from `RobotConfig.TeleTourney.SHOOTER_REVERSED`).
The gate starts **OFF/FORWARD** and only moves after a B press.

- Gate "FORWARD" is the motor's physical REVERSE (`RobotConfig.GateMotor.MOTOR_DIRECTION`).
- **A** swaps gate FORWARD/REVERSE without changing whether it is running.
- **B** toggles gate start/stop; releasing it does not stop the motor.
- **X** toggles Shoot1 on/off (it starts ON at Play); the selected RPM is kept.
- **D-pad up/down** changes Shoot1 RPM by 50 (starts at 4500, range 0-6000).
- **D-pad left/right** changes gate RPM by 25 (starts at 100, range 0-435).
- Either controller works. Held/overlapping buttons act only once until released.
- Driver Station STOP or exiting the test stops all three motors.

Shoot1 uses closed-loop encoder velocity, and telemetry shows target and actual RPM.
The gate also uses closed-loop encoder velocity (RPM x 384.5 / 60 ticks/s) with
BRAKE at zero power, and telemetry shows its target and measured RPM. Starting values are
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
