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

## Automatic jam recovery

Jam recovery is enabled in `GateMotorSubsystem`, used by **Test: Gate Motor** and
both **BIOBUZZ: Red/Blue 3 Batches + Park** autos.
When the gate is requested ON at 5 RPM or higher, it allows 400 ms for startup.
It then compares encoder movement over a 500 ms window with the commanded speed.
Both encoder progress and measured RPM must be below 20% of the target to declare
a stall; a short speed dip alone does not trigger recovery.

Each recovery stops for 100 ms, reverses for 150 ms at up to 60 RPM (never faster
than the selected speed), pauses for 100 ms, and resumes the original direction
and selected RPM. The loop continues updating the other mechanisms throughout.
Changing direction or speed cancels the recovery pulse; B OFF and Driver Station
STOP stop the gate immediately.

Three consecutive retry cycles are allowed. If it remains stalled after those,
the gate latches **FAULT** and commands zero velocity. Clear the obstruction, then
press B to select OFF and again to select ON to retry. Two seconds of sustained
healthy encoder movement reset the consecutive retry count. Telemetry shows
recovery state, retry count, total recovery cycles started, and signed applied RPM.

All thresholds and timings are in `RobotConfig.GateMotor.JAM_*`; tune the reverse
duration on the actual separator. The encoder cable must be connected. This detects
a stalled motor, not a slipping wheel that jams while the motor shaft still turns.
The indexed `AutomaticNectarSeparator` helper remains disabled until calibrated;
this change applies to continuous-velocity gate control. In Red/Blue autos the gate
runs during shooting feed pulses at 60 RPM, pauses the intake during recovery, and
credits pulse progress from encoder travel. Shoot1 is 2900 RPM, reverse. See
[live Red/Blue auto setup](BIOBUZZ_PRELOADED_HIVE_AUTO.md). TeleTourney currently
does not initialize a gate motor.

The indexed separator helper's settings live in `RobotConfig.GateMotor`.
Automatic feeding remains disabled and uncalibrated (`AUTO_FEED_ENABLED = false`,
`TICKS_PER_INDEX = 0`); these settings do not change this manual test's A/B controls.

Red/Blue autos now use real gated feeding. TeleTourney controls are unchanged.
Motor travel alone cannot count actual nectar or guarantee that two pieces will not
arrive together. Automatic feed and recovery keep updating the drivetrain without sleeps.

Rebuild/deploy source and separately update/activate the hardware configuration
on the Robot Controller; repository XML edits are not automatically uploaded.
The reference XML now matches the current assigned devices, including Pinpoint.
Verify hub/module identifiers and physical wiring before importing it.
