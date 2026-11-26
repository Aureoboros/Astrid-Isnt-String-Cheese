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
The gate is unpowered during INIT. At Play it enters **HOLD/FORWARD**; B starts feeding. The intake starts
running **FORWARD continuously at 100% power** on Play (`RobotConfig.Mechanisms.INTAKE_POWER`).
Shoot1 starts ON at Play at its selected RPM; X toggles it. No drive, servo or
camera is initialized. Keep hands clear before Play.

- **A** swaps FORWARD/REVERSE without changing whether the motor is running.
- **B** toggles gate START/HOLD; releasing it keeps the selected state.
- **D-pad left/right** decreases/increases gate RPM by 10; it starts selected at 60 RPM.
- **X** toggles Shoot1; **D-pad up/down** increases/decreases shooter RPM by 50.
- Either controller works. Held/overlapping buttons toggle only once until released.
- Driver Station STOP or exiting the test stops all three motors.

Feeding uses closed-loop encoder velocity control. HOLD captures the current encoder
position once, switches to `RUN_TO_POSITION`, and applies up to
`RobotConfig.GateMotor.HOLD_POWER = 0.25` power. The hub corrects displacement back
toward that target; repeated updates and direction/RPM selection while holding do
not move the target. `HOLD_POSITION_TOLERANCE_TICKS = 2` sets the tolerance.
Telemetry shows RUN/HOLD, hold target/error ticks, direction, and selected/measured RPM.
Test holding torque under load and adjust HOLD_POWER if needed. Holding the current
angle does not find a mechanically closed gate position; stop feeding with the
separator positioned to block nectar. The encoder cable is required for holding.
Driver Station STOP/cleanup releases holding power and uses BRAKE.
The gate's configured base motor direction is FORWARD. Stop and allow coast-down before
reversing; keep hands clear. Check the separator unloaded first.
Telemetry also shows the continuously running intake and configured odometry
ticks-to-inches coefficient; this test does not initialize or read Pinpoint.

## Automatic jam recovery

Jam recovery is enabled in `GateMotorSubsystem`, used by **Test: Gate Motor**,
**TeleTourney**, and both **BIOBUZZ: Red/Blue 3 Batches + Park** autos.
When the gate is requested ON at 5 RPM or higher, it allows 400 ms for startup.
It then compares encoder movement over a 500 ms window with the commanded speed.
Both encoder progress and measured RPM must be below 20% of the target to declare
a stall; a short speed dip alone does not trigger recovery.

Each recovery stops for 100 ms, reverses for 150 ms at up to 60 RPM (never faster
than the selected speed), pauses for 100 ms, and resumes the original direction
and selected RPM. The loop continues updating the other mechanisms throughout.
Changing direction or speed cancels the recovery pulse; B HOLD cancels recovery
and holds the current position. Driver Station STOP removes motor power.
Temporary recovery pauses remain zero-velocity pauses, retaining the existing
stop/reverse/retry sequence.

Three consecutive retry cycles are allowed. If it remains stalled after those,
the gate latches **FAULT** and commands zero velocity. Clear the obstruction, then
press B to select HOLD and again to select START to retry. Two seconds of sustained
healthy encoder movement reset the consecutive retry count. Telemetry shows
recovery state, retry count, total recovery cycles started, and signed applied RPM.

All thresholds and timings are in `RobotConfig.GateMotor.JAM_*`; tune the reverse
duration on the actual separator. The encoder cable must be connected. This detects
a stalled motor, not a slipping wheel that jams while the motor shaft still turns.
The indexed `AutomaticNectarSeparator` helper remains disabled until calibrated;
this change applies to continuous-velocity gate control and its inactive position hold. In Red/Blue autos the gate
runs during shooting feed pulses at 60 RPM, pauses the intake during recovery, and
credits pulse progress from encoder travel. Shoot1 is 2900 RPM, reverse. See
[live Red/Blue auto setup](BIOBUZZ_PRELOADED_HIVE_AUTO.md).

## TeleTourney controls

TeleTourney initializes gateMotor unpowered, then holds its position at Play.
**B** on either controller toggles START/HOLD at **60 RPM forward**, using
`RobotConfig.GateMotor.TEST_INITIAL_RPM`.
Held or overlapping B presses toggle only once until both controllers release B.
The gate runs independently of X (shooter) and Y (intake), with automatic jam
recovery. A gate fault stays stopped until B HOLD then B START resets it. Driver
Station STOP stops the gate with the other motors. Telemetry shows requested
state, hold target/error ticks, target/actual/applied RPM, recovery state and retry count.

**Y** only toggles intake ON/OFF, retaining its selected direction.
**A** reverses intake direction while preserving its current ON/OFF state.
**Left bumper** re-anchors the tag/odometry check; **Back** clears it.
B is exclusively the gate toggle in TeleTourney.

The indexed separator helper's settings live in `RobotConfig.GateMotor`.
Automatic feeding remains disabled and uncalibrated (`AUTO_FEED_ENABLED = false`,
`TICKS_PER_INDEX = 0`); these settings do not change this manual test's A/B controls.

Red/Blue autos use real gated feeding. TeleTourney uses the manual B gate toggle.
Motor travel alone cannot count actual nectar or guarantee that two pieces will not
arrive together. Automatic feed and recovery keep updating the drivetrain without sleeps.

Rebuild/deploy source and separately update/activate the hardware configuration
on the Robot Controller; repository XML edits are not automatically uploaded.
The reference XML now matches the current assigned devices, including Pinpoint.
Verify hub/module identifiers and physical wiring before importing it.
