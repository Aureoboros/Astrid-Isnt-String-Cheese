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
The gate is unpowered during INIT. At Play it enters **HOLD/REVERSE**; B starts feeding. The intake starts
running **FORWARD continuously at 100% power** on Play (`RobotConfig.Mechanisms.INTAKE_POWER`).
Shoot1 starts ON at Play at its selected RPM; X toggles it. No drive, servo or
camera is initialized. Keep hands clear before Play.

- **A** swaps FORWARD/REVERSE without changing whether the motor is running.
- **B** toggles gate START/HOLD; releasing it keeps the selected state.
- **D-pad left/right** decreases/increases gate RPM by 10; it starts selected at 190 RPM.
- **X** toggles Shoot1; **D-pad up/down** increases/decreases shooter RPM by 50.
- Either controller works. Held/overlapping buttons toggle only once until released.
- Driver Station STOP or exiting the test stops all three motors.

Feeding uses closed-loop encoder velocity control. HOLD captures the current encoder
position once after cutting feeding power and commanding zero velocity, switches
to `RUN_TO_POSITION`, and applies up to
`RobotConfig.GateMotor.HOLD_POWER = 0.25` power. The hub corrects displacement back
toward that target; repeated updates and direction/RPM selection while holding do
not move the target. `HOLD_POSITION_TOLERANCE_TICKS = 2` sets the tolerance.
The hold power is a limit for correcting displacement; it does not command
continuous rotation. B HOLD stops feeding and locks the angle at that press.
Telemetry shows RUN/HOLD, hold target/error ticks, direction, and selected/measured RPM.
Test holding torque under load and adjust HOLD_POWER if needed. Holding the current
angle does not find a mechanically closed gate position; stop feeding with the
separator positioned to block nectar. The encoder cable is required for holding.
Driver Station STOP/cleanup releases holding power and uses BRAKE.
The gate's configured base motor direction is FORWARD. Stop and allow coast-down before
reversing; keep hands clear. Check the separator unloaded first.
Telemetry also shows the continuously running intake and configured odometry
ticks-to-inches coefficient; this test does not initialize or read Pinpoint.

## Encoder speed versus position

The configured 435-RPM profile uses **384.5 ticks per output-shaft revolution**.
`GateMotorSubsystem.getActualRpm()` computes `raw ticks/s × 60 / ticks per revolution`.
Both Test: Gate Motor and TeleTourney show the raw speed, accumulated position,
and fixed resolution separately. For this calibration, 60 RPM is 384.5 ticks/s;
120 RPM is 769 ticks/s; 400 ticks/s is about 62.4 RPM. Encoder position is an
accumulated count and need not match speed. The resolution remains 384.5 at all
speeds. HOLD retains its position and should read near zero speed after settling.

In Test: Gate Motor, D-pad LEFT/RIGHT changes the gate target by 10 RPM per press;
UP/DOWN changes the shooter. Compare selected RPM, applied RPM and measured speed
while the recovery state is RUNNING. If raw speed stays near 400 while selected
speed changes, the observation needs a hardware/control check, rather than a
change to the conversion formula. If the position remains fixed while the shaft
rotates, check its encoder cable on Expansion Hub motor port 1.

The operator confirmed the physical gate motor is 435 RPM. The reference XML has
an older 1500-RPM comment; the source calibration is 435 RPM / 384.5 ticks/rev.

## Checking a fixed measured RPM at different targets

The subsystem explicitly installs REV velocity PIDF starting gains after setting
motor metadata. Changing a generic motor's metadata alone does not install a
matching velocity PIDF controller. `RobotConfig.GateMotor.VELOCITY_*` defines the
starting gains: `F = 32767 / (435 × 384.5 / 60)`, `P = 0.1F`, `I = 0.1P`, `D = 0`.
These affect feeding velocity; position HOLD and the jam state machine are unchanged.
The test displays the hub's actual PIDF coefficients during INIT. Tune gains under
load if measured speed still overshoots or oscillates; they are not hardware-verified.

Run **Test: Gate Encoder Only** to isolate gate feedback. It initializes only the
gate, leaving intake and Shoot1 unpowered. B toggles START/HOLD, A reverses, and
D-pad LEFT/RIGHT adjusts the selected gate speed by 10 RPM. Check 60, 70, and 80
RPM while recovery reports RUNNING. Expected commanded encoder speeds are
384.5, 448.6, and 512.7 ticks/s. Telemetry also compares SDK velocity with RPM
computed independently from encoder position changes over a 250-ms window.

Both gate power and encoder cables must use **Expansion Hub port 1**. The older
combined test runs intake continuously and Shoot1 by default: an encoder cable
from one of those motors plugged into the gate encoder port can report its speed
instead of the gate's. If the gate physically spins in this isolated test but its
position count does not advance, check that wiring. If both speed estimates
remain near 396 RPM with only the gate powered, the discrepancy is real and needs
on-hardware PIDF/encoder inspection. A unit test verifies conversions/commands,
not the physical response of the controller or motor.

## Negative measured speed with a positive applied target

If the applied target is positive (for example +60 RPM) but both measured speed
and encoder travel are negative, the feedback direction is inconsistent with
the commanded motor direction. The controller can increase power trying to
correct an error that gets larger, causing near-full-speed rotation. Increasing
PIDF gains, taking an absolute value, or changing `setDirection` does not correct
the relative polarity: the FTC SDK reverses both commands and encoder readings.

Run **Test: Gate Polarity** unloaded to check the motor/encoder pair without a
closed-loop controller. Only gateMotor is initialized. It uses
`RUN_WITHOUT_ENCODER` and at most **10% open-loop power**. Hold **B** for a positive
pulse or **A** for a negative pulse on either controller; release stops immediately.
A pulse ends after **1 second** even if held, and both buttons must be released
before another pulse starts. Opposing buttons cancel. Driver Station STOP removes
power. This diagnostic intentionally uses no position HOLD.

Check **Encoder position / pulse travel** as well as velocity. A positive pulse
should increase the count; a negative pulse should decrease it. If the sign is
opposite, inspect the motor power lead polarity and encoder extension/cable pinout.
Both cables must belong to gateMotor and connect to Expansion Hub port 1. Restore
correct manufacturer wiring or replace the suspect cable; changing displayed signs
only hides the mismatch. If velocity reports -400 while the position count does not
change, it is not evidence of the gate actually rotating at -400 RPM. If the shaft
turns but its encoder count does not, inspect encoder pairing, cable and sensor.
Confirm the polarity test before resuming closed-loop feeding/holding. Hardware
polarity cannot be verified or repaired through source changes alone.

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

## Tournament controls

Both TeleTourney and TeleOpTourney initialize gateMotor unpowered, then hold its
position at Play. **X** toggles START/HOLD at **190 RPM reverse** on either
controller. D-pad LEFT/RIGHT decreases/increases selected speed by 10 RPM,
bounded to 0–435. Speed changes during HOLD do not start the motor. A fresh
X HOLD then X START clears a latched recovery fault. Existing automatic jam
recovery and encoder position holding are unchanged.

Y toggles intake ON/OFF; B reverses intake. Back toggles Shoot1, which starts ON
at 3200 RPM reverse. A resets absolute field pose from mapped Limelight data;
left bumper re-anchors the relative tag/odometry comparison. See
[current tournament controls](TOURNAMENT_CONTROLS.md) for the full map and defaults.
The isolated Test: Gate Motor retains A direction / B START/HOLD / X shooter.

The indexed separator helper's settings live in `RobotConfig.GateMotor`.
Automatic feeding remains disabled and uncalibrated (`AUTO_FEED_ENABLED = false`,
`TICKS_PER_INDEX = 0`); these settings do not change this manual test's A/B controls.

Red/Blue autos use real gated feeding. Tournament modes use the manual X gate toggle.
Motor travel alone cannot count actual nectar or guarantee that two pieces will not
arrive together. Automatic feed and recovery keep updating the drivetrain without sleeps.

Rebuild/deploy source and separately update/activate the hardware configuration
on the Robot Controller; repository XML edits are not automatically uploaded.
The reference XML now matches the current assigned devices, including Pinpoint.
Verify hub/module identifiers and physical wiring before importing it.
