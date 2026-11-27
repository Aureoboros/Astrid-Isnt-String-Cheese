# Tournament controls and absolute pose reset

Both **TeleTourney** and **TeleOpTourney** use this map on either controller.
Buttons act once per press; overlapping presses on both controllers count once.

| Input | Action |
| --- | --- |
| Left stick Y / X | Forward/back / strafe |
| Left / right trigger | Rotate left / right |
| Y | Intake START/HOLD, retaining the selected direction |
| B | Reverse intake direction, retaining RUN/HOLD |
| X | Gate START/HOLD; HOLD captures and holds the stopped encoder position |
| Back | Shoot1 ON/OFF; starts ON at Play, 3200 RPM reverse |
| D-pad up / down | Shooter + / −50 RPM, bounded to 0–6000 |
| D-pad left / right | Pan turret left / right while held; automatic tracking resumes on release |
| A | One-shot absolute Limelight field position **and heading** reset |
| Left bumper | Re-anchor the relative tag/odometry comparison |

The gate starts in HOLD at Play with 40 RPM reverse selected. Its existing jam
recovery remains active while feeding. Selected speeds survive mechanism toggles. Motors remain unpowered during INIT and stop on Driver Station STOP.
Automatic camera tracking runs alongside driving and mechanism controls. Manual
D-pad panning takes priority while held; opposing left/right inputs hold position.
Travel stays within 0.05–0.35, with 0.005 steps every 0.1 seconds. Tournament
gate feeding is fixed at 40 RPM (`RobotConfig.TeleTourney.GATE_RPM`).

Intake running speed defaults to **435 RPM** in both tournament entries and
**Test: Intake** (`RobotConfig.Mechanisms.INTAKE_TARGET_RPM`). The confirmed
435-RPM goBILDA intake uses **384.5 ticks per output-shaft revolution**, configured
in `RobotConfig.IntakeMotor`. Its cloned motor profile and explicit velocity PIDF
apply to this intake without changing the gate or drive motor profiles. Running
uses `RUN_USING_ENCODER`; reversing requests −435 RPM. Telemetry shows selected,
signed target and measured RPM, plus encoder ticks per revolution. The controller
uses initial PIDF gains that can be tuned under the actual intake load.

Intake HOLD captures the current encoder position once when running stops, then
uses `RUN_TO_POSITION` to resist movement. B changes the direction for the next
start while retaining the held target. It does not continuously recapture the
position. Hold power is **0.25**, with **2 ticks** of position tolerance, configured
in `RobotConfig.Mechanisms.INTAKE_HOLD_POWER` / `INTAKE_HOLD_TOLERANCE_TICKS`.
Telemetry shows intake RUN/HOLD, selected direction and hold target/error ticks.
The intake encoder must be connected to Expansion Hub motor port 0's matching
encoder input. Driver Station STOP releases active holding torque.

**Test: Intake** exercises the same holding behavior independently: **A** START/HOLD,
**Y** forward/reverse. Holding begins after Play. `IntakeSubsystem` enables holding
explicitly for these tournament/test modes; other users retain their existing
inactive behavior.

## Heading hold during strafing

Both tournament entries capture Pinpoint heading when strafe input starts with
no rotation command. While strafing continues, a separate heading PID adds a
bounded Pedro counterclockwise turn correction. This includes diagonal movement
with a strafe component. It holds the captured heading; it does not align the
robot to field zero. A start at −5° therefore holds −5°.

Any accepted trigger rotation, including a small squared command, immediately
overrides and clears heading hold. Releasing the triggers while still strafing
captures the new heading. Releasing strafe clears the target, so forward-only
movement and idle do not receive this correction. Existing stopped-position push
recovery remains active after the robot settles. A successful absolute field-pose
reset and a Pinpoint fault also clear the heading target.

Telemetry shows **Strafe heading hold**, **Heading target / error**, and **Heading
correction**. Correction is normalized Pedro CCW turn before the entry's overall
drive power cap. At TeleTourney's 35% cap, a 0.30 heading correction is limited
along with the rest of the drive command.

Tune `RobotConfig.TeleTourney.STRAFE_HEADING_*`: enabled=true, P=0.03, I=0.001,
D=0, maximum turn=0.30, integral limit=20 degree-seconds, deadband=1°. Heading
errors wrap across ±180° using the shortest turn. Integral is bounded and paused
when the previous correction is saturated in the error direction. Invalid or
long (>0.25 s) sample intervals reset integral/derivative history. Gains are initial
values requiring on-robot tuning; they do not change Pedro autonomous PID settings.

Implementation: `control/StrafeHeadingHold.java`, integrated into the existing
manual-drive/push-recovery loop in `pedroPathing/PushTunaFinal.java`.

## What A does

The operator confirmed the Limelight map matches Pedro's field frame: center
(0,0), +X toward Blue, +Y away from the audience. The reset reads the robot's
mapped `botpose` (meters converted to inches, yaw in degrees), rather than the
tag's relative distance or a previously captured odometry baseline. A visible
tag must be included in the uploaded field map (`botpose_tagcount > 0`). An
unmapped tag can still be tracked but cannot supply this absolute reset.

During an A measurement, manual pan is temporarily suppressed so the camera
can stay centered; held D-pad inputs take effect again after the attempt.

A briefly holds the camera at CENTER **0.2**, applies/verifies the known center
mount (forward 1 inch, right −3.75 inches, up 17 inches, pitch 45°, yaw 0°),
and waits for settling plus fresh frames carrying that mount. Release drive
sticks/triggers while the reset collects **5 distinct frames**. It averages
vision-minus-odometry offsets, then applies them to the current Pedro/Pinpoint
pose, accounting for small movement during collection. Push resistance drops
its previous hold target and re-arms at the corrected location.

Samples must agree within **2 inches** and **5°** of their average correction.
Invalid/nonfinite poses, robot height beyond 6 inches, robot roll/pitch beyond
15°, and positions beyond the 144-inch field are rejected. A real mapped pose
at (0,0) is valid; absent or malformed poses are not substituted with zeros.
There is a **4-second** measurement timeout. Missing map data, a failed mount
update, disagreement, or timeout leaves odometry unchanged and reports the
reason under **A: absolute pose reset**. Driving and mechanisms keep updating
without sleeps; active drive input clears the collected samples.

After the attempt, automatic image tracking resumes and robot-space poses are
withheld again before the servo moves, since off-center pan yaw remains
uncalibrated. Pinpoint stays the position source between explicit A resets.

## Entry differences and quick edits

| Setting | TeleTourney | TeleOpTourney |
| --- | --- | --- |
| Intake at Play | HOLD | RUN (retains imported default) |
| Drive/push recovery cap | 35% | 100% (retains imported manual cap) |
| Initial pose | (0,0,0) | Saved autonomous end pose or selected Red/Blue start |

TeleOpTourney INIT D-pad left/right selects Red/Blue, down uses the alliance
start, and up restores a saved autonomous pose with its matching alliance.
The shared runtime implementation is `opmodes/TeleTourney.java`;
`opmodes/TeleOpTourney.java` supplies these entry-specific defaults.

- Button map/defaults: `control/TeleTourneyMechanismControls.java`;
  `control/TeleOpTourneyControls.java` selects the intake-ON default.
- Reset limits: `config/RobotConfig.java`, `TeleTourney.POSE_RESET_*`.
- Reset math: `localization/AbsolutePoseReset.java`.
- Field-pose parsing/units: `localization/LimelightFieldPose.java` and `AprilTagTracker.java`.
- Center-mount measurement/async configuration: `control/LimelightPanController.java`;
  mounting dimensions and servo bounds: `config/CameraPanConfig.java`.

Rebuild and install the APK to use these source changes on the robot. Hardware
motion, tag-map accuracy and the center mounting measurement need on-robot checking.
