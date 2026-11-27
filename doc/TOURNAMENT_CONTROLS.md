# Tournament controls and absolute pose reset

Both **TeleTourney** and **TeleOpTourney** use this map on either controller.
Buttons act once per press; overlapping presses on both controllers count once.

| Input | Action |
| --- | --- |
| Left stick Y / X | Forward/back / strafe |
| Left / right trigger | Rotate left / right |
| Y | Intake ON/OFF, retaining the selected direction |
| B | Reverse intake direction, retaining ON/OFF |
| X | Gate START/HOLD; HOLD captures and holds the stopped encoder position |
| Back | Shoot1 ON/OFF; starts ON at Play, 3200 RPM reverse |
| D-pad up / down | Shooter + / −50 RPM, bounded to 0–6000 |
| D-pad right / left | Gate + / −10 RPM, bounded to 0–435 |
| A | One-shot absolute Limelight field position **and heading** reset |
| Left bumper | Re-anchor the relative tag/odometry comparison |

The gate starts in HOLD at Play with 190 RPM reverse selected. Its existing jam
recovery remains active while feeding. Selected speeds survive START/HOLD and
ON/OFF toggles. Motors remain unpowered during INIT and stop on Driver Station STOP.
Automatic camera tracking runs alongside driving and mechanism controls.

## What A does

The operator confirmed the Limelight map matches Pedro's field frame: center
(0,0), +X toward Blue, +Y away from the audience. The reset reads the robot's
mapped `botpose` (meters converted to inches, yaw in degrees), rather than the
tag's relative distance or a previously captured odometry baseline. A visible
tag must be included in the uploaded field map (`botpose_tagcount > 0`). An
unmapped tag can still be tracked but cannot supply this absolute reset.

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
| Intake at Play | OFF | ON (retains imported default) |
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
