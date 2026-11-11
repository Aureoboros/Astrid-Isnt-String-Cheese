# Quick code-change reference

Java file links below open the current sources. Settings apply only to the OpModes
that use them; controller buttons are not one universal map.

## Hardware, movement, and tuning

| Change | File and setting |
| --- | --- |
| Hardware names | [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java): `Hardware`. Drive names are `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`, `backRightMotor`; mechanisms are `intake`, `Shoot1`, `Shoot2`, `servoMotor`. |
| Hub ports / device types | Activate the configuration on the Robot Controller. [.run/RobotConfig.xml](../.run/RobotConfig.xml) is a reference with some obsolete entries, not a complete current configuration. Confirmed: Expansion motor 0 = intake, motor 2 = Shoot1; Control servo 0 = servoMotor; Control I²C bus 1 = Pinpoint. Shoot2 is not installed. `.run/TeamCode.run.xml` is an Android Studio launch configuration, not hub wiring. |
| Drive motor directions — Pedro | [Constants.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java): `mecanumConstants`, the four `...MotorDirection(...)` calls. Used by TeleTourney and Pedro-based autonomous/tuning. Currently left motors REVERSE, right motors FORWARD. |
| Drive motor directions — legacy / isolated motor test | [MecanumDrive.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MecanumDrive.java): constructor `setDirection(...)`. Currently left motors explicitly REVERSE; right motors have no explicit direction assignment. Keep this consistent with Pedro when changing wiring. |
| Swap forward/strafe or reverse rotation | [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java): `Drive.SWAP_FORWARD_AND_STRAFE`, `REVERSE_ROTATION` (both currently false). Shared chassis compensation, separate from individual motor directions. |
| Wheel mixing / final motor powers | [MecanumKinematics.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/MecanumKinematics.java) and `MecanumDrive.setPower(...)` for legacy drive; [RobotMecanum.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/RobotMecanum.java) and [PedroDriveLayout.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/PedroDriveLayout.java) for Pedro's chassis adapter. |
| Odometry coefficient / pod offsets / signs | [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java): `Pinpoint.TICKS_PER_INCH`, derived `INCHES_PER_TICK`, `FORWARD_POD_Y_INCHES` (positive left), `STRAFE_POD_X_INCHES` (positive forward), and both encoder directions. Used by Pedro and the custom localizer. |
| Pedro PID / motion tuning | [Constants.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java): `followerConstants`, `mecanumConstants`, `pathConstraints`. [Tuning.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java) exposes Translational, Heading, Drive, velocity and other tuners. Copy tuned values into source for redeploy/restart persistence. |
| Legacy autonomous PID / speed / tolerances | [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java): `Autonomous.TRANSLATION_*`, `HEADING_*`, drive/turn limits and waypoint timeout. [HolonomicPoseController.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/navigation/HolonomicPoseController.java) consumes them; these are not Pedro's PID gains. |
| Intake power / shooter RPM | [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java): `Mechanisms.INTAKE_POWER`, `SHOOTER_TARGET_RPM`; `Diagnostics` holds diagnostic power/RPM settings. [IntakeSubsystem.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeSubsystem.java) applies intake direction; [SingleShooterSubsystem.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/SingleShooterSubsystem.java) controls Shoot1 in the single-shooter test. |

## Controller assignments

| Controls / OpMode | Where to edit |
| --- | --- |
| Shared driving: left stick Y forward/back, X strafe, triggers rotate | [DualGamepadDriveControls.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/DualGamepadDriveControls.java): `read(...)`, `fromInputs(...)`, deadband and rotation shaping. Gamepad 1 takes priority while active. TeleTourney converts right/CW commands to Pedro's left/CCW convention. |
| TeleTourney: X tracking toggle, Y camera center, A re-anchor, B clear tag/odo check | [TeleTourney.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/TeleTourney.java): button reads inside `runOpMode()`. It does not use the legacy mechanism button map. |
| Legacy mechanisms: A intake on/off, Y intake reverse, X shoot while held, B camera tracking toggle | [MechanismControls.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/MechanismControls.java): `read(...)` / `readButtons(...)`; [GameMechanisms.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/GameMechanisms.java) applies actions. B's historical servo action now requests camera tracking, not feeder power. |
| Test: Shooter — X on/off, Y reverse | [ShooterTestControls.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/ShooterTestControls.java); [ShooterTestTeleOp.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/ShooterTestTeleOp.java) coordinates Shoot1 and telemetry. Stop/coast before reversing physically. |
| Test: Servo Motor — B enable/hold, A center, D-pad position steps | [ServoMotorTestTeleOp.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/ServoMotorTestTeleOp.java). Manual test enabled; no movement at INIT. |
| Test: Limelight — X tracking, Y center, A display freeze | [LimelightTestTeleOp.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/LimelightTestTeleOp.java). |

## Servo and Limelight geometry

| Change | File and setting |
| --- | --- |
| Servo min / center / max | [CameraPanConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/CameraPanConfig.java): `MIN_POSITION = 0`, `CENTER_POSITION = 0.2`, `MAX_POSITION = 0.4`; manual step `MANUAL_POSITION_STEP = 0.005`. These are normalized servo positions, not degrees. |
| Servo yaw mapping / direction / tracking gain | Same file: `YAW_AT_MIN/CENTER/MAX_DEGREES`, `TX_TO_POSITION_SIGN`, `UI_YAW_TO_CCW_SIGN`, `RESULT_YAW_SIGN`, gain, deadband, maximum step and settling time. `TRAVEL_VERIFIED` is true for manual testing; automatic `ENABLED` and `GEOMETRY_VERIFIED` remain false pending calibration. |
| Camera mounting position and orientation | Same file: `PIVOT_FORWARD/RIGHT/UP_METERS`, `LENS_*_FROM_PIVOT_METERS`, `FIXED_PITCH_DEGREES`, `FIXED_ROLL_DEGREES`. Defaults are 1 inch forward, 3.75 inches left, 17 inches up, pitch 45°. Measure pivot/lens offsets and verify pitch; these are not all commissioned values. |
| Limelight browser mount fields | LL Forward/Right/Up = `rsf`/`rss`/`rsu` in meters; roll/pitch/yaw = `rsroll`/`rspitch`/`rsyaw` in degrees. [CameraMountPose.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/localization/CameraMountPose.java) maps these fields. Automatic pan updates them through [LimelightPipelineConfiguration.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/localization/LimelightPipelineConfiguration.java); with automatic pan disabled, configure the stationary mount in the browser yourself. |
| Pipeline / camera IP / tag size / accepted IDs | [RobotConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConfig.java): `Localization.APRILTAG_PIPELINE`, `LIMELIGHT_API_HOST`, `APRILTAG_SIZE_MM` (82.55 mm), `ALLOW_ALL_APRILTAG_IDS`, poll rate and staleness. [AprilTagTracker.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/localization/AprilTagTracker.java) applies pipeline size/filter during INIT; these HTTP changes do not request persistent flash writes. |
| Tag axes / distance calculations | [AprilTagObservation.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/localization/AprilTagObservation.java): raw robot-space X right, Y down, Z forward becomes forward = Z, left = −X, height = −Y. Tracker converts meters to inches. Range is 3D distance from the configured robot origin to tag center, not lens distance. |
| Odo vs. tag validation tolerances | `RobotConfig.Localization.VALIDATION_*`; [PedroAprilTagValidation.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/PedroAprilTagValidation.java) implements Pedro checks. Pinpoint remains the positioning source; tag checks do not overwrite its pose. |

## Autonomous and telemetry

- Field/start locations: [BiobuzzFieldMap.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/navigation/BiobuzzFieldMap.java); route points: [BiobuzzAutoRoutes.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/navigation/BiobuzzAutoRoutes.java).
- Hive shooting/pickup times, RPM, height bands and safety gates: [HiveAutoConfig.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/HiveAutoConfig.java). Square-test geometry/completion: [SquareTestRoutes.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/SquareTestRoutes.java) / [SquareTestCompletion.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/SquareTestCompletion.java).
- Shared coefficient print: [OdometryTelemetry.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/telemetry/OdometryTelemetry.java); camera pan display: [CameraPanTelemetry.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/telemetry/CameraPanTelemetry.java). Other displayed fields are often in the selected OpMode's `telemetry.addData(...)` calls.

After source edits, rebuild and deploy. Source edits alone do not update the hub's
active hardware configuration. Test drive directions with wheels raised and servo
travel slowly before full-power operation. Detailed setup: [Pinpoint](PINPOINT_SETUP.md),
[Limelight pipeline](LIMELIGHT_PIPELINE_SETUP.md), [camera pan](LIMELIGHT_PAN_SETUP.md).
