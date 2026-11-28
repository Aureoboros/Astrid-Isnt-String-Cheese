package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.pedroPathing.PushTunaFinal;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.localization.AbsolutePoseReset;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAprilTagValidation;
import org.firstinspires.ftc.teamcode.telemetry.PedroValidationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.CameraPanTelemetry;
import org.firstinspires.ftc.teamcode.control.LimelightPanController;
import org.firstinspires.ftc.teamcode.control.TeleTourneyMechanismControls;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateMotorSubsystem;
import org.firstinspires.ftc.teamcode.control.GateJamRecoveryController;
/** Competition drive TeleOp with live two-wheel odometry and Limelight AprilTag telemetry. */
@TeleOp(name = "TeleTourney", group = "Competition")
public class TeleTourney extends LinearOpMode {
  protected double getDrivePowerLimit() { return RobotConfig.TeleTourney.MAX_DRIVE_POWER; }
  protected TeleTourneyMechanismControls createMechanismControls() { return new TeleTourneyMechanismControls(); }
  protected Pose awaitStartingPose() { waitForStart(); return new Pose(0, 0, 0); }

  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    double drivePowerLimit = getDrivePowerLimit();
    follower.setMaxPower(drivePowerLimit);
    PushTunaFinal pushTuna = new PushTunaFinal(follower);
    GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConfig.Hardware.PINPOINT);
    AprilTagTracker aprilTags = new AprilTagTracker(hardwareMap);
    LimelightPanController pan = new LimelightPanController(hardwareMap, aprilTags);
    PedroAprilTagValidation validation = new PedroAprilTagValidation(
        RobotConfig.Localization.VALIDATION_POSITION_TOLERANCE_INCHES,
        RobotConfig.Localization.VALIDATION_HEIGHT_TOLERANCE_INCHES, true);

    SingleShooterSubsystem shooter = null;
    IntakeSubsystem intake = null;
    GateMotorSubsystem gateMotor = null;
    TeleTourneyMechanismControls mechanisms = createMechanismControls();
    AbsolutePoseReset poseReset = new AbsolutePoseReset(RobotConfig.TeleTourney.POSE_RESET_FRAMES,
        RobotConfig.TeleTourney.POSE_RESET_TIMEOUT_SECONDS, RobotConfig.TeleTourney.POSE_RESET_SPREAD_INCHES,
        RobotConfig.TeleTourney.POSE_RESET_HEADING_SPREAD_DEGREES, 72);
    try {
      shooter = new SingleShooterSubsystem(hardwareMap);
      shooter.setState(false, mechanisms.isShooterReversed(), mechanisms.getShooterRpm());
      intake = new IntakeSubsystem(hardwareMap, true);
      gateMotor = new GateMotorSubsystem(hardwareMap);
      telemetry.setMsTransmissionInterval(50);
      telemetry.addData("Status", "Drive, intake, Shoot1, gateMotor, odometry, and Limelight initialized");
      telemetry.addLine("Left stick: drive/strafe | Triggers: rotate");
      telemetry.addLine("Strafe heading hold: automatic; triggers override immediately");
      telemetry.addData("Drive power limit", "%.0f%%", drivePowerLimit * 100);
      telemetry.addLine("Y: intake START/HOLD (preserves selected direction)");
      telemetry.addLine("Intake HOLD requires its encoder | B: reverse intake | A: absolute LL pose reset");
      telemetry.addLine("Back: Shoot1 ON/OFF (starts ON at Play) | D-pad up/down: +/-50 RPM");
      telemetry.addData("X: gateMotor START/HOLD", "%s | Either gamepad",
          mechanisms.isGateReversed() ? "REVERSE" : "FORWARD");
      telemetry.addLine("D-pad left/right: turret pan left/right | Release: automatic tag tracking");
      telemetry.addLine("Left bumper: drive to (29, 42) | Right bumper: drive to (29, -42) | Any stick/trigger input cancels");
      telemetry.addLine("A reset requires a mapped tag and neutral drive");
      telemetry.addLine("Camera pan: horizontal image tracking; robot-space poses need calibration");
      telemetry.addData("Intake / Shoot1", "OFF / OFF");
      telemetry.addData("GateMotor", "OFF / %.0f RPM selected", mechanisms.getGateRpm());
      telemetry.addData("Shoot1 Direction", shooter.isReversed() ? "REVERSE" : "FORWARD");
      telemetry.addData("Shoot1 Selected RPM", "%.0f", mechanisms.getShooterRpm());
      telemetry.addData("Shoot1 Encoder ticks/rev", "%.0f", shooter.getEncoderTicksPerRevolution());
      CameraPanTelemetry.add(telemetry, pan, aprilTags);
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      Pose startingPose = awaitStartingPose();
      if (isStopRequested()) {
        return;
      }
      AutoPoseHandoff.clear(); // Consume/invalidate the handoff before driving away from this starting pose.
      follower.setStartingPose(startingPose);
      follower.setPose(startingPose);
      follower.startTeleopDrive(true);
      aprilTags.start();
      pan.start();
      shooter.setState(mechanisms.isShooterEnabled(), mechanisms.isShooterReversed(), mechanisms.getShooterRpm());
      intake.setState(mechanisms.isIntakeEnabled(), IntakeSubsystem.Direction.FORWARD);
      gateMotor.setState(false, mechanisms.isGateReversed(), mechanisms.getGateRpm());
      long lastTelemetryNanos = 0;
      long previousLoopNanos = System.nanoTime();
      boolean previousLeftBumper = false;
      boolean previousRightBumper = false;
      boolean bumperNavActive = false;
      long bumperNavStartedNanos = 0;
      double bumperNavTargetX = 0;
      double bumperNavTargetY = 0;
      String bumperNavStatus = "IDLE";
      while (opModeIsActive()) {
        long loopNanos = System.nanoTime();
        double loopMilliseconds = (loopNanos - previousLoopNanos) / 1e6;
        previousLoopNanos = loopNanos;
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        boolean leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean leftBumperPressed = leftBumper && !previousLeftBumper;
        boolean rightBumperPressed = rightBumper && !previousRightBumper;
        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;
        boolean pinpointReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;

        if (bumperNavActive) {
          // Driver input always wins; the same call hands the drivetrain back to the sticks.
          if (command.isActive()) {
            pushTuna.reset();
            bumperNavActive = false;
            bumperNavStatus = "CANCELLED: driver input";
          } else if (!pinpointReady) {
            pushTuna.reset();
            bumperNavActive = false;
            bumperNavStatus = "ABORTED: Pinpoint not READY";
          } else {
            follower.update();
            Pose navPose = follower.getPose();
            double navError = Math.hypot(bumperNavTargetX - navPose.getX(), bumperNavTargetY - navPose.getY());
            if (navError <= RobotConfig.TeleTourney.BUMPER_NAV_ARRIVE_INCHES) {
              pushTuna.reset(); // Back to driver control; position recovery re-arms at this spot.
              bumperNavActive = false;
              bumperNavStatus = String.format("ARRIVED at (%.0f, %.0f)", bumperNavTargetX, bumperNavTargetY);
            } else if ((loopNanos - bumperNavStartedNanos) / 1e9 >= RobotConfig.TeleTourney.BUMPER_NAV_TIMEOUT_SECONDS) {
              pushTuna.reset();
              bumperNavActive = false;
              bumperNavStatus = String.format("TIMED OUT, %.1f in short", navError);
            } else {
              bumperNavStatus = String.format("Driving to (%.0f, %.0f), %.1f in away",
                  bumperNavTargetX, bumperNavTargetY, navError);
            }
          }
        } else {
          if (leftBumperPressed != rightBumperPressed) { // Both pressed at once is ambiguous: ignore.
            if (poseReset.isActive()) {
              bumperNavStatus = "NOT STARTED: pose reset in progress";
            } else if (!pinpointReady) {
              bumperNavStatus = "NOT STARTED: Pinpoint not READY";
            } else if (command.isActive()) {
              bumperNavStatus = "NOT STARTED: release sticks/triggers first";
            } else {
              bumperNavTargetX = leftBumperPressed ? RobotConfig.TeleTourney.LEFT_BUMPER_TARGET_X_INCHES
                  : RobotConfig.TeleTourney.RIGHT_BUMPER_TARGET_X_INCHES;
              bumperNavTargetY = leftBumperPressed ? RobotConfig.TeleTourney.LEFT_BUMPER_TARGET_Y_INCHES
                  : RobotConfig.TeleTourney.RIGHT_BUMPER_TARGET_Y_INCHES;
              follower.updatePose();
              Pose from = follower.getPose();
              if (Math.hypot(bumperNavTargetX - from.getX(), bumperNavTargetY - from.getY())
                  <= RobotConfig.TeleTourney.BUMPER_NAV_ARRIVE_INCHES) {
                bumperNavStatus = String.format("Already at (%.0f, %.0f)", bumperNavTargetX, bumperNavTargetY);
              } else {
                // Straight line, keeping the heading the robot has right now.
                Path path = new Path(new BezierLine(from, new Pose(bumperNavTargetX, bumperNavTargetY, from.getHeading())));
                path.setConstantHeadingInterpolation(from.getHeading());
                pushTuna.reset(); // Clears any held pose and the strafe heading hold.
                follower.followPath(path, true);
                bumperNavActive = true;
                bumperNavStartedNanos = loopNanos;
                bumperNavStatus = String.format("Driving to (%.0f, %.0f)", bumperNavTargetX, bumperNavTargetY);
              }
            }
          }
          if (!bumperNavActive) {
            // Pedro uses positive left strafe and counterclockwise turn; the mapper uses right/CW.
            pushTuna.update(command.getDrive(), -command.getStrafe(), -command.getRotate());
          }
        }

        aprilTags.update();
        mechanisms.update(gamepad1, gamepad2);
        intake.setState(mechanisms.isIntakeEnabled(), mechanisms.isIntakeReversed()
            ? IntakeSubsystem.Direction.REVERSE : IntakeSubsystem.Direction.FORWARD);
        shooter.setState(mechanisms.isShooterEnabled(), mechanisms.isShooterReversed(), mechanisms.getShooterRpm());
        gateMotor.setState(mechanisms.isGateEnabled(), mechanisms.isGateReversed(), mechanisms.getGateRpm());
        if (mechanisms.isOdometryResetRequested() && !poseReset.isActive() && !bumperNavActive) {
          if (!aprilTags.isConnected()) { poseReset.unavailable("Limelight disconnected"); }
          else if (pan.beginFieldPoseMeasurement()) { poseReset.begin(System.nanoTime() / 1e9); }
          else { poseReset.unavailable("camera cannot provide a centered field measurement"); }
        }
        // A needs a stationary centered camera; manual panning resumes after that attempt.
        pan.update(!poseReset.isActive() && mechanisms.isTurretLeftRequested(),
            !poseReset.isActive() && mechanisms.isTurretRightRequested());
        Pose pose = follower.getPose();
        List<AprilTagObservation> visibleTags = aprilTags.getObservations();
        AprilTagObservation nearestTag = aprilTags.getNearestObservation();
        poseReset.update(System.nanoTime() / 1e9, aprilTags.isConnected(), pan.isFieldPoseMeasurementReady(),
            command.isActive(), aprilTags.getResultTimestamp(),
            new PoseEstimate(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())), aprilTags.getFieldPose());
        PoseEstimate corrected = poseReset.takeCorrection();
        if (corrected != null) {
          pushTuna.reset();
          follower.setPose(new Pose(corrected.getX(), corrected.getY(), Math.toRadians(corrected.getHeadingDegrees())));
          pose = follower.getPose();
          validation.resetBaseline();
        }
        if (!poseReset.isActive()) { pan.endFieldPoseMeasurement(); }
        validation.update(pose, visibleTags, aprilTags.isConnected(),
            false, false);

        // Control and aiming run every loop; expensive telemetry reads/formatting run at 10 Hz.
        if (loopNanos - lastTelemetryNanos >= 100_000_000L) {
          lastTelemetryNanos = loopNanos;
          telemetry.addData("Control loop", "%.1f ms (drive + camera active)", loopMilliseconds);
          telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
          telemetry.addData("Drive power limit", "%.0f%%", drivePowerLimit * 100);
          telemetry.addData("Position recovery", pushTuna.getState());
          telemetry.addData("Bumper nav (L: 29,42 | R: 29,-42)", bumperNavStatus);
          telemetry.addData("Strafe heading hold", pushTuna.isStrafeHeadingHolding() ? "ACTIVE" : "INACTIVE");
          if (pushTuna.isStrafeHeadingHolding()) {
            telemetry.addData("Heading target / error", "%.1f / %.1f deg",
                pushTuna.getStrafeHeadingTargetDegrees(), pushTuna.getStrafeHeadingErrorDegrees());
            telemetry.addData("Heading correction", "%.3f (Pedro CCW, before drive power cap)",
                pushTuna.getStrafeHeadingCorrection());
          }
          telemetry.addData("Camera control", pan.isManualPan() ? "MANUAL: D-pad pan"
              : pan.isTracking() ? "AUTO: tag tracking" : "HOLD");
          telemetry.addData("Intake", "%s / %s", intake.isEnabled() ? "RUN" : "HOLD", intake.getDirection());
          telemetry.addData("Intake target / actual RPM", "%.0f / %.0f (signed)",
              intake.getCommandedRpm(), intake.getActualRpm());
          telemetry.addData("Intake selected RPM / ticks per rev", "%.0f / %.1f",
              intake.getTargetRpm(), intake.getEncoderTicksPerRevolution());
          if (intake.isHolding()) {
            telemetry.addData("Intake hold target / error", "%d / %d ticks",
                intake.getHoldPositionTicks(), intake.getHoldErrorTicks());
          }
          telemetry.addData("Gate requested", "%s / %s", mechanisms.isGateEnabled() ? "RUN" : "HOLD",
              mechanisms.isGateReversed() ? "REVERSE" : "FORWARD");
          telemetry.addData("Gate mode", gateMotor.isHolding() ? "HOLD POSITION" : "VELOCITY / RECOVERY");
          telemetry.addData("Gate hub mode", gateMotor.getRunMode());
          if (gateMotor.isHolding()) {
            telemetry.addData("Gate hold target / error", "%d / %d ticks",
                gateMotor.getHoldPositionTicks(), gateMotor.getHoldErrorTicks());
          }
          telemetry.addData("Gate target / actual RPM", "%.0f / %.0f", gateMotor.getTargetRpm(), gateMotor.getActualRpm());
          telemetry.addData("Gate raw encoder speed", "%.1f ticks/s (signed)", gateMotor.getRawEncoderTicksPerSecond());
          telemetry.addData("Gate encoder position", "%d ticks (accumulated)", gateMotor.getPositionTicks());
          telemetry.addData("Gate encoder resolution", "%.1f ticks/rev (fixed)", gateMotor.getEncoderTicksPerRevolution());
          telemetry.addData("Gate applied RPM", "%.0f", gateMotor.getOutputRpm());
          telemetry.addData("Gate recovery / retries", "%s / %d", gateMotor.getRecoveryState(), gateMotor.getRecoveryAttempts());
          if (gateMotor.getRecoveryState() == GateJamRecoveryController.State.FAULT) {
            telemetry.addLine("Gate jam fault: clear obstruction, then X HOLD and X START to retry");
          }
          telemetry.addLine("Y: intake START/HOLD | X: gate START/HOLD | B: intake reverse | Back: shooter ON/OFF");
          telemetry.addData("A: absolute pose reset", poseReset.getStatus());
          telemetry.addData("Mapped tags for field pose", aprilTags.getMappedTagCount());
          telemetry.addLine("D-pad L/R: turret left/right | D-pad U/D: shooter +/-50 RPM");
          telemetry.addData("Shoot1", shooter.isActive() ? "ON" : "OFF");
          telemetry.addData("Shoot1 Direction", shooter.isReversed() ? "REVERSE" : "FORWARD");
          telemetry.addData("Shoot1 Selected RPM", "%.0f", shooter.getTargetRpm());
          double actualRpm = shooter.getActualRpm();
          telemetry.addData("Shoot1 Signed target RPM", "%.0f", shooter.getCommandedRpm());
          telemetry.addData("Shoot1 Actual speed RPM", "%.0f", Math.abs(actualRpm));
          telemetry.addData("Shoot1 Signed actual RPM", "%.0f", actualRpm);
          telemetry.addData("Shoot1 Encoder ticks/rev", "%.0f", shooter.getEncoderTicksPerRevolution());
          telemetry.addData("Shoot1 Encoder ticks/s", "%.1f", shooter.getRawEncoderTicksPerSecond());
          telemetry.addData("Shoot1 Error", "%.0f RPM", shooter.getCommandedRpm() - actualRpm);
          telemetry.addData(
              "Drive / Strafe / Rotate",
              "%.2f / %.2f / %.2f",
              command.getDrive(),
              command.getStrafe(),
              command.getRotate());
          telemetry.addData("Field Position", "X %.2f in, Y %.2f in", pose.getX(), pose.getY());
          telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
          telemetry.addData("Forward Odo", "%d ticks", pinpoint.getEncoderX());
          telemetry.addData("Strafe Odo", "%d ticks", pinpoint.getEncoderY());
          telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
          telemetry.addData("Pose Source", "Pedro: Pinpoint (2 pods + onboard IMU)");
          telemetry.addData("Limelight", aprilTags.isConnected() ? "CONNECTED" : "DISCONNECTED");
          telemetry.addData("AprilTags detected", aprilTags.getAimTargets().size());
          telemetry.addData("Tags with robot pose", visibleTags.size());
          if (nearestTag == null) {
            telemetry.addData("Nearest Tag", aprilTags.getAimTargets().isEmpty()
                ? "None visible" : "Robot-space measurements withheld");
          } else {
            telemetry.addData("Nearest Tag", "ID %d", nearestTag.getId());
            telemetry.addData(
                "Tag Position",
                "forward %.1f in, left %.1f in",
                nearestTag.getForwardInches(),
                nearestTag.getLeftInches());
            telemetry.addData(
                "Tag Distance",
                "horizontal %.1f in, range %.1f in",
                nearestTag.getHorizontalDistanceInches(),
                nearestTag.getRangeInches());
            telemetry.addData("Tag Height", "%.1f in", nearestTag.getHeightInches());
          }
          PedroValidationTelemetry.add(telemetry, validation);
          CameraPanTelemetry.add(telemetry, pan, aprilTags);
          OdometryTelemetry.add(telemetry);
          telemetry.update();
        }
        idle();
      }
    } finally {
      try {
        if (shooter != null) { shooter.stop(); }
      } finally {
        try {
          if (intake != null) { intake.stop(); }
        } finally {
          try {
            if (gateMotor != null) { gateMotor.stop(); }
          } finally {
            try {
              follower.breakFollowing();
            } finally {
              try { pan.close(); } finally { aprilTags.stop(); }
            }
          }
        }
      }
    }
  }
}
