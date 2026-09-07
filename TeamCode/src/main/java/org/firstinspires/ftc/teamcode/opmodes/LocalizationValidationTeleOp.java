package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagOdometryValidator;
import org.firstinspires.ftc.teamcode.localization.AprilTagOdometryValidator.ValidationResult;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;
import org.firstinspires.ftc.teamcode.localization.RobotLocalization;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.telemetry.LocalizationTelemetry;

/** On-robot test comparing Limelight tag motion against two-wheel odometry. */
@TeleOp(name = "Test: Odo vs AprilTag", group = "Diagnostics")
public final class LocalizationValidationTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    RobotLocalization localization = new RobotLocalization(hardwareMap);
    MecanumDrive drive = new MecanumDrive(hardwareMap);
    LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry();
    AprilTagOdometryValidator validator =
        new AprilTagOdometryValidator(
            RobotConfig.Localization.VALIDATION_POSITION_TOLERANCE_INCHES,
            RobotConfig.Localization.VALIDATION_HEIGHT_TOLERANCE_INCHES);

    localization.resetPose(0, 0, 0);
    localization.start();
    try {
      telemetry.addLine("Point at one AprilTag, press Play, then press A to capture a baseline.");
      telemetry.addLine("Drive slowly while keeping that tag visible. Press B to clear.");
      telemetry.update();
      waitForStart();

      boolean previousA = false;
      boolean previousB = false;
      int sampledFrames = 0;
      int detectedFrames = 0;

      while (opModeIsActive()) {
        localization.update();
        PoseEstimate pose = localization.getPoseEstimate();
        AprilTagObservation nearestTag = localization.getNearestAprilTag();

        sampledFrames++;
        if (nearestTag != null) {
          detectedFrames++;
        }

        boolean aDown = gamepad1.a || gamepad2.a;
        boolean bDown = gamepad1.b || gamepad2.b;
        boolean aPressed = aDown && !previousA;
        boolean bPressed = bDown && !previousB;
        previousA = aDown;
        previousB = bDown;

        if (aPressed && nearestTag != null) {
          validator.captureBaseline(pose, nearestTag);
          sampledFrames = 0;
          detectedFrames = 0;
        }
        if (bPressed) {
          validator.clearBaseline();
          sampledFrames = 0;
          detectedFrames = 0;
        }

        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        drive.driveRobotCentric(command.getDrive(), command.getStrafe(), command.getRotate());

        localizationTelemetry.add(telemetry, localization);
        telemetry.addLine();
        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData(
            "Detection Rate",
            "%.0f%% (%d/%d)",
            sampledFrames == 0 ? 0 : 100.0 * detectedFrames / sampledFrames,
            detectedFrames,
            sampledFrames);
        if (!localization.isLimelightConnected()) {
          telemetry.addData("AprilTag Detection", "FAIL - Limelight disconnected");
        } else if (nearestTag == null) {
          telemetry.addData("AprilTag Detection", "WAITING - no tag visible");
        } else {
          telemetry.addData("AprilTag Detection", "PASS - seeing tag %d", nearestTag.getId());
        }

        if (!validator.hasBaseline()) {
          telemetry.addData("Validation", "WAITING - press A while a tag is visible");
        } else {
          int tagId = validator.getTrackedTagId();
          AprilTagObservation trackedTag = localization.getAprilTagObservation(tagId);
          ValidationResult result = validator.validate(pose, trackedTag);
          telemetry.addData("Tracked Tag", tagId);

          if (!result.isAvailable()) {
            telemetry.addData("Validation", "NO DATA - tracked tag is not visible");
          } else {
            telemetry.addData("Validation", result.isPassing() ? "PASS" : "FAIL");
            telemetry.addData(
                "Horizontal Distance",
                "expected %.2f in, LL %.2f in",
                result.getExpectedDistanceInches(),
                result.getMeasuredDistanceInches());
            telemetry.addData(
                "Robot-relative Position Error",
                "%.2f in",
                result.getPositionErrorInches());
            telemetry.addData("Distance Error", "%.2f in", result.getDistanceErrorInches());
            telemetry.addData("Height Drift", "%.2f in", result.getHeightErrorInches());
            telemetry.addData(
                "Expected Forward / Left",
                "%.2f / %.2f in",
                result.getExpectedForwardInches(),
                result.getExpectedLeftInches());
          }
        }
        telemetry.update();
      }
    } finally {
      drive.stop();
      localization.stop();
    }
  }
}
