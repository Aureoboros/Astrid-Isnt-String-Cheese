package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAprilTagValidation;
import org.firstinspires.ftc.teamcode.telemetry.PedroValidationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.CameraPanTelemetry;
import org.firstinspires.ftc.teamcode.control.LimelightPanController;
import org.firstinspires.ftc.teamcode.control.TeleTourneyMechanismControls;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;

/** Competition drive TeleOp with live two-wheel odometry and Limelight AprilTag telemetry. */
@TeleOp(name = "TeleTourney", group = "Competition")
public final class TeleTourney extends LinearOpMode {
  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConfig.Hardware.PINPOINT);
    AprilTagTracker aprilTags = new AprilTagTracker(hardwareMap);
    LimelightPanController pan = new LimelightPanController(hardwareMap, aprilTags);
    PedroAprilTagValidation validation = new PedroAprilTagValidation(
        RobotConfig.Localization.VALIDATION_POSITION_TOLERANCE_INCHES,
        RobotConfig.Localization.VALIDATION_HEIGHT_TOLERANCE_INCHES, true);

    SingleShooterSubsystem shooter = null;
    IntakeSubsystem intake = null;
    TeleTourneyMechanismControls mechanisms = new TeleTourneyMechanismControls();
    try {
      shooter = new SingleShooterSubsystem(hardwareMap);
      shooter.setState(false, mechanisms.isShooterReversed(), mechanisms.getShooterRpm());
      intake = new IntakeSubsystem(hardwareMap);
      telemetry.setMsTransmissionInterval(50);
      telemetry.addData("Status", "Drive, intake, Shoot1, odometry, and Limelight initialized");
      telemetry.addLine("Left stick: drive/strafe | Triggers: rotate");
      telemetry.addLine("Y: intake FORWARD -> STOP -> REVERSE -> STOP (per press)");
      telemetry.addLine("X: Shoot1 ON/OFF | D-pad up/down: +/-100 RPM");
      telemetry.addLine("First visible tag anchors the check. A: re-anchor | B: clear");
      telemetry.addLine("Camera pan: automatic only if calibrated and enabled in config");
      telemetry.addData("Intake / Shoot1", "OFF / OFF");
      telemetry.addData("Shoot1 Direction", shooter.isReversed() ? "REVERSE" : "FORWARD");
      telemetry.addData("Shoot1 Selected RPM", "%.0f", mechanisms.getShooterRpm());
      telemetry.addData("Shoot1 Encoder ticks/rev", "%.0f", shooter.getEncoderTicksPerRevolution());
      CameraPanTelemetry.add(telemetry, pan, aprilTags);
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) {
        return;
      }
      follower.setStartingPose(new Pose(0, 0, 0));
      follower.setPose(new Pose(0, 0, 0));
      follower.startTeleopDrive(true);
      aprilTags.start();
      pan.start();
      while (opModeIsActive()) {
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        // Pedro uses positive left strafe and counterclockwise turn; the mapper uses right/CW.
        follower.setTeleOpDrive(command.getDrive(), -command.getStrafe(), -command.getRotate(), true);

        follower.update();
        aprilTags.update();
        mechanisms.update(gamepad1, gamepad2);
        intake.setState(mechanisms.isIntakeEnabled(), mechanisms.isIntakeReversed()
            ? IntakeSubsystem.Direction.REVERSE : IntakeSubsystem.Direction.FORWARD);
        shooter.setState(mechanisms.isShooterEnabled(), mechanisms.isShooterReversed(), mechanisms.getShooterRpm());
        pan.update();
        Pose pose = follower.getPose();
        List<AprilTagObservation> visibleTags = aprilTags.getObservations();
        AprilTagObservation nearestTag = aprilTags.getNearestObservation();
        validation.update(pose, visibleTags, aprilTags.isConnected(),
            gamepad1.a || gamepad2.a, gamepad1.b || gamepad2.b);

        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData("Intake", intake.isEnabled() ? "ON / " + intake.getDirection() : "OFF");
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
        telemetry.addData("AprilTags", visibleTags.size());
        if (nearestTag == null) {
          telemetry.addData("Nearest Tag", "None visible");
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
            follower.breakFollowing();
          } finally {
            try { pan.close(); } finally { aprilTags.stop(); }
          }
        }
      }
    }
  }
}
