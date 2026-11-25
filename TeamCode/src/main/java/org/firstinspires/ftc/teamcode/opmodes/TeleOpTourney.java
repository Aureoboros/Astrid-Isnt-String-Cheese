package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.control.GateJamRecoveryController;
import org.firstinspires.ftc.teamcode.control.TeleOpTourneyControls;
import org.firstinspires.ftc.teamcode.localization.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute.Stage;
import org.firstinspires.ftc.teamcode.navigation.PreloadedHiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PushTunaFinal;
import org.firstinspires.ftc.teamcode.subsystems.GateMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/**
 * Competition TeleOp.
 * Left stick: drive/strafe | Triggers: rotate | A: drive to the autonomous shooting pose
 * Y: intake toggle (starts ON) | X: gate toggle (starts OFF) | B (hold): spit
 * D-pad up/down: launcher RPM. Launcher (Shoot1) runs the whole match from 3200 RPM.
 * The gate uses automatic jam recovery.
 */
@TeleOp(name = "TeleOpTourney", group = "Competition")
public final class TeleOpTourney extends LinearOpMode {
  private Follower follower;
  private boolean drivingToShoot;
  private double driveToShootStarted;
  private String driveToShootStatus = "Idle";

  @Override
  public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    BiobuzzFieldMap field = new BiobuzzFieldMap(
        RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
    boolean useAutoPose = AutoPoseHandoff.hasPose();
    Alliance alliance = useAutoPose ? AutoPoseHandoff.getAlliance() : Alliance.RED;

    follower = Constants.createFollower(hardwareMap);
    PushTunaFinal pushTuna = new PushTunaFinal(follower);
    TeleOpTourneyControls controls = new TeleOpTourneyControls();
    SingleShooterSubsystem shooter = null;
    IntakeSubsystem intake = null;
    GateMotorSubsystem gate = null;
    try {
      shooter = new SingleShooterSubsystem(hardwareMap);
      intake = new IntakeSubsystem(hardwareMap);
      gate = new GateMotorSubsystem(hardwareMap);
      shooter.setState(false, RobotConfig.TeleTourney.SHOOTER_REVERSED,
          controls.getShooterRpm());
      gate.setState(false, false, RobotConfig.TeleOpTourney.GATE_RPM);

      while (opModeInInit()) {
        if (gamepad1.dpad_left || gamepad2.dpad_left) { alliance = Alliance.RED; }
        if (gamepad1.dpad_right || gamepad2.dpad_right) { alliance = Alliance.BLUE; }
        if (gamepad1.dpad_down || gamepad2.dpad_down) { useAutoPose = false; }
        if ((gamepad1.dpad_up || gamepad2.dpad_up) && AutoPoseHandoff.hasPose()) {
          useAutoPose = true;
          alliance = AutoPoseHandoff.getAlliance();
        }
        Waypoint start = startPose(field, alliance, useAutoPose);
        Waypoint shoot = shootPose(alliance);
        telemetry.addData("Alliance", "%s (D-pad left = RED, right = BLUE)", alliance);
        telemetry.addData("Starting pose source", useAutoPose
            ? "End of autonomous (D-pad down: use alliance start instead)"
            : "Alliance autonomous start" + (AutoPoseHandoff.hasPose()
                ? " (D-pad up: use autonomous end pose)" : ""));
        telemetry.addData("Starting pose", "X %.1f, Y %.1f, %.0f°",
            start.getX(), start.getY(), start.getHeadingDegrees());
        telemetry.addData("Shoot pose (A)", "X %.1f, Y %.1f, %.0f°",
            shoot.getX(), shoot.getY(), shoot.getHeadingDegrees());
        telemetry.addLine("Left stick: drive | Triggers: rotate | A: go to shoot pose");
        telemetry.addLine("Y: intake on/off (starts ON) | X: gate on/off (starts OFF)");
        telemetry.addLine("B (hold): spit - intake + gate reversed");
        telemetry.addData("Launcher", "ON whole match, starts at %.0f RPM (D-pad up/down: +/-%.0f)",
            controls.getShooterRpm(), RobotConfig.TeleOpTourney.SHOOTER_RPM_STEP);
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
      if (isStopRequested()) { return; }

      Waypoint start = startPose(field, alliance, useAutoPose);
      Waypoint shoot = shootPose(alliance);
      // Consumed: a second TeleOp run without a new autonomous must not reuse a stale pose.
      AutoPoseHandoff.clear();
      follower.setStartingPose(toPose(start));
      follower.setPose(toPose(start));
      follower.startTeleopDrive(true);

      while (opModeIsActive()) {
        controls.update(gamepad1, gamepad2);
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        boolean driverInput = command.isActive();

        if (controls.isDriveToShootRequested() && !drivingToShoot) {
          startDriveToShoot(shoot);
        }
        if (drivingToShoot) {
          follower.update();
          String finished = driveToShootFinished(shoot, driverInput);
          if (finished != null) {
            endDriveToShoot(finished);
            pushTuna.reset(); // Old hold pose is wherever the driver was before A.
          }
        } else {
          // Pedro uses positive left strafe and counterclockwise turn; the mapper uses right/CW.
          pushTuna.update(command.getDrive(), -command.getStrafe(), -command.getRotate());
        }

        shooter.setState(true, RobotConfig.TeleTourney.SHOOTER_REVERSED,
            controls.getShooterRpm());
        intake.setState(controls.isIntakeEnabled(), controls.isIntakeReversed()
            ? IntakeSubsystem.Direction.REVERSE : IntakeSubsystem.Direction.FORWARD);
        // Called every loop: this is what advances the gate's jam-recovery state machine.
        gate.setState(controls.isGateEnabled(), controls.isGateReversed(),
            RobotConfig.TeleOpTourney.GATE_RPM);

        Pose pose = follower.getPose();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive to shoot (A)", driveToShootStatus);
        telemetry.addData("Position recovery", pushTuna.getState());
        telemetry.addData("Field pose", "X %.1f, Y %.1f, %.1f°",
            pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Shoot pose", "X %.1f, Y %.1f, %.0f°",
            shoot.getX(), shoot.getY(), shoot.getHeadingDegrees());
        telemetry.addData("Spit (B)", controls.isSpitting() ? "ACTIVE" : "off");
        telemetry.addData("Intake (Y)", intake.isEnabled() ? "ON / " + intake.getDirection() : "OFF");
        telemetry.addData("Gate (X)", "%s %s", gate.isActive() ? "ON" : "OFF",
            gate.isReversed() ? "REVERSE" : "FORWARD");
        telemetry.addData("Gate target / actual", "%.0f / %.0f RPM",
            gate.getTargetRpm(), Math.abs(gate.getActualRpm()));
        telemetry.addData("Gate recovery", "%s (retry %d/%d, total %d)", gate.getRecoveryState(),
            gate.getRecoveryAttempts(), RobotConfig.GateMotor.JAM_MAX_ATTEMPTS, gate.getRecoveryCount());
        if (gate.getRecoveryState() == GateJamRecoveryController.State.FAULT) {
          telemetry.addLine("GATE JAMMED: clear it, then X off and X on to retry.");
        }
        telemetry.addData("Launcher (D-pad up/down) target / actual", "%.0f / %.0f RPM",
            shooter.getTargetRpm(), Math.abs(shooter.getActualRpm()));
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      try {
        if (gate != null) { gate.stop(); }
      } finally {
        try {
          if (intake != null) { intake.stop(); }
        } finally {
          try {
            if (shooter != null) { shooter.stop(); }
          } finally {
            follower.setMaxPower(1.0);
            follower.breakFollowing();
          }
        }
      }
    }
  }

  private void startDriveToShoot(Waypoint shoot) {
    Pose from = follower.getPose();
    Pose to = toPose(shoot);
    follower.breakFollowing();
    follower.setMaxPower(RobotConfig.TeleOpTourney.DRIVE_TO_SHOOT_MAX_POWER);
    if (Math.hypot(to.getX() - from.getX(), to.getY() - from.getY()) < 0.1) {
      follower.holdPoint(to, false);
    } else {
      Path path = new Path(new BezierLine(from, to));
      path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
      follower.followPath(path, true);
    }
    drivingToShoot = true;
    driveToShootStarted = getRuntime();
    driveToShootStatus = "Driving to shoot pose (move sticks to cancel)";
  }

  /** Returns a status message when the automatic move should end, otherwise null. */
  private String driveToShootFinished(Waypoint shoot, boolean driverInput) {
    if (driverInput) { return "Cancelled by driver"; }
    Pose current = follower.getPose();
    double positionError = Math.hypot(shoot.getX() - current.getX(), shoot.getY() - current.getY());
    double angle = Math.toRadians(shoot.getHeadingDegrees()) - current.getHeading();
    double headingError = Math.toDegrees(Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle))));
    if (positionError <= RobotConfig.TeleOpTourney.DRIVE_TO_SHOOT_POSITION_TOLERANCE_INCHES
        && headingError <= RobotConfig.TeleOpTourney.DRIVE_TO_SHOOT_HEADING_TOLERANCE_DEGREES) {
      return "At shoot pose";
    }
    if (getRuntime() - driveToShootStarted >= RobotConfig.TeleOpTourney.DRIVE_TO_SHOOT_TIMEOUT_SECONDS) {
      return String.format(java.util.Locale.US, "Timed out %.1f in / %.0f° away",
          positionError, headingError);
    }
    return null;
  }

  private void endDriveToShoot(String status) {
    drivingToShoot = false;
    driveToShootStatus = status;
    follower.breakFollowing();
    follower.setMaxPower(1.0);
    follower.startTeleopDrive(true);
  }

  private static Waypoint startPose(BiobuzzFieldMap field, Alliance alliance, boolean useAutoPose) {
    if (useAutoPose && AutoPoseHandoff.hasPose()) { return AutoPoseHandoff.getPose(); }
    return field.startPose(new PreloadedHiveAutoRoute(alliance).start());
  }

  /** Final waypoint of the autonomous preload-shot approach: the robot's shooting pose. */
  private static Waypoint shootPose(Alliance alliance) {
    HiveAutoRoute route = new PreloadedHiveAutoRoute(alliance);
    List<Waypoint> approach = route.approach(Stage.PRELOAD_SHOT);
    return approach.get(approach.size() - 1);
  }

  private static Pose toPose(Waypoint p) {
    return new Pose(p.getX(), p.getY(), Math.toRadians(p.getHeadingDegrees()));
  }
}
