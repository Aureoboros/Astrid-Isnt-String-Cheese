package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.PIDController;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;

/** Converts a field-relative pose error into a bounded robot-relative drive command. */
public final class HolonomicPoseController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController headingController;
  private final double maxDrivePower;
  private final double maxTurnPower;

  public static HolonomicPoseController createDefault() {
    PIDController xController =
        new PIDController(
            RobotConfig.Autonomous.TRANSLATION_KP,
            RobotConfig.Autonomous.TRANSLATION_KI,
            RobotConfig.Autonomous.TRANSLATION_KD);
    PIDController yController =
        new PIDController(
            RobotConfig.Autonomous.TRANSLATION_KP,
            RobotConfig.Autonomous.TRANSLATION_KI,
            RobotConfig.Autonomous.TRANSLATION_KD);
    PIDController headingController =
        new PIDController(
            RobotConfig.Autonomous.HEADING_KP,
            RobotConfig.Autonomous.HEADING_KI,
            RobotConfig.Autonomous.HEADING_KD);

    xController.setMaxIntegral(RobotConfig.Autonomous.TRANSLATION_MAX_INTEGRAL);
    yController.setMaxIntegral(RobotConfig.Autonomous.TRANSLATION_MAX_INTEGRAL);
    headingController.setMaxIntegral(RobotConfig.Autonomous.HEADING_MAX_INTEGRAL);
    xController.setDeadband(RobotConfig.Autonomous.TRANSLATION_DEADBAND);
    yController.setDeadband(RobotConfig.Autonomous.TRANSLATION_DEADBAND);
    headingController.setDeadband(RobotConfig.Autonomous.HEADING_DEADBAND);

    return new HolonomicPoseController(
        xController,
        yController,
        headingController,
        RobotConfig.Autonomous.MAX_DRIVE_POWER,
        RobotConfig.Autonomous.MAX_TURN_POWER);
  }

  public HolonomicPoseController(
      PIDController xController,
      PIDController yController,
      PIDController headingController,
      double maxDrivePower,
      double maxTurnPower) {
    this.xController = xController;
    this.yController = yController;
    this.headingController = headingController;
    this.maxDrivePower = maxDrivePower;
    this.maxTurnPower = maxTurnPower;
  }

  public void reset() {
    xController.reset();
    yController.reset();
    headingController.reset();
  }

  public PoseControlOutput calculate(PoseEstimate currentPose, Waypoint target) {
    double errorX = target.getX() - currentPose.getX();
    double errorY = target.getY() - currentPose.getY();
    double headingError =
        normalizeAngle(target.getHeadingDegrees() - currentPose.getHeadingDegrees());

    double fieldXPower = xController.calculate(errorX);
    double fieldYPower = yController.calculate(errorY);
    double turnPower = headingController.calculate(headingError);

    double headingRadians = Math.toRadians(currentPose.getHeadingDegrees());
    double robotDrive =
        fieldXPower * Math.cos(headingRadians) + fieldYPower * Math.sin(headingRadians);
    double robotStrafe =
        -fieldXPower * Math.sin(headingRadians) + fieldYPower * Math.cos(headingRadians);

    return new PoseControlOutput(
        clip(robotDrive, -maxDrivePower, maxDrivePower),
        clip(robotStrafe, -maxDrivePower, maxDrivePower),
        clip(turnPower, -maxTurnPower, maxTurnPower),
        Math.hypot(errorX, errorY),
        headingError);
  }

  private static double normalizeAngle(double degrees) {
    while (degrees > 180) {
      degrees -= 360;
    }
    while (degrees < -180) {
      degrees += 360;
    }
    return degrees;
  }

  private static double clip(double value, double minimum, double maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }
}
