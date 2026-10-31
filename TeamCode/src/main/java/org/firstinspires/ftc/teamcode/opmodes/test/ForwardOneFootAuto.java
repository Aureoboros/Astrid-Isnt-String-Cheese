package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Drives straight forward one foot (12 inches) from the start pose using Pinpoint odometry. */
@Autonomous(name = "Forward One Foot Auto", group = "Pedro Pathing")
public final class ForwardOneFootAuto extends LinearOpMode {
  private static final double DISTANCE_INCHES = 12.0;
  private static final double MAX_POWER = 0.35;
  private static final double TIMEOUT_SECONDS = 10.0;

  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    Pose start = new Pose(0, 0, 0);
    Pose end = new Pose(DISTANCE_INCHES, 0, 0);
    follower.setStartingPose(start);
    follower.setMaxPower(MAX_POWER);
    try {
      telemetry.addLine("Forward one foot (12 in) at 35% power. Clear space in front of the robot.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) {
        return;
      }
      follower.setPose(start);
      Path path = new Path(new BezierLine(start, end));
      path.setConstantHeadingInterpolation(0);
      follower.followPath(path, false);
      ElapsedTime timer = new ElapsedTime();
      while (opModeIsActive() && follower.isBusy() && timer.seconds() < TIMEOUT_SECONDS) {
        follower.update();
        Pose pose = follower.getPose();
        telemetry.addData("Target", "%.1f in forward", DISTANCE_INCHES);
        telemetry.addData("Pose", "X %.2f, Y %.2f in; heading %.1f deg",
            pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
      telemetry.addData("Status", follower.isBusy() ? "Timed out or stopped" : "Done");
      telemetry.update();
    } finally {
      follower.breakFollowing();
    }
  }
}
