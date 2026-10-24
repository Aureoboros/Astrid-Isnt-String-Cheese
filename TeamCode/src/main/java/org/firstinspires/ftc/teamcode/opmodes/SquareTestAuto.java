package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.SquareTestRoutes.Corner;
import org.firstinspires.ftc.teamcode.pedroPathing.SquareTestRoutes.Shape;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/**
 * Drives every {@link Shape} in order from one starting spot and reports how far the robot ends
 * from where it started after each one. Needs at least 4 ft of clear floor ahead and to the left.
 */
@Autonomous(name = "Test: Square Test", group = "Pedro Pathing")
public final class SquareTestAuto extends LinearOpMode {
  private static final double MAX_POWER = 0.35;
  // Diagnostic tolerances allow modest drift while the physical follower is being tuned.
  private static final double STEP_TIMEOUT_SECONDS = 15.0;
  private static final double POSITION_TOLERANCE_INCHES = 1.5;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5.0);
  private static final double SETTLE_SECONDS = 0.1;
  private static final double PAUSE_BETWEEN_SHAPES_SECONDS = 1.0;

  private Follower follower;
  private GoBildaPinpointDriver pinpoint;
  private final List<String> results = new ArrayList<>();
  private String status = "Ready";

  @Override
  public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    telemetry.addLine("Keep still during Pinpoint calibration.");
    telemetry.addLine("Place the robot at the RIGHT corner; shapes go forward, then left.");
    telemetry.addLine("Clear at least 4 ft ahead and 4 ft to the left.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    follower = Constants.createFollower(hardwareMap);
    pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
    follower.setStartingPose(new Pose(0, 0, 0));
    follower.setMaxPower(MAX_POWER);

    try {
      waitForStart();
      if (isStopRequested()) {
        return;
      }
      follower.setPose(new Pose(0, 0, 0));

      for (Shape shape : Shape.values()) {
        if (!runShape(shape)) {
          break;
        }
        status = "Finished " + shape.getLabel();
        if (!pause(PAUSE_BETWEEN_SHAPES_SECONDS)) {
          break;
        }
      }
      if (opModeIsActive() && results.size() == Shape.values().length) {
        status = "All shapes complete";
      }
      follower.breakFollowing();
      while (opModeIsActive()) {
        follower.updatePose();
        showTelemetry(null, 0, 0);
        idle();
      }
    } finally {
      follower.breakFollowing();
    }
  }

  /** Returns false if the run was stopped, timed out, or lost the Pinpoint. */
  private boolean runShape(Shape shape) {
    List<Corner> corners = shape.getCorners();
    double worstCornerError = 0;
    Corner previous = null;
    for (int i = 0; i < corners.size(); i++) {
      Corner target = corners.get(i);
      if (i == 0 && atTarget(target)) {
        previous = target;
        continue;
      }
      if (previous == null || target.samePositionAs(previous)) {
        // Turn in place (or re-align before the shape) by holding the target pose.
        follower.holdPoint(toPose(target));
      } else {
        Pose start = follower.getPose();
        Path path = new Path(new BezierLine(start, toPose(target)));
        path.setConstantHeadingInterpolation(Math.toRadians(target.getHeadingDegrees()));
        follower.followPath(path, true);
      }
      if (!driveUntilSettled(shape, i, corners.size() - 1, target)) {
        follower.breakFollowing();
        results.add(String.format("%s: ABORTED at step %d (%s)", shape.getLabel(), i, status));
        return false;
      }
      worstCornerError = Math.max(worstCornerError, positionError(target));
      previous = target;
    }

    Corner home = corners.get(corners.size() - 1);
    Pose pose = follower.getPose();
    results.add(String.format(
        "%s: ends %.2f in / %.1f° from start; worst corner %.2f in; at X %.2f, Y %.2f",
        shape.getLabel(), positionError(home), Math.toDegrees(headingError(home)),
        worstCornerError, pose.getX(), pose.getY()));
    return true;
  }

  private boolean driveUntilSettled(Shape shape, int step, int lastStep, Corner target) {
    ElapsedTime timer = new ElapsedTime();
    double settledSince = -1;
    while (opModeIsActive()) {
      follower.update();
      if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
        status = "Pinpoint fault: stopped; check status and pod wiring";
        return false;
      }
      double elapsed = timer.seconds();
      if (!atTarget(target)) {
        settledSince = -1;
      } else if (settledSince < 0) {
        settledSince = elapsed;
      } else if (elapsed - settledSince >= SETTLE_SECONDS) {
        return true;
      }
      if (elapsed >= STEP_TIMEOUT_SECONDS) {
        status = "Timed out";
        return false;
      }
      status = String.format("Running %s, step %d / %d", shape.getLabel(), step, lastStep);
      showTelemetry(target, step, lastStep);
      idle();
    }
    status = "Stopped";
    return false;
  }

  /** Keeps holding the current target while waiting; returns false if the OpMode stops. */
  private boolean pause(double seconds) {
    ElapsedTime timer = new ElapsedTime();
    while (opModeIsActive() && timer.seconds() < seconds) {
      follower.update();
      showTelemetry(null, 0, 0);
      idle();
    }
    return opModeIsActive();
  }

  private void showTelemetry(Corner target, int step, int lastStep) {
    Pose pose = follower.getPose();
    telemetry.addData("Status", status);
    telemetry.addData("Pose", "X %.2f, Y %.2f in; heading %.1f deg",
        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    if (target != null) {
      telemetry.addData("Target", "X %.2f, Y %.2f in; heading %.1f deg",
          target.getX(), target.getY(), target.getHeadingDegrees());
      telemetry.addData("Target Error", "%.2f in, %.1f deg",
          positionError(target), Math.toDegrees(headingError(target)));
    }
    telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
    telemetry.addData("Motor Power Limit", "%.0f%%", MAX_POWER * 100);
    telemetry.addData("Corner Tolerance", "%.1f in / %.1f deg for %.2f s",
        POSITION_TOLERANCE_INCHES, Math.toDegrees(HEADING_TOLERANCE_RADIANS), SETTLE_SECONDS);
    telemetry.addData("Step Timeout", "%.0f s", STEP_TIMEOUT_SECONDS);
    for (String result : results) {
      telemetry.addLine(result);
    }
    OdometryTelemetry.add(telemetry);
    telemetry.update();
  }

  private boolean atTarget(Corner target) {
    return positionError(target) <= POSITION_TOLERANCE_INCHES
        && Math.abs(headingError(target)) <= HEADING_TOLERANCE_RADIANS;
  }

  private double positionError(Corner target) {
    Pose pose = follower.getPose();
    return Math.hypot(target.getX() - pose.getX(), target.getY() - pose.getY());
  }

  private double headingError(Corner target) {
    double error = Math.toRadians(target.getHeadingDegrees()) - follower.getPose().getHeading();
    return Math.atan2(Math.sin(error), Math.cos(error));
  }

  private static Pose toPose(Corner corner) {
    return new Pose(corner.getX(), corner.getY(), Math.toRadians(corner.getHeadingDegrees()));
  }
}
