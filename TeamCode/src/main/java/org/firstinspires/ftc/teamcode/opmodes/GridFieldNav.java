package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.navigation.GridField;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * 6 x 6 grid navigation. During INIT the robot reads AprilTag 40 (grid point 2,1) to find where it
 * is. After Play, the D-pad picks a grid point (up/down = Y, right/left = X), A drives there in a
 * straight line, then the robot turns to face the field center (3, 3). B cancels a move.
 */
@TeleOp(name = "Grid Field Nav", group = "Pedro Pathing")
public final class GridFieldNav extends LinearOpMode {
  /**
   * Field heading the robot has while it looks at tag 40. 90 degrees means the robot faces +Y, so
   * it starts below the tag (smaller Y). Change this if the robot starts facing the tag from a
   * different side.
   */
  private static final double START_HEADING_DEGREES = 90;
  private static final int TAG_SAMPLES = 10;
  private static final double MAX_POWER = 0.5;
  private static final double POSITION_TOLERANCE_INCHES = 1.5;
  private static final double HEADING_TOLERANCE_DEGREES = 2;
  private static final double SETTLE_SECONDS = 0.15;
  private static final double DRIVE_TIMEOUT_SECONDS = 8;
  private static final double TURN_TIMEOUT_SECONDS = 4;

  private Follower follower;
  private AprilTagTracker tags;
  private int targetX;
  private int targetY;
  private boolean targetChosen;
  private boolean lastUp, lastDown, lastLeft, lastRight;
  private String status = "";

  @Override
  public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    double startHeading = Math.toRadians(START_HEADING_DEGREES);
    follower = Constants.createFollower(hardwareMap);
    follower.setMaxPower(MAX_POWER);
    tags = new AprilTagTracker(hardwareMap);
    tags.start();

    double sumX = 0;
    double sumY = 0;
    double[] xs = new double[TAG_SAMPLES];
    double[] ys = new double[TAG_SAMPLES];
    int samples = 0;
    double lastFrame = Double.NaN;

    try {
      // Locate the robot from tag 40 while waiting for Play.
      while (opModeInInit() || (opModeIsActive() && samples == 0)) {
        tags.update();
        AprilTagObservation tag = tags.getObservation(GridField.ANCHOR_TAG_ID);
        double frame = tags.getResultTimestamp();
        if (tag != null && frame != lastFrame) {
          lastFrame = frame;
          double[] p = GridField.robotPositionFromTag(
              tag.getForwardInches(), tag.getLeftInches(), startHeading);
          int slot = samples % TAG_SAMPLES;
          if (samples >= TAG_SAMPLES) {
            sumX -= xs[slot];
            sumY -= ys[slot];
          }
          xs[slot] = p[0];
          ys[slot] = p[1];
          sumX += p[0];
          sumY += p[1];
          samples++;
          int n = Math.min(samples, TAG_SAMPLES);
          Pose start = new Pose(sumX / n, sumY / n, startHeading);
          follower.setStartingPose(start);
          follower.setPose(start);
          if (!targetChosen) {
            // Until the driver picks a point, start the selection where the robot already is.
            targetX = GridField.nearestGrid(start.getX());
            targetY = GridField.nearestGrid(start.getY());
          }
        }
        follower.updatePose();
        handleDpad();
        status = samples == 0
            ? "Looking for AprilTag " + GridField.ANCHOR_TAG_ID + " - point the robot at it"
            : "Tag " + GridField.ANCHOR_TAG_ID + " locked (" + Math.min(samples, TAG_SAMPLES)
                + " frames). Press Play.";
        show(tag != null);
        idle();
      }
      if (isStopRequested()) {
        return;
      }

      status = "D-pad to pick a point, A to go";

      boolean lastA = false;
      while (opModeIsActive()) {
        follower.update();
        tags.update();
        handleDpad();
        boolean a = gamepad1.a;
        if (a && !lastA) {
          goTo(targetX, targetY);
        }
        lastA = a;
        show(tags.getObservation(GridField.ANCHOR_TAG_ID) != null);
        idle();
      }
    } finally {
      follower.breakFollowing();
      tags.stop();
    }
  }

  /** Straight line to the grid point keeping the current heading, then turn to face (3, 3). */
  private void goTo(int gridX, int gridY) {
    Pose from = follower.getPose();
    double x = GridField.toInches(gridX);
    double y = GridField.toInches(gridY);
    double heading = from.getHeading();

    if (Math.hypot(x - from.getX(), y - from.getY()) > POSITION_TOLERANCE_INCHES) {
      Path path = new Path(new BezierLine(from, new Pose(x, y, heading)));
      path.setConstantHeadingInterpolation(heading);
      follower.followPath(path, true);
      status = String.format("Driving to (%d, %d)", gridX, gridY);
      if (!waitUntilSettled(x, y, heading, DRIVE_TIMEOUT_SECONDS)) {
        return;
      }
    }

    double faceCenter = GridField.headingToCenter(x, y, heading);
    follower.holdPoint(new Pose(x, y, faceCenter), false);
    status = String.format("Turning to face center (%.0f deg)", Math.toDegrees(faceCenter));
    if (waitUntilSettled(x, y, faceCenter, TURN_TIMEOUT_SECONDS)) {
      status = String.format("Arrived at (%d, %d), facing center. D-pad + A for next point",
          gridX, gridY);
    }
    // Keep holding the final pose so the robot doesn't drift while the next point is chosen.
    follower.holdPoint(new Pose(x, y, faceCenter), false);
  }

  private boolean waitUntilSettled(double x, double y, double heading, double timeoutSeconds) {
    ElapsedTime timer = new ElapsedTime();
    double settledAt = -1;
    while (opModeIsActive()) {
      follower.update();
      tags.update();
      if (gamepad1.b) {
        follower.breakFollowing();
        status = "Cancelled with B";
        return false;
      }
      Pose pose = follower.getPose();
      boolean atPose = Math.hypot(x - pose.getX(), y - pose.getY()) <= POSITION_TOLERANCE_INCHES
          && Math.abs(GridField.angleError(heading, pose.getHeading()))
              <= Math.toRadians(HEADING_TOLERANCE_DEGREES);
      double now = timer.seconds();
      if (!atPose) {
        settledAt = -1;
      } else if (settledAt < 0) {
        settledAt = now;
      } else if (now - settledAt >= SETTLE_SECONDS) {
        return true;
      }
      if (now >= timeoutSeconds) {
        follower.breakFollowing();
        status = "Timed out - check tuning / path";
        return false;
      }
      show(tags.getObservation(GridField.ANCHOR_TAG_ID) != null);
      idle();
    }
    return false;
  }

  /** Rising-edge D-pad: up/down change Y, right/left change X, clamped to 0..6. */
  private void handleDpad() {
    boolean up = gamepad1.dpad_up;
    boolean down = gamepad1.dpad_down;
    boolean left = gamepad1.dpad_left;
    boolean right = gamepad1.dpad_right;
    if (up && !lastUp) targetY = GridField.clamp(targetY + 1);
    if (down && !lastDown) targetY = GridField.clamp(targetY - 1);
    if (right && !lastRight) targetX = GridField.clamp(targetX + 1);
    if (left && !lastLeft) targetX = GridField.clamp(targetX - 1);
    targetChosen |= (up && !lastUp) || (down && !lastDown) || (right && !lastRight)
        || (left && !lastLeft);
    lastUp = up;
    lastDown = down;
    lastLeft = left;
    lastRight = right;
  }

  private void show(boolean tagVisible) {
    Pose pose = follower.getPose();
    telemetry.addData("Status", status);
    telemetry.addData("Tag " + GridField.ANCHOR_TAG_ID, tagVisible ? "visible" : "not visible");
    telemetry.addData("Robot (grid)", "X %.2f, Y %.2f",
        GridField.toGrid(pose.getX()), GridField.toGrid(pose.getY()));
    telemetry.addData("Robot (in)", "X %.1f, Y %.1f, heading %.1f deg",
        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    telemetry.addData("Target (grid)", "(%d, %d)", targetX, targetY);
    telemetry.addLine("D-pad: up/down = Y, right/left = X | A = go | B = cancel");
    telemetry.update();
  }
}
