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
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.AutonomousStartControls;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzAutoRoutes;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.telemetry.FieldLocationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Select and confirm a BIOBUZZ field start during INIT, then execute named field destinations. */
@Autonomous(name = "BIOBUZZ: Field Location Auto", group = "Competition")
public final class BiobuzzFieldAuto extends LinearOpMode {
  private static final double MAX_POWER = 0.25;
  private static final double LEG_TIMEOUT_SECONDS = 15.0;
  private static final double AUTO_BUDGET_SECONDS = 28.0;
  private static final double POSITION_TOLERANCE_INCHES = 1.5;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5);
  private static final double SETTLE_SECONDS = 0.1;

  private Follower follower;
  private GoBildaPinpointDriver pinpoint;
  private final ElapsedTime autoClock = new ElapsedTime();
  private String status = "INIT: confirm physical placement with A";
  private String targetName;
  private Waypoint target;

  @Override
  public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    telemetry.addLine("BIOBUZZ TU06: keep robot still while Pinpoint calibrates.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    BiobuzzFieldMap field = new BiobuzzFieldMap(
        RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
    // Register additional team destinations here; program their ordered names in BiobuzzAutoRoutes.
    AutonomousStartControls selection = new AutonomousStartControls();
    follower = Constants.createFollower(hardwareMap);
    pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
    follower.setMaxPower(MAX_POWER);
    follower.setStartingPose(toPose(field.startPose(selection.getStart())));

    try {
      while (opModeInInit()) {
        boolean changed = selection.update(gamepad1.dpad_left || gamepad2.dpad_left,
            gamepad1.dpad_right || gamepad2.dpad_right, gamepad1.dpad_up || gamepad2.dpad_up,
            gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.left_bumper || gamepad2.left_bumper,
            gamepad1.right_bumper || gamepad2.right_bumper, gamepad1.a || gamepad2.a);
        if (changed) {
          // Assign the field coordinate origin; never recalibrate the gyro in the INIT loop.
          follower.setPose(toPose(field.startPose(selection.getStart())));
        }
        follower.updatePose();
        status = selection.isConfirmed() ? "INIT: confirmed; keep robot motionless until Play"
            : "INIT: confirm physical placement with A";
        showSelection(field, selection);
        showTelemetry(true);
        idle();
      }
      if (isStopRequested()) {
        return;
      }
      // Selection is now frozen; gamepad inputs cannot change field origin during autonomous.
      autoClock.reset();
      Waypoint start = field.startPose(selection.getStart());
      if (!selection.isConfirmed()) {
        status = "NO MOVEMENT: start placement was not confirmed with A during INIT";
      } else if (!field.footprintInsideField(start)) {
        status = "NO MOVEMENT: configured starting footprint is outside the field";
      } else {
        follower.setPose(toPose(start));
        follower.updatePose();
        List<String> route = BiobuzzAutoRoutes.destinations(selection.getStart(), selection.getPlan());
        boolean completed = validateRoute(field, route);
        for (String locationName : completed ? route : java.util.Collections.<String>emptyList()) {
          targetName = locationName;
          target = field.getLocation(locationName);
          if (!moveToLocation(targetName, target)) {
            completed = false;
            break;
          }
        }
        if (completed) {
          status = route.isEmpty() ? "POSE ONLY: selected field origin active; no movement programmed"
              : "Programmed route complete; motors stopped";
        }
      }
      follower.breakFollowing();
      while (opModeIsActive() && autoClock.seconds() < AUTO_BUDGET_SECONDS) {
        follower.updatePose();
        showSelection(field, selection);
        showTelemetry(true);
        idle();
      }
    } finally {
      follower.breakFollowing();
    }
  }

  /** Resolve and bound-check every programmed endpoint before executing the first leg. */
  private boolean validateRoute(BiobuzzFieldMap field, List<String> route) {
    for (String name : route) {
      try {
        if (!field.footprintInsideField(field.getLocation(name))) {
          status = "NO MOVEMENT: target footprint outside field: " + name;
          return false;
        }
      } catch (IllegalArgumentException e) {
        status = "NO MOVEMENT: " + e.getMessage();
        return false;
      }
    }
    return true;
  }

  /** Every target is an absolute field pose resolved by name, not an encoder-distance move. */
  private boolean moveToLocation(String name, Waypoint destination) {
    if (!canContinue()) {
      return false;
    }
    Pose start = follower.getPose();
    Path path = new Path(new BezierLine(start, toPose(destination)));
    path.setLinearHeadingInterpolation(start.getHeading(), Math.toRadians(destination.getHeadingDegrees()));
    follower.followPath(path, true);
    ElapsedTime legClock = new ElapsedTime();
    double settledSince = -1;
    try {
      while (opModeIsActive()) {
        if (!canContinue()) {
          return false;
        }
        follower.update();
        if (!sensorReady()) {
          status = "ABORTED: Pinpoint fault / invalid pose";
          return false;
        }
        Pose pose = follower.getPose();
        double positionError = Math.hypot(destination.getX() - pose.getX(), destination.getY() - pose.getY());
        double angle = Math.toRadians(destination.getHeadingDegrees()) - pose.getHeading();
        double headingError = Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle)));
        double elapsed = legClock.seconds();
        if (positionError > POSITION_TOLERANCE_INCHES || headingError > HEADING_TOLERANCE_RADIANS) {
          settledSince = -1;
        } else if (settledSince < 0) {
          settledSince = elapsed;
        } else if (elapsed - settledSince >= SETTLE_SECONDS) {
          return true;
        }
        if (elapsed >= LEG_TIMEOUT_SECONDS) {
          status = "ABORTED: 15-second timeout at " + name;
          return false;
        }
        status = "Moving to " + name;
        telemetry.addData("Leg Time", "%.1f / %.1f s", elapsed, LEG_TIMEOUT_SECONDS);
        showTelemetry(false);
        idle();
      }
      status = "Stopped";
      return false;
    } finally {
      follower.breakFollowing();
    }
  }

  private boolean canContinue() {
    if (!opModeIsActive() || autoClock.seconds() >= AUTO_BUDGET_SECONDS) {
      status = "Stopped / autonomous time budget exhausted";
      return false;
    }
    if (gamepad1.b || gamepad2.b) {
      status = "ABORTED by B";
      return false;
    }
    if (!sensorReady()) {
      status = "ABORTED: Pinpoint must be READY with a finite pose";
      return false;
    }
    return true;
  }

  private boolean sensorReady() {
    Pose pose = follower.getPose();
    return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
        && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getHeading());
  }

  private void showSelection(BiobuzzFieldMap field, AutonomousStartControls selection) {
    Waypoint start = field.startPose(selection.getStart());
    telemetry.addData("Alliance / Start", "%s / %s", selection.getAlliance(), selection.getStart());
    telemetry.addData("Selected Start Pose", "X %.1f, Y %.1f in; heading %.0f deg",
        start.getX(), start.getY(), start.getHeadingDegrees());
    telemetry.addData("Plan", selection.getPlan());
    telemetry.addData("Placement Confirmed", selection.isConfirmed());
    telemetry.addData("Starting Footprint", "%.1f x %.1f in (length x width)",
        RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
    if (opModeInInit()) {
      telemetry.addLine("D-pad Left/Right: red/blue | Up/Down: start | Bumpers: plan");
      telemetry.addLine("A: confirm exact wall placement + heading | B during AUTO: abort");
      telemetry.addLine("POSE_ONLY is default. Check field obstacles and loading zones before enabling movement.");
      telemetry.addLine("Enable the Driver Station 30-second AUTO timer (G305).");
    }
  }

  private void showTelemetry(boolean matrix) {
    telemetry.addData("Status", status);
    telemetry.addData("Max Motor Power", "%.0f%%", MAX_POWER * 100);
    FieldLocationTelemetry.add(telemetry, follower.getPose(), pinpoint, targetName, target, matrix);
    telemetry.update();
  }

  private static Pose toPose(Waypoint waypoint) {
    return new Pose(waypoint.getX(), waypoint.getY(), Math.toRadians(waypoint.getHeadingDegrees()));
  }
}
