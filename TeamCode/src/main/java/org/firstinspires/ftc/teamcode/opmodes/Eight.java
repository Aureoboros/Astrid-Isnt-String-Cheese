import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.AutonomousStartControls;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.HiveTagRelocalizer;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzAutoRoutes;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.telemetry.FieldLocationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Select and confirm a BIOBUZZ field start during INIT, then execute named field destinations. */
/**
 * Select and confirm a BIOBUZZ field start during INIT, then execute named field destinations.
 * Plan HIVE_CHECK_AND_PARK drives in front of the own raised hive cell, fixes X/Y from its
 * AprilTags with the Limelight (tag map in HiveAutoConfig), then parks using the corrected pose.
 */
@Autonomous(name = "8", group = "Competition")
public final class Eight extends LinearOpMode {
  private static final double MAX_POWER = 0.25;

  private Follower follower;
  private GoBildaPinpointDriver pinpoint;
  private AprilTagTracker tags;
  private String fixStatus = "not used by this plan";
  private final ElapsedTime autoClock = new ElapsedTime();
  private String status = "INIT: confirm physical placement with A";
  private String targetName;
    pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
    follower.setMaxPower(MAX_POWER);
    follower.setStartingPose(toPose(field.startPose(selection.getStart())));
    try {
      // Read-only camera; optional so POSE_ONLY / LEAVE_WALL still run without a Limelight.
      tags = new AprilTagTracker(hardwareMap);
      tags.start();
    } catch (RuntimeException missing) {
      tags = null;
    }

    try {
      while (opModeInInit()) {
            completed = false;
            break;
          }
          if (BiobuzzAutoRoutes.isHiveCheck(locationName)) {
            // Stopped where it THINKS the check pose is: fix X/Y from the hive tags, then drive
            // out the remaining error. Every later leg (park) uses the corrected position.
            if (!relocalizeAtHive(selection.getAlliance()) || !moveToLocation(targetName, target)) {
              completed = false;
              break;
            }
          }
        }
        if (completed) {
          status = route.isEmpty() ? "POSE ONLY: selected field origin active; no movement programmed"
      }
    } finally {
      follower.breakFollowing();
      if (tags != null) {
        tags.stop();
      }
    }
  }

  /**
   * Holds the check pose, averages fresh Limelight frames of the measured hive tags, and if they
   * agree overwrites Pedro/Pinpoint X/Y (heading stays on the IMU). Any vision problem leaves
   * odometry unchanged and continues; returns false only for STOP / B / time / Pinpoint faults.
   */
  private boolean relocalizeAtHive(Alliance alliance) {
    HiveTagRelocalizer fix = new HiveTagRelocalizer(
        alliance == Alliance.RED ? HiveAutoConfig.RED_HIVE_TAGS : HiveAutoConfig.BLUE_HIVE_TAGS,
        HiveAutoConfig.RELOCALIZE_FRAMES, HiveAutoConfig.RELOCALIZE_MAX_SPREAD_INCHES,
        HiveAutoConfig.RELOCALIZE_MAX_CORRECTION_INCHES);
    if (tags == null || !tags.isConnected()) {
      fixStatus = "SKIPPED: Limelight missing or disconnected; using odometry";
      return true;
    }
    if (!fix.hasTagPositions()) {
      fixStatus = "SKIPPED: hive tag positions not measured (HiveAutoConfig); using odometry";
      return true;
    }
    follower.holdPoint(toPose(target), false);
    ElapsedTime checkClock = new ElapsedTime();
    try {
      while (fix.getFrameCount() < HiveAutoConfig.RELOCALIZE_FRAMES
          && checkClock.seconds() < HiveAutoConfig.RELOCALIZE_TIMEOUT_SECONDS) {
        if (!canContinue()) {
          return false;
        }
        follower.update();
        tags.update();
        Pose pose = follower.getPose();
        fix.addFrame(tags.getResultTimestamp(), pose.getX(), pose.getY(), pose.getHeading(),
            tags.getObservations());
        status = "Limelight position check in front of hive";
        fixStatus = "reading hive tags: " + fix.getFrameCount() + " / " + HiveAutoConfig.RELOCALIZE_FRAMES;
        showTelemetry(false);
        idle();
      }
    } finally {
      follower.breakFollowing();
    }
    if (fix.getStatus() != HiveTagRelocalizer.Status.OK) {
      fixStatus = String.format(Locale.US,
          "NOT APPLIED (%s, %d frames, spread %.1f in, offset %.1f in): using odometry",
          fix.getStatus(), fix.getFrameCount(), fix.getSpreadInches(), fix.getCorrectionInches());
      return true;
    }
    Pose pose = follower.getPose();
    double[] corrected = fix.corrected(pose.getX(), pose.getY());
    double[] offset = fix.getOffset();
    follower.setPose(new Pose(corrected[0], corrected[1], pose.getHeading()));
    fixStatus = String.format(Locale.US,
        "APPLIED: dX %+.1f, dY %+.1f in (tags %s, %d frames, spread %.1f in)",
        offset[0], offset[1], fix.getUsedTagIds(), fix.getFrameCount(), fix.getSpreadInches());
    return true;
  }

  /** Resolve and bound-check every programmed endpoint before executing the first leg. */
  private boolean validateRoute(BiobuzzFieldMap field, List<String> route) {
    for (String name : route) {
      return false;
    }
    Pose start = follower.getPose();
    Path path = new Path(new BezierLine(start, toPose(destination)));
    path.setLinearHeadingInterpolation(start.getHeading(), Math.toRadians(destination.getHeadingDegrees()));
    follower.followPath(path, true);
    if (Math.hypot(destination.getX() - start.getX(), destination.getY() - start.getY()) < 0.1) {
      // Already there (e.g. a tiny Limelight correction): hold instead of a zero-length path.
      follower.holdPoint(toPose(destination), false);
    } else {
      Path path = new Path(new BezierLine(start, toPose(destination)));
      path.setLinearHeadingInterpolation(start.getHeading(),
          Math.toRadians(destination.getHeadingDegrees()));
      follower.followPath(path, true);
    }
    ElapsedTime legClock = new ElapsedTime();
    double settledSince = -1;
    try {
        RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
    if (opModeInInit()) {
      telemetry.addLine("D-pad Left/Right: red/blue | Up/Down: start | Bumpers: plan");
      telemetry.addLine("Plans: POSE_ONLY | LEAVE_WALL_12_INCHES | HIVE_CHECK_AND_PARK (Limelight)");
      telemetry.addLine("A: confirm exact wall placement + heading | B during AUTO: abort");
      telemetry.addLine("POSE_ONLY is default. Check field obstacles and loading zones before enabling movement.");
      telemetry.addLine("Enable the Driver Station 30-second AUTO timer (G305).");
  private void showTelemetry(boolean matrix) {
    telemetry.addData("Status", status);
    telemetry.addData("Max Motor Power", "%.0f%%", MAX_POWER * 100);
    telemetry.addData("Limelight Position Fix", fixStatus);
    if (tags != null) {
      telemetry.addData("Limelight Connected / Visible Tags", "%s / %d",
          tags.isConnected(), tags.getObservations().size());
    } else {
      telemetry.addData("Limelight", "not configured (only needed for HIVE_CHECK_AND_PARK)");
    }
    FieldLocationTelemetry.add(telemetry, follower.getPose(), pinpoint, targetName, target, matrix);
    telemetry.update();
  }
