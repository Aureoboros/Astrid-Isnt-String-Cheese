import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.HiveShotController;
import org.firstinspires.ftc.teamcode.control.LimelightPanController;
import org.firstinspires.ftc.teamcode.control.PollenBatchInventory;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.HiveTagRelocalizer;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.AutonomousStartControls;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzAutoRoutes;
import org.firstinspires.ftc.teamcode.localization.HiveTipDetector;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.CameraPanTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.FieldLocationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/**
 * Restored field-location autonomous, listed as "8" on the Driver Station.
 * Uses the existing POSE_ONLY and LEAVE_WALL_12_INCHES plans. The original hive-position
 * fix helper is retained, but no HIVE_CHECK_AND_PARK route is defined in BiobuzzAutoRoutes.
 * Listed as "8". Hive route (see doc/BIOBUZZ_HIVE_AUTO.md):
 * hive side 1 (Limelight position fix, shoot 4 preloads) -> flower 1 -> hive side 2 (shoot 4)
 * -> flower 2 -> hive side 2 (shoot 4, confirm tip) -> park. No garden intake.
 * HiveAutoConfig commissioning gates (ROUTE_VERIFIED, DRY_RUN, ...) still apply.
 */
@Autonomous(name = "8", group = "Competition")
public final class Eight extends LinearOpMode {
  private static final double MAX_POWER = 0.25;
  private static final double LEG_TIMEOUT_SECONDS = 15.0;
  private static final double AUTO_BUDGET_SECONDS = 28.0;
  private static final double POSITION_TOLERANCE_INCHES = 1.5;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5);
  private static final double SETTLE_SECONDS = 0.1;
  /** Ordered stages. Side 1 = initially raised cell; side 2 = opposite cell. */
  enum Stage {
    SIDE_1_PRELOAD_SHOT(false, false, true),
    FLOWER_1_PICKUP(true, false, false),
    SIDE_2_FIRST_SHOT(false, false, false),
    FLOWER_2_PICKUP(true, false, false),
    SIDE_2_SECOND_SHOT(false, true, false);
    final boolean pickup, requireTip, side1;
    Stage(boolean pickup, boolean requireTip, boolean side1) {
      this.pickup = pickup; this.requireTip = requireTip; this.side1 = side1;
    }
  }

  private Follower follower;
  private GoBildaPinpointDriver pinpoint;
  private AprilTagTracker tags;
  private String fixStatus = "not used by the currently defined plans";
  private final ElapsedTime autoClock = new ElapsedTime();
  private String status = "INIT: confirm physical placement with A";
  private String targetName;
  private LimelightPanController pan;
  private ShooterSubsystem shooter;
  private DcMotor intakeFeeder;
  private DcMotor pickupIntake;
  private final ElapsedTime clock = new ElapsedTime();
  private String status = "INIT";
  private Waypoint target;
  private int confirmedTips;
  private boolean movementFault;
  private final PollenBatchInventory inventory = new PollenBatchInventory();
  private Stage currentStage;
  private String fixStatus = HiveAutoConfig.RELOCALIZE_AT_HIVE ? "pending" : "disabled in HiveAutoConfig";

  @Override
  public void runOpMode() {
  @Override public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    telemetry.addLine("BIOBUZZ TU06: keep robot still while Pinpoint calibrates.");
    telemetry.addLine("Keep robot still during Pinpoint calibration.");
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

    Alliance alliance = Alliance.RED;
    HiveAutoRoute route = new HiveAutoRoute(alliance);
    boolean confirmed = false, oldA = false;
    try {
      // Optional, read-only camera. Missing camera must not prevent the existing pose/wall plans.
      try {
      follower = Constants.createFollower(hardwareMap);
      pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
      follower.setMaxPower(HiveAutoConfig.MAX_DRIVE_POWER);
      follower.setStartingPose(pose(field.startPose(route.start())));
      if (!HiveAutoConfig.DRY_RUN && HiveAutoConfig.canRun()) {
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.setTargetRpms(HiveAutoConfig.SHOOT_RPM_1, HiveAutoConfig.SHOOT_RPM_2);
        pickupIntake = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.INTAKE);
        pickupIntake.setPower(0);
        if (HiveAutoConfig.FEEDER == HiveAutoConfig.Feeder.INTAKE) {
          intakeFeeder = pickupIntake;
        }
        setFeed(false);
        tags = new AprilTagTracker(hardwareMap);
        pan = new LimelightPanController(hardwareMap, tags);
        tags.start();
      }
      if (tags == null && HiveAutoConfig.RELOCALIZE_AT_HIVE && HiveAutoConfig.canRun()) {
        // Read-only camera use; allowed in DRY_RUN so the position fix can be commissioned first.
        tags = new AprilTagTracker(hardwareMap);
        tags.start();
      } catch (RuntimeException missingCamera) {
        if (tags != null) { tags.stop(); }
        tags = null;
      }
      while (opModeInInit()) {
        boolean changed = selection.update(gamepad1.dpad_left || gamepad2.dpad_left,
            gamepad1.dpad_right || gamepad2.dpad_right, gamepad1.dpad_up || gamepad2.dpad_up,
            gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.left_bumper || gamepad2.left_bumper,
            gamepad1.right_bumper || gamepad2.right_bumper, gamepad1.a || gamepad2.a);
        if (changed) {
          // Assign the field coordinate origin; never recalibrate the gyro in the INIT loop.
          follower.setPose(toPose(field.startPose(selection.getStart())));
        Alliance next = (gamepad1.dpad_right || gamepad2.dpad_right) ? Alliance.BLUE
            : (gamepad1.dpad_left || gamepad2.dpad_left) ? Alliance.RED : alliance;
        boolean a = gamepad1.a || gamepad2.a;
        if (next != alliance) {
          alliance = next;
          route = new HiveAutoRoute(alliance);
          confirmed = false;
          follower.setPose(pose(field.startPose(route.start())));
        } else if (a && !oldA) {
          confirmed = true;
          follower.setPose(pose(field.startPose(route.start())));
        }
        oldA = a;
        follower.updatePose();
        if (tags != null) { tags.update(); }
        status = selection.isConfirmed() ? "INIT: confirmed; keep robot motionless until Play"
            : "INIT: confirm physical placement with A";
        showSelection(field, selection);
        showTelemetry(true);
        telemetry.addData("Alliance / Required Start", "%s / %s", alliance, route.start());
        telemetry.addData("Placement Confirmed", confirmed);
        telemetry.addData("Calibration Allows Run", HiveAutoConfig.canRun());
        telemetry.addData("Mode", HiveAutoConfig.DRY_RUN ? "DRY RUN: no shooting, no tip claims" : "LIVE");
        telemetry.addLine("Side 1: fix + 4 preloads -> flower 1 -> side 2 -> flower 2 -> side 2 / tip -> park");
        telemetry.addLine("D-pad Left/Right: alliance | A: confirm placement | DS STOP: abort");
        telemetry.addLine("Enable DS 30-second AUTO timer. No gamepad interaction after Play (G401).");
        show();
        idle();
      }
      if (isStopRequested()) {
        return;
      if (isStopRequested()) { return; }
      clock.reset();
      if (!confirmed || !HiveAutoConfig.canRun()) {
        status = "NO MOVEMENT: confirm placement and commission HiveAutoConfig first";
        show(); return;
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
      follower.setPose(pose(field.startPose(route.start())));
      follower.updatePose();
      if (!HiveAutoConfig.DRY_RUN && tags != null && !tags.isConnected()) {
        status = "NO MOVEMENT: Limelight is disconnected"; show(); return;
      }
      for (Stage stage : Stage.values()) {
        if (!validate(field, approach(stage, alliance, route))
            || (stage.pickup && !field.footprintInsideField(pickupTarget(stage, alliance)))) {
          status = "NO MOVEMENT: invalid route/pickup footprint at " + stage; show(); return;
        }
      }
      if (pan != null) { pan.start(); }
      HiveTipDetector side1 = new HiveTipDetector(route.firstTagMin());
      HiveTipDetector side2 = new HiveTipDetector(route.secondTagMin());
      for (Stage stage : Stage.values()) {
        currentStage = stage;
        int tagMin = stage.side1 ? route.firstTagMin() : route.secondTagMin();
        if (pan != null) {
          pan.setTrackingEnabled(!stage.pickup);
          if (!stage.pickup) { pan.setTargetIdRange(tagMin, tagMin + 3); }
        }
        if (completed) {
          status = route.isEmpty() ? "POSE ONLY: selected field origin active; no movement programmed"
              : "Programmed route complete; motors stopped";
        if (clock.seconds() >= scoringDeadline()) { status = "Parking reserve reached"; break; }
        if (stage.pickup && !inventory.canPickup()) {
          status = "Pickup blocked: expected inventory not empty (G407)"; break;
        }
        List<Waypoint> approach = approach(stage, alliance, route);
        if (!moveAll(approach, false)) { break; }
        if (stage == Stage.SIDE_1_PRELOAD_SHOT && HiveAutoConfig.RELOCALIZE_AT_HIVE) {
          // Stopped at the ASSUMED pose in front of hive side 1: fix X/Y from its tags, then
          // drive the (usually small) remaining error out so the whole route runs from the fix.
          if (!relocalizeAtHive(alliance)) { break; }
          if (!move(approach.get(approach.size() - 1), false)) { break; }
        }
        if (stage.pickup) {
          if (!pickup(pickupTarget(stage, alliance))) { break; }
        } else if (!shootBatch(tagMin, stage.requireTip, stage.side1 ? side1 : side2)) {
          break;
        }
      }
      follower.breakFollowing();
      while (opModeIsActive() && autoClock.seconds() < AUTO_BUDGET_SECONDS) {
        follower.updatePose();
        showSelection(field, selection);
        showTelemetry(true);
        idle();
      stopMechanisms();
      if (!movementFault && healthy()) {
        List<Waypoint> parking = route.parkFrom(waypoint(follower.getPose()));
        if (validate(field, parking) && moveAll(parking, true)) {
          status = HiveAutoConfig.DRY_RUN ? "DRY RUN route finished; parked, no tips claimed"
              : "At nominal loading-zone park pose; vision-confirmed transitions: " + confirmedTips;
        }
      }
      follower.breakFollowing();
      show();
    } finally {
      try { follower.breakFollowing(); } finally { if (tags != null) { tags.stop(); } }
      stopMechanisms();
      if (follower != null) { follower.breakFollowing(); }
      if (pan != null) { pan.close(); }
      if (tags != null) { tags.stop(); }
    }
  }

  /**
   * Holds the check pose, averages fresh Limelight frames of the measured hive tags, and if they
   * agree overwrites Pedro/Pinpoint X/Y (heading stays on the IMU). Any vision problem leaves
   * odometry unchanged and continues; returns false only for STOP / B / time / Pinpoint faults.
   * Robot-center waypoints for each stage, written for RED and mirrored for BLUE.
   * Side 1 shooting pose (-12.75, -40, 90); side 2 shooting pose (-12.75, 40, 270).
   * Hive changes go through the outside corridor at X = -44; never straight through the hive.
   */
  static List<Waypoint> approach(Stage stage, Alliance alliance, HiveAutoRoute route) {
    switch (stage) {
      case SIDE_1_PRELOAD_SHOT: return route.firstApproach();
      case FLOWER_1_PICKUP:
        return points(alliance, new Waypoint(-44, -40, 180), new Waypoint(-44, -24, 180),
            new Waypoint(-49, -24, 180));
      case SIDE_2_FIRST_SHOT:
        // Back off the flower wall first, then up the corridor to hive side 2.
        return points(alliance, new Waypoint(-44, -24, 180), new Waypoint(-44, 40, 270),
            new Waypoint(-12.75, 40, 270));
      case FLOWER_2_PICKUP:
        return points(alliance, new Waypoint(-24, 40, 270), new Waypoint(-24, 40, 90),
            new Waypoint(-24, 49, 90));
      case SIDE_2_SECOND_SHOT:
        return points(alliance, new Waypoint(-24, 49, 90), new Waypoint(-24, 40, 270),
            new Waypoint(-12.75, 40, 270));
      default: throw new IllegalArgumentException("Unknown stage");
    }
  }

  /** Last 8-inch intake creep. Confirm actual reach and flower clearance first. */
  static Waypoint pickupTarget(Stage stage, Alliance alliance) {
    switch (stage) {
      case FLOWER_1_PICKUP: return points(alliance, new Waypoint(-57, -24, 180)).get(0);
      case FLOWER_2_PICKUP: return points(alliance, new Waypoint(-24, 57, 90)).get(0);
      default: throw new IllegalArgumentException("Not a pickup stage");
    }
  }

  private static List<Waypoint> points(Alliance alliance, Waypoint... red) {
    if (alliance == Alliance.BLUE) {
      for (int i = 0; i < red.length; i++) {
        Waypoint p = red[i];
        red[i] = new Waypoint(-p.getX(), -p.getY(), (p.getHeadingDegrees() + 180) % 360);
      }
    }
    return Collections.unmodifiableList(Arrays.asList(red));
  }

  private boolean moveAll(List<Waypoint> points, boolean parking) {
    for (Waypoint p : points) {
      if (!move(p, parking)) { return false; }
    }
    return true;
  }

  private boolean move(Waypoint p, boolean parking) {
    target = p;
    if (!healthy()) { return false; }
    Pose from = follower.getPose();
    if (Math.hypot(p.getX() - from.getX(), p.getY() - from.getY()) < 0.1) {
      // Hold (and turn in place) rather than constructing a zero-length path.
      follower.holdPoint(pose(p), false);
    } else {
      Path path = new Path(new BezierLine(from, pose(p)));
      path.setLinearHeadingInterpolation(from.getHeading(), Math.toRadians(p.getHeadingDegrees()));
      follower.followPath(path, true);
    }
    double started = clock.seconds(), settled = -1;
    try {
      while (opModeIsActive()) {
        follower.update();
        updateTags();
        if (!healthy()) { return false; }
        double now = clock.seconds();
        if (!parking && now >= scoringDeadline()) {
          status = "Parking reserve reached: interrupt scoring route"; return false;
        }
        if (!atPose(p)) { settled = -1; }
        else if (settled < 0) { settled = now; }
        else if (now - settled >= 0.1) { return true; }
        if (now - started >= HiveAutoConfig.LEG_TIMEOUT_SECONDS) {
          movementFault = true;
          status = "ABORTED: movement timeout; no blind recovery"; return false;
        }
        status = parking ? "Driving to loading-zone park" : "Moving to " + currentStage;
        show(); idle();
      }
      return false;
    } finally { follower.breakFollowing(); }
  }

  /**
   * Holds the assumed side 1 shooting pose, averages fresh Limelight frames of the measured hive
   * tags, and if they agree overwrites Pedro/Pinpoint X/Y (heading stays on the IMU). Any vision
   * problem leaves odometry unchanged and continues; returns false only for STOP / time / Pinpoint faults.
   */
  private boolean relocalizeAtHive(Alliance alliance) {
    HiveTagRelocalizer fix = new HiveTagRelocalizer(
        alliance == Alliance.RED ? HiveAutoConfig.RED_HIVE_TAGS : HiveAutoConfig.BLUE_HIVE_TAGS,
        HiveAutoConfig.RELOCALIZE_FRAMES, HiveAutoConfig.RELOCALIZE_MAX_SPREAD_INCHES,
        HiveAutoConfig.RELOCALIZE_MAX_CORRECTION_INCHES);
    if (tags == null || !tags.isConnected()) {
      fixStatus = "SKIPPED: Limelight missing or disconnected; using odometry";
      return true;
      fixStatus = "SKIPPED: Limelight missing or disconnected; using odometry"; return true;
    }
    if (!fix.hasTagPositions()) {
      fixStatus = "SKIPPED: hive tag positions not measured (HiveAutoConfig); using odometry";
      return true;
      fixStatus = "SKIPPED: hive tag positions not measured (HiveAutoConfig); using odometry"; return true;
    }
    follower.holdPoint(toPose(target), false);
    ElapsedTime checkClock = new ElapsedTime();
    follower.holdPoint(pose(target), false);
    double started = clock.seconds();
    try {
      while (fix.getFrameCount() < HiveAutoConfig.RELOCALIZE_FRAMES
          && checkClock.seconds() < HiveAutoConfig.RELOCALIZE_TIMEOUT_SECONDS) {
        if (!canContinue()) {
          return false;
        }
      while (opModeIsActive() && fix.getFrameCount() < HiveAutoConfig.RELOCALIZE_FRAMES
          && clock.seconds() - started < HiveAutoConfig.RELOCALIZE_TIMEOUT_SECONDS) {
        follower.update();
        tags.update();
        Pose pose = follower.getPose();
        fix.addFrame(tags.getResultTimestamp(), pose.getX(), pose.getY(), pose.getHeading(),
        updateTags();
        if (!healthy()) { fixStatus = "SKIPPED: STOP/Pinpoint fault"; return false; }
        Pose p = follower.getPose();
        fix.addFrame(tags.getResultTimestamp(), p.getX(), p.getY(), p.getHeading(),
            tags.getObservations());
        status = "Limelight position check in front of hive";
        fixStatus = "reading hive tags: " + fix.getFrameCount() + " / " + HiveAutoConfig.RELOCALIZE_FRAMES;
        showTelemetry(false);
        idle();
        status = "Limelight position check in front of hive side 1";
        show(); idle();
      }
    } finally {
      follower.breakFollowing();
    }
    } finally { follower.breakFollowing(); }
    if (!opModeIsActive()) { return false; }
    if (fix.getStatus() != HiveTagRelocalizer.Status.OK) {
      fixStatus = String.format(Locale.US,
          "NOT APPLIED (%s, %d frames, spread %.1f in, offset %.1f in): using odometry",
          fix.getStatus(), fix.getFrameCount(), fix.getSpreadInches(), fix.getCorrectionInches());
      return true;
    }
    Pose pose = follower.getPose();
    double[] corrected = fix.corrected(pose.getX(), pose.getY());
    Pose p = follower.getPose();
    double[] corrected = fix.corrected(p.getX(), p.getY());
    double[] offset = fix.getOffset();
    follower.setPose(new Pose(corrected[0], corrected[1], pose.getHeading()));
    follower.setPose(new Pose(corrected[0], corrected[1], p.getHeading()));
    fixStatus = String.format(Locale.US,
        "APPLIED: dX %+.1f, dY %+.1f in (tags %s, %d frames, spread %.1f in)",
        offset[0], offset[1], fix.getUsedTagIds(), fix.getFrameCount(), fix.getSpreadInches());
    return true;
  }

  /** Resolve and bound-check every programmed endpoint before executing the first leg. */
  private boolean validateRoute(BiobuzzFieldMap field, List<String> route) {
    for (String name : route) {
      try {
        if (!field.footprintInsideField(field.getLocation(name))) {
          status = "NO MOVEMENT: target footprint outside field: " + name;
          return false;
  private boolean shootBatch(int minTag, boolean requireTip, HiveTipDetector detector) {
    if (!inventory.isKnown() || inventory.getRemaining() != PollenBatchInventory.CAPACITY) {
      status = "Shot blocked: four-pollen expected batch required"; return false;
    }
    if (HiveAutoConfig.DRY_RUN) {
      inventory.recordFed(4);
      status = "DRY RUN: simulated four-pollen batch at cluster " + minTag + "; no tip claimed";
      show(); return true;
    }
    HiveShotController shot = new HiveShotController(clock.seconds(), HiveAutoConfig.FEED_PULSE_SECONDS, 4, requireTip);
    boolean accounted = false;
    follower.holdPoint(pose(target), false);
    try {
      while (opModeIsActive() && clock.seconds() < scoringDeadline()) {
        follower.update(); // Drive/heading hold continues while the feeder waits; no sleeps.
        if (pan != null) { pan.setTrackingEnabled(shot.getState() == HiveShotController.State.WAIT_HIGH); }
        updateTags();
        if (!healthy()) { return false; }
        double now = clock.seconds();
        detector.update(now, tags.getResultTimestamp(), tags.getObservations());
        boolean rpmReady = Double.isFinite(shooter.getActualRpm1())
            && Double.isFinite(shooter.getActualRpm2())
            && Math.abs(shooter.getActualRpm1() - HiveAutoConfig.SHOOT_RPM_1) <= 150
            && Math.abs(shooter.getActualRpm2() - HiveAutoConfig.SHOOT_RPM_2) <= 150;
        shot.update(now, tags.isConnected(), detector.isHighConfirmed() && detector.isHighVisible(),
            detector.isLowCandidate(), detector.isTipped(), rpmReady, atPose(target));
        shooter.setActive(shot.shooterOn());
        setFeed(shot.feederOn());
        telemetry.addData("Cluster / Locked Tag", "%d-%d / %d", minTag, minTag + 3, detector.getLockedId());
        telemetry.addData("Shot State / Commanded Pulses", "%s / %d", shot.getState(), shot.getPulses());
        telemetry.addData("Completed Feed Pulses (estimated pollen)", "%d / 4", shot.getCompletedPulses());
        telemetry.addData("Tip Required After This Batch", requireTip);
        telemetry.addData("Actual Shooter RPM", "%.0f / %.0f", shooter.getActualRpm1(), shooter.getActualRpm2());
        status = "Shooting: " + shot.getState();
        show();
        if (shot.isDone()) {
          if (shot.isTipConfirmed()) { confirmedTips++; }
          if (shot.getState() == HiveShotController.State.COMPLETE && shot.getCompletedPulses() == 4
              && (!requireTip || shot.isTipConfirmed())) {
            // Only a fully completed four-pulse batch empties expected inventory.
            inventory.recordFed(4);
            accounted = true;
            if (!requireTip && shot.isTipConfirmed()) {
              status = "Unexpected tip before final batch: stop scoring and park"; return false;
            }
            return true;
          }
          status = "Batch interrupted or tip unconfirmed: no further pickups; attempt parking"; return false;
        }
      } catch (IllegalArgumentException e) {
        status = "NO MOVEMENT: " + e.getMessage();
        return false;
        idle();
      }
      status = "Parking reserve reached: stop shooting"; return false;
    } finally {
      if (!accounted) { inventory.invalidate(); }
      stopMechanisms(); follower.breakFollowing();
    }
    return true;
  }

  /** Every target is an absolute field pose resolved by name, not an encoder-distance move. */
  private boolean moveToLocation(String name, Waypoint destination) {
    if (!canContinue()) {
      return false;
    }
    Pose start = follower.getPose();
    if (Math.hypot(destination.getX() - start.getX(), destination.getY() - start.getY()) < 0.1) {
      // Hold rather than constructing a zero-length path after a small position correction.
      follower.holdPoint(toPose(destination), false);
    } else {
      Path path = new Path(new BezierLine(start, toPose(destination)));
      path.setLinearHeadingInterpolation(start.getHeading(), Math.toRadians(destination.getHeadingDegrees()));
      follower.followPath(path, true);
    }
    ElapsedTime legClock = new ElapsedTime();
    double settledSince = -1;
  /** Intake runs during an 8-inch creep and bounded dwell; updates continue without sleeps. */
  private boolean pickup(Waypoint destination) {
    if (!inventory.canPickup()) { status = "Pickup blocked by four-element limit"; return false; }
    boolean accounted = false;
    try {
      while (opModeIsActive()) {
        if (!canContinue()) {
          return false;
        }
        follower.update();
        if (tags != null) { tags.update(); }
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
      follower.setMaxPower(HiveAutoConfig.PICKUP_DRIVE_POWER);
      if (pickupIntake != null) { pickupIntake.setPower(HiveAutoConfig.PICKUP_INTAKE_POWER); }
      if (!move(destination, false)) { return false; }
      if (!HiveAutoConfig.DRY_RUN) {
        follower.holdPoint(pose(destination), false);
        double started = clock.seconds();
        while (opModeIsActive() && clock.seconds() - started < HiveAutoConfig.PICKUP_HOLD_SECONDS) {
          follower.update();
          updateTags();
          if (!healthy() || clock.seconds() >= scoringDeadline()) {
            status = "Pickup interrupted: no refill assumed; attempt parking"; return false;
          }
          if (!atPose(destination)) {
            status = "Pickup pose drift: no refill assumed; attempt parking"; return false;
          }
          status = "Timed pickup: expected four pollen (not sensor counted)";
          show(); idle();
        }
        status = "Moving to " + name;
        telemetry.addData("Leg Time", "%.1f / %.1f s", elapsed, LEG_TIMEOUT_SECONDS);
        showTelemetry(false);
        idle();
      }
      status = "Stopped";
      return false;
      if (!healthy() || clock.seconds() >= scoringDeadline()) { return false; }
      inventory.recordPickup();
      accounted = true;
      status = HiveAutoConfig.DRY_RUN ? "DRY RUN: simulated four-pollen refill" : "Calibrated timed refill complete";
      return true;
    } finally {
      if (!accounted) { inventory.invalidate(); }
      stopMechanisms();
      follower.breakFollowing();
      follower.setMaxPower(HiveAutoConfig.MAX_DRIVE_POWER);
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
  private boolean atPose(Waypoint p) {
    Pose current = follower.getPose();
    double angle = Math.toRadians(p.getHeadingDegrees()) - current.getHeading();
    return Math.hypot(p.getX() - current.getX(), p.getY() - current.getY()) <= 1.5
        && Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle))) <= Math.toRadians(5);
  }

  private boolean healthy() {
    Pose p = follower.getPose();
    boolean ready = opModeIsActive() && clock.seconds() < HiveAutoConfig.AUTO_BUDGET_SECONDS
        && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
        && Double.isFinite(p.getX()) && Double.isFinite(p.getY()) && Double.isFinite(p.getHeading());
    if (!ready) {
      movementFault = true;
      status = "ABORTED: STOP, time budget or Pinpoint fault";
    }
    return ready;
  }

  private static boolean validate(BiobuzzFieldMap field, List<Waypoint> points) {
    for (Waypoint p : points) { if (!field.footprintInsideField(p)) { return false; } }
    return true;
  }

  private boolean sensorReady() {
    Pose pose = follower.getPose();
    return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
        && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getHeading());
  private static double scoringDeadline() {
    return HiveAutoConfig.AUTO_BUDGET_SECONDS - HiveAutoConfig.PARK_RESERVE_SECONDS;
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
  private void setFeed(boolean on) {
    double power = on ? HiveAutoConfig.FEED_POWER : 0;
    if (intakeFeeder != null) { intakeFeeder.setPower(power); }
  }

  private void updateTags() {
    if (tags != null) { tags.update(); }
    if (pan != null) { pan.update(); }
  }

  private void stopMechanisms() {
    setFeed(false);
    if (pickupIntake != null) { pickupIntake.setPower(0); }
    if (shooter != null) { shooter.stop(); }
  }

  private void showTelemetry(boolean matrix) {
  private void show() {
    telemetry.addData("Status", status);
    telemetry.addData("Max Motor Power", "%.0f%%", MAX_POWER * 100);
    telemetry.addData("Stage", currentStage == null ? "INIT" : currentStage);
    telemetry.addData("Expected Pollen / Limit", inventory.isKnown()
        ? inventory.getRemaining() + " / 4 (timing model, not sensor)" : "UNKNOWN: interrupted batch/pickup");
    telemetry.addData("Completed Batch Feed Estimate", inventory.getTotalFed());
    telemetry.addData("AUTO Time / Budget", "%.1f / %.0f s", clock.seconds(), HiveAutoConfig.AUTO_BUDGET_SECONDS);
    telemetry.addData("Vision-confirmed transitions (not official score)", confirmedTips);
    telemetry.addData("Limelight Position Fix", fixStatus);
    if (tags != null) {
      telemetry.addData("Limelight Connected / Visible Tags", "%s / %d",
          tags.isConnected(), tags.getObservations().size());
    } else {
      telemetry.addData("Limelight", "not configured; using odometry");
      if (pan != null) { CameraPanTelemetry.add(telemetry, pan, tags); }
      telemetry.addData("Limelight Connected / Visible Tags", "%s / %d", tags.isConnected(), tags.getObservations().size());
      Pose robot = follower.getPose();
      for (AprilTagObservation tag : tags.getObservations()) {
        // "field XY" assumes the current odometry pose is right: use it when measuring the tag map.
        double[] tagXY = HiveTagRelocalizer.tagFromRobot(robot.getX(), robot.getY(),
            tag.getForwardInches(), tag.getLeftInches(), robot.getHeading());
        telemetry.addData("Tag " + tag.getId(),
            "fwd %.1f, left %.1f, height %.1f in; field XY (%.1f, %.1f)",
            tag.getForwardInches(), tag.getLeftInches(),
            tag.getHeightInches() + HiveAutoConfig.ROBOT_ORIGIN_HEIGHT_INCHES, tagXY[0], tagXY[1]);
      }
    }
    FieldLocationTelemetry.add(telemetry, follower.getPose(), pinpoint, targetName, target, matrix);
    FieldLocationTelemetry.add(telemetry, follower.getPose(), pinpoint, "Hive route (8)", target, false);
    telemetry.update();
  }

  private static Pose toPose(Waypoint waypoint) {
    return new Pose(waypoint.getX(), waypoint.getY(), Math.toRadians(waypoint.getHeadingDegrees()));
  }
  private static Pose pose(Waypoint p) { return new Pose(p.getX(), p.getY(), Math.toRadians(p.getHeadingDegrees())); }
  private static Waypoint waypoint(Pose p) { return new Waypoint(p.getX(), p.getY(), Math.toDegrees(p.getHeading())); }
}
