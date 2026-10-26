package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.HiveShotController;
import org.firstinspires.ftc.teamcode.control.PollenBatchInventory;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.HiveTipDetector;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute.Stage;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.FieldLocationTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Commissioning draft: attempt two tips on OWN hive, then partially enter own loading zone. */
// Listed on the Driver Station; HiveAutoConfig commissioning gates still prevent unverified operation.
@Autonomous(name = "BIOBUZZ: Two Hive Tips + Park", group = "Competition")
public final class BiobuzzHiveAuto extends LinearOpMode {
  private Follower follower;
  private GoBildaPinpointDriver pinpoint;
  private AprilTagTracker tags;
  private ShooterSubsystem shooter;
  private DcMotor intakeFeeder;
  private DcMotor pickupIntake;
  private CRServo servoFeeder;
  private final ElapsedTime clock = new ElapsedTime();
  private String status = "INIT";
  private Waypoint target;
  private int confirmedTips;
  private boolean movementFault;
  private final PollenBatchInventory inventory = new PollenBatchInventory();
  private Stage currentStage;

  @Override public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    telemetry.addLine("Commissioning draft. Keep robot still during Pinpoint calibration.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    BiobuzzFieldMap field = new BiobuzzFieldMap(
        RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
    Alliance alliance = Alliance.RED;
    HiveAutoRoute route = new HiveAutoRoute(alliance);
    boolean confirmed = false, oldA = false;
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
        } else {
          servoFeeder = hardwareMap.get(CRServo.class, RobotConfig.Hardware.SERVO_MOTOR);
        }
        setFeed(false);
        tags = new AprilTagTracker(hardwareMap);
        tags.start();
      }
      while (opModeInInit()) {
        Alliance next = (gamepad1.dpad_right || gamepad2.dpad_right) ? Alliance.BLUE
            : (gamepad1.dpad_left || gamepad2.dpad_left) ? Alliance.RED : alliance;
        boolean changed = next != alliance;
        boolean a = gamepad1.a || gamepad2.a;
        if (changed) {
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
        telemetry.addData("Alliance / Required Start", "%s / %s", alliance, route.start());
        telemetry.addData("Placement Confirmed", confirmed);
        telemetry.addData("Calibration Allows Run", HiveAutoConfig.canRun());
        telemetry.addData("Mode", HiveAutoConfig.DRY_RUN ? "DRY RUN: no shooting, no tip claims" : "LIVE");
        telemetry.addLine("4 preloads -> flower 1 -> first tip -> flower 2 -> garden -> second tip -> park");
        telemetry.addLine("Live pickups/counting require separate commissioning; counts are timed estimates.");
        telemetry.addLine("D-pad Left/Right: alliance | A: confirm placement | DS STOP: abort");
        telemetry.addLine("Enable DS 30-second AUTO timer. No gamepad interaction after Play (G401).");
        show();
        idle();
      }
      if (isStopRequested()) { return; }
      clock.reset();
      if (!confirmed || !HiveAutoConfig.canRun()) {
        status = "NO MOVEMENT: confirm placement and commission HiveAutoConfig first";
        show(); return;
      }
      follower.setPose(pose(field.startPose(route.start())));
      follower.updatePose();
      if (tags != null && !tags.isConnected()) {
        status = "NO MOVEMENT: Limelight is disconnected"; show(); return;
      }
      for (Stage stage : Stage.values()) {
        if (!validate(field, route.approach(stage))
            || (stage.isPickup() && !field.footprintInsideField(route.pickupTarget(stage)))) {
          status = "NO MOVEMENT: invalid route/pickup footprint"; show(); return;
        }
      }
      HiveTipDetector firstCell = new HiveTipDetector(route.firstTagMin());
      HiveTipDetector secondCell = new HiveTipDetector(route.secondTagMin());
      for (Stage stage : Stage.values()) {
        currentStage = stage;
        if (clock.seconds() >= scoringDeadline()) { status = "Parking reserve reached"; break; }
        if (stage.isPickup() && !inventory.canPickup()) {
          status = "Pickup blocked: expected inventory not empty (G407)"; break;
        }
        if (!moveAll(route.approach(stage), false)) { break; }
        if (stage.isPickup()) {
          if (!pickup(route.pickupTarget(stage))) { break; }
        } else {
          HiveTipDetector detector = stage == Stage.PRELOAD_SHOT || stage == Stage.FIRST_REFILL_SHOT
              ? firstCell : secondCell;
          if (!shootBatch(route.tagMin(stage), stage.requiresTip(), detector)) { break; }
        }
      }
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
      stopMechanisms();
      if (follower != null) { follower.breakFollowing(); }
      if (tags != null) { tags.stop(); }
    }
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
        if (tags != null) { tags.update(); }
        if (!healthy()) { return false; }
        double now = clock.seconds();
        if (!parking && now >= scoringDeadline()) {
          status = "Parking reserve reached: interrupt scoring route"; return false;
        }
        Pose current = follower.getPose();
        double angle = Math.toRadians(p.getHeadingDegrees()) - current.getHeading();
        boolean atPose = Math.hypot(p.getX() - current.getX(), p.getY() - current.getY()) <= 1.5
            && Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle))) <= Math.toRadians(5);
        if (!atPose) { settled = -1; }
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
        follower.update(); // Drive/heading hold continues while the feeder waits; no sleeps/worker threads.
        tags.update();
        if (!healthy()) { return false; }
        double now = clock.seconds();
        detector.update(now, tags.getResultTimestamp(), tags.getObservations());
        boolean rpmReady = Double.isFinite(shooter.getActualRpm1())
            && Double.isFinite(shooter.getActualRpm2())
            && Math.abs(shooter.getActualRpm1() - HiveAutoConfig.SHOOT_RPM_1) <= 150
            && Math.abs(shooter.getActualRpm2() - HiveAutoConfig.SHOOT_RPM_2) <= 150;
        Pose current = follower.getPose();
        double angle = Math.toRadians(target.getHeadingDegrees()) - current.getHeading();
        boolean poseReady = Math.hypot(target.getX() - current.getX(), target.getY() - current.getY()) <= 1.5
            && Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle))) <= Math.toRadians(5);
        shot.update(now, tags.isConnected(), detector.isHighConfirmed() && detector.isHighVisible(),
            detector.isLowCandidate(), detector.isTipped(), rpmReady, poseReady);
        shooter.setActive(shot.shooterOn());
        setFeed(shot.feederOn());
        telemetry.addData("Cluster / Locked Tag", "%d-%d / %d", minTag, minTag + 3, detector.getLockedId());
        telemetry.addData("Shot State / Commanded Pulses", "%s / %d", shot.getState(), shot.getPulses());
        telemetry.addData("Completed Feed Pulses (estimated pollen)", "%d / 4", shot.getCompletedPulses());
        telemetry.addData("Tip Required After This Batch", requireTip);
        telemetry.addData("Actual Shooter RPM", "%.0f / %.0f", shooter.getActualRpm1(), shooter.getActualRpm2());
        status = "Attempting hive tip: " + shot.getState();
        show();
        if (shot.isDone()) {
          if (shot.isTipConfirmed()) { confirmedTips++; }
          if (shot.getState() == HiveShotController.State.COMPLETE && shot.getCompletedPulses() == 4
              && (!requireTip || shot.isTipConfirmed())) {
            // Only a fully completed four-pulse batch empties expected inventory.
            inventory.recordFed(4);
            accounted = true;
            // Unexpected early tipping invalidates the planned two-batch sequence; do not shoot into a low cell.
            if (!requireTip && shot.isTipConfirmed()) {
              status = "Unexpected first-batch tip: stop scoring and park"; return false;
            }
            return true;
          }
          status = "Batch interrupted or tip unconfirmed: no further pickups; attempt parking"; return false;
        }
        idle();
      }
      status = "Parking reserve reached: stop shooting"; return false;
    } finally {
      if (!accounted) { inventory.invalidate(); }
      stopMechanisms(); follower.breakFollowing();
    }
  }

  /** Intake runs during an 8-inch creep and bounded dwell; updates continue without sleeps. */
  private boolean pickup(Waypoint destination) {
    if (!inventory.canPickup()) { status = "Pickup blocked by four-element limit"; return false; }
    boolean accounted = false;
    try {
      follower.setMaxPower(HiveAutoConfig.PICKUP_DRIVE_POWER);
      if (pickupIntake != null) { pickupIntake.setPower(HiveAutoConfig.PICKUP_INTAKE_POWER); }
      if (!move(destination, false)) { return false; }
      if (!HiveAutoConfig.DRY_RUN) {
        follower.holdPoint(pose(destination), false);
        double started = clock.seconds();
        while (opModeIsActive() && clock.seconds() - started < HiveAutoConfig.PICKUP_HOLD_SECONDS) {
          follower.update();
          tags.update();
          if (!healthy() || clock.seconds() >= scoringDeadline()) {
            status = "Pickup interrupted: no refill assumed; attempt parking"; return false;
          }
          Pose p = follower.getPose();
          double angle = Math.toRadians(destination.getHeadingDegrees()) - p.getHeading();
          if (Math.hypot(destination.getX() - p.getX(), destination.getY() - p.getY()) > 1.5
              || Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle))) > Math.toRadians(5)) {
            status = "Pickup pose drift: no refill assumed; attempt parking"; return false;
          }
          status = "Timed pickup: expected four pollen (not sensor counted)";
          show(); idle();
        }
      }
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

  private static double scoringDeadline() {
    return HiveAutoConfig.AUTO_BUDGET_SECONDS - HiveAutoConfig.PARK_RESERVE_SECONDS;
  }

  private void setFeed(boolean on) {
    double power = on ? HiveAutoConfig.FEED_POWER : 0;
    if (intakeFeeder != null) { intakeFeeder.setPower(power); }
    if (servoFeeder != null) { servoFeeder.setPower(power); }
  }

  private void stopMechanisms() {
    setFeed(false);
    if (pickupIntake != null) { pickupIntake.setPower(0); }
    if (shooter != null) { shooter.stop(); }
  }

  private void show() {
    telemetry.addData("Status", status);
    telemetry.addData("Stage", currentStage == null ? "INIT" : currentStage);
    telemetry.addData("Expected Pollen / Limit", inventory.isKnown()
        ? inventory.getRemaining() + " / 4 (timing model, not sensor)" : "UNKNOWN: interrupted batch/pickup");
    telemetry.addData("Completed Batch Feed Estimate", inventory.getTotalFed());
    telemetry.addData("AUTO Time / Budget", "%.1f / %.0f s", clock.seconds(), HiveAutoConfig.AUTO_BUDGET_SECONDS);
    telemetry.addData("Vision-confirmed transitions (not official score)", confirmedTips);
    if (tags != null) {
      telemetry.addData("Limelight Connected / Visible Tags", "%s / %d", tags.isConnected(), tags.getObservations().size());
      for (org.firstinspires.ftc.teamcode.localization.AprilTagObservation tag : tags.getObservations()) {
        telemetry.addData("Tag " + tag.getId(), "distance %.1f in; estimated field height %.1f in",
            tag.getHorizontalDistanceInches(), tag.getHeightInches() + HiveAutoConfig.ROBOT_ORIGIN_HEIGHT_INCHES);
      }
    }
    FieldLocationTelemetry.add(telemetry, follower.getPose(), pinpoint, "Hive route", target, false);
    telemetry.update();
  }

  private static Pose pose(Waypoint p) { return new Pose(p.getX(), p.getY(), Math.toRadians(p.getHeadingDegrees())); }
  private static Waypoint waypoint(Pose p) { return new Waypoint(p.getX(), p.getY(), Math.toDegrees(p.getHeading())); }
}
