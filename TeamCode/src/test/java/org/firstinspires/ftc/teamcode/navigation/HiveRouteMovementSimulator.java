package org.firstinspires.ftc.teamcode.navigation;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.HiveShotController;
import org.firstinspires.ftc.teamcode.control.PollenBatchInventory;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.HiveAutoRoute.Stage;

/**
 * Test-only kinematic approximation, NOT a Pedro/Pinpoint hardware simulator. Uses production
 * waypoints/config and the production inventory/shot controller, but ideal linear motion,
 * power-scaled speed, instantaneous acceleration, perfect pose feedback, and synthetic tags/RPM.
 * No live mechanism verification flags are changed by an offline live-timing scenario.
 */
final class HiveRouteMovementSimulator {
  private static final double DT = 0.02;
  final List<Waypoint> samples = new ArrayList<>();
  final List<String> events = new ArrayList<>();
  final PollenBatchInventory inventory = new PollenBatchInventory();
  final PreloadedHiveAutoRoute route;
  final BiobuzzFieldMap field = new BiobuzzFieldMap(
      RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
  final double fullPowerInchesPerSecond, fullPowerDegreesPerSecond;
  final boolean dryRun;
  Waypoint current;
  double time, distance;
  int completedStages, confirmedTips;
  boolean parked, movementFault, reserveReached;
  String outcome = "INIT";

  HiveRouteMovementSimulator(Alliance alliance, double linearSpeed, double angularSpeed, boolean dryRun) {
    if (!Double.isFinite(linearSpeed) || linearSpeed < 0
        || !Double.isFinite(angularSpeed) || angularSpeed <= 0) {
      throw new IllegalArgumentException("Finite nonnegative speed and positive turn rate required");
    }
    route = new PreloadedHiveAutoRoute(alliance);
    fullPowerInchesPerSecond = linearSpeed;
    fullPowerDegreesPerSecond = angularSpeed;
    this.dryRun = dryRun;
    current = field.startPose(route.start());
    samples.add(current);
  }

  HiveRouteMovementSimulator run() {
    for (Stage stage : route.stages()) {
      if (time >= scoringDeadline()) { reserveReached = true; break; }
      boolean arrived = true;
      for (Waypoint p : route.approach(stage)) {
        if (!move(p, HiveAutoConfig.MAX_DRIVE_POWER, false)) { arrived = false; break; }
      }
      if (!arrived) { break; }
      if (stage == Stage.PRELOAD_SHOT && HiveAutoConfig.RELOCALIZE_AT_HIVE) {
        // Default tag map is unmeasured: the fix is skipped; executor still re-holds this pose.
        if (!move(current, HiveAutoConfig.MAX_DRIVE_POWER, false)) { break; }
      }
      if (stage.isPickup()) {
        if (!inventory.canPickup()) { throw new AssertionError("Pickup with nonempty inventory"); }
        if (!move(route.pickupTarget(stage), HiveAutoConfig.PICKUP_DRIVE_POWER, false)) { break; }
        if (!dryRun && !dwell(HiveAutoConfig.PICKUP_HOLD_SECONDS)) { inventory.invalidate(); break; }
        inventory.recordPickup();
      } else if (dryRun) {
        inventory.recordFed(4);
      } else if (!shoot(stage)) {
        inventory.invalidate(); break;
      }
      completedStages++;
      events.add(String.format(java.util.Locale.US, "%.2fs %s", time, stage));
    }
    if (!movementFault && time < HiveAutoConfig.AUTO_BUDGET_SECONDS) {
      parked = true;
      for (Waypoint p : route.parkFrom(current)) {
        if (!move(p, HiveAutoConfig.MAX_DRIVE_POWER, true)) { parked = false; break; }
      }
    }
    if (parked) { outcome = reserveReached ? "PARKED_AFTER_SCORING_CUTOFF" : "FULL_ROUTE_PARKED"; }
    return this;
  }

  private boolean move(Waypoint to, double power, boolean parking) {
    Waypoint from = current;
    double length = Math.hypot(to.getX() - from.getX(), to.getY() - from.getY());
    double angle = ((to.getHeadingDegrees() - from.getHeadingDegrees() + 540) % 360) - 180;
    double travel = length == 0 ? 0 : fullPowerInchesPerSecond == 0
        ? Double.POSITIVE_INFINITY : length / (fullPowerInchesPerSecond * power);
    double duration = Math.max(travel, Math.abs(angle) / (fullPowerDegreesPerSecond * power));
    double started = time, atPoseSince = -1;
    while (true) {
      double before = Math.hypot(current.getX() - from.getX(), current.getY() - from.getY());
      time += DT;
      double progress = duration == 0 ? 1 : Math.min(1, (time - started) / duration);
      current = new Waypoint(from.getX() + progress * (to.getX() - from.getX()),
          from.getY() + progress * (to.getY() - from.getY()),
          from.getHeadingDegrees() + progress * angle);
      distance += length * progress - before;
      samples.add(current);
      if (time >= HiveAutoConfig.AUTO_BUDGET_SECONDS) {
        movementFault = true; outcome = "AUTO_BUDGET_ABORT"; return false;
      }
      if (!parking && time >= scoringDeadline()) { reserveReached = true; return false; }
      // Conservative: settle at the exact endpoint, instead of accepting 1.5in/5deg early.
      if (progress == 1) {
        if (atPoseSince < 0) { atPoseSince = time; }
        else if (time - atPoseSince >= 0.1) { return true; }
      }
      if (time - started >= HiveAutoConfig.LEG_TIMEOUT_SECONDS) {
        movementFault = true; outcome = "LEG_TIMEOUT_ABORT"; return false;
      }
    }
  }

  private boolean dwell(double duration) {
    double started = time;
    while (time - started < duration) {
      time += DT;
      if (time >= scoringDeadline()) { reserveReached = true; return false; }
    }
    return true;
  }

  private boolean shoot(Stage stage) {
    double started = time, fourthPulseAt = -1;
    HiveShotController shot = new HiveShotController(started, HiveAutoConfig.FEED_PULSE_SECONDS,
        4, route.requiresTip(stage));
    while (time < scoringDeadline()) {
      time += DT;
      // Synthetic vision: stable high after .3s; required tip .4s after pulse 4 completes.
      if (shot.getCompletedPulses() == 4 && fourthPulseAt < 0) { fourthPulseAt = time; }
      boolean tipped = route.requiresTip(stage) && fourthPulseAt >= 0 && time - fourthPulseAt >= 0.4;
      shot.update(time, true, time - started >= 0.3 && !tipped, tipped, tipped, true, true);
      if (shot.isDone()) {
        if (shot.getState() != HiveShotController.State.COMPLETE || shot.getCompletedPulses() != 4) {
          outcome = "SYNTHETIC_SHOT_FAILED"; return false;
        }
        inventory.recordFed(4);
        if (shot.isTipConfirmed()) { confirmedTips++; }
        return true;
      }
    }
    reserveReached = true; return false;
  }

  String summary(Alliance alliance) {
    return String.format(java.util.Locale.US,
        "%s %s: %s; %.2fs, %.2fin, %d/5 stages, %d feed estimate, %d synthetic tips; final (%.2f, %.2f)",
        alliance, dryRun ? "driving-only" : "synthetic live timing", outcome, time, distance,
        completedStages, inventory.getTotalFed(), confirmedTips, current.getX(), current.getY());
  }

  private static double scoringDeadline() {
    return HiveAutoConfig.AUTO_BUDGET_SECONDS - HiveAutoConfig.PARK_RESERVE_SECONDS;
  }
}
