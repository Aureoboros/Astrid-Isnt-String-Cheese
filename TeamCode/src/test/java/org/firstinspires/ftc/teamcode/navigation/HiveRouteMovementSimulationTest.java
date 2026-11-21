package org.firstinspires.ftc.teamcode.navigation;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;
import org.firstinspires.ftc.teamcode.config.PreloadedHiveAutoConfig;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.junit.Test;

/** Illustrative speed profiles, NOT measured robot velocities or a test of Pedro PID dynamics. */
public final class HiveRouteMovementSimulationTest {
  @Test public void unlimitedPracticeCompletesBothRoutesAtSlowSpeedBeyondThirtySeconds() {
    for (Alliance a : Alliance.values()) {
      HiveRouteMovementSimulator s = new HiveRouteMovementSimulator(a, 20, 72, true,
          PreloadedHiveAutoConfig.autoBudgetSeconds(), PreloadedHiveAutoConfig.legTimeoutSeconds()).run();
      System.out.println("UNLIMITED PRACTICE " + s.summary(a));
      assertEquals("FULL_ROUTE_PARKED", s.outcome);
      assertEquals(5, s.completedStages);
      assertEquals(12, s.inventory.getTotalFed());
      assertTrue(s.time > 30);
      assertFalse(s.reserveReached); assertFalse(s.movementFault); assertTrue(s.parked);
      assertEquals(a == Alliance.RED ? -61 : 61, s.current.getX(), 1e-6);
      assertEquals(a == Alliance.RED ? 48 : -48, s.current.getY(), 1e-6);
    }
  }

  @Test public void unlimitedPracticeStallWaitsBeyondOldLegTimeoutUntilDriverStop() {
    for (Alliance a : Alliance.values()) {
      HiveRouteMovementSimulator s = new HiveRouteMovementSimulator(a, 0, 360, true,
          PreloadedHiveAutoConfig.autoBudgetSeconds(), PreloadedHiveAutoConfig.legTimeoutSeconds());
      s.stopAtSeconds = 40;
      s.run();
      assertTrue(s.movementFault); assertFalse(s.parked);
      assertEquals("DRIVER_STOP_ABORT", s.outcome);
      assertEquals(40, s.time, 0.03);
    }
  }

  @Test public void individualLegCanExceedThirtySecondsWithoutAbortingRoute() {
    for (Alliance a : Alliance.values()) {
      HiveRouteMovementSimulator s = new HiveRouteMovementSimulator(a, 5, 18, true,
          PreloadedHiveAutoConfig.autoBudgetSeconds(), PreloadedHiveAutoConfig.legTimeoutSeconds()).run();
      assertEquals("FULL_ROUTE_PARKED", s.outcome);
      assertTrue(s.time > 200);
      assertEquals(5, s.completedStages); assertTrue(s.parked); assertFalse(s.movementFault);
    }
  }

  @Test public void bothDrivingOnlyRoutesFinishAndParkUnderNominalAssumedSpeed() {
    HiveRouteMovementSimulator red = simulate(Alliance.RED, 100, 360, true);
    HiveRouteMovementSimulator blue = simulate(Alliance.BLUE, 100, 360, true);
    for (HiveRouteMovementSimulator s : new HiveRouteMovementSimulator[] {red, blue}) {
      assertEquals("FULL_ROUTE_PARKED", s.outcome);
      assertEquals(5, s.completedStages);
      assertEquals(12, s.inventory.getTotalFed());
      assertEquals(0, s.confirmedTips);
      assertFalse(s.movementFault);
      assertTrue(s.time < HiveAutoConfig.AUTO_BUDGET_SECONDS);
    }
    assertEquals(-61, red.current.getX(), 1e-6); assertEquals(48, red.current.getY(), 1e-6);
    assertEquals(61, blue.current.getX(), 1e-6); assertEquals(-48, blue.current.getY(), 1e-6);
    assertEquals(red.time, blue.time, 1e-6);
    assertEquals(red.distance, blue.distance, 1e-6);
    assertEquals(red.samples.size(), blue.samples.size());
    for (int i = 0; i < red.samples.size(); i++) {
      assertEquals(-red.samples.get(i).getX(), blue.samples.get(i).getX(), 1e-6);
      assertEquals(-red.samples.get(i).getY(), blue.samples.get(i).getY(), 1e-6);
    }
  }

  @Test public void slowerDrivingReachesScoringDeadlineAndParksWithoutFinishingAllStages() {
    for (Alliance a : Alliance.values()) {
      HiveRouteMovementSimulator s = simulate(a, 50, 180, true);
      assertTrue(s.reserveReached); assertTrue(s.parked);
      assertTrue(s.completedStages < 5);
      assertEquals("PARKED_AFTER_SCORING_CUTOFF", s.outcome);
    }
  }

  @Test public void stalledMovementAbortsWithoutAttemptingBlindParking() {
    for (Alliance a : Alliance.values()) {
      HiveRouteMovementSimulator s = simulate(a, 0, 360, true);
      assertEquals("LEG_TIMEOUT_ABORT", s.outcome);
      assertTrue(s.movementFault); assertFalse(s.parked);
      assertEquals(0, s.completedStages);
      assertEquals(HiveAutoConfig.LEG_TIMEOUT_SECONDS, s.time, 0.03);
    }
  }

  @Test public void addingIdealShotAndPickupDwellTimingCanInterruptScoringEvenAtNominalSpeed() {
    for (Alliance a : Alliance.values()) {
      HiveRouteMovementSimulator s = simulate(a, 100, 360, false);
      assertTrue(s.reserveReached); assertTrue(s.parked);
      assertTrue(s.completedStages < 5);
      assertEquals("PARKED_AFTER_SCORING_CUTOFF", s.outcome);
    }
  }

  private static HiveRouteMovementSimulator simulate(Alliance a, double linear, double angular, boolean dry) {
    HiveRouteMovementSimulator s = new HiveRouteMovementSimulator(a, linear, angular, dry).run();
    System.out.println(s.summary(a));
    for (String event : s.events) { System.out.println("  " + event); }
    for (Waypoint p : s.samples) {
      assertTrue("Simulated footprint left field", s.field.footprintInsideField(p));
      double extent = 9 * (Math.abs(Math.cos(Math.toRadians(p.getHeadingDegrees())))
          + Math.abs(Math.sin(Math.toRadians(p.getHeadingDegrees()))));
      assertTrue("Simulated footprint crossed alliance half", a == Alliance.RED ? p.getX() + extent <= 0
          : p.getX() - extent >= 0);
      double radius = Math.hypot(9, 9);
      assertFalse("Simulated nominal hive-frame collision", p.getX() > -49.46 / 2 - radius
          && p.getX() < 49.46 / 2 + radius && p.getY() > -38.95 / 2 - radius
          && p.getY() < 38.95 / 2 + radius);
    }
    return s;
  }
}
