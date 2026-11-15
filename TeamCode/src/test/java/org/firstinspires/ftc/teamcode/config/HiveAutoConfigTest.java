package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import org.junit.Test;

/** Driving-only configuration must allow the route without arming uncommissioned mechanisms. */
public final class HiveAutoConfigTest {
  @Test public void verifiedRouteAllowsAutonomousDriving() {
    assertTrue(HiveAutoConfig.ROUTE_VERIFIED);
    assertTrue(HiveAutoConfig.canRun());
  }

  @Test public void mechanismsRemainSimulatedAndLiveScoringUncommissioned() {
    assertTrue(HiveAutoConfig.DRY_RUN);
    assertEquals(HiveAutoConfig.Feeder.NONE, HiveAutoConfig.FEEDER);
    assertFalse(HiveAutoConfig.SHOOTING_AND_CAMERA_VERIFIED);
    assertFalse(HiveAutoConfig.PICKUPS_VERIFIED);
    assertFalse(HiveAutoConfig.TIMED_POLLEN_COUNTING_VERIFIED);
  }

  @Test public void conservativeDriveLimitsAndParkingReserveRemainEnabled() {
    assertEquals(0.35, HiveAutoConfig.MAX_DRIVE_POWER, 0);
    assertEquals(5, HiveAutoConfig.LEG_TIMEOUT_SECONDS, 0);
    assertTrue(HiveAutoConfig.AUTO_BUDGET_SECONDS <= 30);
    assertTrue(HiveAutoConfig.PARK_RESERVE_SECONDS > 0);
    assertTrue(HiveAutoConfig.PARK_RESERVE_SECONDS < HiveAutoConfig.AUTO_BUDGET_SECONDS);
  }
}
