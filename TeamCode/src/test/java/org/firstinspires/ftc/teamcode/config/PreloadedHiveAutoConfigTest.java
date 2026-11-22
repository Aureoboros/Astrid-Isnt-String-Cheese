package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import org.junit.Test;

public final class PreloadedHiveAutoConfigTest {
  @Test public void operatorApprovedRedBlueRoutesUseLiveMechanismsWithoutEnablingLegacyAuto() {
    assertTrue(PreloadedHiveAutoConfig.ROUTE_VERIFIED);
    assertTrue(PreloadedHiveAutoConfig.canRun());
    assertTrue(HiveAutoConfig.DRY_RUN);
    assertTrue(PreloadedHiveAutoConfig.MECHANISMS_ENABLED);
    assertEquals(60, PreloadedHiveAutoConfig.GATE_RPM, 0);
    assertFalse(PreloadedHiveAutoConfig.GATE_REVERSED);
    assertEquals(Double.POSITIVE_INFINITY, PreloadedHiveAutoConfig.autoBudgetSeconds(), 0);
    assertEquals(Double.POSITIVE_INFINITY, PreloadedHiveAutoConfig.legTimeoutSeconds(), 0);
  }

  @Test public void usesRequested2900RpmAndExistingShootOneDirection() {
    assertEquals(2900, PreloadedHiveAutoConfig.SHOOT_RPM, 0);
    assertEquals(4800, RobotConfig.TeleTourney.SHOOTER_INITIAL_RPM, 0);
    assertTrue(PreloadedHiveAutoConfig.SHOOT_REVERSED);
  }
}
