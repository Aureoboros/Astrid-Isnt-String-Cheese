package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import org.junit.Test;

public final class PreloadedHiveAutoConfigTest {
  @Test public void operatorApprovedNewRouteCanDriveWithMechanismsStillSimulated() {
    assertTrue(PreloadedHiveAutoConfig.ROUTE_VERIFIED);
    assertTrue(PreloadedHiveAutoConfig.canRun());
    assertTrue(HiveAutoConfig.DRY_RUN);
  }

  @Test public void usesInstalledShootOneTeleopSpeedAndDirection() {
    assertEquals(4800, PreloadedHiveAutoConfig.SHOOT_RPM, 0);
    assertTrue(PreloadedHiveAutoConfig.SHOOT_REVERSED);
  }
}
