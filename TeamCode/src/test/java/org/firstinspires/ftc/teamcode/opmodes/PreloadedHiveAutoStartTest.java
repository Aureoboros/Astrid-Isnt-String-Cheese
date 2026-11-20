package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.junit.Test;

public final class PreloadedHiveAutoStartTest {
  @Test public void blueAndRedUsePlayConfirmationAndKeepTheirAllianceLocked() {
    BiobuzzHiveAuto blue = new BluePreloadedHiveAuto();
    BiobuzzHiveAuto red = new RedPreloadedHiveAuto();
    assertFalse(blue.requiresPlacementConfirmation());
    assertFalse(red.requiresPlacementConfirmation());
    assertFalse(blue.allowAllianceSelection());
    assertFalse(red.allowAllianceSelection());
    assertEquals(Alliance.BLUE, blue.initialAlliance());
    assertEquals(Alliance.RED, red.initialAlliance());
    assertTrue(blue.canRunRoute());
    assertTrue(red.canRunRoute());
  }

  @Test public void originalHiveAutoStillRequiresExplicitPlacementConfirmation() {
    assertTrue(new BiobuzzHiveAuto().requiresPlacementConfirmation());
  }
}
