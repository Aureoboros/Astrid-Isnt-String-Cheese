package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;

import org.junit.Test;

public final class TeleOpTourneyControlsTest {
  @Test public void startsWithIntakeOnAndGateOff() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    assertTrue(c.isIntakeEnabled());
    assertFalse(c.isIntakeReversed());
    assertFalse(c.isGateEnabled());
    assertFalse(c.isDriveToShootRequested());
  }

  @Test public void yAndXToggleOncePerPress() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    c.updateButtons(false, false, true, true);
    assertFalse(c.isIntakeEnabled());
    assertTrue(c.isGateEnabled());
    c.updateButtons(false, false, true, true); // held: no repeat toggle
    assertFalse(c.isIntakeEnabled());
    assertTrue(c.isGateEnabled());
    c.updateButtons(false, false, false, false);
    c.updateButtons(false, false, true, true);
    assertTrue(c.isIntakeEnabled());
    assertFalse(c.isGateEnabled());
  }

  @Test public void holdingBSpitsThenRestoresToggles() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    c.updateButtons(false, false, false, true); // intake off
    c.updateButtons(false, true, false, false);
    assertTrue(c.isIntakeEnabled() && c.isIntakeReversed());
    assertTrue(c.isGateEnabled() && c.isGateReversed());
    c.updateButtons(false, false, false, false);
    assertFalse(c.isIntakeEnabled());
    assertFalse(c.isGateEnabled());
    assertFalse(c.isGateReversed());
  }

  @Test public void aRequestsOnlyOnNewPress() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    c.updateButtons(true, false, false, false);
    assertTrue(c.isDriveToShootRequested());
    c.updateButtons(true, false, false, false);
    assertFalse(c.isDriveToShootRequested());
  }

  @Test public void launcherStartsAt3200AndDpadStepsOncePerPress() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    assertEquals(3200, c.getShooterRpm(), 0);
    c.updateButtons(false, false, false, false, true, false);
    c.updateButtons(false, false, false, false, true, false); // held: one step only
    assertEquals(3250, c.getShooterRpm(), 0);
    c.updateButtons(false, false, false, false, false, true);
    c.updateButtons(false, false, false, false, false, false);
    c.updateButtons(false, false, false, false, false, true);
    assertEquals(3150, c.getShooterRpm(), 0);
  }
}
