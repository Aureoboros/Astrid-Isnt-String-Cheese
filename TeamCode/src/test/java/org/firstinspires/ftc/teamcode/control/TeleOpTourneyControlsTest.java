package org.firstinspires.ftc.teamcode.control;
import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;
public final class TeleOpTourneyControlsTest {
  @Test public void importedEntryRetainsIntakeOnButUsesSameShooterAndGateDefaults() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    assertTrue(c.isIntakeEnabled()); assertFalse(c.isIntakeReversed());
    assertTrue(c.isShooterEnabled()); assertTrue(c.isShooterReversed());
    assertEquals(3200, c.getShooterRpm(), 0); assertEquals(190, c.getGateRpm(), 0);
    assertFalse(c.isGateEnabled()); assertTrue(c.isGateReversed());
  }
  @Test public void sameMapHandlesResetReverseGateAndShooterWithoutSpitOrAutoDrive() {
    TeleOpTourneyControls c = new TeleOpTourneyControls();
    Gamepad p = new Gamepad(), q = new Gamepad();
    q.a = q.b = q.x = q.back = true; c.update(p, q);
    assertTrue(c.isOdometryResetRequested()); assertTrue(c.isIntakeEnabled());
    assertTrue(c.isIntakeReversed()); assertTrue(c.isGateEnabled()); assertFalse(c.isShooterEnabled());
    c.update(p, q); assertFalse(c.isOdometryResetRequested());
    q.y = true; c.update(p, q); assertFalse(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
  }
}
