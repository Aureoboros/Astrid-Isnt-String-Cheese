package org.firstinspires.ftc.teamcode.control;
import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;

public final class TeleTourneyMechanismControlsTest {
  private final TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
  private final Gamepad p = new Gamepad(), q = new Gamepad();
  private void update() { c.update(p, q); }
  @Test public void playDefaultsAreShooterOnGateHoldAndIntakeOff() {
    assertTrue(c.isShooterEnabled()); assertTrue(c.isShooterReversed());
    assertEquals(3200, c.getShooterRpm(), 0);
    assertFalse(c.isIntakeEnabled()); assertFalse(c.isIntakeReversed());
    assertFalse(c.isGateEnabled()); assertTrue(c.isGateReversed());
    assertEquals(190, c.getGateRpm(), 0);
  }
  @Test public void yTogglesIntakeOnlyAndDoesNotRepeatWhileHeld() {
    p.y = true; update(); update();
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isShooterEnabled()); assertFalse(c.isGateEnabled());
    p.y = false; update(); q.y = true; update(); assertFalse(c.isIntakeEnabled());
  }
  @Test public void bSelectsReverseWhileStoppedAndYPreservesDirection() {
    q.b = true; update(); update(); assertTrue(c.isIntakeReversed()); assertFalse(c.isIntakeEnabled());
    q.b = false; p.y = true; update(); assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    p.y = false; update(); p.y = true; update(); assertFalse(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    p.y = false; update(); p.y = true; update(); assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
  }
  @Test public void bReversesRunningIntakeWithoutStoppingIt() {
    p.y = true; update(); p.y = false; update(); p.b = true; update();
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    p.b = false; update(); p.b = true; update(); assertTrue(c.isIntakeEnabled()); assertFalse(c.isIntakeReversed());
  }
  @Test public void xTogglesGateWithoutChangingShooterOrIntake() {
    p.x = true; update(); update(); assertTrue(c.isGateEnabled());
    p.x = false; update(); q.x = true; update(); assertFalse(c.isGateEnabled());
    assertTrue(c.isShooterEnabled()); assertFalse(c.isIntakeEnabled());
  }
  @Test public void overlappingControllersDoNotToggleTwice() {
    p.x = p.y = p.b = p.back = true; update();
    q.x = q.y = q.b = q.back = true; update();
    p.x = p.y = p.b = p.back = false; update();
    assertTrue(c.isGateEnabled()); assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    assertFalse(c.isShooterEnabled());
    q.x = q.y = q.b = q.back = false; update();
    q.x = q.y = q.b = q.back = true; update();
    assertFalse(c.isGateEnabled()); assertFalse(c.isIntakeEnabled()); assertFalse(c.isIntakeReversed());
    assertTrue(c.isShooterEnabled());
  }
  @Test public void aRequestsOneResetPerPressAndDoesNotChangeMechanisms() {
    q.a = true; update(); assertTrue(c.isOdometryResetRequested());
    p.a = true; update(); assertFalse(c.isOdometryResetRequested());
    q.a = false; update(); assertFalse(c.isOdometryResetRequested());
    p.a = false; update(); p.a = true; update(); assertTrue(c.isOdometryResetRequested());
    assertTrue(c.isShooterEnabled()); assertFalse(c.isGateEnabled()); assertFalse(c.isIntakeReversed());
  }
  @Test public void allActionsAreIndependent() {
    p.a = p.x = p.y = p.b = p.back = true; update();
    assertTrue(c.isOdometryResetRequested()); assertTrue(c.isGateEnabled());
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed()); assertFalse(c.isShooterEnabled());
  }
  @Test public void bumperDoesNotChangeMechanisms() {
    p.left_bumper = q.right_bumper = true; update();
    assertTrue(c.isShooterEnabled()); assertFalse(c.isGateEnabled()); assertFalse(c.isOdometryResetRequested());
  }
  @Test public void dpadStepsOnceAcrossControllersAndOpposingFreshPressesCancel() {
    q.dpad_right = q.dpad_up = true; update(); update();
    p.dpad_right = p.dpad_up = true; update();
    q.dpad_right = q.dpad_up = false; update();
    assertEquals(200, c.getGateRpm(), 0); assertEquals(3250, c.getShooterRpm(), 0);
    p.dpad_right = p.dpad_up = false; update();
    p.dpad_left = p.dpad_down = q.dpad_right = q.dpad_up = true; update();
    assertEquals(200, c.getGateRpm(), 0); assertEquals(3250, c.getShooterRpm(), 0);
  }
  @Test public void rpmSelectionSurvivesMechanismToggles() {
    p.back = p.dpad_down = p.dpad_left = true; update();
    assertFalse(c.isShooterEnabled()); assertFalse(c.isGateEnabled());
    p.back = p.dpad_down = p.dpad_left = false; update();
    q.back = q.x = true; update();
    assertTrue(c.isShooterEnabled()); assertTrue(c.isGateEnabled());
    assertEquals(3150, c.getShooterRpm(), 0); assertEquals(180, c.getGateRpm(), 0);
  }
  @Test public void speedsClampToMotorRatingsAndZero() {
    for (int i=0; i<150; i++) {
      p.dpad_up = p.dpad_right = true; update(); p.dpad_up = p.dpad_right = false; update();
    }
    assertEquals(6000, c.getShooterRpm(), 0); assertEquals(435, c.getGateRpm(), 0);
    for (int i=0; i<150; i++) {
      q.dpad_down = q.dpad_left = true; update(); q.dpad_down = q.dpad_left = false; update();
    }
    assertEquals(0, c.getShooterRpm(), 0); assertEquals(0, c.getGateRpm(), 0);
  }
}
