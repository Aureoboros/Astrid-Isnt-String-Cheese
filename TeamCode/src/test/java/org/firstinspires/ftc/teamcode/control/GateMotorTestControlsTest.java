package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;

public final class GateMotorTestControlsTest {
  private void release(GateMotorTestControls c) { c.updateButtons(false, false, false, false); }

  @Test public void startsStoppedAndForward() {
    GateMotorTestControls c = new GateMotorTestControls();
    assertFalse(c.isActive()); assertFalse(c.isReversed());
  }

  @Test public void aSwapsDirectionOnFreshPressWithoutStartingMotor() {
    GateMotorTestControls c = new GateMotorTestControls();
    c.updateButtons(true, false, false, false);
    assertTrue(c.isReversed()); assertFalse(c.isActive());
    c.updateButtons(true, false, false, false);
    assertTrue(c.isReversed());
    release(c); c.updateButtons(true, false, false, false);
    assertFalse(c.isReversed());
  }

  @Test public void bStartsAndStopsOnFreshPressNotOnReleaseOrHold() {
    GateMotorTestControls c = new GateMotorTestControls();
    c.updateButtons(false, true, false, false);
    assertTrue(c.isActive());
    c.updateButtons(false, true, false, false);
    release(c); assertTrue(c.isActive());
    c.updateButtons(false, true, false, false);
    assertFalse(c.isActive());
  }

  @Test public void overlappingControllersDoNotDoubleToggleAndBothButtonsWorkTogether() {
    GateMotorTestControls c = new GateMotorTestControls();
    c.updateButtons(true, true, true, true);
    c.updateButtons(false, false, true, true);
    c.updateButtons(true, true, true, true);
    assertTrue(c.isActive()); assertTrue(c.isReversed());
    release(c); c.updateButtons(false, false, true, true);
    assertFalse(c.isActive()); assertFalse(c.isReversed());
  }

  @Test public void gateToggleMappingUsesAAndBOnly() {
    GateMotorTestControls c = new GateMotorTestControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.x = first.y = first.dpad_up = true;
    c.update(first, second);
    assertFalse(c.isActive()); assertFalse(c.isReversed());
    second.a = second.b = true; c.update(first, second);
    assertTrue(c.isActive()); assertTrue(c.isReversed());
  }

  @Test public void startsAtConfiguredGateAndShooterRpm() {
    GateMotorTestControls c = new GateMotorTestControls();
    assertEquals(200, c.getGateRpm(), 0);
    assertEquals(4500, c.getShooterRpm(), 0);
  }

  @Test public void dpadUpDownStepsShooterOnFreshPressesOnly() {
    GateMotorTestControls c = new GateMotorTestControls();
    c.updateDpad(true, false, false, false);
    c.updateDpad(true, false, false, false);
    assertEquals(4550, c.getShooterRpm(), 0);
    c.updateDpad(false, false, false, false);
    c.updateDpad(false, true, false, false);
    c.updateDpad(false, false, false, false);
    c.updateDpad(false, true, false, false);
    assertEquals(4450, c.getShooterRpm(), 0);
    assertEquals(200, c.getGateRpm(), 0);
  }

  @Test public void dpadLeftRightStepsGateOnFreshPressesOnly() {
    GateMotorTestControls c = new GateMotorTestControls();
    c.updateDpad(false, false, false, true);
    c.updateDpad(false, false, false, true);
    assertEquals(225, c.getGateRpm(), 0);
    c.updateDpad(false, false, true, false);
    c.updateDpad(false, false, false, false);
    c.updateDpad(false, false, true, false);
    assertEquals(175, c.getGateRpm(), 0);
    assertEquals(4500, c.getShooterRpm(), 0);
  }

  @Test public void rpmClampsBetweenZeroAndMotorRatings() {
    GateMotorTestControls c = new GateMotorTestControls();
    for (int i = 0; i < 100; i++) { c.updateDpad(true, false, false, true); c.updateDpad(false, false, false, false); }
    assertEquals(6000, c.getShooterRpm(), 0); assertEquals(435, c.getGateRpm(), 0);
    for (int i = 0; i < 200; i++) { c.updateDpad(false, true, true, false); c.updateDpad(false, false, false, false); }
    assertEquals(0, c.getShooterRpm(), 0); assertEquals(0, c.getGateRpm(), 0);
  }

  @Test public void eitherControllersDpadWorks() {
    GateMotorTestControls c = new GateMotorTestControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    second.dpad_up = true; first.dpad_right = true;
    c.update(first, second);
    assertEquals(4550, c.getShooterRpm(), 0); assertEquals(225, c.getGateRpm(), 0);
  }

  @Test public void xTogglesShooterOnFreshPressesStartingOn() {
    GateMotorTestControls c = new GateMotorTestControls();
    assertTrue(c.isShooterActive());
    c.updateShooterToggle(true); c.updateShooterToggle(true);
    assertFalse(c.isShooterActive());
    c.updateShooterToggle(false); c.updateShooterToggle(true);
    assertTrue(c.isShooterActive());
    assertFalse(c.isActive());
  }

  @Test public void eitherControllersXTogglesShooter() {
    GateMotorTestControls c = new GateMotorTestControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    second.x = true; c.update(first, second);
    assertFalse(c.isShooterActive());
  }
}
