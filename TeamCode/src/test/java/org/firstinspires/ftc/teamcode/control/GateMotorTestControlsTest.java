package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;

public final class GateMotorTestControlsTest {
  private void release(GateMotorTestControls c) { c.updateButtons(false, false, false, false); }

  @Test public void startsStoppedAndReverse() {
    GateMotorTestControls c = new GateMotorTestControls();
    assertFalse(c.isActive()); assertTrue(c.isReversed());
    assertEquals(190, c.getGateRpm(), 0);
  }

  @Test public void aSwapsDirectionOnFreshPressWithoutStartingMotor() {
    GateMotorTestControls c = new GateMotorTestControls();
    c.updateButtons(true, false, false, false);
    assertFalse(c.isReversed()); assertFalse(c.isActive());
    c.updateButtons(true, false, false, false);
    assertFalse(c.isReversed());
    release(c); c.updateButtons(true, false, false, false);
    assertTrue(c.isReversed());
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
    assertTrue(c.isActive()); assertFalse(c.isReversed());
    release(c); c.updateButtons(false, false, true, true);
    assertFalse(c.isActive()); assertTrue(c.isReversed());
  }

  @Test public void gamepadMappingUsesAAndBOnly() {
    GateMotorTestControls c = new GateMotorTestControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.x = first.y = first.dpad_up = true;
    c.update(first, second);
    assertFalse(c.isActive()); assertTrue(c.isReversed());
    second.a = second.b = true; c.update(first, second);
    assertTrue(c.isActive()); assertFalse(c.isReversed());
  }
}
