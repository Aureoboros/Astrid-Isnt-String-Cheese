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

  @Test public void gamepadMappingUsesAAndBOnly() {
    GateMotorTestControls c = new GateMotorTestControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.x = first.y = first.dpad_up = true;
    c.update(first, second);
    assertFalse(c.isActive()); assertFalse(c.isReversed());
    second.a = second.b = true; c.update(first, second);
    assertTrue(c.isActive()); assertTrue(c.isReversed());
  }
}
