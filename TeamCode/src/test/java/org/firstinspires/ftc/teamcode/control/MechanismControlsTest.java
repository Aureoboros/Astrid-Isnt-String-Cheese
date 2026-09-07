package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.control.MechanismControls.Actions;
import org.junit.Test;

public final class MechanismControlsTest {
  @Test
  public void toggleButtonsFireOnlyOnRisingEdge() {
    MechanismControls controls = new MechanismControls();

    Actions firstPress = controls.readButtons(true, true, false, true);
    Actions held = controls.readButtons(true, true, false, true);
    controls.readButtons(false, false, false, false);
    Actions secondPress = controls.readButtons(true, true, false, true);

    assertTrue(firstPress.shouldToggleIntakeEnabled());
    assertTrue(firstPress.shouldToggleServoEnabled());
    assertTrue(firstPress.shouldToggleIntakeDirection());
    assertFalse(held.shouldToggleIntakeEnabled());
    assertFalse(held.shouldToggleServoEnabled());
    assertFalse(held.shouldToggleIntakeDirection());
    assertTrue(secondPress.shouldToggleIntakeEnabled());
    assertTrue(secondPress.shouldToggleServoEnabled());
    assertTrue(secondPress.shouldToggleIntakeDirection());
  }

  @Test
  public void shooterIsActiveOnlyWhileXIsHeld() {
    MechanismControls controls = new MechanismControls();

    assertTrue(controls.readButtons(false, false, true, false).shouldShoot());
    assertTrue(controls.readButtons(false, false, true, false).shouldShoot());
    assertFalse(controls.readButtons(false, false, false, false).shouldShoot());
  }
}
