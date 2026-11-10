package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Test;

public final class ShooterTestControlsTest {
  @Test
  public void startsOffAndForward() {
    ShooterTestControls controls = new ShooterTestControls();
    assertFalse(controls.isActive());
    assertFalse(controls.isReversed());
  }

  @Test
  public void xTogglesOnlyOnFreshPressAndReleaseDoesNotStop() {
    ShooterTestControls controls = new ShooterTestControls();
    controls.updateButtons(true, false, false, false);
    assertTrue(controls.isActive());
    controls.updateButtons(true, false, false, false);
    assertTrue(controls.isActive());
    release(controls);
    assertTrue(controls.isActive());
    controls.updateButtons(true, false, false, false);
    assertFalse(controls.isActive());
  }

  @Test
  public void yTogglesOnlyOnFreshPressWithoutStartingMotor() {
    ShooterTestControls controls = new ShooterTestControls();
    controls.updateButtons(false, true, false, false);
    assertTrue(controls.isReversed());
    assertFalse(controls.isActive());
    controls.updateButtons(false, true, false, false);
    assertTrue(controls.isReversed());
    release(controls);
    controls.updateButtons(false, true, false, false);
    assertFalse(controls.isReversed());
  }

  @Test
  public void gamepad2CanOperateBothTogglesAndReverseWhileOn() {
    ShooterTestControls controls = new ShooterTestControls();
    controls.updateButtons(false, false, true, false);
    assertTrue(controls.isActive());
    release(controls);
    controls.updateButtons(false, false, false, true);
    assertTrue(controls.isActive());
    assertTrue(controls.isReversed());
    release(controls);
    controls.updateButtons(false, false, true, false);
    assertFalse(controls.isActive());
    assertTrue(controls.isReversed());
  }

  @Test
  public void simultaneousAndOverlappingControllerPressesDoNotDoubleToggle() {
    ShooterTestControls controls = new ShooterTestControls();
    controls.updateButtons(true, true, true, true);
    assertTrue(controls.isActive());
    assertTrue(controls.isReversed());
    controls.updateButtons(false, false, true, true);
    controls.updateButtons(true, true, true, true);
    assertTrue(controls.isActive());
    assertTrue(controls.isReversed());
    release(controls);
    controls.updateButtons(false, false, true, true);
    assertFalse(controls.isActive());
    assertFalse(controls.isReversed());
  }

  private static void release(ShooterTestControls controls) {
    controls.updateButtons(false, false, false, false);
  }
}
