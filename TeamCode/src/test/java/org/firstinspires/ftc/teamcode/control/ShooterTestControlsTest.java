package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;
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

  @Test public void rpmChangesOncePerPressWhileStopped() {
    ShooterTestControls c = new ShooterTestControls();
    assertEquals(6000, c.getTargetRpm(), 0);
    c.updateButtons(false, false, false, false, false, true, false, false);
    assertEquals(5900, c.getTargetRpm(), 0);
    c.updateButtons(false, false, false, false, false, true, false, false);
    assertEquals(5900, c.getTargetRpm(), 0);
    release(c);
    c.updateButtons(false, false, false, false, true, false, false, false);
    assertEquals(6000, c.getTargetRpm(), 0);
    assertFalse(c.isActive());
  }

  @Test public void gamepad2RpmAndOverlappingPressesAreCombined() {
    ShooterTestControls c = new ShooterTestControls();
    c.updateButtons(false, false, false, false, false, false, false, true);
    assertEquals(5900, c.getTargetRpm(), 0);
    c.updateButtons(false, false, false, false, false, true, false, true);
    c.updateButtons(false, false, false, false, false, true, false, false);
    assertEquals(5900, c.getTargetRpm(), 0);
    release(c);
    c.updateButtons(false, false, false, false, false, false, true, false);
    assertEquals(6000, c.getTargetRpm(), 0);
  }

  @Test public void rpmClampsAtBothLimitsAndOpposingFreshPressesCancel() {
    ShooterTestControls c = new ShooterTestControls();
    c.updateButtons(false, false, false, false, true, false, false, false);
    assertEquals(6000, c.getTargetRpm(), 0);
    release(c);
    c.updateButtons(false, false, false, false, true, true, false, false);
    assertEquals(6000, c.getTargetRpm(), 0);
    for (int i = 0; i < 65; i++) {
      release(c);
      c.updateButtons(false, false, false, false, false, true, false, false);
    }
    assertEquals(0, c.getTargetRpm(), 0);
    release(c);
    c.updateButtons(false, false, false, false, false, false, true, true);
    assertEquals(0, c.getTargetRpm(), 0);
  }

  @Test public void rpmAdjustmentDoesNotInterfereWithOnOffOrReverse() {
    ShooterTestControls c = new ShooterTestControls();
    c.updateButtons(true, true, false, false, false, true, false, false);
    assertTrue(c.isActive());
    assertTrue(c.isReversed());
    assertEquals(5900, c.getTargetRpm(), 0);
    release(c);
    c.updateButtons(true, false, false, false, false, false, false, false);
    assertFalse(c.isActive());
    assertEquals(5900, c.getTargetRpm(), 0);
  }
}
