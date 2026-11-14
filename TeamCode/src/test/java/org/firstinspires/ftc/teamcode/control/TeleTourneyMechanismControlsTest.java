package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;

public final class TeleTourneyMechanismControlsTest {
  private static void release(TeleTourneyMechanismControls controls) {
    controls.updateButtons(false, false, false, false, false, false, false, false);
  }

  @Test public void startsStoppedWithSixThousandRpmSelected() {
    TeleTourneyMechanismControls controls = new TeleTourneyMechanismControls();
    assertFalse(controls.isIntakeEnabled());
    assertFalse(controls.isIntakeReversed());
    assertFalse(controls.isShooterEnabled());
    assertEquals(6000, controls.getShooterRpm(), 0);
  }

  @Test public void intakeCyclesForwardStopReverseStopAndRepeatsWithoutHoldRepeats() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    boolean[] enabled = {true, false, true, false, true, false, true, false};
    boolean[] reversed = {false, true, true, false, false, true, true, false};
    for (int i = 0; i < enabled.length; i++) {
      // Alternate the operator; the intake cycle belongs to the robot, not one controller.
      boolean first = i % 2 == 0;
      for (int heldLoop = 0; heldLoop < 3; heldLoop++) {
        c.updateButtons(false, first, false, false, false, !first, false, false);
        assertEquals(enabled[i], c.isIntakeEnabled());
        assertEquals(reversed[i], c.isIntakeReversed());
        assertFalse(c.isShooterEnabled());
        assertEquals(6000, c.getShooterRpm(), 0);
      }
      release(c);
      assertEquals(enabled[i], c.isIntakeEnabled());
    }
  }

  @Test public void overlappingYDoesNotSkipStopOrAdvanceReverseState() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateButtons(false, true, false, false, false, false, false, false);
    c.updateButtons(false, true, false, false, false, true, false, false);
    c.updateButtons(false, false, false, false, false, true, false, false);
    assertTrue(c.isIntakeEnabled());
    assertFalse(c.isIntakeReversed());
    release(c);
    c.updateButtons(false, false, false, false, false, true, false, false);
    assertFalse(c.isIntakeEnabled());
    release(c);
    c.updateButtons(false, true, false, false, false, true, false, false);
    assertTrue(c.isIntakeEnabled());
    assertTrue(c.isIntakeReversed());
  }

  @Test public void xAndYIndependentlyToggleOnlyOnFreshPresses() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateButtons(true, false, false, false, false, false, false, false);
    assertTrue(c.isShooterEnabled());
    assertFalse(c.isIntakeEnabled());
    c.updateButtons(true, true, false, false, false, false, false, false);
    c.updateButtons(true, true, false, false, false, false, false, false);
    release(c);
    assertTrue(c.isIntakeEnabled());
    assertTrue(c.isShooterEnabled());
    c.updateButtons(false, true, false, false, false, false, false, false);
    assertFalse(c.isIntakeEnabled());
    assertTrue(c.isShooterEnabled());
    release(c);
    c.updateButtons(true, false, false, false, false, false, false, false);
    assertFalse(c.isShooterEnabled());
  }

  @Test public void gamepad2WorksAndOverlappingInputsDoNotDoubleToggleOrStep() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateButtons(false, false, false, false, true, true, false, true);
    assertTrue(c.isShooterEnabled());
    assertTrue(c.isIntakeEnabled());
    assertEquals(5900, c.getShooterRpm(), 0);
    c.updateButtons(true, true, false, true, true, true, false, true);
    c.updateButtons(true, true, false, true, false, false, false, false);
    assertTrue(c.isShooterEnabled());
    assertTrue(c.isIntakeEnabled());
    assertEquals(5900, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(false, false, false, false, true, true, true, false);
    assertFalse(c.isShooterEnabled());
    assertFalse(c.isIntakeEnabled());
    assertEquals(6000, c.getShooterRpm(), 0);
  }

  @Test public void rpmChangesWhileOffAndOnAndSurvivesStopToggle() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateButtons(false, false, false, true, false, false, false, false);
    assertEquals(5900, c.getShooterRpm(), 0);
    assertFalse(c.isShooterEnabled());
    release(c);
    c.updateButtons(true, false, false, true, false, false, false, false);
    assertTrue(c.isShooterEnabled());
    assertEquals(5800, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(true, false, false, false, false, false, false, false);
    assertFalse(c.isShooterEnabled());
    assertEquals(5800, c.getShooterRpm(), 0);
  }

  @Test public void limitsAreEnforcedAndOpposingFreshPressesCancel() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateButtons(false, false, true, false, false, false, false, false);
    assertEquals(6000, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(false, false, true, false, false, false, false, true);
    assertEquals(6000, c.getShooterRpm(), 0);
    for (int i = 0; i < 65; i++) {
      release(c);
      c.updateButtons(false, false, false, true, false, false, false, false);
    }
    assertEquals(0, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(false, false, true, true, false, false, false, false);
    assertEquals(0, c.getShooterRpm(), 0);
  }

  @Test public void publicGamepadMappingIgnoresValidationButtons() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.a = first.b = true;
    c.update(first, second);
    assertFalse(c.isIntakeEnabled());
    assertFalse(c.isShooterEnabled());
    second.x = second.y = second.dpad_down = true;
    c.update(first, second);
    assertTrue(c.isIntakeEnabled());
    assertTrue(c.isShooterEnabled());
    assertEquals(5900, c.getShooterRpm(), 0);
  }
}
