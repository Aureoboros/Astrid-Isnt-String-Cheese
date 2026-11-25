package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;

public final class TeleTourneyMechanismControlsTest {
  private static void release(TeleTourneyMechanismControls controls) {
    controls.updateButtons(false, false, false, false, false, false, false, false);
  }

  @Test public void startsStoppedWith4800RpmAndReverseSelected() {
    TeleTourneyMechanismControls controls = new TeleTourneyMechanismControls();
    assertFalse(controls.isIntakeEnabled());
    assertFalse(controls.isIntakeReversed());
    assertFalse(controls.isShooterEnabled());
    assertFalse(controls.isGateEnabled());
    assertEquals(60, controls.getGateRpm(), 0);
    assertEquals(4800, controls.getShooterRpm(), 0);
    assertTrue(controls.isShooterReversed());
  }

  @Test public void yOnlyTogglesOnOffWithoutChangingDirectionOrRepeatingWhileHeld() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    boolean[] enabled = {true, false, true, false, true, false, true, false};
    for (int i = 0; i < enabled.length; i++) {
      // Alternate the operator; both controllers share the intake state.
      boolean first = i % 2 == 0;
      for (int heldLoop = 0; heldLoop < 3; heldLoop++) {
        c.updateButtons(false, first, false, false, false, !first, false, false);
        assertEquals(enabled[i], c.isIntakeEnabled());
        assertFalse(c.isIntakeReversed());
        assertFalse(c.isShooterEnabled());
        assertEquals(4800, c.getShooterRpm(), 0);
      }
      release(c);
      assertEquals(enabled[i], c.isIntakeEnabled());
    }
  }

  @Test public void overlappingYDoesNotDoubleToggleOrChangeDirection() {
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
    assertFalse(c.isIntakeReversed());
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
    assertEquals(4750, c.getShooterRpm(), 0);
    c.updateButtons(true, true, false, true, true, true, false, true);
    c.updateButtons(true, true, false, true, false, false, false, false);
    assertTrue(c.isShooterEnabled());
    assertTrue(c.isIntakeEnabled());
    assertEquals(4750, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(false, false, false, false, true, true, true, false);
    assertFalse(c.isShooterEnabled());
    assertFalse(c.isIntakeEnabled());
    assertEquals(4800, c.getShooterRpm(), 0);
  }

  @Test public void rpmChangesWhileOffAndOnAndSurvivesStopToggle() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateButtons(false, false, false, true, false, false, false, false);
    assertEquals(4750, c.getShooterRpm(), 0);
    assertFalse(c.isShooterEnabled());
    release(c);
    c.updateButtons(true, false, false, true, false, false, false, false);
    assertTrue(c.isShooterEnabled());
    assertEquals(4700, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(true, false, false, false, false, false, false, false);
    assertFalse(c.isShooterEnabled());
    assertEquals(4700, c.getShooterRpm(), 0);
    assertTrue(c.isShooterReversed());
  }

  @Test public void limitsAreEnforcedAndOpposingFreshPressesCancel() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    for (int i = 0; i < 30; i++) {
      release(c);
      c.updateButtons(false, false, true, false, false, false, false, false);
    }
    assertEquals(6000, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(false, false, true, false, false, false, false, true);
    assertEquals(6000, c.getShooterRpm(), 0);
    for (int i = 0; i < 125; i++) {
      release(c);
      c.updateButtons(false, false, false, true, false, false, false, false);
    }
    assertEquals(0, c.getShooterRpm(), 0);
    release(c);
    c.updateButtons(false, false, true, true, false, false, false, false);
    assertEquals(0, c.getShooterRpm(), 0);
  }

  @Test public void publicGamepadMappingHandlesIntakeAAndGateBIndependently() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.a = first.b = true;
    c.update(first, second);
    assertFalse(c.isIntakeEnabled());
    assertTrue(c.isIntakeReversed());
    assertFalse(c.isShooterEnabled());
    assertTrue(c.isGateEnabled());
    second.x = second.y = second.dpad_down = true;
    c.update(first, second);
    assertTrue(c.isIntakeEnabled());
    assertTrue(c.isShooterEnabled());
    assertEquals(4750, c.getShooterRpm(), 0);
    assertTrue(c.isGateEnabled());
  }

  @Test public void bTogglesGateOncePerPressIncludingOverlappingControllers() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    c.updateGateButton(false, true);
    assertTrue(c.isGateEnabled());
    c.updateGateButton(true, true);
    c.updateGateButton(true, false);
    assertTrue(c.isGateEnabled());
    c.updateGateButton(false, false);
    c.updateGateButton(true, false);
    assertFalse(c.isGateEnabled());
    assertFalse(c.isIntakeEnabled());
    assertFalse(c.isShooterEnabled());
  }

  @Test public void gateControlsWorkIndependentlyOfShooterIntakeAndValidation() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    second.b = second.x = second.y = true;
    c.update(first, second);
    assertTrue(c.isGateEnabled()); assertTrue(c.isShooterEnabled()); assertTrue(c.isIntakeEnabled());
    second.b = second.x = second.y = false;
    second.left_bumper = second.back = true;
    c.update(first, second);
    assertTrue(c.isGateEnabled()); assertTrue(c.isShooterEnabled()); assertTrue(c.isIntakeEnabled());
    first.b = true;
    c.update(first, second);
    assertFalse(c.isGateEnabled());
    assertTrue(c.isShooterEnabled()); assertTrue(c.isIntakeEnabled());
  }

  @Test public void aReversesRunningIntakeAndHeldOverlappingButtonsToggleOnlyOnce() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.y = true; c.update(first, second);
    first.y = false; c.update(first, second);
    second.a = true; c.update(first, second);
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    first.a = true; c.update(first, second);
    second.a = false; c.update(first, second);
    assertTrue(c.isIntakeReversed());
    first.a = false; c.update(first, second);
    first.a = true; c.update(first, second);
    assertTrue(c.isIntakeEnabled()); assertFalse(c.isIntakeReversed());
    assertFalse(c.isGateEnabled()); assertFalse(c.isShooterEnabled());
  }

  @Test public void aWhileStoppedSelectsReverseForNextYWithoutStartingMotor() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.a = true; c.update(first, second);
    assertFalse(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    first.a = false; c.update(first, second);
    second.y = true; c.update(first, second);
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    second.y = false; c.update(first, second);
    second.y = true; c.update(first, second);
    assertFalse(c.isIntakeEnabled());
    assertTrue(c.isIntakeReversed());
    second.y = false; c.update(first, second);
    first.y = true; c.update(first, second);
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
  }

  @Test public void simultaneousYAndAStartsInReverseAndBumperDoesNotChangeIntake() {
    TeleTourneyMechanismControls c = new TeleTourneyMechanismControls();
    Gamepad first = new Gamepad(), second = new Gamepad();
    first.y = second.a = true; c.update(first, second);
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    first.y = second.a = false;
    first.left_bumper = second.back = true; c.update(first, second);
    assertTrue(c.isIntakeEnabled()); assertTrue(c.isIntakeReversed());
    assertFalse(c.isGateEnabled()); assertFalse(c.isShooterEnabled());
  }
}
