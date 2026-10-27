package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzAutoRoutes.Plan;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;
import org.junit.Test;

public final class AutonomousStartControlsTest {
  private static void release(AutonomousStartControls controls) {
    controls.update(false, false, false, false, false, false, false);
  }

  @Test
  public void defaultIsPoseOnlyAndUnconfirmed() {
    AutonomousStartControls controls = new AutonomousStartControls();
    assertEquals(Alliance.RED, controls.getAlliance());
    assertEquals(StartLocation.RED_A3_WEST, controls.getStart());
    assertEquals(Plan.POSE_ONLY, controls.getPlan());
    assertFalse(controls.isConfirmed());
  }

  @Test
  public void confirmationRequiresSeparateFreshPressAfterSelectionChange() {
    AutonomousStartControls controls = new AutonomousStartControls();
    controls.update(false, true, false, false, false, false, true);
    assertEquals(Alliance.BLUE, controls.getAlliance());
    assertFalse(controls.isConfirmed());
    controls.update(false, false, false, false, false, false, true);
    assertFalse(controls.isConfirmed());
    release(controls);
    controls.update(false, false, false, false, false, false, true);
    assertTrue(controls.isConfirmed());
  }

  @Test
  public void changingAllianceResetsSlotAndInvalidatesConfirmation() {
    AutonomousStartControls controls = new AutonomousStartControls();
    controls.update(false, false, true, false, false, false, false);
    release(controls);
    controls.update(false, false, false, false, false, false, true);
    assertTrue(controls.isConfirmed());
    controls.update(false, true, false, false, false, false, false);
    assertEquals(StartLocation.BLUE_F4_EAST, controls.getStart());
    assertFalse(controls.isConfirmed());
  }

  @Test
  public void heldDpadDoesNotCycleAndSlotsWrap() {
    AutonomousStartControls controls = new AutonomousStartControls();
    controls.update(false, false, false, true, false, false, false);
    assertEquals(StartLocation.RED_B6_FAR, controls.getStart());
    controls.update(false, false, false, true, false, false, false);
    assertEquals(StartLocation.RED_B6_FAR, controls.getStart());
    release(controls);
    controls.update(false, false, true, false, false, false, false);
    assertEquals(StartLocation.RED_A3_WEST, controls.getStart());
  }

  @Test
  public void changingPlanInvalidatesConfirmationAndHeldBumperDoesNotToggleAgain() {
    AutonomousStartControls controls = new AutonomousStartControls();
    controls.update(false, false, false, false, false, false, true);
    controls.update(false, false, false, false, false, true, false);
    assertEquals(Plan.LEAVE_WALL_12_INCHES, controls.getPlan());
    assertFalse(controls.isConfirmed());
    controls.update(false, false, false, false, false, true, false);
    assertEquals(Plan.LEAVE_WALL_12_INCHES, controls.getPlan());
    release(controls);
    controls.update(false, false, false, false, true, false, false);
    assertEquals(Plan.POSE_ONLY, controls.getPlan());
  }
}
