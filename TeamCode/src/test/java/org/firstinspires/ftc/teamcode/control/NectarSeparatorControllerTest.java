package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.junit.Test;

public final class NectarSeparatorControllerTest {
  private NectarSeparatorController gate(int ticks) {
    return new NectarSeparatorController(ticks, 2, 150, 1000, 0.25, 0.1, 0.05, 1);
  }
  private void spinup(NectarSeparatorController c, int position, double rpm) {
    c.update(true, rpm, rpm, position, 0);
    c.update(true, rpm, rpm, position, 0.3);
    assertEquals(NectarSeparatorController.State.INDEXING, c.getState());
  }

  @Test public void offHasNoPositionCommandAndSignedRpmMustBeStableBeforeFeeding() {
    NectarSeparatorController c = gate(100);
    c.update(false, 4800, 4800, 20, 0);
    assertFalse(c.shouldHoldPosition());
    c.update(true, -4800, 4800, 20, 0.1);
    c.update(true, -4800, 4800, 20, 1);
    assertEquals(NectarSeparatorController.State.WAIT_FOR_RPM, c.getState());
    c.update(true, -4800, -4800, 20, 1.1);
    c.update(true, -4800, -4800, 20, 1.3);
    assertEquals(20, c.getTargetTicks());
    c.update(true, -4800, -4800, 20, 1.4);
    assertEquals(120, c.getTargetTicks());
  }

  @Test public void oneCompletedIndexPausesAndWaitsForFreshRecoveryBeforeNext() {
    NectarSeparatorController c = gate(100); spinup(c, 20, 4800);
    c.update(true, 4800, 4800, 120, 0.4);
    c.update(true, 4800, 4800, 120, 0.46);
    assertEquals(1, c.getCompletedIndexes());
    assertEquals(NectarSeparatorController.State.RECOVERING, c.getState());
    c.update(true, 4800, 4000, 120, 0.7);
    assertEquals(120, c.getTargetTicks());
    c.update(true, 4800, 4800, 120, 0.8);
    c.update(true, 4800, 4800, 120, 1.06);
    assertEquals(220, c.getTargetTicks());
    assertEquals(1, c.getCompletedIndexes());
  }

  @Test public void targetRpmChangesResetStabilityAndNegativeGateIndexWorks() {
    NectarSeparatorController c = gate(-100);
    c.update(true, -4800, -4800, 20, 0);
    c.update(true, -4850, -4850, 20, 0.2);
    c.update(true, -4850, -4850, 20, 0.4);
    assertEquals(20, c.getTargetTicks());
    c.update(true, -4850, -4850, 20, 0.5);
    assertEquals(-80, c.getTargetTicks());
  }

  @Test public void stoppingMidIndexLatchesFaultAndNeverBlindlyResumes() {
    NectarSeparatorController c = gate(100); spinup(c, 0, 4800);
    c.update(false, 0, 4000, 50, 0.4);
    assertEquals(NectarSeparatorController.State.FAULT, c.getState());
    assertFalse(c.shouldHoldPosition());
    c.update(true, 4800, 4800, 50, 5);
    assertEquals(NectarSeparatorController.State.FAULT, c.getState());
    assertEquals(0, c.getCompletedIndexes());
  }

  @Test public void stableBlockCanStopAndRestartWithoutChangingEncoderOrigin() {
    NectarSeparatorController c = gate(100);
    c.update(true, 4800, 4700, 25, 0);
    c.update(false, 0, 0, 25, 0.1);
    assertEquals(NectarSeparatorController.State.OFF, c.getState());
    c.update(true, 4800, 4800, 25, 1);
    c.update(true, 4800, 4800, 25, 1.3);
    assertEquals(125, c.getTargetTicks());
  }

  @Test public void stallShooterLossOrRetargetDuringIndexStopsAndDoesNotCountRelease() {
    for (int failure = 0; failure < 3; failure++) {
      NectarSeparatorController c = gate(100); spinup(c, 0, 4800);
      if (failure == 0) { c.update(true, 4800, 4800, 0, 1.4); }
      if (failure == 1) { c.update(true, 4800, 3000, 40, 0.4); }
      if (failure == 2) { c.update(true, 4850, 4850, 40, 0.4); }
      assertEquals(NectarSeparatorController.State.FAULT, c.getState());
      assertFalse(c.shouldHoldPosition()); assertEquals(0, c.getCompletedIndexes());
    }
  }

  @Test public void blockDriftZeroTargetInvalidRpmAndTimerAreHandledSafely() {
    NectarSeparatorController c = gate(100);
    c.update(true, 0, 0, 0, 0);
    c.update(true, Double.NaN, Double.NaN, 0, 1);
    assertEquals(NectarSeparatorController.State.WAIT_FOR_RPM, c.getState());
    c.update(true, 4800, 4800, 10, 2);
    assertEquals(NectarSeparatorController.State.FAULT, c.getState());
    c = gate(100); c.update(true, 4800, 4800, 0, Double.NaN);
    assertFalse(c.shouldHoldPosition());
    assertEquals(NectarSeparatorController.State.FAULT, c.getState());
  }

  @Test public void encoderOverflowIsAStoppedFault() {
    NectarSeparatorController c = gate(100);
    c.update(true, 4800, 4800, Integer.MAX_VALUE - 5, 0);
    c.update(true, 4800, 4800, Integer.MAX_VALUE - 5, 0.3);
    assertEquals(NectarSeparatorController.State.FAULT, c.getState());
  }

  @Test public void minimumIntervalCannotBeBypassedByTurningShooterOffAndOn() {
    NectarSeparatorController c = new NectarSeparatorController(100, 2, 150, 1000, 0.25, 1, 0.05, 1);
    spinup(c, 0, 4800);
    c.update(true, 4800, 4800, 100, 0.4);
    c.update(true, 4800, 4800, 100, 0.46);
    c.update(false, 0, 0, 100, 0.5);
    c.update(true, 4800, 4800, 100, 0.6);
    c.update(true, 4800, 4800, 100, 0.9);
    assertEquals(100, c.getTargetTicks());
    c.update(true, 4800, 4800, 100, 1.5);
    assertEquals(200, c.getTargetTicks());
  }

  @Test(expected = IllegalArgumentException.class) public void uncalibratedZeroIndexRejected() { gate(0); }
}
