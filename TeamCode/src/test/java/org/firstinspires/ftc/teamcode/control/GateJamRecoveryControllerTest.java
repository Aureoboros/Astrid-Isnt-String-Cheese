package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.control.GateJamRecoveryController.State;

public final class GateJamRecoveryControllerTest {
  private final GateJamRecoveryController control = new GateJamRecoveryController(
      384.5, .4, .5, .2, .1, .15, .1, 60, 2, 5, 3);
  private double step(double command, double rpm, int ticks, double now) {
    return control.update(command, rpm, ticks, now);
  }
  private void jam() {
    step(60, 0, 0, 0);
    step(60, 0, 0, .91);
    assertEquals(State.STOPPING, control.getState());
  }
  @Test public void waitsForStartupGraceAndFullStallWindow() {
    assertEquals(60, step(60, 0, 0, 0), 0);
    assertEquals(60, step(60, 0, 0, .39), 0);
    assertEquals(60, step(60, 0, 0, .8), 0);
    assertEquals(0, step(60, 0, 0, .91), 0);
    assertEquals(1, control.getAttempts());
  }
  @Test public void normalMotionAndBriefSpeedDipDoNotTriggerRecovery() {
    step(60, 0, 0, 0);
    step(60, 0, 0, .4);
    step(60, 60, 200, .91);
    step(60, 0, 200, 1.1); // Brief pause shorter than a detection window.
    assertEquals(60, step(60, 60, 400, 1.42), 0);
    assertEquals(State.RUNNING, control.getState());
    assertEquals(0, control.getRecoveryCount());
  }
  @Test public void requiresBothLowRpmAndLowEncoderMovement() {
    step(60, 0, 0, 0);
    assertEquals(60, step(60, 60, 0, .91), 0);
    assertEquals(60, step(60, 0, 400, 1.42), 0);
    assertEquals(0, control.getRecoveryCount());
  }
  @Test public void stopsReversesPausesAndResumesSelectedSpeed() {
    jam();
    assertEquals(0, step(60, 0, 0, .95), 0);
    assertEquals(-60, step(60, 0, 0, 1.02), 0);
    assertEquals(State.REVERSING, control.getState());
    assertEquals(-60, step(60, -40, -20, 1.1), 0);
    assertEquals(0, step(60, -40, -30, 1.18), 0);
    assertEquals(State.RESTARTING, control.getState());
    assertEquals(60, step(60, 0, -30, 1.29), 0);
    assertEquals(State.RUNNING, control.getState());
  }
  @Test public void reversePulseOpposesSelectedDirectionAndDoesNotExceedSelectedSpeed() {
    step(-20, 0, 0, 0);
    step(-20, 0, 0, .91);
    assertEquals(20, step(-20, 0, 0, 1.02), 0);
  }
  @Test public void offCancelsEveryRecoveryPhaseAndClearsAttempts() {
    for (double end : new double[] {.91, 1.02, 1.18}) {
      GateJamRecoveryController c = new GateJamRecoveryController(384.5,.4,.5,.2,.1,.15,.1,60,2,5,3);
      c.update(60,0,0,0); c.update(60,0,0,.91);
      if (end >= 1.02) { c.update(60,0,0,1.02); }
      if (end >= 1.18) { c.update(60,0,0,1.18); }
      assertEquals(0, c.update(0,0,0,end+.01), 0);
      assertEquals(State.OFF, c.getState());
      assertEquals(0, c.getAttempts());
    }
  }
  @Test public void directionOrSpeedChangeCancelsReverseImmediately() {
    jam(); step(60,0,0,1.02);
    assertEquals(-40, step(-40,0,0,1.03), 0);
    assertEquals(State.RUNNING, control.getState());
    assertEquals(1, control.getAttempts());
  }
  @Test public void persistentJamFaultsAfterThreeRetriesAndNeedsOffOnReset() {
    step(60,0,0,0);
    for (int i = 0; i < 3; i++) {
      double base = i * 2;
      assertEquals(0, step(60,0,0,base+.91), 0);
      assertEquals(State.STOPPING, control.getState());
      step(60,0,0,base+1.02); step(60,0,0,base+1.18); step(60,0,0,base+1.29);
    }
    assertEquals(0, step(60,0,0,6.91), 0);
    assertEquals(State.FAULT, control.getState());
    assertEquals(3, control.getRecoveryCount());
    assertEquals(0, step(-40,0,0,7), 0); // Changing direction cannot reset a fault.
    step(0,0,0,7.1);
    assertEquals(60, step(60,0,0,7.2), 0);
    assertEquals(0, control.getAttempts());
  }
  @Test public void sustainedHealthyMovementResetsConsecutiveRetryCounter() {
    jam(); step(60,0,0,1.02); step(60,0,0,1.18); step(60,0,0,1.29);
    for (int i = 0; i < 6; i++) { step(60,60,200*(i+1),2.2+i*.51); }
    assertEquals(0, control.getAttempts());
    assertEquals(1, control.getRecoveryCount());
  }
  @Test public void zeroAndVeryLowTargetsDoNotTriggerFalseStalls() {
    step(0,0,0,0);
    assertEquals(State.OFF, control.getState());
    step(2,0,0,1);
    assertEquals(2, step(2,0,0,10), 0);
    assertEquals(0, control.getRecoveryCount());
  }
  @Test public void invalidFeedbackStopsMotorButOffStillClearsFault() {
    step(60,Double.NaN,0,0);
    assertEquals(State.FAULT, control.getState());
    assertEquals(0, control.getOutputRpm(), 0);
    step(0,Double.NaN,0,.1);
    assertEquals(State.OFF, control.getState());
  }
}
