package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.junit.Test;

public final class HiveShotControllerTest {
  @Test public void newThreeBatchPlanConfirmsFirstAndThirdTipsButNotMiddleBatch() {
    boolean[] requireTip = {true, false, true};
    int totalFed = 0;
    for (boolean required : requireTip) {
      HiveShotController c = new HiveShotController(0, 0.15, 4, required);
      for (int i = 0; i < 500 && !c.isDone(); i++) {
        c.update(i / 100.0, true, true, false, false, true, true);
      }
      assertEquals(4, c.getCompletedPulses());
      if (required) {
        assertFalse(c.isDone());
        c.update(5.1, true, false, true, true, true, true);
      }
      assertEquals(HiveShotController.State.COMPLETE, c.getState());
      assertEquals(required, c.isTipConfirmed());
      assertFalse(c.feederOn());
      totalFed += c.getCompletedPulses();
    }
    assertEquals(12, totalFed);
  }

  private HiveShotController feeding() {
    HiveShotController c = new HiveShotController(0, 0.15);
    c.update(0, true, true, false, false, false, true);
    c.update(0.1, true, true, false, false, true, true);
    c.update(0.4, true, true, false, false, true, true);
    assertTrue(c.feederOn()); return c;
  }
  @Test public void waitsForHighAndStableRpmBeforeFeeding() {
    HiveShotController c = new HiveShotController(0, 0.15);
    c.update(0, true, false, false, false, true, true);
    assertFalse(c.shooterOn()); assertFalse(c.feederOn());
    c = feeding(); assertEquals(1, c.getPulses());
  }
  @Test public void lowEvidenceStopsFeederImmediatelyAndConfirmedTipStopsAll() {
    HiveShotController c = feeding();
    c.update(0.41, true, false, true, false, true, true);
    assertFalse(c.feederOn()); assertFalse(c.shooterOn());
    c.update(0.8, true, false, true, true, true, true);
    assertEquals(HiveShotController.State.COMPLETE, c.getState());
  }
  @Test public void lostVisibilityOrRpmStopsFeed() {
    HiveShotController c = feeding();
    c.update(0.41, true, false, false, false, true, true);
    assertFalse(c.feederOn());
    c = feeding(); c.update(0.41, true, true, false, false, false, true);
    assertFalse(c.feederOn());
  }
  @Test public void disconnectPoseFaultAndTimeoutFailClosed() {
    HiveShotController c = feeding();
    c.update(0.41, false, true, false, false, true, true); assertTrue(c.isDone());
    c = feeding(); c.update(0.41, true, true, false, false, true, false); assertTrue(c.isDone());
    c = feeding(); c.update(6, true, true, false, false, true, true);
    assertEquals(HiveShotController.State.FAILED, c.getState()); assertFalse(c.feederOn());
  }
  @Test public void atMostTwoPulsesAndNoTimedSuccess() {
    HiveShotController c = feeding();
    for (int i = 5; i <= 59; i++) { c.update(i / 10.0, true, true, false, false, true, true); }
    assertEquals(2, c.getPulses()); assertFalse(c.feederOn());
    c.update(6, true, true, false, false, true, true);
    assertEquals(HiveShotController.State.FAILED, c.getState());
  }
  @Test(expected = IllegalArgumentException.class) public void invalidPulseRejected() {
    new HiveShotController(0, Double.NaN);
  }
  @Test public void firstFourShotBatchCompletesWithoutClaimingTip() {
    HiveShotController c = new HiveShotController(0, 0.15, 4, false);
    for (int i = 0; i < 600 && !c.isDone(); i++) {
      c.update(i / 100.0, true, true, false, false, true, true);
    }
    assertEquals(HiveShotController.State.COMPLETE, c.getState());
    assertEquals(4, c.getCompletedPulses()); assertEquals(4, c.getPulses());
    assertFalse(c.isTipConfirmed()); assertFalse(c.feederOn());
  }
  @Test public void finalFourShotBatchMustObserveTip() {
    HiveShotController c = new HiveShotController(0, 0.15, 4, true);
    for (int i = 0; i < 500; i++) { c.update(i / 100.0, true, true, false, false, true, true); }
    assertEquals(4, c.getCompletedPulses()); assertFalse(c.isDone());
    c.update(5.1, true, false, true, true, true, true);
    assertEquals(HiveShotController.State.COMPLETE, c.getState()); assertTrue(c.isTipConfirmed());
  }
  @Test public void finalBatchWithoutTipTimesOutDespiteFourCompletedPulses() {
    HiveShotController c = new HiveShotController(0, 0.15, 4, true);
    for (int i = 0; i <= 600; i++) { c.update(i / 100.0, true, true, false, false, true, true); }
    assertEquals(4, c.getCompletedPulses()); assertEquals(HiveShotController.State.FAILED, c.getState());
    assertFalse(c.isTipConfirmed());
  }
  @Test public void partialFeedIsNotCountedAsCompletedPollen() {
    HiveShotController c = feeding();
    c.update(0.41, true, true, false, false, false, true);
    assertEquals(0, c.getCompletedPulses()); assertEquals(HiveShotController.State.FAILED, c.getState());
  }
  @Test(expected = IllegalArgumentException.class) public void moreThanFourPulsesRejected() {
    new HiveShotController(0, 0.15, 5, false);
  }
}
