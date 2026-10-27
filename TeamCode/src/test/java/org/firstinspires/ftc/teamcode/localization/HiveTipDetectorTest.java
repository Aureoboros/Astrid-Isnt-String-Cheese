package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.*;
import java.util.Collections;
import org.junit.Test;

public final class HiveTipDetectorTest {
  private void observe(HiveTipDetector d, double time, double stamp, int id, double height) {
    d.update(time, stamp, Collections.singletonList(new AprilTagObservation(id, 24, 0, height, 0, 0)));
  }
  private void high(HiveTipDetector d) {
    observe(d, 0, 1, 34, 50); observe(d, 0.15, 2, 34, 50); observe(d, 0.3, 3, 34, 50);
  }
  @Test public void sameTagHighToStableLowConfirmsTransition() {
    HiveTipDetector d = new HiveTipDetector(34); high(d);
    assertTrue(d.isHighConfirmed());
    observe(d, 0.4, 4, 34, 34); assertTrue(d.isLowCandidate()); assertFalse(d.isTipped());
    observe(d, 0.6, 5, 34, 34); observe(d, 0.8, 6, 34, 34);
    assertTrue(d.isTipped());
  }
  @Test public void duplicateFramesCannotConfirmHighOrLow() {
    HiveTipDetector d = new HiveTipDetector(34);
    observe(d, 0, 1, 34, 50); observe(d, 1, 1, 34, 50); observe(d, 2, 1, 34, 50);
    assertFalse(d.isHighConfirmed());
    d = new HiveTipDetector(34); high(d);
    observe(d, 0.4, 4, 34, 34); observe(d, 1, 4, 34, 34); observe(d, 2, 4, 34, 34);
    assertFalse(d.isTipped());
  }
  @Test public void missingTagAndDifferentTagAreNotTips() {
    HiveTipDetector d = new HiveTipDetector(34); high(d);
    d.update(0.4, Double.NaN, Collections.emptyList());
    assertFalse(d.isHighVisible()); assertFalse(d.isTipped());
    observe(d, 0.5, 4, 35, 34); observe(d, 1, 5, 35, 34); observe(d, 2, 6, 35, 34);
    assertFalse(d.isTipped());
  }
  @Test public void wrongAllianceAndLowWithoutHighAreRejected() {
    HiveTipDetector d = new HiveTipDetector(34);
    observe(d, 0, 1, 38, 50); observe(d, 1, 2, 38, 50); observe(d, 2, 3, 38, 50);
    assertFalse(d.isHighConfirmed());
    observe(d, 3, 4, 34, 34); observe(d, 4, 5, 34, 34); observe(d, 5, 6, 34, 34);
    assertFalse(d.isTipped());
  }
  @Test public void invalidHeightAndOcclusionResetLowStability() {
    HiveTipDetector d = new HiveTipDetector(34); high(d);
    observe(d, 0.4, 4, 34, 34); observe(d, 0.8, 5, 34, Double.NaN);
    observe(d, 1, 6, 34, 34); observe(d, 1.2, 7, 34, 34);
    assertFalse(d.isTipped());
  }
}
