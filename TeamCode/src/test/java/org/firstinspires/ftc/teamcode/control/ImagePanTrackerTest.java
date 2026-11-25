package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import java.util.Collections;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;
import org.junit.Test;

public final class ImagePanTrackerTest {
  private static ImagePanTracker continuousTracker() {
    ImagePanTracker t = new ImagePanTracker(CameraPanConfig.MIN_POSITION,
        CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION,
        CameraPanConfig.START_POSITION, CameraPanConfig.POSITION_GAIN_PER_TX_DEGREE,
        CameraPanConfig.MAX_POSITION_STEP, CameraPanConfig.TX_DEADBAND_DEGREES,
        CameraPanConfig.IMAGE_SETTLE_SECONDS, CameraPanConfig.IMAGE_UPDATE_INTERVAL_SECONDS,
        CameraPanConfig.LOST_TARGET_HOLD_SECONDS);
    t.setDirection(CameraPanConfig.TX_TO_POSITION_SIGN);
    t.start(0, 1); t.toggleTracking();
    return t;
  }

  @Test public void continuousTrackingFollowsChangingBearingWithoutSettlingBarrier() {
    ImagePanTracker t = continuousTracker();
    t.update(tag(35, 20), 2, 0.051);
    assertEquals(0.19, t.getPosition(), 1e-9); // First eligible fresh frame corrects.
    t.update(tag(35, 20), 3, 0.102);
    assertEquals(0.18, t.getPosition(), 1e-9);
    t.update(tag(35, -20), 4, 0.153); // Bearing changes as the robot turns/passes the tag.
    assertEquals(0.19, t.getPosition(), 1e-9);
    t.update(tag(35, 0), 5, 0.204);
    assertEquals(0.19, t.getPosition(), 1e-9);
    assertTrue(t.getStatus().contains("centered"));
  }

  @Test public void continuousTrackingStillRejectsDuplicatesAndLimitsCorrectionRate() {
    ImagePanTracker t = continuousTracker();
    t.update(tag(35, 20), 2, 0.051);
    t.update(tag(35, 20), 3, 0.06); // Fresh but too soon.
    t.update(tag(35, 20), 2, 0.11); // Old frame, even after interval elapsed.
    t.update(tag(35, 20), Double.NaN, 0.12);
    assertEquals(0.19, t.getPosition(), 1e-9);
    t.update(tag(35, 20), 4, 0.13);
    assertEquals(0.18, t.getPosition(), 1e-9);
    t.update(Collections.emptyList(), Double.NaN, 0.2);
    assertEquals(0.18, t.getPosition(), 1e-9);
  }

  @Test public void continuousTrackingResumesQuicklyAfterManualPanAndHonorsLimits() {
    ImagePanTracker t = continuousTracker();
    ServoTrackingTestControls controls = new ServoTrackingTestControls(t, 0.005, 0.1);
    controls.update(false, false, true, false, tag(35, 20), 2, 0.1);
    assertEquals(0.205, t.getPosition(), 1e-9);
    controls.update(false, false, false, false, tag(35, 20), 3, 0.151);
    assertEquals(0.195, t.getPosition(), 1e-9);
    for (int i = 0; i < 50; i++) {
      t.update(tag(35, 30), 4 + i, 0.21 + i * 0.06);
      assertTrue(t.getPosition() >= CameraPanConfig.MIN_POSITION);
    }
    assertEquals(CameraPanConfig.MIN_POSITION, t.getPosition(), 1e-9);
    assertTrue(t.getStatus().contains("travel limit"));
  }

  @Test public void selectedTagTestIgnoresOtherIdsAndCanSwitchTargets() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(-1); t.setTargetIdRange(35, 35); t.toggleTracking();
    t.update(tag(20, 20), 2, 0.6);
    assertEquals(0.2, t.getPosition(), 0);
    assertEquals(-1, t.getTargetId());
    t.update(tag(35, 0), 3, 0.7);
    assertEquals(35, t.getTargetId());
    t.setTargetIdRange(20, 20);
    t.update(tag(35, 20), 4, 0.8);
    assertEquals(-1, t.getTargetId());
    t.update(tag(20, 20), 5, 0.9);
    assertEquals(20, t.getTargetId());
    assertEquals(0.19, t.getPosition(), 1e-9);
  }
  private static ImagePanTracker tracker() {
    return new ImagePanTracker(0.05, 0.2, 0.35, 0.2, 0.001, 0.01, 2, 0.35, 0.5, 1);
  }
  private static java.util.List<AprilTagAim> tag(int id, double tx) {
    return Collections.singletonList(new AprilTagAim(id, tx));
  }

  @Test public void requiresPlayAndExplicitDirectionBeforeTracking() {
    ImagePanTracker t = tracker();
    assertTrue(Double.isNaN(t.getPosition()));
    t.toggleTracking();
    assertFalse(t.isTracking());
    t.start(0, 1);
    assertEquals(0.2, t.getPosition(), 0);
    t.toggleTracking();
    t.update(tag(35, 20), 2, 1);
    assertFalse(t.isTracking());
    assertEquals(0.2, t.getPosition(), 0);
    t.setDirection(1);
    t.toggleTracking();
    assertTrue(t.isTracking());
  }

  @Test public void feedbackWorksInBothDirectionsAndUsesBoundedSteps() {
    for (int sign : new int[] {1, -1}) {
      ImagePanTracker t = tracker();
      t.start(0, 1); t.setDirection(sign); t.toggleTracking();
      t.update(tag(35, 30), 2, 0.6);
      assertEquals(0.2, t.getPosition(), 1e-9);
      t.update(tag(35, 30), 3, 0.7);
      assertEquals(0.2 + sign * 0.01, t.getPosition(), 1e-9);
      t.update(tag(35, -30), 4, 1.3);
      t.update(tag(35, -30), 5, 1.4);
      assertEquals(0.2, t.getPosition(), 1e-9);
    }
  }

  @Test public void settlingBarrierAndDuplicateFramesCannotCauseRepeatedCorrections() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(1); t.toggleTracking();
    t.update(tag(35, 10), 2, 0.1);
    t.update(tag(35, 10), 3, 0.2);
    assertEquals(0.2, t.getPosition(), 0);
    t.update(tag(35, 10), 4, 0.6); // Barrier frame.
    t.update(tag(35, 10), 4, 0.7); // Same frame: no correction.
    assertEquals(0.2, t.getPosition(), 0);
    t.update(tag(35, 10), 5, 0.8);
    assertEquals(0.21, t.getPosition(), 1e-9);
    t.update(tag(35, 10), 5, 2);
    assertEquals(0.21, t.getPosition(), 1e-9);
    t.update(tag(35, 10), 6, 2.1);
    assertEquals(0.21, t.getPosition(), 1e-9);
  }

  @Test public void centeredTagDeadbandPreventsJitter() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(1); t.toggleTracking();
    t.update(tag(35, 1.5), 2, 0.6);
    t.update(tag(35, -1.5), 3, 0.7);
    assertEquals(0.2, t.getPosition(), 0);
    assertTrue(t.getStatus().contains("centered"));
  }

  @Test public void stopsAtSafeLimitsRatherThanSweepingPastThem() {
    for (int sign : new int[] {1, -1}) {
      ImagePanTracker t = tracker();
      t.start(0, 1); t.setDirection(sign); t.toggleTracking();
      for (int i = 0; i < 25; i++) {
        t.update(tag(35, 30), 2 + i * 2, 0.6 + i);
        t.update(tag(35, 30), 3 + i * 2, 0.7 + i);
        assertTrue(t.getPosition() >= 0.05 && t.getPosition() <= 0.35);
      }
      assertEquals(sign == 1 ? 0.35 : 0.05, t.getPosition(), 1e-9);
      assertTrue(t.getStatus().contains("travel limit"));
    }
  }

  @Test public void lossHoldsThenReacquiresAnActuallyVisibleTag() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(1); t.toggleTracking();
    t.update(tag(35, 0), 2, 0.6);
    t.update(tag(35, 0), 3, 0.7);
    t.update(tag(34, 20), 4, 0.8);
    assertEquals(35, t.getTargetId());
    assertEquals(0.2, t.getPosition(), 0);
    t.update(Collections.emptyList(), Double.NaN, 1.5);
    assertEquals(0.2, t.getPosition(), 0);
    t.update(tag(34, 20), 5, 1.8);
    assertEquals(34, t.getTargetId());
    assertEquals(0.21, t.getPosition(), 1e-9);
  }

  @Test public void manualStepsRequireTrackingOffAndCenterDisarmsIt() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(1); t.toggleTracking();
    t.nudge(10, 1, 2);
    assertEquals(0.2, t.getPosition(), 0);
    t.center(1, 2);
    assertFalse(t.isTracking());
    t.nudge(10, 2, 3);
    assertEquals(0.35, t.getPosition(), 0);
    t.nudge(-10, 3, 4);
    assertEquals(0.05, t.getPosition(), 0);
    t.center(4, 5);
    assertEquals(0.2, t.getPosition(), 0);
    t.stop();
    assertFalse(t.isTracking());
    assertTrue(Double.isNaN(t.getPosition()));
  }

  @Test public void changingOrClearingDirectionDisarmsTracking() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(1); t.toggleTracking();
    t.setDirection(-1);
    assertFalse(t.isTracking());
    t.toggleTracking();
    t.clearDirection(); t.toggleTracking();
    assertFalse(t.isTracking());
    assertEquals(0, t.getDirection());
  }

  @Test public void invalidOrStaleFrameCannotMoveServo() {
    ImagePanTracker t = tracker();
    t.start(0, 1); t.setDirection(1); t.toggleTracking();
    t.update(tag(35, 20), Double.NaN, 1);
    t.update(tag(35, 20), 1, 1.1);
    t.update(tag(35, 20), 2, Double.NaN);
    t.update(tag(35, Double.NaN), 2, 1.2);
    assertEquals(0.2, t.getPosition(), 0);
  }

  @Test public void configuredLeftTurningServoDecreasesPositionForTagOnRight() {
    assertTrue(CameraPanConfig.IMAGE_ONLY_ENABLED);
    assertFalse(CameraPanConfig.ENABLED);
    assertFalse(CameraPanConfig.GEOMETRY_VERIFIED);
    assertEquals(-1, CameraPanConfig.TX_TO_POSITION_SIGN);
    ImagePanTracker t = tracker();
    t.setDirection(CameraPanConfig.TX_TO_POSITION_SIGN);
    t.start(0, 1); t.toggleTracking();
    t.update(tag(35, 10), 2, 0.6);
    t.update(tag(35, 10), 3, 0.7);
    assertEquals(0.19, t.getPosition(), 1e-9);
  }

  @Test public void targetRangeChangesReleaseOldLockAndIgnoreOtherTags() {
    ImagePanTracker t = tracker();
    t.setDirection(-1); t.start(0, 1); t.toggleTracking();
    t.update(tag(35, 0), 2, 0.6);
    assertEquals(35, t.getTargetId());
    t.setTargetIdRange(20, 24);
    t.update(tag(35, 30), 3, 0.7);
    assertEquals(-1, t.getTargetId());
    assertEquals(0.2, t.getPosition(), 0);
    t.update(tag(24, 10), 4, 0.8);
    assertEquals(24, t.getTargetId());
    assertEquals(0.19, t.getPosition(), 1e-9);
  }
}
