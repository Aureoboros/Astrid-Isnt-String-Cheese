package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;
import org.junit.Test;

public final class ServoTrackingTestControlsTest {
  private final ImagePanTracker aim = new ImagePanTracker(
      0.05, 0.2, 0.35, 0.2, 0.001, 0.01, 2, 0.35, 0.5, 1);
  private final ServoTrackingTestControls controls = new ServoTrackingTestControls(aim, 0.005, 0.1);

  private void start() { aim.setDirection(-1); controls.start(0, 1); }
  private List<AprilTagAim> tag(double tx) { return Collections.singletonList(new AprilTagAim(35, tx)); }
  private void manual(boolean left, boolean right, double time) {
    controls.update(false, false, left, right, tag(30), time + 1, time);
  }
  private void auto(double tx, double time) {
    controls.update(false, false, false, false, tag(tx), time + 1, time);
  }

  @Test public void startsAutoOnlyAfterPlayAndNeedsVerifiedDirection() {
    assertTrue(Double.isNaN(aim.getPosition()));
    controls.start(0, 1);
    assertFalse(aim.isTracking());
    aim.stop();
    start();
    assertTrue(aim.isTracking());
    assertEquals(0.2, aim.getPosition(), 0);
  }

  @Test public void heldManualPanRepeatsAtControlledRateAndOverridesAuto() {
    start();
    manual(true, false, 0.1);
    assertEquals(0.205, aim.getPosition(), 1e-9);
    manual(true, false, 0.15);
    assertEquals(0.205, aim.getPosition(), 1e-9);
    manual(true, false, 0.25);
    assertEquals(0.21, aim.getPosition(), 1e-9);
    assertTrue(controls.isManual());
    assertTrue(aim.isTracking()); // Request retained, but no automatic update on held input.
    manual(false, true, 0.26);
    assertEquals(0.205, aim.getPosition(), 1e-9);
  }

  @Test public void releaseReacquiresFreshTagAndAlignsAfterManualDisplacement() {
    start();
    manual(true, false, 0.1);
    auto(5, 0.2); // Still settling after manual motion.
    assertEquals(0.205, aim.getPosition(), 1e-9);
    auto(5, 0.7); // First post-settle frame is discarded.
    assertEquals(0.205, aim.getPosition(), 1e-9);
    auto(5, 0.8);
    assertEquals(0.2, aim.getPosition(), 1e-9);
    auto(0, 1.4); auto(0, 1.5);
    assertTrue(aim.getStatus().contains("centered"));
    assertFalse(controls.isManual());
    assertEquals(35, aim.getTargetId());
  }

  @Test public void followsChangingTagBearingWithoutAnyManualInput() {
    start();
    auto(10, 0.6); auto(10, 0.7);
    assertEquals(0.19, aim.getPosition(), 1e-9);
    auto(-10, 1.3); auto(-10, 1.4);
    assertEquals(0.2, aim.getPosition(), 1e-9);
  }

  @Test public void pauseSurvivesManualReleaseAndXIsEdgeTriggered() {
    start();
    controls.update(true, false, false, false, tag(20), 2, 1);
    controls.update(true, false, false, false, tag(20), 3, 2);
    assertFalse(aim.isTracking());
    manual(true, false, 2.1);
    auto(20, 3);
    assertFalse(aim.isTracking());
    assertEquals(0.205, aim.getPosition(), 1e-9);
    controls.update(true, false, false, false, tag(20), 5, 4);
    assertTrue(aim.isTracking());
  }

  @Test public void centerHasPriorityAndHoldsUntilResumed() {
    start();
    manual(true, false, 0.1);
    controls.update(true, true, true, false, tag(20), 2, 0.2);
    assertFalse(aim.isTracking());
    assertEquals(0.2, aim.getPosition(), 0);
    controls.update(false, true, true, false, tag(20), 3, 1);
    assertEquals(0.2, aim.getPosition(), 0);
    auto(20, 2);
    assertEquals(0.2, aim.getPosition(), 0);
  }

  @Test public void noVisibleTagHoldsAndOpposingManualInputsCancelWithinBounds() {
    start();
    manual(true, true, 0.6);
    manual(true, true, 0.7);
    assertEquals(0.2, aim.getPosition(), 0);
    for (int i = 0; i < 50; i++) { manual(true, false, 1 + i * 0.2); }
    assertEquals(0.35, aim.getPosition(), 0);
    controls.update(false, false, false, false, Collections.emptyList(), 20, 15);
    assertEquals(0.35, aim.getPosition(), 0);
    assertTrue(aim.getStatus().contains("no locked tag"));
    for (int i = 0; i < 70; i++) { manual(false, true, 16 + i * 0.2); }
    assertEquals(0.05, aim.getPosition(), 0);
  }
}
