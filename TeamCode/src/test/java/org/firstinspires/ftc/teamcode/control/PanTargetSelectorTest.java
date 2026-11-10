package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import java.util.Arrays;
import java.util.Collections;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;
import org.junit.Test;

public final class PanTargetSelectorTest {
  @Test public void initiallyChoosesVisibleTagNearestImageCenter() {
    PanTargetSelector selector = new PanTargetSelector(1);
    assertEquals(35, selector.select(Arrays.asList(new AprilTagAim(34, 10), new AprilTagAim(35, -2)), 0).getId());
  }
  @Test public void preservesLockEvenWhenAnotherTagBecomesMoreCentered() {
    PanTargetSelector selector = new PanTargetSelector(1);
    selector.select(Collections.singletonList(new AprilTagAim(35, 10)), 0);
    assertEquals(35, selector.select(Arrays.asList(new AprilTagAim(34, 0), new AprilTagAim(35, 20)), 0.1).getId());
  }
  @Test public void briefLossHoldsThenReacquiresOnlyVisibleTargets() {
    PanTargetSelector selector = new PanTargetSelector(1);
    selector.select(Collections.singletonList(new AprilTagAim(35, 0)), 0);
    assertNull(selector.select(Collections.singletonList(new AprilTagAim(34, 0)), 0.5));
    assertEquals(34, selector.select(Collections.singletonList(new AprilTagAim(34, 0)), 1.1).getId());
    assertNull(selector.select(Collections.emptyList(), 10));
  }
  @Test public void ignoresNonfiniteOffsetsAndRestrictsHiveCluster() {
    PanTargetSelector selector = new PanTargetSelector(1);
    selector.setIdRange(34, 37);
    assertEquals(35, selector.select(Arrays.asList(new AprilTagAim(30, 0),
        new AprilTagAim(34, Double.NaN), new AprilTagAim(35, 5)), 0).getId());
    selector.setIdRange(30, 33);
    assertEquals(-1, selector.getId());
    assertEquals(30, selector.select(Collections.singletonList(new AprilTagAim(30, 0)), 0.1).getId());
  }
  @Test public void clearReleasesExistingLock() {
    PanTargetSelector selector = new PanTargetSelector(1);
    selector.select(Collections.singletonList(new AprilTagAim(35, 0)), 0);
    selector.clear();
    assertEquals(-1, selector.getId());
  }
}
