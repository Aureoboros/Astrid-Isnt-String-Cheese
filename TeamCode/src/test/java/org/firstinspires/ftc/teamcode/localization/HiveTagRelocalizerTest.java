package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;
import org.junit.Test;

public final class HiveTagRelocalizerTest {
  private static final double EPS = 1e-6;
  private static final double[][] MAP = {{34, -10, -16}, {35, -16, -16}, {36, Double.NaN, Double.NaN}};

  /** What the Limelight would report for a tag at (tagX, tagY) seen from a TRUE robot pose. */
  private static AprilTagObservation seen(int id, double tagX, double tagY,
      double robotX, double robotY, double headingRad) {
    double dx = tagX - robotX, dy = tagY - robotY;
    double forward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
    double left = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
    return new AprilTagObservation(id, forward, left, 30, 0, 0);
  }

  private static HiveTagRelocalizer relocalizer() {
    return new HiveTagRelocalizer(MAP, 3, 2.0, 8.0);
  }

  @Test public void robotAndTagTransformsAreInverses() {
    double h = Math.toRadians(37);
    double[] tag = HiveTagRelocalizer.tagFromRobot(-12, -40, 20, 4, h);
    double[] robot = HiveTagRelocalizer.robotFromTag(tag[0], tag[1], 20, 4, h);
    assertEquals(-12, robot[0], EPS);
    assertEquals(-40, robot[1], EPS);
  }

  @Test public void facingPlusYTagIsAheadAndLeftMeansMinusX() {
    // Heading 90: forward is +Y, left is -X (the simple hand-measuring rule in the docs).
    double[] tag = HiveTagRelocalizer.tagFromRobot(-12.75, -40, 24, 3, Math.toRadians(90));
    assertEquals(-15.75, tag[0], EPS);
    assertEquals(-16, tag[1], EPS);
  }

  @Test public void placementErrorIsRecoveredFromTags() {
    double h = Math.toRadians(90);
    double trueX = -11.0, trueY = -42.5;          // where the robot really is
    double odoX = -12.75, odoY = -40.0;           // where odometry thinks it is
    HiveTagRelocalizer fix = relocalizer();
    for (int frame = 0; frame < 3; frame++) {
      List<AprilTagObservation> tags = Arrays.asList(
          seen(34, -10, -16, trueX, trueY, h), seen(35, -16, -16, trueX, trueY, h));
      assertTrue(fix.addFrame(frame, odoX, odoY, h, tags));
    }
    assertEquals(HiveTagRelocalizer.Status.OK, fix.getStatus());
    double[] corrected = fix.corrected(odoX, odoY);
    assertEquals(trueX, corrected[0], EPS);
    assertEquals(trueY, corrected[1], EPS);
    assertEquals(Arrays.asList(34, 35), fix.getUsedTagIds());
  }

  @Test public void offsetIsAppliedToCurrentPoseNotSamplingPose() {
    double h = Math.toRadians(90);
    HiveTagRelocalizer fix = relocalizer();
    for (int frame = 0; frame < 3; frame++) {
      // Robot creeps +0.2 in each frame; odometry tracks the creep but is offset by (+1, -2).
      double trueX = -12 + 0.2 * frame, trueY = -41;
      fix.addFrame(frame, trueX + 1, trueY - 2, h,
          Collections.singletonList(seen(34, -10, -16, trueX, trueY, h)));
    }
    double[] corrected = fix.corrected(-10.6 + 1, -43);
    assertEquals(-10.6, corrected[0], EPS);
    assertEquals(-41, corrected[1], EPS);
  }

  @Test public void repeatedFramesUnmappedTagsAndBadDataAreIgnored() {
    double h = Math.toRadians(90);
    HiveTagRelocalizer fix = relocalizer();
    List<AprilTagObservation> good = Collections.singletonList(seen(34, -10, -16, -12, -40, h));
    assertTrue(fix.addFrame(1, -12, -40, h, good));
    assertFalse(fix.addFrame(1, -12, -40, h, good));                 // same frame polled again
    assertFalse(fix.addFrame(Double.NaN, -12, -40, h, good));        // no fresh frame
    assertFalse(fix.addFrame(2, -12, -40, h,                          // tag 36 not measured
        Collections.singletonList(seen(36, -10, -16, -12, -40, h))));
    assertFalse(fix.addFrame(3, -12, -40, h,                          // tag 99 not in map
        Collections.singletonList(seen(99, -10, -16, -12, -40, h))));
    assertFalse(fix.addFrame(4, -12, -40, h,                          // behind the camera
        Collections.singletonList(new AprilTagObservation(34, -20, 0, 30, 0, 0))));
    assertFalse(fix.addFrame(5, -12, -40, h,                          // too close to trust
        Collections.singletonList(new AprilTagObservation(34, 6, 0, 30, 0, 0))));
    assertEquals(1, fix.getFrameCount());
    assertEquals(HiveTagRelocalizer.Status.WAITING, fix.getStatus());
  }

  @Test public void disagreeingFramesAreRejected() {
    double h = Math.toRadians(90);
    HiveTagRelocalizer fix = relocalizer();
    double[] trueYs = {-40, -40, -45};
    for (int frame = 0; frame < 3; frame++) {
      fix.addFrame(frame, -12, -40, h,
          Collections.singletonList(seen(34, -10, -16, -12, trueYs[frame], h)));
    }
    assertEquals(HiveTagRelocalizer.Status.TOO_NOISY, fix.getStatus());
  }

  @Test public void implausiblyLargeCorrectionIsRejected() {
    double h = Math.toRadians(90);
    HiveTagRelocalizer fix = relocalizer();
    for (int frame = 0; frame < 3; frame++) {
      fix.addFrame(frame, -12, -40, h,
          Collections.singletonList(seen(34, -10, -16, -12, -28, h)));   // 12 in "error"
    }
    assertEquals(HiveTagRelocalizer.Status.CORRECTION_TOO_LARGE, fix.getStatus());
  }

  @Test public void unmeasuredDefaultConfigSkipsTheFix() {
    assertFalse(new HiveTagRelocalizer(HiveAutoConfig.RED_HIVE_TAGS, 5, 2, 8).hasTagPositions());
    assertFalse(new HiveTagRelocalizer(HiveAutoConfig.BLUE_HIVE_TAGS, 5, 2, 8).hasTagPositions());
    assertEquals(HiveTagRelocalizer.Status.NO_TAG_POSITIONS,
        new HiveTagRelocalizer(HiveAutoConfig.RED_HIVE_TAGS, 5, 2, 8).getStatus());
  }
}
