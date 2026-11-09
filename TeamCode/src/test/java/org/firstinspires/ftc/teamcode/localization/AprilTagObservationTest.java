package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public final class AprilTagObservationTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void calculatesHorizontalAndThreeDimensionalDistance() {
    AprilTagObservation observation = new AprilTagObservation(7, 3, 4, 12, 0, 0);

    assertEquals(5.0, observation.getHorizontalDistanceInches(), EPSILON);
    assertEquals(13.0, observation.getRangeInches(), EPSILON);
    assertEquals(12.0, observation.getHeightInches(), EPSILON);
  }

  @Test
  public void mapsAllSixAxisDirectionsFromLimelightRobotSpace() {
    // Raw right/down/forward -> application forward/left/up.
    double[][] cases = {
      {1, 0, 0, 0, -1, 0},
      {-1, 0, 0, 0, 1, 0},
      {0, 1, 0, 0, 0, -1},
      {0, -1, 0, 0, 0, 1},
      {0, 0, 1, 1, 0, 0},
      {0, 0, -1, -1, 0, 0}
    };
    for (double[] axis : cases) {
      AprilTagObservation observation = AprilTagObservation.fromLimelightRobotSpaceInches(
          35, axis[0], axis[1], axis[2], 0, 0);
      assertEquals(axis[3], observation.getForwardInches(), EPSILON);
      assertEquals(axis[4], observation.getLeftInches(), EPSILON);
      assertEquals(axis[5], observation.getHeightInches(), EPSILON);
      assertEquals(1, observation.getRangeInches(), EPSILON);
    }
  }

  @Test
  public void mapsRecordedTag35FrameAndPreservesRangeIdAndAngles() {
    // Actual t6t_rs translation from the Nov 8 test, converted from meters to inches.
    double right = -0.1251013877373353 / 0.0254;
    double down = -0.9201555088215321 / 0.0254;
    double forward = 0.5384736429900224 / 0.0254;
    AprilTagObservation observation = AprilTagObservation.fromLimelightRobotSpaceInches(
        35, right, down, forward, -2.4971424334968333, -2.516741006254719);

    assertEquals(21.19974972401663, observation.getForwardInches(), EPSILON);
    assertEquals(4.925251485721863, observation.getLeftInches(), EPSILON);
    assertEquals(36.22659483549339, observation.getHeightInches(), EPSILON);
    assertEquals(Math.hypot(forward, right), observation.getHorizontalDistanceInches(), EPSILON);
    assertEquals(42.2617281228956, observation.getRangeInches(), EPSILON);
    assertEquals(35, observation.getId());
    assertEquals(-2.4971424334968333, observation.getHorizontalAngleDegrees(), EPSILON);
    assertEquals(-2.516741006254719, observation.getVerticalAngleDegrees(), EPSILON);
  }

  @Test
  public void mapsCenteredTagAheadAndAboveRobotOrigin() {
    AprilTagObservation observation = AprilTagObservation.fromLimelightRobotSpaceInches(
        35, 0, -15, 25, 0, 0);
    assertEquals(25, observation.getForwardInches(), EPSILON);
    assertEquals(0, observation.getLeftInches(), EPSILON);
    assertEquals(15, observation.getHeightInches(), EPSILON);
    assertEquals(25, observation.getHorizontalDistanceInches(), EPSILON);
    assertEquals(Math.hypot(25, 15), observation.getRangeInches(), EPSILON);
  }
}
