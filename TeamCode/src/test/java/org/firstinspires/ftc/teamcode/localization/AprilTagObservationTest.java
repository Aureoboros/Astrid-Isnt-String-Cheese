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
}
