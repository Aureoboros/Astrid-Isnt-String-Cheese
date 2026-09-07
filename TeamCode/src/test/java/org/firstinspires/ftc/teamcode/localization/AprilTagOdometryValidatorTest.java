package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.localization.AprilTagOdometryValidator.ValidationResult;
import org.junit.Test;

public final class AprilTagOdometryValidatorTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void forwardOdometryMotionMatchesReducedLimelightDistance() {
    AprilTagOdometryValidator validator = new AprilTagOdometryValidator(0.5, 0.5);
    validator.captureBaseline(
        new PoseEstimate(0, 0, 0), new AprilTagObservation(5, 24, 0, 8, 0, 0));

    ValidationResult result =
        validator.validate(
            new PoseEstimate(6, 0, 0), new AprilTagObservation(5, 18, 0, 8, 0, 0));

    assertTrue(result.isAvailable());
    assertTrue(result.isPassing());
    assertEquals(18.0, result.getExpectedDistanceInches(), EPSILON);
    assertEquals(0.0, result.getPositionErrorInches(), EPSILON);
  }

  @Test
  public void rotationTransformsFixedTagIntoRobotCoordinates() {
    AprilTagOdometryValidator validator = new AprilTagOdometryValidator(0.5, 0.5);
    validator.captureBaseline(
        new PoseEstimate(0, 0, 0), new AprilTagObservation(9, 24, 0, 6, 0, 0));

    ValidationResult result =
        validator.validate(
            new PoseEstimate(0, 0, 90), new AprilTagObservation(9, 0, -24, 6, 0, 0));

    assertTrue(result.isPassing());
    assertEquals(0.0, result.getExpectedForwardInches(), EPSILON);
    assertEquals(-24.0, result.getExpectedLeftInches(), EPSILON);
  }

  @Test
  public void inconsistentLimelightDistanceFails() {
    AprilTagOdometryValidator validator = new AprilTagOdometryValidator(1.0, 1.0);
    validator.captureBaseline(
        new PoseEstimate(0, 0, 0), new AprilTagObservation(3, 24, 0, 8, 0, 0));

    ValidationResult result =
        validator.validate(
            new PoseEstimate(6, 0, 0), new AprilTagObservation(3, 14, 0, 8, 0, 0));

    assertTrue(result.isAvailable());
    assertFalse(result.isPassing());
    assertEquals(-4.0, result.getDistanceErrorInches(), EPSILON);
  }

  @Test
  public void missingTrackedTagReturnsUnavailable() {
    AprilTagOdometryValidator validator = new AprilTagOdometryValidator(1.0, 1.0);
    validator.captureBaseline(
        new PoseEstimate(0, 0, 0), new AprilTagObservation(3, 24, 0, 8, 0, 0));

    ValidationResult result = validator.validate(new PoseEstimate(0, 0, 0), null);

    assertFalse(result.isAvailable());
    assertFalse(result.isPassing());
  }
}
