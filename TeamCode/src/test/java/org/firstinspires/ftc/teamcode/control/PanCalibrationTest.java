package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import org.firstinspires.ftc.teamcode.localization.CameraMountPose;
import org.junit.Test;

public final class PanCalibrationTest {
  private static final double EPS = 1e-9;
  static PanCalibration calibration(double lensForward, double lensRight, int sign) {
    return new PanCalibration(0.2, 0.5, 0.8, -90, 0, 90, sign,
        0.0254, -0.09525, 0.4318, lensForward, lensRight, 0, 0, 45);
  }
  @Test public void mapsMeasuredEndpointsCenterAndIntermediatePositions() {
    PanCalibration c = calibration(0, 0, 1);
    assertEquals(-90, c.yawAt(0.2), EPS);
    assertEquals(0, c.yawAt(0.5), EPS);
    assertEquals(90, c.yawAt(0.8), EPS);
    assertEquals(-45, c.yawAt(0.35), EPS);
    assertEquals(45, c.yawAt(0.65), EPS);
    assertEquals(-90, c.yawAt(-1), EPS);
    assertEquals(90, c.yawAt(2), EPS);
  }
  @Test public void acceptsDecreasingMeasuredYawMapping() {
    PanCalibration c = new PanCalibration(0.2, 0.5, 0.8, 60, 0, -60, 1,
        0, 0, 0.4318, 0, 0, 0, 0, 45);
    assertEquals(-30, c.yawAt(0.65), EPS);
  }
  @Test public void lensOnPivotKeepsOffsetsFixedWhileYawChanges() {
    CameraMountPose mount = calibration(0, 0, 1).mountAt(0.8);
    assertEquals(0.0254, mount.getForwardMeters(), EPS);
    assertEquals(-0.09525, mount.getRightMeters(), EPS);
    assertEquals(0.4318, mount.getUpMeters(), EPS);
    assertEquals(90, mount.getYawDegrees(), EPS);
  }
  @Test public void rotatesOffPivotLensTranslationInPhysicalCcwDirection() {
    CameraMountPose mount = calibration(0.1, 0, 1).mountAt(0.8);
    assertEquals(0.0254, mount.getForwardMeters(), EPS);
    assertEquals(-0.19525, mount.getRightMeters(), EPS);
    CameraMountPose oppositeUiConvention = calibration(0.1, 0, -1).mountAt(0.8);
    assertEquals(0.00475, oppositeUiConvention.getRightMeters(), EPS);
  }
  @Test public void feedbackHandlesBothSignsAndBoundsEachStepAndTravel() {
    PanCalibration c = calibration(0, 0, 1);
    assertEquals(0.51, c.stepTowardTag(0.5, 100, 1, 0.001, 0.01, 2), EPS);
    assertEquals(0.49, c.stepTowardTag(0.5, 100, -1, 0.001, 0.01, 2), EPS);
    assertEquals(0.8, c.stepTowardTag(0.799, 100, 1, 0.001, 0.01, 2), EPS);
    assertEquals(0.2, c.stepTowardTag(0.201, -100, 1, 0.001, 0.01, 2), EPS);
  }
  @Test public void deadbandAndInvalidMeasurementsHoldPosition() {
    PanCalibration c = calibration(0, 0, 1);
    assertEquals(0.5, c.stepTowardTag(0.5, 2, 1, 0.001, 0.01, 2), EPS);
    assertEquals(0.5, c.stepTowardTag(0.5, Double.NaN, 1, 0.001, 0.01, 2), EPS);
  }
  @Test(expected = IllegalArgumentException.class) public void rejectsIncompleteCalibration() {
    new PanCalibration(0.2, 0.5, 0.8, Double.NaN, 0, 90, 1, 0, 0, 0, 0, 0, 0, 0, 45);
  }
  @Test(expected = IllegalArgumentException.class) public void rejectsUnverifiedFeedbackSign() {
    calibration(0, 0, 1).stepTowardTag(0.5, 10, 0, 0.001, 0.01, 2);
  }
}
