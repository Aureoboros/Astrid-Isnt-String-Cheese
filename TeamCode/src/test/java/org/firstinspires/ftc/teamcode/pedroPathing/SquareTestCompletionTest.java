package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.*;
import org.junit.Test;

public final class SquareTestCompletionTest {
  private static boolean atTarget(double positionError, double headingDegrees, boolean turnOnly) {
    return SquareTestCompletion.atTarget(positionError, Math.toRadians(headingDegrees),
        turnOnly, 1.5, Math.toRadians(5));
  }

  @Test
  public void turnCanCompleteDespiteReportedTranslationDrift() {
    assertTrue(atTarget(8, 2, true));
    assertTrue(atTarget(8, -2, true));
  }

  @Test
  public void turnStillRequiresHeadingWithinTolerance() {
    assertFalse(atTarget(0, 6, true));
    assertFalse(atTarget(0, -6, true));
  }

  @Test
  public void drivingLegRequiresBothPositionAndHeading() {
    assertTrue(atTarget(1, 3, false));
    assertFalse(atTarget(2, 3, false));
    assertFalse(atTarget(1, 6, false));
  }

  @Test
  public void boundaryValuesAreAccepted() {
    assertTrue(atTarget(1.5, 5, false));
    assertTrue(atTarget(1.5, -5, false));
  }

  @Test
  public void invalidSensorErrorsNeverCompleteEvenDuringTurn() {
    assertFalse(atTarget(Double.NaN, 0, true));
    assertFalse(atTarget(Double.POSITIVE_INFINITY, 0, true));
    assertFalse(atTarget(0, Double.NaN, true));
  }
}
