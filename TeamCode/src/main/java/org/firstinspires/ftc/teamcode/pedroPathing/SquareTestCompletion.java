package org.firstinspires.ftc.teamcode.pedroPathing;

/** A turn completes on heading; a driving leg must settle in both position and heading. */
public final class SquareTestCompletion {
  private SquareTestCompletion() {}

  public static boolean atTarget(double positionError, double headingErrorRadians,
      boolean turnOnly, double positionTolerance, double headingToleranceRadians) {
    return Double.isFinite(positionError) && Double.isFinite(headingErrorRadians)
        && (turnOnly || positionError <= positionTolerance)
        && Math.abs(headingErrorRadians) <= headingToleranceRadians;
  }
}
