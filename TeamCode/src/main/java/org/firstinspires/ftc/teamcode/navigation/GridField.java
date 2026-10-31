package org.firstinspires.ftc.teamcode.navigation;

/**
 * A 6 x 6 tile coordinate field laid over Pedro's 0-144 inch field.
 *
 * <p>Grid points run from (0, 0) to (6, 6); one grid unit is one 24-inch field tile. AprilTag 40
 * marks grid point (2, 1). All headings are field-relative radians, counter-clockwise from +X.
 */
public final class GridField {
  public static final int MIN = 0;
  public static final int MAX = 6;
  public static final double TILE_INCHES = 24.0;

  public static final int ANCHOR_TAG_ID = 40;
  public static final int ANCHOR_GRID_X = 2;
  public static final int ANCHOR_GRID_Y = 1;

  public static final double CENTER_GRID_X = 3;
  public static final double CENTER_GRID_Y = 3;

  private GridField() {}

  public static double toInches(double gridUnits) {
    return gridUnits * TILE_INCHES;
  }

  public static double toGrid(double inches) {
    return inches / TILE_INCHES;
  }

  public static int clamp(int gridValue) {
    return Math.max(MIN, Math.min(MAX, gridValue));
  }

  /** Nearest on-field grid point to an inch coordinate. */
  public static int nearestGrid(double inches) {
    return clamp((int) Math.round(toGrid(inches)));
  }

  /**
   * Robot field position in inches from a robot-relative tag measurement.
   *
   * @param forwardInches tag distance ahead of the robot
   * @param leftInches tag distance to the robot's left
   * @param robotHeading field heading of the robot when the tag was seen
   * @return {x, y} in inches
   */
  public static double[] robotPositionFromTag(
      double forwardInches, double leftInches, double robotHeading) {
    double cos = Math.cos(robotHeading);
    double sin = Math.sin(robotHeading);
    double tagX = toInches(ANCHOR_GRID_X);
    double tagY = toInches(ANCHOR_GRID_Y);
    return new double[] {
      tagX - (forwardInches * cos - leftInches * sin),
      tagY - (forwardInches * sin + leftInches * cos)
    };
  }

  /**
   * Heading that points the robot at the field center (3, 3), or {@code fallback} when the robot is
   * already on the center and no direction is defined.
   */
  public static double headingToCenter(double xInches, double yInches, double fallback) {
    double dx = toInches(CENTER_GRID_X) - xInches;
    double dy = toInches(CENTER_GRID_Y) - yInches;
    if (Math.hypot(dx, dy) < 1e-6) {
      return fallback;
    }
    return Math.atan2(dy, dx);
  }

  /** Smallest signed angle from {@code from} to {@code to}, in (-pi, pi]. */
  public static double angleError(double to, double from) {
    double diff = to - from;
    return Math.atan2(Math.sin(diff), Math.cos(diff));
  }
}
