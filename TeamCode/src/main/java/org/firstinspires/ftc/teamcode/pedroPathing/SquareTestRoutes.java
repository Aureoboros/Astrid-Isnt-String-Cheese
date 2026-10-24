package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Hardware-independent shapes for the square test, in field inches and degrees from the starting
 * pose (0, 0, 0°). Squares run forward, left, backward, right in their configured orientation.
 * Positive X is forward and positive Y is left at the starting heading.
 */
public final class SquareTestRoutes {
  public static final double SIDE_6_INCHES = 6.0;
  public static final double SIDE_1_FOOT = 12.0;
  public static final double SIDE_4_FEET = 48.0;
  public static final double TRIANGLE_SIDE_INCHES = 24.0;
  public static final double ROTATED_SQUARE_HEADING_DEGREES = 30.0;

  private SquareTestRoutes() {}

  public enum Shape {
    SQUARE_6_INCH("6-inch square", square(SIDE_6_INCHES, 0)),
    SQUARE_1_FOOT("1-foot square", square(SIDE_1_FOOT, 0)),
    SQUARE_4_FOOT("4-foot square", square(SIDE_4_FEET, 0)),
    SQUARE_1_FOOT_ROTATED("1-foot square rotated +30°",
        square(SIDE_1_FOOT, ROTATED_SQUARE_HEADING_DEGREES)),
    TRIANGLE_2_FOOT("2-foot equilateral triangle", triangle(TRIANGLE_SIDE_INCHES));

    private final String label;
    private final List<Corner> corners;

    Shape(String label, List<Corner> corners) {
      this.label = label;
      this.corners = corners;
    }

    public String getLabel() { return label; }

    /** Targets in order, beginning with the start corner and ending back at it. */
    public List<Corner> getCorners() { return corners; }
  }

  /**
   * A target pose. Two consecutive corners at the same position form a turn in place; otherwise
   * the robot drives a straight leg between them.
   */
  public static final class Corner {
    private final double x;
    private final double y;
    private final double headingDegrees;

    public Corner(double x, double y, double headingDegrees) {
      this.x = x;
      this.y = y;
      this.headingDegrees = headingDegrees;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingDegrees() { return headingDegrees; }

    public boolean samePositionAs(Corner other) {
      return Math.hypot(x - other.x, y - other.y) < 1e-6;
    }
  }

  /**
   * Rotate once at the starting corner, then drive all four rotated sides at a constant heading.
   * The returned route includes the explicit initial turn when the rotation is nonzero.
   */
  static List<Corner> square(double side, double rotationDegrees) {
    double[][] positions = {{side, 0}, {side, side}, {0, side}, {0, 0}};
    Corner[] corners = new Corner[1 + positions.length + (rotationDegrees == 0 ? 0 : 1)];
    int index = 0;
    corners[index++] = new Corner(0, 0, 0);
    if (rotationDegrees != 0) {
      corners[index++] = new Corner(0, 0, rotationDegrees);
    }
    double radians = Math.toRadians(rotationDegrees);
    double cos = Math.cos(radians);
    double sin = Math.sin(radians);
    for (double[] position : positions) {
      corners[index++] = new Corner(position[0] * cos - position[1] * sin,
          position[0] * sin + position[1] * cos, rotationDegrees);
    }
    return Collections.unmodifiableList(Arrays.asList(corners));
  }

  /** Equilateral triangle from the right corner: to the forward apex, the left corner, then home. */
  static List<Corner> triangle(double side) {
    double height = side * Math.sqrt(3) / 2;
    return Collections.unmodifiableList(Arrays.asList(
        new Corner(0, 0, 0),
        new Corner(height, side / 2, 0),
        new Corner(0, side, 0),
        new Corner(0, 0, 0)));
  }
}
