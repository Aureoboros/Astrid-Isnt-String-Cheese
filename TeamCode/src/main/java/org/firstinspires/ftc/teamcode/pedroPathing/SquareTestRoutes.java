package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Hardware-independent shapes for the square test, in field inches and degrees from the starting
 * pose (0, 0, 0°). Every shape starts at its right corner and runs counterclockwise: forward first,
 * then left. Positive X is forward and positive Y is left at the starting heading.
 */
public final class SquareTestRoutes {
  public static final double SIDE_6_INCHES = 6.0;
  public static final double SIDE_1_FOOT = 12.0;
  public static final double SIDE_4_FEET = 48.0;
  public static final double TRIANGLE_SIDE_INCHES = 24.0;
  public static final double CORNER_TURN_DEGREES = 30.0;

  private SquareTestRoutes() {}

  public enum Shape {
    SQUARE_6_INCH("6-inch square", square(SIDE_6_INCHES, 0)),
    SQUARE_1_FOOT("1-foot square", square(SIDE_1_FOOT, 0)),
    SQUARE_4_FOOT("4-foot square", square(SIDE_4_FEET, 0)),
    SQUARE_4_FOOT_TURNING("4-foot square, +30° per corner", square(SIDE_4_FEET, CORNER_TURN_DEGREES)),
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
   * Square from the right corner: forward, left, backward, right. With a nonzero corner turn the
   * robot stops at each corner, turns in place by that amount, then drives the next straight side.
   */
  static List<Corner> square(double side, double cornerTurnDegrees) {
    double[][] positions = {{side, 0}, {side, side}, {0, side}, {0, 0}};
    Corner[] corners = new Corner[1 + positions.length * (cornerTurnDegrees == 0 ? 1 : 2)];
    int index = 0;
    double heading = 0;
    corners[index++] = new Corner(0, 0, heading);
    for (double[] position : positions) {
      corners[index++] = new Corner(position[0], position[1], heading);
      if (cornerTurnDegrees != 0) {
        heading += cornerTurnDegrees;
        corners[index++] = new Corner(position[0], position[1], heading);
      }
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
