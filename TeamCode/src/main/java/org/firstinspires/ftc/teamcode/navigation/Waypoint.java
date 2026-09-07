package org.firstinspires.ftc.teamcode.navigation;

/** Field-relative target pose in inches and degrees. */
public final class Waypoint {
  private final double x;
  private final double y;
  private final double headingDegrees;

  public Waypoint(double x, double y, double headingDegrees) {
    this.x = x;
    this.y = y;
    this.headingDegrees = headingDegrees;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getHeadingDegrees() {
    return headingDegrees;
  }
}
