package org.firstinspires.ftc.teamcode.localization;

/** A Limelight AprilTag measurement expressed in robot-relative inches. */
public final class AprilTagObservation {
  private final int id;
  private final double forwardInches;
  private final double leftInches;
  private final double heightInches;
  private final double horizontalDistanceInches;
  private final double rangeInches;
  private final double horizontalAngleDegrees;
  private final double verticalAngleDegrees;

  public AprilTagObservation(
      int id,
      double forwardInches,
      double leftInches,
      double heightInches,
      double horizontalAngleDegrees,
      double verticalAngleDegrees) {
    this.id = id;
    this.forwardInches = forwardInches;
    this.leftInches = leftInches;
    this.heightInches = heightInches;
    this.horizontalDistanceInches = Math.hypot(forwardInches, leftInches);
    this.rangeInches = Math.hypot(horizontalDistanceInches, heightInches);
    this.horizontalAngleDegrees = horizontalAngleDegrees;
    this.verticalAngleDegrees = verticalAngleDegrees;
  }

  public int getId() {
    return id;
  }

  public double getForwardInches() {
    return forwardInches;
  }

  public double getLeftInches() {
    return leftInches;
  }

  /** Returns the tag's signed vertical offset from the configured robot origin. */
  public double getHeightInches() {
    return heightInches;
  }

  /** Returns distance to the tag projected onto the robot's horizontal plane. */
  public double getHorizontalDistanceInches() {
    return horizontalDistanceInches;
  }

  /** Returns straight-line three-dimensional distance from the robot origin to the tag. */
  public double getRangeInches() {
    return rangeInches;
  }

  public double getHorizontalAngleDegrees() {
    return horizontalAngleDegrees;
  }

  public double getVerticalAngleDegrees() {
    return verticalAngleDegrees;
  }
}
