package org.firstinspires.ftc.teamcode.navigation;

/** A robot-relative drive command plus its associated pose errors. */
public final class PoseControlOutput {
  private final double drive;
  private final double strafe;
  private final double turn;
  private final double distanceError;
  private final double headingErrorDegrees;

  PoseControlOutput(
      double drive,
      double strafe,
      double turn,
      double distanceError,
      double headingErrorDegrees) {
    this.drive = drive;
    this.strafe = strafe;
    this.turn = turn;
    this.distanceError = distanceError;
    this.headingErrorDegrees = headingErrorDegrees;
  }

  public double getDrive() {
    return drive;
  }

  public double getStrafe() {
    return strafe;
  }

  public double getTurn() {
    return turn;
  }

  public double getDistanceError() {
    return distanceError;
  }

  public double getHeadingErrorDegrees() {
    return headingErrorDegrees;
  }
}
