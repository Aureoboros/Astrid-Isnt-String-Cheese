package org.firstinspires.ftc.teamcode.drive;

/** Normalized power values for a four-motor mecanum drivetrain. */
public final class DrivePower {
  private final double frontLeft;
  private final double frontRight;
  private final double backLeft;
  private final double backRight;

  public DrivePower(double frontLeft, double frontRight, double backLeft, double backRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }

  public double getFrontLeft() {
    return frontLeft;
  }

  public double getFrontRight() {
    return frontRight;
  }

  public double getBackLeft() {
    return backLeft;
  }

  public double getBackRight() {
    return backRight;
  }
}
