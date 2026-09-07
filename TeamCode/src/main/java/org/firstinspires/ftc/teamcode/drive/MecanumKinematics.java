package org.firstinspires.ftc.teamcode.drive;

/** Pure mecanum-drive math, kept independent from FTC hardware for reuse and testing. */
public final class MecanumKinematics {
  private MecanumKinematics() {}

  public static DrivePower robotCentric(double drive, double strafe, double rotate) {
    double frontLeft = drive + strafe + rotate;
    double frontRight = drive - strafe - rotate;
    double backLeft = drive - strafe + rotate;
    double backRight = drive + strafe - rotate;

    double scale =
        Math.max(
            1.0,
            Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight))));

    return new DrivePower(
        frontLeft / scale, frontRight / scale, backLeft / scale, backRight / scale);
  }
}
