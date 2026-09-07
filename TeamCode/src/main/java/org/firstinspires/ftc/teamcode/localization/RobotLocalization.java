package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

/** Coordinates independent odometry positioning and Limelight AprilTag ranging. */
public final class RobotLocalization {
  private final TwoWheelOdometryLocalizer odometry;
  private final AprilTagTracker aprilTags;

  public RobotLocalization(HardwareMap hardwareMap) {
    odometry = new TwoWheelOdometryLocalizer(hardwareMap);
    aprilTags = new AprilTagTracker(hardwareMap);
  }

  public void start() {
    aprilTags.start();
  }

  public void stop() {
    aprilTags.stop();
  }

  public void update() {
    odometry.update();
    aprilTags.update();
  }

  public void resetPose(double x, double y, double headingDegrees) {
    odometry.resetPose(x, y, headingDegrees);
  }

  /** Returns translation from the two odometry pods and heading from the hub IMU. */
  public PoseEstimate getPoseEstimate() {
    return odometry.getPoseEstimate();
  }

  public List<AprilTagObservation> getAprilTagObservations() {
    return aprilTags.getObservations();
  }

  public AprilTagObservation getAprilTagObservation(int tagId) {
    return aprilTags.getObservation(tagId);
  }

  public AprilTagObservation getNearestAprilTag() {
    return aprilTags.getNearestObservation();
  }

  public boolean isLimelightConnected() {
    return aprilTags.isConnected();
  }
}
