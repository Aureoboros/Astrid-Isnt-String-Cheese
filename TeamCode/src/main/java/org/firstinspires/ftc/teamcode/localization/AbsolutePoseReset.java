package org.firstinspires.ftc.teamcode.localization;

import java.util.ArrayList;
import java.util.List;

/** Nonblocking one-shot correction from consistent, distinct mapped-tag field poses. */
public final class AbsolutePoseReset {
  private final int requiredFrames;
  private final double timeout, spreadLimit, headingSpreadLimit, fieldHalfSize;
  private final List<double[]> offsets = new ArrayList<>();
  private boolean active;
  private double startedAt, lastTimestamp;
  private PoseEstimate correction;
  private String status = "Ready: A resets from a mapped field pose";

  public AbsolutePoseReset(int requiredFrames, double timeout, double spreadLimit,
      double headingSpreadLimit, double fieldHalfSize) {
    if (requiredFrames < 2) { throw new IllegalArgumentException("Need multiple reset frames"); }
    for (double value : new double[] {timeout, spreadLimit, headingSpreadLimit, fieldHalfSize}) {
      if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException("Invalid reset limit"); }
    }
    this.requiredFrames = requiredFrames; this.timeout = timeout; this.spreadLimit = spreadLimit;
    this.headingSpreadLimit = headingSpreadLimit; this.fieldHalfSize = fieldHalfSize;
  }

  public void begin(double now) {
    if (!Double.isFinite(now)) { return; }
    offsets.clear(); correction = null; lastTimestamp = Double.NEGATIVE_INFINITY;
    startedAt = now; active = true;
    status = "Centering camera; waiting for mapped AprilTag frames";
  }

  public void update(double now, boolean connected, boolean cameraReady, boolean driverActive,
      double timestamp, PoseEstimate odometry, PoseEstimate fieldPose) {
    if (!active) { return; }
    if (!Double.isFinite(now) || now - startedAt >= timeout) {
      active = false; status = "No reset: fresh, stable mapped pose unavailable"; return;
    }
    if (!connected) { status = "Waiting: Limelight disconnected"; return; }
    if (driverActive) { offsets.clear(); status = "Release drive controls to reset pose"; return; }
    if (!cameraReady) { status = "Waiting: camera centered mount and fresh frame"; return; }
    if (!finite(odometry) || !finite(fieldPose) || !inside(fieldPose)
        || !Double.isFinite(timestamp) || timestamp <= lastTimestamp) {
      status = "Waiting: valid mapped field pose"; return;
    }
    lastTimestamp = timestamp;
    offsets.add(new double[] {fieldPose.getX() - odometry.getX(), fieldPose.getY() - odometry.getY(),
        wrap(fieldPose.getHeadingDegrees() - odometry.getHeadingDegrees())});
    status = "Collecting field pose: " + offsets.size() + "/" + requiredFrames;
    if (offsets.size() < requiredFrames) { return; }
    double x = 0, y = 0, sin = 0, cos = 0;
    for (double[] o : offsets) {
      x += o[0]; y += o[1]; sin += Math.sin(Math.toRadians(o[2])); cos += Math.cos(Math.toRadians(o[2]));
    }
    x /= offsets.size(); y /= offsets.size();
    double heading = Math.toDegrees(Math.atan2(sin, cos));
    for (double[] o : offsets) {
      if (Math.hypot(o[0] - x, o[1] - y) > spreadLimit
          || Math.abs(wrap(o[2] - heading)) > headingSpreadLimit) {
        active = false; status = "No reset: field pose samples disagree"; return;
      }
    }
    PoseEstimate result = new PoseEstimate(odometry.getX() + x, odometry.getY() + y,
        wrap(odometry.getHeadingDegrees() + heading));
    active = false;
    if (!inside(result)) { status = "No reset: pose outside field"; return; }
    correction = result;
    status = "Applied absolute field position and heading";
  }

  public void unavailable(String reason) { active = false; correction = null; status = "No reset: " + reason; }
  public boolean isActive() { return active; }
  public int getFrameCount() { return offsets.size(); }
  public String getStatus() { return status; }
  public PoseEstimate takeCorrection() { PoseEstimate result = correction; correction = null; return result; }
  private boolean inside(PoseEstimate p) { return Math.abs(p.getX()) <= fieldHalfSize && Math.abs(p.getY()) <= fieldHalfSize; }
  private static boolean finite(PoseEstimate p) {
    return p != null && Double.isFinite(p.getX()) && Double.isFinite(p.getY()) && Double.isFinite(p.getHeadingDegrees());
  }
  private static double wrap(double degrees) {
    return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)), Math.cos(Math.toRadians(degrees))));
  }
}
