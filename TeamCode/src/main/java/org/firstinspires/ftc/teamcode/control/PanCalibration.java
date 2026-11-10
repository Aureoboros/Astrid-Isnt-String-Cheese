package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.localization.CameraMountPose;

/** Three-point servo/yaw calibration and rigid-body lens translation about a vertical pan pivot. */
public final class PanCalibration {
  private final double min, center, max, yawMin, yawCenter, yawMax;
  private final int yawSign;
  private final double pivotForward, pivotRight, pivotUp, lensForward, lensRight, lensUp, roll, pitch;

  public PanCalibration(double min, double center, double max,
      double yawMin, double yawCenter, double yawMax, int yawSign,
      double pivotForward, double pivotRight, double pivotUp,
      double lensForward, double lensRight, double lensUp, double roll, double pitch) {
    double[] inputs = {min, center, max, yawMin, yawCenter, yawMax, pivotForward,
        pivotRight, pivotUp, lensForward, lensRight, lensUp, roll, pitch};
    for (double input : inputs) {
      if (!Double.isFinite(input)) { throw new IllegalArgumentException("Pan calibration incomplete"); }
    }
    if (!(0 <= min && min < center && center < max && max <= 1)
        || (yawSign != 1 && yawSign != -1)
        || (yawCenter - yawMin) * (yawMax - yawCenter) <= 0) {
      throw new IllegalArgumentException("Invalid pan limits or monotonic yaw mapping");
    }
    this.min = min; this.center = center; this.max = max;
    this.yawMin = yawMin; this.yawCenter = yawCenter; this.yawMax = yawMax;
    this.yawSign = yawSign;
    this.pivotForward = pivotForward; this.pivotRight = pivotRight; this.pivotUp = pivotUp;
    this.lensForward = lensForward; this.lensRight = lensRight; this.lensUp = lensUp;
    this.roll = roll; this.pitch = pitch;
  }

  public double clamp(double position) {
    if (!Double.isFinite(position)) { throw new IllegalArgumentException("Non-finite position"); }
    return Math.max(min, Math.min(max, position));
  }

  public double yawAt(double position) {
    double p = clamp(position);
    return p <= center
        ? yawMin + (p - min) / (center - min) * (yawCenter - yawMin)
        : yawCenter + (p - center) / (max - center) * (yawMax - yawCenter);
  }

  public CameraMountPose mountAt(double position) {
    double yaw = yawAt(position);
    double radians = Math.toRadians((yaw - yawCenter) * yawSign);
    // Positive physical CCW/left pan rotates forward offset toward negative robot-right.
    double forward = Math.cos(radians) * lensForward + Math.sin(radians) * lensRight;
    double right = -Math.sin(radians) * lensForward + Math.cos(radians) * lensRight;
    return new CameraMountPose(pivotForward + forward, pivotRight + right, pivotUp + lensUp,
        roll, pitch, yaw);
  }

  public double stepTowardTag(double position, double tx, int direction,
      double gain, double maxStep, double deadband) {
    if ((direction != 1 && direction != -1) || !Double.isFinite(gain) || gain <= 0
        || !Double.isFinite(maxStep) || maxStep <= 0
        || !Double.isFinite(deadband) || deadband < 0) {
      throw new IllegalArgumentException("Invalid pan feedback settings");
    }
    double p = clamp(position);
    if (!Double.isFinite(tx) || Math.abs(tx) <= deadband) { return p; }
    double change = Math.max(-maxStep, Math.min(maxStep, direction * gain * tx));
    return clamp(p + change);
  }
}
