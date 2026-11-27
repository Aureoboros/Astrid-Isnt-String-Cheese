package org.firstinspires.ftc.teamcode.localization;

import com.google.gson.JsonObject;
import com.google.gson.JsonArray;

/** Mapped Limelight botpose in the operator-confirmed Pedro field frame. */
public final class LimelightFieldPose {
  private LimelightFieldPose() {}

  /** Reject missing/malformed arrays rather than accepting SDK fallback zeros as the origin. */
  public static PoseEstimate fromFrame(JsonObject frame) {
    try {
      JsonArray pose = frame.getAsJsonArray("botpose");
      if (pose == null || pose.size() != 6) { return null; }
      return fromMeters(frame.get("botpose_tagcount").getAsInt(),
          pose.get(0).getAsDouble(), pose.get(1).getAsDouble(), pose.get(2).getAsDouble(),
          pose.get(3).getAsDouble(), pose.get(4).getAsDouble(), pose.get(5).getAsDouble());
    } catch (RuntimeException malformed) { return null; }
  }

  public static PoseEstimate fromMeters(int mappedTags, double x, double y, double z,
      double roll, double pitch, double yaw) {
    if (mappedTags < 1) { return null; }
    for (double value : new double[] {x, y, z, roll, pitch, yaw}) {
      if (!Double.isFinite(value)) { return null; }
    }
    if (Math.abs(z) > 0.1524 || Math.abs(roll) > 15 || Math.abs(pitch) > 15) { return null; }
    return new PoseEstimate(x / 0.0254, y / 0.0254, yaw);
  }
}
