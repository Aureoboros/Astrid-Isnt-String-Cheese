package org.firstinspires.ftc.teamcode.localization;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

/** Limelight UI / t6c_rs order: forward, right, up (meters), roll, pitch, yaw (degrees). */
public final class CameraMountPose {
  private static final String[] KEYS = {"rsf", "rss", "rsu", "rsroll", "rspitch", "rsyaw"};
  private final double[] values;

  public CameraMountPose(double forward, double right, double up,
      double roll, double pitch, double yaw) {
    values = new double[] {forward, right, up, roll, pitch, yaw};
    for (double value : values) {
      if (!Double.isFinite(value)) { throw new IllegalArgumentException("Non-finite mount pose"); }
    }
  }

  public double getYawDegrees() { return values[5]; }
  public double getForwardMeters() { return values[0]; }
  public double getRightMeters() { return values[1]; }
  public double getUpMeters() { return values[2]; }

  public CameraMountPose forResultYawSign(int sign) {
    if (sign != 1 && sign != -1) { throw new IllegalArgumentException("Calibrate result yaw sign"); }
    return new CameraMountPose(values[0], values[1], values[2], values[3], values[4], values[5] * sign);
  }

  public JsonObject updatePipeline(JsonObject original) {
    if (!original.has("pipeline_type")
        || !"pipe_fiducial".equals(original.get("pipeline_type").getAsString())) {
      throw new IllegalArgumentException("Pan requires a fiducial pipeline");
    }
    JsonObject updated = new JsonParser().parse(original.toString()).getAsJsonObject();
    for (int i = 0; i < KEYS.length; i++) {
      if (!original.has(KEYS[i])) { throw new IllegalArgumentException("Missing " + KEYS[i]); }
      updated.addProperty(KEYS[i], values[i]);
    }
    return updated;
  }

  public static CameraMountPose fromFrame(JsonObject frame) {
    if (!frame.has("t6c_rs") || !frame.get("t6c_rs").isJsonArray()) { return null; }
    JsonArray array = frame.getAsJsonArray("t6c_rs");
    if (array.size() != 6) { return null; }
    try {
      return new CameraMountPose(array.get(0).getAsDouble(), array.get(1).getAsDouble(),
          array.get(2).getAsDouble(), array.get(3).getAsDouble(),
          array.get(4).getAsDouble(), array.get(5).getAsDouble());
    } catch (RuntimeException invalid) { return null; }
  }

  public boolean matchesPipeline(JsonObject pipeline) {
    try {
      for (int i = 0; i < KEYS.length; i++) {
        if (!pipeline.has(KEYS[i])
            || !Double.isFinite(pipeline.get(KEYS[i]).getAsDouble())
            || Math.abs(pipeline.get(KEYS[i]).getAsDouble() - values[i]) > 0.000001) {
          return false;
        }
      }
      return true;
    } catch (RuntimeException invalid) { return false; }
  }

  public boolean matchesFrame(CameraMountPose actual) {
    if (actual == null) { return false; }
    for (int i = 0; i < values.length; i++) {
      if (Math.abs(values[i] - actual.values[i]) > (i < 3 ? 0.001 : 0.1)) { return false; }
    }
    return true;
  }
}
