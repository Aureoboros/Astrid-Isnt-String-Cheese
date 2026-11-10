package org.firstinspires.ftc.teamcode.localization;

/** Rejects robot-relative poses until mounting update, settling, and a matching NEW frame agree. */
public final class PanPoseGate {
  private final double settleSeconds;
  private CameraMountPose expected;
  private double oldTimestamp;
  private double readyAfter = Double.POSITIVE_INFINITY;
  private double firstSettledTimestamp = Double.NaN;

  public PanPoseGate(double settleSeconds) {
    if (!Double.isFinite(settleSeconds) || settleSeconds < 0) {
      throw new IllegalArgumentException("Invalid settling time");
    }
    this.settleSeconds = settleSeconds;
  }

  public void beginMove(CameraMountPose expected, double oldTimestamp) {
    this.expected = expected;
    this.oldTimestamp = Double.isFinite(oldTimestamp) ? oldTimestamp : Double.NEGATIVE_INFINITY;
    readyAfter = Double.POSITIVE_INFINITY;
    firstSettledTimestamp = Double.NaN;
  }

  public void acknowledgeUpdate(double now) {
    if (!Double.isFinite(now)) { throw new IllegalArgumentException("Invalid acknowledgement time"); }
    readyAfter = now + settleSeconds;
  }

  public boolean ready(CameraMountPose frameMount, double timestamp, double now) {
    if (expected == null || !Double.isFinite(now) || !Double.isFinite(timestamp) || timestamp <= oldTimestamp
        || now < readyAfter || !expected.matchesFrame(frameMount)) { return false; }
    if (!Double.isFinite(firstSettledTimestamp)) {
      firstSettledTimestamp = timestamp;
      return false;
    }
    return timestamp > firstSettledTimestamp;
  }
}
