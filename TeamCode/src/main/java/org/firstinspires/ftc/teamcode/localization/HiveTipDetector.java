package org.firstinspires.ftc.teamcode.localization;

import java.util.List;
import org.firstinspires.ftc.teamcode.config.HiveAutoConfig;

/** Same-tag, fresh-frame high-to-low evidence; this is not official damper-contact confirmation. */
public final class HiveTipDetector {
  private final int minId;
  private int lockedId = -1;
  private int highFrames, lowFrames;
  private double lastTimestamp = Double.NaN, highSince = -1, lowSince = -1;
  private boolean highConfirmed, highVisible, lowCandidate, tipped;

  public HiveTipDetector(int minId) { this.minId = minId; }

  public void update(double now, double timestamp, List<AprilTagObservation> tags) {
    if (!Double.isFinite(timestamp) || tags.isEmpty()) {
      resetStability();
      return;
    }
    if (Double.isFinite(lastTimestamp) && timestamp <= lastTimestamp) { return; }
    lastTimestamp = timestamp;
    AprilTagObservation selected = null;
    for (AprilTagObservation tag : tags) {
      if (tag.getId() >= minId && tag.getId() <= minId + 3
          && (lockedId < 0 || tag.getId() == lockedId)
          && Double.isFinite(tag.getHeightInches())
          && Double.isFinite(tag.getHorizontalDistanceInches())
          && tag.getForwardInches() > 0 && tag.getHorizontalDistanceInches() >= 12
          && tag.getHorizontalDistanceInches() <= 42) {
        selected = tag;
        break;
      }
    }
    if (selected == null) { resetStability(); return; }
    double height = selected.getHeightInches() + HiveAutoConfig.ROBOT_ORIGIN_HEIGHT_INCHES;
    boolean high = height >= HiveAutoConfig.HIGH_MIN_INCHES && height <= HiveAutoConfig.HIGH_MAX_INCHES;
    highVisible = high;
    lowCandidate = height >= HiveAutoConfig.LOW_MIN_INCHES && height <= HiveAutoConfig.LOW_MAX_INCHES;
    if (!highConfirmed) {
      if (high) {
        if (lockedId < 0) { lockedId = selected.getId(); }
        if (highSince < 0) { highSince = now; }
        highFrames++;
        highConfirmed = highFrames >= 3 && now - highSince >= 0.25;
      } else { highSince = -1; highFrames = 0; }
    } else if (lowCandidate) {
      if (lowSince < 0) { lowSince = now; }
      lowFrames++;
      tipped |= lowFrames >= 3 && now - lowSince >= 0.35;
    } else { lowSince = -1; lowFrames = 0; }
  }

  private void resetStability() {
    highSince = lowSince = -1;
    highFrames = lowFrames = 0;
    highVisible = lowCandidate = false;
  }

  public boolean isHighConfirmed() { return highConfirmed; }
  public boolean isHighVisible() { return highVisible; }
  public boolean isLowCandidate() { return lowCandidate; }
  public boolean isTipped() { return tipped; }
  public int getLockedId() { return lockedId; }
}
