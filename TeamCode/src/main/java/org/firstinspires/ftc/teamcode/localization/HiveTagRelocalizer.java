package org.firstinspires.ftc.teamcode.localization;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * One-shot absolute X/Y fix from hive AprilTags whose FIELD positions were measured by the team.
 *
 * <p>Each fresh Limelight frame gives, for every visible mapped tag, where the robot center must
 * be: tag field position minus the tag's robot-relative offset rotated by the odometry heading.
 * The offset between that vision position and the odometry position at the same instant is
 * averaged over several distinct frames. Applying the offset to the CURRENT odometry pose keeps
 * the fix correct even if the robot creeps slightly while holding position. Heading is NOT
 * corrected; the Pinpoint IMU heading is far more reliable than a single-tag yaw estimate.
 *
 * <p>Pure Java so it can be unit-tested without robot hardware.
 */
public final class HiveTagRelocalizer {
  public enum Status { WAITING, NO_TAG_POSITIONS, OK, TOO_NOISY, CORRECTION_TOO_LARGE }

  /** Closer than this the tag may be partly out of frame; farther the range noise grows. */
  public static final double MIN_TAG_DISTANCE_INCHES = 12;
  public static final double MAX_TAG_DISTANCE_INCHES = 60;

  private final Map<Integer, double[]> tagFieldXY = new HashMap<>();
  private final int requiredFrames;
  private final double maxSpreadInches;
  private final double maxCorrectionInches;
  private final List<double[]> offsets = new ArrayList<>();
  private final List<Integer> usedTagIds = new ArrayList<>();
  private double lastTimestamp = Double.NaN;

  /**
   * @param tagTable rows of {id, fieldX, fieldY} in field-center inches; rows with NaN are ignored
   * @param requiredFrames distinct Limelight frames to average before the fix is accepted
   * @param maxSpreadInches largest allowed distance of any frame's offset from the mean offset
   * @param maxCorrectionInches a larger fix is rejected as a wrong tag map or bad camera transform
   */
  public HiveTagRelocalizer(double[][] tagTable, int requiredFrames, double maxSpreadInches,
      double maxCorrectionInches) {
    if (requiredFrames < 1 || !(maxSpreadInches > 0) || !(maxCorrectionInches > 0)) {
      throw new IllegalArgumentException("Relocalizer limits must be positive");
    }
    for (double[] row : tagTable) {
      if (row.length != 3) {
        throw new IllegalArgumentException("Tag rows must be {id, fieldX, fieldY}");
      }
      if (Double.isFinite(row[1]) && Double.isFinite(row[2])) {
        tagFieldXY.put((int) row[0], new double[] {row[1], row[2]});
      }
    }
    this.requiredFrames = requiredFrames;
    this.maxSpreadInches = maxSpreadInches;
    this.maxCorrectionInches = maxCorrectionInches;
  }

  public boolean hasTagPositions() { return !tagFieldXY.isEmpty(); }

  /**
   * Adds one Limelight frame. Repeated polls of the same frame, frames without a usable mapped
   * tag, and non-finite odometry are ignored.
   *
   * @return true when the frame contributed a sample
   */
  public boolean addFrame(double timestamp, double odoX, double odoY, double odoHeadingRadians,
      List<AprilTagObservation> tags) {
    if (!Double.isFinite(timestamp) || !Double.isFinite(odoX) || !Double.isFinite(odoY)
        || !Double.isFinite(odoHeadingRadians) || tags == null) {
      return false;
    }
    if (Double.isFinite(lastTimestamp) && timestamp <= lastTimestamp) {
      return false;
    }
    lastTimestamp = timestamp;
    double sumX = 0, sumY = 0;
    int count = 0;
    for (AprilTagObservation tag : tags) {
      double[] field = tagFieldXY.get(tag.getId());
      if (field == null || !usable(tag)) {
        continue;
      }
      double[] robot = robotFromTag(field[0], field[1], tag.getForwardInches(),
          tag.getLeftInches(), odoHeadingRadians);
      sumX += robot[0];
      sumY += robot[1];
      count++;
      if (!usedTagIds.contains(tag.getId())) {
        usedTagIds.add(tag.getId());
      }
    }
    if (count == 0) {
      return false;
    }
    offsets.add(new double[] {sumX / count - odoX, sumY / count - odoY});
    return true;
  }

  public int getFrameCount() { return offsets.size(); }

  public List<Integer> getUsedTagIds() { return Collections.unmodifiableList(usedTagIds); }

  /** Mean vision-minus-odometry offset {dx, dy}, or NaNs before any sample. */
  public double[] getOffset() {
    if (offsets.isEmpty()) {
      return new double[] {Double.NaN, Double.NaN};
    }
    double sumX = 0, sumY = 0;
    for (double[] o : offsets) {
      sumX += o[0];
      sumY += o[1];
    }
    return new double[] {sumX / offsets.size(), sumY / offsets.size()};
  }

  /** Largest distance between any single frame's offset and the mean offset. */
  public double getSpreadInches() {
    double[] mean = getOffset();
    double spread = 0;
    for (double[] o : offsets) {
      spread = Math.max(spread, Math.hypot(o[0] - mean[0], o[1] - mean[1]));
    }
    return offsets.isEmpty() ? Double.NaN : spread;
  }

  public double getCorrectionInches() {
    double[] mean = getOffset();
    return Math.hypot(mean[0], mean[1]);
  }

  public Status getStatus() {
    if (!hasTagPositions()) {
      return Status.NO_TAG_POSITIONS;
    }
    if (offsets.size() < requiredFrames) {
      return Status.WAITING;
    }
    if (getSpreadInches() > maxSpreadInches) {
      return Status.TOO_NOISY;
    }
    if (getCorrectionInches() > maxCorrectionInches) {
      return Status.CORRECTION_TOO_LARGE;
    }
    return Status.OK;
  }

  /** Corrected {x, y} for the given current odometry position; only meaningful when OK. */
  public double[] corrected(double odoX, double odoY) {
    double[] mean = getOffset();
    return new double[] {odoX + mean[0], odoY + mean[1]};
  }

  /** Robot center from a tag's field position and its robot-relative forward/left offset. */
  public static double[] robotFromTag(double tagX, double tagY, double forwardInches,
      double leftInches, double headingRadians) {
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);
    return new double[] {
        tagX - (forwardInches * cos - leftInches * sin),
        tagY - (forwardInches * sin + leftInches * cos)};
  }

  /** Tag field position implied by a robot pose; used to display/measure the tag map. */
  public static double[] tagFromRobot(double robotX, double robotY, double forwardInches,
      double leftInches, double headingRadians) {
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);
    return new double[] {
        robotX + forwardInches * cos - leftInches * sin,
        robotY + forwardInches * sin + leftInches * cos};
  }

  private static boolean usable(AprilTagObservation tag) {
    return Double.isFinite(tag.getForwardInches()) && Double.isFinite(tag.getLeftInches())
        && tag.getForwardInches() > 0
        && tag.getHorizontalDistanceInches() >= MIN_TAG_DISTANCE_INCHES
        && tag.getHorizontalDistanceInches() <= MAX_TAG_DISTANCE_INCHES;
  }
}
