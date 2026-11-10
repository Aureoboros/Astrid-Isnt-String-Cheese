package org.firstinspires.ftc.teamcode.control;

import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;

/** Keeps a tag lock through brief losses; reacquires only an actually visible tag. */
public final class PanTargetSelector {
  private final double lostHoldSeconds;
  private int id = -1;
  private int minId = Integer.MIN_VALUE, maxId = Integer.MAX_VALUE;
  private double lastSeen = Double.NEGATIVE_INFINITY;
  public PanTargetSelector(double lostHoldSeconds) {
    if (!Double.isFinite(lostHoldSeconds) || lostHoldSeconds < 0) {
      throw new IllegalArgumentException("Invalid target hold time");
    }
    this.lostHoldSeconds = lostHoldSeconds;
  }
  public AprilTagAim select(List<AprilTagAim> targets, double now) {
    for (AprilTagAim aim : targets) {
      if (aim.getId() == id && Double.isFinite(aim.getTxDegrees())) { lastSeen = now; return aim; }
    }
    if (id >= 0 && now - lastSeen < lostHoldSeconds) { return null; }
    AprilTagAim nearest = null;
    for (AprilTagAim aim : targets) {
      if (!Double.isFinite(aim.getTxDegrees()) || aim.getId() < minId || aim.getId() > maxId) { continue; }
      if (nearest == null || Math.abs(aim.getTxDegrees()) < Math.abs(nearest.getTxDegrees())) { nearest = aim; }
    }
    if (nearest != null) { id = nearest.getId(); lastSeen = now; }
    return nearest;
  }
  public void clear() { id = -1; lastSeen = Double.NEGATIVE_INFINITY; }
  public void setIdRange(int minId, int maxId) {
    if (minId > maxId) { throw new IllegalArgumentException("Invalid tag ID range"); }
    this.minId = minId; this.maxId = maxId;
    if (id < minId || id > maxId) { clear(); }
  }
  public int getId() { return id; }
}
