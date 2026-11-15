package org.firstinspires.ftc.teamcode.control;

import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;

/** Image-only horizontal centering; no assumed servo-angle mapping or robot-space distances. */
public final class ImagePanTracker {
  private final double min, center, max, start, gain, maxStep, deadband, settle, interval;
  private final PanTargetSelector selector;
  private double position, lastMove, frameAtMove, settledFrame, lastFrame;
  private double targetTx = Double.NaN;
  private int direction;
  private boolean started, tracking;
  private String status = "READY: no movement before Play";

  public ImagePanTracker(double min, double center, double max, double start,
      double gain, double maxStep, double deadband, double settle, double interval, double lostHold) {
    for (double value : new double[] {min, center, max, start, gain, maxStep, deadband,
        settle, interval, lostHold}) {
      if (!Double.isFinite(value)) { throw new IllegalArgumentException("Non-finite pan setting"); }
    }
    if (!(0 <= min && min < center && center < max && max <= 1 && min <= start && start <= max)
        || gain <= 0 || maxStep <= 0 || deadband < 0 || settle < 0 || interval <= 0 || lostHold < 0) {
      throw new IllegalArgumentException("Invalid image-pan settings");
    }
    this.min = min; this.center = center; this.max = max; this.start = start;
    this.gain = gain; this.maxStep = maxStep; this.deadband = deadband;
    this.settle = settle; this.interval = interval;
    selector = new PanTargetSelector(lostHold);
  }

  /** Called only after Play; returned commanded position initially equals configured start. */
  public void start(double now, double frameTimestamp) {
    if (started) { return; }
    started = true;
    move(start, now, frameTimestamp);
    status = "HOLD: verify direction, then X to track";
  }

  public void setDirection(int direction) {
    if (direction != 1 && direction != -1) { throw new IllegalArgumentException("Direction must be +/-1"); }
    this.direction = direction;
    tracking = false;
    selector.clear();
    targetTx = Double.NaN;
    status = "HOLD: direction selected; X to track";
  }

  public void clearDirection() {
    direction = 0;
    tracking = false;
    selector.clear();
    targetTx = Double.NaN;
    status = "BLOCKED: direction not selected";
  }

  public void toggleTracking() {
    if (!started || direction == 0) {
      status = "BLOCKED: select verified direction with D-pad left/right first";
      return;
    }
    tracking = !tracking;
    status = tracking ? "TRACKING: waiting for a fresh tag" : "HOLD: tracking OFF";
  }

  public void center(double now, double timestamp) {
    if (!started) { return; }
    tracking = false;
    selector.clear();
    targetTx = Double.NaN;
    move(center, now, timestamp);
    status = "HOLD: centered; tracking OFF";
  }

  /** Small manual direction-check steps are allowed only with tracking OFF. */
  public void nudge(double step, double now, double timestamp) {
    if (!started || tracking || !Double.isFinite(step)) { return; }
    manualOverride(step, now, timestamp);
  }

  /** Explicit manual override; caller must suspend update() until manual input is released. */
  public void manualOverride(double step, double now, double timestamp) {
    if (!started || !Double.isFinite(step)) { return; }
    move(position + step, now, timestamp);
    targetTx = Double.NaN;
    status = "MANUAL: automatic corrections suspended until release";
  }

  public void update(List<AprilTagAim> tags, double timestamp, double now) {
    if (!started || !tracking || !Double.isFinite(now)) { return; }
    AprilTagAim target = selector.select(tags, now);
    targetTx = target == null ? Double.NaN : target.getTxDegrees();
    if (target == null) { status = "HOLD: no locked tag visible (no search sweep)"; return; }
    if (now - lastMove < Math.max(settle, interval)) {
      status = "SETTLING: waiting before next correction";
      return;
    }
    if (!Double.isFinite(timestamp) || timestamp <= frameAtMove || timestamp <= lastFrame) { return; }
    lastFrame = timestamp;
    // Discard the first new frame after settling: it may have been captured during servo motion.
    if (!Double.isFinite(settledFrame)) {
      settledFrame = timestamp;
      status = "SETTLING: waiting for another fresh frame";
      return;
    }
    if (timestamp <= settledFrame) { return; }
    if (Math.abs(targetTx) <= deadband) { status = "TRACKING: horizontally centered"; return; }
    double change = Math.max(-maxStep, Math.min(maxStep, direction * gain * targetTx));
    double next = clamp(position + change);
    if (Math.abs(next - position) < 1e-6) {
      status = "HOLD: travel limit; tag not centered";
      return;
    }
    move(next, now, timestamp);
    status = "TRACKING: correcting horizontal offset";
  }

  private void move(double requested, double now, double timestamp) {
    if (!Double.isFinite(now)) { throw new IllegalArgumentException("Non-finite time"); }
    position = clamp(requested);
    lastMove = now;
    frameAtMove = Double.isFinite(timestamp) ? timestamp : Double.NEGATIVE_INFINITY;
    settledFrame = Double.NaN;
    lastFrame = frameAtMove;
  }

  private double clamp(double value) { return Math.max(min, Math.min(max, value)); }
  public void setTargetIdRange(int minId, int maxId) { selector.setIdRange(minId, maxId); }
  public double getPosition() { return started ? position : Double.NaN; }
  public boolean isTracking() { return started && tracking; }
  public int getDirection() { return direction; }
  public int getTargetId() { return selector.getId(); }
  public double getTargetTx() { return targetTx; }
  public String getStatus() { return status; }
  public void stop() { tracking = false; started = false; }
}
