package org.firstinspires.ftc.teamcode.control;

import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;

/** Manual pan takes priority; releasing it restores the previous automatic-tracking request. */
public final class ServoTrackingTestControls {
  private final ImagePanTracker aim;
  private final double step, repeatSeconds;
  private boolean previousX, previousY, manual;
  private int previousDirection;
  private double lastNudge = Double.NEGATIVE_INFINITY;

  public ServoTrackingTestControls(ImagePanTracker aim, double step, double repeatSeconds) {
    if (!Double.isFinite(step) || step <= 0
        || !Double.isFinite(repeatSeconds) || repeatSeconds <= 0) {
      throw new IllegalArgumentException("Invalid manual pan timing/step");
    }
    this.aim = aim;
    this.step = step;
    this.repeatSeconds = repeatSeconds;
  }

  public void start(double now, double timestamp) {
    aim.start(now, timestamp);
    if (aim.getDirection() != 0 && !aim.isTracking()) { aim.toggleTracking(); }
  }

  /** Inputs are merged across gamepads by the OpMode. Left/right mean physical camera direction. */
  public void update(boolean x, boolean y, boolean left, boolean right,
      List<AprilTagAim> tags, double timestamp, double now) {
    if (y && !previousY) {
      aim.center(now, timestamp); // Center is a stop/hold request, even if X is also pressed.
      manual = false;
    } else {
      if (x && !previousX && !y) { aim.toggleTracking(); }
      manual = !y && (left || right);
      if (manual) {
        int direction = (left ? 1 : 0) - (right ? 1 : 0);
        if (direction != 0 && aim.getDirection() != 0
            && (direction != previousDirection || now - lastNudge >= repeatSeconds)) {
          // Positive tx is right; the opposite of its feedback sign pans physically left.
          aim.manualOverride(-aim.getDirection() * direction * step, now, timestamp);
          lastNudge = now;
        }
        // No automatic update while either manual button is held, including opposing inputs.
      } else if (!y) {
        aim.update(tags, timestamp, now);
      }
    }
    previousDirection = manual ? (left ? 1 : 0) - (right ? 1 : 0) : 0;
    previousX = x;
    previousY = y;
  }

  public boolean isManual() { return manual; }
}
