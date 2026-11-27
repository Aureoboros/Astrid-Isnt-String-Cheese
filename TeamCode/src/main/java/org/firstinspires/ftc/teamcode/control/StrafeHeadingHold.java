package org.firstinspires.ftc.teamcode.control;

/** Captures heading at strafe start; returns Pedro CCW rotation with immediate driver override. */
public final class StrafeHeadingHold {
  private final double kP, kI, kD, maxTurn, maxIntegral, deadbandDegrees;
  private boolean active;
  private double target, error, correction, integral, lastError, lastTime;

  public StrafeHeadingHold(double kP, double kI, double kD, double maxTurn,
      double maxIntegral, double deadbandDegrees) {
    for (double v : new double[] {kP, kI, kD, maxIntegral, deadbandDegrees}) {
      if (!Double.isFinite(v) || v < 0) { throw new IllegalArgumentException("Invalid heading PID setting"); }
    }
    if (!Double.isFinite(maxTurn) || maxTurn <= 0 || maxTurn > 1) {
      throw new IllegalArgumentException("Invalid heading turn limit");
    }
    this.kP = kP; this.kI = kI; this.kD = kD; this.maxTurn = maxTurn;
    this.maxIntegral = maxIntegral; this.deadbandDegrees = deadbandDegrees;
    reset();
  }

  /** Heading is radians; time is monotonic seconds; rotation is positive counterclockwise. */
  public double update(double strafe, double requestedTurn, double heading, double now, boolean ready) {
    if (!ready || !Double.isFinite(strafe) || !Double.isFinite(requestedTurn)
        || !Double.isFinite(heading) || !Double.isFinite(now)) {
      reset(); return 0;
    }
    // Any nonzero mapped trigger command wins, including squared commands smaller than 0.05.
    if (requestedTurn != 0 || Math.abs(strafe) <= 0.05) {
      reset(); return requestedTurn;
    }
    if (!active) {
      active = true; target = heading; lastTime = now; lastError = 0;
      return 0;
    }
    error = Math.toDegrees(wrap(target - heading));
    double dt = now - lastTime;
    double derivative = 0;
    if (dt > 0 && dt <= 0.25) {
      derivative = Math.toDegrees(wrap(Math.toRadians(error - lastError))) / dt;
      // Do not accumulate integral while the previous output is saturated in this direction.
      if (Math.abs(correction) < maxTurn || error * correction < 0) {
        integral = clip(integral + error * dt, maxIntegral);
      }
    } else {
      integral = 0; // Stale or discontinuous timing must not create a derivative kick or windup.
    }
    lastTime = now; lastError = error;
    if (Math.abs(error) <= deadbandDegrees) {
      integral = 0; correction = 0;
    } else {
      correction = clip(kP * error + kI * integral + kD * derivative, maxTurn);
    }
    return correction;
  }

  public void reset() {
    active = false; target = Double.NaN; error = correction = integral = lastError = 0;
    lastTime = Double.NaN;
  }
  public boolean isActive() { return active; }
  public double getTargetDegrees() { return Math.toDegrees(target); }
  public double getErrorDegrees() { return error; }
  public double getCorrection() { return correction; }
  private static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
  private static double clip(double value, double limit) { return Math.max(-limit, Math.min(limit, value)); }
}
