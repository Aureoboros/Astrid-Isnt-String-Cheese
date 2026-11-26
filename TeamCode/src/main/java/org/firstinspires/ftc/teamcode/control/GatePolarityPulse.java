package org.firstinspires.ftc.teamcode.control;

/** Low-power diagnostic pulse; release both buttons before another pulse is allowed. */
public final class GatePolarityPulse {
  private final double powerLimit, duration;
  private double power, startedAt;
  private boolean armed = true;

  public GatePolarityPulse(double powerLimit, double duration) {
    if (!Double.isFinite(powerLimit) || powerLimit <= 0 || powerLimit > 1
        || !Double.isFinite(duration) || duration <= 0) {
      throw new IllegalArgumentException("Invalid polarity pulse settings");
    }
    this.powerLimit = powerLimit;
    this.duration = duration;
  }

  public double update(boolean positive, boolean negative, double now) {
    if (!Double.isFinite(now)) { power = 0; armed = false; return power; }
    if (!positive && !negative) { power = 0; armed = true; return power; }
    if (positive && negative) { power = 0; armed = false; return power; }
    if (armed) {
      armed = false;
      startedAt = now;
      power = positive ? powerLimit : -powerLimit;
    }
    if (now < startedAt || now - startedAt >= duration
        || (power > 0 && !positive) || (power < 0 && !negative)) { power = 0; }
    return power;
  }
}
