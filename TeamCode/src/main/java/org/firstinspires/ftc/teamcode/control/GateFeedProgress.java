package org.firstinspires.ftc.teamcode.control;

/** Feed time credited from forward encoder travel; reverse travel must be regained first. */
public final class GateFeedProgress {
  private final double ticksPerSecond;
  private final int direction;
  private int previous;
  private double debt;
  private boolean initialized;
  public GateFeedProgress(double ticksPerRev, double signedRpm) {
    if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0
        || !Double.isFinite(signedRpm) || signedRpm == 0) {
      throw new IllegalArgumentException("Invalid feed encoder calibration");
    }
    ticksPerSecond = ticksPerRev * Math.abs(signedRpm) / 60;
    direction = signedRpm > 0 ? 1 : -1;
  }
  public void start(int position) { previous = position; debt = 0; initialized = true; }
  public double update(int position, boolean advancing, double elapsedSeconds) {
    if (!initialized) { start(position); return 0; }
    double delta = ((long) position - previous) * direction;
    previous = position;
    if (delta < 0) { debt -= delta; return 0; }
    double repay = Math.min(debt, delta);
    debt -= repay;
    if (!advancing || !Double.isFinite(elapsedSeconds) || elapsedSeconds <= 0) { return 0; }
    return Math.min(elapsedSeconds, (delta - repay) / ticksPerSecond);
  }
}
