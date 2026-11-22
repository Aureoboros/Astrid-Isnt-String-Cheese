package org.firstinspires.ftc.teamcode.control;

/** Encoder-based stall detection and bounded reverse/retry; update every loop, never sleep. */
public final class GateJamRecoveryController {
  public enum State { OFF, RUNNING, STOPPING, REVERSING, RESTARTING, FAULT }
  private final double ticksPerRev, startupGrace, stallWindow, minSpeedFraction;
  private final double stopTime, reverseTime, restartPause, reverseRpm, healthyReset, minRpm;
  private final int maxAttempts;
  private State state = State.OFF;
  private double requested, output, stateSince, windowSince, healthySince = Double.NaN;
  private int windowPosition, attempts, recoveries;

  public GateJamRecoveryController(double ticksPerRev, double startupGrace, double stallWindow,
      double minSpeedFraction, double stopTime, double reverseTime, double restartPause,
      double reverseRpm, double healthyReset, double minRpm, int maxAttempts) {
    for (double value : new double[] {ticksPerRev, startupGrace, stallWindow, minSpeedFraction,
        stopTime, reverseTime, restartPause, reverseRpm, healthyReset, minRpm}) {
      if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException("Invalid jam setting"); }
    }
    if (minSpeedFraction >= 1 || maxAttempts < 1) { throw new IllegalArgumentException("Invalid jam limit"); }
    this.ticksPerRev = ticksPerRev; this.startupGrace = startupGrace;
    this.stallWindow = stallWindow; this.minSpeedFraction = minSpeedFraction;
    this.stopTime = stopTime; this.reverseTime = reverseTime; this.restartPause = restartPause;
    this.reverseRpm = reverseRpm; this.healthyReset = healthyReset; this.minRpm = minRpm;
    this.maxAttempts = maxAttempts;
  }

  /** requestedRpm is signed in the motor's configured direction; zero cancels and clears fault. */
  public double update(double requestedRpm, double actualRpm, int position, double now) {
    if (!Double.isFinite(requestedRpm) || !Double.isFinite(now)) {
      fault(); return output;
    }
    if (requestedRpm == 0) { stop(); return output; }
    if (!Double.isFinite(actualRpm)) { fault(); return output; }
    if (state == State.FAULT) { return 0; }
    if (state == State.OFF || Double.compare(requestedRpm, requested) != 0) {
      requested = requestedRpm;
      beginRunning(position, now); // Driver direction/RPM changes cancel an active reverse pulse.
    }
    switch (state) {
      case STOPPING:
        if (now - stateSince >= stopTime) {
          state = State.REVERSING; stateSince = now;
          output = -Math.copySign(Math.min(Math.abs(requested), reverseRpm), requested);
        }
        break;
      case REVERSING:
        if (now - stateSince >= reverseTime) {
          state = State.RESTARTING; stateSince = now; output = 0;
        }
        break;
      case RESTARTING:
        if (now - stateSince >= restartPause) { beginRunning(position, now); }
        break;
      case RUNNING:
        if (now - stateSince < startupGrace || Math.abs(requested) < minRpm) {
          windowSince = Math.max(now, stateSince + startupGrace);
          windowPosition = position; healthySince = Double.NaN;
          break;
        }
        double elapsed = now - windowSince;
        if (elapsed < stallWindow) { break; }
        double expectedTicks = Math.abs(requested) * ticksPerRev / 60 * elapsed;
        double movement = Math.abs((long) position - windowPosition);
        boolean stalled = movement < expectedTicks * minSpeedFraction
            && Math.abs(actualRpm) < Math.abs(requested) * minSpeedFraction;
        windowSince = now; windowPosition = position;
        if (stalled) {
          healthySince = Double.NaN;
          if (attempts >= maxAttempts) { fault(); break; }
          attempts++; recoveries++;
          state = State.STOPPING; stateSince = now; output = 0;
        } else if (movement >= expectedTicks * minSpeedFraction
            && Math.abs(actualRpm) >= Math.abs(requested) * minSpeedFraction) {
          if (!Double.isFinite(healthySince)) { healthySince = now; }
          if (now - healthySince >= healthyReset) { attempts = 0; }
        } else { healthySince = Double.NaN; }
        break;
      default: break;
    }
    return output;
  }

  private void beginRunning(int position, double now) {
    state = State.RUNNING; output = requested; stateSince = now;
    windowSince = now + startupGrace; windowPosition = position; healthySince = Double.NaN;
  }
  private void fault() { state = State.FAULT; output = 0; }
  public void stop() {
    state = State.OFF; requested = output = 0; attempts = 0; healthySince = Double.NaN;
  }
  public State getState() { return state; }
  public int getAttempts() { return attempts; }
  public int getRecoveryCount() { return recoveries; }
  public double getOutputRpm() { return output; }
}
