package org.firstinspires.ftc.teamcode.control;

/** Nonblocking one-ball indexing. An interrupted index latches FAULT: no blind restart/re-feed. */
public final class NectarSeparatorController {
  public enum State { OFF, WAIT_FOR_RPM, INDEXING, RECOVERING, FAULT }
  private final int indexTicks, tolerance;
  private final double rpmTolerance, minRpm, rpmStable, interval, settle, timeout;
  private State state = State.OFF;
  private boolean initialized;
  private int target, completed;
  private double stableSince = Double.NaN, positionSince = Double.NaN;
  private double indexStarted, recoveredAt, selectedRpm = Double.NaN;
  private String status = "OFF: X starts shooter and requests feeding";

  public NectarSeparatorController(int indexTicks, int tolerance, double rpmTolerance,
      double minRpm, double rpmStable, double interval, double settle, double timeout) {
    if (tolerance < 0 || Math.abs((long) indexTicks) <= tolerance
        || !Double.isFinite(rpmTolerance) || rpmTolerance <= 0
        || !Double.isFinite(minRpm) || minRpm <= rpmTolerance
        || !Double.isFinite(rpmStable) || rpmStable <= 0
        || !Double.isFinite(interval) || interval < 0.1
        || !Double.isFinite(settle) || settle < 0
        || !Double.isFinite(timeout) || timeout <= settle) {
      throw new IllegalArgumentException("Invalid separator calibration/timing");
    }
    this.indexTicks = indexTicks; this.tolerance = tolerance;
    this.rpmTolerance = rpmTolerance; this.minRpm = minRpm; this.rpmStable = rpmStable;
    this.interval = interval; this.settle = settle; this.timeout = timeout;
  }

  public void update(boolean shooterOn, double requestedRpm, double actualRpm, int position, double now) {
    if (state == State.FAULT) { return; }
    if (!Double.isFinite(now)) { fault("Invalid timer"); return; }
    if (!shooterOn) { stop(); return; }
    if (!initialized) { target = position; initialized = true; }
    if (state == State.OFF) { state = State.WAIT_FOR_RPM; }
    boolean changed = Double.compare(requestedRpm, selectedRpm) != 0;
    if (changed) { selectedRpm = requestedRpm; stableSince = Double.NaN; }
    boolean rpmReady = Double.isFinite(requestedRpm) && Double.isFinite(actualRpm)
        && Math.abs(requestedRpm) >= minRpm && actualRpm * requestedRpm > 0
        && Math.abs(actualRpm - requestedRpm) <= rpmTolerance;
    if (!rpmReady) { stableSince = Double.NaN; }
    else if (!Double.isFinite(stableSince)) { stableSince = now; }
    boolean stable = rpmReady && now - stableSince >= rpmStable;

    if (state == State.INDEXING) {
      if (changed || !rpmReady) { fault("Shooter speed/direction changed during index"); return; }
      if (Math.abs((long) position - target) <= tolerance) {
        if (!Double.isFinite(positionSince)) { positionSince = now; }
        if (now - positionSince >= settle) {
          completed++;
          state = State.RECOVERING;
          recoveredAt = now;
          stableSince = Double.NaN; // Require fresh RPM recovery after every completed cycle.
          status = "BLOCK: minimum pause + shooter recovery";
          return;
        }
      } else { positionSince = Double.NaN; }
      if (now - indexStarted >= timeout) { fault("Gate index timeout/stall; check encoder/jam"); }
      return;
    }

    // Outside an index the mechanism must already be at its expected blocking position.
    if (Math.abs((long) position - target) > tolerance) {
      fault("Gate moved away from blocking position"); return;
    }
    if (completed > 0 && now - recoveredAt < interval) {
      status = "BLOCK: minimum ball interval"; return;
    }
    if (!stable) { status = "BLOCK: waiting for stable signed shooter RPM"; return; }
    try { target = Math.addExact(target, indexTicks); }
    catch (ArithmeticException overflow) { fault("Encoder target overflow"); return; }
    state = State.INDEXING;
    indexStarted = now;
    positionSince = Double.NaN;
    status = "FEED ONE: indexing to next blocking position";
  }

  public void stop() {
    if (state == State.INDEXING) { fault("Index interrupted; manually re-home before restarting OpMode"); return; }
    if (state != State.FAULT) {
      state = State.OFF;
      stableSince = Double.NaN;
      status = "OFF: gate stopped";
    }
  }
  private void fault(String reason) { state = State.FAULT; status = "FAULT: " + reason; }
  public State getState() { return state; }
  public String getStatus() { return status; }
  public int getTargetTicks() { return target; }
  public int getCompletedIndexes() { return completed; }
  public boolean shouldHoldPosition() { return initialized && state != State.OFF && state != State.FAULT; }
}
