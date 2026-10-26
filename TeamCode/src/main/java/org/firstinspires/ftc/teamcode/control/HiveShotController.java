package org.firstinspires.ftc.teamcode.control;

/** Nonblocking pulse batches. Batch completion and observed tipping are separate outcomes. */
public final class HiveShotController {
  public enum State { WAIT_HIGH, SPIN_UP, FEED, WAIT_TIP, COMPLETE, FAILED }
  private State state = State.WAIT_HIGH;
  private final double started, pulseSeconds;
  private double since, rpmReadySince = -1;
  private int pulses;
  private int completedPulses;
  private final int pulseLimit;
  private final boolean requireTip;
  private boolean tipConfirmed;
  public HiveShotController(double now, double pulseSeconds) {
    this(now, pulseSeconds, 2, true);
  }

  public HiveShotController(double now, double pulseSeconds, int pulseLimit, boolean requireTip) {
    if (!Double.isFinite(now) || !Double.isFinite(pulseSeconds)
        || pulseSeconds <= 0 || pulseSeconds > 0.5 || pulseLimit < 1 || pulseLimit > 4) {
      throw new IllegalArgumentException("Finite time and feed pulse in (0, 0.5] required");
    }
    started = since = now;
    this.pulseSeconds = pulseSeconds;
    this.pulseLimit = pulseLimit;
    this.requireTip = requireTip;
  }

  public void update(double now, boolean connected, boolean highConfirmed,
      boolean lowCandidate, boolean tipped, boolean rpmsReady, boolean poseReady) {
    if (isDone()) { return; }
    if (!Double.isFinite(now) || now < since || !connected || !poseReady || now - started >= 6) {
      transition(State.FAILED, now); return;
    }
    if (tipped) { tipConfirmed = true; transition(State.COMPLETE, now); return; }
    // Stop immediately on low evidence, before waiting for stable confirmation.
    if (lowCandidate && state == State.FEED) { transition(State.WAIT_TIP, now); }
    switch (state) {
      case WAIT_HIGH:
        if (highConfirmed) { transition(State.SPIN_UP, now); }
        break;
      case SPIN_UP:
        if (!highConfirmed) { transition(State.WAIT_HIGH, now); }
        else if (!rpmsReady) { rpmReadySince = -1; }
        else if (rpmReadySince < 0) { rpmReadySince = now; }
        else if (now - rpmReadySince >= 0.25 && !lowCandidate) {
          pulses++; transition(State.FEED, now);
        }
        break;
      case FEED:
        if (!highConfirmed || !rpmsReady) { transition(State.FAILED, now); }
        else if (now - since >= pulseSeconds) {
          completedPulses++;
          transition(State.WAIT_TIP, now);
        }
        break;
      case WAIT_TIP:
        if (now - since >= 0.7) {
          if (completedPulses == pulseLimit && !requireTip) { transition(State.COMPLETE, now); }
          else if (pulses < pulseLimit && !lowCandidate && highConfirmed) {
            transition(State.SPIN_UP, now);
          }
        }
        break;
      default: break;
    }
  }

  private void transition(State next, double now) { state = next; since = now; rpmReadySince = -1; }
  public State getState() { return state; }
  public int getPulses() { return pulses; }
  public int getCompletedPulses() { return completedPulses; }
  public boolean isTipConfirmed() { return tipConfirmed; }
  public boolean isDone() { return state == State.COMPLETE || state == State.FAILED; }
  public boolean shooterOn() { return state == State.SPIN_UP || state == State.FEED; }
  public boolean feederOn() { return state == State.FEED; }
}
