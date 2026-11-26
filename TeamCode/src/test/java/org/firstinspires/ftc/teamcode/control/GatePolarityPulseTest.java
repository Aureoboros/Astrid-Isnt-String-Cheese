package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.junit.Test;

public final class GatePolarityPulseTest {
  @Test public void positivePulseIsBoundedAndHeldButtonCannotRestartIt() {
    GatePolarityPulse pulse = new GatePolarityPulse(0.1, 1);
    assertEquals(0.1, pulse.update(true, false, 0), 0);
    assertEquals(0.1, pulse.update(true, false, 0.99), 0);
    assertEquals(0, pulse.update(true, false, 1), 0);
    assertEquals(0, pulse.update(true, false, 5), 0);
    assertEquals(0, pulse.update(false, false, 6), 0);
    assertEquals(-0.1, pulse.update(false, true, 7), 0);
  }

  @Test public void releaseAndOpposingButtonsStopImmediatelyAndRequireReleaseBeforeRestart() {
    GatePolarityPulse pulse = new GatePolarityPulse(0.1, 1);
    pulse.update(true, false, 0);
    assertEquals(0, pulse.update(false, false, 0.1), 0);
    assertEquals(-0.1, pulse.update(false, true, 0.2), 0);
    assertEquals(0, pulse.update(true, true, 0.3), 0);
    assertEquals(0, pulse.update(true, false, 0.4), 0);
    pulse.update(false, false, 0.5);
    assertEquals(0.1, pulse.update(true, false, 0.6), 0);
  }

  @Test public void directionCannotReverseWithoutBothButtonsBeingReleased() {
    GatePolarityPulse pulse = new GatePolarityPulse(0.1, 1);
    pulse.update(true, false, 0);
    assertEquals(0, pulse.update(false, true, .1), 0);
    assertEquals(0, pulse.update(true, false, .2), 0);
    assertEquals(0, pulse.update(true, false, Double.NaN), 0);
  }
}
