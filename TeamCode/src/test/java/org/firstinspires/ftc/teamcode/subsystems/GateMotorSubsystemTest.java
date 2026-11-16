package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.Test;

public final class GateMotorSubsystemTest {
  private final List<Double> powers = new ArrayList<>();
  private final List<String> setup = new ArrayList<>();
  private DcMotor motor() {
    return (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
        new Class<?>[] {DcMotor.class}, (proxy, method, args) -> {
          if (method.getName().equals("setPower")) { powers.add((double) args[0]); }
          else if (method.getName().equals("setDirection") || method.getName().equals("setMode")
              || method.getName().equals("setZeroPowerBehavior")) { setup.add(args[0].toString()); }
          else { throw new AssertionError("Unexpected hardware access: " + method.getName()); }
          return null;
        });
  }

  @Test public void initOnlyStopsAndConfiguresOpenLoopWithBrake() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 0.2);
    assertEquals(Arrays.asList(0.0), powers);
    assertEquals(Arrays.asList("FORWARD", "RUN_WITHOUT_ENCODER", "BRAKE"), setup);
    assertFalse(gate.isActive()); assertEquals(0, gate.getCommandedPower(), 0);
  }

  @Test public void directionWhileOffStaysOffAndStartStopUsesSignedPower() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 0.2);
    gate.setState(false, true);
    gate.setState(true, true);
    gate.setState(true, false);
    gate.setState(false, true);
    gate.stop();
    assertEquals(Arrays.asList(0.0, 0.0, -0.2, 0.2, 0.0, 0.0), powers);
    assertFalse(gate.isActive());
  }

  @Test public void invalidPowerRejectedBeforeTouchingHardware() {
    for (double power : new double[] {0, -0.2, 1.1, Double.NaN, Double.POSITIVE_INFINITY}) {
      try { new GateMotorSubsystem(motor(), power); fail("Invalid power accepted"); }
      catch (IllegalArgumentException expected) { assertTrue(powers.isEmpty()); assertTrue(setup.isEmpty()); }
    }
  }
}
