package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.Test;

public final class GateMotorSubsystemTest {
  private static final double TICKS = 384.5;
  private final List<Double> velocities = new ArrayList<>();
  private final List<String> setup = new ArrayList<>();
  private double measuredTicksPerSecond;
  private MotorConfigurationType motorType = new MotorConfigurationType();
  private final MotorConfigurationType original = motorType;

  private DcMotorEx motor() {
    return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
        new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
          String name = method.getName();
          if (name.equals("setVelocity") && args.length == 1) { velocities.add((double) args[0]); }
          else if (name.equals("getMotorType")) { return motorType; }
          else if (name.equals("setMotorType")) { motorType = (MotorConfigurationType) args[0]; }
          else if (name.equals("getVelocity") && (args == null || args.length == 0)) { return measuredTicksPerSecond; }
          else if (name.equals("setDirection") || name.equals("setMode")
              || name.equals("setZeroPowerBehavior")) { setup.add(args[0].toString()); }
          else { throw new AssertionError("Unexpected hardware access: " + name); }
          return null;
        });
  }

  private static double ticksPerSecond(double rpm) { return rpm * TICKS / 60.0; }

  @Test public void initConfiguresEncoderVelocityWithBrakeAndStops() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, TICKS, 200);
    assertEquals(Arrays.asList("REVERSE", "BRAKE", "RUN_USING_ENCODER"), setup);
    assertEquals(Arrays.asList(0.0), velocities);
    assertFalse(gate.isActive()); assertEquals(0, gate.getCommandedRpm(), 0);
    assertEquals(200, gate.getTargetRpm(), 0);
    assertNotSame(original, motorType);
    assertEquals(TICKS, motorType.getTicksPerRev(), 1e-9);
    assertEquals(435, motorType.getMaxRPM(), 1e-9);
  }

  @Test public void directionWhileOffStaysOffAndStartStopUsesSignedVelocity() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, TICKS, 200);
    gate.setState(false, true);
    gate.setState(true, true);
    gate.setState(true, false);
    gate.stop();
    assertEquals(Arrays.asList(0.0, 0.0, -ticksPerSecond(200), ticksPerSecond(200), 0.0), velocities);
    assertFalse(gate.isActive());
  }

  @Test public void rpmTargetClampsToRating() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, TICKS, 200);
    gate.setState(true, false, 1000);
    assertEquals(435, gate.getTargetRpm(), 0);
    assertEquals(ticksPerSecond(435), velocities.get(velocities.size() - 1), 1e-9);
    gate.setState(true, false, -50);
    assertEquals(0, gate.getTargetRpm(), 0);
  }

  @Test public void actualRpmIsMeasuredFromEncoder() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, TICKS, 200);
    measuredTicksPerSecond = ticksPerSecond(180);
    assertEquals(180, gate.getActualRpm(), 1e-9);
  }

  @Test public void invalidSettingsRejectedBeforeTouchingHardware() {
    double[][] bad = {{0, TICKS, 200}, {435, 0, 200}, {Double.NaN, TICKS, 200},
        {435, Double.POSITIVE_INFINITY, 200}, {435, TICKS, Double.NaN}};
    for (double[] b : bad) {
      try { new GateMotorSubsystem(motor(), b[0], b[1], b[2]); fail("Invalid settings accepted"); }
      catch (IllegalArgumentException expected) { assertTrue(velocities.isEmpty()); assertTrue(setup.isEmpty()); assertSame(original, motorType); }
    }
  }
}
