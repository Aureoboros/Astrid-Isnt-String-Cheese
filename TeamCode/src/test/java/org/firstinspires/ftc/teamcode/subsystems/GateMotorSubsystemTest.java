package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

public final class GateMotorSubsystemTest {
  private final List<Double> velocities = new ArrayList<>();
  private MotorConfigurationType profile = new MotorConfigurationType();
  private DcMotor.Direction direction;
  private DcMotor.RunMode mode;
  private DcMotor.ZeroPowerBehavior zeroPower;
  private double measuredVelocity;

  private DcMotorEx motor() {
    return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
        new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
          switch (method.getName()) {
            case "getMotorType": return profile;
            case "setMotorType": profile = (MotorConfigurationType) args[0]; break;
            case "setDirection": direction = (DcMotor.Direction) args[0]; break;
            case "setMode": mode = (DcMotor.RunMode) args[0]; break;
            case "setZeroPowerBehavior": zeroPower = (DcMotor.ZeroPowerBehavior) args[0]; break;
            case "setVelocity": velocities.add((double) args[0]); break;
            case "getVelocity": return measuredVelocity;
            default: throw new AssertionError("Unexpected hardware access: " + method.getName());
          }
          return null;
        });
  }

  @Test public void initConfigures435RpmEncoderProfileAndNeverRunsMotor() {
    MotorConfigurationType original = profile;
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    assertEquals(Arrays.asList(0.0), velocities);
    assertNotSame(original, profile);
    assertEquals(435, profile.getMaxRPM(), 0);
    assertEquals(384.5, profile.getTicksPerRev(), 0);
    assertEquals(RobotConfig.GateMotor.MOTOR_DIRECTION, direction);
    assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, mode);
    assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, zeroPower);
    assertFalse(gate.isActive()); assertEquals(0, gate.getCommandedRpm(), 0);
  }

  @Test public void directionWhileOffStaysOffAndStartStopUsesSignedEncoderVelocity() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    gate.setState(false, true);
    gate.setState(true, true);
    gate.setState(true, false);
    measuredVelocity = 100 * 384.5 / 60;
    assertEquals(100, gate.getActualRpm(), 1e-9);
    gate.setState(false, true);
    gate.stop();
    double speed = 100 * 384.5 / 60;
    assertEquals(Arrays.asList(0.0, 0.0, -speed, speed, 0.0, 0.0), velocities);
    assertFalse(gate.isActive());
  }

  @Test public void targetRpmClampsToMotorRating() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    gate.setState(true, false, 6000);
    assertEquals(435, gate.getTargetRpm(), 0);
    assertEquals(435 * 384.5 / 60, velocities.get(velocities.size() - 1), 1e-9);
    gate.setState(true, false, -25);
    assertEquals(0, gate.getCommandedRpm(), 0);
  }

  @Test public void invalidCalibrationRejectedBeforeCommands() {
    for (double invalid : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
      try { new GateMotorSubsystem(motor(), invalid, 384.5, 100); fail("Invalid RPM accepted"); }
      catch (IllegalArgumentException expected) { assertTrue(velocities.isEmpty()); }
      try { new GateMotorSubsystem(motor(), 435, invalid, 100); fail("Invalid encoder resolution accepted"); }
      catch (IllegalArgumentException expected) { assertTrue(velocities.isEmpty()); }
    }
  }
}
