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
import org.firstinspires.ftc.teamcode.control.GateJamRecoveryController;
import org.junit.Test;

public final class GateMotorSubsystemTest {
  private final List<Double> velocities = new ArrayList<>();
  private MotorConfigurationType profile = new MotorConfigurationType();
  private DcMotor.Direction direction;
  private DcMotor.RunMode mode;
  private DcMotor.ZeroPowerBehavior zeroPower;
  private double measuredVelocity;
  private int position;
  private int targetPosition, targetWrites, tolerance;
  private double power;

  private DcMotorEx motor() {
    return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
        new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
          switch (method.getName()) {
            case "getMotorType": return profile;
            case "setMotorType": profile = (MotorConfigurationType) args[0]; break;
            case "setDirection": direction = (DcMotor.Direction) args[0]; break;
            case "setMode": mode = (DcMotor.RunMode) args[0]; break;
            case "setTargetPosition": targetPosition = (int) args[0]; targetWrites++; break;
            case "setTargetPositionTolerance": tolerance = (int) args[0]; break;
            case "setPower": power = (double) args[0]; break;
            case "setZeroPowerBehavior": zeroPower = (DcMotor.ZeroPowerBehavior) args[0]; break;
            case "setVelocity": velocities.add((double) args[0]); break;
            case "getVelocity": return measuredVelocity;
            case "getCurrentPosition": return position;
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
    assertFalse(gate.isHolding()); assertEquals(0, power, 0);
    assertEquals(RobotConfig.GateMotor.HOLD_POSITION_TOLERANCE_TICKS, tolerance);
  }

  @Test public void directionWhileHoldingStaysHeldAndFeedingUsesSignedEncoderVelocity() {
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

  @Test public void holdCapturesPositionOnceAndKeepsTargetWhenNectarPushesIt() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    position = 123;
    gate.setState(false, false, 100, 0);
    assertTrue(gate.isHolding()); assertFalse(gate.isActive());
    assertEquals(DcMotor.RunMode.RUN_TO_POSITION, mode);
    assertEquals(123, targetPosition);
    assertEquals(RobotConfig.GateMotor.HOLD_POWER, power, 0);
    position = 117; // External load moves shaft; the hub retains a restoring position target.
    gate.setState(false, true, 200, 0.5);
    assertEquals(123, targetPosition); assertEquals(1, targetWrites);
    assertEquals(123, gate.getHoldPositionTicks()); assertEquals(6, gate.getHoldErrorTicks());
    assertEquals(GateJamRecoveryController.State.OFF, gate.getRecoveryState());
    assertEquals(0, gate.getOutputRpm(), 0);
  }

  @Test public void restartReturnsToVelocityAndNextHoldCapturesNewPosition() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    position = 10; gate.setState(false, false, 100, 0);
    gate.setState(true, true, 100, 0.1);
    assertFalse(gate.isHolding()); assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, mode);
    assertEquals(-100 * 384.5 / 60, velocities.get(velocities.size() - 1), 1e-9);
    position = 55; gate.setState(false, false, 100, 0.2);
    assertTrue(gate.isHolding()); assertEquals(55, targetPosition); assertEquals(2, targetWrites);
    gate.stop();
    assertFalse(gate.isHolding()); assertFalse(gate.isActive());
    assertEquals(0, power, 0); assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, mode);
    assertEquals(0, velocities.get(velocities.size() - 1), 0);
  }

  @Test public void zeroRpmHoldsAndPositiveRpmResumesFeeding() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    position = 87; gate.setState(true, false, 0, 0);
    assertTrue(gate.isHolding()); assertEquals(87, targetPosition);
    gate.setState(true, false, 60, 0.1);
    assertFalse(gate.isHolding()); assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, mode);
    assertEquals(60, gate.getOutputRpm(), 0);
  }

  @Test public void holdCancelsReverseRecoveryAndRestartPreservesJamDetection() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    gate.setState(true, false, 100, 0);
    gate.setState(true, false, 100, .91);
    gate.setState(true, false, 100, 1.02);
    assertEquals(GateJamRecoveryController.State.REVERSING, gate.getRecoveryState());
    gate.setState(false, false, 100, 1.03);
    assertTrue(gate.isHolding()); assertEquals(0, gate.getRecoveryAttempts());
    gate.setState(true, false, 100, 1.1);
    gate.setState(true, false, 100, 2.02);
    assertFalse(gate.isHolding());
    assertEquals(GateJamRecoveryController.State.STOPPING, gate.getRecoveryState());
    // Recovery's temporary zero velocity is still a pause, not a position hold.
    assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, mode);
    assertEquals(1, gate.getRecoveryAttempts());
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

  @Test public void jamRecoveryAppliesPauseReverseAndRestartWithoutChangingSelectedDirection() {
    GateMotorSubsystem gate = new GateMotorSubsystem(motor(), 435, 384.5, 100);
    gate.setState(true, false, 100, 0);
    gate.setState(true, false, 100, .91);
    assertEquals(0, gate.getOutputRpm(), 0);
    gate.setState(true, false, 100, 1.02);
    assertEquals(-60, gate.getOutputRpm(), 0);
    assertEquals(-60 * 384.5 / 60, velocities.get(velocities.size() - 1), 1e-9);
    assertFalse(gate.isReversed());
    assertEquals(100, gate.getCommandedRpm(), 0);
    gate.setState(true, false, 100, 1.18);
    assertEquals(0, gate.getOutputRpm(), 0);
    gate.setState(true, false, 100, 1.29);
    assertEquals(100, gate.getOutputRpm(), 0);
    measuredVelocity = 100 * 384.5 / 60;
    position = 600;
    gate.setState(true, false, 100, 2.3);
    assertEquals(100, gate.getOutputRpm(), 0);
    assertEquals(1, gate.getRecoveryCount());
    gate.stop();
    assertEquals(0, gate.getOutputRpm(), 0);
  }
}
