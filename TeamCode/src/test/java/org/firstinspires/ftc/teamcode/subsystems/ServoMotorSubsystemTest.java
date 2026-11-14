package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.junit.Test;

public final class ServoMotorSubsystemTest {
  @Test public void uncalibratedServoNeverReceivesMovementEvenWhenControlsAreUsed() {
    Servo device = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
        new Class<?>[] {Servo.class}, (proxy, method, args) -> {
          throw new AssertionError("Uncalibrated hardware command: " + method.getName());
        });
    ServoMotorSubsystem servo = new ServoMotorSubsystem(device, false, 0.4, 0.5, 0.6);
    servo.toggleEnabled(); servo.nudge(1); servo.center(); servo.stop();
    assertFalse(servo.isEnabled());
  }

  @Test public void calibratedManualCommandsClampToSafeRangeAndStopDoesNotMove() {
    java.util.List<Double> commands = new java.util.ArrayList<>();
    Servo device = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
        new Class<?>[] {Servo.class}, (proxy, method, args) -> {
          if (!"setPosition".equals(method.getName())) { throw new AssertionError(method.getName()); }
          commands.add((double) args[0]);
          return null;
        });
    assertTrue(CameraPanConfig.travelReady());
    assertEquals(0.05, CameraPanConfig.MIN_POSITION, 0.0);
    assertEquals(0.2, CameraPanConfig.CENTER_POSITION, 0.0);
    assertEquals(0.35, CameraPanConfig.MAX_POSITION, 0.0);
    assertEquals(0.2, CameraPanConfig.START_POSITION, 0.0);
    ServoMotorSubsystem servo = new ServoMotorSubsystem(device, CameraPanConfig.travelReady(),
        CameraPanConfig.MIN_POSITION, CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION,
        CameraPanConfig.START_POSITION);
    assertTrue(commands.isEmpty());
    assertTrue(Double.isNaN(servo.getPosition()));
    servo.toggleEnabled();
    servo.nudge(10); servo.nudge(-10); servo.center(); servo.stop(); servo.nudge(10);
    assertEquals(java.util.Arrays.asList(0.2, 0.35, 0.05, 0.2), commands);
    assertFalse(servo.isEnabled());
    servo.toggleEnabled();
    assertEquals(0.2, servo.getPosition(), 0);
    assertEquals(java.util.Arrays.asList(0.2, 0.35, 0.05, 0.2, 0.2), commands);
  }

  @Test public void invalidStartPositionBlocksAllServoMovement() {
    Servo device = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
        new Class<?>[] {Servo.class}, (proxy, method, args) -> {
          throw new AssertionError("Invalid-start hardware command: " + method.getName());
        });
    for (double start : new double[] {0.6, 1, Double.NaN, Double.POSITIVE_INFINITY}) {
      ServoMotorSubsystem servo = new ServoMotorSubsystem(device, true, 0.7, 0.8, 0.9, start);
      servo.toggleEnabled(); servo.nudge(1); servo.center(); servo.stop();
      assertFalse(servo.isEnabled());
      assertTrue(Double.isNaN(servo.getPosition()));
    }
  }
}
