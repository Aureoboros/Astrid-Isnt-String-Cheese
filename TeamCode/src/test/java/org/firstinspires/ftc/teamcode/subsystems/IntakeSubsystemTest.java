package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

public final class IntakeSubsystemTest {
  @Test public void combinedStateCommandsForwardStopReverseStopWithNoTransientPulse() {
    List<Double> commands = new ArrayList<>();
    DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
        new Class<?>[] {DcMotor.class}, (proxy, method, args) -> {
          if (!"setPower".equals(method.getName())) { throw new AssertionError(method.getName()); }
          commands.add((double) args[0]);
          return null;
        });
    IntakeSubsystem intake = new IntakeSubsystem(motor);
    intake.setState(true, IntakeSubsystem.Direction.FORWARD);
    intake.setState(false, IntakeSubsystem.Direction.REVERSE);
    intake.setState(true, IntakeSubsystem.Direction.REVERSE);
    assertTrue(intake.isEnabled());
    assertEquals(IntakeSubsystem.Direction.REVERSE, intake.getDirection());
    intake.setState(false, IntakeSubsystem.Direction.FORWARD);
    intake.setState(true, IntakeSubsystem.Direction.FORWARD);
    intake.stop();
    double power = RobotConfig.Mechanisms.INTAKE_POWER;
    assertEquals(Arrays.asList(0.0, power, 0.0, -power, 0.0, power, 0.0), commands);
    assertFalse(intake.isEnabled());
  }

  @Test public void legacyTogglesStillWorkAndStopPreservesDirection() {
    List<Double> commands = new ArrayList<>();
    DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
        new Class<?>[] {DcMotor.class}, (proxy, method, args) -> {
          if (!"setPower".equals(method.getName())) { throw new AssertionError(method.getName()); }
          commands.add((double) args[0]);
          return null;
        });
    IntakeSubsystem intake = new IntakeSubsystem(motor);
    intake.toggleDirection();
    intake.toggleEnabled();
    assertTrue(intake.isEnabled());
    assertEquals(-RobotConfig.Mechanisms.INTAKE_POWER, commands.get(commands.size() - 1), 0);
    intake.stop();
    intake.setEnabled(true);
    assertEquals(IntakeSubsystem.Direction.REVERSE, intake.getDirection());
    assertEquals(-RobotConfig.Mechanisms.INTAKE_POWER, commands.get(commands.size() - 1), 0);
  }
}
