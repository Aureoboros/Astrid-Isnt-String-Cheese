package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

public final class IntakeSubsystemTest {
  private static final class HoldMotor {
    int position, target, targetWrites, tolerance;
    double power, velocity, measuredVelocity;
    MotorConfigurationType profile = new MotorConfigurationType();
    double velocityF;
    DcMotor.RunMode mode;
    DcMotor.ZeroPowerBehavior zeroPower;
    final List<String> calls = new ArrayList<>();
    DcMotorEx create() {
      return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
          new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
            switch (method.getName()) {
              case "getMotorType": return profile;
              case "setMotorType": profile = (MotorConfigurationType) args[0]; break;
              case "setVelocityPIDFCoefficients": velocityF = (double) args[3]; break;
              case "setPower": power = (double) args[0]; calls.add("power:" + power); break;
              case "setVelocity": assertEquals(AngleUnit.DEGREES, args[1]); velocity = (double) args[0]; calls.add("velocity:" + velocity); break;
              case "getVelocity": assertEquals(AngleUnit.DEGREES, args[0]); return measuredVelocity;
              case "setMode": mode = (DcMotor.RunMode) args[0]; calls.add("mode:" + mode); break;
              case "setZeroPowerBehavior": zeroPower = (DcMotor.ZeroPowerBehavior) args[0]; break;
              case "setTargetPositionTolerance": tolerance = (int) args[0]; break;
              case "getCurrentPosition": return position;
              case "setTargetPosition": target = (int) args[0]; targetWrites++; calls.add("target:" + target); break;
              default: throw new AssertionError(method.getName());
            }
            return null;
          });
    }
  }

  @Test public void holdInitConfiguresEncoderModeButNeverAppliesTorque() {
    HoldMotor m = new HoldMotor(); IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    assertEquals(0, m.power, 0); assertFalse(intake.isHolding());
    assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, m.mode);
    assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, m.zeroPower);
    assertEquals(RobotConfig.Mechanisms.INTAKE_HOLD_TOLERANCE_TICKS, m.tolerance);
    assertEquals(0, m.targetWrites);
    assertEquals(0, m.velocity, 0);
  }

  @Test public void holdCutsPowerBeforeCapturingAndDoesNotFollowAnExternalPush() {
    HoldMotor m = new HoldMotor(); IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    intake.setEnabled(true); m.position = 123; m.calls.clear(); intake.setEnabled(false);
    assertEquals(Arrays.asList("power:0.0", "velocity:0.0", "target:123", "mode:RUN_TO_POSITION",
        "power:" + RobotConfig.Mechanisms.INTAKE_HOLD_POWER), m.calls);
    assertTrue(intake.isHolding()); assertFalse(intake.isEnabled());
    m.position = 115; intake.setEnabled(false); intake.toggleDirection();
    assertEquals(123, m.target); assertEquals(1, m.targetWrites);
    assertEquals(123, intake.getHoldPositionTicks()); assertEquals(8, intake.getHoldErrorTicks());
    assertEquals(IntakeSubsystem.Direction.REVERSE, intake.getDirection());
  }

  @Test public void restartExitsHoldAndRunsInTheSelectedReverseDirection() {
    HoldMotor m = new HoldMotor(); IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    intake.setState(false, IntakeSubsystem.Direction.REVERSE); m.calls.clear(); intake.toggleEnabled();
    assertEquals(Arrays.asList("power:0.0", "mode:RUN_USING_ENCODER",
        "velocity:" + -RobotConfig.Mechanisms.INTAKE_TARGET_RPM * 6.0), m.calls);
    assertTrue(intake.isEnabled()); assertFalse(intake.isHolding());
    m.position = -50; intake.toggleEnabled(); assertEquals(-50, m.target); assertEquals(2, m.targetWrites);
  }

  @Test public void shutdownReleasesHoldingTorqueAndDoesNotRecaptureTarget() {
    HoldMotor m = new HoldMotor(); IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    intake.setEnabled(false); intake.stop(); intake.stop();
    assertFalse(intake.isHolding()); assertFalse(intake.isEnabled()); assertEquals(0, m.power, 0); assertEquals(0,m.velocity,0);
    assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, m.mode); assertEquals(1, m.targetWrites);
  }

  @Test public void reversingWhileRunningKeepsItRunningWithoutEnteringHold() {
    HoldMotor m = new HoldMotor(); IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    intake.setEnabled(true); intake.toggleDirection();
    assertEquals(-435 * 6.0, m.velocity, 0);
    assertTrue(intake.isEnabled()); assertFalse(intake.isHolding()); assertEquals(0, m.targetWrites);
  }
  @Test public void default435RpmUsesAngularVelocityAndReportsSignedMeasuredRpm() {
    HoldMotor m = new HoldMotor(); IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    intake.setEnabled(true); assertEquals(2610,m.velocity,0); assertEquals(435,intake.getTargetRpm(),0);
    m.measuredVelocity = 1140; assertEquals(190,intake.getActualRpm(),0);
    intake.toggleDirection(); assertEquals(-2610,m.velocity,0); assertEquals(-435,intake.getCommandedRpm(),0);
    m.measuredVelocity = -1140; assertEquals(-190,intake.getActualRpm(),0);
    intake.setEnabled(false); assertEquals(0,intake.getCommandedRpm(),0); assertTrue(intake.isHolding());
    assertEquals(435,intake.getTargetRpm(),0);
  }

  @Test public void clones435RpmIntakeProfileAndInitializesVelocityFeedforward() {
    HoldMotor m = new HoldMotor(); MotorConfigurationType original = m.profile;
    IntakeSubsystem intake = new IntakeSubsystem(m.create(), true);
    assertNotSame(original, m.profile);
    assertEquals(435, m.profile.getMaxRPM(), 0);
    assertEquals(384.5, intake.getEncoderTicksPerRevolution(), 0);
    assertEquals(32767.0 / (435 * 384.5 / 60), m.velocityF, 1e-9);
    assertFalse(intake.isEnabled()); assertEquals(0, m.velocity, 0);
  }

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
