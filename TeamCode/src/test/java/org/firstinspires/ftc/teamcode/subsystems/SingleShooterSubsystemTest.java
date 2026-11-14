package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.Test;

public final class SingleShooterSubsystemTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void initializesOneEncoderMotorStoppedAtSixThousandRpmTarget() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
    assertEquals(0, motor.commandedVelocity, EPSILON);
    assertEquals(6000, shooter.getTargetRpm(), EPSILON);
    assertFalse(shooter.isActive());
  }

  @Test
  public void activeAndReleasedCommandsRunThenStopTheSingleMotor() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setActive(true);
    assertTrue(shooter.isActive());
    assertEquals(36000, motor.commandedVelocity, EPSILON);
    shooter.setActive(false);
    assertFalse(shooter.isActive());
    assertEquals(0, motor.commandedVelocity, EPSILON);
  }

  @Test
  public void convertsEncoderDegreesPerSecondToActualRpm() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    motor.measuredVelocity = 35400;
    assertEquals(5900, shooter.getActualRpm(), EPSILON);
  }

  @Test
  public void stopIsSafeBeforeStartingAndAfterRunning() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.stop();
    shooter.setActive(true);
    shooter.stop();
    shooter.stop();
    assertFalse(shooter.isActive());
    assertEquals(0, motor.commandedVelocity, EPSILON);
  }

  @Test
  public void reverseUsesNegativeRpmAndForwardRestoresPositiveRpm() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(true, true);
    assertTrue(shooter.isReversed());
    assertEquals(-6000, shooter.getCommandedRpm(), EPSILON);
    assertEquals(-36000, motor.commandedVelocity, EPSILON);
    motor.measuredVelocity = -35400;
    assertEquals(-5900, shooter.getActualRpm(), EPSILON);
    shooter.setState(true, false);
    assertFalse(shooter.isReversed());
    assertEquals(36000, motor.commandedVelocity, EPSILON);
  }

  @Test
  public void directionCanBeSelectedWhileOffAndPersistsAcrossStop() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(false, true);
    assertFalse(shooter.isActive());
    assertTrue(shooter.isReversed());
    assertEquals(0, motor.commandedVelocity, EPSILON);
    shooter.setActive(true);
    assertEquals(-36000, motor.commandedVelocity, EPSILON);
    shooter.stop();
    assertEquals(0, motor.commandedVelocity, EPSILON);
    assertEquals(0, shooter.getCommandedRpm(), EPSILON);
    assertTrue(shooter.isReversed());
  }

  @Test public void adjustableRpmAppliesLiveInEitherDirectionAndPersistsWhileOff() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(true, false, 5800);
    assertEquals(34800, motor.commandedVelocity, EPSILON);
    shooter.setState(true, true, 5700);
    assertEquals(-34200, motor.commandedVelocity, EPSILON);
    shooter.setState(false, true, 5600);
    assertEquals(0, motor.commandedVelocity, EPSILON);
    assertEquals(5600, shooter.getTargetRpm(), EPSILON);
    shooter.setActive(true);
    assertEquals(-33600, motor.commandedVelocity, EPSILON);
    shooter.stop();
    assertEquals(0, motor.commandedVelocity, EPSILON);
    assertEquals(5600, shooter.getTargetRpm(), EPSILON);
  }

  @Test public void requestedRpmIsBoundedToMotorRating() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(true, false, 7000);
    assertEquals(6000, shooter.getTargetRpm(), EPSILON);
    assertEquals(36000, motor.commandedVelocity, EPSILON);
    shooter.setState(true, true, -100);
    assertEquals(0, shooter.getTargetRpm(), EPSILON);
    assertEquals(0, motor.commandedVelocity, EPSILON);
  }

  @Test public void nonFiniteRpmIsRejectedBeforeChangingMotorState() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(true, false, 5000);
    for (double rpm : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      try {
        shooter.setState(false, true, rpm);
        org.junit.Assert.fail("Expected non-finite RPM rejection");
      } catch (IllegalArgumentException expected) {
        assertEquals(30000, motor.commandedVelocity, EPSILON);
        assertEquals(5000, shooter.getTargetRpm(), EPSILON);
        assertTrue(shooter.isActive());
        assertFalse(shooter.isReversed());
      }
    }
  }

  /** Fails on unexpected hardware calls; no real Hub or second motor is needed. */
  private static final class FakeMotor {
    DcMotor.RunMode mode;
    double commandedVelocity;
    double measuredVelocity;
    final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
        DcMotorEx.class.getClassLoader(), new Class<?>[] {DcMotorEx.class},
        (proxy, method, args) -> {
          switch (method.getName()) {
            case "setMode":
              mode = (DcMotor.RunMode) args[0];
              return null;
            case "setVelocity":
              assertEquals(AngleUnit.DEGREES, args[1]);
              commandedVelocity = (double) args[0];
              return null;
            case "getVelocity":
              assertEquals(AngleUnit.DEGREES, args[0]);
              return measuredVelocity;
            default:
              throw new AssertionError("Unexpected hardware call: " + method.getName());
          }
        });
  }
}
