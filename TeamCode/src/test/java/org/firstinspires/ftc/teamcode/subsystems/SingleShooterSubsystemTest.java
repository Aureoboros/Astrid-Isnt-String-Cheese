package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

public final class SingleShooterSubsystemTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void initializesOneEncoderMotorStoppedAtSixThousandRpmTarget() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
    assertEquals(28, motor.motorType.getTicksPerRev(), EPSILON);
    assertEquals(6000, motor.motorType.getMaxRPM(), EPSILON);
    assertEquals(1, motor.motorType.getGearing(), EPSILON);
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

  @Test public void teleTourneyDefaultCommandsReverse4800AndStaysOffUntilEnabled() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(false, RobotConfig.TeleTourney.SHOOTER_REVERSED,
        RobotConfig.TeleTourney.SHOOTER_INITIAL_RPM);
    assertEquals(4800, shooter.getTargetRpm(), EPSILON);
    assertTrue(shooter.isReversed());
    assertEquals(0, motor.commandedVelocity, EPSILON);
    shooter.setActive(true);
    assertEquals(-4800, shooter.getCommandedRpm(), EPSILON);
    assertEquals(-28800, motor.commandedVelocity, EPSILON);
    shooter.stop();
    shooter.setActive(true);
    assertEquals(-28800, motor.commandedVelocity, EPSILON);
  }

  @Test public void rpmSetterKeepsStoppedMotorOffAndRetainsReverseWhileRunning() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(false, true);
    shooter.setTargetRpm(4800);
    assertFalse(shooter.isActive());
    assertTrue(shooter.isReversed());
    assertEquals(4800, shooter.getTargetRpm(), EPSILON);
    assertEquals(0, motor.commandedVelocity, EPSILON);
    shooter.setActive(true);
    shooter.setTargetRpm(4900);
    assertTrue(shooter.isActive());
    assertTrue(shooter.isReversed());
    assertEquals(-29400, motor.commandedVelocity, EPSILON);
    shooter.setTargetRpm(7000);
    assertEquals(-36000, motor.commandedVelocity, EPSILON);
    shooter.setTargetRpm(-100);
    assertEquals(0, motor.commandedVelocity, EPSILON);
    assertEquals(0, shooter.getTargetRpm(), EPSILON);
    shooter.stop();
  }

  @Test public void rpmSetterRejectsNonFiniteValuesWithoutChangingState() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(true, true, 4800);
    for (double rpm : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      try {
        shooter.setTargetRpm(rpm);
        org.junit.Assert.fail("Expected non-finite RPM rejection");
      } catch (IllegalArgumentException expected) {
        assertEquals(4800, shooter.getTargetRpm(), EPSILON);
        assertEquals(-28800, motor.commandedVelocity, EPSILON);
        assertTrue(shooter.isActive());
        assertTrue(shooter.isReversed());
      }
    }
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
  @Test public void correctedProfilePreventsVelocityClippingAndRpmStepsReachTheHub() {
    FakeMotor motor = new FakeMotor();
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(motor.device);
    shooter.setState(true, true, 4800);
    assertEquals(-2240, motor.commandedTicksPerSecond, EPSILON);
    shooter.setTargetRpm(4900);
    assertEquals(-2287, motor.commandedTicksPerSecond, EPSILON);
    shooter.setTargetRpm(4700);
    assertEquals(-2193, motor.commandedTicksPerSecond, EPSILON);
    shooter.setTargetRpm(6000);
    assertEquals(-2800, motor.commandedTicksPerSecond, EPSILON);
    motor.measuredVelocity = -28800;
    assertEquals(-4800, shooter.getActualRpm(), EPSILON);
    assertEquals(-2240, shooter.getRawEncoderTicksPerSecond(), EPSILON);
    assertEquals(28, shooter.getEncoderTicksPerRevolution(), EPSILON);
  }

  /** Simulates SDK degree/tick conversion and the hub's 32767 ticks/s velocity limit. */
  private static final class FakeMotor {
    DcMotor.RunMode mode;
    double commandedVelocity;
    double commandedTicksPerSecond;
    double measuredVelocity;
    MotorConfigurationType motorType = new MotorConfigurationType();
    {
      // Start with the incorrect generic SDK series profile to exercise the runtime correction.
      motorType.setTicksPerRev(2786);
      motorType.setGearing(99.5);
      motorType.setMaxRPM(60);
    }
    final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
        DcMotorEx.class.getClassLoader(), new Class<?>[] {DcMotorEx.class},
        (proxy, method, args) -> {
          switch (method.getName()) {
            case "getMotorType":
              return motorType;
            case "setMotorType":
              motorType = (MotorConfigurationType) args[0];
              return null;
            case "setMode":
              mode = (DcMotor.RunMode) args[0];
              return null;
            case "setVelocity":
              assertEquals(AngleUnit.DEGREES, args[1]);
              commandedVelocity = (double) args[0];
              commandedTicksPerSecond = Math.max(-32767, Math.min(32767,
                  Math.round(commandedVelocity / 360 * motorType.getTicksPerRev())));
              return null;
            case "getVelocity":
              if (args == null || args.length == 0) {
                return measuredVelocity / 360 * motorType.getTicksPerRev();
              }
              assertEquals(AngleUnit.DEGREES, args[0]);
              return measuredVelocity;
            default:
              throw new AssertionError("Unexpected hardware call: " + method.getName());
          }
        });
  }
}
