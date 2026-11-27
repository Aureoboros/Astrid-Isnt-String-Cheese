package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Encoder velocity/position holding in tournament/test modes; legacy users retain power control. */
public final class IntakeSubsystem {
  public enum Direction {
    FORWARD(1.0),
    REVERSE(-1.0);

    private final double multiplier;

    Direction(double multiplier) {
      this.multiplier = multiplier;
    }
  }

  private final DcMotor motor;
  private final boolean holdWhenIdle;
  private boolean holding;
  private int holdPositionTicks;
  private volatile Direction direction = Direction.FORWARD;
  private volatile boolean enabled;

  public IntakeSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotor.class, RobotConfig.Hardware.INTAKE));
  }

  /** Hold is enabled by tournament modes and the isolated intake test; requires an encoder. */
  public IntakeSubsystem(HardwareMap hardwareMap, boolean holdWhenIdle) {
    this(hardwareMap.get(DcMotor.class, RobotConfig.Hardware.INTAKE), holdWhenIdle);
  }

  IntakeSubsystem(DcMotor motor) {
    this(motor, false);
  }

  IntakeSubsystem(DcMotor motor, boolean holdWhenIdle) {
    if (holdWhenIdle && (!(motor instanceof DcMotorEx)
        || !Double.isFinite(RobotConfig.Mechanisms.INTAKE_HOLD_POWER)
        || RobotConfig.Mechanisms.INTAKE_HOLD_POWER <= 0
        || RobotConfig.Mechanisms.INTAKE_HOLD_POWER > 1
        || RobotConfig.Mechanisms.INTAKE_HOLD_TOLERANCE_TICKS < 0
        || !Double.isFinite(RobotConfig.Mechanisms.INTAKE_TARGET_RPM)
        || RobotConfig.Mechanisms.INTAKE_TARGET_RPM <= 0)) {
      throw new IllegalArgumentException("Intake hold requires DcMotorEx and valid hold settings");
    }
    this.motor = motor;
    this.holdWhenIdle = holdWhenIdle;
    stop();
    if (holdWhenIdle) {
      // Generic XML metadata describes a different gear ratio. Clone so other motors are untouched.
      MotorConfigurationType profile = motor.getMotorType().clone();
      profile.setTicksPerRev(RobotConfig.IntakeMotor.TICKS_PER_REVOLUTION);
      profile.setMaxRPM(RobotConfig.IntakeMotor.MAX_RPM);
      motor.setMotorType(profile);
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ((DcMotorEx) motor).setTargetPositionTolerance(RobotConfig.Mechanisms.INTAKE_HOLD_TOLERANCE_TICKS);
      ((DcMotorEx) motor).setVelocityPIDFCoefficients(RobotConfig.IntakeMotor.VELOCITY_P,
          RobotConfig.IntakeMotor.VELOCITY_I, RobotConfig.IntakeMotor.VELOCITY_D,
          RobotConfig.IntakeMotor.VELOCITY_F);
    }
  }

  public void toggleDirection() {
    direction = direction == Direction.FORWARD ? Direction.REVERSE : Direction.FORWARD;
    applyPower();
  }

  public void toggleEnabled() {
    setEnabled(!enabled);
  }

  public void setEnabled(boolean enabled) {
    setState(enabled, direction);
  }

  /** Applies enable and direction together, without an intermediate running command. */
  public void setState(boolean enabled, Direction direction) {
    if (direction == null) { throw new IllegalArgumentException("Intake direction is required"); }
    this.enabled = enabled;
    this.direction = direction;
    applyPower();
  }

  public Direction getDirection() {
    return direction;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public boolean isHolding() { return holding; }
  public int getHoldPositionTicks() { return holdPositionTicks; }
  public long getHoldErrorTicks() { return (long) holdPositionTicks - motor.getCurrentPosition(); }
  public double getTargetRpm() {
    return Math.min(RobotConfig.Mechanisms.INTAKE_TARGET_RPM, RobotConfig.IntakeMotor.MAX_RPM);
  }
  public double getCommandedRpm() { return enabled ? getTargetRpm() * direction.multiplier : 0; }
  public double getActualRpm() {
    return motor instanceof DcMotorEx ? ((DcMotorEx) motor).getVelocity(AngleUnit.DEGREES) / 6.0 : Double.NaN;
  }
  public double getEncoderTicksPerRevolution() { return motor.getMotorType().getTicksPerRev(); }

  public void stop() {
    enabled = false;
    motor.setPower(0);
    if (holding) { motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }
    holding = false;
    if (holdWhenIdle) { ((DcMotorEx) motor).setVelocity(0, AngleUnit.DEGREES); }
  }

  private void applyPower() {
    if (holdWhenIdle && !enabled) {
      if (!holding) {
        motor.setPower(0);
        ((DcMotorEx) motor).setVelocity(0, AngleUnit.DEGREES);
        holdPositionTicks = motor.getCurrentPosition();
        motor.setTargetPosition(holdPositionTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RobotConfig.Mechanisms.INTAKE_HOLD_POWER);
        holding = true;
      }
      // Capture once: repeated calls/direction changes must not follow external movement.
      return;
    }
    if (holding) {
      motor.setPower(0);
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      holding = false;
    }
    if (holdWhenIdle) {
      ((DcMotorEx) motor).setVelocity(getCommandedRpm() * 6.0, AngleUnit.DEGREES);
      return;
    }
    motor.setPower(
        enabled ? RobotConfig.Mechanisms.INTAKE_POWER * direction.multiplier : 0);
  }
}
