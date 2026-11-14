package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Controls only the intake motor. */
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
  private volatile Direction direction = Direction.FORWARD;
  private volatile boolean enabled;

  public IntakeSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotor.class, RobotConfig.Hardware.INTAKE));
  }

  IntakeSubsystem(DcMotor motor) {
    this.motor = motor;
    stop();
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

  public void stop() {
    enabled = false;
    motor.setPower(0);
  }

  private void applyPower() {
    motor.setPower(
        enabled ? RobotConfig.Mechanisms.INTAKE_POWER * direction.multiplier : 0);
  }
}
