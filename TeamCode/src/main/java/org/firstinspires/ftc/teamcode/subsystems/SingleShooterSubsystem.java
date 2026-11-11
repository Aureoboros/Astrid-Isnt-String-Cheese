package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Isolated Shoot1 control: never looks up or commands Shoot2. */
public final class SingleShooterSubsystem {
  private static final double DEGREES_PER_SECOND_PER_RPM = 6.0;
  private final DcMotorEx motor;
  private boolean active;
  private boolean reversed;
  private double targetRpm = RobotConfig.Mechanisms.SHOOTER_TARGET_RPM;

  public SingleShooterSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.SHOOTER_1));
  }

  SingleShooterSubsystem(DcMotorEx motor) {
    this.motor = motor;
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    stop();
  }

  public void setActive(boolean active) {
    setState(active, reversed);
  }

  /** Applies on/off and direction together so simultaneous toggles issue one velocity command. */
  public void setState(boolean active, boolean reversed) {
    this.active = active;
    this.reversed = reversed;
    motor.setVelocity(getCommandedRpm() * DEGREES_PER_SECOND_PER_RPM,
        AngleUnit.DEGREES);
  }

  public boolean isReversed() {
    return reversed;
  }

  /** Signed target: positive forward, negative reverse, zero when OFF. */
  public double getCommandedRpm() {
    return active ? (reversed ? -getTargetRpm() : getTargetRpm()) : 0;
  }

  public boolean isActive() {
    return active;
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  /** Sets the ON speed magnitude, clamped to 0..SHOOTER_TARGET_RPM, and applies it immediately. */
  public void setTargetRpm(double rpm) {
    if (!Double.isFinite(rpm)) {
      return;
    }
    targetRpm = Math.max(0, Math.min(RobotConfig.Mechanisms.SHOOTER_TARGET_RPM, rpm));
    setState(active, reversed);
  }

  public double getActualRpm() {
    return motor.getVelocity(AngleUnit.DEGREES) / DEGREES_PER_SECOND_PER_RPM;
  }

  public void stop() {
    setActive(false);
  }
}
