package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.GoBilda6000RpmMotor;

/** Isolated Shoot1 control: never looks up or commands Shoot2. */
public final class SingleShooterSubsystem {
  private static final double DEGREES_PER_SECOND_PER_RPM = 6.0;
  private final DcMotorEx motor;
  private boolean active;
  private boolean reversed;
  private double targetRpm = Math.max(0, Math.min(6000, RobotConfig.Mechanisms.SHOOTER_TARGET_RPM));

  public SingleShooterSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.SHOOTER_1));
  }

  SingleShooterSubsystem(DcMotorEx motor) {
    this.motor = motor;
    GoBilda6000RpmMotor.configure(motor);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    stop();
  }

  public void setActive(boolean active) {
    setState(active, reversed);
  }

  /** Changes speed while retaining the current enable and direction state. */
  public void setTargetRpm(double requestedRpm) {
    setState(active, reversed, requestedRpm);
  }

  /** Applies on/off and direction together so simultaneous toggles issue one velocity command. */
  public void setState(boolean active, boolean reversed) {
    setState(active, reversed, targetRpm);
  }

  /** Applies RPM magnitude, direction and enable state together; motor rating is 6000 RPM. */
  public void setState(boolean active, boolean reversed, double requestedRpm) {
    if (!Double.isFinite(requestedRpm)) {
      throw new IllegalArgumentException("Shooter RPM must be finite");
    }
    targetRpm = Math.max(0, Math.min(6000, requestedRpm));
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

  public double getActualRpm() {
    return motor.getVelocity(AngleUnit.DEGREES) / DEGREES_PER_SECOND_PER_RPM;
  }

  public double getEncoderTicksPerRevolution() { return motor.getMotorType().getTicksPerRev(); }

  public double getRawEncoderTicksPerSecond() { return motor.getVelocity(); }

  public void stop() {
    setActive(false);
  }
}
