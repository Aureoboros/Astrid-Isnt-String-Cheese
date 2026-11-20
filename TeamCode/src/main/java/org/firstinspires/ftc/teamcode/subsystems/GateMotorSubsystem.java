package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Isolated gate test with closed-loop encoder velocity: RPM targets are converted to encoder
 * ticks/s using the motor's counts per output revolution, and actual RPM is measured the same way.
 * Requires the gate motor's encoder cable to be connected.
 */
public final class GateMotorSubsystem {
  private final DcMotorEx motor;
  private final double maxRpm;
  private final double ticksPerRevolution;
  private double targetRpm;
  private boolean active, reversed;

  public GateMotorSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.GATE_MOTOR),
        RobotConfig.GateMotor.MAX_RPM, RobotConfig.GateMotor.TICKS_PER_REVOLUTION,
        RobotConfig.GateMotor.TEST_INITIAL_RPM);
  }

  GateMotorSubsystem(DcMotorEx motor, double maxRpm, double ticksPerRevolution, double initialRpm) {
    if (!Double.isFinite(maxRpm) || maxRpm <= 0) {
      throw new IllegalArgumentException("Gate rated RPM must be finite and positive");
    }
    if (!Double.isFinite(ticksPerRevolution) || ticksPerRevolution <= 0) {
      throw new IllegalArgumentException("Gate ticks per revolution must be finite and positive");
    }
    if (!Double.isFinite(initialRpm)) {
      throw new IllegalArgumentException("Gate RPM must be finite");
    }
    this.motor = motor;
    this.maxRpm = maxRpm;
    this.ticksPerRevolution = ticksPerRevolution;
    this.targetRpm = clampRpm(initialRpm);
    // The hub XML lists a generic Motor; give this instance the real 435 RPM profile so the
    // hub's velocity controller is tuned for it. Cloned so no shared SDK profile changes.
    MotorConfigurationType type = motor.getMotorType().clone();
    type.setTicksPerRev(ticksPerRevolution);
    type.setMaxRPM(maxRpm);
    motor.setMotorType(type);
    motor.setDirection(RobotConfig.GateMotor.MOTOR_DIRECTION);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    stop();
  }

  /** Apply simultaneous direction/enable changes together, without an intermediate running command. */
  public void setState(boolean active, boolean reversed) {
    setState(active, reversed, targetRpm);
  }

  /** Applies enable, direction and RPM target (clamped to 0..rated RPM) in one velocity command. */
  public void setState(boolean active, boolean reversed, double requestedRpm) {
    if (!Double.isFinite(requestedRpm)) {
      throw new IllegalArgumentException("Gate RPM must be finite");
    }
    this.targetRpm = clampRpm(requestedRpm);
    this.active = active;
    this.reversed = reversed;
    motor.setVelocity(getCommandedRpm() * ticksPerRevolution / 60.0);
  }

  public boolean isActive() { return active; }
  public boolean isReversed() { return reversed; }
  public double getTargetRpm() { return targetRpm; }
  /** Signed target: positive forward, negative reverse, zero when OFF. */
  public double getCommandedRpm() { return active ? (reversed ? -targetRpm : targetRpm) : 0; }
  /** Measured output-shaft RPM from the encoder (signed). */
  public double getActualRpm() { return motor.getVelocity() * 60.0 / ticksPerRevolution; }
  public void stop() { active = false; motor.setVelocity(0); }

  private double clampRpm(double rpm) { return Math.max(0, Math.min(maxRpm, rpm)); }
}
