package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.GateJamRecoveryController;

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
  private final GateJamRecoveryController jam;

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
    jam = new GateJamRecoveryController(ticksPerRevolution,
        RobotConfig.GateMotor.JAM_STARTUP_GRACE_SECONDS,
        RobotConfig.GateMotor.JAM_STALL_WINDOW_SECONDS,
        RobotConfig.GateMotor.JAM_MIN_SPEED_FRACTION,
        RobotConfig.GateMotor.JAM_STOP_SECONDS, RobotConfig.GateMotor.JAM_REVERSE_SECONDS,
        RobotConfig.GateMotor.JAM_RESTART_PAUSE_SECONDS, RobotConfig.GateMotor.JAM_REVERSE_MAX_RPM,
        RobotConfig.GateMotor.JAM_HEALTHY_RESET_SECONDS, RobotConfig.GateMotor.JAM_MIN_COMMAND_RPM,
        RobotConfig.GateMotor.JAM_MAX_ATTEMPTS);
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
    setState(active, reversed, requestedRpm, System.nanoTime() / 1e9);
  }

  /** Repeated loop calls advance recovery without blocking drivetrain or other mechanisms. */
  void setState(boolean active, boolean reversed, double requestedRpm, double now) {
    if (!Double.isFinite(requestedRpm)) {
      throw new IllegalArgumentException("Gate RPM must be finite");
    }
    this.targetRpm = clampRpm(requestedRpm);
    this.active = active;
    this.reversed = reversed;
    double command = getCommandedRpm();
    double output = RobotConfig.GateMotor.JAM_RECOVERY_ENABLED
        ? jam.update(command, getActualRpm(), motor.getCurrentPosition(), now) : command;
    motor.setVelocity(output * ticksPerRevolution / 60.0);
  }

  public boolean isActive() { return active; }
  public boolean isReversed() { return reversed; }
  public double getTargetRpm() { return targetRpm; }
  /** Signed target: positive forward, negative reverse, zero when OFF. */
  public double getCommandedRpm() { return active ? (reversed ? -targetRpm : targetRpm) : 0; }
  /** Measured output-shaft RPM from the encoder (signed). */
  public double getActualRpm() { return motor.getVelocity() * 60.0 / ticksPerRevolution; }
  /** Actual command can be zero or temporarily reversed while recovery is active. */
  public double getOutputRpm() {
    return RobotConfig.GateMotor.JAM_RECOVERY_ENABLED ? jam.getOutputRpm() : getCommandedRpm();
  }
  public GateJamRecoveryController.State getRecoveryState() { return jam.getState(); }
  public int getRecoveryAttempts() { return jam.getAttempts(); }
  public int getRecoveryCount() { return jam.getRecoveryCount(); }
  public int getPositionTicks() { return motor.getCurrentPosition(); }
  public void stop() { active = false; jam.stop(); motor.setVelocity(0); }

  private double clampRpm(double rpm) { return Math.max(0, Math.min(maxRpm, rpm)); }
}
