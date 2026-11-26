package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.GateJamRecoveryController;

/**
 * Isolated gate test with closed-loop encoder velocity: RPM targets are converted to encoder
 * ticks/s using the motor's counts per output revolution, and actual RPM is measured the same way.
 * Inactive feeding holds its last encoder position; stop() removes motor power.
 * Requires the gate motor's encoder cable to be connected.
 */
public final class GateMotorSubsystem {
  private final DcMotorEx motor;
  private final double maxRpm;
  private final double ticksPerRevolution;
  private double targetRpm;
  private boolean active, reversed;
  private boolean holding;
  private int holdPositionTicks;
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
    if (!Double.isFinite(RobotConfig.GateMotor.HOLD_POWER)
        || RobotConfig.GateMotor.HOLD_POWER <= 0 || RobotConfig.GateMotor.HOLD_POWER > 1
        || RobotConfig.GateMotor.HOLD_POSITION_TOLERANCE_TICKS < 0) {
      throw new IllegalArgumentException("Invalid gate hold settings");
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
    // Correct the generic XML metadata without modifying another motor's shared profile.
    MotorConfigurationType type = motor.getMotorType().clone();
    type.setTicksPerRev(ticksPerRevolution);
    type.setMaxRPM(maxRpm);
    motor.setMotorType(type);
    motor.setDirection(RobotConfig.GateMotor.MOTOR_DIRECTION);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setTargetPositionTolerance(RobotConfig.GateMotor.HOLD_POSITION_TOLERANCE_TICKS);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    stop();
    // Metadata alone does not select PIDF gains for a generic XML motor. Explicitly
    // initialize the hub's velocity controller rather than inherit unknown coefficients.
    motor.setVelocityPIDFCoefficients(RobotConfig.GateMotor.VELOCITY_P,
        RobotConfig.GateMotor.VELOCITY_I, RobotConfig.GateMotor.VELOCITY_D,
        RobotConfig.GateMotor.VELOCITY_F);
  }

  /** Apply simultaneous direction/enable changes together, without an intermediate running command. */
  public void setState(boolean active, boolean reversed) {
    setState(active, reversed, targetRpm);
  }

  /** Feeding uses encoder velocity; inactive/zero-RPM feeding holds the captured position. */
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
    if (command == 0) {
      jam.stop(); // HOLD cancels recovery and clears the retry latch as OFF did previously.
      if (!holding) {
        // Cut feeding power before capturing the lock point and changing control mode.
        // HOLD_POWER below is a correction limit, not a continuous rotation command.
        motor.setPower(0);
        motor.setVelocity(0);
        holdPositionTicks = motor.getCurrentPosition();
        motor.setTargetPosition(holdPositionTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RobotConfig.GateMotor.HOLD_POWER);
        holding = true;
      }
      // Keep the original target: recapturing each loop would allow nectar to push the gate.
      return;
    }
    if (holding) {
      motor.setPower(0);
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      holding = false;
    }
    double output = RobotConfig.GateMotor.JAM_RECOVERY_ENABLED
        ? jam.update(command, getActualRpm(), motor.getCurrentPosition(), now) : command;
    motor.setVelocity(output * ticksPerRevolution / 60.0);
  }

  public boolean isActive() { return active; }
  public boolean isHolding() { return holding; }
  public int getHoldPositionTicks() { return holdPositionTicks; }
  public long getHoldErrorTicks() { return (long) holdPositionTicks - motor.getCurrentPosition(); }
  public boolean isReversed() { return reversed; }
  public double getTargetRpm() { return targetRpm; }
  /** Signed feed target: positive forward, negative reverse, zero when holding/stopped. */
  public double getCommandedRpm() { return active ? (reversed ? -targetRpm : targetRpm) : 0; }
  /** Measured output-shaft RPM from the encoder (signed). */
  public double getActualRpm() { return getRawEncoderTicksPerSecond() * 60.0 / ticksPerRevolution; }
  /** SDK encoder velocity in ticks/second, not RPM or accumulated position ticks. */
  public double getRawEncoderTicksPerSecond() { return motor.getVelocity(); }
  /** Fixed calibration: encoder ticks for one output-shaft revolution, independent of speed. */
  public double getEncoderTicksPerRevolution() { return ticksPerRevolution; }
  public DcMotor.RunMode getRunMode() { return motor.getMode(); }
  public PIDFCoefficients getVelocityPidf() {
    return motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  /** Actual command can be zero or temporarily reversed while recovery is active. */
  public double getOutputRpm() {
    return RobotConfig.GateMotor.JAM_RECOVERY_ENABLED ? jam.getOutputRpm() : getCommandedRpm();
  }
  public GateJamRecoveryController.State getRecoveryState() { return jam.getState(); }
  public int getRecoveryAttempts() { return jam.getAttempts(); }
  public int getRecoveryCount() { return jam.getRecoveryCount(); }
  public int getPositionTicks() { return motor.getCurrentPosition(); }
  /** Shutdown/fault cleanup: release active holding torque and stop, even from position mode. */
  public void stop() {
    active = false;
    jam.stop();
    motor.setPower(0);
    if (holding) { motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }
    holding = false;
    motor.setVelocity(0);
  }

  private double clampRpm(double rpm) { return Math.max(0, Math.min(maxRpm, rpm)); }
}
