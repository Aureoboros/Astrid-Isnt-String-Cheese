package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Isolated, open-loop gate test. RPM targets are converted to power as RPM / rated RPM; there is
 * no guessed encoder-to-RPM conversion, so actual speed is not regulated or measured.
 */
public final class GateMotorSubsystem {
  private final DcMotor motor;
  private final double maxRpm;
  private double targetRpm;
  private boolean active, reversed;

  public GateMotorSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotor.class, RobotConfig.Hardware.GATE_MOTOR),
        RobotConfig.GateMotor.MAX_RPM, RobotConfig.GateMotor.TEST_INITIAL_RPM);
  }

  GateMotorSubsystem(DcMotor motor, double maxRpm, double initialRpm) {
    if (!Double.isFinite(maxRpm) || maxRpm <= 0) {
      throw new IllegalArgumentException("Gate rated RPM must be finite and positive");
    }
    if (!Double.isFinite(initialRpm)) {
      throw new IllegalArgumentException("Gate RPM must be finite");
    }
    this.motor = motor;
    this.maxRpm = maxRpm;
    this.targetRpm = clampRpm(initialRpm);
    stop();
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /** Apply simultaneous direction/enable changes together, without an intermediate running command. */
  public void setState(boolean active, boolean reversed) {
    setState(active, reversed, targetRpm);
  }

  /** Applies enable, direction and RPM target (clamped to 0..rated RPM) in one power command. */
  public void setState(boolean active, boolean reversed, double requestedRpm) {
    if (!Double.isFinite(requestedRpm)) {
      throw new IllegalArgumentException("Gate RPM must be finite");
    }
    this.targetRpm = clampRpm(requestedRpm);
    this.active = active;
    this.reversed = reversed;
    motor.setPower(getCommandedPower());
  }

  public boolean isActive() { return active; }
  public boolean isReversed() { return reversed; }
  public double getTargetRpm() { return targetRpm; }
  public double getCommandedPower() {
    double power = targetRpm / maxRpm;
    return active ? (reversed ? -power : power) : 0;
  }
  public void stop() { active = false; motor.setPower(0); }

  private double clampRpm(double rpm) { return Math.max(0, Math.min(maxRpm, rpm)); }
}
