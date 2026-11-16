package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Isolated, open-loop gate test; no guessed encoder-to-RPM conversion or automatic feed logic. */
public final class GateMotorSubsystem {
  private final DcMotor motor;
  private final double testPower;
  private boolean active, reversed;

  public GateMotorSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(DcMotor.class, RobotConfig.Hardware.GATE_MOTOR), RobotConfig.GateMotor.TEST_POWER);
  }

  GateMotorSubsystem(DcMotor motor, double testPower) {
    if (!Double.isFinite(testPower) || testPower <= 0 || testPower > 1) {
      throw new IllegalArgumentException("Gate test power must be finite and in (0, 1]");
    }
    this.motor = motor;
    this.testPower = testPower;
    stop();
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /** Apply simultaneous direction/enable changes together, without an intermediate running command. */
  public void setState(boolean active, boolean reversed) {
    this.active = active;
    this.reversed = reversed;
    motor.setPower(getCommandedPower());
  }

  public boolean isActive() { return active; }
  public boolean isReversed() { return reversed; }
  public double getCommandedPower() { return active ? (reversed ? -testPower : testPower) : 0; }
  public void stop() { active = false; motor.setPower(0); }
}
