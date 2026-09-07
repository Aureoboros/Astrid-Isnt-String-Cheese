package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Owns the intake, shooter, and continuous-rotation servo hardware. */
public final class GameMechanisms {
  public enum IntakeDirection {
    FORWARD(1.0),
    REVERSE(-1.0);

    private final double multiplier;

    IntakeDirection(double multiplier) {
      this.multiplier = multiplier;
    }
  }

  private final DcMotor intake;
  private final DcMotor shooter;
  private final CRServo servoMotor;

  private volatile IntakeDirection intakeDirection = IntakeDirection.FORWARD;
  private volatile boolean intakeEnabled;
  private volatile boolean shooterActive;
  private volatile boolean servoEnabled;

  public GameMechanisms(HardwareMap hardwareMap) {
    intake = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.INTAKE);
    shooter = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.SHOOTER);
    servoMotor = hardwareMap.get(CRServo.class, RobotConfig.Hardware.SERVO_MOTOR);
    stopAll();
  }

  public void toggleIntakeDirection() {
    intakeDirection =
        intakeDirection == IntakeDirection.FORWARD
            ? IntakeDirection.REVERSE
            : IntakeDirection.FORWARD;
    applyIntakePower();
  }

  public void toggleIntakeEnabled() {
    intakeEnabled = !intakeEnabled;
    applyIntakePower();
  }

  /** The shooter intentionally uses hold-to-run behavior. */
  public void setShooterActive(boolean active) {
    shooterActive = active;
    shooter.setPower(active ? RobotConfig.Mechanisms.SHOOTER_POWER : 0);
  }

  public void toggleServoEnabled() {
    servoEnabled = !servoEnabled;
    servoMotor.setPower(servoEnabled ? RobotConfig.Mechanisms.SERVO_POWER : 0);
  }

  public IntakeDirection getIntakeDirection() {
    return intakeDirection;
  }

  public boolean isIntakeEnabled() {
    return intakeEnabled;
  }

  public boolean isServoEnabled() {
    return servoEnabled;
  }

  public boolean isShooterActive() {
    return shooterActive;
  }

  public void stopAll() {
    intakeEnabled = false;
    shooterActive = false;
    servoEnabled = false;
    intake.setPower(0);
    shooter.setPower(0);
    servoMotor.setPower(0);
  }

  private void applyIntakePower() {
    double power =
        intakeEnabled
            ? RobotConfig.Mechanisms.INTAKE_POWER * intakeDirection.multiplier
            : 0;
    intake.setPower(power);
  }
}
