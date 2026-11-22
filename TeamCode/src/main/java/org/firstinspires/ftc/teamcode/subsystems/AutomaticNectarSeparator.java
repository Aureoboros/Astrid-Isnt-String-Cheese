/**
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.NectarSeparatorController;

Sole gate owner in shooter integrations; never opens it without explicit index calibration. 
public final class AutomaticNectarSeparator implements AutoCloseable {
  private final DcMotor motor;
  private final NectarSeparatorController controller;
  private final double indexPower;
  private boolean closed;
  private int position;

  public AutomaticNectarSeparator(HardwareMap hardwareMap) {
    indexPower = RobotConfig.GateMotor.INDEX_POWER;
    if (!RobotConfig.GateMotor.autoFeedReady()) {
      motor = null; controller = null; return;
    }
    controller = new NectarSeparatorController(RobotConfig.GateMotor.TICKS_PER_INDEX,
        RobotConfig.GateMotor.POSITION_TOLERANCE_TICKS, RobotConfig.GateMotor.SHOOTER_RPM_TOLERANCE,
        RobotConfig.GateMotor.MIN_SHOOTER_RPM, RobotConfig.GateMotor.SHOOTER_STABLE_SECONDS,
        RobotConfig.GateMotor.MIN_BALL_INTERVAL_SECONDS, RobotConfig.GateMotor.POSITION_SETTLE_SECONDS,
        RobotConfig.GateMotor.INDEX_TIMEOUT_SECONDS);
    motor = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.GATE_MOTOR);
    initializeMotor();
  }

  AutomaticNectarSeparator(DcMotor motor, NectarSeparatorController controller, double indexPower) {
    if (motor == null || controller == null || !Double.isFinite(indexPower)
        || indexPower <= 0 || indexPower > 1) { throw new IllegalArgumentException("Invalid indexed gate"); }
    this.motor = motor; this.controller = controller; this.indexPower = indexPower;
    initializeMotor();
  }

  private void initializeMotor() {
    motor.setPower(0);
    motor.setDirection(RobotConfig.GateMotor.MOTOR_DIRECTION);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void update(boolean shooterOn, double requestedRpm, double actualRpm) {
    update(shooterOn, requestedRpm, actualRpm, System.nanoTime() / 1e9);
  }

  void update(boolean shooterOn, double requestedRpm, double actualRpm, double now) {
    if (closed || motor == null) { return; }
    position = motor.getCurrentPosition();
    controller.update(shooterOn, requestedRpm, actualRpm, position, now);
    if (controller.shouldHoldPosition()) {
      motor.setTargetPosition(controller.getTargetTicks());
      if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) { motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
      motor.setPower(indexPower);
    } else { motor.setPower(0); }
  }

  public String getStatus() {
    return controller == null ? "BLOCKED: calibrate GateMotor index/start position and enable AUTO_FEED"
        : controller.getStatus();
  }
  public int getCompletedIndexes() { return controller == null ? 0 : controller.getCompletedIndexes(); }
  public int getPositionTicks() { return position; }
  public int getTargetTicks() { return controller == null ? 0 : controller.getTargetTicks(); }
  public boolean isCalibrated() { return controller != null; }
  @Override public void close() {
    closed = true;
    if (controller != null) { controller.stop(); }
    if (motor != null) { motor.setPower(0); }
  }
}
*/
