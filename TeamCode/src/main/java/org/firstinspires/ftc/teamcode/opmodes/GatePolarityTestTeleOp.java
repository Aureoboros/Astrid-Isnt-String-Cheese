package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.GatePolarityPulse;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Bypasses velocity/position feedback to diagnose mismatched motor and encoder polarity. */
@TeleOp(name = "Test: Gate Polarity", group = "Diagnostics")
public final class GatePolarityTestTeleOp extends LinearOpMode {
  @Override public void runOpMode() {
    DcMotorEx motor = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.GATE_MOTOR);
    try {
      motor.setPower(0);
      MotorConfigurationType type = motor.getMotorType().clone();
      type.setTicksPerRev(RobotConfig.GateMotor.TICKS_PER_REVOLUTION);
      type.setMaxRPM(RobotConfig.GateMotor.MAX_RPM);
      motor.setMotorType(type);
      motor.setDirection(RobotConfig.GateMotor.MOTOR_DIRECTION);
      motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor.setPower(0);
      GatePolarityPulse pulse = new GatePolarityPulse(0.1, 1.0);
      telemetry.setMsTransmissionInterval(100);
      telemetry.addLine("Gate only. Open-loop 10% power; no velocity PID or position HOLD.");
      telemetry.addLine("Hold B: positive pulse | Hold A: negative pulse | Release: stop");
      telemetry.addLine("Each pulse ends after 1 second. Release both buttons before retrying.");
      telemetry.addLine("Run unloaded to check the gate power/encoder pair on Expansion Hub port 1.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) { return; }
      double previousPower = 0;
      int pulseStartPosition = motor.getCurrentPosition(), pulseSign = 0;
      while (opModeIsActive()) {
        int position = motor.getCurrentPosition();
        double power = pulse.update(gamepad1.b || gamepad2.b,
            gamepad1.a || gamepad2.a, System.nanoTime() / 1e9);
        if (previousPower == 0 && power != 0) {
          pulseStartPosition = position;
          pulseSign = power > 0 ? 1 : -1;
        }
        motor.setPower(power);
        previousPower = power;
        long delta = (long) position - pulseStartPosition;
        double speed = motor.getVelocity();
        telemetry.addData("Gate power", "%.0f%% (open-loop)", power * 100);
        telemetry.addData("Gate mode", motor.getMode());
        telemetry.addData("Pulse direction", pulseSign > 0 ? "POSITIVE" : pulseSign < 0 ? "NEGATIVE" : "NONE");
        telemetry.addData("Encoder position / pulse travel", "%d / %d ticks", position, delta);
        telemetry.addData("Raw encoder speed / measured RPM", "%.1f ticks/s / %.1f RPM",
            speed, speed * 60 / RobotConfig.GateMotor.TICKS_PER_REVOLUTION);
        telemetry.addData("Feedback sign", pulseSign == 0 ? "Run a pulse"
            : delta == 0 ? "No encoder travel yet"
            : Long.signum(delta) == pulseSign ? "Matches pulse direction"
            : "OPPOSITE: check encoder pairing and power/encoder polarity");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      motor.setPower(0);
    }
  }
}
