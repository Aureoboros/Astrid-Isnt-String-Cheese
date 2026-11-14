package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ServoMotorSubsystem;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;

/** Isolated positional pan calibration; no drivetrain or Limelight is initialized. */
@TeleOp(name = "Test: Servo Motor", group = "Diagnostics")
public final class ServoMotorTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    ServoMotorSubsystem servoMotor = new ServoMotorSubsystem(hardwareMap);

    telemetry.addLine("B: enable/hold | D-pad left/right: small position step | A: center");
    telemetry.addLine("Press Play, then B to enable at start. A moves to center. No automatic sweep.");
    telemetry.addData("Servo start", "%.3f", CameraPanConfig.START_POSITION);
    telemetry.addData("Servo min / center / max", "%.3f / %.3f / %.3f",
        CameraPanConfig.MIN_POSITION, CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION);
    telemetry.addData("Safe travel verified", CameraPanConfig.travelReady());
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    boolean previousB = false, previousLeft = false, previousRight = false, previousA = false;
    try {
      waitForStart();
      while (opModeIsActive()) {
        boolean bDown = gamepad1.b || gamepad2.b;
        if (bDown && !previousB) {
          servoMotor.toggleEnabled();
        }
        previousB = bDown;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean a = gamepad1.a || gamepad2.a;
        if (left && !previousLeft) { servoMotor.nudge(-CameraPanConfig.MANUAL_POSITION_STEP); }
        if (right && !previousRight) { servoMotor.nudge(CameraPanConfig.MANUAL_POSITION_STEP); }
        if (a && !previousA) { servoMotor.center(); }
        previousLeft = left; previousRight = right; previousA = a;

        telemetry.addData("Camera servo", servoMotor.isEnabled() ? "MANUAL ENABLED" : "HOLD / BLOCKED");
        telemetry.addData("Commanded position", "%.3f", servoMotor.getPosition());
        telemetry.addData("Servo start", "%.3f", CameraPanConfig.START_POSITION);
        telemetry.addData("Servo min / center / max", "%.3f / %.3f / %.3f",
            CameraPanConfig.MIN_POSITION, CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION);
        telemetry.addData("Safe travel verified", CameraPanConfig.travelReady());
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      servoMotor.stop();
    }
  }
}
