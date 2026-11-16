package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.GateMotorTestControls;
import org.firstinspires.ftc.teamcode.subsystems.GateMotorSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Driver Station diagnostic: initializes only gateMotor, with no motion until Play and B. */
@TeleOp(name = "Test: Gate Motor", group = "Diagnostics")
public final class GateMotorTestTeleOp extends LinearOpMode {
  @Override public void runOpMode() {
    GateMotorSubsystem gate = new GateMotorSubsystem(hardwareMap);
    GateMotorTestControls controls = new GateMotorTestControls();
    try {
      telemetry.addLine("gateMotor: Expansion Hub motor port 1 (1500-RPM rating).");
      telemetry.addLine("A: swap FORWARD/REVERSE | B: start/stop (either controller, fresh presses).");
      telemetry.addData("Starts OFF / test power", "%.0f%% open-loop; actual RPM not measured",
          RobotConfig.GateMotor.TEST_POWER * 100);
      telemetry.addLine("Stop and allow coast-down before reversing. Keep hands clear of the separator.");
      telemetry.addLine("No drive, intake, shooter, servo or camera is initialized.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      while (opModeIsActive()) {
        controls.update(gamepad1, gamepad2);
        gate.setState(controls.isActive(), controls.isReversed());
        telemetry.addData("Gate motor", gate.isActive() ? "ON" : "OFF");
        telemetry.addData("Direction", gate.isReversed() ? "REVERSE" : "FORWARD");
        telemetry.addData("Commanded power", "%+.0f%%", gate.getCommandedPower() * 100);
        telemetry.addData("Rated speed", "%.0f RPM (not actual speed)", RobotConfig.GateMotor.MAX_RPM);
        telemetry.addLine("A: direction | B: start/stop | DS STOP: abort");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      gate.stop();
    }
  }
}
