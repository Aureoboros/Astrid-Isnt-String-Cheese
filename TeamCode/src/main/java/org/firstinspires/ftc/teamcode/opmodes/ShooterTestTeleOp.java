package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.ShooterTestControls;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;

/** Initializes and tests only Shoot1; no second shooter motor is required. */
@TeleOp(name = "Test: Shooter", group = "Diagnostics")
public final class ShooterTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    SingleShooterSubsystem shooter = new SingleShooterSubsystem(hardwareMap);
    ShooterTestControls controls = new ShooterTestControls();
    try {
      telemetry.addLine("Shoot1: Expansion Hub motor port 2; encoder required.");
      telemetry.addLine("X toggles Shoot1 ON/OFF; Y toggles FORWARD/REVERSE (either gamepad).");
      telemetry.addLine("D-pad up/down: +/-100 RPM per press, 0-6000 RPM (either gamepad).");
      telemetry.addLine("Stop the shooter before reversing a fast-spinning mechanism.");
      telemetry.addLine("Shoot2 is not initialized by this test.");
      telemetry.addData("Shoot1 target", "%.0f RPM magnitude when ON", shooter.getTargetRpm());
      telemetry.addData("Shoot1", "OFF / FORWARD");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();

      while (opModeIsActive()) {
        controls.update(gamepad1, gamepad2);
        shooter.setState(controls.isActive(), controls.isReversed(), controls.getTargetRpm());
        double commandedRpm = shooter.getCommandedRpm();
        double actualRpm = shooter.getActualRpm();
        telemetry.addData("Shoot1", shooter.isActive() ? "ON" : "OFF");
        telemetry.addData("Shoot1 Direction", shooter.isReversed() ? "REVERSE" : "FORWARD");
        telemetry.addData("Shoot1 Selected RPM", "%.0f RPM magnitude", shooter.getTargetRpm());
        telemetry.addData("Shoot1 RPM", "target %.0f, actual %.0f", commandedRpm, actualRpm);
        telemetry.addData("Shoot1 Error", "%.0f RPM", commandedRpm - actualRpm);
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      shooter.stop();
    }
  }
}
