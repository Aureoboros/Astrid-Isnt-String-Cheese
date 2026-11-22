package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.GateMotorTestControls;
import org.firstinspires.ftc.teamcode.subsystems.GateMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/**
 * Driver Station diagnostic: initializes gateMotor, the intake and Shoot1. From Play the intake
 * runs forward continuously and Shoot1 spins at its selected RPM (X toggles it); the gate motor
 * stays off until B.
 */
@TeleOp(name = "Test: Gate Motor", group = "Diagnostics")
public final class GateMotorTestTeleOp extends LinearOpMode {
  @Override public void runOpMode() {
    GateMotorSubsystem gate = new GateMotorSubsystem(hardwareMap);
    GateMotorTestControls controls = new GateMotorTestControls();
    IntakeSubsystem intake = null;
    SingleShooterSubsystem shooter = null;
    boolean shooterReversed = RobotConfig.TeleTourney.SHOOTER_REVERSED;
    try {
      intake = new IntakeSubsystem(hardwareMap);
      shooter = new SingleShooterSubsystem(hardwareMap);
      shooter.setState(false, shooterReversed, controls.getShooterRpm());
      gate.setState(false, false, controls.getGateRpm());
      telemetry.addLine("gateMotor: Expansion Hub motor port 1 (goBILDA 435 RPM, encoder required).");
      telemetry.addLine("A: gate FORWARD/REVERSE | B: gate start/stop (either controller).");
      telemetry.addLine("X: shooter on/off | D-pad left/right: gate RPM | D-pad up/down: shooter RPM.");
      telemetry.addData("Gate starts OFF at", "%.0f RPM (closed-loop)", controls.getGateRpm());
      telemetry.addData("Shoot1 starts ON at Play", "%.0f RPM (closed-loop)", controls.getShooterRpm());
      telemetry.addLine("Intake runs FORWARD continuously from Play. Keep hands clear.");
      telemetry.addLine("Stop and allow coast-down before reversing the gate.");
      telemetry.addLine("Auto jam recovery: stop/reverse/retry; after 3 failed retries, B OFF then ON resets.");
      telemetry.addLine("No drive, servo or camera is initialized.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (opModeIsActive()) { intake.setState(true, IntakeSubsystem.Direction.FORWARD); }
      while (opModeIsActive()) {
        controls.update(gamepad1, gamepad2);
        gate.setState(controls.isActive(), controls.isReversed(), controls.getGateRpm());
        shooter.setState(controls.isShooterActive(), shooterReversed, controls.getShooterRpm());
        telemetry.addData("Intake", intake.isEnabled() ? "ON (always)" : "OFF");
        telemetry.addData("Shoot1", "%s %s", shooter.isActive() ? "ON" : "OFF",
            shooter.isReversed() ? "REVERSE" : "FORWARD");
        telemetry.addData("Shoot1 target / actual", "%.0f / %.0f RPM",
            shooter.getTargetRpm(), Math.abs(shooter.getActualRpm()));
        telemetry.addData("Gate motor", gate.isActive() ? "ON" : "OFF");
        telemetry.addData("Gate direction", gate.isReversed() ? "REVERSE" : "FORWARD");
        telemetry.addData("Gate target / actual", "%.0f / %.0f RPM",
            gate.getTargetRpm(), Math.abs(gate.getActualRpm()));
        telemetry.addData("Gate applied RPM (signed)", "%.0f", gate.getOutputRpm());
        telemetry.addData("Gate recovery", gate.getRecoveryState());
        telemetry.addData("Gate retry / limit", "%d / %d",
            gate.getRecoveryAttempts(), RobotConfig.GateMotor.JAM_MAX_ATTEMPTS);
        telemetry.addData("Gate jam recoveries", gate.getRecoveryCount());
        if (gate.getRecoveryState() == org.firstinspires.ftc.teamcode.control.GateJamRecoveryController.State.FAULT) {
          telemetry.addLine("Gate stopped: jam persists. Clear obstruction; B OFF then ON to retry.");
        }
        telemetry.addLine("A: gate dir | B: gate on/off | X: shooter on/off");
        telemetry.addLine("D-pad L/R: gate RPM | D-pad U/D: shooter RPM");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      gate.stop();
      if (intake != null) { intake.stop(); }
      if (shooter != null) { shooter.stop(); }
    }
  }
}
