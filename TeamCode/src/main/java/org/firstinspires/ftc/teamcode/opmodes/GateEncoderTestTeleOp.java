package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.GateMotorTestControls;
import org.firstinspires.ftc.teamcode.subsystems.GateMotorSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Gate-only diagnostic to separate gate feedback from intake/shooter encoder wiring. */
@TeleOp(name = "Test: Gate Encoder Only", group = "Diagnostics")
public final class GateEncoderTestTeleOp extends LinearOpMode {
  @Override public void runOpMode() {
    GateMotorSubsystem gate = new GateMotorSubsystem(hardwareMap);
    GateMotorTestControls controls = new GateMotorTestControls();
    try {
      PIDFCoefficients gains = gate.getVelocityPidf();
      telemetry.setMsTransmissionInterval(100);
      telemetry.addLine("Gate only: intake and Shoot1 are not initialized.");
      telemetry.addLine("B: START/HOLD | A: reverse | D-pad LEFT/RIGHT: -/+10 RPM");
      telemetry.addData("Hub velocity PIDF", "P %.3f I %.3f D %.3f F %.3f / %s",
          gains.p, gains.i, gains.d, gains.f, gains.algorithm);
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) { return; }
      long sampledAt = System.nanoTime();
      int sampledPosition = gate.getPositionTicks();
      double positionDerivedRpm = Double.NaN;
      while (opModeIsActive()) {
        controls.update(gamepad1, gamepad2);
        gate.setState(controls.isActive(), controls.isReversed(), controls.getGateRpm());
        int encoderPosition = gate.getPositionTicks();
        long sampleNow = System.nanoTime();
        if (sampleNow - sampledAt >= 250_000_000L) {
          // Independent check of SDK velocity using encoder travel over a 250-ms window.
          positionDerivedRpm = ((long) encoderPosition - sampledPosition) * 60e9
              / (gate.getEncoderTicksPerRevolution() * (sampleNow - sampledAt));
          sampledAt = sampleNow;
          sampledPosition = encoderPosition;
        }
        telemetry.addData("Gate", gate.isHolding() ? "HOLD" : "RUN / RECOVERY");
        telemetry.addData("Hub mode", gate.getRunMode());
        telemetry.addData("Selected / applied / measured RPM", "%.1f / %.1f / %.1f",
            gate.getTargetRpm(), gate.getOutputRpm(), gate.getActualRpm());
        telemetry.addData("Commanded / measured encoder speed", "%.1f / %.1f ticks/s",
            gate.getOutputRpm() * gate.getEncoderTicksPerRevolution() / 60,
            gate.getRawEncoderTicksPerSecond());
        telemetry.addData("RPM from encoder position change", "%.1f (250 ms window)", positionDerivedRpm);
        telemetry.addData("Gate encoder position", "%d ticks", encoderPosition);
        telemetry.addData("Gate encoder resolution", "%.1f ticks/rev (fixed)", gate.getEncoderTicksPerRevolution());
        telemetry.addData("Gate recovery / retries", "%s / %d",
            gate.getRecoveryState(), gate.getRecoveryAttempts());
        if (gate.isHolding()) {
          telemetry.addData("Hold target / error", "%d / %d ticks",
              gate.getHoldPositionTicks(), gate.getHoldErrorTicks());
        }
        telemetry.addLine("Gate power AND encoder cables must both use Expansion Hub port 1.");
        telemetry.addData("Rated gate RPM", "%.0f", RobotConfig.GateMotor.MAX_RPM);
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      gate.stop();
    }
  }
}
