package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.MechanismControls;
import org.firstinspires.ftc.teamcode.control.MechanismControls.Actions;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/** Initializes and tests only the intake motor. */
@TeleOp(name = "Test: Intake", group = "Diagnostics")
public final class IntakeTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, true);
    MechanismControls controls = new MechanismControls();

    telemetry.addLine("A toggles intake START/HOLD; Y toggles forward/reverse. Encoder required.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    waitForStart();

    try {
      if (isStopRequested()) { return; }
      intake.setEnabled(false); // Begin holding only after Play.
      while (opModeIsActive()) {
        Actions actions = controls.read(gamepad1, gamepad2);
        if (actions.shouldToggleIntakeEnabled()) {
          intake.toggleEnabled();
        }
        if (actions.shouldToggleIntakeDirection()) {
          intake.toggleDirection();
        }

        telemetry.addData("Intake", intake.isEnabled() ? "RUN" : "HOLD");
        if (intake.isHolding()) {
          telemetry.addData("Hold target / error", "%d / %d ticks",
              intake.getHoldPositionTicks(), intake.getHoldErrorTicks());
        }
        telemetry.addData("Direction", intake.getDirection());
        telemetry.addData("Target / actual RPM", "%.0f / %.0f (signed)",
            intake.getCommandedRpm(), intake.getActualRpm());
        telemetry.addData("Selected RPM / ticks per rev", "%.0f / %.1f",
            intake.getTargetRpm(), intake.getEncoderTicksPerRevolution());
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      intake.stop();
    }
  }
}
