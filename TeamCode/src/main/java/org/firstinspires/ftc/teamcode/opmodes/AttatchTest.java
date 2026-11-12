package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/**
 * Combined attachment test: intake, launch motor (Shoot1) and servo in one OpMode.
 *
 * <p>Controls (either gamepad):
 * <ul>
 *   <li>Y: intake ON/OFF</li>
 *   <li>B: intake FORWARD/REVERSE</li>
 *   <li>X: launch motor ON/OFF</li>
 *   <li>D-pad up/down: launch motor target RPM up/down</li>
 *   <li>D-pad left/right: small servo position step (one per press)</li>
 *   <li>A: center the servo</li>
 * </ul>
 */
@TeleOp(name = "Test: Attachments", group = "Diagnostics")
public final class AttatchTest extends LinearOpMode {
  private boolean previousX;
  private boolean previousY;
  private boolean previousB;
  private boolean previousUp;
  private boolean previousDown;
  private boolean previousLeft;
  private boolean previousRight;
  private boolean previousA;

  @Override
  public void runOpMode() {
    IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
    SingleShooterSubsystem launcher = new SingleShooterSubsystem(hardwareMap);
    ServoMotorSubsystem servo = new ServoMotorSubsystem(hardwareMap);

    try {
      telemetry.addLine("Y: intake on/off | B: intake reverse");
      telemetry.addLine("X: launcher on/off | D-pad up/down: launcher RPM");
      telemetry.addLine("D-pad left/right: small servo step | A: center servo");
      telemetry.addLine("The servo is enabled at center when Play is pressed.");
      telemetry.addData("Servo min / center / max", "%.3f / %.3f / %.3f",
          CameraPanConfig.MIN_POSITION, CameraPanConfig.CENTER_POSITION,
          CameraPanConfig.MAX_POSITION);
      telemetry.addData("Launcher target", "%.0f RPM", launcher.getTargetRpm());
      telemetry.addData("Servo travel verified", CameraPanConfig.travelReady());
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      // Same as pressing B in ServoMotorTestTeleOp: enable and hold at the commanded position.
      if (!servo.isEnabled()) {
        servo.toggleEnabled();
      }

      while (opModeIsActive()) {
        boolean x = gamepad1.x || gamepad2.x;
        boolean y = gamepad1.y || gamepad2.y;
        boolean b = gamepad1.b || gamepad2.b;
        boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean a = gamepad1.a || gamepad2.a;

        // Intake
        if (y && !previousY) {
          intake.toggleEnabled();
        }
        if (b && !previousB) {
          intake.toggleDirection();
        }

        // Launch motor
        if (x && !previousX) {
          launcher.setActive(!launcher.isActive());
        }
        if (up && !previousUp) {
          launcher.setTargetRpm(launcher.getTargetRpm() + RobotConfig.Diagnostics.SHOOTER_RPM_STEP);
        }
        if (down && !previousDown) {
          launcher.setTargetRpm(launcher.getTargetRpm() - RobotConfig.Diagnostics.SHOOTER_RPM_STEP);
        }

        // Servo: same edge-triggered logic as ServoMotorTestTeleOp.
        if (left && !previousLeft) {
          servo.nudge(-CameraPanConfig.MANUAL_POSITION_STEP);
        }
        if (right && !previousRight) {
          servo.nudge(CameraPanConfig.MANUAL_POSITION_STEP);
        }
        if (a && !previousA) {
          servo.center();
        }

        previousX = x;
        previousY = y;
        previousB = b;
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        previousA = a;

        double commandedRpm = launcher.getCommandedRpm();
        double actualRpm = launcher.getActualRpm();
        telemetry.addData("Intake", "%s / %s",
            intake.isEnabled() ? "ON" : "OFF", intake.getDirection());
        telemetry.addData("Launcher", launcher.isActive() ? "ON" : "OFF");
        telemetry.addData("Launcher target", "%.0f RPM", launcher.getTargetRpm());
        telemetry.addData("Launcher RPM", "commanded %.0f, actual %.0f", commandedRpm, actualRpm);
        telemetry.addData("Launcher error", "%.0f RPM", commandedRpm - actualRpm);
        telemetry.addData("Servo", servo.isEnabled() ? "MANUAL ENABLED" : "HOLD / BLOCKED");
        telemetry.addData("Servo commanded position", "%.3f", servo.getPosition());
        telemetry.addData("Servo min / center / max", "%.3f / %.3f / %.3f",
            CameraPanConfig.MIN_POSITION, CameraPanConfig.CENTER_POSITION,
            CameraPanConfig.MAX_POSITION);
        telemetry.addData("Safe travel verified", CameraPanConfig.travelReady());
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      launcher.stop();
      intake.stop();
      servo.stop();
    }
  }
}
