package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
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
 *   <li>D-pad left/right (hold): move the servo</li>
 * </ul>
 */
@TeleOp(name = "Test: Attachments", group = "Diagnostics")
public final class AttatchTest extends LinearOpMode {
  /** Servo travel speed while a d-pad side is held, in servo position units per second. */
  private static final double SERVO_SPEED_PER_SECOND = 0.2;

  private boolean previousX;
  private boolean previousY;
  private boolean previousB;
  private boolean previousUp;
  private boolean previousDown;

  @Override
  public void runOpMode() {
    IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
    SingleShooterSubsystem launcher = new SingleShooterSubsystem(hardwareMap);
    ServoMotorSubsystem servo = new ServoMotorSubsystem(hardwareMap);
    ElapsedTime loopTimer = new ElapsedTime();

    try {
      telemetry.addLine("Y: intake on/off | B: intake reverse");
      telemetry.addLine("X: launcher on/off | D-pad up/down: launcher RPM");
      telemetry.addLine("D-pad left/right (hold): move servo");
      telemetry.addLine("The servo stays still until the first d-pad left/right press.");
      telemetry.addData("Launcher target", "%.0f RPM", launcher.getTargetRpm());
      telemetry.addData("Servo travel verified", CameraPanConfig.travelReady());
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      loopTimer.reset();

      while (opModeIsActive()) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        boolean x = gamepad1.x || gamepad2.x;
        boolean y = gamepad1.y || gamepad2.y;
        boolean b = gamepad1.b || gamepad2.b;
        boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;

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

        // Servo: enabled on the first left/right press, then moves while held.
        if (left != right) {
          if (!servo.isEnabled()) {
            servo.toggleEnabled();
          }
          double step = SERVO_SPEED_PER_SECOND * dt;
          servo.nudge(right ? step : -step);
        }

        previousX = x;
        previousY = y;
        previousB = b;
        previousUp = up;
        previousDown = down;

        double commandedRpm = launcher.getCommandedRpm();
        double actualRpm = launcher.getActualRpm();
        telemetry.addData("Intake", "%s / %s",
            intake.isEnabled() ? "ON" : "OFF", intake.getDirection());
        telemetry.addData("Launcher", launcher.isActive() ? "ON" : "OFF");
        telemetry.addData("Launcher target", "%.0f RPM", launcher.getTargetRpm());
        telemetry.addData("Launcher RPM", "commanded %.0f, actual %.0f", commandedRpm, actualRpm);
        telemetry.addData("Launcher error", "%.0f RPM", commandedRpm - actualRpm);
        telemetry.addData("Servo", servo.isEnabled() ? "ACTIVE" : "WAITING FOR D-PAD");
        telemetry.addData("Servo position", "%.3f (range %.3f to %.3f)", servo.getPosition(),
            CameraPanConfig.MIN_POSITION, CameraPanConfig.MAX_POSITION);
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
