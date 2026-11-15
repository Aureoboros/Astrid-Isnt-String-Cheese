package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.ImagePanTracker;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.PanPoseGate;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Isolated camera/servo aiming without trusting uncalibrated robot-space geometry. */
@TeleOp(name = "Test: Tag Servo Tracking", group = "Diagnostics")
public final class LimelightServoTrackingTeleOp extends LinearOpMode {
  @Override public void runOpMode() {
    if (!CameraPanConfig.travelReady()) {
      telemetry.addLine("BLOCKED: verify camera servo travel limits first");
      telemetry.update();
      return;
    }
    Servo servo = hardwareMap.get(Servo.class, RobotConfig.Hardware.SERVO_MOTOR);
    AprilTagTracker tags = new AprilTagTracker(hardwareMap);
    // Permanently closed in this image-only test. No yaw/offset calibration is assumed or uploaded.
    tags.attachPanGate(new PanPoseGate(CameraPanConfig.SETTLE_SECONDS));
    ImagePanTracker aim = new ImagePanTracker(CameraPanConfig.MIN_POSITION,
        CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION, CameraPanConfig.START_POSITION,
        CameraPanConfig.POSITION_GAIN_PER_TX_DEGREE, CameraPanConfig.MAX_POSITION_STEP,
        CameraPanConfig.TX_DEADBAND_DEGREES, CameraPanConfig.SETTLE_SECONDS,
        CameraPanConfig.MIN_UPDATE_INTERVAL_SECONDS, CameraPanConfig.LOST_TARGET_HOLD_SECONDS);
    if (CameraPanConfig.TX_TO_POSITION_SIGN == 1 || CameraPanConfig.TX_TO_POSITION_SIGN == -1) {
      aim.setDirection(CameraPanConfig.TX_TO_POSITION_SIGN);
    }
    try {
      tags.start();
      telemetry.addLine("After Play: A/B nudge +/- position while tracking OFF to verify pan direction");
      telemetry.addLine("D-pad RIGHT: increasing position turns camera right; LEFT: turns left");
      telemetry.addLine("X: tracking ON/OFF | Y: center and stop tracking (either gamepad)");
      telemetry.addLine("Horizontal aiming only. Distance/height/robot poses withheld; no drive or shooter.");
      telemetry.addData("Start / min / center / max", "%.3f / %.3f / %.3f / %.3f",
          CameraPanConfig.START_POSITION, CameraPanConfig.MIN_POSITION,
          CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION);
      telemetry.addData("Pipeline configuration", tags.getIdFilterStatus());
      telemetry.update();
      waitForStart();
      if (isStopRequested()) { return; }
      tags.update();
      aim.start(now(), tags.getAimFrameTimestamp());
      servo.setPosition(aim.getPosition());
      boolean oldX = false, oldY = false, oldA = false, oldB = false, oldLeft = false, oldRight = false;
      while (opModeIsActive()) {
        tags.update();
        double before = aim.getPosition();
        double time = now(), timestamp = tags.getAimFrameTimestamp();
        boolean x = gamepad1.x || gamepad2.x, y = gamepad1.y || gamepad2.y;
        boolean a = gamepad1.a || gamepad2.a, b = gamepad1.b || gamepad2.b;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean directionChanged = (left && !oldLeft) || (right && !oldRight);
        if (directionChanged) {
          // Direction changes disarm tracking. Opposing new direction selections leave it stopped.
          if (left && !oldLeft && right && !oldRight) { aim.clearDirection(); }
          else if (left && !oldLeft) { aim.setDirection(-1); }
          else { aim.setDirection(1); }
        }
        if (y && !oldY) { aim.center(time, timestamp); }
        else if (x && !oldX && !directionChanged) { aim.toggleTracking(); }
        if (!(y && !oldY)) {
          int steps = (a && !oldA ? 1 : 0) - (b && !oldB ? 1 : 0);
          if (steps != 0) { aim.nudge(steps * CameraPanConfig.MANUAL_POSITION_STEP, time, timestamp); }
          aim.update(tags.getAimTargets(), timestamp, time);
        }
        if (Math.abs(aim.getPosition() - before) > 1e-6) { servo.setPosition(aim.getPosition()); }
        oldX = x; oldY = y; oldA = a; oldB = b; oldLeft = left; oldRight = right;
        telemetry.addData("Connected", tags.isConnected());
        telemetry.addData("Aiming status", aim.getStatus());
        telemetry.addData("Tracking", aim.isTracking());
        telemetry.addData("Position / min / max", "%.3f / %.3f / %.3f",
            aim.getPosition(), CameraPanConfig.MIN_POSITION, CameraPanConfig.MAX_POSITION);
        telemetry.addData("Feedback sign", aim.getDirection());
        telemetry.addData("Target ID / tx", "%d / %.2f deg", aim.getTargetId(), aim.getTargetTx());
        telemetry.addData("Visible aiming tags", tags.getAimTargets().size());
        telemetry.addLine("Distance/height withheld: image-only pan; fixed pitch, no vertical aiming");
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      aim.stop();
      tags.stop();
      // No blind recentering and no blanket hub PWM disable. SDK STOP may remove holding torque.
    }
  }
  private static double now() { return System.nanoTime() / 1e9; }
}
