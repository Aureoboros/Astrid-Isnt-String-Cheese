package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.ImagePanTracker;
import org.firstinspires.ftc.teamcode.control.ServoTrackingTestControls;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.PanPoseGate;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Isolated camera/servo aiming without trusting uncalibrated robot-space geometry. */
@TeleOp(name = "Test: Tag Servo Tracking", group = "Diagnostics")
public final class LimelightServoTrackingTeleOp extends LinearOpMode {
  @Override public void runOpMode() {
    if (!CameraPanConfig.travelReady()
        || Math.abs(CameraPanConfig.TX_TO_POSITION_SIGN) != 1) {
      telemetry.addLine("BLOCKED: verify camera servo travel limits and direction first");
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
    aim.setDirection(CameraPanConfig.TX_TO_POSITION_SIGN);
    ServoTrackingTestControls controls = new ServoTrackingTestControls(aim,
        CameraPanConfig.MANUAL_POSITION_STEP, 0.1);
    try {
      tags.start();
      telemetry.addLine("After Play: automatic tag alignment is ON");
      telemetry.addLine("Hold D-pad LEFT/RIGHT (or A/B) to pan left/right; release to re-align");
      telemetry.addLine("X: pause/resume tracking | Y: center and pause (either gamepad)");
      telemetry.addLine("Keep the tag visible. Do not force the powered servo by hand.");
      telemetry.addLine("Horizontal aiming only. Distance/height/robot poses withheld; no drive or shooter.");
      telemetry.addData("Start / min / center / max", "%.3f / %.3f / %.3f / %.3f",
          CameraPanConfig.START_POSITION, CameraPanConfig.MIN_POSITION,
          CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION);
      telemetry.addData("Pipeline configuration", tags.getIdFilterStatus());
      telemetry.update();
      waitForStart();
      if (isStopRequested()) { return; }
      tags.update();
      controls.start(now(), tags.getAimFrameTimestamp());
      servo.setPosition(aim.getPosition());
      while (opModeIsActive()) {
        tags.update();
        double before = aim.getPosition();
        double time = now(), timestamp = tags.getAimFrameTimestamp();
        boolean x = gamepad1.x || gamepad2.x, y = gamepad1.y || gamepad2.y;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left || gamepad1.a || gamepad2.a;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right || gamepad1.b || gamepad2.b;
        controls.update(x, y, left, right, tags.getAimTargets(), timestamp, time);
        if (Math.abs(aim.getPosition() - before) > 1e-6) { servo.setPosition(aim.getPosition()); }
        telemetry.addData("Connected", tags.isConnected());
        telemetry.addData("Aiming status", aim.getStatus());
        telemetry.addData("Tracking", aim.isTracking());
        telemetry.addData("Control mode", controls.isManual() ? "MANUAL: release to resume"
            : aim.isTracking() ? "AUTO: align with visible tag" : "PAUSED: X to resume");
        telemetry.addData("Position / min / max", "%.3f / %.3f / %.3f",
            aim.getPosition(), CameraPanConfig.MIN_POSITION, CameraPanConfig.MAX_POSITION);
        telemetry.addData("Feedback sign", aim.getDirection());
        telemetry.addData("Target ID / tx", "%d / %.2f deg", aim.getTargetId(), aim.getTargetTx());
        telemetry.addData("Visible aiming tags", tags.getAimTargets().size());
        telemetry.addLine("Hold D-pad left/right or A/B: manual pan | X: pause/resume | Y: center/pause");
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
