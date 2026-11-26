package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.ImagePanTracker;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Selected-tag image aiming with the same positional-servo limits as TeleTourney. */
@TeleOp(name = "Spin Spin Test", group = "Diagnostics")
public class SpinSpinServoPositions extends LinearOpMode {
    @Override public void runOpMode() {
        if (!CameraPanConfig.travelReady() || Math.abs(CameraPanConfig.TX_TO_POSITION_SIGN) != 1) {
            telemetry.addLine("Verify shared servo travel and direction before running");
            telemetry.update();
            return;
        }
        Servo servo = hardwareMap.get(Servo.class, RobotConfig.Hardware.SERVO_MOTOR);
        AprilTagTracker tags = new AprilTagTracker(hardwareMap);
        tags.attachPanGate(new PanPoseGate(CameraPanConfig.SETTLE_SECONDS));
        ImagePanTracker aim = CameraPanConfig.imageTracker();
        int target = -1;
        boolean previousB = false, previousX = false, previousY = false;
        try {
            tags.start(); // Shared pipeline 0 allows all IDs; selection happens below.
            telemetry.addLine("B: toggle tag 35 | X: toggle tag 20 | Y: center and pause");
            telemetry.addData("Start / min / center / max", "%.3f / %.3f / %.3f / %.3f",
                    CameraPanConfig.START_POSITION, CameraPanConfig.MIN_POSITION,
                    CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION);
            telemetry.addLine("After Play the servo starts at the shared start position");
            OdometryTelemetry.add(telemetry);
            telemetry.update();
            waitForStart();
            if (isStopRequested()) { return; }
            tags.update();
            aim.start(now(), tags.getAimFrameTimestamp());
            servo.setPosition(aim.getPosition());
            while (opModeIsActive()) {
                tags.update();
                boolean b = gamepad1.b || gamepad2.b;
                boolean x = gamepad1.x || gamepad2.x;
                boolean y = gamepad1.y || gamepad2.y;
                double before = aim.getPosition();
                double time = now(), timestamp = tags.getAimFrameTimestamp();
                if ((b && !previousB) || (x && !previousX)) {
                    int requested = b && !previousB ? 35 : 20;
                    target = target == requested ? -1 : requested;
                    if (aim.isTracking()) { aim.toggleTracking(); }
                    if (target >= 0) {
                        aim.setTargetIdRange(target, target);
                        aim.toggleTracking();
                    }
                }
                if (y && !previousY) {
                    target = -1;
                    aim.center(time, timestamp);
                }
                previousB = b; previousX = x; previousY = y;
                aim.update(tags.getAimTargets(), timestamp, time);
                if (Math.abs(aim.getPosition() - before) > 1e-6) { servo.setPosition(aim.getPosition()); }
                telemetry.addData("Connected", tags.isConnected());
                telemetry.addData("Selected tag", target);
                telemetry.addData("Tracking status", aim.getStatus());
                telemetry.addData("Target ID / tx", "%d / %.2f deg", aim.getTargetId(), aim.getTargetTx());
                telemetry.addData("Position / min / max", "%.3f / %.3f / %.3f",
                        aim.getPosition(), CameraPanConfig.MIN_POSITION, CameraPanConfig.MAX_POSITION);
                telemetry.addData("Visible tags", tags.getAimTargets().size());
                telemetry.addData("Pipeline configuration", tags.getIdFilterStatus());
                telemetry.addLine("B: tag 35 | X: tag 20 | Y: center/pause (either gamepad)");
                telemetry.addLine("Horizontal aiming only; distance/height withheld during uncalibrated pan");
                OdometryTelemetry.add(telemetry);
                telemetry.update();
                idle();
            }
        } finally {
            aim.stop();
            tags.stop();
        }
    }
    private static double now() { return System.nanoTime() / 1e9; }
}
