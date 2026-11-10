package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.localization.AprilTagObservation;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.control.LimelightPanController;
import org.firstinspires.ftc.teamcode.telemetry.CameraPanTelemetry;

/** Initializes and tests only Limelight AprilTag detection. */
@TeleOp(name = "Test: Limelight", group = "Diagnostics")
public final class LimelightTestTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    AprilTagTracker tracker = new AprilTagTracker(hardwareMap);
    LimelightPanController pan = new LimelightPanController(hardwareMap, tracker);
    List<AprilTagObservation> displayedTags = Collections.emptyList();
    boolean frozen = false;
    boolean previousA = false;
    boolean previousX = false, previousY = false;

    tracker.start();
    try {
      telemetry.addLine("Press A on either gamepad to freeze/unfreeze measurements.");
      telemetry.addLine("X: pan tracking toggle | Y: center (after calibration)");
      CameraPanTelemetry.add(telemetry, pan, tracker);
      telemetry.addData("Pipeline switch", tracker.didPipelineSwitchSucceed());
      telemetry.addData("AprilTag pipeline config", tracker.getIdFilterStatus());
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) { return; }
      pan.start();

      while (opModeIsActive()) {
        tracker.update();
        boolean x = gamepad1.x || gamepad2.x, y = gamepad1.y || gamepad2.y;
        if (x && !previousX) { pan.toggleTracking(); }
        if (y && !previousY) { pan.requestCenter(); }
        previousX = x; previousY = y;
        pan.update();
        boolean aDown = gamepad1.a || gamepad2.a;
        if (aDown && !previousA) {
          frozen = !frozen;
        }
        previousA = aDown;
        if (!frozen) {
          displayedTags = tracker.getObservations();
        }
        if (!tracker.isRobotPoseReady()) { displayedTags = Collections.emptyList(); }

        telemetry.addData("Connected", tracker.isConnected());
        telemetry.addData("Pipeline switch", tracker.didPipelineSwitchSucceed());
        telemetry.addData("AprilTag pipeline config", tracker.getIdFilterStatus());
        telemetry.addData("Display", frozen ? "FROZEN" : "LIVE");
        telemetry.addData("Tags", displayedTags.size());
        CameraPanTelemetry.add(telemetry, pan, tracker);
        for (AprilTagObservation tag : displayedTags) {
          telemetry.addData(
              "Tag " + tag.getId(),
              "forward %.1f, left %.1f, range %.1f, height %.1f in",
              tag.getForwardInches(),
              tag.getLeftInches(),
              tag.getRangeInches(),
              tag.getHeightInches());
        }
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      pan.close();
      tracker.stop();
    }
  }
}
