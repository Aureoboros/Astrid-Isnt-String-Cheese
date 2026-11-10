package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.LimelightPanController;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;

public final class CameraPanTelemetry {
  private CameraPanTelemetry() {}
  public static void add(Telemetry telemetry, LimelightPanController pan, AprilTagTracker tags) {
    telemetry.addData("Camera pan", pan.getStatus());
    telemetry.addData("Pan tracking", pan.isTracking());
    telemetry.addData("Pan servo / yaw", "%.3f / %.1f deg (commanded)",
        pan.getPosition(), pan.getYawDegrees());
    telemetry.addData("Pan target ID", pan.getTargetId());
    telemetry.addData("Raw aiming tags", tags.getAimTargets().size());
    telemetry.addData("Pan pose gate", tags.isRobotPoseReady() ? "OPEN" : "WITHHELD");
  }
}
