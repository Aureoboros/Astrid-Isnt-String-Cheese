package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.TeleOpTourneyControls;
import org.firstinspires.ftc.teamcode.control.TeleTourneyMechanismControls;
import org.firstinspires.ftc.teamcode.localization.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.PreloadedHiveAutoRoute;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

/** Shared tournament controls and vision reset, with alliance/auto-end pose selection in INIT. */
@TeleOp(name = "TeleOpTourney", group = "Competition")
public final class TeleOpTourney extends TeleTourney {
  @Override protected double getDrivePowerLimit() { return RobotConfig.TeleOpTourney.MAX_DRIVE_POWER; }
  @Override protected TeleTourneyMechanismControls createMechanismControls() { return new TeleOpTourneyControls(); }

  @Override protected Pose awaitStartingPose() {
    BiobuzzFieldMap field = new BiobuzzFieldMap(
        RobotConfig.Autonomous.START_LENGTH_INCHES, RobotConfig.Autonomous.START_WIDTH_INCHES);
    boolean useAutoPose = AutoPoseHandoff.hasPose();
    Alliance alliance = useAutoPose ? AutoPoseHandoff.getAlliance() : Alliance.RED;
    while (opModeInInit()) {
      if (gamepad1.dpad_left || gamepad2.dpad_left) { alliance = Alliance.RED; useAutoPose = false; }
      if (gamepad1.dpad_right || gamepad2.dpad_right) { alliance = Alliance.BLUE; useAutoPose = false; }
      if (gamepad1.dpad_down || gamepad2.dpad_down) { useAutoPose = false; }
      if ((gamepad1.dpad_up || gamepad2.dpad_up) && AutoPoseHandoff.hasPose()) {
        useAutoPose = true; alliance = AutoPoseHandoff.getAlliance();
      }
      Waypoint start = startPose(field, alliance, useAutoPose);
      telemetry.addData("Alliance", "%s (INIT: D-pad left RED, right BLUE)", alliance);
      telemetry.addData("Starting pose source", useAutoPose ? "Autonomous end pose" : "Alliance start");
      telemetry.addData("Starting pose", "X %.1f, Y %.1f, %.0f deg",
          start.getX(), start.getY(), start.getHeadingDegrees());
      telemetry.addLine("INIT: D-pad up restores auto pose; down selects alliance start");
      telemetry.addLine("At Play: Y intake | X gate | B intake reverse | A absolute LL pose reset");
      telemetry.addLine("Back: shooter ON/OFF (starts ON); D-pad adjusts gate/shooter RPM");
      telemetry.update();
      idle();
    }
    Waypoint start = startPose(field, alliance, useAutoPose);
    return new Pose(start.getX(), start.getY(), Math.toRadians(start.getHeadingDegrees()));
  }

  private static Waypoint startPose(BiobuzzFieldMap field, Alliance alliance, boolean useAutoPose) {
    if (useAutoPose && AutoPoseHandoff.hasPoseFor(alliance)) { return AutoPoseHandoff.getPose(); }
    return field.startPose(new PreloadedHiveAutoRoute(alliance).start());
  }
}
