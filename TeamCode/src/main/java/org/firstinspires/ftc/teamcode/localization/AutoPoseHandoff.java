package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

/**
 * Carries the autonomous end pose into TeleOp. Static fields survive between OpModes while the
 * Robot Controller app stays running; they are lost on an app restart or robot power cycle.
 */
public final class AutoPoseHandoff {
  private static Waypoint pose;
  private static Alliance alliance;

  private AutoPoseHandoff() {}

  public static synchronized void save(Waypoint endPose, Alliance endAlliance) {
    if (endPose == null || endAlliance == null || !Double.isFinite(endPose.getX())
        || !Double.isFinite(endPose.getY()) || !Double.isFinite(endPose.getHeadingDegrees())) {
      return;
    }
    pose = endPose;
    alliance = endAlliance;
  }

  public static synchronized boolean hasPose() { return pose != null; }
  public static synchronized Waypoint getPose() { return pose; }
  public static synchronized Alliance getAlliance() { return alliance; }
  public static synchronized void clear() { pose = null; alliance = null; }
}
