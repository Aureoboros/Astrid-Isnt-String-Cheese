package org.firstinspires.ftc.teamcode.navigation;

import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;

/** Named, programmed routes. Destinations are not an obstacle-avoiding path planner. */
public final class BiobuzzAutoRoutes {
  public enum Plan { POSE_ONLY, LEAVE_WALL_12_INCHES }

  private BiobuzzAutoRoutes() {}

  public static List<String> destinations(StartLocation start, Plan plan) {
    switch (plan) {
      case POSE_ONLY: return Collections.emptyList();
      case LEAVE_WALL_12_INCHES: return Collections.singletonList(start.clearWallLocationName());
      default: throw new IllegalArgumentException("Unknown autonomous plan");
    }
  }
}
