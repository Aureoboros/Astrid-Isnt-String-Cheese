package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;

/** Named, programmed routes. Destinations are not an obstacle-avoiding path planner. */
public final class BiobuzzAutoRoutes {
  /**
   * POSE_ONLY: no movement. LEAVE_WALL_12_INCHES: drive 12 in straight off the wall.
   * HIVE_CHECK_AND_PARK: drive in front of own raised hive cell, fix X/Y from its AprilTags with
   * the Limelight, then park partly in own loading zone using the corrected position.
   */
  public enum Plan { POSE_ONLY, LEAVE_WALL_12_INCHES, HIVE_CHECK_AND_PARK }

  /** Name suffix of the stop where the Limelight position check runs. */
  public static final String HIVE_CHECK_SUFFIX = "_HIVE_CHECK";

  private BiobuzzAutoRoutes() {}

  public static List<String> destinations(StartLocation start, Plan plan) {
    switch (plan) {
      case POSE_ONLY: return Collections.emptyList();
      case LEAVE_WALL_12_INCHES: return Collections.singletonList(start.clearWallLocationName());
      case HIVE_CHECK_AND_PARK: {
        String side = start.getAlliance().name();
        return Collections.unmodifiableList(Arrays.asList(
            start.clearWallLocationName(),
            side + "_HIVE_APPROACH",
            side + HIVE_CHECK_SUFFIX,
            side + "_HIVE_APPROACH",
            side + "_PARK_CORRIDOR",
            side + "_PARK"));
      }
      default: throw new IllegalArgumentException("Unknown autonomous plan");
    }
  }

  /** True for the stop where the robot should hold still and run the Limelight position check. */
  public static boolean isHiveCheck(String locationName) {
    return locationName.endsWith(HIVE_CHECK_SUFFIX);
  }

  public static boolean usesLimelight(Plan plan) { return plan == Plan.HIVE_CHECK_AND_PARK; }
}
