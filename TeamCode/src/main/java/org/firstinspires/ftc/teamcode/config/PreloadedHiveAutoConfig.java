package org.firstinspires.ftc.teamcode.config;

/** New three-batch route commissioning; shared timing/vision settings remain in HiveAutoConfig. */
public final class PreloadedHiveAutoConfig {
  private PreloadedHiveAutoConfig() {}
  // Operator explicitly approved enabling the new route for driving-only use.
  public static final boolean ROUTE_VERIFIED = true;
  // Practice only: no overall deadline while shooting/pickups are simulated.
  public static final boolean UNLIMITED_DRIVING_TEST = true;
  // Operator requested no movement timeouts for driving-only practice; STOP/fault checks remain.
  public static final double PRACTICE_LEG_TIMEOUT_SECONDS = Double.POSITIVE_INFINITY;
  public static final double PRACTICE_POSITION_TOLERANCE_INCHES = 3;
  // Flowers protrude onto the field: stop this far BEFORE the original pickup target.
  public static final double FLOWER_PICKUP_DEPTH_INCHES = 6;
  public static final double FLOWER_PICKUP_EXTRA_REACH_INCHES = 3;
  public static final double FLOWER_2_ADDITIONAL_REACH_INCHES = 4;
  public static final double PICKUP_POSITION_TOLERANCE_INCHES = 1;
  public static final double OPPOSITE_SHOT_BACKOFF_INCHES = 6;
  // Nearest inner corner of the existing nominal Red loading-zone rectangle.
  public static final double RED_PARK_CORNER_X = -61;
  public static final double RED_PARK_CORNER_Y = 48;
  public static final double SHOOT_RPM = RobotConfig.TeleTourney.SHOOTER_INITIAL_RPM;
  public static final boolean SHOOT_REVERSED = RobotConfig.TeleTourney.SHOOTER_REVERSED;

  public static boolean unlimitedDrivingTest() {
    return UNLIMITED_DRIVING_TEST && HiveAutoConfig.DRY_RUN;
  }

  public static double autoBudgetSeconds() {
    return unlimitedDrivingTest() ? Double.POSITIVE_INFINITY : HiveAutoConfig.AUTO_BUDGET_SECONDS;
  }

  public static double legTimeoutSeconds() {
    return unlimitedDrivingTest() ? PRACTICE_LEG_TIMEOUT_SECONDS : HiveAutoConfig.LEG_TIMEOUT_SECONDS;
  }

  public static double positionToleranceInches() {
    return HiveAutoConfig.DRY_RUN ? PRACTICE_POSITION_TOLERANCE_INCHES : 1.5;
  }

  public static boolean canRun() {
    return ROUTE_VERIFIED && HiveAutoConfig.canRun()
        && Double.isFinite(SHOOT_RPM) && SHOOT_RPM > 0 && SHOOT_RPM <= 6000;
  }
}
