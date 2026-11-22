package org.firstinspires.ftc.teamcode.config;

/** New three-batch route commissioning; shared timing/vision settings remain in HiveAutoConfig. */
public final class PreloadedHiveAutoConfig {
  private PreloadedHiveAutoConfig() {}
  // Operator validated the mechanisms and enabled live Red/Blue three-batch operation.
  public static final boolean ROUTE_VERIFIED = true;
  public static final boolean MECHANISMS_ENABLED = true;
  public static final double GATE_RPM = 60;
  public static final boolean GATE_REVERSED = false;
  // Feedback fault guard, independent of the operator-disabled movement deadlines.
  public static final double FEED_PROGRESS_FAULT_SECONDS = 10;
  public static final double SHOOT_BATCH_TIMEOUT_SECONDS = 20;
  // Retains the operator-requested untimed field test, including live mechanisms.
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
  public static final double SHOOT_RPM = 2900;
  public static final boolean SHOOT_REVERSED = RobotConfig.TeleTourney.SHOOTER_REVERSED;

  public static boolean unlimitedDrivingTest() {
    return UNLIMITED_DRIVING_TEST;
  }

  public static double autoBudgetSeconds() {
    return unlimitedDrivingTest() ? Double.POSITIVE_INFINITY : HiveAutoConfig.AUTO_BUDGET_SECONDS;
  }

  public static double legTimeoutSeconds() {
    return unlimitedDrivingTest() ? PRACTICE_LEG_TIMEOUT_SECONDS : HiveAutoConfig.LEG_TIMEOUT_SECONDS;
  }

  public static double positionToleranceInches() {
    return PRACTICE_POSITION_TOLERANCE_INCHES;
  }

  public static boolean canRun() {
    return ROUTE_VERIFIED && HiveAutoConfig.canRun()
        && Double.isFinite(SHOOT_RPM) && SHOOT_RPM > 0 && SHOOT_RPM <= 6000
        && Double.isFinite(GATE_RPM) && GATE_RPM >= RobotConfig.GateMotor.JAM_MIN_COMMAND_RPM
        && GATE_RPM <= RobotConfig.GateMotor.MAX_RPM
        && Double.isFinite(HiveAutoConfig.FEED_POWER) && HiveAutoConfig.FEED_POWER > 0
        && HiveAutoConfig.FEED_POWER <= 1
        && Double.isFinite(HiveAutoConfig.PICKUP_INTAKE_POWER) && HiveAutoConfig.PICKUP_INTAKE_POWER > 0
        && HiveAutoConfig.PICKUP_INTAKE_POWER <= 1
        && Double.isFinite(HiveAutoConfig.PICKUP_HOLD_SECONDS) && HiveAutoConfig.PICKUP_HOLD_SECONDS > 0
        && HiveAutoConfig.PICKUP_HOLD_SECONDS <= 5
        && Double.isFinite(HiveAutoConfig.FEED_PULSE_SECONDS) && HiveAutoConfig.FEED_PULSE_SECONDS > 0
        && HiveAutoConfig.FEED_PULSE_SECONDS <= .5 && CameraPanConfig.travelReady()
        && Double.isFinite(FEED_PROGRESS_FAULT_SECONDS) && FEED_PROGRESS_FAULT_SECONDS > 0
        && Double.isFinite(SHOOT_BATCH_TIMEOUT_SECONDS) && SHOOT_BATCH_TIMEOUT_SECONDS > 0;
  }
}
