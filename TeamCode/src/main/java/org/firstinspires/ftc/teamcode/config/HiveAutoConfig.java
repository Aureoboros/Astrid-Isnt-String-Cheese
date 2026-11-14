package org.firstinspires.ftc.teamcode.config;

/** Driver Station listing is enabled; commission on the physical field before allowing operation. */
public final class HiveAutoConfig {
  public enum Feeder { NONE, INTAKE, SERVO }
  private HiveAutoConfig() {}

  public static final boolean ROUTE_VERIFIED = false;
  public static final boolean SHOOTING_AND_CAMERA_VERIFIED = false;
  public static final boolean PICKUPS_VERIFIED = false;
  // No counting sensor is configured. Only enable after verifying ONE pollen per pulse and FOUR per pickup.
  public static final boolean TIMED_POLLEN_COUNTING_VERIFIED = false;
  public static final boolean DRY_RUN = true;
  public static final Feeder FEEDER = Feeder.NONE;
  public static final double FEED_POWER = 0.5;
  public static final double FEED_PULSE_SECONDS = 0.15;
  public static final double PICKUP_INTAKE_POWER = 0.6;
  public static final double PICKUP_DRIVE_POWER = 0.15;
  public static final double PICKUP_HOLD_SECONDS = 1.5;
  public static final double SHOOT_RPM_1 = 6000;
  public static final double SHOOT_RPM_2 = 6000;
  // Robot-space Z origin configured in Limelight, NOT lens height (lens is currently 17 in).
  public static final double ROBOT_ORIGIN_HEIGHT_INCHES = 0;
  // Provisional height bands around manual-derived centers ~34.2 / 49.9 in. Calibrate in both states.
  public static final double LOW_MIN_INCHES = 30;
  public static final double LOW_MAX_INCHES = 39;
  public static final double HIGH_MIN_INCHES = 45;
  public static final double HIGH_MAX_INCHES = 55;
  public static final double MAX_DRIVE_POWER = 0.35;
  public static final double LEG_TIMEOUT_SECONDS = 5;
  public static final double AUTO_BUDGET_SECONDS = 28;
  public static final double PARK_RESERVE_SECONDS = 7;

  // ---- Limelight position fix in front of own (initially raised) hive cell ----
  // After the first approach the robot stops at the first shooting pose, reads the hive tags,
  // corrects its X/Y, then re-drives to the exact shooting pose. Heading stays on the Pinpoint IMU.
  public static final boolean RELOCALIZE_AT_HIVE = true;
  /** Distinct Limelight frames averaged before the correction is accepted. */
  public static final int RELOCALIZE_FRAMES = 5;
  /** Give up and continue on odometry if enough frames are not seen in this time. */
  public static final double RELOCALIZE_TIMEOUT_SECONDS = 1.0;
  /** Reject the fix if individual frames disagree by more than this. */
  public static final double RELOCALIZE_MAX_SPREAD_INCHES = 2.0;
  /** A bigger "correction" means a wrong tag map / camera transform, not placement error. */
  public static final double RELOCALIZE_MAX_CORRECTION_INCHES = 8.0;

  /**
   * MEASURED field positions of hive tag CENTERS in field-center inches: {id, X, Y}.
   * Measure with the hive in its STARTING (staged) position, because tags move when it tips.
   * NaN = not measured; unmeasured tags are ignored. If none are measured, the fix is skipped
   * and the route continues on odometry. See doc/BIOBUZZ_HIVE_AUTO.md for the measuring steps.
   */
  public static final double[][] RED_HIVE_TAGS = {
      {34, Double.NaN, Double.NaN},
      {35, Double.NaN, Double.NaN},
      {36, Double.NaN, Double.NaN},
      {37, Double.NaN, Double.NaN},
  };
  public static final double[][] BLUE_HIVE_TAGS = {
      {42, Double.NaN, Double.NaN},
      {43, Double.NaN, Double.NaN},
      {44, Double.NaN, Double.NaN},
      {45, Double.NaN, Double.NaN},
  };

  public static boolean canRun() {
    return ROUTE_VERIFIED && (DRY_RUN || (SHOOTING_AND_CAMERA_VERIFIED && PICKUPS_VERIFIED
        && TIMED_POLLEN_COUNTING_VERIFIED && FEEDER == Feeder.INTAKE
        && finitePositive(SHOOT_RPM_1) && SHOOT_RPM_1 <= 6000
        && finitePositive(SHOOT_RPM_2) && SHOOT_RPM_2 <= 6000
        && finitePositive(FEED_PULSE_SECONDS) && FEED_PULSE_SECONDS <= 0.5
        && Double.isFinite(FEED_POWER) && Math.abs(FEED_POWER) > 0 && Math.abs(FEED_POWER) <= 1
        && Double.isFinite(PICKUP_INTAKE_POWER) && Math.abs(PICKUP_INTAKE_POWER) > 0
        && Math.abs(PICKUP_INTAKE_POWER) <= 1 && finitePositive(PICKUP_DRIVE_POWER)
        && PICKUP_DRIVE_POWER <= MAX_DRIVE_POWER && finitePositive(PICKUP_HOLD_SECONDS)
        && PICKUP_HOLD_SECONDS <= 5
        && Double.isFinite(ROBOT_ORIGIN_HEIGHT_INCHES)
        && Double.isFinite(LOW_MIN_INCHES) && Double.isFinite(LOW_MAX_INCHES)
        && Double.isFinite(HIGH_MIN_INCHES) && Double.isFinite(HIGH_MAX_INCHES)
        && LOW_MIN_INCHES < LOW_MAX_INCHES && LOW_MAX_INCHES < HIGH_MIN_INCHES
        && HIGH_MIN_INCHES < HIGH_MAX_INCHES));
  }

  private static boolean finitePositive(double value) { return Double.isFinite(value) && value > 0; }
}
