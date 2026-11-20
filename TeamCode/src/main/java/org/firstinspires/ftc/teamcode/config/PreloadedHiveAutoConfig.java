package org.firstinspires.ftc.teamcode.config;

/** New three-batch route commissioning; shared timing/vision settings remain in HiveAutoConfig. */
public final class PreloadedHiveAutoConfig {
  private PreloadedHiveAutoConfig() {}
  // Operator explicitly approved enabling the new route for driving-only use.
  public static final boolean ROUTE_VERIFIED = true;
  public static final double SHOOT_RPM = RobotConfig.TeleTourney.SHOOTER_INITIAL_RPM;
  public static final boolean SHOOT_REVERSED = RobotConfig.TeleTourney.SHOOTER_REVERSED;

  public static boolean canRun() {
    return ROUTE_VERIFIED && HiveAutoConfig.canRun()
        && Double.isFinite(SHOOT_RPM) && SHOOT_RPM > 0 && SHOOT_RPM <= 6000;
  }
}
