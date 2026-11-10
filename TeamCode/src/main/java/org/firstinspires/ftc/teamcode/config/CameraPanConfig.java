package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.control.PanCalibration;

/** Commission the mechanical limits and yaw mapping before enabling camera movement. */
public final class CameraPanConfig {
  private CameraPanConfig() {}
  public static final boolean ENABLED = false;
  public static final boolean TRAVEL_VERIFIED = true;
  public static final boolean GEOMETRY_VERIFIED = false;
  // Operator-specified travel limits for the isolated manual servo test.
  public static final double MIN_POSITION = 0.0;
  public static final double CENTER_POSITION = 0.2;
  public static final double MAX_POSITION = 0.4;
  // Enter yaw values measured/calibrated in the Limelight UI at these three positions.
  public static final double YAW_AT_MIN_DEGREES = Double.NaN;
  public static final double YAW_AT_CENTER_DEGREES = 0;
  public static final double YAW_AT_MAX_DEGREES = Double.NaN;
  // +1 or -1: which position direction reduces a positive image tx? Verify physically.
  public static final int TX_TO_POSITION_SIGN = 0;
  // +1 if positive UI yaw means physical counterclockwise/left pan, otherwise -1.
  public static final int UI_YAW_TO_CCW_SIGN = 1;
  // Compare /results t6c_rs[5] with UI yaw: +1 for same sign, -1 for opposite sign.
  public static final int RESULT_YAW_SIGN = 0;
  public static final double FIXED_PITCH_DEGREES = 45;
  public static final double FIXED_ROLL_DEGREES = 0;
  // Robot-space pivot position, in meters (forward/right/up).
  // These are the known CENTER lens coordinates; only valid as a pivot if lens is on axis.
  public static final double PIVOT_FORWARD_METERS = 0.0254;
  public static final double PIVOT_RIGHT_METERS = -0.09525;
  public static final double PIVOT_UP_METERS = 0.4318;
  // Lens offset from pivot at CENTER; measure these if the lens is off the pan axis.
  public static final double LENS_FORWARD_FROM_PIVOT_METERS = 0;
  public static final double LENS_RIGHT_FROM_PIVOT_METERS = 0;
  public static final double LENS_UP_FROM_PIVOT_METERS = 0;
  public static final double POSITION_GAIN_PER_TX_DEGREE = 0.001;
  public static final double MAX_POSITION_STEP = 0.01;
  public static final double TX_DEADBAND_DEGREES = 2;
  public static final double SETTLE_SECONDS = 0.35;
  public static final double MIN_UPDATE_INTERVAL_SECONDS = 0.5;
  public static final double LOST_TARGET_HOLD_SECONDS = 1;
  public static final double MANUAL_POSITION_STEP = 0.005;

  public static boolean travelReady() {
    return TRAVEL_VERIFIED && Double.isFinite(MIN_POSITION)
        && Double.isFinite(CENTER_POSITION) && Double.isFinite(MAX_POSITION)
        && MIN_POSITION >= 0 && MIN_POSITION < CENTER_POSITION
        && CENTER_POSITION < MAX_POSITION && MAX_POSITION <= 1;
  }

  public static PanCalibration calibration() {
    return new PanCalibration(MIN_POSITION, CENTER_POSITION, MAX_POSITION,
        YAW_AT_MIN_DEGREES, YAW_AT_CENTER_DEGREES, YAW_AT_MAX_DEGREES,
        UI_YAW_TO_CCW_SIGN, PIVOT_FORWARD_METERS, PIVOT_RIGHT_METERS, PIVOT_UP_METERS,
        LENS_FORWARD_FROM_PIVOT_METERS, LENS_RIGHT_FROM_PIVOT_METERS,
        LENS_UP_FROM_PIVOT_METERS, FIXED_ROLL_DEGREES, FIXED_PITCH_DEGREES);
  }
}
