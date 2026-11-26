package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Central configuration for hardware names and robot-wide tuning values. */
public final class RobotConfig {
  private RobotConfig() {}

  public static final class Hardware {
    public static final String FRONT_LEFT = "frontLeftMotor";
    public static final String FRONT_RIGHT = "frontRightMotor";
    public static final String BACK_LEFT = "backLeftMotor";
    public static final String BACK_RIGHT = "backRightMotor";
    public static final String PINPOINT = "pinpoint";
    // Physical bus selection is made in the active Robot Controller hardware configuration.
    public static final int PINPOINT_I2C_BUS = 1;
    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu";
    public static final String INTAKE = "intake";
    public static final String GATE_MOTOR = "gateMotor";
    public static final String SHOOTER_1 = "Shoot1";
    public static final String SHOOTER_2 = "Shoot2";
    public static final String SERVO_MOTOR = "servoMotor";

    private Hardware() {}
  }

  public static final class Drive {
    /**
     * The installed drivetrain maps the standard mecanum forward/strafe patterns to the opposite
     * physical axes and directions. Keep this enabled for the current wiring.
     */
    public static final boolean SWAP_FORWARD_AND_STRAFE = false;
    public static final boolean REVERSE_ROTATION = false;

    private Drive() {}
  }

  public static final class Localization {
    public static RevHubOrientationOnRobot imuOrientation() {
      return new RevHubOrientationOnRobot(
          RevHubOrientationOnRobot.LogoFacingDirection.UP,
          RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    public static final int APRILTAG_PIPELINE = 0;
    // Camera/server address, NOT the DHCP client address assigned to the Hub or Mac.
    // Must match the Limelight IP configured in the Robot Controller hardware configuration.
    public static final String LIMELIGHT_API_HOST = "172.29.2.1";
    public static final boolean ALLOW_ALL_APRILTAG_IDS = true;
    /** BIOBUZZ AprilTag black-square edge length: 3.25 inches, in Limelight's millimeters. */
    public static final double APRILTAG_SIZE_MM = 82.55;
    public static final int POLL_RATE_HZ = 100;
    public static final long APRILTAG_RESULT_STALE_MS = 250;

    public static final double VALIDATION_POSITION_TOLERANCE_INCHES = 3.0;
    public static final double VALIDATION_HEIGHT_TOLERANCE_INCHES = 2.0;

    private Localization() {}
  }

  public static final class Pinpoint {
    // Preserves the existing pod specification; confirm against the installed pods.
    public static final double TICKS_PER_INCH = (13000 / 6.0) / (2.0 * Math.PI * 0.6889);
    /** Configured distance per encoder tick, shared by both Pinpoint pods. */
    public static final double INCHES_PER_TICK = 1.0 / TICKS_PER_INCH;
    // Forward/X pod: positive offset left of center. Strafe/Y pod: positive offset ahead of center.
    public static final double FORWARD_POD_Y_INCHES = 0.0;
    public static final double STRAFE_POD_X_INCHES = 0.0;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_DIRECTION =
        GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_DIRECTION =
        GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private Pinpoint() {}
  }

  public static final class Mechanisms {
    public static final double INTAKE_POWER = 1.0;
    public static final double SHOOTER_TARGET_RPM = 6000.0;
    public static final double SERVO_POWER = 1.0;

    private Mechanisms() {}
  }

  /** Confirmed Shoot1: goBILDA 5202/3/4, 1:1, 6000 RPM. Not drive/intake motor metadata. */
  public static final class ShooterMotor {
    public static final double TICKS_PER_REVOLUTION = 28.0;
    public static final double GEAR_RATIO = 1.0;
    public static final double MAX_RPM = 6000.0;

    private ShooterMotor() {}
  }

  /** Nectar separator upstream of Shoot1: goBILDA 5203 435 RPM (13.7:1). Feed timing not calibrated. */
  public static final class GateMotor {
    // Port is wiring documentation; the active RC hardware configuration assigns it.
    public static final int EXPANSION_HUB_PORT = 1;
    public static final double MAX_RPM = 435.0;
    /** goBILDA 435 RPM encoder counts per output-shaft revolution (28 x 13.7:1). */
    public static final double TICKS_PER_REVOLUTION = 384.5;
    /** Motor direction that the code calls FORWARD; REVERSE flips the physical spin. */
    public static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    // Closed-loop (encoder) diagnostic speed.
    public static final double TEST_INITIAL_RPM = 190.0;
    /** Initial feed direction for TeleTourney and gate controller tests. */
    public static final boolean START_REVERSED = true;
    public static final double TEST_RPM_STEP = 10.0;
    // REV hub PIDF output uses a 32767 full-scale command. Starting gains need load testing.
    public static final double VELOCITY_F = 32767.0 / (MAX_RPM * TICKS_PER_REVOLUTION / 60.0);
    public static final double VELOCITY_P = 0.1 * VELOCITY_F;
    public static final double VELOCITY_I = 0.1 * VELOCITY_P;
    public static final double VELOCITY_D = 0;
    // Position hold when feeding is toggled off; maximum position-controller power.
    public static final double HOLD_POWER = 0.25;
    public static final int HOLD_POSITION_TOLERANCE_TICKS = 2;

    // Continuous-velocity gate jam recovery (Test: Gate Motor). All timings are nonblocking.
    public static final boolean JAM_RECOVERY_ENABLED = true;
    public static final double JAM_STARTUP_GRACE_SECONDS = 0.4;
    public static final double JAM_STALL_WINDOW_SECONDS = 0.5;
    public static final double JAM_MIN_SPEED_FRACTION = 0.2;
    public static final double JAM_STOP_SECONDS = 0.1;
    public static final double JAM_REVERSE_SECONDS = 0.15;
    public static final double JAM_RESTART_PAUSE_SECONDS = 0.1;
    public static final double JAM_REVERSE_MAX_RPM = 60;
    public static final double JAM_HEALTHY_RESET_SECONDS = 2;
    public static final double JAM_MIN_COMMAND_RPM = 5;
    public static final int JAM_MAX_ATTEMPTS = 3;

    // Motor metadata does not calibrate one-ball travel or the gate's blocking position.
    public static final boolean AUTO_FEED_ENABLED = false;
    public static final boolean ONE_BALL_INDEX_VERIFIED = false;
    public static final boolean START_BLOCK_POSITION_VERIFIED = false;
    // Signed encoder ticks in MOTOR_DIRECTION between successive blocking positions.
    public static final int TICKS_PER_INDEX = 0;
    public static final double INDEX_POWER = 0.2;
    public static final int POSITION_TOLERANCE_TICKS = 2;
    public static final double POSITION_SETTLE_SECONDS = 0.05;
    public static final double INDEX_TIMEOUT_SECONDS = 1.0;
    public static final double MIN_BALL_INTERVAL_SECONDS = 0.1;
    public static final double SHOOTER_RPM_TOLERANCE = 150;
    public static final double SHOOTER_STABLE_SECONDS = 0.25;
    public static final double MIN_SHOOTER_RPM = 1000;

    public static boolean autoFeedReady() {
      return AUTO_FEED_ENABLED && ONE_BALL_INDEX_VERIFIED && START_BLOCK_POSITION_VERIFIED
          && Math.abs((long) TICKS_PER_INDEX) > POSITION_TOLERANCE_TICKS
          && POSITION_TOLERANCE_TICKS >= 0
          && Double.isFinite(INDEX_POWER) && INDEX_POWER > 0 && INDEX_POWER <= 1
          && Double.isFinite(POSITION_SETTLE_SECONDS) && POSITION_SETTLE_SECONDS >= 0
          && Double.isFinite(INDEX_TIMEOUT_SECONDS) && INDEX_TIMEOUT_SECONDS > POSITION_SETTLE_SECONDS
          && Double.isFinite(MIN_BALL_INTERVAL_SECONDS) && MIN_BALL_INTERVAL_SECONDS >= 0.1
          && Double.isFinite(SHOOTER_RPM_TOLERANCE) && SHOOTER_RPM_TOLERANCE > 0
          && Double.isFinite(SHOOTER_STABLE_SECONDS) && SHOOTER_STABLE_SECONDS > 0
          && Double.isFinite(MIN_SHOOTER_RPM) && MIN_SHOOTER_RPM > SHOOTER_RPM_TOLERANCE;
    }

    private GateMotor() {}
  }

  public static final class TeleTourney {
    /** Temporary drive power limit for testing (1.0 restores full power). */
    public static final double MAX_DRIVE_POWER = 0.35;
    public static final double SHOOTER_INITIAL_RPM = 3200.0;
    public static final boolean SHOOTER_REVERSED = true;

    private TeleTourney() {}
  }

  public static final class Diagnostics {
    public static final double DRIVE_MOTOR_TEST_MAX_POWER = 0.35;
    public static final double SHOOTER_INITIAL_RPM = 6000.0;
    public static final double SHOOTER_RPM_STEP = 50.0;
    /** Shoot1 starting target in the gate motor test; D-pad up/down adjusts it. */
    public static final double GATE_TEST_SHOOTER_RPM = 4500.0;

    private Diagnostics() {}
  }

  public static final class PositionHold {
    /** Below this translation speed the robot is treated as stopped and its pose is locked. */
    public static final double SETTLED_SPEED_INCHES_PER_SECOND = 2.0;
    public static final double SETTLED_TURN_DEGREES_PER_SECOND = 10.0;
    /** Lock the pose anyway if the robot is still sliding this long after the sticks release. */
    public static final double SETTLE_TIMEOUT_SECONDS = 0.5;

    private PositionHold() {}
  }

  public static final class Autonomous {
    // Actual bumper-to-bumper STARTING footprint, not wheelbase; presets assume wall contact.
    // Defaults use the R102 maximum. Measure these before using the BIOBUZZ field presets.
    public static final double START_LENGTH_INCHES = 18.0;
    public static final double START_WIDTH_INCHES = 18.0;
    public static final double POSITION_TOLERANCE_INCHES = 1.5;
    public static final double HEADING_TOLERANCE_DEGREES = 2.0;
    public static final double MAX_DRIVE_POWER = 0.6;
    public static final double MAX_TURN_POWER = 0.5;
    public static final double WAYPOINT_TIMEOUT_SECONDS = 5.0;
    public static final double SETTLE_TIME_SECONDS = 0.3;

    public static final double TRANSLATION_KP = 0.05;
    public static final double TRANSLATION_KI = 0.002;
    public static final double TRANSLATION_KD = 0.01;
    public static final double TRANSLATION_MAX_INTEGRAL = 0.5;
    public static final double TRANSLATION_DEADBAND = 0.5;

    public static final double HEADING_KP = 0.03;
    public static final double HEADING_KI = 0.001;
    public static final double HEADING_KD = 0.005;
    public static final double HEADING_MAX_INTEGRAL = 0.3;
    public static final double HEADING_DEADBAND = 1.0;

    private Autonomous() {}
  }
}
