package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Shared tournament map: Y intake, X gate, B reverse intake, A pose reset, Back shooter. */
public class TeleTourneyMechanismControls {
  private boolean intakeEnabled, intakeReversed, gateEnabled;
  private boolean shooterEnabled = true;
  private boolean odometryResetRequested;
  private boolean previousY, previousX, previousB, previousA, previousBack;
  private boolean previousUp, previousDown;
  private boolean turretLeft, turretRight;
  private final double gateRpm = clamp(RobotConfig.TeleTourney.GATE_RPM, RobotConfig.GateMotor.MAX_RPM);
  private double shooterRpm = clamp(RobotConfig.TeleTourney.SHOOTER_INITIAL_RPM, 6000);

  public TeleTourneyMechanismControls() { this(false); }
  public TeleTourneyMechanismControls(boolean intakeStartsOn) { intakeEnabled = intakeStartsOn; }

  public void update(Gamepad first, Gamepad second) {
    boolean y = first.y || second.y, x = first.x || second.x;
    boolean b = first.b || second.b, a = first.a || second.a;
    boolean back = first.back || second.back;
    boolean up = first.dpad_up || second.dpad_up, down = first.dpad_down || second.dpad_down;
    turretLeft = first.dpad_left || second.dpad_left;
    turretRight = first.dpad_right || second.dpad_right;
    if (y && !previousY) { intakeEnabled = !intakeEnabled; }
    if (x && !previousX) { gateEnabled = !gateEnabled; }
    if (b && !previousB) { intakeReversed = !intakeReversed; }
    if (back && !previousBack) { shooterEnabled = !shooterEnabled; }
    odometryResetRequested = a && !previousA;
    int shooterSteps = (up && !previousUp ? 1 : 0) - (down && !previousDown ? 1 : 0);
    shooterRpm = clamp(shooterRpm + shooterSteps * RobotConfig.Diagnostics.SHOOTER_RPM_STEP, 6000);
    previousY = y; previousX = x; previousB = b; previousA = a; previousBack = back;
    previousUp = up; previousDown = down;
  }

  public boolean isIntakeEnabled() { return intakeEnabled; }
  public boolean isIntakeReversed() { return intakeReversed; }
  public boolean isShooterEnabled() { return shooterEnabled; }
  public boolean isShooterReversed() { return RobotConfig.TeleTourney.SHOOTER_REVERSED; }
  public boolean isGateEnabled() { return gateEnabled; }
  public boolean isGateReversed() { return RobotConfig.GateMotor.START_REVERSED; }
  public boolean isOdometryResetRequested() { return odometryResetRequested; }
  public double getShooterRpm() { return shooterRpm; }
  public double getGateRpm() { return gateRpm; }
  public boolean isTurretLeftRequested() { return turretLeft; }
  public boolean isTurretRightRequested() { return turretRight; }
  private static double clamp(double rpm, double max) { return Math.max(0, Math.min(max, rpm)); }
}
