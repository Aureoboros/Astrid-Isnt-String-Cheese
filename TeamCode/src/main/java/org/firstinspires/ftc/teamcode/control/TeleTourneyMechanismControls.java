package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Competition-only mechanism map; both controllers share edge-triggered buttons. */
public final class TeleTourneyMechanismControls {
  private boolean intakeEnabled;
  private boolean intakeReversed;
  private boolean shooterEnabled;
  private boolean gateEnabled, previousB, previousA;
  private boolean previousX, previousY, previousUp, previousDown;
  private boolean previousLeft, previousRight;
  private double gateRpm = Math.max(0, Math.min(RobotConfig.GateMotor.MAX_RPM,
      RobotConfig.GateMotor.TEST_INITIAL_RPM));
  private double shooterRpm = Math.max(0, Math.min(6000, RobotConfig.TeleTourney.SHOOTER_INITIAL_RPM));

  public void update(Gamepad gamepad1, Gamepad gamepad2) {
    updateButtons(gamepad1.x, gamepad1.y, gamepad1.dpad_up, gamepad1.dpad_down,
        gamepad2.x, gamepad2.y, gamepad2.dpad_up, gamepad2.dpad_down);
    updateGateButton(gamepad1.b, gamepad2.b);
    updateIntakeReverse(gamepad1.a, gamepad2.a);
    updateGateSpeed(gamepad1.dpad_left || gamepad2.dpad_left,
        gamepad1.dpad_right || gamepad2.dpad_right);
  }

  private void updateGateSpeed(boolean left, boolean right) {
    int steps = (right && !previousRight ? 1 : 0) - (left && !previousLeft ? 1 : 0);
    gateRpm = Math.max(0, Math.min(RobotConfig.GateMotor.MAX_RPM,
        gateRpm + steps * RobotConfig.GateMotor.TEST_RPM_STEP));
    previousLeft = left;
    previousRight = right;
  }

  /** Flip the selected direction without changing ON/OFF, including while stopped. */
  void updateIntakeReverse(boolean a1, boolean a2) {
    boolean a = a1 || a2;
    if (a && !previousA) {
      intakeReversed = !intakeReversed;
    }
    previousA = a;
  }

  void updateGateButton(boolean b1, boolean b2) {
    boolean b = b1 || b2;
    if (b && !previousB) { gateEnabled = !gateEnabled; }
    previousB = b;
  }

  void updateButtons(boolean x1, boolean y1, boolean up1, boolean down1,
      boolean x2, boolean y2, boolean up2, boolean down2) {
    boolean x = x1 || x2, y = y1 || y2;
    boolean up = up1 || up2, down = down1 || down2;
    if (x && !previousX) { shooterEnabled = !shooterEnabled; }
    if (y && !previousY) {
      intakeEnabled = !intakeEnabled;
    }
    int steps = (up && !previousUp ? 1 : 0) - (down && !previousDown ? 1 : 0);
    shooterRpm = Math.max(0, Math.min(6000,
        shooterRpm + steps * RobotConfig.Diagnostics.SHOOTER_RPM_STEP));
    previousX = x; previousY = y; previousUp = up; previousDown = down;
  }

  public boolean isIntakeEnabled() {
    return intakeEnabled;
  }
  public boolean isIntakeReversed() {
    return intakeReversed;
  }
  public boolean isShooterEnabled() { return shooterEnabled; }
  public boolean isShooterReversed() { return RobotConfig.TeleTourney.SHOOTER_REVERSED; }
  public double getShooterRpm() { return shooterRpm; }
  public boolean isGateEnabled() { return gateEnabled; }
  public boolean isGateReversed() { return RobotConfig.GateMotor.START_REVERSED; }
  public double getGateRpm() { return gateRpm; }
}
