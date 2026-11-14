package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Competition-only mechanism map; both controllers share edge-triggered buttons. */
public final class TeleTourneyMechanismControls {
  private enum IntakeState { STOP_BEFORE_FORWARD, FORWARD, STOP_BEFORE_REVERSE, REVERSE }
  private IntakeState intakeState = IntakeState.STOP_BEFORE_FORWARD;
  private boolean shooterEnabled;
  private boolean previousX, previousY, previousUp, previousDown;
  private double shooterRpm = Math.max(0, Math.min(6000, RobotConfig.Mechanisms.SHOOTER_TARGET_RPM));

  public void update(Gamepad gamepad1, Gamepad gamepad2) {
    updateButtons(gamepad1.x, gamepad1.y, gamepad1.dpad_up, gamepad1.dpad_down,
        gamepad2.x, gamepad2.y, gamepad2.dpad_up, gamepad2.dpad_down);
  }

  void updateButtons(boolean x1, boolean y1, boolean up1, boolean down1,
      boolean x2, boolean y2, boolean up2, boolean down2) {
    boolean x = x1 || x2, y = y1 || y2;
    boolean up = up1 || up2, down = down1 || down2;
    if (x && !previousX) { shooterEnabled = !shooterEnabled; }
    if (y && !previousY) {
      intakeState = IntakeState.values()[(intakeState.ordinal() + 1) % IntakeState.values().length];
    }
    int steps = (up && !previousUp ? 1 : 0) - (down && !previousDown ? 1 : 0);
    shooterRpm = Math.max(0, Math.min(6000,
        shooterRpm + steps * RobotConfig.Diagnostics.SHOOTER_RPM_STEP));
    previousX = x; previousY = y; previousUp = up; previousDown = down;
  }

  public boolean isIntakeEnabled() {
    return intakeState == IntakeState.FORWARD || intakeState == IntakeState.REVERSE;
  }
  public boolean isIntakeReversed() {
    return intakeState == IntakeState.REVERSE || intakeState == IntakeState.STOP_BEFORE_REVERSE;
  }
  public boolean isShooterEnabled() { return shooterEnabled; }
  public double getShooterRpm() { return shooterRpm; }
}
