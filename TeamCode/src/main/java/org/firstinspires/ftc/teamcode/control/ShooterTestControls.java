package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Edge-triggered single-shooter test toggles, shared by both controllers. */
public final class ShooterTestControls {
  private boolean active;
  private boolean reversed;
  private boolean previousX;
  private boolean previousY;
  private boolean previousUp;
  private boolean previousDown;
  private double targetRpm = Math.max(0, Math.min(6000, RobotConfig.Mechanisms.SHOOTER_TARGET_RPM));

  public void update(Gamepad gamepad1, Gamepad gamepad2) {
    updateButtons(gamepad1.x, gamepad1.y, gamepad2.x, gamepad2.y,
        gamepad1.dpad_up, gamepad1.dpad_down, gamepad2.dpad_up, gamepad2.dpad_down);
  }

  void updateButtons(boolean x1, boolean y1, boolean x2, boolean y2) {
    updateButtons(x1, y1, x2, y2, false, false, false, false);
  }

  void updateButtons(boolean x1, boolean y1, boolean x2, boolean y2,
      boolean up1, boolean down1, boolean up2, boolean down2) {
    boolean x = x1 || x2;
    boolean y = y1 || y2;
    if (x && !previousX) {
      active = !active;
    }
    if (y && !previousY) {
      reversed = !reversed;
    }
    previousX = x;
    previousY = y;
    boolean up = up1 || up2;
    boolean down = down1 || down2;
    int steps = (up && !previousUp ? 1 : 0) - (down && !previousDown ? 1 : 0);
    targetRpm = Math.max(0, Math.min(6000,
        targetRpm + steps * RobotConfig.Diagnostics.SHOOTER_RPM_STEP));
    previousUp = up;
    previousDown = down;
  }

  public boolean isActive() { return active; }

  public boolean isReversed() { return reversed; }

  public double getTargetRpm() { return targetRpm; }
}
