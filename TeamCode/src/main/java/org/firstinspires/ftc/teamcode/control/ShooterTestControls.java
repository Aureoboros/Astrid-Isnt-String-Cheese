package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

/** Edge-triggered single-shooter test toggles, shared by both controllers. */
public final class ShooterTestControls {
  private boolean active;
  private boolean reversed;
  private boolean previousX;
  private boolean previousY;

  public void update(Gamepad gamepad1, Gamepad gamepad2) {
    updateButtons(gamepad1.x, gamepad1.y, gamepad2.x, gamepad2.y);
  }

  void updateButtons(boolean x1, boolean y1, boolean x2, boolean y2) {
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
  }

  public boolean isActive() { return active; }

  public boolean isReversed() { return reversed; }
}
