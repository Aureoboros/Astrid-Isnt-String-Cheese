package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

/** Combines mechanism buttons from both gamepads and reports rising-edge toggle actions. */
public final class MechanismControls {
  private boolean previousA;
  private boolean previousB;
  private boolean previousY;

  public Actions read(Gamepad gamepad1, Gamepad gamepad2) {
    return readButtons(
        gamepad1.a || gamepad2.a,
        gamepad1.b || gamepad2.b,
        gamepad1.x || gamepad2.x,
        gamepad1.y || gamepad2.y);
  }

  Actions readButtons(boolean aDown, boolean bDown, boolean xDown, boolean yDown) {
    Actions actions =
        new Actions(
            aDown && !previousA,
            bDown && !previousB,
            xDown,
            yDown && !previousY);
    previousA = aDown;
    previousB = bDown;
    previousY = yDown;
    return actions;
  }

  public static final class Actions {
    private final boolean toggleIntakeEnabled;
    private final boolean toggleServoEnabled;
    private final boolean shoot;
    private final boolean toggleIntakeDirection;

    private Actions(
        boolean toggleIntakeEnabled,
        boolean toggleServoEnabled,
        boolean shoot,
        boolean toggleIntakeDirection) {
      this.toggleIntakeEnabled = toggleIntakeEnabled;
      this.toggleServoEnabled = toggleServoEnabled;
      this.shoot = shoot;
      this.toggleIntakeDirection = toggleIntakeDirection;
    }

    public boolean shouldToggleIntakeEnabled() {
      return toggleIntakeEnabled;
    }

    public boolean shouldToggleServoEnabled() {
      return toggleServoEnabled;
    }

    public boolean shouldShoot() {
      return shoot;
    }

    public boolean shouldToggleIntakeDirection() {
      return toggleIntakeDirection;
    }
  }
}
