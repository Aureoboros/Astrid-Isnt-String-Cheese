package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

/** Gate diagnostic only: fresh A swaps direction, fresh B toggles start/stop. */
public final class GateMotorTestControls {
  private boolean active, reversed, previousA, previousB;

  public void update(Gamepad first, Gamepad second) {
    updateButtons(first.a, first.b, second.a, second.b);
  }

  void updateButtons(boolean a1, boolean b1, boolean a2, boolean b2) {
    boolean a = a1 || a2, b = b1 || b2;
    if (a && !previousA) { reversed = !reversed; }
    if (b && !previousB) { active = !active; }
    previousA = a;
    previousB = b;
  }

  public boolean isActive() { return active; }
  public boolean isReversed() { return reversed; }
}
