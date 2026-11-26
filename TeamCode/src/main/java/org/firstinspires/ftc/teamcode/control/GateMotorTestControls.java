package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Gate diagnostic, shared by both controllers (fresh presses only): A swaps gate direction,
 * B toggles gate start/hold, X toggles the shooter (starts ON), D-pad left/right steps gate RPM,
 * D-pad up/down steps shooter RPM.
 */
public final class GateMotorTestControls {
  private boolean active, previousA, previousB;
  private boolean reversed = RobotConfig.GateMotor.START_REVERSED;
  private boolean shooterActive = true, previousX;
  private boolean previousUp, previousDown, previousLeft, previousRight;
  private double gateRpm = clamp(RobotConfig.GateMotor.TEST_INITIAL_RPM, RobotConfig.GateMotor.MAX_RPM);
  private double shooterRpm = clamp(RobotConfig.Diagnostics.GATE_TEST_SHOOTER_RPM,
      RobotConfig.ShooterMotor.MAX_RPM);

  public void update(Gamepad first, Gamepad second) {
    updateButtons(first.a, first.b, second.a, second.b);
    updateShooterToggle(first.x || second.x);
    updateDpad(first.dpad_up || second.dpad_up, first.dpad_down || second.dpad_down,
        first.dpad_left || second.dpad_left, first.dpad_right || second.dpad_right);
  }

  void updateButtons(boolean a1, boolean b1, boolean a2, boolean b2) {
    boolean a = a1 || a2, b = b1 || b2;
    if (a && !previousA) { reversed = !reversed; }
    if (b && !previousB) { active = !active; }
    previousA = a;
    previousB = b;
  }

  void updateShooterToggle(boolean x) {
    if (x && !previousX) { shooterActive = !shooterActive; }
    previousX = x;
  }

  void updateDpad(boolean up, boolean down, boolean left, boolean right) {
    int shooterSteps = (up && !previousUp ? 1 : 0) - (down && !previousDown ? 1 : 0);
    int gateSteps = (right && !previousRight ? 1 : 0) - (left && !previousLeft ? 1 : 0);
    shooterRpm = clamp(shooterRpm + shooterSteps * RobotConfig.Diagnostics.SHOOTER_RPM_STEP,
        RobotConfig.ShooterMotor.MAX_RPM);
    gateRpm = clamp(gateRpm + gateSteps * RobotConfig.GateMotor.TEST_RPM_STEP,
        RobotConfig.GateMotor.MAX_RPM);
    previousUp = up; previousDown = down; previousLeft = left; previousRight = right;
  }

  public boolean isActive() { return active; }
  public boolean isReversed() { return reversed; }
  public boolean isShooterActive() { return shooterActive; }
  public double getGateRpm() { return gateRpm; }
  public double getShooterRpm() { return shooterRpm; }

  private static double clamp(double rpm, double max) { return Math.max(0, Math.min(max, rpm)); }
}
