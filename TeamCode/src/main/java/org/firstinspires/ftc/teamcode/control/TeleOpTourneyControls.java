package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * TeleOpTourney mechanism map; either controller's buttons count.
 * Y toggles intake (starts ON), X toggles gate (starts OFF), B held = spit
 * (intake + gate reversed), A pressed = drive to shooting pose request.
 */
public final class TeleOpTourneyControls {
  private boolean intakeEnabled = true;
  private boolean gateEnabled;
  private boolean spitting;
  private boolean driveToShootRequested;
  private boolean previousA, previousX, previousY;

  public void update(Gamepad gamepad1, Gamepad gamepad2) {
    updateButtons(gamepad1.a || gamepad2.a, gamepad1.b || gamepad2.b,
        gamepad1.x || gamepad2.x, gamepad1.y || gamepad2.y);
  }

  void updateButtons(boolean a, boolean b, boolean x, boolean y) {
    if (y && !previousY) { intakeEnabled = !intakeEnabled; }
    if (x && !previousX) { gateEnabled = !gateEnabled; }
    driveToShootRequested = a && !previousA;
    spitting = b;
    previousA = a; previousX = x; previousY = y;
  }

  /** Intake runs when toggled on, and always while spitting. */
  public boolean isIntakeEnabled() { return spitting || intakeEnabled; }
  public boolean isIntakeReversed() { return spitting; }
  /** Gate runs when toggled on, and always while spitting. */
  public boolean isGateEnabled() { return spitting || gateEnabled; }
  public boolean isGateReversed() { return spitting; }
  public boolean isSpitting() { return spitting; }
  /** True only on the loop where A was newly pressed. */
  public boolean isDriveToShootRequested() { return driveToShootRequested; }
  /** The toggled states, independent of a held spit. */
  public boolean isIntakeToggledOn() { return intakeEnabled; }
  public boolean isGateToggledOn() { return gateEnabled; }
}
