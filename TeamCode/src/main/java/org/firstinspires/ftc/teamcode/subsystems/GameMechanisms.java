package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Intake/shooter hardware owner; camera toggle is a request, not a second servo hardware owner. */
public final class GameMechanisms {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private volatile boolean cameraTrackingEnabled = true;

  public GameMechanisms(HardwareMap hardwareMap) {
    intake = new IntakeSubsystem(hardwareMap);
    shooter = new ShooterSubsystem(hardwareMap);
    intake.stop();
    shooter.stop();
  }

  public void toggleIntakeDirection() {
    intake.toggleDirection();
  }

  public void toggleIntakeEnabled() {
    intake.toggleEnabled();
  }

  /** The shooter intentionally uses hold-to-run behavior. */
  public void setShooterActive(boolean active) {
    shooter.setActive(active);
  }

  public void toggleServoEnabled() {
    cameraTrackingEnabled = !cameraTrackingEnabled;
  }

  public IntakeSubsystem.Direction getIntakeDirection() {
    return intake.getDirection();
  }

  public boolean isIntakeEnabled() {
    return intake.isEnabled();
  }

  public boolean isServoEnabled() {
    return cameraTrackingEnabled;
  }

  public boolean isShooterActive() {
    return shooter.isActive();
  }

  public void stopAll() {
    intake.stop();
    shooter.stop();
    cameraTrackingEnabled = false;
  }
}
