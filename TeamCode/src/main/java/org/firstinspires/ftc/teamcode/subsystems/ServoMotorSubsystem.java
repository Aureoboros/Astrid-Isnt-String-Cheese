package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;

/** Positional camera mount for isolated manual commissioning; never used alongside auto pan. */
public final class ServoMotorSubsystem {
  private final Servo servo;
  private final boolean travelReady;
  private final double minPosition, centerPosition, maxPosition;
  private volatile boolean enabled;
  private double position;
  private boolean hasPositionCommand;

  public ServoMotorSubsystem(HardwareMap hardwareMap) {
    this(hardwareMap.get(Servo.class, RobotConfig.Hardware.SERVO_MOTOR), CameraPanConfig.travelReady(),
        CameraPanConfig.MIN_POSITION, CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION);
  }

  ServoMotorSubsystem(Servo servo, boolean verified, double min, double center, double max) {
    this.servo = servo;
    minPosition = min; centerPosition = center; maxPosition = max; position = center;
    travelReady = verified && Double.isFinite(min) && Double.isFinite(center)
        && Double.isFinite(max) && 0 <= min && min < center && center < max && max <= 1;
    // No initialization movement until the operator verifies safe travel and enables control.
  }

  public void toggleEnabled() {
    enabled = travelReady && !enabled;
    if (enabled) { setPosition(position); }
  }

  public void nudge(double step) { if (enabled) { setPosition(position + step); } }
  public void center() { if (enabled) { setPosition(centerPosition); } }
  public double getPosition() { return hasPositionCommand ? position : Double.NaN; }

  private void setPosition(double requested) {
    if (!travelReady || !Double.isFinite(requested)) { return; }
    position = Math.max(minPosition, Math.min(maxPosition, requested));
    servo.setPosition(position);
    hasPositionCommand = true;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void stop() {
    enabled = false;
    // A positional servo holds its last commanded angle; stopping never sweeps the mount.
  }
}
