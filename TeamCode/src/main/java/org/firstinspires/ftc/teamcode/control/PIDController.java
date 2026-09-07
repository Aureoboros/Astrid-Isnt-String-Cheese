package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Reusable PID controller with integral limiting, deadband, and static feedforward. */
public final class PIDController {
  private double kP;
  private double kI;
  private double kD;
  private double kF;

  private double integralSum;
  private double lastError;
  private double maxIntegral = 1.0;
  private double deadband;
  private boolean hasRun;
  private final ElapsedTime timer = new ElapsedTime();

  public PIDController(double kP, double kI, double kD) {
    this(kP, kI, kD, 0);
  }

  public PIDController(double kP, double kI, double kD, double kF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
  }

  public double calculate(double error) {
    double deltaSeconds = timer.seconds();
    timer.reset();

    if (!hasRun) {
      hasRun = true;
      lastError = error;
      return kP * error + kF * Math.signum(error);
    }

    if (Math.abs(error) < deadband) {
      reset();
      return 0;
    }

    integralSum += error * deltaSeconds;
    integralSum = clip(integralSum, -maxIntegral, maxIntegral);

    double derivative = deltaSeconds > 0 ? (error - lastError) / deltaSeconds : 0;
    lastError = error;

    return kP * error + kI * integralSum + kD * derivative + kF * Math.signum(error);
  }

  public double calculate(double target, double current) {
    return calculate(target - current);
  }

  public void reset() {
    integralSum = 0;
    lastError = 0;
    hasRun = false;
    timer.reset();
  }

  public void setCoefficients(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public void setFeedforward(double kF) {
    this.kF = kF;
  }

  public void setMaxIntegral(double maxIntegral) {
    this.maxIntegral = maxIntegral;
  }

  public void setDeadband(double deadband) {
    this.deadband = deadband;
  }

  public double getLastError() {
    return lastError;
  }

  public boolean atTarget(double tolerance) {
    return Math.abs(lastError) < tolerance;
  }

  private static double clip(double value, double minimum, double maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }
}
