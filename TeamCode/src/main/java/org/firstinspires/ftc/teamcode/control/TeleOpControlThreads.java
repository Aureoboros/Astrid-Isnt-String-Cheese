package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.control.MechanismControls.Actions;
import org.firstinspires.ftc.teamcode.subsystems.GameMechanisms;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/** Runs drivetrain and mechanism controls on independent, safely managed worker threads. */
public final class TeleOpControlThreads {
  private static final long CONTROL_LOOP_DELAY_MS = 10;

  private final Gamepad gamepad1;
  private final Gamepad gamepad2;
  private final MecanumDrive drive;
  private final GameMechanisms mechanisms;
  private final MechanismControls mechanismControls = new MechanismControls();
  private final AtomicBoolean running = new AtomicBoolean();
  private final AtomicReference<DriveCommand> latestDriveCommand =
      new AtomicReference<>(DualGamepadDriveControls.idle());
  private final AtomicReference<Throwable> failure = new AtomicReference<>();

  private Thread driveThread;
  private Thread mechanismThread;

  public TeleOpControlThreads(
      Gamepad gamepad1, Gamepad gamepad2, MecanumDrive drive, GameMechanisms mechanisms) {
    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;
    this.drive = drive;
    this.mechanisms = mechanisms;
  }

  public void start() {
    if (!running.compareAndSet(false, true)) {
      throw new IllegalStateException("TeleOp control threads are already running");
    }

    driveThread = new Thread(this::runDriveControls, "FTC-Drive-Controls");
    mechanismThread = new Thread(this::runMechanismControls, "FTC-Mechanism-Controls");
    driveThread.start();
    mechanismThread.start();
  }

  public void stop() {
    running.set(false);
    interrupt(driveThread);
    interrupt(mechanismThread);
    join(driveThread);
    join(mechanismThread);
    drive.stop();
    mechanisms.stopAll();
  }

  public DriveCommand getLatestDriveCommand() {
    return latestDriveCommand.get();
  }

  public Throwable getFailure() {
    return failure.get();
  }

  private void runDriveControls() {
    try {
      while (running.get() && !Thread.currentThread().isInterrupted()) {
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        latestDriveCommand.set(command);
        drive.driveRobotCentric(command.getDrive(), command.getStrafe(), command.getRotate());
        if (!pauseControlLoop()) {
          break;
        }
      }
    } catch (Throwable throwable) {
      recordFailure(throwable);
    } finally {
      drive.stop();
    }
  }

  private void runMechanismControls() {
    try {
      while (running.get() && !Thread.currentThread().isInterrupted()) {
        Actions actions = mechanismControls.read(gamepad1, gamepad2);
        if (actions.shouldToggleIntakeDirection()) {
          mechanisms.toggleIntakeDirection();
        }
        if (actions.shouldToggleIntakeEnabled()) {
          mechanisms.toggleIntakeEnabled();
        }
        if (actions.shouldToggleServoEnabled()) {
          mechanisms.toggleServoEnabled();
        }
        mechanisms.setShooterActive(actions.shouldShoot());
        if (!pauseControlLoop()) {
          break;
        }
      }
    } catch (Throwable throwable) {
      recordFailure(throwable);
    } finally {
      mechanisms.stopAll();
    }
  }

  private void recordFailure(Throwable throwable) {
    failure.compareAndSet(null, throwable);
    running.set(false);
  }

  private static boolean pauseControlLoop() {
    try {
      Thread.sleep(CONTROL_LOOP_DELAY_MS);
      return true;
    } catch (InterruptedException interrupted) {
      Thread.currentThread().interrupt();
      return false;
    }
  }

  private static void interrupt(Thread thread) {
    if (thread != null) {
      thread.interrupt();
    }
  }

  private static void join(Thread thread) {
    if (thread == null) {
      return;
    }
    try {
      thread.join(500);
    } catch (InterruptedException interrupted) {
      Thread.currentThread().interrupt();
    }
  }
}
