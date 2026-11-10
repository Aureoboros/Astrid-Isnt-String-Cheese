package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.firstinspires.ftc.teamcode.config.CameraPanConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.localization.AprilTagAim;
import org.firstinspires.ftc.teamcode.localization.AprilTagTracker;
import org.firstinspires.ftc.teamcode.localization.CameraMountPose;
import org.firstinspires.ftc.teamcode.localization.LimelightPipelineConfiguration;
import org.firstinspires.ftc.teamcode.localization.PanPoseGate;

/** Small pan steps with asynchronous configuration and fail-closed robot-relative pose gating. */
public final class LimelightPanController implements AutoCloseable {
  private final AprilTagTracker tags;
  private Servo servo;
  private PanCalibration calibration;
  private PanPoseGate gate;
  private PanTargetSelector selector;
  private ExecutorService worker;
  private Future<?> pending;
  private boolean started, tracking = true, waitingForFrame, fault, centerRequested;
  private double position = CameraPanConfig.CENTER_POSITION;
  private double lastMove = Double.NEGATIVE_INFINITY;
  private double lastAimTimestamp = Double.NaN;
  private String status;

  public LimelightPanController(HardwareMap hardwareMap, AprilTagTracker tags) {
    this.tags = tags;
    status = "DISABLED: set CameraPanConfig.ENABLED after calibration";
    if (!CameraPanConfig.ENABLED) { return; }
    status = "BLOCKED: verify servo travel and pan geometry";
    if (!CameraPanConfig.travelReady() || !CameraPanConfig.GEOMETRY_VERIFIED) { return; }
    try {
      calibration = CameraPanConfig.calibration();
      calibration.mountAt(position).forResultYawSign(CameraPanConfig.RESULT_YAW_SIGN);
      calibration.stepTowardTag(position, 0, CameraPanConfig.TX_TO_POSITION_SIGN,
          CameraPanConfig.POSITION_GAIN_PER_TX_DEGREE, CameraPanConfig.MAX_POSITION_STEP,
          CameraPanConfig.TX_DEADBAND_DEGREES);
      gate = new PanPoseGate(CameraPanConfig.SETTLE_SECONDS);
      selector = new PanTargetSelector(CameraPanConfig.LOST_TARGET_HOLD_SECONDS);
      if (!Double.isFinite(CameraPanConfig.MIN_UPDATE_INTERVAL_SECONDS)
          || CameraPanConfig.MIN_UPDATE_INTERVAL_SECONDS <= 0
          || !Double.isFinite(CameraPanConfig.LOST_TARGET_HOLD_SECONDS)
          || CameraPanConfig.LOST_TARGET_HOLD_SECONDS < 0) {
        throw new IllegalArgumentException("Invalid pan timing");
      }
      servo = hardwareMap.get(Servo.class, RobotConfig.Hardware.SERVO_MOTOR);
      tags.attachPanGate(gate);
      status = "READY: center command begins after Play";
    } catch (IllegalArgumentException invalid) {
      servo = null;
      status = "BLOCKED: " + invalid.getMessage();
    }
  }

  public void start() {
    if (started || servo == null) { return; }
    started = true;
    worker = Executors.newSingleThreadExecutor(task -> {
      Thread thread = new Thread(task, "FTC-Limelight-Pan-Config");
      thread.setDaemon(true);
      return thread;
    });
    move(CameraPanConfig.CENTER_POSITION);
  }

  public void toggleTracking() { tracking = !tracking; }
  public void setTrackingEnabled(boolean enabled) { tracking = enabled; }
  public void setTargetIdRange(int minId, int maxId) {
    if (selector != null) { selector.setIdRange(minId, maxId); }
  }
  public void requestCenter() { if (!fault) { centerRequested = true; } }

  /** Nonblocking: hardware commands occur only here/start; network calls occur only on worker. */
  public void update() {
    if (!started || fault) { return; }
    if (pending != null) {
      if (!pending.isDone()) { return; }
      try {
        pending.get();
        pending = null;
        gate.acknowledgeUpdate(now());
        waitingForFrame = true;
        status = "SETTLING: waiting for fresh matching mount frame";
      } catch (Exception failed) {
        fault = true;
        tracking = false;
        Throwable cause = failed.getCause() == null ? failed : failed.getCause();
        status = "FAULT: " + cause.getMessage() + "; pan held, tag robot poses blocked";
        return;
      }
    }
    if (centerRequested) {
      centerRequested = false;
      selector.clear();
      move(CameraPanConfig.CENTER_POSITION);
      return;
    }
    if (waitingForFrame) {
      if (!tags.isRobotPoseReady()) { return; }
      waitingForFrame = false;
    }
    if (!tracking) { status = "HOLD: tracking OFF"; return; }
    AprilTagAim target = selector.select(tags.getAimTargets(), now());
    if (target == null) { status = "HOLD: no tag (no blind search)"; return; }
    double timestamp = tags.getAimFrameTimestamp();
    if (timestamp == lastAimTimestamp
        || now() - lastMove < CameraPanConfig.MIN_UPDATE_INTERVAL_SECONDS) { return; }
    lastAimTimestamp = timestamp;
    double next = calibration.stepTowardTag(position, target.getTxDegrees(),
        CameraPanConfig.TX_TO_POSITION_SIGN, CameraPanConfig.POSITION_GAIN_PER_TX_DEGREE,
        CameraPanConfig.MAX_POSITION_STEP, CameraPanConfig.TX_DEADBAND_DEGREES);
    if (Math.abs(next - position) < 0.000001) {
      status = Math.abs(target.getTxDegrees()) <= CameraPanConfig.TX_DEADBAND_DEGREES
          ? "TRACKING: centered" : "HOLD: safe travel limit";
    } else { move(next); }
  }

  private void move(double requestedPosition) {
    position = calibration.clamp(requestedPosition);
    CameraMountPose mount = calibration.mountAt(position);
    gate.beginMove(mount.forResultYawSign(CameraPanConfig.RESULT_YAW_SIGN), tags.getAimFrameTimestamp());
    lastMove = now();
    // Gate observations BEFORE commanding movement; no second hardware owner in these OpModes.
    servo.setPosition(position);
    status = "UPDATING: yaw/offsets on background worker";
    pending = worker.submit(() -> {
      LimelightPipelineConfiguration.applyMount(RobotConfig.Localization.LIMELIGHT_API_HOST,
          RobotConfig.Localization.APRILTAG_PIPELINE, mount);
      return null;
    });
  }

  public String getStatus() { return status; }
  public boolean isTracking() { return started && tracking && !fault; }
  public double getPosition() { return started ? position : Double.NaN; }
  public double getYawDegrees() { return !started || calibration == null ? Double.NaN : calibration.yawAt(position); }
  public int getTargetId() { return selector == null ? -1 : selector.getId(); }

  @Override public void close() {
    started = false;
    if (pending != null) { pending.cancel(true); }
    if (worker != null) { worker.shutdownNow(); }
    // A positional servo holds its last angle; do not send a blind return-to-center command.
  }

  private static double now() { return System.nanoTime() / 1e9; }
}
