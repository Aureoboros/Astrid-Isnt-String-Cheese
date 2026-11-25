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
  private ImagePanTracker imageAim;
  private ServoTrackingTestControls manualImageControls;
  private ExecutorService worker;
  private Future<?> pending;
  private boolean started, tracking = true, waitingForFrame, fault, centerRequested;
  private boolean manualPan;
  private double position = CameraPanConfig.START_POSITION;
  private double lastMove = Double.NEGATIVE_INFINITY;
  private double lastAimTimestamp = Double.NaN;
  private String status;

  public LimelightPanController(HardwareMap hardwareMap, AprilTagTracker tags) {
    this.tags = tags;
    status = "DISABLED: set CameraPanConfig.ENABLED after calibration";
    if (!CameraPanConfig.ENABLED) {
      if (!CameraPanConfig.IMAGE_ONLY_ENABLED) { return; }
      try {
        if (!CameraPanConfig.travelReady()) {
          throw new IllegalArgumentException("Verify servo travel first");
        }
        imageAim = new ImagePanTracker(CameraPanConfig.MIN_POSITION,
            CameraPanConfig.CENTER_POSITION, CameraPanConfig.MAX_POSITION,
            CameraPanConfig.START_POSITION, CameraPanConfig.POSITION_GAIN_PER_TX_DEGREE,
            CameraPanConfig.MAX_POSITION_STEP, CameraPanConfig.TX_DEADBAND_DEGREES,
            CameraPanConfig.SETTLE_SECONDS, CameraPanConfig.MIN_UPDATE_INTERVAL_SECONDS,
            CameraPanConfig.LOST_TARGET_HOLD_SECONDS);
        imageAim.setDirection(CameraPanConfig.TX_TO_POSITION_SIGN);
        manualImageControls = new ServoTrackingTestControls(imageAim,
            CameraPanConfig.MANUAL_POSITION_STEP, CameraPanConfig.MANUAL_REPEAT_SECONDS);
        servo = hardwareMap.get(Servo.class, RobotConfig.Hardware.SERVO_MOTOR);
        // Never acknowledge this gate: unknown yaw must not publish misleading robot poses.
        tags.attachPanGate(new PanPoseGate(CameraPanConfig.SETTLE_SECONDS));
        status = "READY: image-only tracking after Play; robot poses WITHHELD";
      } catch (IllegalArgumentException invalid) {
        servo = null;
        imageAim = null;
        status = "BLOCKED: " + invalid.getMessage();
      }
      return;
    }
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
      status = "READY: start-position command begins after Play";
    } catch (IllegalArgumentException invalid) {
      servo = null;
      status = "BLOCKED: " + invalid.getMessage();
    }
  }

  public void start() {
    if (started || servo == null) { return; }
    started = true;
    if (imageAim != null) {
      imageAim.start(now(), tags.getAimFrameTimestamp());
      if (tracking) { imageAim.toggleTracking(); }
      servo.setPosition(imageAim.getPosition());
      return;
    }
    worker = Executors.newSingleThreadExecutor(task -> {
      Thread thread = new Thread(task, "FTC-Limelight-Pan-Config");
      thread.setDaemon(true);
      return thread;
    });
    move(CameraPanConfig.START_POSITION);
  }

  public void toggleTracking() { setTrackingEnabled(!tracking); }
  public void setTrackingEnabled(boolean enabled) {
    tracking = enabled;
    if (started && imageAim != null && imageAim.isTracking() != enabled) {
      imageAim.toggleTracking();
    }
  }
  public void setTargetIdRange(int minId, int maxId) {
    if (selector != null) { selector.setIdRange(minId, maxId); }
    if (imageAim != null) { imageAim.setTargetIdRange(minId, maxId); }
  }
  public void requestCenter() { if (!fault) { centerRequested = true; } }

  /** Nonblocking: hardware commands occur only here/start; network calls occur only on worker. */
  public void update() {
    update(false, false);
  }

  /** Manual physical left/right inputs take priority over tracking while held. */
  public void update(boolean manualLeft, boolean manualRight) {
    if (!started || fault) { return; }
    manualPan = manualLeft || manualRight;
    if (imageAim != null) {
      double before = imageAim.getPosition();
      if (centerRequested) {
        centerRequested = false;
        imageAim.center(now(), tags.getAimFrameTimestamp());
        // Preserve existing center semantics: tracking resumes unless explicitly disabled.
        if (tracking) { imageAim.toggleTracking(); }
      } else {
        manualImageControls.update(false, false, manualLeft, manualRight,
            tags.getAimTargets(), tags.getAimFrameTimestamp(), now());
      }
      position = imageAim.getPosition();
      if (Math.abs(before - position) > 1e-6) { servo.setPosition(position); }
      return;
    }
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
    if (manualPan) {
      int direction = (manualLeft ? 1 : 0) - (manualRight ? 1 : 0);
      // A missing tag must not block manual panning. Pending HTTP work is handled above,
      // and move() keeps robot-space poses gated until the new mount/frame settles.
      if (direction != 0 && now() - lastMove >= Math.max(
          CameraPanConfig.MANUAL_REPEAT_SECONDS, CameraPanConfig.MIN_UPDATE_INTERVAL_SECONDS)) {
        double next = calibration.clamp(position - CameraPanConfig.TX_TO_POSITION_SIGN
            * direction * CameraPanConfig.MANUAL_POSITION_STEP);
        if (Math.abs(next - position) > 1e-6) { move(next); }
      }
      return; // Opposing inputs hold position and also suppress automatic corrections.
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

  public String getStatus() {
    return started && imageAim != null
        ? "IMAGE ONLY: " + imageAim.getStatus() + "; robot poses WITHHELD" : status;
  }
  public boolean isImageOnly() { return imageAim != null; }
  public boolean isManualPan() { return started && !fault && manualPan; }
  public double getTargetTxDegrees() {
    return imageAim == null ? Double.NaN : imageAim.getTargetTx();
  }
  public boolean isTracking() { return started && tracking && !fault; }
  public double getPosition() { return started ? position : Double.NaN; }
  public double getYawDegrees() { return !started || calibration == null ? Double.NaN : calibration.yawAt(position); }
  public int getTargetId() {
    return imageAim != null ? imageAim.getTargetId() : selector == null ? -1 : selector.getId();
  }

  @Override public void close() {
    started = false;
    if (imageAim != null) { imageAim.stop(); }
    if (pending != null) { pending.cancel(true); }
    if (worker != null) { worker.shutdownNow(); }
    // A positional servo holds its last angle; do not send a blind return-to-center command.
  }

  private static double now() { return System.nanoTime() / 1e9; }
}
