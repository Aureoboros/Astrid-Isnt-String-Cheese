package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Reads robot-relative AprilTag measurements without modifying the robot's odometry pose. */
public final class AprilTagTracker {
  private final Limelight3A limelight;
  private List<AprilTagObservation> observations = Collections.emptyList();

  public AprilTagTracker(HardwareMap hardwareMap) {
    this(hardwareMap, RobotConfig.Hardware.LIMELIGHT);
  }

  public AprilTagTracker(HardwareMap hardwareMap, String deviceName) {
    limelight = hardwareMap.get(Limelight3A.class, deviceName);
    limelight.setPollRateHz(RobotConfig.Localization.POLL_RATE_HZ);
    limelight.pipelineSwitch(RobotConfig.Localization.APRILTAG_PIPELINE);
  }

  public void start() {
    limelight.start();
  }

  public void stop() {
    limelight.stop();
    observations = Collections.emptyList();
  }

  public void update() {
    LLResult result = limelight.getLatestResult();
    if (result == null
        || !result.isValid()
        || result.getStaleness() > RobotConfig.Localization.APRILTAG_RESULT_STALE_MS) {
      observations = Collections.emptyList();
      return;
    }

    List<AprilTagObservation> currentObservations = new ArrayList<>();
    for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
      Pose3D targetPose = fiducial.getTargetPoseRobotSpace();
      if (targetPose == null) {
        continue;
      }

      Position targetPosition = targetPose.getPosition().toUnit(DistanceUnit.INCH);
      currentObservations.add(
          new AprilTagObservation(
              fiducial.getFiducialId(),
              targetPosition.x,
              targetPosition.y,
              targetPosition.z,
              fiducial.getTargetXDegrees(),
              fiducial.getTargetYDegrees()));
    }
    observations = Collections.unmodifiableList(currentObservations);
  }

  public List<AprilTagObservation> getObservations() {
    return observations;
  }

  /** Returns the requested tag observation, or {@code null} when that tag is not visible. */
  public AprilTagObservation getObservation(int tagId) {
    for (AprilTagObservation observation : observations) {
      if (observation.getId() == tagId) {
        return observation;
      }
    }
    return null;
  }

  /** Returns the nearest visible tag, or {@code null} when no tag is visible. */
  public AprilTagObservation getNearestObservation() {
    AprilTagObservation nearest = null;
    for (AprilTagObservation observation : observations) {
      if (nearest == null || observation.getRangeInches() < nearest.getRangeInches()) {
        nearest = observation;
      }
    }
    return nearest;
  }

  public boolean isConnected() {
    return limelight.isConnected();
  }
}
