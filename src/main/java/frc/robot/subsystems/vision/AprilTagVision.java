package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Camera.VisionResult;
import frc.robot.subsystems.vision.Camera.VisionResultStatus;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  public static final boolean DO_SUMMARY_LOGGING = true;
  public static final boolean DO_CAMERA_LOGGING = true;

  private final Camera[] cameras;

  private List<Consumer<TimestampedRobotPoseEstimate>> timestampRobotPoseEstimateConsumers =
      new ArrayList<>();

  private boolean hasVisionEstimate = false;

  public AprilTagVision(CameraIO... camerasIO) {
    this.cameras = Arrays.stream(camerasIO).map(io -> new Camera(io)).toArray(Camera[]::new);
  }

  @Override
  public void periodic() {

    // Run periodic for all cameras, as their are not real subsystems
    for (Camera camera : cameras) {
      camera.periodic();
    }

    String root = "Vision";

    List<Pose3d> robotPosesAccepted = new ArrayList<>();
    List<Pose3d> robotPosesRejected = new ArrayList<>();
    List<Pose3d> tagPoses = new ArrayList<>();

    hasVisionEstimate = false;

    // Loop through all cameras
    for (Camera camera : cameras) {
      String cameraRoot = root + "/" + camera.getCameraName();

      // Loop through all results that the camera has
      for (VisionResult result : camera.getResults()) {

        if (!result.hasNewData()) {
          Logger.recordOutput(cameraRoot + "/tagsUsedPositions", new Pose3d[] {});
          continue;
        }

        // Get Data
        TimestampedRobotPoseEstimate visionEstimate =
            new TimestampedRobotPoseEstimate(
                result.estimatedRobotPose(),
                result.timestampSecondFPGA(),
                result.standardDeviation(),
                result.status());

        hasVisionEstimate = hasVisionEstimate || visionEstimate.isSuccess();

        // Logging
        if (DO_CAMERA_LOGGING) {
          Logger.recordOutput(cameraRoot + "/tagsUsedPositions", result.tagPositionsOnField());

          Logger.recordOutput(cameraRoot + "/positionEstimate", visionEstimate.robotPose());

          Logger.recordOutput(cameraRoot + "/status", visionEstimate.status);
          Logger.recordOutput(cameraRoot + "/statusIsSuccess", visionEstimate.status.isSuccess());
        }

        if (DO_SUMMARY_LOGGING) {
          if (visionEstimate.isSuccess()) {
            robotPosesAccepted.add(visionEstimate.robotPose());
          } else {
            robotPosesRejected.add(visionEstimate.robotPose());
          }
          tagPoses.addAll(Arrays.asList(result.tagPositionsOnField()));
        }

        for (Consumer<TimestampedRobotPoseEstimate> consumer :
            timestampRobotPoseEstimateConsumers) {
          consumer.accept(visionEstimate);
        }
      }
    }

    if (DO_SUMMARY_LOGGING) {
      Logger.recordOutput(root + "/robotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
      Logger.recordOutput(root + "/robotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
      Logger.recordOutput(root + "/tagPoses", tagPoses.toArray(Pose3d[]::new));
    }
  }

  /** Get whether or not the vision system has a valid estimate */
  public boolean hasVisionEstimate() {
    return hasVisionEstimate;
  }

  /**
   * Set the last robot pose supplier for all cameras
   *
   * @param lastRobotPose the last robot pose supplier
   */
  public void setLastRobotPoseSupplier(Supplier<Pose2d> lastRobotPose) {
    for (Camera camera : cameras) {
      camera.setLastRobotPoseSupplier(lastRobotPose);
    }
  }

  /**
   * Add a consumer for the vision estimate
   *
   * @param timestampRobotPoseEstimateConsumer the consumer for the vision estimate
   */
  public void addVisionEstimateConsumer(
      Consumer<TimestampedRobotPoseEstimate> timestampRobotPoseEstimateConsumer) {
    timestampRobotPoseEstimateConsumers.add(timestampRobotPoseEstimateConsumer);
  }

  @Override
  public String toString() {
    return String.format(
        "%s(%s)",
        getClass().getName(),
        Arrays.stream(cameras).map(Camera::getCameraName).collect(Collectors.joining(", ")));
  }

  public record TimestampedRobotPoseEstimate(
      Pose3d robotPose,
      double timestampSeconds,
      Matrix<N3, N1> standardDeviations,
      VisionResultStatus status) {

    public Pose2d robotPose2d() {
      return robotPose.toPose2d();
    }

    public boolean isSuccess() {
      return status.isSuccess();
    }
  }
}
