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

  private int singleTagId = -1;

  private List<Consumer<TimeStampedRobotPoseEstimate>> visionEstimateConsumers = new ArrayList<>();

  private List<Consumer<TimeStampedRobotPoseEstimate>> visionSpecializedEstimateConsumers =
      new ArrayList<>();

  private boolean hasVisionEstimate = false;

  private boolean hasSpecializedVisionEstimate = false;

  public AprilTagVision(CameraIO... camerasIO) {
    this.cameras = Arrays.stream(camerasIO).map(io -> new Camera(io)).toArray(Camera[]::new);
  }

  @Override
  public void periodic() {

    // Run periodic for all cameras, as they are not real subsystems
    for (Camera camera : cameras) {
      camera.periodic();
    }

    String root = "Vision";

    List<Pose3d> robotPosesAccepted = new ArrayList<>();
    List<Pose3d> robotPosesRejected = new ArrayList<>();
    List<Pose3d> tagPoses = new ArrayList<>();
    List<Pose3d> specializedRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> specializedRobotPosesRejected = new ArrayList<>();

    hasVisionEstimate = false;
    hasSpecializedVisionEstimate = false;

    String specializedRoot = root + "/specialized";

    // Loop through all cameras
    for (Camera camera : cameras) {
      String cameraRoot = root + "/" + camera.getCameraName();

      if (DO_CAMERA_LOGGING && camera.getResults().length == 0) {
        recordBlankVisionEstimate(cameraRoot);
      }

      // Loop through all results that the camera has
      for (VisionResult result : camera.getResults()) {

        if (DO_CAMERA_LOGGING) {
          recordVisionEstimate(cameraRoot, result);
        }

        if (!result.hasNewData()) {
          continue;
        }

        // Get Data
        TimeStampedRobotPoseEstimate visionEstimate = new TimeStampedRobotPoseEstimate(result);

        hasVisionEstimate = hasVisionEstimate || visionEstimate.isSuccess();

        // Logging

        if (DO_SUMMARY_LOGGING) {
          if (visionEstimate.isSuccess()) {
            robotPosesAccepted.add(visionEstimate.robotPose());
          } else {
            robotPosesRejected.add(visionEstimate.robotPose());
          }
          tagPoses.addAll(Arrays.asList(result.tagPositionsOnField()));
        }

        for (Consumer<TimeStampedRobotPoseEstimate> consumer : visionEstimateConsumers) {
          consumer.accept(visionEstimate);
        }
      }

      // Specialized estimation
      if (singleTagId > 0 && singleTagId <= VisionConstants.AprilTagCount) {
        for (VisionResult result : camera.getSpecializedResults()) {
          if (DO_CAMERA_LOGGING) {
            recordVisionEstimate(specializedRoot, result);
          }

          if (!result.hasNewData()) {
            continue;
          }
          TimeStampedRobotPoseEstimate specializedVisionEstimate =
              new TimeStampedRobotPoseEstimate(result);
          hasSpecializedVisionEstimate =
              hasSpecializedVisionEstimate || specializedVisionEstimate.isSuccess();

          if (DO_SUMMARY_LOGGING) {
            if (specializedVisionEstimate.isSuccess()) {
              specializedRobotPosesAccepted.add(specializedVisionEstimate.robotPose);
            } else {
              specializedRobotPosesRejected.add(specializedVisionEstimate.robotPose);
            }
          }

          for (Consumer<TimeStampedRobotPoseEstimate> consumer :
              visionSpecializedEstimateConsumers) {
            consumer.accept(specializedVisionEstimate);
          }
        }
      }
    }

    if (DO_SUMMARY_LOGGING) {
      // Log results
      Logger.recordOutput(root + "/robotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
      Logger.recordOutput(root + "/robotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
      Logger.recordOutput(root + "/tagPoses", tagPoses.toArray(Pose3d[]::new));

      // Log specialized results
      if (singleTagId > 0 && singleTagId <= VisionConstants.AprilTagCount) {
        Logger.recordOutput(
            specializedRoot + "/robotPosesAccepted",
            specializedRobotPosesAccepted.toArray(Pose3d[]::new));
        Logger.recordOutput(
            specializedRoot + "/robotPosesRejected",
            specializedRobotPosesRejected.toArray(Pose3d[]::new));
        Logger.recordOutput(
            specializedRoot + "/tagPose", VisionConstants.FIELD.getTagPose(singleTagId).get());
      }
    }
  }

  /** Get whether or not the vision system has a valid estimate */
  public boolean hasVisionEstimate() {
    return hasVisionEstimate;
  }

  public boolean hasSpecializedVisionEstimate() {
    return hasSpecializedVisionEstimate;
  }

  public void setSingleTagEstimateId(int fiducialId) {
    singleTagId = fiducialId;
    for (Camera camera : cameras) {
      camera.setSingleTagId(fiducialId);
    }
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
   * @param timeStampedRobotPoseEstimateConsumer the consumer for the vision estimate
   */
  public void addVisionEstimateConsumer(
      Consumer<TimeStampedRobotPoseEstimate> timeStampedRobotPoseEstimateConsumer) {
    visionEstimateConsumers.add(timeStampedRobotPoseEstimateConsumer);
  }

  public void addSpecializedVisionEstimateConsumer(
      Consumer<TimeStampedRobotPoseEstimate> estimateConsumer) {}

  @Override
  public String toString() {
    return String.format(
        "%s(%s)",
        getClass().getName(),
        Arrays.stream(cameras).map(Camera::getCameraName).collect(Collectors.joining(", ")));
  }

  private void recordBlankVisionEstimate(String cameraRoot) {
    Logger.recordOutput(cameraRoot + "/tagsUsedPositions", new Pose3d[] {});
    Logger.recordOutput(cameraRoot + "/positionEstimate", new Pose3d[] {});
    Logger.recordOutput(cameraRoot + "/status", VisionResultStatus.NO_DATA);
    Logger.recordOutput(cameraRoot + "/success", false);
  }

  private void recordVisionEstimate(String cameraRoot, VisionResult result) {
    Logger.recordOutput(cameraRoot + "/tagsUsedPositions", result.tagPositionsOnField());
    Logger.recordOutput(cameraRoot + "/positionEstimate", result.estimatedRobotPose());
    Logger.recordOutput(cameraRoot + "/status", result.status());
    Logger.recordOutput(cameraRoot + "/success", result.status().isSuccess());
  }

  public record TimeStampedRobotPoseEstimate(
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

    public TimeStampedRobotPoseEstimate(VisionResult result) {
      this(
          result.estimatedRobotPose(),
          result.timestampSecondFPGA(),
          result.standardDeviation(),
          result.status());
    }
  }
}
