package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Camera.AbsoluteTrackedTarget;
import frc.robot.subsystems.vision.Camera.RelativeTrackedTarget;
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
  private final Camera[] cameras;

  private final Debouncer debouncer = new Debouncer(0.2, DebounceType.kFalling);

  private List<Consumer<TimestampedRobotPoseEstimate>> timestampRobotPoseEstimateConsumers =
      new ArrayList<>();

  private boolean hasVisionEstimate = false;

  private AprilTagFieldLayout fieldTags = null;
  private List<SimControlledTarget> simulatedTargets = new ArrayList<>();

  public AprilTagVision(CameraIO... camerasIO) {
    this.cameras = Arrays.stream(camerasIO).map(io -> new Camera(io)).toArray(Camera[]::new);
  }

  @Override
  public void periodic() {

    AprilTagFieldLayout fieldTags = this.fieldTags;
    for (SimControlledTarget target : simulatedTargets) {
      fieldTags = target.createFieldWithTarget(fieldTags);
    }
    for (Camera camera : cameras) {
      camera.setFieldTags(fieldTags);
    }

    // Run periodic for all cameras, as they are not real subsystems
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

      Logger.recordOutput(cameraRoot + "/tagsUsedPositions", new Pose3d[] {});
      Logger.recordOutput(cameraRoot + "/positionEstimate", new Pose3d[] {});
      Logger.recordOutput(cameraRoot + "/status", VisionResultStatus.NO_DATA);
      Logger.recordOutput(cameraRoot + "/success", false);

      // Loop through all results that the camera has
      for (VisionResult result : camera.getResults()) {

        if (!result.hasNewData()) {
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

        Logger.recordOutput(cameraRoot + "/tagsUsedPositions", result.tagPositionsOnField());
        Logger.recordOutput(cameraRoot + "/positionEstimate", visionEstimate.robotPose());
        Logger.recordOutput(cameraRoot + "/status", visionEstimate.status());
        Logger.recordOutput(cameraRoot + "/success", visionEstimate.status().isSuccess());

        if (visionEstimate.isSuccess()) {
          robotPosesAccepted.add(visionEstimate.robotPose());
        } else {
          robotPosesRejected.add(visionEstimate.robotPose());
        }
        tagPoses.addAll(Arrays.asList(result.tagPositionsOnField()));

        for (Consumer<TimestampedRobotPoseEstimate> consumer :
            timestampRobotPoseEstimateConsumers) {
          consumer.accept(visionEstimate);
        }
      }
    }

    Logger.recordOutput(root + "/robotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(root + "/robotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
    Logger.recordOutput(root + "/tagPoses", tagPoses.toArray(Pose3d[]::new));
  }

  public List<RelativeTrackedTarget> getRelativeTrackedTargets() {
    return Arrays.stream(cameras).flatMap(camera -> camera.getRelativeTargets().stream()).toList();
  }

  public List<AbsoluteTrackedTarget> getAbsoluteTrackedTargets() {
    return Arrays.stream(cameras).flatMap(camera -> camera.getAbsoluteTargets().stream()).toList();
  }

  public void setFieldTags(AprilTagFieldLayout fieldTags) {
    this.fieldTags = fieldTags;
  }

  public void addSimulatedTarget(SimControlledTarget target) {
    simulatedTargets.add(target);
  }

  /** Get whether or not the vision system has a valid estimate */
  public boolean hasVisionEstimate() {
    return hasVisionEstimate;
  }

  public boolean hasStableVisionEstimate() {
    return debouncer.calculate(hasVisionEstimate);
  }

  /**
   * Set the last robot pose supplier for all cameras
   *
   * @param filter if robot pose filtering should be applied
   * @param lastRobotPose the last robot pose supplier
   */
  public void filterBasedOnLastPose(boolean filter, Supplier<Pose2d> lastRobotPose) {
    for (Camera camera : cameras) {
      camera.filterBasedOnLastPose(filter, lastRobotPose);
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
