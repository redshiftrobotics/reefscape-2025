package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Camera.AbsoluteTrackedTarget;
import frc.robot.subsystems.vision.Camera.RelativeTrackedTarget;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraIOPhotonVision implements CameraIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;

  private final CameraConfig config;

  private List<PhotonTrackedTarget> latestTargets = new ArrayList<>();

  private Supplier<Pose2d> lastPoseSupplier;

  public CameraIOPhotonVision(CameraConfig config) {
    this.config = config;

    // --- Setup Camera ---
    camera = new PhotonCamera(config.cameraName());

    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);

    // --- Setup Pose Estimator ---

    // MULTI_TAG_PNP_ON_COPROCESSOR:
    // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#enabling-multitag

    // PhotonPoseEstimator:
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#using-a-photonposeestimator

    photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.DEFAULT_FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  protected PhotonCamera getCamera() {
    return camera;
  }

  protected PhotonPoseEstimator getPhotonPoseEstimator() {
    return photonPoseEstimator;
  }

  @Override
  public void setLastPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.lastPoseSupplier = robotPoseSupplier;
  }

  @Override
  public CameraConfig getCameraConfig() {
    return config;
  }

  @Override
  public void setAprilTagFieldLayout(AprilTagFieldLayout fieldTags) {
    photonPoseEstimator.setFieldTags(fieldTags);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {

    List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

    Pose3d[] estimatedRobotPose = new Pose3d[pipelineResults.size()];
    double[] timestampSecondFPGA = new double[pipelineResults.size()];
    int[][] tagsUsed = new int[pipelineResults.size()][];
    boolean[] hasNewData = new boolean[pipelineResults.size()];

    inputs.updatesReceived = pipelineResults.size();

    if (lastPoseSupplier != null) {
      photonPoseEstimator.setLastPose(lastPoseSupplier.get());
    }

    this.latestTargets.clear();

    for (int i = 0; i < pipelineResults.size(); i++) {
      PhotonPipelineResult cameraResult = pipelineResults.get(i);
      Optional<EstimatedRobotPose> estimatedRobotPoseOptional =
          photonPoseEstimator.update(cameraResult);

      this.latestTargets.addAll(cameraResult.getTargets());

      if (estimatedRobotPoseOptional.isPresent()) {

        EstimatedRobotPose estimateRobotPose = estimatedRobotPoseOptional.get();

        estimatedRobotPose[i] = estimateRobotPose.estimatedPose;
        timestampSecondFPGA[i] = estimateRobotPose.timestampSeconds;
        tagsUsed[i] =
            estimateRobotPose.targetsUsed.stream()
                .map(PhotonTrackedTarget::getFiducialId)
                .mapToInt(Integer::intValue)
                .toArray();
        hasNewData[i] = true;
      } else {
        estimatedRobotPose[i] = Pose3d.kZero;
        timestampSecondFPGA[i] = 0;
        tagsUsed[i] = new int[0];
        hasNewData[i] = false;
      }
    }

    inputs.estimatedRobotPose = estimatedRobotPose;
    inputs.timestampSecondFPGA = timestampSecondFPGA;
    inputs.tagsUsed = tagsUsed;
    inputs.hasNewData = hasNewData;
    inputs.connected = camera.isConnected();
  }

  @Override
  public List<RelativeTrackedTarget> getRelativeTargets() {
    return latestTargets.stream()
        .map(
            t ->
                new RelativeTrackedTarget(
                    t.getFiducialId(), t.getBestCameraToTarget(), config, t.getPoseAmbiguity()))
        .toList();
  }

  @Override
  public List<AbsoluteTrackedTarget> getAbsoluteTargets() {
    if (lastPoseSupplier == null) {
      throw new IllegalStateException(
          "Robot pose supplier not set for absolute target calculation");
    }
    return getRelativeTargets().stream()
        .map(r -> new AbsoluteTrackedTarget(r, lastPoseSupplier.get()))
        .toList();
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }
}
