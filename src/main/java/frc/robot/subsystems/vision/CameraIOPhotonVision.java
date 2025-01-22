package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.Optional;
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

  public CameraIOPhotonVision(CameraConfig config) {

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
            VisionConstants.FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void setAprilTagFieldLayout(AprilTagFieldLayout fieldTags) {
    photonPoseEstimator.setFieldTags(fieldTags);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> estimatedRobotPoseOptional = photonPoseEstimator.update(result);

      if (estimatedRobotPoseOptional.isPresent()) {
        inputs.hasNewData = true;

        EstimatedRobotPose estimateRobotPose = estimatedRobotPoseOptional.get();

        inputs.timestampSecondsFPGA = estimateRobotPose.timestampSeconds;
        inputs.estimatedRobotPose = estimateRobotPose.estimatedPose;
        inputs.tagsUsed =
            estimateRobotPose.targetsUsed.stream()
                .mapToInt(PhotonTrackedTarget::getFiducialId)
                .toArray();
      } else {
        inputs.hasNewData = false;
      }
    }

    inputs.connected = camera.isConnected();
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }
}
