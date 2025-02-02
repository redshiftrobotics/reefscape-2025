package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
  private final Transform3d robotToCamera;

  public CameraIOPhotonVision(CameraConfig config) {

    // --- Setup Camera ---
    camera = new PhotonCamera(config.cameraName());

    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);

    robotToCamera = config.robotToCamera();

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

  protected PhotonCamera getCamera() {
    return camera;
  }

  protected PhotonPoseEstimator getPhotonPoseEstimator() {
    return photonPoseEstimator;
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {

    List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

    Pose3d[] estimatedRobotPose = new Pose3d[pipelineResults.size()];
    double[] timestampSecondFPGA = new double[pipelineResults.size()];
    int[][] tagsUsed = new int[pipelineResults.size()][];
    PhotonTrackedTarget[][] targets = new PhotonTrackedTarget[pipelineResults.size()][];
    boolean[] hasNewData = new boolean[pipelineResults.size()];

    inputs.updatesReceived = pipelineResults.size();

    for (int i = 0; i < pipelineResults.size(); i++) {
      Optional<EstimatedRobotPose> estimatedRobotPoseOptional =
          photonPoseEstimator.update(pipelineResults.get(i));

      if (estimatedRobotPoseOptional.isPresent()) {

        EstimatedRobotPose estimateRobotPose = estimatedRobotPoseOptional.get();

        estimatedRobotPose[i] = estimateRobotPose.estimatedPose;
        timestampSecondFPGA[i] = estimateRobotPose.timestampSeconds;
        targets[i] = (PhotonTrackedTarget[]) estimateRobotPose.targetsUsed.toArray();
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
        targets[i] = new PhotonTrackedTarget[0];
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
  public String getCameraName() {
    return camera.getName();
  }
  @Override
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }
}
