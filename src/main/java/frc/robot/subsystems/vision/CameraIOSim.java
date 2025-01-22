package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraIOSim implements CameraIO {

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;
  private final PhotonPoseEstimator photonPoseEstimator;

  private final Supplier<Pose2d> robotPoseSupplier;

  public CameraIOSim(CameraConfig config, Supplier<Pose2d> robotPoseSupplier) {

    this.robotPoseSupplier = robotPoseSupplier;

    // --- Camera Props ---

    SimCameraProperties cameraProperties = new SimCameraProperties();

    // https://www.uctronics.com/download/Amazon/B0332_OV9281_Global_Shutter_UVC_Camera_Datasheet.pdf
    // These values depend on photonvision config, update them as well as in assets config
    cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(70));
    cameraProperties.setCalibError(0.01, 0.10);
    cameraProperties.setFPS(15);

    cameraProperties.setAvgLatencyMs(25);
    cameraProperties.setLatencyStdDevMs(10);

    // --- Sim Camera ---

    camera = new PhotonCamera(config.cameraName());
    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    cameraSim.enableDrawWireframe(true);
    cameraSim.enableProcessedStream(true);

    // --- Vision Sim ---
    visionSim = new VisionSystemSim(config.cameraName());
    visionSim.addCamera(cameraSim, config.robotToCamera());

    // --- Pose Estimator ---
    photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {

    Pose2d robotPose = robotPoseSupplier.get();
    visionSim.update(robotPose);

    List<PhotonPipelineResult> result = camera.getAllUnreadResults();

    if (result.isEmpty()) {
      inputs.hasNewData = false;
      inputs.connected = camera.isConnected();
      return;
    }

    Optional<EstimatedRobotPose> estimatedRobotPoseOptional =
        photonPoseEstimator.update(result.get(0));

    if (estimatedRobotPoseOptional.isPresent()) {
      EstimatedRobotPose estimateRobotPose = estimatedRobotPoseOptional.get();

      inputs.hasNewData = true;
      inputs.timestampSecondsFPGA = estimateRobotPose.timestampSeconds;
      inputs.estimatedRobotPose = estimateRobotPose.estimatedPose;
      inputs.tagsUsed =
          estimateRobotPose.targetsUsed.stream()
              .mapToInt(PhotonTrackedTarget::getFiducialId)
              .toArray();
    } else {
      inputs.hasNewData = false;
    }

    inputs.connected = camera.isConnected();
  }

  @Override
  public void setAprilTagFieldLayout(AprilTagFieldLayout layout) {

    visionSim.clearAprilTags();
    visionSim.addAprilTags(layout);

    photonPoseEstimator.setFieldTags(layout);
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }
}
