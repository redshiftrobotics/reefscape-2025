package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class CameraIOSim extends CameraIOPhotonVision {

  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  private final Supplier<Pose2d> robotPoseSupplier;

  public CameraIOSim(CameraConfig config, Supplier<Pose2d> robotPoseSupplier) {
    super(config);

    this.robotPoseSupplier = robotPoseSupplier;

    // --- Camera Props ---

    SimCameraProperties cameraProperties = new SimCameraProperties();

    // These values depend on photonvision config, update them as well as in assets config
    cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(100));
    cameraProperties.setCalibError(0.01, 0.10);
    cameraProperties.setFPS(15);

    cameraProperties.setAvgLatencyMs(25);
    cameraProperties.setLatencyStdDevMs(10);

    // --- Sim Camera ---

    cameraSim = new PhotonCameraSim(getCamera(), cameraProperties);

    cameraSim.enableDrawWireframe(true);
    cameraSim.enableProcessedStream(true);

    // --- Vision Sim ---
    visionSim = new VisionSystemSim(config.cameraName());
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    Pose2d robotPose = robotPoseSupplier.get();
    visionSim.update(robotPose);
    super.updateInputs(inputs);
  }

  @Override
  public void setAprilTagFieldLayout(AprilTagFieldLayout layout) {
    super.setAprilTagFieldLayout(layout);

    visionSim.clearAprilTags();
    visionSim.addAprilTags(layout);
  }
}
