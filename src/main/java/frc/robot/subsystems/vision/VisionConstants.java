package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // --- Vision Config ---

  public record CameraConfig(String cameraName, Transform3d robotToCamera) {}

  // Set cameraName on PhotonVision web interface. Edit camera name from camera type to camera
  // position. To find robotToCamera, measure the distance from the camera to the center of the
  // robot or use the robot's CAD model.

  // Docs: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html

  public static final CameraConfig FRONT_CAMERA =
      new CameraConfig(
          "frontCam",
          new Transform3d(
              new Translation3d(0, 0, 0.1), new Rotation3d(0, Units.degreesToRadians(0), 0)));

  public static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
