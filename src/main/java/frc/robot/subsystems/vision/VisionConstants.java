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
              new Translation3d(Units.inchesToMeters(27.5 / 2.0 + 1.0), 0, Units.inchesToMeters(6)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));

  public static final CameraConfig COMP_FRONT_LEFT_CAMERA =
      new CameraConfig(
          "frontLeftCam",
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters((27.5 - 2.5) / 2.0 - 3.0),
                  Units.inchesToMeters((27.5 - 2.5) / 2.0),
                  Units.inchesToMeters(9)),
              new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-30))));
  public static final CameraConfig COMP_FRONT_RIGHT_CAMERA =
      new CameraConfig(
          "frontRightCam",
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters((27.5 - 3) / 2.0 - 3.0),
                  -Units.inchesToMeters((27.5 - 3) / 2.0),
                  Units.inchesToMeters(9)),
              new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(30))));
  public static final CameraConfig COMP_BACK_LEFT_CAMERA =
      new CameraConfig(
          "backLeftCam",
          new Transform3d(
              new Translation3d(
                  -Units.inchesToMeters((27.5 - 3) / 2.0),
                  Units.inchesToMeters((27.5 - 3) / 2.0),
                  Units.inchesToMeters(9)),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(135))));
  public static final CameraConfig COMP_BACK_RIGHT_CAMERA =
      new CameraConfig(
          "backRightCam",
          new Transform3d(
              new Translation3d(
                  -Units.inchesToMeters((27.5 - 3) / 2.0),
                  -Units.inchesToMeters((27.5 - 3) / 2.0),
                  Units.inchesToMeters(9)),
              new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-135))));

  public static final CameraConfig WOODV2_LEFT_CAMERA =
      new CameraConfig(
          "leftCamera",
          new Transform3d(
              new Translation3d(0, Units.inchesToMeters(27.5 / 2.0 - 0.5), 0),
              new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(90))));

  public static final CameraConfig WOODV2_RIGHT_CAMERA =
      new CameraConfig(
          "rightCamera",
          new Transform3d(
              new Translation3d(
                  0, -Units.inchesToMeters(27.5 / 2.0 + 1.0), Units.inchesToMeters(3)),
              new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(-90))));

  public static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
