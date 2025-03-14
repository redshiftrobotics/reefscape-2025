package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // --- Vision Config ---

  public static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Set cameraName on PhotonVision web interface. Edit camera name from camera type to camera
  // position. To find robotToCamera, measure the distance from the camera to the center of the
  // robot or use the robot's CAD model.

  // Docs: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html

  public record CameraConfig(String cameraName, String cameraPosition, Transform3d robotToCamera) {}

  public static final CameraConfig SIM_FRONT_CAMERA =
      new CameraConfig(
          "frontCam",
          "front",
          new Transform3d(
              new Translation3d(Units.inchesToMeters(27.5 / 2.0 + 1.0), 0, Units.inchesToMeters(6)),
              new Rotation3d(0, Units.degreesToRadians(0), 0)));

  public static final CameraConfig WOODV2_LEFT_CAMERA =
      new CameraConfig(
          "leftCamera",
          "left",
          new Transform3d(
              new Translation3d(0, Units.inchesToMeters(27.5 / 2.0 - 0.5), 0),
              new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(90))));

  public static final CameraConfig WOODV2_RIGHT_CAMERA =
      new CameraConfig(
          "rightCamera",
          "right",
          new Transform3d(
              new Translation3d(
                  0, -Units.inchesToMeters(27.5 / 2.0 + 1.0), Units.inchesToMeters(3)),
              new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(-90))));

  private static final double CAMERA_OFFSET_X = Units.inchesToMeters(9.922 + 0.6);
  private static final double CAMERA_OFFSET_Y = Units.inchesToMeters(9.906);

  private static final double FRONT_CAMERA_YAW = Units.degreesToRadians(45); // 45 degrees outward
  private static final double FRONT_CAMERA_PITCH =
      Units.degreesToRadians(20); // more bent back mount
  private static final double FRONT_CAMERA_OFFSET_Z = Units.inchesToMeters(8.0);

  private static final double BACK_CAMERA_OFFSET_Z = Units.inchesToMeters(8.0);
  private static final double BACK_CAMERA_PITCH =
      Units.degreesToRadians(11); // less bent back mount

  public static final CameraConfig COMP_FRONT_LEFT_CAMERA =
      new CameraConfig(
          "plzwork",
          "Front Left",
          new Transform3d(
              new Translation3d(+CAMERA_OFFSET_X, +CAMERA_OFFSET_Y, FRONT_CAMERA_OFFSET_Z),
              new Rotation3d(0, -FRONT_CAMERA_PITCH, +FRONT_CAMERA_YAW)));

  public static final CameraConfig COMP_FRONT_RIGHT_CAMERA =
      new CameraConfig(
          "FrontRight8032",
          "Front Right",
          new Transform3d(
              new Translation3d(+CAMERA_OFFSET_X, -CAMERA_OFFSET_Y, FRONT_CAMERA_OFFSET_Z),
              new Rotation3d(0, -FRONT_CAMERA_PITCH, -FRONT_CAMERA_YAW)));

  public static final CameraConfig COMP_BACK_LEFT_CAMERA =
      new CameraConfig(
          "jessiecam",
          "Back Left",
          new Transform3d(
              new Translation3d(-CAMERA_OFFSET_X, +CAMERA_OFFSET_Y, BACK_CAMERA_OFFSET_Z),
              new Rotation3d(0, -BACK_CAMERA_PITCH, Units.degreesToRadians(180 + 30))));

  public static final CameraConfig COMP_BACK_RIGHT_CAMERA =
      new CameraConfig(
          "BackRightCamera8032",
          "Back Right",
          new Transform3d(
              new Translation3d(-CAMERA_OFFSET_X, -CAMERA_OFFSET_Y, BACK_CAMERA_OFFSET_Z),
              new Rotation3d(0, -BACK_CAMERA_PITCH, Units.degreesToRadians(180 - 30))));
}
