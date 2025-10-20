package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.wrist.WristConstants;
import java.util.List;

public class TagFollowUtil {

  public static final int mode = 1;

  public static final double ELEVATOR_HEIGHT_OFF_GROUND =
      switch (mode) {
        case 0 -> ElevatorConstants.elevatorDistanceFromGround + ElevatorConstants.carriageHeight;
        case 1 -> ElevatorConstants.elevatorDistanceFromGround
            + SuperstructureVisualizer.ROOT_Y
            + ElevatorConstants.carriageHeight;
        default -> Units.inchesToMeters(12);
      };

  public static final double WRIST_LENGTH =
      switch (mode) {
        case 0, 1 -> WristConstants.WRIST_LENGTH;
        default -> Units.inchesToMeters(18);
      };

  public record TagFollowingSuperstructureState(double elevatorHeight, Rotation2d wristAngle) {}

  public static Pose3d averagePoses(List<Pose3d> poses) {
    double x = 0;
    double y = 0;
    double z = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    for (Pose3d pose : poses) {
      x += pose.getX();
      y += pose.getY();
      z += pose.getZ();
      roll += pose.getRotation().getX();
      pitch += pose.getRotation().getY();
      yaw += pose.getRotation().getZ();
    }

    int count = poses.size();
    return new Pose3d(
        x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
  }
}
