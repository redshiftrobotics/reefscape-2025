package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureUtil {
  public record SuperstructureState(double elevatorHeight, Rotation2d wristAngle) {}

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

  public static SuperstructureState getSuperstructureState(Pose3d tagPose) {
    final Rotation2d wristAngle = new Rotation2d(tagPose.getRotation().getY());
    final double elevatorHeight =
        tagPose.getTranslation().getZ()
            - WRIST_LENGTH * wristAngle.getSin()
            - ELEVATOR_HEIGHT_OFF_GROUND;
    return new SuperstructureState(elevatorHeight, wristAngle);
  }
}
