package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.controllers.JoystickInputController;
import frc.robot.commands.controllers.SpeedLevelController.SpeedLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.util.Optional;

public class ManualAlignCommands {
  public static Command alignToCage(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(AllianceFlipUtil.apply(Rotation2d.kZero)),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  public static Command alignToSourceLeft(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(AllianceFlipUtil.apply(CoralStation.leftCenterFace.getRotation())),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  public static Command alignToSourceRight(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(AllianceFlipUtil.apply(CoralStation.rightCenterFace.getRotation())),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  public static Command alignToReef(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> {
          Translation2d robotPose = drivetrain.getRobotPose().getTranslation();
          Translation2d reefPose = AllianceFlipUtil.apply(Reef.center);

          double dx = robotPose.getX() - reefPose.getX();
          double dy = robotPose.getY() - reefPose.getY();

          double angleRad = Math.atan2(dy, dx);

          for (Pose2d face : Reef.alignmentFaces) {
            double faceRad = face.getRotation().getRadians();

            if (MathUtil.isNear(faceRad, angleRad, 0.5)) {
              angleRad = faceRad;
            }
          }

          return Optional.of(Rotation2d.fromRadians(angleRad));
        },
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }
}
