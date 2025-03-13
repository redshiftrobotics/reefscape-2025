package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.Barge;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.controllers.JoystickInputController;
import frc.robot.commands.controllers.SpeedLevelController.SpeedLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

public class ManualAlignCommands {

  public static Command alignToCage(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(allianceRotation().plus(Rotation2d.k180deg)),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  public static Command alignToCageAdv(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> {
          Translation2d robotPose = drivetrain.getRobotPose().getTranslation();

          List<Translation2d> cagePose =
              Stream.of(Barge.farCage, Barge.middleCage, Barge.closeCage)
                  .map(AllianceFlipUtil::apply)
                  .toList();

          return Optional.of(
              robotPose.minus(robotPose.nearest(cagePose)).getAngle().plus(allianceRotation()));
        },
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

          double angleRad = robotPose.minus(reefPose).getAngle().getRadians();

          for (Pose2d face : Reef.alignmentFaces) {
            double faceRad = face.getRotation().getRadians();

            if (MathUtil.isNear(faceRad, angleRad, Units.degreesToRadians(25))) {
              angleRad = faceRad;
            }
          }

          return Optional.of(AllianceFlipUtil.apply(Rotation2d.fromRadians(angleRad)));
        },
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  private static Rotation2d allianceRotation() {
    return AllianceFlipUtil.shouldFlip() ? Rotation2d.kZero : Rotation2d.k180deg;
  }
}
