package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.commands.controllers.JoystickInputController;
import frc.robot.commands.controllers.SpeedLevelController.SpeedLevel;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class ManualAlignCommands {
  public static Command alignToCage(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(Rotation2d.kZero),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  public static Command alignToSourceLeft(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(CoralStation.leftCenterFace.getRotation().minus(allianceRotation())),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  public static Command alignToSourceRight(Drive drivetrain, JoystickInputController input) {
    return DriveCommands.joystickHeadingDrive(
        drivetrain,
        input::getTranslationMetersPerSecond,
        () -> Optional.of(CoralStation.rightCenterFace.getRotation().minus(allianceRotation())),
        () -> SpeedLevel.NO_LEVEL,
        () -> true);
  }

  private static Rotation2d allianceRotation() {
    boolean isBlue =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue;

    return Rotation2d.fromDegrees(isBlue ? 0 : 180);
  }
}
