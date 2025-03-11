package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.subsystems.drive.Drive;

public class ManualAlignCommands {
  public static Command alignToCage(Drive drivetrain) {
    return alignToRotation(drivetrain, Rotation2d.kZero);
  }

  public static Command alignToSourceLeft(Drive drivetrain) {
    return alignToRotation(
        drivetrain, CoralStation.leftCenterFace.getRotation().minus(allianceRotation()));
  }

  public static Command alignToSourceRight(Drive drivetrain) {
    return alignToRotation(
        drivetrain, CoralStation.rightCenterFace.getRotation().minus(allianceRotation()));
  }

  private static Rotation2d allianceRotation() {
    Alliance alliance =
        DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;

    return Rotation2d.fromDegrees(alliance == Alliance.Blue ? 0 : 180);
  }

  private static Command alignToRotation(Drive drivetrain, Rotation2d rotation) {
    return new DriveToPose(
            drivetrain,
            () ->
                new Pose2d(
                    drivetrain.getRobotPose().getX(), drivetrain.getRobotPose().getY(), rotation))
        .until(
            () ->
                MathUtil.isNear(
                    rotation.getDegrees(),
                    drivetrain.getRobotPose().getRotation().getDegrees(),
                    2));
  }
}
