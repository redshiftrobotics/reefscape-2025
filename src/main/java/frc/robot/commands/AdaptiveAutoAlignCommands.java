package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

public class AdaptiveAutoAlignCommands {
  private final List<Pose2d> poses;

  public AdaptiveAutoAlignCommands(List<Pose2d> poses) {
    Transform2d offset =
        new Transform2d(DRIVE_CONFIG.bumperCornerToCorner().getX() / 1, 0, Rotation2d.kPi);
    this.poses = poses.stream().map(pose -> pose.transformBy(offset)).toList();
  }

  private int getClosestPoseIndex(Drive drive) {
    Pose2d currentPose = drive.getPose();
    return IntStream.range(0, poses.size())
        .boxed()
        .min(Comparator.comparingDouble(i -> closestScore(poses.get(i), currentPose)))
        .get();
  }

  private double closestScore(Pose2d pose1, Pose2d pose2) {
    Transform2d transform = pose1.minus(pose2);
    return transform.getTranslation().getNorm()
        + transform.getRotation().getRadians() * DRIVE_CONFIG.driveBaseRadius();
  }

  private Command driveToClosest(Drive drive, int offset) {
    return Commands.defer(
        () -> {
          Pose2d targetPose =
              poses.get((getClosestPoseIndex(drive) + offset + poses.size()) % poses.size());
          return Commands.none();
        },
        Set.of(drive));
  }

  public Command driveToClosest(Drive drive) {
    return driveToClosest(drive, 0);
  }

  public Command driveToNext(Drive drive) {
    return driveToClosest(drive, +1);
  }

  public Command driveToPrevious(Drive drive) {
    return driveToClosest(drive, -1);
  }
}
