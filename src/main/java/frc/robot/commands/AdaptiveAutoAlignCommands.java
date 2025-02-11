package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class AdaptiveAutoAlignCommands {
  private final List<Pose2d> poses;

  private int offset = 0;

  public AdaptiveAutoAlignCommands(List<Pose2d> poses, Transform2d offset) {
    this.poses = poses.stream().map(pose -> pose.transformBy(offset)).toList();
  }

  private Pose2d getPose(int index) {
    return AllianceFlipUtil.apply(poses.get(index));
  }

  private int getClosestPoseIndex(Drive drive) {
    Pose2d currentPose = drive.getRobotPose();
    return IntStream.range(0, poses.size())
        .boxed()
        .min(Comparator.comparingDouble(i -> closestScore(getPose(i), currentPose)))
        .get();
  }

  private double closestScore(Pose2d pose1, Pose2d pose2) {
    Transform2d transform = pose1.minus(pose2);
    return Math.abs(transform.getTranslation().getNorm())
        + Math.abs(transform.getRotation().getRadians()) * DRIVE_CONFIG.driveBaseRadius();
  }

  private Command align(Drive drive) {
    return Commands.defer(
        () -> {
          Pose2d pose = getPose(offset);
          return DriveCommands.pathfindToPoseCommand(drive, pose, 0.25, 0)
              .andThen(DriveCommands.driveToPosePrecise(drive, pose))
              .beforeStarting(() -> Logger.recordOutput("AutoAlignGoal", new Pose2d[] {pose}))
              .finallyDo(() -> Logger.recordOutput("AutoAlignGoal", new Pose2d[] {}));
        },
        Set.of(drive));
  }

  public Command driveToClosest(Drive drive) {
    return align(drive).beforeStarting(() -> this.offset = getClosestPoseIndex(drive));
  }

  public Command driveToNext(Drive drive) {
    return align(drive)
        .beforeStarting(() -> this.offset = (this.offset + 1 + poses.size()) % poses.size());
  }

  public Command driveToPrevious(Drive drive) {
    return align(drive)
        .beforeStarting(() -> this.offset = (this.offset - 1 + poses.size()) % poses.size());
  }

  public Command stop(Drive drive) {
    return Commands.run(drive::stop);
  }
}
