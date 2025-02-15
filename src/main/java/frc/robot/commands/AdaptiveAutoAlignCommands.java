package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class AdaptiveAutoAlignCommands {

  private static Pose2d currentAutoAlignGoal = null;

  private final List<Pose2d> poses;

  private final Transform2d robotVisionSystemOffset;

  private final Transform2d mechanismOffset;
  private final Transform2d roughLineupOffset;

  private int poseIndex = 0;

  public AdaptiveAutoAlignCommands(
      List<Pose2d> poses,
      Transform2d positionOffset,
      Transform2d mechanismOffset,
      Translation2d roughLineupOffset) {

    this.mechanismOffset = mechanismOffset;
    this.roughLineupOffset = new Transform2d(roughLineupOffset, Rotation2d.kZero);

    robotVisionSystemOffset =
        switch (Constants.getRobot()) {
          case WOOD_BOT_TWO_2025 -> new Transform2d(Translation2d.kZero, Rotation2d.kCW_90deg);
          default -> Transform2d.kZero;
        };

    this.poses = poses.stream().map(pose -> pose.transformBy(positionOffset)).toList();
  }

  public static Pose2d getCurrentAutoAlignGoal() {
    return currentAutoAlignGoal;
  }

  private static void setCurrentAutoAlignGoal(Pose2d pose) {
    currentAutoAlignGoal = pose;
    Logger.recordOutput("AutoAlignGoal", pose == null ? new Pose2d[] {} : new Pose2d[] {pose});
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
          Pose2d pose = getPose(0);
          return DriveCommands.pathfindToPoseCommand(
                  drive,
                  pose.plus(roughLineupOffset.inverse())
                      .plus(mechanismOffset)
                      .plus(robotVisionSystemOffset),
                  1,
                  1)
              .andThen(Commands.runOnce(drive::stop))
              .andThen(
                  DriveCommands.driveToPosePrecise(
                      drive, pose.plus(mechanismOffset).plus(robotVisionSystemOffset)))
              .beforeStarting(() -> setCurrentAutoAlignGoal(pose))
              .finallyDo(() -> setCurrentAutoAlignGoal(null));
        },
        Set.of(drive));
  }

  public Command driveToClosest(Drive drive) {
    return align(drive).beforeStarting(() -> this.poseIndex = getClosestPoseIndex(drive));
  }

  public Command driveToNext(Drive drive) {
    return align(drive)
        .beforeStarting(() -> this.poseIndex = (this.poseIndex + 1 + poses.size()) % poses.size());
  }

  public Command driveToPrevious(Drive drive) {
    return align(drive)
        .beforeStarting(() -> this.poseIndex = (this.poseIndex - 1 + poses.size()) % poses.size());
  }

  public Command stop(Drive drive) {
    return drive.runOnce(drive::stop);
  }
}
