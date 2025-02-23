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

  private final Transform2d mechanismOffset;
  private final Transform2d roughLineupOffset;

  private int offset = 0;

  private Command startCommand, finalAlignCommand, endCommand;

  public AdaptiveAutoAlignCommands(
      List<Pose2d> poses,
      Transform2d positionOffset,
      Transform2d mechanismOffset,
      Translation2d roughLineupOffset) {

    this.mechanismOffset = mechanismOffset;
    this.roughLineupOffset = new Transform2d(roughLineupOffset, Rotation2d.kZero);

    final Transform2d robotOffset =
        switch (Constants.getRobot()) {
          case WOOD_BOT_TWO_2025 -> new Transform2d(Translation2d.kZero, Rotation2d.kCW_90deg);
          default -> Transform2d.kZero;
        };

    this.poses =
        poses.stream()
            .map(pose -> pose.transformBy(robotOffset).transformBy(positionOffset))
            .toList();

    setStartCommand(Commands.none());
    setFinalAlignCommand(Commands.none());
    setEndCommand(Commands.none());
  }

  public void setStartCommand(Command startCommand) {
    this.startCommand = startCommand;
  }

  public void setFinalAlignCommand(Command finalAlignCommand) {
    this.finalAlignCommand = finalAlignCommand;
  }

  public void setEndCommand(Command endCommand) {
    this.endCommand = endCommand;
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
          Pose2d pose = getPose(offset);
          return startCommand
              .andThen(
                  DriveCommands.pathfindToPoseCommand(
                      drive, pose.plus(roughLineupOffset.inverse()).plus(mechanismOffset), 0.25, 0))
              .andThen(Commands.runOnce(drive::stop))
              .andThen(finalAlignCommand)
              .andThen(DriveCommands.driveToPosePrecise(drive, pose.plus(mechanismOffset)))
              .andThen(endCommand)
              .beforeStarting(() -> setCurrentAutoAlignGoal(pose))
              .finallyDo(() -> setCurrentAutoAlignGoal(null));
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
    return drive.runOnce(drive::stop);
  }
}
