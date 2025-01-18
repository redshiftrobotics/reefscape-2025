package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.Supplier;

/** Pathfind to a pose with pathplanner, only gets you roughly to target pose. */
public class PathfindToPoseCommand extends DeferredCommand {

  /**
   * Pathfind to a pose with pathplanner, only gets you roughly to target pose.
   *
   * @param drive the drive subsystem
   * @param desiredPoseSupplier the target pose
   * @param speedMultiplier the speed multiplier, 1 would be full speed
   * @param goalEndVelocity the goal end velocity, 0 would be a full stop
   */
  public PathfindToPoseCommand(
      Drive drive,
      Supplier<Pose2d> desiredPoseSupplier,
      double speedMultiplier,
      double goalEndVelocity) {
    super(
        () ->
            AutoBuilder.pathfindToPose(
                desiredPoseSupplier.get(), DRIVE_CONFIG.getPathConstraints(), goalEndVelocity),
        Set.of(drive));
  }
}
