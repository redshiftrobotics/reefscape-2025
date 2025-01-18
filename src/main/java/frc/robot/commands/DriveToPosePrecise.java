package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPosePrecise extends Command {

  private final Drive drive;
  private final Supplier<Pose2d> desiredPoseSupplier;

  public DriveToPosePrecise(
      Drive drive, Supplier<Pose2d> desiredPoseSupplier, Pose2d tolerance, double maxSpeedMPS) {
    this.drive = drive;
    this.desiredPoseSupplier = desiredPoseSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Pose2d desiredPose = desiredPoseSupplier.get();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
