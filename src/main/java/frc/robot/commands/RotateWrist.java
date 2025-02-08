package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class RotateWrist extends Command {
  private Wrist wrist;
  private Rotation2d currentRotation, targetRotation;
  private boolean inProgress;

  /**
   * @param wristSystem The wrist subsystem to use
   * @param initialTarget The initial target to move to when executed
   */
  public RotateWrist(Wrist wristSystem, Rotation2d initialTarget) {
    initWristAndCurrentRotation(wristSystem);
    targetRotation = initialTarget;
  }

  /**
   * @param wristSystem The wrist subsystem to use
   * @apiNote Sets initial target to current wrist rotation
   */
  public RotateWrist(Wrist wristSystem) {
    initWristAndCurrentRotation(wristSystem);
    targetRotation = wrist.getRotation();
  }

  private void initWristAndCurrentRotation(Wrist wristSystem) {
    wrist = wristSystem;
    currentRotation = wrist.getRotation();
  }

  /**
   * @param target The new target
   * @apiNote Cancels execution if called while command is in progress
   */
  public void setTarget(Rotation2d target) {
    if (inProgress && !isFinished()) {
      cancel();
    }
    targetRotation = target;
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Wrist Rotation Target", targetRotation.getRadians());
    currentRotation = wrist.getRotation();
    inProgress = true;
  }

  @Override
  public void execute() {
    currentRotation = wrist.getRotation();
    wrist.setRotation(targetRotation);
  }

  @Override
  public boolean isFinished() {
    double rotInRad = currentRotation.getRotations();
    return targetRotation.getRotations() > (rotInRad - WristConstants.WRIST_MOVE_DONE_THRESHOLD)
        && targetRotation.getRotations() < (rotInRad + WristConstants.WRIST_MOVE_DONE_THRESHOLD);
  }

  @Override
  public void end(boolean interrupted) {
    inProgress = false;
    wrist.stop();
  }
}
