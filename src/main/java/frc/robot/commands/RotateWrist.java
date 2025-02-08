package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

public class RotateWrist extends Command {
  private Wrist wrist;
  private Rotation2d currentRotation, targetRotation;

  /**
   * @param wristSystem The wrist subsystem to use
   * @param initialTarget The initial target to move to when executed
   */
  public RotateWrist(Wrist wristSystem, Rotation2d initialTarget) {
    wrist = wristSystem;
    currentRotation = wrist.getRotation();
    targetRotation = initialTarget;
  }

  /**
   * @param wristSystem The wrist subsystem to use
   * @apiNote Sets initial target to current wrist rotation
   */
  public RotateWrist(Wrist wristSystem) {
    this(wristSystem, wristSystem.getRotation());
  }

  /**
   * @param target The new target
   * @apiNote Cancels execution if called while command is in progress
   */
  public void setTarget(Rotation2d target) {
    if (!isFinished()) cancel();
    targetRotation = target;
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Wrist Rotation Target", targetRotation.getDegrees());
    currentRotation = wrist.getRotation();
  }

  @Override
  public void execute() {
    currentRotation = wrist.getRotation();
    wrist.setRotation(targetRotation);
  }

  @Override
  public boolean isFinished() {
    return currentRotation.getRotations() == targetRotation.getRotations();
  }

  @Override
  public void end(boolean interrupted) {}
}
