package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

public class AdjustWrist extends Command {
  private Wrist wrist;
  private Rotation2d currentRotation, adjustment, targetRotation;

  /**
   * @param wristSystem The wrist subsystem to use
   * @param initialTarget The initial target to move to when executed
   */
  public AdjustWrist(Wrist wristSystem, Rotation2d adjustmentAmount) {
    wrist = wristSystem;
    currentRotation = wrist.getRotation();
    adjustment = adjustmentAmount;
    targetRotation = Rotation2d.kZero;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    currentRotation = wrist.getRotation();
    targetRotation =
        Rotation2d.fromRadians(wrist.getTargetRotation().getRadians() + adjustment.getRadians());
    SmartDashboard.putNumber("Wrist Rotation Target", targetRotation.getRadians());
  }

  @Override
  public void execute() {
    currentRotation = wrist.getRotation();
    wrist.setRotation(targetRotation);
  }

  // TODO: Uncomment the commented stuff below when the wrist is working. This is just here to make
  // testing it more flexible while it isn't working.

  @Override
  public boolean isFinished() {
    /*double rotInRad = currentRotation.getRotations();
    return targetRotation.getRotations() > (rotInRad - WristConstants.WRIST_MOVE_DONE_THRESHOLD)
        && targetRotation.getRotations() < (rotInRad + WristConstants.WRIST_MOVE_DONE_THRESHOLD);*/
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    // wrist.stop();
  }
}
