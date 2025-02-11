package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.HangConstants;

public class SetHangArmPosition extends Command {
  private Hang hang;
  private double setpoint;

  public SetHangArmPosition(Hang hang, double setpoint) {
    this.setpoint = setpoint;
    this.hang = hang;

    addRequirements(hang);
  }

  @Override
  public void initialize() {
    hang.setSetpoint(setpoint);
  }

  @Override
  public boolean isFinished() {
    double i = Math.abs(hang.getPosition());

    return i > (i - HangConstants.TOLERANCE) && i < (i + HangConstants.TOLERANCE);
  }

  @Override
  public void end(boolean interrupted) {
    hang.stop();
  }
}
