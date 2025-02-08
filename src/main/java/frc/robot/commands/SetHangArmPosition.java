package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

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
    final double deadband = 1; // TODO

    // if within deadband range make value 0
    // therefore when value is 0 we are within finished range
    return MathUtil.applyDeadband(hang.getPosition(), deadband) == 0;
  }

  @Override
  public void end(boolean interrupted) {
    hang.stop();
  }
}
