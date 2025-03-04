package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.wrist.Wrist;

public class SetWrist extends Command {

  private double setpoint;
  private Wrist wrist;

  public SetWrist(Wrist wrist, double setpoint) {
    this.setpoint = setpoint;
    this.wrist = wrist;
  }

  @Override
  public void initialize() {
    wrist.setGoal(setpoint);
  }

  @Override
  public boolean isFinished() {
    return wrist.atGoal();
  }
}
