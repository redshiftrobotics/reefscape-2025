package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.CoralIntake;

public class ScoreCoral extends Command {

  private Joystick joystick;
  private CoralIntake coralIntake;

  public ScoreCoral(Joystick joystick,CoralIntake coralIntake) {
    this.joystick = joystick;
    this.coralIntake = coralIntake;
  }


  @Override
  public boolean isFinished() {
    return true;
  }
}
