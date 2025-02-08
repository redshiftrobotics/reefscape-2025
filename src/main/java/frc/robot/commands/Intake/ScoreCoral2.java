package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.CoralIntake;

public class ScoreCoral2 extends Command {

  private CoralIntake coralIntake;

  public ScoreCoral2(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }
  
  // for l2
  @Override
  public void execute() {
    coralIntake.MoveRightMotor(0.3);
    coralIntake.MoveLeftMotor(0.3);
  }


  @Override
  public boolean isFinished() {
    return true;
  }
}
