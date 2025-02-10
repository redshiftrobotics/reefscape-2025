package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.CoralIntake;

public class IntakeCoral extends Command {
  
  private CoralIntake coralIntake;

  public IntakeCoral(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }
  

  @Override
  public void execute() {
    coralIntake.moveRightMotor(-0.3);
    coralIntake.moveLeftMotor(-0.3);
  }
  

  @Override
  public boolean isFinished() {
    return true;
  }


  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }
} 