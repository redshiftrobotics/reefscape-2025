package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.CoralIntake;

public class ScoreCoral1 extends Command {

  private CoralIntake coralIntake;

  public ScoreCoral1(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }

  // for l1
  @Override
  public void execute() {
    coralIntake.moveRightMotor(0.3);
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