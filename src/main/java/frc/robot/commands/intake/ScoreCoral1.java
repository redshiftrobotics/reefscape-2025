package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;

public class ScoreCoral1 extends Command {

  private CoralIntake coralIntake;

  public ScoreCoral1(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }

  @Override
  public void execute() {
    // scores on l1
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
