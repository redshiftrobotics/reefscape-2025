package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;

/**
 * use while true on the contoller bindings because end stops motors excute starts them
 */
public class ScoreCoral2 extends Command {

  private CoralIntake coralIntake;

  public ScoreCoral2(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }

  @Override
  public void execute() {
    //TODO: use whlie true button in bindings
    // score on not l1
    coralIntake.moveRightMotor(0.3);
    coralIntake.moveLeftMotor(0.3);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }
}
