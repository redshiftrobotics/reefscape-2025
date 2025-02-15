package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;

/**
 * use while true on the contoller bindings because end stops motors excute starts them
 */
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
    //TODO: use whlie true button in bindings
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }
}
