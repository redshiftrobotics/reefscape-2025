package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.intake.CoralIntake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;

/** use while true on the contoller bindings because end stops motors excute starts them */
public class ScoreCoral2 extends Command {

  private CoralIntake coralIntake;

  public ScoreCoral2(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }

  @Override
  public void initialize() {
    // TODO: use whlie true button in bindings
    // score on not l1
    coralIntake.setMotors(IntakeConstants.CORAL_INTAKE_MOTOR_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    coralIntake.stopMotors();
  }
}
