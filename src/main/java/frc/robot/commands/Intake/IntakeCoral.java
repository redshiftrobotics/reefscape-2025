package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.intake.CoralIntake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;

/** use while true on the contoller bindings because end stops motors excute starts them */
public class IntakeCoral extends Command {

  private CoralIntake coralIntake;

  public IntakeCoral(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }

  @Override
  public void initialize() {
    coralIntake.setMotors(-IntakeConstants.CORAL_INTAKE_MOTOR_SPEED);
  }

  @Override
  public boolean isFinished() {
    // TODO: use whlie true button in bindings
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    coralIntake.stopMotors();
  }
}
