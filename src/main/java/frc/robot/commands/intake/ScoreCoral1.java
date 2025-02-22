package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.intake.CoralIntake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;

/** use while true on the contoller bindings because end stops motors excute starts them */
public class ScoreCoral1 extends Command {

  private CoralIntake coralIntake;

  public ScoreCoral1(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;

    addRequirements(coralIntake);
  }

  @Override
  public void initialize() {
    // scores on l1
    coralIntake.setRightMotor(IntakeConstants.CORAL_INTAKE_MOTOR_SPEED);
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
