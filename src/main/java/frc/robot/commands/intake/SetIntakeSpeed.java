package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;

public class SetIntakeSpeed extends Command {
  private IntakeSubsystem intake;

  private double speed;

  public SetIntakeSpeed(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (speed == 0) {
      intake.stopMotors();
    } else {
      intake.setMotors(speed);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
