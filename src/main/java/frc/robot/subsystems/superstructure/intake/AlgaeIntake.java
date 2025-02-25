package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase implements IntakeSubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public AlgaeIntake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake", inputs);
  }

  @Override
  public void setLeftMotor(double speed) {
    io.setLeftMotor(speed);
  }

  @Override
  public void setRightMotor(double speed) {
    io.setRightMotor(speed);
  }

  @Override
  public void setMotors(double speed) {
    io.setMotors(speed);
  }

  @Override
  public void stopMotors() {
    io.stopMotors();
  }
}
