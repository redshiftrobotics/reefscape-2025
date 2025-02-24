package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Wheels for subsystem */
public class CoralIntake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Template. */
  public CoralIntake(IntakeIO io) {
    this.io = io;
  }

  public void setLeftMotor(double speed) {
    io.setLeftMotor(speed);
  }

  public void setRightMotor(double speed) {
    io.setRightMotor(speed);
  }

  public void setMotors(double speed) {
    io.setMotors(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);
  }

  public void stopMotors() {
    io.stopMotors();
  }
}
