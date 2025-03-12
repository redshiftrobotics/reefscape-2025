package frc.robot.subsystems.superstructure.algae;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.subsystems.superstructure.intake.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert;

  public Algae(IntakeIO io) {
    this.io = io;

    motorDisconnectedAlert = new Alert("Algae motor/s disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public Command runMotors() {
    return runEnd(
        () -> {
          io.setLeftMotor(AlgaeConstants.SPEED);
          io.setRightMotor(AlgaeConstants.SPEED);
        },
        io::stopMotors);
  }
}
