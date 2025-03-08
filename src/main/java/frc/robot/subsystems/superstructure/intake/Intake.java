package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final String name;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert;

  public Intake(String name, IntakeIO io) {
    this.name = name;
    this.io = io;

    motorDisconnectedAlert =
        new Alert("Intake " + name + " motor/s disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake " + name, inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public Command intake(double speed) {
    return startEnd(() -> setMotors(speed), this::stopMotors);
  }

  public Command intake(double leftSpeed, double rightSpeed) {
    return startEnd(
        () -> {
          setLeftMotor(leftSpeed);
          setRightMotor(rightSpeed);
        },
        this::stopMotors);
  }

  public void setLeftMotor(double speed) {
    io.setLeftMotor(speed);
  }

  public void setRightMotor(double speed) {
    io.setRightMotor(speed);
  }

  public void setMotors(double speed) {
    io.setLeftMotor(speed);
    io.setRightMotor(speed);
  }

  public boolean isIntakeRunning() {
    return (inputs.speedLeft + inputs.speedRight) != 0;
  }

  public void stopMotors() {
    io.stopMotors();
  }
}
