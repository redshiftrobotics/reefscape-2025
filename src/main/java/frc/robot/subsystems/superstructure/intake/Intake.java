package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.intake.sensor.Sensor;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Sensor sensor;
  private boolean useSensor = true;

  private final Alert motorDisconnectedAlert;

  public Intake(IntakeIO io) {
    this(io, null);
  }

  public Intake(IntakeIO io, Sensor sensor) {
    this.io = io;

    this.sensor = sensor;

    motorDisconnectedAlert = new Alert("Intake motor/s disconnected!", Alert.AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public Command runMotors(double speed) {
    return runMotors(speed, speed);
  }

  public Command runMotors(double leftSpeed, double rightSpeed) {
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

  public double getRightMotor() {
    return inputs.speedRight;
  }

  public double getLeftMotor() {
    return inputs.speedLeft;
  }

  public double getMotorsAvg() {
    return (inputs.speedLeft + inputs.speedRight) / 2;
  }

  public void stopMotors() {
    io.stopMotors();
  }

  public boolean usingSensor() {
    return sensor != null && useSensor;
  }

  public void setUseSensor(boolean useSensor) {
    this.useSensor = useSensor;
  }

  public void toggleUseSensor() {
    this.useSensor = !useSensor;
  }

  public Optional<Boolean> hasCoral() {
    return usingSensor() ? Optional.of(sensor.isDetected()) : Optional.empty();
  }
}
