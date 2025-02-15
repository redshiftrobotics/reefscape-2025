package frc.robot.subsystems.superstructure.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Hardware implementation of the TemplateIO. */
public class IntakeIOHardware implements IntakeIO {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private AnalogInput analogInput;

  public IntakeIOHardware(int motorIdLeft, int motorIdRight, int analogInputId) {
    leftMotor = new SparkMax(motorIdLeft, MotorType.kBrushless);
    rightMotor = new SparkMax(motorIdRight, MotorType.kBrushless);
    analogInput = new AnalogInput(analogInputId);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speedLeft = leftMotor.get();
    inputs.speedRight = rightMotor.get();
  }

  @Override
  public void setLeftMotor(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public void setRightMotor(double speed) {
    rightMotor.set(speed);
  }

  @Override
  public boolean isOccupied() {
    double sensorState = analogInput.getValue();
    SmartDashboard.putNumber("input", sensorState);

    //TODO check if 5 or 4 is on because it alternates between 5 and 4 volts
    return sensorState == 5;
  }

  @Override
  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
