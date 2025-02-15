package frc.robot.subsystems.superstructure.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Hardware implementation of the TemplateIO. */
public class IntakeIOHardware implements IntakeIO {
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  public IntakeIOHardware(int motorIdLeft, int motorIdRight) {
    leftMotor = new SparkMax(motorIdLeft, MotorType.kBrushless);
    rightMotor = new SparkMax(motorIdRight, MotorType.kBrushless);
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
    // TODO Auto-generated method stub
    return IntakeIO.super.isOccupied();
  }

  @Override
  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
