package frc.robot.subsystems.superstructure.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Hardware implementation of the TemplateIO. */
public class IntakeIOHardware implements IntakeIO {
  private SparkMax motor1;
  private SparkMax motor2;

  public IntakeIOHardware(int motorId1, int motorId2)
  {
      motor1 = new SparkMax(motorId1, MotorType.kBrushless);
      motor2 = new SparkMax(motorId2, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void setMotorSpeed(double speed) {
    // TODO Auto-generated method stub
    IntakeIO.super.setMotorSpeed(speed);
  }
}
