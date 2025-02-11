package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeIO extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private AnalogInput analogInput;

  public CoralIntakeIO(int leftMotor, int rightMotor, int analogInput) {

    this.leftMotor = new SparkMax(leftMotor, MotorType.kBrushless);
    this.rightMotor = new SparkMax(rightMotor, MotorType.kBrushless);

    this.analogInput = new AnalogInput(analogInput);
  }

  // motor stuff

  public void moveLeftMotor(double Speed) {
    leftMotor.set(MathUtil.clamp(Speed, -0.5, 0.5));
  }

  public void moveRightMotor(double Speed) {
    rightMotor.set(MathUtil.clamp(Speed, -0.5, 0.5));
  }

  public boolean checkSensor() {
    
    SmartDashboard.putNumber("0",analogInput.getValue());
    
    return true;
  }


  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  // TODO add the sensor stuff
}