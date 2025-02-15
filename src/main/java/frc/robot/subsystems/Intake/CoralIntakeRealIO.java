package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeRealIO implements CoralIntakeIO{

  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private AnalogInput analogInput;

  public CoralIntakeRealIO(int leftMotor, int rightMotor, int analogInput) {

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
    double sensorState = analogInput.getValue();
    SmartDashboard.putNumber("input", sensorState);

    if (sensorState == 5) return true;
    else return false;
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  // TODO add the sensor stuff
}
