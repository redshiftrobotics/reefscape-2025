package frc.robot.subsystems.superstructure.intake;

import static frc.robot.subsystems.superstructure.intake.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

/** Hardware implementation of the TemplateIO. */
public class IntakeIOHardware implements IntakeIO {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private AnalogInput sensor;

  public IntakeIOHardware(int motorIdLeft, int motorIdRight, int analogInputId) {
    leftMotor = new SparkMax(motorIdLeft, MotorType.kBrushless);
    rightMotor = new SparkMax(motorIdRight, MotorType.kBrushless);
    sensor = new AnalogInput(analogInputId);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speedLeft = leftMotor.get();
    inputs.speedRight = rightMotor.get();

    // maybe we coulda had 4 vars but i just did what the example had
    inputs.appliedVolts = new double[] {
        leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(),
        rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()
    };
    
    inputs.supplyCurrentAmps = new double[] {
        leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()
    };
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
    double sensorState = sensor.getValue();
    SmartDashboard.putNumber("input", sensorState);

    return MathUtil.isNear(sensorState, SIGNAL_SENSOR_OCCUPIED, SENSOR_VOLTAGE_TOLERANCE);
  }

  @Override
  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
