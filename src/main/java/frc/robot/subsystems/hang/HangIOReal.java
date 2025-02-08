package frc.robot.subsystems.hang;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class HangIOReal implements HangIO {
  SparkMax wenchMotor = new SparkMax(0, MotorType.kBrushless); // TODO CAN ID

  PIDController controller = new PIDController(0, 0, 0); // TODO Tune

  double setpoint = 0.0;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    wenchMotor.set(controller.calculate(getPosition(), setpoint));

    inputs.armSetpoint = setpoint;
    inputs.armPosition = getPosition();
  }

  @Override
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public double getPosition() {
    return wenchMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void stop() {
    wenchMotor.stopMotor();
  }
}
