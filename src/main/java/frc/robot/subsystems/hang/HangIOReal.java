package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class HangIOReal implements HangIO {
  private final SparkMax wenchMotor;
  private final PIDController controller;

  double setpoint = 0.0;

  public HangIOReal(int deviceId) {
    wenchMotor = new SparkMax(deviceId, MotorType.kBrushless);
    controller = new PIDController(REAL_HANG_ARM_P, REAL_HANG_ARM_I, REAL_HANG_ARM_D);
    controller.setTolerance(TOLERANCE);
  }

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
  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void stop() {
    wenchMotor.stopMotor();
  }
}
