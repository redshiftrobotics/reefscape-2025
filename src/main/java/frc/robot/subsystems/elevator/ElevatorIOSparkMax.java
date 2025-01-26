package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public ElevatorIOSparkMax() {
    motor = new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  public void setGoalHeight(double targetHeight) {}

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void updateMotors() {}

  public void configurePID(
      double maxVelocity,
      double maxAcceleration,
      double Kp,
      double Ki,
      double Kd,
      double Ks,
      double Kg,
      double Kv) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(Kp, Ki, Kd, 0.0, null);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    motor.stopMotor();
  }
}
