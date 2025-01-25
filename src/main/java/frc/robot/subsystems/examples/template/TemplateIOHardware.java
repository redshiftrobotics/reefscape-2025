package frc.robot.subsystems.examples.template;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Hardware implementation of the TemplateIO. */
public class TemplateIOHardware implements TemplateIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public TemplateIOHardware() {

    motor = new SparkMax(TemplateConstants.CAN_ID, MotorType.kBrushless);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.voltageCompensation(12.0).smartCurrentLimit(30).idleMode(IdleMode.kCoast);
    leaderConfig.encoder.velocityConversionFactor(1.0).velocityConversionFactor(1.0);

    encoder = motor.getEncoder();

    motor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TemplateIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
