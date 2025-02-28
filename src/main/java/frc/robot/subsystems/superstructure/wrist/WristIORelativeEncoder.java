package frc.robot.subsystems.superstructure.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Wrist implementation using the built-in relative encoder. */
public class WristIORelativeEncoder implements WristIO {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;
  private final RelativeEncoder encoder;

  private double setpoint;

  public WristIORelativeEncoder(int motorId) {
    motor = new SparkMax(motorId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(WristConstants.CURRENT_LIMIT)
        .voltageCompensation(12);
    config.encoder.positionConversionFactor(1.0 / WristConstants.GEAR_REDUCTION);
    config.encoder.velocityConversionFactor(1.0 / WristConstants.GEAR_REDUCTION);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = motor.getClosedLoopController();

    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.setpointRotations = setpoint;
    inputs.positionRotations = encoder.getPosition();
  }

  @Override
  public void runPosition(double setpoint) {
    this.setpoint = setpoint;

    pidController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setPid(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);

    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
