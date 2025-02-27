package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.subsystems.superstructure.wrist.WristConstants.RELATIVE_CONVERSION_FACTOR;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Wrist implementation using the built-in relative encoder. */
public class WristIORelativeEncoder implements WristIO {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;
  private final RelativeEncoder encoder;

  private double setpoint;

  public WristIORelativeEncoder(int motorId) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(RELATIVE_CONVERSION_FACTOR);
    config.encoder.velocityConversionFactor(RELATIVE_CONVERSION_FACTOR);

    motor = new SparkMax(motorId, MotorType.kBrushless);
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
