package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.ABSOLUTE_ENCODER_OFFSET;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

public class WristIOAbsoluteEncoder implements WristIO {
  private final SparkMax motor;
  private final PIDController pidController;
  private final CANcoder encoder;
  private final Supplier<Angle> positionSupplier;

  public WristIOAbsoluteEncoder(int motorId, int encoderId) {
    motor = new SparkMax(motorId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(WristConstants.CURRENT_LIMIT)
        .voltageCompensation(12);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = new CANcoder(encoderId);

    pidController = new PIDController(0, 0, 0);

    MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs();
    cancoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
    cancoderConfig.MagnetOffset = ABSOLUTE_ENCODER_OFFSET;

    encoder.getConfigurator().apply(cancoderConfig);

    positionSupplier = encoder.getPosition().asSupplier();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    motor.set(pidController.calculate(getPosition()));

    inputs.setpointRotations = pidController.getSetpoint();
    inputs.positionRotations = getPosition();
  }

  @Override
  public void runPosition(double setpoint) {
    pidController.setSetpoint(setpoint);
  }

  @Override
  public void setPid(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);

    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private double getPosition() {
    return positionSupplier.get().in(Rotations);
  }
}
