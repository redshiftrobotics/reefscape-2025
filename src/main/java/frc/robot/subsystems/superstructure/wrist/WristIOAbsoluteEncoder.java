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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.utility.records.PIDConstants;
import java.util.function.Supplier;

public class WristIOAbsoluteEncoder implements WristIO {
  private final SparkMax motor;
  private final PIDController pidController;
  private final CANcoder encoder;
  private final Supplier<Angle> positionSupplier;

  private double setpoint;

  public WristIOAbsoluteEncoder(int motorId, int encoderId) {
    SparkMaxConfig config = new SparkMaxConfig();

    motor = new SparkMax(motorId, MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = new CANcoder(encoderId);

    PIDConstants pid = WristConstants.getPidConstants();
    pidController = new PIDController(pid.kP(), pid.kI(), pid.kD());

    MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs();
    cancoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
    cancoderConfig.MagnetOffset = ABSOLUTE_ENCODER_OFFSET;

    encoder.getConfigurator().apply(cancoderConfig);

    positionSupplier = encoder.getPosition().asSupplier();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    motor.set(pidController.calculate(getPosition(), setpoint));

    inputs.setpointRotations = setpoint;
    inputs.positionRotations = getPosition();
  }

  @Override
  public void runPosition(double setpoint) {
    this.setpoint = setpoint;
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
