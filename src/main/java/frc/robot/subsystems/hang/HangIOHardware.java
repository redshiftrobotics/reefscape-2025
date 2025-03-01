package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.utility.SparkUtil.tryUntilOk;

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
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.hang.HangConstants.HangConfig;
import java.util.function.Supplier;

public class HangIOHardware implements HangIO {
  private final SparkMax motor;

  private final CANcoder cancoder;

  private final Supplier<Angle> position;
  private final Supplier<AngularVelocity> velocity;

  private final PIDController control;

  private boolean breakMode = true;

  public HangIOHardware(HangConfig config) {
    motor = new SparkMax(config.motorId(), MotorType.kBrushless);
    control = new PIDController(0, 0, 0);

    cancoder = new CANcoder(config.cancoderId());

    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    magnetSensorConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    magnetSensorConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
    magnetSensorConfig.MagnetOffset = config.absoluteEncoderOffset().getRotations();
    cancoder.getConfigurator().apply(magnetSensorConfig);

    position = cancoder.getPosition().asSupplier();
    velocity = cancoder.getVelocity().asSupplier();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HangConstants.MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    inputs.positionRotations = position.get().in(Rotations) * (1.0 / HangConstants.GEAR_REDUCTION);
    inputs.velocityRPM = velocity.get().in(RPM) * (1.0 / HangConstants.GEAR_REDUCTION);

    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
    inputs.appliedVolts = new double[] {motor.getAppliedOutput() * motor.getBusVoltage()};
  }

  @Override
  public void periodic() {
    motor.set(control.calculate(position.get().baseUnitMagnitude()));
  }

  @Override
  public void runPosition(double positionRotations) {
    control.setSetpoint(positionRotations);
  }

  @Override
  public void runOpenLoop(double output) {
    motor.set(output);
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    control.setP(kP);
    control.setI(kI);
    control.setD(kD);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (breakMode != enable) {
      breakMode = enable;
      SparkMaxConfig motorConfig = new SparkMaxConfig();
      motorConfig.idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast);
      tryUntilOk(
          motor,
          5,
          () ->
              motor.configure(
                  motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }
  }
}
