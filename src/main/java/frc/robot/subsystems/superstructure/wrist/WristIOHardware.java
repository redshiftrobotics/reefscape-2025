package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.utility.SparkUtil.ifOk;
import static frc.robot.utility.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.wrist.WristConstants.WristConfig;
import frc.robot.utility.SparkUtil;

public class WristIOHardware implements WristIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkAbsoluteEncoder absEncoder;
  private final SparkClosedLoopController control;

  private final Debouncer connectDebounce = new Debouncer(0.5);

  private boolean breakMode = true;
  private double setpointRad;

  public WristIOHardware(WristConfig config) {
    motor = new SparkMax(config.motorId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    absEncoder = motor.getAbsoluteEncoder();
    control = motor.getClosedLoopController();

    final SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(WristConstants.MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12)
        .inverted(config.motorInverted());
    motorConfig
        .encoder
        .positionConversionFactor(1.0 / WristConstants.GEAR_REDUCTION)
        .velocityConversionFactor(1.0 / WristConstants.GEAR_REDUCTION);
    motorConfig
        .absoluteEncoder
        .inverted(config.encoderInverted())
        .zeroOffset(config.absoluteEncoderOffset())
        .zeroCentered(true);
    motorConfig.closedLoop.pidf(0, 0, 0, 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(absEncoder.getPosition());
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    SparkUtil.clearStickyFault();

    inputs.setpointRad = setpointRad;

    ifOk(
        motor, encoder::getPosition, value -> inputs.positionRad = Units.rotationsToRadians(value));
    ifOk(
        motor,
        encoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));

    ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = value);
    ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);

    inputs.motorConnected = connectDebounce.calculate(!SparkUtil.hasStickyFault());
  }

  @Override
  public void runPosition(double setpointRad, double feedforwardVolts) {
    this.setpointRad = setpointRad;
    control.setReference(
        Units.radiansToRotations(setpointRad),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0, ClosedLoopSlot.kSlot0);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
