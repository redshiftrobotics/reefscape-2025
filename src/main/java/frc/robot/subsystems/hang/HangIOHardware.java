package frc.robot.subsystems.hang;

import static frc.robot.utility.SparkUtil.ifOk;
import static frc.robot.utility.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.hang.HangConstants.HangConfig;
import frc.robot.utility.SparkUtil;

public class HangIOHardware implements HangIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkAbsoluteEncoder absEncoder;
  private final SparkClosedLoopController control;

  private boolean breakMode = true;

  private final Debouncer connectDebounce = new Debouncer(0.5);

  public HangIOHardware(HangConfig config) {
    motor = new SparkMax(config.motorId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    absEncoder = motor.getAbsoluteEncoder();
    control = motor.getClosedLoopController();

    final SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(config.motorInverted())
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(HangConstants.MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(1.0 / HangConstants.GEAR_REDUCTION)
        .velocityConversionFactor(1.0 / HangConstants.GEAR_REDUCTION);
    motorConfig
        .absoluteEncoder
        .inverted(config.encoderInverted())
        .zeroOffset(config.absoluteEncoderOffset())
        .zeroCentered(true);
    motorConfig.closedLoop.pidf(0, 0, 0, 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    SparkUtil.clearStickyFault();

    ifOk(motor, encoder::getPosition, value -> inputs.positionRotations = value);
    ifOk(motor, encoder::getVelocity, value -> inputs.velocityRPM = value);

    ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = new double[] {value});
    ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = new double[] {value});

    inputs.motorConnected = connectDebounce.calculate(!SparkUtil.hasStickyFault());
  }

  @Override
  public void setLimits(double forward, double backward) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(forward)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(backward);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void runPosition(double setpointRotations) {
    control.setReference(setpointRotations, ControlType.kPosition);
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
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);

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
