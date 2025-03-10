package frc.robot.subsystems.hang;

import static frc.robot.utility.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.hang.HangConstants.HangConfig;

public class HangIOHardware implements HangIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController control;

  private boolean breakMode = true;

  public HangIOHardware(HangConfig config) {
    motor = new SparkMax(config.motorId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    control = motor.getClosedLoopController();

    final SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(HangConstants.MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    motorConfig.absoluteEncoder.zeroOffset(config.absoluteEncoderOffset());
    motorConfig.closedLoop.pidf(0, 0, 0, 0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();

    inputs.appliedVolts = new double[] {motor.getAppliedOutput() * motor.getBusVoltage()};
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
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
  public void runPosition(double setpoint) {
    control.setReference(setpoint, ControlType.kPosition);
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
