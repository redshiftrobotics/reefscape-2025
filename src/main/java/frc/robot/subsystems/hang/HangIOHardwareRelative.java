package frc.robot.subsystems.hang;

import static frc.robot.utility.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

public class HangIOHardwareRelative implements HangIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController control;

  private boolean breakMode = true;

  public HangIOHardwareRelative(HangConfig config) {
    motor = new SparkMax(config.motorId(), MotorType.kBrushless);

    encoder = motor.getEncoder();
    control = motor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HangConstants.MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(1.0 / HangConstants.GEAR_REDUCTION)
        .velocityConversionFactor(1.0 / HangConstants.GEAR_REDUCTION);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0.0, 0.0, 0.0, 0.0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();

    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
    inputs.appliedVolts = new double[] {motor.getAppliedOutput() * motor.getBusVoltage()};
  }

  @Override
  public void runPosition(double positionRotations) {
    control.setReference(positionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.closedLoop.pidf(kP, kI, kD, 0.0);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
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
