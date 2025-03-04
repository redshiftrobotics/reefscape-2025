package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.utility.SparkUtil.tryUntilOk;

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
import frc.robot.subsystems.superstructure.wrist.WristConstants.WristConfig;

public class WristIOHardware implements WristIO {
  private final SparkMax motor;
  private final SparkAbsoluteEncoder encoder;
  private final SparkClosedLoopController control;

  private boolean breakMode = true;

  public WristIOHardware(WristConfig config) {
    motor = new SparkMax(config.motorId(), MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    control = motor.getClosedLoopController();

    final SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(WristConstants.MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12)
        .inverted(config.motorInverted());
    motorConfig
        .absoluteEncoder
        .zeroOffset(config.absoluteEncoderOffset())
        .inverted(config.encoderInverted());
    motorConfig.closedLoop.pidf(0, 0, 0, 0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();

    inputs.appliedVolts = new double[] {motor.getAppliedOutput() * motor.getBusVoltage()};
    inputs.supplyCurrentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void runPosition(double setpoint) {
    control.setReference(setpoint, ControlType.kPosition);
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
