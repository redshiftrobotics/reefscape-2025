package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.utility.SparkUtil.ifOk;
import static frc.robot.utility.SparkUtil.tryUntilOk;

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
  private final SparkAbsoluteEncoder encoder;
  private final SparkClosedLoopController control;

  private final Debouncer connectDebounce = new Debouncer(0.5);

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
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(WristConstants.MAX_POSITION)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(WristConstants.MIN_POSITION);
    motorConfig
        .absoluteEncoder
        .inverted(config.encoderInverted())
        .zeroOffset(config.absoluteEncoderOffset());
    motorConfig
        .closedLoop
        .pidf(0, 0, 0, 0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 1.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    SparkUtil.clearStickyFault();

    ifOk(motor, encoder::getPosition, value -> inputs.positionRotations = value);
    ifOk(motor, encoder::getVelocity, value -> inputs.velocityRPM = value);
    inputs.positionDegrees = Units.rotationsToDegrees(inputs.positionRotations);

    ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = value);
    ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = value);

    inputs.motorConnected = connectDebounce.calculate(!SparkUtil.hasStickyFault());
  }

  @Override
  public void runPosition(double setpoint, double feedforward) {
    control.setReference(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
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
