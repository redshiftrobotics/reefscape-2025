package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.utility.SparkUtil.ifOk;
import static frc.robot.utility.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.utility.SparkUtil;

/** Hardware implementation of the TemplateIO. */
public class ElevatorIOHardware implements ElevatorIO {

  private final SparkMax motor =
      new SparkMax(ElevatorConstants.drumMotorCanID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkClosedLoopController control = motor.getClosedLoopController();

  private final Debouncer connectDebounce = new Debouncer(0.5);

  private boolean breakMode = true;

  public ElevatorIOHardware() {

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(20)
        .voltageCompensation(12);
    motorConfig
        .encoder
        .positionConversionFactor(1 / ElevatorConstants.drumGearReduction)
        .velocityConversionFactor(1 / ElevatorConstants.drumGearReduction);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    SparkUtil.clearStickyFault();
    ifOk(
        motor, encoder::getPosition, value -> inputs.positionRad = Units.rotationsToRadians(value));
    ifOk(
        motor,
        encoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    ifOk(
        motor,
        () -> motor.getAppliedOutput() * motor.getBusVoltage(),
        value -> inputs.appliedVolts = new double[] {value});
    ifOk(motor, motor::getOutputCurrent, value -> inputs.supplyCurrentAmps = new double[] {value});

    inputs.motorConnected = connectDebounce.calculate(!SparkUtil.hasStickyFault());
  }

  @Override
  public void setGoalPosition(double positionRad, double feedforward) {
    control.setReference(
        Units.radiansToRotations(positionRad),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
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
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
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
                  motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
  }
}
