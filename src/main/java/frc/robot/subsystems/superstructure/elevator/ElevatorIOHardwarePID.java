package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.utility.SparkUtil.ifOk;
import static frc.robot.utility.SparkUtil.ifOkMulti;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants.ElevatorConfig;
import frc.robot.utility.SparkUtil;
import java.util.function.DoubleSupplier;

/** Hardware implementation of the TemplateIO. */
public class ElevatorIOHardwarePID implements ElevatorIO {

  private final SparkMax primary;
  private final SparkMax secondary;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController control;

  private final RelativeEncoder secondaryEncoder;
  private final SparkClosedLoopController secondaryControl;

  private final Debouncer connectDebounce = new Debouncer(0.5);

  private boolean breakMode = true;

  public ElevatorIOHardwarePID(ElevatorConfig config) {

    primary = new SparkMax(config.leaderCanId(), MotorType.kBrushless);
    secondary = new SparkMax(config.followerCanId(), MotorType.kBrushless);

    encoder = primary.getEncoder();
    control = primary.getClosedLoopController();

    secondaryEncoder = secondary.getEncoder();
    secondaryControl = secondary.getClosedLoopController();

    SparkMaxConfig primaryConfig = new SparkMaxConfig();
    primaryConfig
        .inverted(false)
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12);
    primaryConfig
        .encoder
        .positionConversionFactor(1 / ElevatorConstants.gearReduction)
        .velocityConversionFactor(1 / ElevatorConstants.gearReduction);
    primaryConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    SparkMaxConfig secondaryConfig = new SparkMaxConfig();
    secondaryConfig
        .inverted(config.inverted())
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12);
    secondaryConfig
        .encoder
        .positionConversionFactor(1 / ElevatorConstants.gearReduction)
        .velocityConversionFactor(1 / ElevatorConstants.gearReduction);
    secondaryConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    tryUntilOk(
        primary,
        5,
        () ->
            primary.configure(
                primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        secondary,
        5,
        () ->
            secondary.configure(
                secondaryConfig.inverted(config.inverted()),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    tryUntilOk(primary, 5, () -> encoder.setPosition(0.0));
    tryUntilOk(secondary, 5, () -> secondaryEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    SparkUtil.clearStickyFault();
    ifOk(
        primary,
        encoder::getPosition,
        value -> inputs.positionRad = Units.rotationsToRadians(value));
    ifOk(
        primary,
        encoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));

    ifOkMulti(
        new SparkMax[] {primary, secondary},
        new DoubleSupplier[] {primary::getAppliedOutput, secondary::getAppliedOutput},
        values -> inputs.dutyCycle = values);
    ifOkMulti(
        new SparkMax[] {primary, secondary},
        new DoubleSupplier[] {
          () -> primary.getAppliedOutput() * primary.getBusVoltage(),
          () -> secondary.getAppliedOutput() * secondary.getBusVoltage()
        },
        values -> inputs.appliedVolts = values);
    ifOkMulti(
        new SparkMax[] {primary, secondary},
        new DoubleSupplier[] {primary::getOutputCurrent, secondary::getOutputCurrent},
        values -> inputs.supplyCurrentAmps = values);

    inputs.motorConnected = connectDebounce.calculate(!SparkUtil.hasStickyFault());
    inputs.followerMotorFollowing = MathUtil.isNear(encoder.getPosition(), secondaryEncoder.getPosition(), 0.1);

    inputs.breakMode = breakMode;
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    control.setReference(
        Units.radiansToRotations(positionRad),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
    secondaryControl.setReference(
        Units.radiansToRotations(positionRad),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void runOpenLoop(double output) {
    primary.set(output);
    secondary.set(output);
  }

  @Override
  public void runVolts(double volts) {
    primary.setVoltage(volts);
    secondary.setVoltage(volts);
  }

  @Override
  public void stop() {
    primary.stopMotor();
    secondary.stopMotor();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);
    tryUntilOk(
        primary,
        5,
        () ->
            primary.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        secondary,
        5,
        () ->
            secondary.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (breakMode != enable) {
      breakMode = enable;
      SparkMaxConfig motorConfig = new SparkMaxConfig();
      motorConfig.idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast);
      tryUntilOk(
          primary,
          5,
          () ->
              primary.configure(
                  motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
      tryUntilOk(
          secondary,
          5,
          () ->
              secondary.configure(
                  motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
  }
}
