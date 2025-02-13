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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants.ElevatorConfig;
import frc.robot.utility.SparkUtil;
import java.util.function.DoubleSupplier;

/** Hardware implementation of the TemplateIO. */
public class ElevatorIOHardware implements ElevatorIO {

  private final SparkMax leader;
  private final SparkMax follower;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController control;

  private final Debouncer connectDebounce = new Debouncer(0.5);

  private boolean breakMode = true;

  public ElevatorIOHardware(ElevatorConfig config) {

    leader = new SparkMax(config.leaderCanId(), MotorType.kBrushless);
    follower = new SparkMax(config.followerCanId(), MotorType.kBrushless);

    encoder = leader.getEncoder();
    control = leader.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12);
    motorConfig
        .encoder
        .positionConversionFactor(1 / ElevatorConstants.drumGearReduction)
        .velocityConversionFactor(1 / ElevatorConstants.drumGearReduction);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                motorConfig.follow(leader, config.inverted()),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    SparkUtil.clearStickyFault();
    ifOk(
        leader,
        encoder::getPosition,
        value -> inputs.positionRad = Units.rotationsToRadians(value));
    ifOk(
        leader,
        encoder::getVelocity,
        value -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));

    ifOkMulti(
        new SparkMax[] {leader, follower},
        new DoubleSupplier[] {
          () -> leader.getAppliedOutput() * leader.getBusVoltage(),
          () -> follower.getAppliedOutput() * follower.getBusVoltage()
        },
        values -> inputs.appliedVolts = values);
    ifOkMulti(
        new SparkMax[] {leader, follower},
        new DoubleSupplier[] {leader::getOutputCurrent, follower::getOutputCurrent},
        values -> inputs.supplyCurrentAmps = values);

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
    leader.set(output);
  }

  @Override
  public void runVolts(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.pidf(kP, kI, kD, 0);
    tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (breakMode != enable) {
      breakMode = enable;
      SparkMaxConfig motorConfig = new SparkMaxConfig();
      motorConfig.idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast);
      tryUntilOk(
          leader,
          5,
          () ->
              leader.configure(
                  motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
  }
}
