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
public class ElevatorIOHardwareFollow implements ElevatorIO {

  private final SparkMax leader;
  private final SparkMax follower;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController control;

  private final Debouncer connectDebounce = new Debouncer(0.5);

  private boolean breakMode = true;

  public ElevatorIOHardwareFollow(ElevatorConfig config) {

    leader = new SparkMax(config.leaderCanId(), MotorType.kBrushless);
    follower = new SparkMax(config.followerCanId(), MotorType.kBrushless);

    encoder = leader.getEncoder();
    control = leader.getClosedLoopController();

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12);
    leaderConfig
        .encoder
        .positionConversionFactor(1 / ElevatorConstants.gearReduction)
        .velocityConversionFactor(1 / ElevatorConstants.gearReduction);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(breakMode ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12)
        .follow(leader, config.inverted());

    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(leader, 5, () -> encoder.setPosition(0.0));
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
        new DoubleSupplier[] {leader::getAppliedOutput, follower::getAppliedOutput},
        values -> inputs.dutyCycle = values);
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
    inputs.followerMotorFollowing = follower.isFollower();

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
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
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
      tryUntilOk(
          follower,
          5,
          () ->
              follower.configure(
                  motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
  }
}
