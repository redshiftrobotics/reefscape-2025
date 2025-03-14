package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Mechanism at end of elevator to move intake/ */
public class Wrist extends SubsystemBase {
  private static final LoggedTunableNumberFactory factory = new LoggedTunableNumberFactory("Wrist");
  private static final LoggedTunableNumber kP =
      factory.getNumber("kP", WristConstants.FEEDBACK.kP());
  private static final LoggedTunableNumber kI =
      factory.getNumber("kI", WristConstants.FEEDBACK.kI());
  private static final LoggedTunableNumber kD =
      factory.getNumber("kD", WristConstants.FEEDBACK.kD());

  private static final LoggedTunableNumber kS =
      factory.getNumber("kS", WristConstants.FEEDFORWARD.kS());
  private static final LoggedTunableNumber kG =
      factory.getNumber("kG", WristConstants.FEEDFORWARD.kG());
  private static final LoggedTunableNumber kV =
      factory.getNumber("kV", WristConstants.FEEDFORWARD.kV());
  private static final LoggedTunableNumber kA =
      factory.getNumber("kA", WristConstants.FEEDFORWARD.kA());

  private static final LoggedTunableNumber toleranceDegrees =
      factory.getNumber("ToleranceDegrees", WristConstants.TOLERANCE_DEGREES);

  private static final LoggedTunableNumber maxVelocity =
      factory.getNumber("maxVelocity", WristConstants.MAX_VELOCITY);
  private static final LoggedTunableNumber maxAcceleration =
      factory.getNumber("maxAcceleration", WristConstants.MAX_ACCELERATION);

  private static final LoggedTunableNumber maxVelocitySlow =
      factory.getNumber("maxVelocitySlow", WristConstants.MAX_VELOCITY_SLOW);
  private static final LoggedTunableNumber maxAccelerationSlow =
      factory.getNumber("maxAccelerationSlow", WristConstants.MAX_ACCELERATION_SLOW);

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private TrapezoidProfile profile;
  private TrapezoidProfile profileSlow;
  private BooleanSupplier slowModeSupplier = () -> false;

  private ArmFeedforward feedforward;

  private State setpoint = null;
  private Supplier<State> goalSupplier = State::new;

  private final Alert motorConnectedAlert =
      new Alert("Wrist motor disconnected!", Alert.AlertType.kError);

  private final Debouncer disabledDebouncer = new Debouncer(3, DebounceType.kRising);

  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;

    profile = new TrapezoidProfile(new Constraints(maxVelocity.get(), maxAcceleration.get()));
    profileSlow = new TrapezoidProfile(new Constraints(maxVelocitySlow.get(), maxAccelerationSlow.get()));

    io.setPID(kP.get(), kI.get(), kD.get());
    io.setBrakeMode(true);
    this.feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist", inputs);

    Logger.recordOutput("Wrist/SlowMode", slowModeSupplier.getAsBoolean());

    io.setBrakeMode(!disabledDebouncer.calculate(DriverStation.isDisabled()));

    if (DriverStation.isEnabled()) {
      Logger.recordOutput("Wrist/shouldRunProfiled", true);

      if (setpoint == null) {
        setpoint = new State(inputs.positionRad, inputs.velocityRadPerSec);
      }

      final TrapezoidProfile currentProfile = slowModeSupplier.getAsBoolean() ? profileSlow : profile;

      setpoint = currentProfile.calculate(Constants.LOOP_PERIOD_SECONDS, setpoint, goalSupplier.get());

      double feedforwardVolts = feedforward.calculate(setpoint.position, setpoint.velocity);

      io.runPosition(setpoint.position, feedforwardVolts);

      Logger.recordOutput("Wrist/Feedforward/Volts", feedforwardVolts);

      Logger.recordOutput("Wrist/Profile/SetpointPositionRotations", setpoint.position);
      Logger.recordOutput("Wrist/Profile/SetpointVelocityRPM", setpoint.velocity);
      Logger.recordOutput("Wrist/Profile/GoalPositionRotations", goalSupplier.get().position);
      Logger.recordOutput("Wrist/Profile/GoalVelocityRPM", goalSupplier.get().velocity);
    } else {
      Logger.recordOutput("Wrist/shouldRunProfiled", false);

      Logger.recordOutput("Wrist/FeedForward/Volts", 0);
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);

      setpoint = new State(inputs.positionRad, inputs.velocityRadPerSec);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          io.setPID(values[0], values[1], values[2]);
        },
        kP,
        kI,
        kD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> feedforward = new ArmFeedforward(values[0], values[1], values[2], values[3]),
        kS,
        kG,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> profile = new TrapezoidProfile(new Constraints(values[0], values[1])),
        maxVelocity,
        maxAcceleration);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> profileSlow = new TrapezoidProfile(new Constraints(values[0], values[1])),
        maxVelocitySlow,
        maxAccelerationSlow);

    motorConnectedAlert.set(!inputs.motorConnected);
  }

  // --- Commands ---

  public Command runPositionPrepare(Rotation2d position) {
    return runOnce(() -> setGoalRotation(position));
  }

  public Command runPositionPrepare(Supplier<Rotation2d> position) {
    return runOnce(() -> setGoalSupplier(() -> new State(position.get().getRadians(), 0)));
  }

  // --- Methods ---

  public void setGoalRotation(Rotation2d position) {
    setGoalSupplier(() -> new State(position.getRadians(), 0));
  }

  public void setGoalSupplier(Supplier<State> positionSupplier) {
    this.goalSupplier = positionSupplier;
  }

  /** Whether wrist is within tolerance of setpoint */
  @AutoLogOutput(key = "Wrist/atGoal")
  public boolean atGoal() {
    return MathUtil.isNear(
        inputs.positionRad,
        goalSupplier.get().position,
        Units.degreesToRadians(toleranceDegrees.get()));
  }

  /** Get position in rotations */
  public Rotation2d getMeasuredRotation() {
    return new Rotation2d(inputs.positionRad);
  }

  /** Get setpoint in rotations */
  public Rotation2d getGoalRotations() {
    return new Rotation2d(goalSupplier.get().position);
  }

  @AutoLogOutput(key = "Wrist/goalDegrees")
  public double getGoalDegrees() {
    return Units.radiansToDegrees(goalSupplier.get().position);
  }

  @AutoLogOutput(key = "Wrist/measuredDegrees")
  public double getMeasuredDegrees() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  public State getGoalSupplier() {
    return goalSupplier.get();
  }

  public State getSetpoint() {
    return setpoint;
  }

  public void setSlowModeSupplier(BooleanSupplier slowModeSupplier) {
    this.slowModeSupplier = slowModeSupplier;
  }
}
