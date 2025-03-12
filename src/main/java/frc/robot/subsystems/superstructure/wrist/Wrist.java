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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
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

  private static final LoggedTunableNumber maxVelocity =
      factory.getNumber("maxVelocity", WristConstants.MAX_VELOCITY);
  private static final LoggedTunableNumber maxAcceleration =
      factory.getNumber("maxAcceleration", WristConstants.MAX_ACCELERATION);

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private TrapezoidProfile profile;
  private ArmFeedforward feedforward;

  private State setpoint = new State();
  private Supplier<State> goalSupplier = State::new;

  private final Alert motorConnectedAlert =
      new Alert("Wrist motor disconnected!", Alert.AlertType.kError);

  private final Debouncer disabledDebouncer = new Debouncer(3, DebounceType.kRising);

  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;

    profile = new TrapezoidProfile(new Constraints(maxVelocity.get(), maxAcceleration.get()));

    io.setPID(kP.get(), kI.get(), kD.get());
    io.setBrakeMode(true);

    this.feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist", inputs);

    io.setBrakeMode(!disabledDebouncer.calculate(DriverStation.isDisabled()));

    setpoint = profile.calculate(Constants.LOOP_PERIOD_SECONDS, setpoint, goalSupplier.get());

    double feedforwardVolts = feedforward.calculate(setpoint.position, setpoint.velocity);

    io.runPosition(setpoint.position, feedforwardVolts);

    Logger.recordOutput("Wrist/FeedForward/Volts", feedforwardVolts);
    Logger.recordOutput(
        "Wrist/FeedForward/MeasuredVolts[Test]", feedforward.calculate(inputs.positionRad, 0));

    Logger.recordOutput("Wrist/Profile/SetpointPositionRotations", setpoint.position);
    Logger.recordOutput("Wrist/Profile/SetpointVelocityRPM", setpoint.velocity);
    Logger.recordOutput("Wrist/Profile/GoalPositionRotations", goalSupplier.get().position);
    Logger.recordOutput("Wrist/Profile/GoalVelocityRPM", goalSupplier.get().velocity);

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

    motorConnectedAlert.set(!inputs.motorConnected);
  }

  public Command runPositionPrepare(Rotation2d position) {
    return runOnce(() -> setGoalRotation(position));
  }

  public Command runPosition(Rotation2d position) {
    return runPositionPrepare(position).andThen(Commands.waitUntil(this::atGoal));
  }

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
        inputs.positionRad, goalSupplier.get().position, WristConstants.TOLERANCE_DEGREES);
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

  public void setConstraints(double maxVelocity, double maxAcceleration) {
    profile = new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
  }

  public void resetContraints() {
    setConstraints(WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION);
  }
}
