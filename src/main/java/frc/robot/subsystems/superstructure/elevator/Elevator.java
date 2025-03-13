package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Elevator of robot, moves around carriage with intake */
public class Elevator extends SubsystemBase {

  private static final LoggedTunableNumberFactory factory =
      new LoggedTunableNumberFactory("Elevator");

  private static final LoggedTunableNumber kP = factory.getNumber("kP", ElevatorConstants.pid.kP());
  private static final LoggedTunableNumber kI = factory.getNumber("kI", ElevatorConstants.pid.kI());
  private static final LoggedTunableNumber kD = factory.getNumber("kD", ElevatorConstants.pid.kD());

  private static final LoggedTunableNumber kS =
      factory.getNumber("kS", ElevatorConstants.feedForward.kS());
  private static final LoggedTunableNumber kG =
      factory.getNumber("kG", ElevatorConstants.feedForward.kG());
  private static final LoggedTunableNumber kV =
      factory.getNumber("kV", ElevatorConstants.feedForward.kV());
  private static final LoggedTunableNumber kA =
      factory.getNumber("kA", ElevatorConstants.feedForward.kA());

  private static final LoggedTunableNumber maxVelocity =
      factory.getNumber("MaxVelocity", ElevatorConstants.maxCarriageVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      factory.getNumber("MaxAcceleration", ElevatorConstants.maxCarriageAcceleration);

  private static final LoggedTunableNumber tolerance =
      factory.getNumber(
          "ToleranceInches", Units.metersToInches(ElevatorConstants.carriagePositionTolerance));

  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      factory.getNumber("/StaticCharacterizationVelocityThresh", 0.1);

  private static final LoggedTunableNumber hardStopVelocityThresh =
      factory.getNumber("/StaticCharacterizationVelocityThresh", 0.05);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kError);

  private final Alert followMotorFollowingAlert =
      new Alert("Elevator follower not following!", Alert.AlertType.kError);

  private final Alert zeroingAlert = new Alert("Elevator needs zeroing!", Alert.AlertType.kInfo);

  private Debouncer disabledDebouncer = new Debouncer(3, DebounceType.kRising);

  enum IdleModeControl {
    BRAKE,
    AUTO,
    COAST
  }

  private boolean zeroedHeightEncoder = true;

  private IdleModeControl coastMode = IdleModeControl.AUTO;
  private boolean brakeModeEnabled = true;

  private boolean stoppedProfile = false;

  private State setpoint = new State();
  private Supplier<State> goalSupplier = State::new;

  private TrapezoidProfile profile;
  private ElevatorFeedforward feedforward;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    profile = new TrapezoidProfile(new Constraints(maxVelocity.get(), maxAcceleration.get()));
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.feedForward.kS(),
            ElevatorConstants.feedForward.kG(),
            ElevatorConstants.feedForward.kV(),
            ElevatorConstants.feedForward.kA());

    io.setPID(kP.get(), kI.get(), kD.get());

    io.setBrakeMode(brakeModeEnabled);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), (values) -> io.setPID(values[0], values[1], values[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) ->
            feedforward = new ElevatorFeedforward(values[0], values[1], values[2], values[3]),
        kS,
        kG,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> profile = new TrapezoidProfile(new Constraints(values[0], values[1])),
        maxVelocity,
        maxAcceleration);

    brakeModeEnabled =
        switch (coastMode) {
          case BRAKE -> true;
          case AUTO -> !disabledDebouncer.calculate(DriverStation.isDisabled());
          case COAST -> false;
        };

    io.setBrakeMode(brakeModeEnabled);

    Logger.recordOutput("Elevator/breakModeEnabled", brakeModeEnabled);

    if (!stoppedProfile && brakeModeEnabled && DriverStation.isEnabled()) {
      Logger.recordOutput("Elevator/shouldRunProfiled", true);

      State goal =
          new State(
              MathUtil.clamp(goalSupplier.get().position, 0.0, ElevatorConstants.carriageMaxHeight),
              goalSupplier.get().velocity);

      setpoint = profile.calculate(Constants.LOOP_PERIOD_SECONDS, setpoint, goal);

      double feedforwardVolts = feedforward.calculate(setpoint.velocity);

      io.runPosition(setpoint.position / ElevatorConstants.drumRadius, feedforwardVolts);

      Logger.recordOutput("Elevator/Feedforward/Volts", feedforwardVolts);
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goal.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goal.velocity);

    } else {
      Logger.recordOutput("Elevator/Profile/ShouldRunProfiled", false);

      Logger.recordOutput("Elevator/FeedForward/Volts", 0);
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }

    motorDisconnectedAlert.set(!inputs.motorConnected);
    followMotorFollowingAlert.set(!inputs.followerMotorFollowing);
    zeroingAlert.set(!zeroedHeightEncoder);
  }

  // --- Commands ---

  public Command runPositionPrepare(double position) {
    return runOnce(() -> setGoalHeightMeters(position));
  }

  public Command runPositionPrepare(DoubleSupplier position) {
    return runOnce(() -> setGoalSupplier(() -> new State(position.getAsDouble(), 0)));
  }

  // --- Methods ---

  public void setGoalSupplier(Supplier<State> goal) {
    this.goalSupplier = goal;
  }

  /** Sets the goal height of the elevator in meters */
  public void setGoalHeightMeters(double position) {
    setGoalSupplier(() -> new State(position, 0));
  }

  /** Gets the goal height of the elevator in meters */
  @AutoLogOutput(key = "Elevator/goalHeightMeters")
  public double getGoalHeightMeters() {
    return goalSupplier.get().position;
  }

  /** Gets the current measured height of the elevator */
  @AutoLogOutput(key = "Elevator/measuredHeightMeters")
  public double getMeasuredHeightMeters() {
    return inputs.positionRad * ElevatorConstants.drumRadius;
  }

  @AutoLogOutput(key = "Elevator/measuredHeightPercent")
  public double getHeightPercent() {
    return getMeasuredHeightMeters() / ElevatorConstants.carriageMaxHeight;
  }

  @AutoLogOutput(key = "Elevator/atGoal")
  public boolean atGoalHeight() {
    return MathUtil.isNear(
        getMeasuredHeightMeters(), getGoalHeightMeters(), Units.inchesToMeters(tolerance.get()));
  }

  public State getGoal() {
    return goalSupplier.get();
  }

  public State getSetpoint() {
    return setpoint;
  }

  public void setCoastMode(IdleModeControl coastMode) {
    this.coastMode = coastMode;
  }

  @AutoLogOutput(key = "Elevator/needsZeroing")
  public boolean needsZeroing() {
    return !zeroedHeightEncoder;
  }

  public Command zeroHeight() {
    return Commands.runEnd(
            () -> {
              io.runOpenLoop(-0.1);
              System.out.println("set -0.1");
            },
            io::stop)
        .until(() -> inputs.velocityRadPerSec <= hardStopVelocityThresh.get())
        .andThen(
            () -> {
              io.zeroEncoder();
              zeroedHeightEncoder = true;
            })
        .andThen(Commands.print("Zeroed Elevator Encoder"))
        .beforeStarting(() -> stoppedProfile = true)
        .finallyDo(() -> stoppedProfile = false)
        .withName("Zero Elevator Height");
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    final Timer timer = new Timer();
    return Commands.run(
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> inputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
        .beforeStarting(
            () -> {
              stoppedProfile = true;
              timer.restart();
            })
        .finallyDo(
            () -> {
              stoppedProfile = false;
              timer.stop();
              System.out.println("CharacterizationOutput: " + state.characterizationOutput);
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
