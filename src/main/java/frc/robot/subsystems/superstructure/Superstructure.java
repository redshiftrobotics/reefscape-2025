package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.wrist.Wrist;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist coralWrist;
  private final Wrist algaeWrist;
  private final Intake coralIntake;
  private final Intake algaeIntake;

  public static enum State {
    STOW,
    INTAKE,

    L1(false),
    L2(false),
    L3(false),
    L4(false),

    L2_ALGAE(true),
    L3_ALGAE(true),
    L4_ALGAE(true);

    public boolean isLevel() {
      return this == L1 || this == L2 || this == L3 || this == L4;
    }

    public boolean isAlgae() {
      return algae;
    }

    public State asAlgae() {
      return switch (this) {
        case L2 -> L2_ALGAE;
        case L3 -> L3_ALGAE;
        case L4 -> L4_ALGAE;
        default -> this;
      };
    }

    public boolean algae;

    private State() {
      this.algae = false;
    }

    private State(boolean algae) {
      this.algae = algae;
    }
  }

  private State goal = State.STOW;

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured", Color.kYellow);
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint", Color.kAliceBlue);
  private final SuperstructureVisualizer goalVisualizer =
      new SuperstructureVisualizer("Goal", Color.kLime);

  public Superstructure(
      Elevator elevator,
      Wrist coralWrist,
      Wrist algaeWrist,
      Intake coralIntake,
      Intake algaeIntake) {
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.algaeWrist = algaeWrist;
    this.coralIntake = coralIntake;
    this.algaeIntake = algaeIntake;
  }

  public Command prepare() {
    return elevator.defer(() -> runPrepare(goal));
  }

  public Command setNextPrepare(State newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal);
  }

  public Command runPrepare(State newGoal) {
    return switch (newGoal) {
      case STOW -> stowLow();
      case L1 -> prepareL1();
      case L2 -> prepareL2();
      case L3 -> prepareL3();
      case L4 -> prepareL4();
      case L2_ALGAE -> prepareAlgaeL2();
      case L3_ALGAE -> prepareAlgaeL3();
      case L4_ALGAE -> prepareAlgaeL4();
      case INTAKE -> prepareIntake();
    };
  }

  public Command run(State goal) {
    return runPrepare(goal)
        .andThen(Commands.waitUntil(() -> elevator.atGoalHeight() && coralWrist.atGoal()))
        .andThen(
            goal.isAlgae()
                ? algaeIntake.intake(1.0).withTimeout(1)
                : coralIntake.intake(1.0).withTimeout(1));
  }

  private static final double algaeHighStow = Units.degreesToRotations(160);
  private static final double coralHighStow = Units.degreesToRotations(160);

  public Command prepareL1() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight / 4.0),
        algaeWrist.runPrepare(algaeHighStow),
        coralWrist.runPrepare(Units.degreesToRotations(55)));
  }

  public Command prepareL2() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight / 2.0),
        algaeWrist.runPrepare(algaeHighStow),
        coralWrist.runPrepare(Units.degreesToRotations(35)));
  }

  public Command prepareL3() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight * (3.0 / 4.0)),
        algaeWrist.runPrepare(algaeHighStow),
        coralWrist.runPrepare(Units.degreesToRotations(35)));
  }

  public Command prepareL4() {
    return Commands.parallel(
        elevator.runOnce(() -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight)),
        algaeWrist.runPrepare(algaeHighStow),
        coralWrist.runPrepare(Units.degreesToRotations(90)));
  }

  public Command prepareAlgaeL1() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight / 4.0),
        coralWrist.runPrepare(coralHighStow),
        algaeWrist.runPrepare(Units.degreesToRotations(90)));
  }

  public Command prepareAlgaeL2() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight / 2.0),
        coralWrist.runPrepare(coralHighStow),
        algaeWrist.runPrepare(Units.degreesToRotations(90)));
  }

  public Command prepareAlgaeL3() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight * (3.0 / 4.0)),
        coralWrist.runPrepare(coralHighStow),
        algaeWrist.runPrepare(Units.degreesToRotations(90)));
  }

  public Command prepareAlgaeL4() {
    return Commands.parallel(
        elevator.runOnce(() -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight)),
        coralWrist.runPrepare(coralHighStow),
        algaeWrist.runPrepare(Units.degreesToRotations(90)));
  }

  public Command prepareIntake() {
    return Commands.parallel(
        elevator.runPositionPrepare(ElevatorConstants.carriageMaxHeight / 4.0),
        coralWrist.runPrepare(Units.degreesToRotations(55)));
  }

  public Command stowLow() {
    return Commands.parallel(
        elevator.runStow(),
        algaeWrist.runPrepare(Units.degreesToRotations(20)),
        coralWrist.runPrepare(Units.degreesToRotations(90)));
  }

  @Override
  public void periodic() {
    measuredVisualizer.update(
        elevator.getHeightMeters(),
        coralWrist.getMeasuredPosition(),
        algaeWrist.getMeasuredPosition(),
        coralIntake.isIntakeRunning(),
        algaeIntake.isIntakeRunning());
    setpointVisualizer.update(
        elevator.getSetpoint().position,
        coralWrist.getSetpoint(),
        algaeWrist.getSetpoint(),
        coralIntake.isIntakeRunning(),
        algaeIntake.isIntakeRunning());
    goalVisualizer.update(
        elevator.getGoalHeightMeters(),
        coralWrist.getSetpoint(),
        algaeWrist.getSetpoint(),
        coralIntake.isIntakeRunning(),
        algaeIntake.isIntakeRunning());
  }
}
