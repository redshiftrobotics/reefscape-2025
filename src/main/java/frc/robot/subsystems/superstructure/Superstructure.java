package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
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
    L4(false);

    // L2_ALGAE(true),
    // L3_ALGAE(true);

    public boolean isLevel() {
      return this == L1 || this == L2 || this == L3 || this == L4;
    }

    public boolean isAlgae() {
      return algae;
    }

    public State asAlgae() {
      return switch (this) {
          // case L2 -> L2_ALGAE;
          // case L3 -> L3_ALGAE;
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

  public Command run() {
    return elevator.defer(() -> run(goal));
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
      case INTAKE -> prepareIntake();
    };
  }

  public Command runWheels(State newGoal) {
    return switch (newGoal) {
      case STOW -> stowLow();
      case L1 -> scoreL1Wheels();
      case L2 -> scoreLevelWheels();
      case L3 -> scoreLevelWheels();
      case L4 -> scoreLevelWheels();
      case INTAKE -> intakeWheels();
      default -> Commands.none();
    };
  }

  public Command run(State goal) {
    return runPrepare(goal)
        .andThen(coralIntake.runOnce(() -> coralIntake.setMotors(-0.1)))
        .andThen(Commands.waitUntil(() -> elevator.atGoalHeight() && coralWrist.atGoal()))
        .andThen(runWheels(goal));
  }

  public Command scoreLevelWheels() {
    return coralIntake.intake(1.0).withTimeout(1.0);
  }

  public Command intakeWheels() {
    return coralIntake.intake(-1.0);
  }

  public Command scoreL1Wheels() {
    return coralIntake.intake(1.0, 0.4).withTimeout(1.0);
  }

  public Command prepareL1() {
    return Commands.parallel(elevator.runPositionPrepare(0), coralWrist.runPrepare(-7.690));
  }

  public Command prepareL2() {
    return Commands.parallel(
        elevator.runPositionPrepare(0.599 + Units.inchesToMeters(1)),
        coralWrist.runPrepare(-7.690));
  }

  public Command prepareL3() {
    return Commands.parallel(
        elevator.runPositionPrepare(1.033 + Units.inchesToMeters(1)),
        coralWrist.runPrepare(-8.262));
  }

  public Command prepareL4() {
    return Commands.parallel(elevator.runPositionPrepare(0), coralWrist.runPrepare(-8.262));
  }

  public Command prepareIntake() {
    return Commands.parallel(
        (elevator.runPositionPrepare(0.218 + Units.inchesToMeters(1.5))),
        coralWrist.runPrepare(-4.214));
  }

  public Command stowLow() {
    return Commands.parallel(
        elevator.runStow(),
        algaeWrist.runPrepare(Units.degreesToRotations(20)),
        coralWrist.runPrepare(-1.619));
  }

  public Command stowLowWait() {
    return Commands.parallel(
        Commands.waitSeconds(0.3).andThen(elevator.runStow()),
        algaeWrist.runPrepare(Units.degreesToRotations(20)),
        coralWrist.runPrepare(-1.619));
  }

  // public Command stowHigh() {
  //   return Commands.parallel(
  //       elevator.runStow(),
  //       algaeWrist.runPrepare(algaeHighStow),
  //       coralWrist.runPrepare(coralHighStow));
  // }

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
