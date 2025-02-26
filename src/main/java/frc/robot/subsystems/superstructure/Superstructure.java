package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.wrist.Wrist;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist wrist;

  public static enum State {
    STOW,
    L1,
    L2,
    L3,
    L4,
    INTAKE,
  }

  private State goal = State.STOW;

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured", Color.kYellow);
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint", Color.kAliceBlue);
  private final SuperstructureVisualizer goalVisualizer =
      new SuperstructureVisualizer("Goal", Color.kGreen);

  public Superstructure(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public Command prepare() {
    return elevator.defer(() -> runPrepare(goal));
  }

  public Command setNextPrepare(State newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal);
  }

  public Command runPrepare(State newGoal) {
    return switch (newGoal) {
      case STOW -> stow();
      case L1 -> prepareL1();
      case L2 -> prepareL2();
      case L3 -> prepareL3();
      case L4 -> prepareL4();
      case INTAKE -> prepareIntake();
    };
  }

  public Command prepareL1() {
    return Commands.parallel(
        elevator.runOnce(
            () -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight / 4.0)),
        wrist.runOnce(() -> wrist.goTo(Units.degreesToRotations(55))));
  }

  public Command prepareL2() {
    return Commands.parallel(
        elevator.runOnce(
            () -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight / 2.0)),
        wrist.runOnce(() -> wrist.goTo(Units.degreesToRotations(35))));
  }

  public Command prepareL3() {
    return Commands.parallel(
        elevator.runOnce(
            () -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight * (3.0 / 4.0))),
        wrist.runOnce(() -> wrist.goTo(Units.degreesToRotations(35))));
  }

  public Command prepareL4() {
    return Commands.parallel(
        elevator.runOnce(() -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight)),
        wrist.runOnce(() -> wrist.goTo(Units.degreesToRotations(90))));
  }

  public Command prepareIntake() {
    return Commands.parallel(
        elevator.runOnce(
            () -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight / 4.0)),
        wrist.runOnce(() -> wrist.goTo(Units.degreesToRotations(55))));
  }

  public Command stow() {
    return elevator.runOnce(() -> elevator.setGoalHeightMeters(0));
  }

  @Override
  public void periodic() {
    measuredVisualizer.update(elevator.getHeightMeters(), wrist.getPosition());
    setpointVisualizer.update(elevator.getSetpoint().position, wrist.getSetpoint());
    goalVisualizer.update(elevator.getGoalHeightMeters(), wrist.getSetpoint());
  }
}
