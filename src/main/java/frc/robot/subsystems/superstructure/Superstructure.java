package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured", Color.kYellow);
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint", Color.kAliceBlue);
  private final SuperstructureVisualizer goalVisualizer =
      new SuperstructureVisualizer("Goal", Color.kGreen);

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  public Command scoreL1() {
    return elevator.runEnd(
        () -> elevator.setGoalHeightMeters(Units.inchesToMeters(18)),
        () -> elevator.setGoalHeightMeters(0));
  }

  public Command scoreL2() {
    return elevator.runEnd(
        () -> elevator.setGoalHeightMeters(Units.inchesToMeters(31.875)),
        () -> elevator.setGoalHeightMeters(0));
  }

  public Command scoreL3() {
    return elevator.runEnd(
        () -> elevator.setGoalHeightMeters(Units.inchesToMeters(47.625)),
        () -> elevator.setGoalHeightMeters(0));
  }

  public Command scoreL4() {
    return elevator.runEnd(
        () -> elevator.setGoalHeightMeters(Units.inchesToMeters(72)),
        () -> elevator.setGoalHeightMeters(0));
  }

  public Command stow() {
    return elevator.runOnce(() -> elevator.setGoalHeightMeters(0));
  }

  @Override
  public void periodic() {
    measuredVisualizer.update(elevator.getHeightMeters());
    setpointVisualizer.update(elevator.getSetpoint().position);
    goalVisualizer.update(elevator.getGoalHeightMeters());
  }
}
