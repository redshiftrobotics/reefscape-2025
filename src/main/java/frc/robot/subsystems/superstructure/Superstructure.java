package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured");
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint");
  private final SuperstructureVisualizer goalVisualizer = new SuperstructureVisualizer("Goal");

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    measuredVisualizer.update(elevator.getHeightMeters());
    setpointVisualizer.update(elevator.getSetpoint().position);
    goalVisualizer.update(elevator.getGoalHeightMeters());
  }
}
