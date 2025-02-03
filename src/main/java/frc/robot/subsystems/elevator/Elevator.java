package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateMotors();
  }

  /** Run open loop at the specified voltage. */
  public void goToStage(int stage) {
    switch (stage) {
      case 0:
        io.setGoalHeight(stage);
      case 1:
        io.setGoalHeight(ElevatorConstants.LEVEL_ONE_HEIGHT);
      case 2:
        io.setGoalHeight(ElevatorConstants.LEVEL_TWO_HEIGHT);
      case 3:
        io.setGoalHeight(ElevatorConstants.LEVEL_THREE_HEIGHT);
      case 4:
        io.setGoalHeight(ElevatorConstants.LEVEL_FOUR_HEIGHT);
    }
  }

  /** Reset. */
  public void reset() {
    goToStage(0);
    io.stop();
  }
}
