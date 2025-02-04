package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double velocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}
}
