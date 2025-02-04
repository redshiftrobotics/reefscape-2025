package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}
}
