package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the CORAL wrist subsystem. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double setpoint;
    public double position;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(WristIOInputs inputs) {}

  /** Set the setpoint in revolutions. */
  default void goTo(double setpoint) {}

  default boolean atSetpoint() {
    return false;
  }
}
