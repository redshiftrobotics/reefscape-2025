package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the CORAL wrist subsystem. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double setpointRotations;
    public double positionRotations;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(WristIOInputs inputs) {}

  /** Set the setpoint in revolutions. */
  // TODO: Nitpick: convention in robotics code is to use runPosition to show you are setting a
  // target in a "set and forget" way,
  // but goTo is fine and not bad at all.
  default void runPosition(double setpoint) {}

  default void setPid(double kP, double kI, double kD) {}
}
