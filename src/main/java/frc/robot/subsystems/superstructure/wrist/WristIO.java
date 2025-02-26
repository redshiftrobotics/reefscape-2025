package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the CORAL wrist subsystem. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    // Rotations, probably
    // TODO: Nitpick: I would put unit in variable name, like setpointRotations or setpointRads
    // Maybe even use Angle from WPILib units library
    public double setpoint;
    public double position;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(WristIOInputs inputs) {}

  /** Set the setpoint in revolutions. */
  // TODO: Nitpick: convention in robotics code is to use runPosition to show you are setting a target in a "set and forget" way,
  // but goTo is fine and not bad at all.
  default void goTo(double setpoint) {}

  // TODO: Could this behavior be done in Wrist.java, seems like logic to me
  default boolean atSetpoint() {
    return false;
  }

  default void setPid(double kP, double kI, double kD) {}
}
