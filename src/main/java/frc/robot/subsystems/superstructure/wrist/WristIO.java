package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the CORAL wrist subsystem. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean motorConnected = false;

    public double positionRotations;
    public double positionDegrees;
    public double velocityRPM;

    public double appliedVolts;
    public double supplyCurrentAmps;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(WristIOInputs inputs) {}

  /** Set the setpoint in revolutions. */
  default void runPosition(double setpoint, double feedfoward) {}

  /** Set the PID constants. */
  default void setPID(double kP, double kI, double kD) {}

  /** Enable break mode */
  public default void setBrakeMode(boolean enable) {}
}
