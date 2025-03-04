package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the CORAL wrist subsystem. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double positionRotations;
    public double velocityRPM;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(WristIOInputs inputs) {}

  /** Set the setpoint in revolutions. */
  default void runPosition(double setpoint) {}

  /** Set the PID constants. */
  default void setPID(double kP, double kI, double kD) {}

  /** Enable break mode */
  public default void setBrakeMode(boolean enable) {}
}
