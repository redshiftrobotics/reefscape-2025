package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motorConnected = false;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double[] appliedVolts = new double[0];
    public double[] supplyCurrentAmps = new double[0];
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setGoalPosition(double positionRad, double feedforward) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enable) {}
}
