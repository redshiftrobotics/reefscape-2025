package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double positionRad;
    public double setpointRad;
    public double appliedVolts;
    public double angularVelocity;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Starts moving the wrist to the specified position in radians */
  public default void moveTo(double pos, double feedforward) {}

  /** Set PID constants. */
  public default void configurePID(double p, double i, double d) {}

  /** Stop the motor */
  public default void stop() {}
}
