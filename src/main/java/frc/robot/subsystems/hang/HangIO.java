package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    public double armSetpoint = 0.0;
    public double armPosition = 0.0;
  }

  public default void updateInputs(HangIOInputs inputs) {}

  /**
   * Set the desired angle for the arm to travel to.
   *
   * @param setpoint The desired angle for the arm to travel to.
   */
  public default void setSetpoint(double setpoint) {}

  /**
   * Get the desired angle for the arm to travel to.
   *
   * @return The angle the arm is currently traveling to.
   */
  public default double getSetpoint() {
    return 0;
  }

  /**
   * Get the current position of the arm.
   *
   * @return The current position of the arm.
   */
  public default double getPosition() {
    return 0;
  }
}
