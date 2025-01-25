package frc.robot.subsystems.hang;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    //
  }

  public void updateInputs(HangIOInputs inputs);

  /**
   * Set the desired angle for the arm to travel to.
   *
   * @param setpoint The desired angle for the arm to travel to.
   */
  public void setSetpoint(Rotation2d setpoint);

  /**
   * Get the desired angle for the arm to travel to.
   *
   * @return The angle the arm is currently traveling to.
   */
  public double getSetpoint();

  /**
   * Get the current position of the arm.
   *
   * @return The current position of the arm.
   */
  public double getPosition();
}
