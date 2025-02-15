package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double speed;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setMotorSpeed(double speed) {}
}
