package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

/** Interface for the IO layers of the Template subsystem. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double speedLeft;
    public double speedRight;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setLeftMotor(double speed) {}

  default void setRightMotor(double speed) {}

  default void stopMotors() {}
}
