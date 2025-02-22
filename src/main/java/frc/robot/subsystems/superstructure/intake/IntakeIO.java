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

  /**
   * Set both motor's speed.
   *
   * @param speed The speed to set both motors to.
   */
  default void setMotors(double speed) {
    setLeftMotor(speed);
    setRightMotor(speed);
  }

  /**
   * Get if the beam is currently broken according to the beam sensor, AKA if the intake is
   * occupied.
   *
   * @return True when there is a game element in the intake.
   */
  default boolean isOccupied() {
    return false;
  }

  default void stopMotors() {}
}
