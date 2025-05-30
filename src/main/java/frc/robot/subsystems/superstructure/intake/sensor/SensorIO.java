package frc.robot.subsystems.superstructure.intake.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
  @AutoLog
  public static class SensorIOInputs {
    boolean connected = false;

    double rawVolts = 0.0;
    double rawValue = 0.0;

    boolean detected = false;
    boolean altDetected = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(SensorIOInputs inputs) {}

  default void simulateItemDesire() {}

  default void simulatedItemNow() {}

  default void simulateItemEjection() {}
}
