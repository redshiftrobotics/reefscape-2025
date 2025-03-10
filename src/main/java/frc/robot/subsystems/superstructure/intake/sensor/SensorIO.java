package frc.robot.subsystems.superstructure.intake.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
  @AutoLog
  public static class SensorIOInputs {
    boolean connected = false;

    double rawValue = 0.0;
    boolean rawDetected = false;

    boolean hasItem;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(SensorIOInputs inputs) {}
}
