package frc.robot.subsystems.rangefinder;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface RangefinderIO {
  @AutoLog
  public static class RangefinderIOInputs {
    public int measurementMillimetres = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(RangefinderIOInputs inputs) {}

  public default Optional<Integer> getMeasurementMillimetres() {
    return Optional.empty();
  }
}
