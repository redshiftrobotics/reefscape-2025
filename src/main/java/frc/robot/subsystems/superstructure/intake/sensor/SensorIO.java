package frc.robot.subsystems.superstructure.intake.sensor;

import org.littletonrobotics.junction.AutoLog;

public class SensorIO {
  @AutoLog
  public static class SensorIOInputs {
    public boolean hasItem;
    public double distance;
  }

  public void updateInputs(SensorIO inputs) {}
}
