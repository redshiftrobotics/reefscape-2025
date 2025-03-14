package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class SensorIOBeam implements SensorIO {

  public static final int SIGNAL_SENSOR_OCCUPIED = 5;
  public static final double SENSOR_VOLTAGE_TOLERANCE = 0.4;

  private final AnalogInput input = new AnalogInput(3);

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = true;

    inputs.rawValue = input.getValue();
    inputs.detected = inputs.rawValue < 200;
  }
}
