package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class SensorIOBeam implements SensorIO {

  private final AnalogInput input = new AnalogInput(3);

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.rawValue = input.getValue();
    inputs.connected = inputs.rawValue > 10;
    inputs.detected = inputs.rawValue < 200;
  }
}
