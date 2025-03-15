package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;

public class SensorIOBeam implements SensorIO {

  private final AnalogInput input = new AnalogInput(3);

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.rawVolts = input.getVoltage();
    inputs.rawValue = input.getValue();

    inputs.connected = inputs.rawValue > 10;
    inputs.detected = inputs.rawValue < 200;

    inputs.altDetected = MathUtil.isNear(inputs.rawVolts, 5, 0.1);
  }
}
