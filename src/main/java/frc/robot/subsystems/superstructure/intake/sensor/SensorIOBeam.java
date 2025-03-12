package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;

public class SensorIOBeam implements SensorIO {

  public static final int SIGNAL_SENSOR_OCCUPIED = 5;
  public static final double SENSOR_VOLTAGE_TOLERANCE = 0.4;

  private final AnalogInput input = new AnalogInput(0);
  private final Debouncer debouncer = new Debouncer(0.1);

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = true;

    inputs.rawValue = input.getValue();
    inputs.rawDetected =
        MathUtil.isNear(inputs.rawValue, SIGNAL_SENSOR_OCCUPIED, SENSOR_VOLTAGE_TOLERANCE);

    inputs.hasItem = debouncer.calculate(inputs.rawDetected);
  }
}
