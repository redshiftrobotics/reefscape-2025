package frc.robot.subsystems.rangefinder;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

import java.util.Optional;

public class RangefinderIOHardware implements RangefinderIO {
  private final LaserCan sensor;

  public RangefinderIOHardware(int id) {
    sensor = new LaserCan(id);

    try {
      sensor.setRangingMode(RangingMode.SHORT);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(RangefinderIOInputs inputs) {
    inputs.measurementMillimetres = getMeasurementMillimetres().orElse(-1);
  }

  @Override
  public Optional<Integer> getMeasurementMillimetres() {
    LaserCan.Measurement measurement = sensor.getMeasurement();

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return Optional.of(measurement.distance_mm);
    } else {
      return Optional.empty();
    }
  }
}
