package frc.robot.subsystems.rangefinder;

import au.grapplerobotics.LaserCan;
import java.util.Optional;

public class RangefinderIOHardware implements RangefinderIO {
  LaserCan sensor;

  public RangefinderIOHardware(int id) {
    sensor = new LaserCan(id);
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
