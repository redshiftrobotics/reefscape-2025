package frc.robot.subsystems.superstructure.intake.sensor;

import java.util.function.BooleanSupplier;

public class SensorIOSim implements SensorIO {

  private BooleanSupplier hasItemSupplier = () -> false;

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = true;

    inputs.altDetected = inputs.detected = hasItemSupplier.getAsBoolean();
    inputs.rawValue = inputs.detected ? 1.0 : 0.0;
    inputs.rawVolts = inputs.detected ? 5.0 : 4.0;
  }

  @Override
  public void setSimulationSource(BooleanSupplier hasItemSupplier) {
    this.hasItemSupplier = hasItemSupplier;
  }
}
