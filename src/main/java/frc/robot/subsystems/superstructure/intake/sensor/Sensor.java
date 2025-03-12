package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;

public class Sensor {

  private final SensorIO io;
  private final SensorIO.SensorIOInputs inputs = new SensorIO.SensorIOInputs();

  private final Alert disconnectedAlert = new Alert("Intake sensor disconnected!", Alert.AlertType.kWarning);

  private final Debouncer debouncer = new Debouncer(0.1);
  private boolean hasItem = false;

  public Sensor(SensorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    hasItem = debouncer.calculate(inputs.detected);

    disconnectedAlert.set(!inputs.connected);
  }

  public boolean isDetected() {
    return hasItem;
  }

  /** For simulation, request item */
  public void simulateItemRequest() {
    io.simulateItemDesire();
  }

  /** For simulation, eject item */
  public void simulateItemEjection() {
    io.simulateItemEjection();
  }
}
