package frc.robot.subsystems.superstructure.intake.sensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Sensor extends SubsystemBase {

  private final SensorIO io;
  private final SensorIOInputsAutoLogged inputs = new SensorIOInputsAutoLogged();

  private final Alert disconnectedAlert =
      new Alert("Intake sensor disconnected!", Alert.AlertType.kWarning);

  private final Debouncer debouncer = new Debouncer(0.03, DebounceType.kBoth);
  private boolean hasItem = false;

  public Sensor(SensorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    hasItem = debouncer.calculate(inputs.detected);

    Logger.processInputs("Sensor", inputs);

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
