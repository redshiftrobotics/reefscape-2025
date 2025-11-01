package frc.robot.subsystems.rangefinder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Rangefinder extends SubsystemBase {
  private final RangefinderIO io;
  private final RangefinderIOInputsAutoLogged inputs = new RangefinderIOInputsAutoLogged();

  public Rangefinder(RangefinderIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rangefinder", inputs);
  }

  /** Get the measurement in millimetres */
  public Optional<Integer> getMeasurement() {
    return io.getMeasurementMillimetres();
  }
}
