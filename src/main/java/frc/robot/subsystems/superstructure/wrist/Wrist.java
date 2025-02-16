package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Mechanism at end of elevator to move intake/ */
public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  // TODO: This is more competant than the way its done in the hang arm
  private final WristVisualizer currentWristVisualizer =
      new WristVisualizer("Wrist/CurrentPosition", 64, 64, 32, 0, Color.kGreen);
  private final WristVisualizer targetWristVisualizer =
      new WristVisualizer("Wrist/SetpointPosition", 64, 64, 32, 0, Color.kOrange);

  /** Creates a new Template. */
  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    currentWristVisualizer.setRotations(inputs.position);
    targetWristVisualizer.setRotations(inputs.setpoint);

    Logger.processInputs("Wrist", inputs);
  }

  public void goTo(double setpoint) {
    io.goTo(setpoint);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }
}
