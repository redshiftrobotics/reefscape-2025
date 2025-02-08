package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  public Hang(HangIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hang", inputs);
  }

  /**
   * Set the desired angle for the arm to travel to.
   *
   * @param setpoint The desired angle for the arm to travel to.
   */
  public void setSetpoint(double setpoint) {
    io.setSetpoint(setpoint);
  }

  /**
   * Get the desired angle for the arm to travel to.
   *
   * @return The angle the arm is currently traveling to.
   */
  public double getSetpoint() {
    return io.getSetpoint();
  }

  /**
   * Get the current position of the arm.
   *
   * @return The current position of the arm.
   */
  public double getPosition() {
    return io.getPosition();
  }

  /** Stop the arm from moving. */
  public void stop() {
    io.stop();
  }
}
