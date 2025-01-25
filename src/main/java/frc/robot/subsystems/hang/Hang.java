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
    Logger.processInputs("Hang Arm", inputs);
  }
}
