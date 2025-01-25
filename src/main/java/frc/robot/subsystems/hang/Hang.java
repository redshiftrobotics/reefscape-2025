package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  public Hang(HangIO io) {
    this.io = io;
  }
}
