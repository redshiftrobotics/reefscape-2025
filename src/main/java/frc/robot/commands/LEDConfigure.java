package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem;

public class LEDConfigure extends Command {
  private LEDSubsystem ledSystem;
  private double pattern;

  //Set this to -1 to have the command apply to all strips
  private int stripID;

  public LEDConfigure(LEDSubsystem led, int strip, double patt) {
    if (strip < -1)
      throw new IllegalArgumentException("LED configuration commands may ONLY use -1 or higher for strip IDs!");
    ledSystem = led;
    pattern = patt;
  }

  public LEDConfigure(LEDSubsystem led, double patt) {
    this(led, -1, patt);
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    if (stripID == -1) {
      ledSystem.applyPatternToAll(pattern);
    } else {
      ledSystem.applyPatternTo(stripID, pattern);
    }
  }

  //Since executing the command is a one-time thing, we always report being done instantly

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
  
}
