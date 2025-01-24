package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem;

public class SetLightPattern extends Command {
  private LEDSubsystem ledSystem;
  private double pattern;

  // Set this to -1 to have the command apply to all strips
  private int stripID;

  public SetLightPattern(LEDSubsystem led, int strip, double pattern) {
    if (strip < -1)
      throw new IllegalArgumentException(
          "LED configuration commands may ONLY use -1 or higher for strip IDs!");
    ledSystem = led;
    this.pattern = pattern;
  }

  public SetLightPattern(LEDSubsystem led, double pattern) {
    this(led, -1, pattern);
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

  // Since executing the command is a one-time thing, we always report being done instantly

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
