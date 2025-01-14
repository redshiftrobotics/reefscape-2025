package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem;

public class LEDPatternCommand extends Command {
  private final LEDSubsystem ledSystem;
  private final int
      section; // If this is -1, that means that this command is targeting the whole strip
  private final LEDPattern pattern;

  public LEDPatternCommand(LEDSubsystem led, LEDPattern pattern, int section) {
    this.section = section;
    this.pattern = pattern;
    ledSystem = led;
  }

  public LEDPatternCommand(LEDSubsystem led, LEDPattern pattern) {
    this(led, pattern, -1);
  }

  public void initialize() {}

  public void execute() {
    if (section < 0) {
      ledSystem.applyPattern(pattern);
    } else {
      ledSystem.applySectionedPattern(pattern, section);
    }
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean interrupted) {}
}
