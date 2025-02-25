package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class SetAddressableLEDPattern extends Command {
  private final AddressableLEDSubsystem ledSystem;
  private final int
      section; // If this is -1, that means that this command is targeting the whole strip
  private final LEDPattern pattern;

  /**
   * @param
   */
  public SetAddressableLEDPattern(AddressableLEDSubsystem led, LEDPattern pattern, int section) {
    this.section = section;
    this.pattern = pattern;
    ledSystem = led;
  }

  public SetAddressableLEDPattern(AddressableLEDSubsystem led, LEDPattern pattern) {
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
