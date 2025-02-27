package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.addressableled.AddressableLEDSubsystem;

public class SetAddressableLEDPattern extends Command {
  private final AddressableLEDSubsystem ledSystem;

  // If this is -1, that means that this command is targeting the whole strip
  private final int section;

  private final LEDPattern pattern;

  /**
   * @param led Addressable LED subsystem to use
   * @param pattern Pattern to apply when command run
   * @param section Section of LED strip to apply pattern to (index into
   *     AddressableLEDConstants.SECTIONS)
   */
  public SetAddressableLEDPattern(
      AddressableLEDSubsystem ledSystem, LEDPattern pattern, int section) {
    this.section = section;
    this.pattern = pattern;
    this.ledSystem = ledSystem;
    addRequirements(ledSystem);
  }

  /**
   * @param led Addressable LED subsystem to use
   * @param pattern Pattern to apply when command run
   */
  public SetAddressableLEDPattern(AddressableLEDSubsystem ledSystem, LEDPattern pattern) {
    section = -1;
    this.pattern = pattern;
    this.ledSystem = ledSystem;
    addRequirements(ledSystem);
  }

  @Override
  public void execute() {
    if (section < 0) {
      ledSystem.applyPattern(pattern);
    } else {
      ledSystem.applySectionedPattern(pattern, section);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
