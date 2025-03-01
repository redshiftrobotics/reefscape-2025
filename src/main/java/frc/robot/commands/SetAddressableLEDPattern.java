package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.addressableled.AddressableLEDSubsystem;

public class SetAddressableLEDPattern extends Command {
  private final AddressableLEDSubsystem ledSystem;

  /**
   * @apiNote If this is empty, that means that this command is targeting the whole strip
   */
  private final int[] sections;

  private final LEDPattern pattern;

  /**
   * @param led Addressable LED subsystem to use
   * @param pattern Pattern to apply when command run
   * @param section Section of LED strip to apply pattern to (index into
   *     AddressableLEDConstants.SECTIONS)
   */
  public SetAddressableLEDPattern(
      AddressableLEDSubsystem ledSystem, LEDPattern pattern, int... sections) {
    this.sections = sections;
    this.pattern = pattern;
    this.ledSystem = ledSystem;
    addRequirements(ledSystem);
  }

  /**
   * @param led Addressable LED subsystem to use
   * @param pattern Pattern to apply when command run
   */
  public SetAddressableLEDPattern(AddressableLEDSubsystem ledSystem, LEDPattern pattern) {
    sections = new int[0];
    this.pattern = pattern;
    this.ledSystem = ledSystem;
    addRequirements(ledSystem);
  }

  @Override
  public void execute() {
    if (sections.length <= 0) {
      ledSystem.applyPattern(pattern);
    } else {
      for (int i = 0; i < sections.length; i++) {
        ledSystem.applySectionedPattern(pattern, sections[i]);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
