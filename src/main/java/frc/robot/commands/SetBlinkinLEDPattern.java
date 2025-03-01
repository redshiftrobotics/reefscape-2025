package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.blinkinled.BlinkinLEDSubsystem;

public class SetBlinkinLEDPattern extends Command {
  private BlinkinLEDSubsystem ledSystem;
  private double pattern;

  /**
   * @apiNote If this is -1, that means that this command is targeting the whole strip
   */
  private int section;

  private boolean invalid;

  public SetBlinkinLEDPattern(BlinkinLEDSubsystem ledSystem, int section, double pattern) {
    invalid = (section < -1 || section >= ledSystem.stripCount);
    this.section = section;
    this.ledSystem = ledSystem;
    this.pattern = pattern;
  }

  public SetBlinkinLEDPattern(BlinkinLEDSubsystem led, double pattern) {
    this(led, -1, pattern);
  }

  @Override
  public void initialize() {
    if (invalid) return;
    if (section == -1) {
      ledSystem.applyPatternToAll(pattern);
    } else {
      ledSystem.applyPatternTo(section, pattern);
    }
  }

  // Since executing the command is a one-time thing, we always report being done instantly
  @Override
  public boolean isFinished() {
    return true;
  }
}
