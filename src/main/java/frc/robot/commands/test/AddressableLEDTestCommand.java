package frc.robot.commands.test;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.addressableled.AddressableLEDConstants;
import frc.robot.subsystems.addressableled.AddressableLEDSubsystem;

public class AddressableLEDTestCommand extends Command {
  private final AddressableLEDSubsystem ledSystem;
  private LEDPattern currentPattern;
  private int ticker = 0;
  private int testCount = 0;

  public AddressableLEDTestCommand(AddressableLEDSubsystem ledSystem) {
    this.ledSystem = ledSystem;
    addRequirements(ledSystem);
  }

  @Override
  public void initialize() {
    ticker = 0;
    testCount = 0;
    currentPattern =
        LEDPattern.rainbow(255, 128)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), AddressableLEDConstants.LED_DENSITY);
  }

  @Override
  public void execute() {
    if (ticker < 500) {
      ticker++;
    } else {
      ticker = 0;
      ledSystem.applySectionedPattern(LEDPattern.kOff, testCount);
      testCount++;
      if (testCount == AddressableLEDConstants.SECTIONS.length) {
        currentPattern = LEDPattern.solid(Color.kLimeGreen).breathe(Seconds.of(2));
        ledSystem.applyPattern(currentPattern);
      } else {
        ledSystem.applySectionedPattern(currentPattern, testCount);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return testCount < (AddressableLEDConstants.SECTIONS.length + 1);
  }

  @Override
  public void end(boolean interrupted) {
    ledSystem.applyPattern(LEDPattern.kOff);
  }
}
