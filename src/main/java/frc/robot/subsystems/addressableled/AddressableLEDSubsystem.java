package frc.robot.subsystems.addressableled;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private AddressableLEDBufferView ledViews[];
  private LEDPattern[] currentPatterns;

  public AddressableLEDSubsystem() {
    // Create strip and buffer
    led = new AddressableLED(AddressableLEDConstants.LED_STRIP_PORT);
    ledBuffer = new AddressableLEDBuffer(AddressableLEDConstants.LED_COUNT);
    led.setLength(AddressableLEDConstants.LED_COUNT);

    //Create current pattern trackers
    currentPatterns = new LEDPattern[AddressableLEDConstants.SECTIONS.length];

    // Create section views
    ledViews = new AddressableLEDBufferView[AddressableLEDConstants.SECTIONS.length];
    for (int i = 0; i < ledViews.length; i++) {
      ledViews[i] =
          new AddressableLEDBufferView(
              ledBuffer,
              AddressableLEDConstants.SECTIONS[i].low(),
              AddressableLEDConstants.SECTIONS[i].high());
    }
  }

  // Periodically update the LED strip
  @Override
  public void periodic() {
    for(int i = 0; i < ledViews.length; i++) {
      currentPatterns[i].applyTo(ledViews[i]);
    }
    led.setData(ledBuffer);
  }

  // Apply a color pattern to a section of the LED strip
  public void applySectionedPattern(LEDPattern pattern, int section) {
    if (section < 0 || section >= ledViews.length) return;
    pattern.applyTo(ledViews[section]);
    currentPatterns[section] = pattern;
  }

  // Apply a color pattern to a section of the LED strip
  public void applyPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    for(int i = 0; i < currentPatterns.length; i++) {
      currentPatterns[i] = pattern;
    }
  }
}
