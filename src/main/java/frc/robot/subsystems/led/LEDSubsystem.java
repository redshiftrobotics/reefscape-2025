package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private record Range(int low, int high) {}

  // Note: change me to real values later
  private static final int LED_COUNT = 64;
  private static final int LED_PORT = 0;
  private static final Range SECTION_RANGES[] = {new Range(0, LED_COUNT)};

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private AddressableLEDBufferView ledViews[];

  public LEDSubsystem() {
    // Create strip and buffer
    led = new AddressableLED(LED_PORT);
    ledBuffer = new AddressableLEDBuffer(LED_COUNT);
    led.setLength(LED_COUNT);

    // Create section views
    ledViews = new AddressableLEDBufferView[SECTION_RANGES.length];
    for (int i = 0; i < ledViews.length; i++) {
      ledViews[i] =
          new AddressableLEDBufferView(ledBuffer, SECTION_RANGES[i].low, SECTION_RANGES[i].high);
    }
  }

  // Periodically update the LED strip
  public void periodic() {
    led.setData(ledBuffer);
  }

  // Apply a color pattern to a section of the LED strip
  public void applySectionedPattern(LEDPattern pattern, int section) {
    if (section < 0 || section >= ledViews.length) return;
    pattern.applyTo(ledViews[section]);
  }

  // Apply a color pattern to a section of the LED strip
  public void applyPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
  }
}
