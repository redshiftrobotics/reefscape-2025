package frc.robot.subsystems.addressableled;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class AddressableLEDConstants {
  /**
   * @param low The lower bound of the range
   * @param high The upper bound of the range
   */
  public record Range(int low, int high) {}

  // TODO: Implement real values
  public static final int LED_COUNT = 64;
  public static final Distance LED_DENSITY = Meters.of(1.0 / LED_COUNT);
  public static final int LED_STRIP_PORT = 0;

  /**
   * @apiNote The lower bound of the range represents the lowest index LED for this section, and the
   *     upper bound represents the highest index LED
   */
  public static final Range SECTIONS[] = {new Range(0, LED_COUNT)};
}
