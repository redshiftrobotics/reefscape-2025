package frc.robot.subsystems.addressableled;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class AddressableLEDConstants {
  /**
   * This record is used to represent a range of LEDs that are grouped together and exhibit one pattern
   * @param low The lowest index LED in the range
   * @param high The highest index LED in the range
   */
  public record Range(int low, int high) {}

  // TODO: Replace these with real values
  public static final int LED_COUNT = 60;
  public static final Distance LED_DENSITY = Meters.of(1.0 / 60.0);
  public static final int LED_STRIP_PORT = 0;

  /**
   * @apiNote The lower bound of the range represents the lowest index LED for this section, and the
   *     upper bound represents the highest index LED
   *     <h2>Sections</h2>
   *     Assume front of robot refers to battery side
   *     <ol>
   *       <li>Front
   *       <li>Left
   *       <li>Back
   *       <li>Right
   *       <li>Superstructure Left
   *       <li>Superstructure Right
   *     </ol>
   */
  public static final Range SECTIONS[] = {
    new Range(0, 9),
    new Range(10, 19),
    new Range(20, 29),
    new Range(30, 39),
    new Range(40, 49),
    new Range(50, 59)
  };
}
