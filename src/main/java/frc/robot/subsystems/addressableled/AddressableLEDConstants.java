package frc.robot.subsystems.addressableled;

public class AddressableLEDConstants {
  private record Range(int low, int high) {}

  // TODO: Implement real values
  public static final int LED_COUNT = 64;
  public static final int LED_STRIP_PORT = 0;
  public static final Range SECTIONS[] = {new Range(0, LED_COUNT)};
}
