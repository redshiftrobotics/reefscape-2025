package frc.robot.subsystems.led;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to manage multiple LED strips with Blinken LED Drivers.
 *
 * <p>LEDs will retain their last set pattern until a new pattern is set, unless a default pattern
 * is given though a default command.
 */
public class LEDSubsystem extends SubsystemBase {

  private LEDStripIO[] strips;
  private LEDStripIOInputsAutoLogged[] inputs;

  private final Debouncer setupDebouncer = new Debouncer(0.8);

  public LEDSubsystem(LEDStripIO... strips) {
    this.strips = strips;

    inputs =
        Arrays.stream(strips)
            .map(s -> new LEDStripIOInputsAutoLogged())
            .toArray(LEDStripIOInputsAutoLogged[]::new);

    Logger.recordOutput("led/numStrips", strips.length);
  }

  @Override
  public void periodic() {
    boolean runSetup =
        !setupDebouncer.calculate(DriverStation.isEnabled()) && DriverStation.isEnabled();

    for (int i = 0; i < strips.length; i++) {
      strips[i].runSetup(runSetup);
      strips[i].updateInputs(inputs[i]);
      Logger.processInputs("LED/strip" + i, inputs[i]);
    }
  }

  public Command applyColor(BlinkenLEDPattern color) {
    return run(() -> set(color));
  }

  public Command applyColor(Supplier<BlinkenLEDPattern> color) {
    return run(() -> set(color.get()));
  }

  public Command applyColor(
      BlinkenLEDPattern colorIfBlue,
      BlinkenLEDPattern colorIfRed,
      BlinkenLEDPattern colorIfUnknown) {
    return applyColor(
        () ->
            DriverStation.getAlliance()
                .map(a -> a == Alliance.Blue ? colorIfBlue : colorIfRed)
                .orElse(colorIfUnknown));
  }

  public Command turnOff() {
    return applyColor(BlinkenLEDPattern.OFF);
  }

  public void set(BlinkenLEDPattern pattern) {
    Stream.of(strips).forEach(strip -> strip.setPattern(pattern));
  }
}
