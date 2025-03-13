package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDConstants.FixedPalettePattern;
import frc.robot.subsystems.led.LEDConstants.SolidColors;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {

  private final Timer startupTimer = new Timer();

  private class LEDStrip {
    private PWM pwm;
    private int pulse;

    public LEDStrip(PWM pwmController, int initialPattern) {
      pwm = pwmController;
      pulse = initialPattern;
    }

    public void update() {
      pwm.setPulseTimeMicroseconds(pulse);
    }
  }

  private LEDStrip[] strips;

  public LEDSubsystem(int... pwmPorts) {
    // Create all the strip objects
    strips = new LEDStrip[pwmPorts.length];
    for (int i = 0; i < pwmPorts.length; i++) {
      strips[i] = new LEDStrip(new PWM(pwmPorts[i]), LEDConstants.DEFAULT);
    }
  }

  private Optional<Alliance> alliance = Optional.empty();

  @Override
  public void periodic() {

    if (DriverStation.isEnabled() && !startupTimer.isRunning()) {
      startupTimer.restart();
    } else if (startupTimer.hasElapsed(0.02)) {
      startupTimer.stop();
    }

    if (startupTimer.isRunning()) {
      applyAll(2125); // 5 volt mode
    } else {
      alliance = DriverStation.getAlliance();

      if (DriverStation.isEStopped()) {
        applyAll(SolidColors.GRAY);
      } else if (DriverStation.isDisabled()) {
        applyAll(SolidColors.BLUE, SolidColors.RED, SolidColors.WHITE);
      } else {
        applyAll(
            FixedPalettePattern.ColorWaves.OCEAN_PALETTE,
            FixedPalettePattern.Fire.LARGE,
            FixedPalettePattern.ColorWaves.FOREST_PALETTE);
      }
    }

    // Update all the strips
    for (LEDStrip strip : strips) {
      strip.update();
    }
  }

  /**
   * Apply a pattern to a specific LED strip
   *
   * @param strip The strip index to apply to. These are the same as the indices of LEDStrip objects
   *     in the constructor
   * @param pattern The pattern to apply to the strip. See the below link for details, or use the
   *     LEDPatterns constant.
   * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  public void apply(int strip, int pattern) {
    if (strip >= 0 && strip < strips.length) strips[strip].pulse = pattern;
  }

  /**
   * Apply a pattern to all LED strips
   *
   * @param pattern The pattern to apply to the strips. See the below link for details, or use the
   *     LEDPatterns constant.
   * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  public void applyAll(int pattern) {
    for (LEDStrip strip : strips) {
      strip.pulse = pattern;
    }
  }

  /**
   * Apply a pattern to all LED strips based on the alliance color
   *
   * @param blue The pattern to apply if the alliance color is blue
   * @param red The pattern to apply if the alliance color is red
   * @param backup The pattern to apply if the alliance color is unknown
   */
  private void applyAll(int blue, int red, int backup) {
    applyAll(alliance.map(a -> a == Alliance.Blue ? blue : red).orElse(backup));
  }
}
