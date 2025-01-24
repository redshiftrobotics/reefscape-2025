package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDConstants.LEDPatterns;

public class LEDSubsystem extends SubsystemBase {
  private class LEDStrip {
    public Spark pwm;
    public double pattern;

    public LEDStrip(Spark pwmController, double initialPattern) {
      pwm = pwmController;
      pattern = initialPattern;
    }
  }

  private LEDStrip[] strips;

  public LEDSubsystem(int... pwmPorts) {
    // Create all the strip objects
    strips = new LEDStrip[pwmPorts.length];
    for (int i = pwmPorts.length; i >= 0; i--) {
      strips[i] = new LEDStrip(new Spark(pwmPorts[i]), LEDPatterns.BLACK);
    }
  }

  @Override
  public void periodic() {
    for (LEDStrip strip : strips) {
      strip.pwm.set(strip.pattern);
    }
  }

  /**
   * @brief Apply a pattern to a specific LED strip
   * @param strip The strip index to apply to. These are the same as the indices of LEDStrip objects
   *     in the constructor
   * @param pattern The pattern to apply to the strip. See the below link for details, or use the
   *     LEDPatterns constant.
   * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  public void applyPatternTo(int strip, double pattern) {
    if (strip >= 0 && strip < strips.length) strips[strip].pattern = pattern;
  }

  /**
   * @brief Apply a pattern to all LED strips
   * @param pattern The pattern to apply to the strips. See the below link for details, or use the
   *     LEDPatterns constant.
   * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  public void applyPatternToAll(double pattern) {
    for (LEDStrip strip : strips) {
      strip.pattern = pattern;
    }
  }
}
