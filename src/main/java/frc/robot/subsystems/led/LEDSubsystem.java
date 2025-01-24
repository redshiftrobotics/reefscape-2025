package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDPatterns;

public class LEDSubsystem extends SubsystemBase {
  private class LEDStrip {
    public Spark pwm;
    public double pattern;

    public LEDStrip(Spark pw, double pattern) {
      pwm = pw;
      this.pattern = pattern;
    }
  }

  private LEDStrip[] strips;
  
  public LEDSubsystem(int... pwmPorts) {
    //Create all the strip objects
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

  //Refer to https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for pattern code info
  public void applyPatternTo(int strip, double pattern) {
    if (strip >= 0 && strip < strips.length)
      strips[strip].pattern = pattern;
  }

  //Refer to https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for pattern code info
  public void applyPatternToAll(double pattern) {
    for (LEDStrip strip : strips) {
      strip.pattern = pattern;
    }
  }
}
