package frc.robot.subsystems.led;

public class LEDStripIOSim implements LEDStripIO {

  private BlinkenLEDPattern pattern;
  private boolean runSetup;

  public LEDStripIOSim(BlinkenLEDPattern initialPattern) {
    pattern = initialPattern;
  }

  @Override
  public void updateInputs(LEDStripIOInputs inputs) {
    inputs.runningSetup = runSetup;
    inputs.requestedPattern = pattern;

    inputs.targetPulse = inputs.requestedPattern.getPulse();
    if (runSetup) {
      inputs.targetPulse = -1;
    }

    inputs.measuredPulse = inputs.targetPulse;
  }

  @Override
  public void runSetup(boolean run) {
    runSetup = run;
  }

  @Override
  public void setPattern(BlinkenLEDPattern pattern) {
    this.pattern = pattern;
  }
}
