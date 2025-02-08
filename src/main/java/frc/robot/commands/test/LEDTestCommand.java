package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.led.LEDSubsystem;

public class LEDTestCommand extends Command {
  private LEDSubsystem ledSystem;

  private double currentPattern;

  private int ticker;
  private final int delay;

  /**
   * @param led LED subsystem object to use
   * @param time Time in robot updates to wait between changes (each update is 20ms, so 50 updates
   *     is 1 second)
   */
  public LEDTestCommand(LEDSubsystem led, int time) {
    ledSystem = led;
    delay = time;
  }

  @Override
  public void initialize() {
    currentPattern = LEDConstants.LEDPatterns.BLACK;
    ticker = 0;
  }

  @Override
  public void execute() {
    if (ticker++ < delay) return;

    // 0.02 is the difference between each state, so we go down the list of all possible colors
    currentPattern -= 0.02;
    ledSystem.applyPatternToAll(currentPattern);
  }

  @Override
  public boolean isFinished() {
    return currentPattern <= LEDConstants.LEDPatterns.HOT_PINK;
  }

  @Override
  public void end(boolean interrupted) {
    currentPattern = LEDConstants.LEDPatterns.BLACK;
    ledSystem.applyPatternToAll(currentPattern);
  }
}
