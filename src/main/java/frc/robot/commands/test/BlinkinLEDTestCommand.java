package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.blinkinled.BlinkinLEDConstants;
import frc.robot.subsystems.blinkinled.BlinkinLEDSubsystem;

public class BlinkinLEDTestCommand extends Command {
  private BlinkinLEDSubsystem ledSystem;

  private double currentPattern;

  private int ticker;
  private final int delay;

  /**
   * @param led LED subsystem object to use
   * @param time Time in robot updates to wait between changes (each update is 20ms, so 50 updates
   *     is 1 second)
   */
  public BlinkinLEDTestCommand(BlinkinLEDSubsystem led, int time) {
    ledSystem = led;
    delay = time;
  }

  @Override
  public void initialize() {
    currentPattern = BlinkinLEDConstants.Patterns.BLACK;
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
    return currentPattern <= BlinkinLEDConstants.Patterns.HOT_PINK;
  }

  @Override
  public void end(boolean interrupted) {
    currentPattern = BlinkinLEDConstants.Patterns.BLACK;
  }
}
