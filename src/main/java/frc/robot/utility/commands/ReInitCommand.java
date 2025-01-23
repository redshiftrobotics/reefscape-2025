package frc.robot.utility.commands;

import edu.wpi.first.wpilibj2.command.Command;

/** A command that cancels and then re-schedules another command on initialization and then ends. */
public class ReInitCommand extends Command {

  private final Command command;

  /**
   * Constructs a new {@link ReInitCommand} that when scheduled cancels and reschedules the given command.
   *
   * @param command the command to wrap
   */
  public ReInitCommand(Command command) {
    this.command = command;
  }

  @Override
  public void initialize() {
    if (command.isScheduled()) {
      command.cancel();
    }

    command.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
