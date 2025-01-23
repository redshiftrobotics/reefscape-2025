package frc.robot.utility.commands;

import edu.wpi.first.wpilibj2.command.Command;

/** A utility class for creating custom commands. */
public class CustomCommands {
  private CustomCommands() {}

  /**
   * Constructs a new {@link ReInitCommand} that when scheduled cancels and reschedules the given command.
   *
   * @param command the command to wrap
   * @return the command
   * @see ReInitCommand
   */
  public static Command reInitCommand(Command command) {
    return new ReInitCommand(command);
  }
}
