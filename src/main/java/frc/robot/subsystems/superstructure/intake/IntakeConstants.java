package frc.robot.subsystems.superstructure.intake;

import frc.robot.Constants;

/** Constants for the Template subsystem. */
public class IntakeConstants {

  public static final int MOTOR_CURRENT_LIMIT = 30;

  public record IntakeConfig(
      int motorIdLeft, int motorIdRight, boolean invertedLeft, boolean invertedRight) {}
  ;

  public static final IntakeConfig CORAL_INTAKE_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new IntakeConfig(0, 1, false, false);
        default -> new IntakeConfig(0, 0, false, false);
      };

  public static final IntakeConfig ALGAE_INTAKE_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new IntakeConfig(2, 3, false, false);
        default -> new IntakeConfig(0, 0, false, false);
      };
}
