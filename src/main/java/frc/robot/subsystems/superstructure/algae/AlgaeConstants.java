package frc.robot.subsystems.superstructure.algae;

import frc.robot.Constants;

/** Constants for the Template subsystem. */
public class AlgaeConstants {

  public static double SPEED = 0.5;

  public record IntakeConfig(
      int motorIdLeft, int motorIdRight, boolean invertedLeft, boolean invertedRight) {}
  ;

  public static final IntakeConfig CORAL_INTAKE_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new IntakeConfig(4, 7, false, true);
        default -> new IntakeConfig(0, 0, false, false);
      };

  public static final IntakeConfig ALGAE_INTAKE_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new IntakeConfig(5, 6, false, false);
        default -> new IntakeConfig(0, 0, false, false);
      };
}
