package frc.robot.subsystems.superstructure.wrist;

import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

/** Constants for the Template subsystem. */
public class WristConstants {
  // TODO Numbers are from game manual but need verificiation
  public static final double CORAL_SCORING_POSITION_L1_L2_L3 = Units.degreesToRotations(35);
  public static final double CORAL_SCORING_POSITION_L4 = Units.degreesToRotations(0);
  public static final double CORAL_PICKUP_POSITION = Units.degreesToRotations(55);

  public static final double ABSOLUTE_ENCODER_OFFSET = 0.0;
  public static final double RELATIVE_CONVERSION_FACTOR = 0.0;

  public static final double TOLERANCE = 0.1;

  public static final int MOTOR_ID = 0;
  public static final int CANCODER_ID = 0;
  public static final double ABSOLUTE_ENCODER_OFFSET = 0.0;

  public static final double GEAR_REDUCTION = 1.0;
  public static final int CURRENT_LIMIT = 30;

  public static final PIDConstants FEEDBACK =
      switch (Constants.getRobot()) {
        case SIM_BOT -> new PIDConstants(0.0, 0.0, 0.0);
        default -> new PIDConstants(0.0, 0.0, 0.0);
      };
}
