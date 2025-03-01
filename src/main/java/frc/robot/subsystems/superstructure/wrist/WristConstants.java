package frc.robot.subsystems.superstructure.wrist;

import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

/** Constants for the Template subsystem. */
public class WristConstants {

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
