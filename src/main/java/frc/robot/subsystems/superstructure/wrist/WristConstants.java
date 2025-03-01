package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
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

  private static final PIDConstants REAL_PID_CONSTANTS = new PIDConstants(0, 0, 0);
  private static final PIDConstants SIM_PID_CONSTANTS = new PIDConstants(0, 0, 0);

  public static PIDConstants getPidConstants() {
    return Constants.getMode() == Mode.SIM ? SIM_PID_CONSTANTS : REAL_PID_CONSTANTS;
  }

  /** Gearing or something, higher numbers are reductions. */
  public static final double SIM_GEARING = 1;
  /** Calculate from OnShape */
  public static final double SIM_MOMENT_OF_INERTIA = 1;
  /** In metres. */
  public static final double SIM_ARM_LENGTH = Units.inchesToMeters(14.585);
  /** In radians. */
  public static final double SIM_ARM_MIN_ANGLE = 0;
  /** In radians. */
  public static final double SIM_ARM_MAX_ANGLE = Math.PI;
  /** In radians. */
  public static final double SIM_ARM_INIT_ANGLE = 0;

  public static final int MOTOR_ID = 0;
}
