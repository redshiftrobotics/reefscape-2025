package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.util.Units;

/** Constants for the Template subsystem. */
public class WristConstants {
  // TODO Numbers are from game manual but need verificiation
  public static final double CORAL_SCORING_POSITION_L1_L2_L3 = Units.degreesToRotations(35);
  public static final double CORAL_SCORING_POSITION_L4 = Units.degreesToRotations(0);
  public static final double CORAL_PICKUP_POSITION = Units.degreesToRotations(55);

  public static final double WRIST_P = 0;
  public static final double WRIST_I = 0;
  public static final double WRIST_D = 0;
  public static final double WRIST_FF = 0;

  public static final double TOLERANCE = 0.1;

  public static final int MOTOR_ID = 0;

  // placeholders are one to prevent crashes
  /** Gearing or something, higher numbers are reductions. */
  public static final double SIM_GEARING = 1;
  /** Calculate from OnShape */
  public static final double SIM_MOMENT_OF_INERTIA = 1;
  /** In metres. */
  public static final double SIM_ARM_LENGTH = Units.inchesToMeters(18);
  /** In radians. */
  public static final double SIM_ARM_MIN_ANGLE = 0;
  /** In radians. */
  public static final double SIM_ARM_MAX_ANGLE = 0;
  /** In radians. */
  public static final double SIM_ARM_INIT_ANGLE = 0;

  public static final double SIM_P = 0;
  public static final double SIM_I = 0;
  public static final double SIM_D = 0;

  public static final double REAL_P = 0;
  public static final double REAL_I = 0;
  public static final double REAL_D = 0;
}
