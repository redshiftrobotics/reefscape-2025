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
}
