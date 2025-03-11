package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.ArmFeedForwardConstants;
import frc.robot.utility.records.PIDConstants;

/** Constants for the Template subsystem. */
public class WristConstants {

  public static final DCMotor MOTOR = DCMotor.getNeo550(1);

  public static final double GEAR_REDUCTION = 20.0;
  public static final int MOTOR_CURRENT_LIMIT = 40;

  public static final double TOLERANCE_DEGREES = 15;

  public static final double MIN_POSITION_DEGREES = Units.rotationsToDegrees(-0.465);
  public static final double MAX_POSITION_DEGREES = Units.rotationsToDegrees(+0.5);

  public record WristConfig(
      int motorId, double absoluteEncoderOffset, boolean motorInverted, boolean encoderInverted) {}

  public static final WristConfig WRIST_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new WristConfig(6, 0.637, false, true);
        default -> new WristConfig(0, 0, false, false);
      };

  public static final double MAX_VELOCITY = 8;
  public static final double MAX_ACCELERATION = 5;

  public static final PIDConstants FEEDBACK =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new PIDConstants(0.15, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(10.0, 0.0, 3.0);
        default -> new PIDConstants(0.0, 0.0, 0.0);
      };

  public static final ArmFeedForwardConstants FEEDFORWARD =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new ArmFeedForwardConstants(0.0, 2.88, 0.78, 0.17);
        default -> new ArmFeedForwardConstants(0.0, 0.0, 0.0, 0.0);
      };
}
