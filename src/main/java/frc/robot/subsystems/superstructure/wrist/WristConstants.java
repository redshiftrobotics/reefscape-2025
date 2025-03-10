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
  public static final int MOTOR_CURRENT_LIMIT = 30;

  public static final double TOLERANCE = Units.degreesToRotations(15);

  public record WristConfig(
      int motorId, double absoluteEncoderOffset, boolean motorInverted, boolean encoderInverted) {}

  public static final WristConfig WRIST_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new WristConfig(13, 0, false, false);
        default -> new WristConfig(0, 0, false, false);
      };

  public static final PIDConstants FEEDBACK =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new PIDConstants(0.3, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(10.0, 0.0, 3.0);
        default -> new PIDConstants(0.0, 0.0, 0.0);
      };

  public static final ArmFeedForwardConstants FEEDFORWARD =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new ArmFeedForwardConstants(0.0, 0.0, 0.0, 0.0);
        default -> new ArmFeedForwardConstants(0.0, 0.0, 0.0, 0.0);
      };
}
