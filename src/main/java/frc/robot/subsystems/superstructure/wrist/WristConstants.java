package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

/** Constants for the Template subsystem. */
public class WristConstants {

  public static final DCMotor MOTOR = DCMotor.getNeo550(1);

  public static final int MOTOR_CURRENT_LIMIT = 30;

  public static final double TOLERANCE = Units.degreesToRotations(5);

  public record WristConfig(
      int motorId,
      double gearReduction,
      double absoluteEncoderOffset,
      boolean motorInverted,
      boolean encoderInverted) {}

  public static final WristConfig CORAL_WRIST_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new WristConfig(1, 9, 0, false, false);
        case SIM_BOT -> new WristConfig(1, 1, 0, false, false);
        default -> new WristConfig(0, 1, 0, false, false);
      };

  public static final WristConfig ALGAE_WRIST_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new WristConfig(1, 2, 0, false, false);
        case SIM_BOT -> new WristConfig(1, 1, 0, false, false);
        default -> new WristConfig(0, 1, 0, false, false);
      };

  public static final PIDConstants CORAL_FEEDBACK =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new PIDConstants(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(10.0, 0.0, 3.0);
        default -> new PIDConstants(0.0, 0.0, 0.0);
      };

  public static final PIDConstants ALGAE_FEEDBACK =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new PIDConstants(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(10.0, 0.0, 3.0);
        default -> new PIDConstants(0.0, 0.0, 0.0);
      };
}
