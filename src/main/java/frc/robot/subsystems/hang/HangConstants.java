package frc.robot.subsystems.hang;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

public class HangConstants {
  private HangConstants() {}

  public static record HangConfig(int motorId, double absoluteEncoderOffset) {}

  public static final HangConfig HANG_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new HangConfig(16, 0.0);
        default -> new HangConfig(0, 0.0);
      };

  public static final PIDConstants FEEDBACK =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new PIDConstants(0.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(10.0, 0.0, 0.0);
        default -> new PIDConstants(1.0, 0.0, 0.0);
      };

  public static final int MOTOR_CURRENT_LIMIT = 40;
  public static final double GEAR_REDUCTION = Math.pow(5, 3);

  public static final double STOWED_POSITION_ROTATIONS = 0.0;
  public static final double DEPLOY_POSITION_ROTATIONS = 0.0;
  public static final double RETRACT_POSITION_ROTATIONS = STOWED_POSITION_ROTATIONS;
  public static final double TOLERANCE = Units.degreesToRotations(0.3);
}
