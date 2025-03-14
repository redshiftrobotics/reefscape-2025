package frc.robot.subsystems.hang;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

public class HangConstants {
  private HangConstants() {}

  public static record HangConfig(
      int motorId, double absoluteEncoderOffset, boolean motorInverted, boolean encoderInverted) {}

  public static final HangConfig HANG_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new HangConfig(16, 0.304, false, false);
        default -> new HangConfig(0, 0.0, false, false);
      };

  public static final PIDConstants FEEDBACK =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new PIDConstants(1.0, 0.0, 0.0);
        case SIM_BOT -> new PIDConstants(20.0, 0.0, 0.0);
        default -> new PIDConstants(1.0, 0.0, 0.0);
      };

  public static final int MOTOR_CURRENT_LIMIT = 40;
  public static final double GEAR_REDUCTION = 5 * 5 * 5;

  public static final Rotation2d STOWED_POSITION_ROTATIONS = Rotation2d.fromDegrees(0);
  public static final Rotation2d DEPLOY_POSITION_ROTATIONS = Rotation2d.fromDegrees(60);
  public static final Rotation2d RETRACT_POSITION_ROTATIONS = Rotation2d.fromDegrees(-30);

  public static final double TOLERANCE = Units.degreesToRotations(0.3);
}
