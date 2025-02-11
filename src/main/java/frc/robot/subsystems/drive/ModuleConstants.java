package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.FeedForwardConstants;
import frc.robot.utility.records.PIDConstants;

public class ModuleConstants {

  // --- Module Config ---

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public static final ModuleConfig FRONT_LEFT_MODULE_CONFIG;
  public static final ModuleConfig FRONT_RIGHT_MODULE_CONFIG;
  public static final ModuleConfig BACK_LEFT_MODULE_CONFIG;
  public static final ModuleConfig BACK_RIGHT_MODULE_CONFIG;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        break;
      
      case COMP_BOT_2025:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        break;

      case T_SHIRT_CANNON_CHASSIS:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(19, 18, 39, Rotation2d.fromRotations(-0.186279296875), true);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(2, 1, 37, Rotation2d.fromRotations(-0.677490234375 + 0.5), true);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(11, 10, 36, Rotation2d.fromRotations(-0.8603515625), true);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 38, Rotation2d.fromRotations(-0.065185546875 + 0.5), true);
        break;

      case CRESCENDO_CHASSIS_2024:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(2, 3, 3, Rotation2d.fromRotations(0.631591796875), false);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(14, 17, 4, Rotation2d.fromRotations(-0.77758789062), false);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 2, Rotation2d.fromRotations(-0.641357421875), false);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(10, 11, 1, Rotation2d.fromRotations(0.453857421875), false);
        break;

      case WOOD_BOT_TWO_2025:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(2, 1, 37, Rotation2d.fromRotations(-0.880126953125), true);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(19, 18, 36, Rotation2d.fromRotations(-0.29833984375), true);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 39, Rotation2d.fromRotations(-0.32373046875), true);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(11, 10, 38, Rotation2d.fromRotations(-0.8935546875), true);
        break;

      default:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.fromRotations(0), false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.fromRotations(0), false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.fromRotations(0), false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, Rotation2d.fromRotations(0), false);
        break;
    }
  }

  // --- Module Constants ---

  public static final DCMotor DRIVE_MOTOR;
  public static final FeedForwardConstants DRIVE_FEED_FORWARD;
  public static final PIDConstants DRIVE_FEEDBACK;
  public static final int DRIVE_MOTOR_CURRENT_LIMIT;
  public static final double DRIVE_REDUCTION;

  public static final DCMotor TURN_MOTOR;
  public static final PIDConstants TURN_FEEDBACK;
  public static final int TURN_MOTOR_CURRENT_LIMIT;
  public static final double TURN_REDUCTION;

  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.000);

  static {
    switch (Constants.getRobot()) {
      case CRESCENDO_CHASSIS_2024:
        DRIVE_MOTOR = DCMotor.getNEO(1);
        DRIVE_FEEDBACK = new PIDConstants(0.000006, 0.0, 0.0);
        DRIVE_FEED_FORWARD = new FeedForwardConstants(0.1, 3.12, 0.40);
        DRIVE_MOTOR_CURRENT_LIMIT = 50;
        DRIVE_REDUCTION = Mk4Reductions.L1.reduction;

        TURN_MOTOR = DCMotor.getNEO(1);
        TURN_FEEDBACK = new PIDConstants(10, 0.0, 0.0002);
        TURN_MOTOR_CURRENT_LIMIT = 20;
        TURN_REDUCTION = Mk4Reductions.TURN.reduction;
        break;

      case SIM_BOT:
        DRIVE_MOTOR = DCMotor.getNEO(1);
        DRIVE_FEEDBACK = new PIDConstants(1.3, 0.0, 0.0);
        DRIVE_FEED_FORWARD = new FeedForwardConstants(0.0, 0.0, 0.0);
        DRIVE_MOTOR_CURRENT_LIMIT = 50;
        DRIVE_REDUCTION = Mk4iReductions.L3.reduction;

        TURN_MOTOR = DCMotor.getNEO(1);
        TURN_FEEDBACK = new PIDConstants(10.0, 0.0, 0.0);
        TURN_MOTOR_CURRENT_LIMIT = 20;
        TURN_REDUCTION = Mk4iReductions.TURN.reduction;
        break;

      case WOOD_BOT_TWO_2025:
      case T_SHIRT_CANNON_CHASSIS:
      case COMP_BOT_2025:
      default:
        DRIVE_MOTOR = DCMotor.getNEO(1);
        DRIVE_FEEDBACK = new PIDConstants(0.0001, 0.0, 0.0);
        DRIVE_FEED_FORWARD = new FeedForwardConstants(0.1, 2.35, 0.53);
        DRIVE_MOTOR_CURRENT_LIMIT = 50;
        DRIVE_REDUCTION = Mk4iReductions.L3.reduction;

        TURN_MOTOR = DCMotor.getNEO(1);
        TURN_FEEDBACK = new PIDConstants(10, 0.0, 0.0);
        TURN_MOTOR_CURRENT_LIMIT = 20;
        TURN_REDUCTION = Mk4iReductions.TURN.reduction;
        break;
    }
  }

  // --- Module reductions ---

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  private enum Mk4iReductions {
    // Note: Mk4i turn motors are inverted!
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }

  // https://www.swervedrivespecialties.com/products/mk4-swerve-module
  private enum Mk4Reductions {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((12.8 / 1.0));

    final double reduction;

    Mk4Reductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
