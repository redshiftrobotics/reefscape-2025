package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Constants for drivetrain/chassis. All constants should be in meters and radians (m/s, m/s^2,
 * rad/s, rad/s^2). Switch expressions must cover all cases.
 */
public class DriveConstants {

  // --- Drive Config ---

  public record DriveConfig(
      Translation2d trackCornerToCorner,
      Translation2d bumperCornerToCorner,
      double maxLinearVelocity,
      double maxLinearAcceleration) {
    public double driveBaseRadius() {
      return trackCornerToCorner.getNorm() * 0.5;
    }

    public double maxAngularVelocity() {
      return maxLinearVelocity / driveBaseRadius();
    }

    public double maxAngularAcceleration() {
      return maxLinearAcceleration / driveBaseRadius();
    }
  }

  public static final DriveConfig DRIVE_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT, T_SHIRT_CANNON_CHASSIS -> new DriveConfig(
            new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);
        case CRESCENDO_CHASSIS_2024 -> new DriveConfig(
            new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 3.81, 14.5);
        case SIM_BOT -> new DriveConfig(
            new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);
      };

  public static final double wheelRadius = Units.inchesToMeters(2.000);

  // --- Path Constraints ---

  public static final PathConstraints pathConstraints =
      new PathConstraints(3.0, 3.0, 3 * Math.PI, 4 * Math.PI); // Currently a bit arbitrary

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

      case T_SHIRT_CANNON_CHASSIS:
        FRONT_LEFT_MODULE_CONFIG =
            new ModuleConfig(19, 18, 39, Rotation2d.fromRotations(-0.186279296875), false);
        FRONT_RIGHT_MODULE_CONFIG =
            new ModuleConfig(2, 1, 37, Rotation2d.fromRotations(-0.677490234375), false);
        BACK_LEFT_MODULE_CONFIG =
            new ModuleConfig(11, 10, 36, Rotation2d.fromRotations(-0.8603515625), false);
        BACK_RIGHT_MODULE_CONFIG =
            new ModuleConfig(8, 9, 38, Rotation2d.fromRotations(-0.065185546875), false);
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

      case COMP_BOT:
      default:
        FRONT_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        FRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_LEFT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        BACK_RIGHT_MODULE_CONFIG = new ModuleConfig(0, 0, 0, new Rotation2d(), false);
        break;
    }
  }

  private static final double TRACK_CENTER_X = DRIVE_CONFIG.trackCornerToCorner().getX() / 2;
  private static final double TRACK_CENTER_Y = DRIVE_CONFIG.trackCornerToCorner().getY() / 2;

  public static final Translation2d FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(TRACK_CENTER_X, TRACK_CENTER_Y);
  public static final Translation2d FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(TRACK_CENTER_X, -TRACK_CENTER_Y);
  public static final Translation2d BACK_LEFT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(-TRACK_CENTER_X, TRACK_CENTER_Y);
  public static final Translation2d BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER =
      new Translation2d(-TRACK_CENTER_X, -TRACK_CENTER_Y);

  // --- Gyro Config ---

  public static final int GYRO_CAN_ID =
      switch (Constants.getRobot()) {
        case CRESCENDO_CHASSIS_2024 -> 40;
        case T_SHIRT_CANNON_CHASSIS -> 40;
        default -> -1;
      };

  // --- Module Constants ---

  public static final DCMotor driveMotor;
  public static final FeedForwardConstants driveFeedforward;
  public static final PIDConstants driveFeedback;
  public static final int driveMotorCurrentLimit;
  public static final double driveReduction;

  public static final DCMotor turnMotor;
  public static final PIDConstants turnFeedback;
  public static final int turnMotorCurrentLimit;
  public static final double turnReduction;

  static {
    switch (Constants.getRobot()) {
      case CRESCENDO_CHASSIS_2024:
        driveMotor = DCMotor.getNEO(1);
        driveFeedback = new PIDConstants(0.000006, 0.0, 0.0);
        driveFeedforward = new FeedForwardConstants(0.1, 3.12, 0.40);
        driveMotorCurrentLimit = 50;
        driveReduction = Mk4Reductions.L1.reduction;

        turnMotor = DCMotor.getNEO(1);
        turnFeedback = new PIDConstants(10, 0.0, 0.0002);
        turnMotorCurrentLimit = 20;
        turnReduction = Mk4Reductions.TURN.reduction;
        break;

      case SIM_BOT:
        driveMotor = DCMotor.getNEO(1);
        driveFeedback = new PIDConstants(1.3, 0.0, 0.0);
        driveFeedforward = new FeedForwardConstants(0.0, 0, 0);
        driveMotorCurrentLimit = 50;
        driveReduction = Mk4iReductions.L3.reduction;

        turnMotor = DCMotor.getNEO(1);
        turnFeedback = new PIDConstants(10.0, 0.0, 0.0);
        turnMotorCurrentLimit = 20;
        turnReduction = Mk4iReductions.TURN.reduction;
        break;

      case COMP_BOT:
      case T_SHIRT_CANNON_CHASSIS:
      default:
        driveMotor = DCMotor.getNEO(1);
        driveFeedback = new PIDConstants(0.0, 0.0, 0.0);
        // driveFeedforward = new FeedForwardConstants(0.1, 2.35, 0.53);
        driveFeedforward = new FeedForwardConstants(0.0, 0.0, 0.0);
        driveMotorCurrentLimit = 50;
        driveReduction = Mk4iReductions.L3.reduction;

        turnMotor = DCMotor.getNEO(1);
        turnFeedback = new PIDConstants(0.0, 0.0, 0.0);
        turnMotorCurrentLimit = 30;
        turnReduction = Mk4iReductions.TURN.reduction;
        break;
    }
  }

  // --- Pathplanner Config ---

  private static final double robotMassKg = 74.088;
  private static final double robotMOI = 6.883;
  private static final double wheelCOF = 1.2;
  private static final Translation2d[] moduleTranslations = {
    FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER,
    FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER,
    BACK_LEFT_MODULE_DISTANCE_FROM_CENTER,
    BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER
  };

  public static final RobotConfig pathPlannerRobotConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new com.pathplanner.lib.config.ModuleConfig(
              wheelRadius,
              DRIVE_CONFIG.maxLinearVelocity(),
              wheelCOF,
              driveMotor.withReduction(driveReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // --- Odometry Frequency ---

  public static final double odometryFrequencyHertz =
      switch (Constants.getRobot()) {
        case SIM_BOT -> 50.0;
        default -> 100.0;
      };

  // --- Heading Controller Config ---

  public static final PIDConstants headingControllerConstants = new PIDConstants(5.0, 0, 0.4);

  // --- General Tuning Records ---

  public record PIDConstants(double Kp, double Ki, double Kd) {}

  public record FeedForwardConstants(double Ks, double Kv, double Ka) {}

  // --- Module reductions ---

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  private enum Mk4iReductions {
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
