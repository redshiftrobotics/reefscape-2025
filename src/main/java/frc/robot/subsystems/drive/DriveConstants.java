package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.utility.records.PIDConstants;

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
      return trackCornerToCorner.getNorm() / 2;
    }

    public double maxAngularVelocity() {
      return maxLinearVelocity() / driveBaseRadius();
    }

    public double maxAngularAcceleration() {
      return maxLinearAcceleration() / driveBaseRadius();
    }

    public Constraints getLinearConstraints() {
      return new Constraints(maxLinearVelocity(), maxLinearAcceleration());
    }

    public Constraints getAngularConstraints() {
      return new Constraints(maxAngularVelocity(), maxAngularAcceleration());
    }

    public PathConstraints getPathConstraints() {
      return getPathConstraints(1);
    }

    public PathConstraints getPathConstraints(double speedMultiplier) {
      return new PathConstraints(
          maxLinearVelocity() * speedMultiplier,
          maxLinearAcceleration(),
          maxAngularVelocity() * speedMultiplier,
          maxAngularAcceleration());
    }
  }

  public static final DriveConfig DRIVE_CONFIG =
      switch (Constants.getRobot()) {
        case WOOD_BOT_TWO_2025, T_SHIRT_CANNON_CHASSIS -> new DriveConfig(
            new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);
        case CRESCENDO_CHASSIS_2024 -> new DriveConfig(
            new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 3.81, 14.5);
        case SIM_BOT -> new DriveConfig(
            new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);
      };

  // --- Gyro Config ---

  public static final int GYRO_CAN_ID =
      switch (Constants.getRobot()) {
        case CRESCENDO_CHASSIS_2024 -> 40;
        case T_SHIRT_CANNON_CHASSIS -> 40;
        case WOOD_BOT_TWO_2025 -> 40;
        default -> -1;
      };

  // --- Pathplanner Config ---

  private static final double robotMassKg = 74.088;
  private static final double robotMOI = 6.883;
  private static final double wheelCOF = 1.2;
  private static final Translation2d[] moduleTranslations = {
    ModuleConstants.FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER,
    ModuleConstants.FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER,
    ModuleConstants.BACK_LEFT_MODULE_DISTANCE_FROM_CENTER,
    ModuleConstants.BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER
  };

  public static final RobotConfig pathPlannerRobotConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new com.pathplanner.lib.config.ModuleConfig(
              ModuleConstants.WHEEL_RADIUS,
              DRIVE_CONFIG.maxLinearVelocity(),
              wheelCOF,
              ModuleConstants.DRIVE_MOTOR.withReduction(ModuleConstants.DRIVE_REDUCTION),
              ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT,
              1),
          moduleTranslations);

  // --- Odometry Frequency ---

  public static final double ODOMETRY_FREQUENCY_HERTZ =
      switch (Constants.getRobot()) {
        case SIM_BOT -> 50.0;
        default -> 100.0;
      };

  // --- Movement Controller Config ---

  public static final PIDConstants TRANSLATION_CONTROLLER_CONSTANTS_TRAJECTORY =
      new PIDConstants(5.0, 0.0, 0.0);
  public static final PIDConstants ROTATION_CONTROLLER_CONSTANTS_TRAJECTORY =
      new PIDConstants(5.0, 0, 0.4);

  public static final PIDConstants TRANSLATION_CONTROLLER_CONSTANTS = new PIDConstants(5.0, 0.0, 1);
  public static final PIDConstants ROTATION_CONTROLLER_CONSTANTS = new PIDConstants(5.0, 0, 1);

  // --- Heading Controller Config ---

  public record HeadingControllerConfig(PIDConstants pid, double toleranceDegrees) {}

  public static final HeadingControllerConfig HEADING_CONTROLLER_CONFIG =
      new HeadingControllerConfig(ROTATION_CONTROLLER_CONSTANTS, 1.0);
}
