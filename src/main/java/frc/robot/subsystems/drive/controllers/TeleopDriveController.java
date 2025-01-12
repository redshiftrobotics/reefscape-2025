package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.JoystickUtil;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Controller for converting joystick values to drive components */
public class TeleopDriveController {
  private final Drive drive;

  private final DoubleSupplier xSupplier, ySupplier, xAngleSupplier, yAngleSupplier;

  /**
   * Creates a new TeleopDriveController object
   *
   * @param drive drivetrain of robot
   * @param xSupplier supplier of translational x (forward+)
   * @param ySupplier supplier of translational y (left+)
   * @param xAngleSupplier supplier of rotational x (angle)
   * @param yAngleSupplier supplier of rotational y (angle, ccw_ omega)
   */
  public TeleopDriveController(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier xAngleSupplier,
      DoubleSupplier yAngleSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.xAngleSupplier = xAngleSupplier;
    this.yAngleSupplier = yAngleSupplier;
  }

  /**
   * Get desired chassis translation from controller (x and y supplier)
   *
   * @return translation x and y in meters per second
   */
  @AutoLogOutput(key = "Drive/TeleopDriveController/translationMetersPerSecond")
  public Translation2d getTranslationMetersPerSecond() {
    if (DriverStation.isAutonomous()) return new Translation2d();

    return TeleopDriveController.getTranslationMetersPerSecond(
        xSupplier.getAsDouble(), ySupplier.getAsDouble(), drive.getMaxLinearSpeedMetersPerSec());
  }

  /**
   * Get desired chassis rotation from controller (y angle supplier)
   *
   * @return rotation in unit per second
   */
  @AutoLogOutput(key = "Drive/TeleopDriveController/omegaRadiansPerSecond")
  public double getOmegaRadiansPerSecond() {
    if (DriverStation.isAutonomous()) return 0;

    return TeleopDriveController.getOmegaRadiansPerSecond(
        yAngleSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec());
  }

  /**
   * Get desired chassis heading from controller (x and y angle supplier)
   *
   * @return heading angle
   */
  @AutoLogOutput(key = "Drive/TeleopDriveController/headingDirection")
  public Optional<Rotation2d> getHeadingDirection() {
    if (DriverStation.isAutonomous()) return Optional.empty();

    return TeleopDriveController.getHeadingDirection(
        xAngleSupplier.getAsDouble(), yAngleSupplier.getAsDouble());
  }

  private static Translation2d getTranslationMetersPerSecond(
      double xInput, double yInput, double maxTranslationSpeedMetersPerSecond) {

    Translation2d translation = new Translation2d(xInput, yInput);

    // get length of linear velocity vector, and apply deadband to it for noise reduction
    double magnitude = JoystickUtil.applyDeadband(translation.getNorm());

    if (magnitude == 0) return new Translation2d();

    // squaring the magnitude of the vector makes for quicker ramp up and slower fine control,
    // magnitude should always be positive
    double magnitudeSquared = Math.copySign(Math.pow(magnitude, 2), 1);

    // get a vector with the same angle as the base linear velocity vector but with the
    // magnitude squared
    Translation2d squaredLinearVelocity =
        new Pose2d(new Translation2d(), translation.getAngle())
            .transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
            .getTranslation();

    // return final value
    return squaredLinearVelocity.times(maxTranslationSpeedMetersPerSecond);
  }

  private static double getOmegaRadiansPerSecond(
      double omegaInput, double maxAngularSpeedRadPerSec) {

    // get rotation speed, and apply deadband
    double omega = JoystickUtil.applyDeadband(omegaInput);

    // square the omega value for quicker ramp up and slower fine control
    // make sure to copy the sign over for direction
    double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

    // return final value
    return omegaSquared * maxAngularSpeedRadPerSec;
  }

  private static Optional<Rotation2d> getHeadingDirection(double xInput, double yInput) {
    // Get desired angle as a vector
    final Translation2d desiredAngle = new Translation2d(xInput, yInput);

    // If the vector length is longer then our deadband update the heading controller
    if (desiredAngle.getNorm() > 0.5) {
      return Optional.of(desiredAngle.getAngle());
    }

    return Optional.empty();
  }
}
