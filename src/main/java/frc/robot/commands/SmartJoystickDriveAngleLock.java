package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.commands.controllers.SpeedLevelController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.ThresholdController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SmartJoystickDriveAngleLock extends Command {

  private final Drive drive;
  private final Supplier<Translation2d> translationSupplier;
  private final DoubleSupplier omegaSupplier;
  private final Supplier<SpeedLevelController.SpeedLevel> speedLevelSupplier;
  private final BooleanSupplier useFieldRelative;

  private final HeadingController headingController;
  private final ThresholdController errorActivationController;

  private boolean isStillRotating = false;

  /**
   * Creates a new SmartJoystickDriveAngleLock command.
   *
   * @param drive drivetrain of robot
   * @param translationSupplier supplier of translation vector
   * @param omegaSupplier supplier of angular velocity
   * @param speedLevelSupplier supplier of speed level
   * @param useFieldRelative supplier of whether to use field relative
   */
  public SmartJoystickDriveAngleLock(
      Drive drive,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier omegaSupplier,
      Supplier<SpeedLevelController.SpeedLevel> speedLevelSupplier,
      BooleanSupplier useFieldRelative) {

    this.drive = drive;
    this.translationSupplier = translationSupplier;
    this.omegaSupplier = omegaSupplier;
    this.speedLevelSupplier = speedLevelSupplier;
    this.useFieldRelative = useFieldRelative;

    this.headingController = new HeadingController(drive);
    this.errorActivationController =
        new ThresholdController(
            Units.degreesToRadians(3.0),
            Units.degreesToRadians(1.0),
            ThresholdController.Direction.DOWN);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    headingController.setGoalToCurrentHeading();
    headingController.reset();
  }

  @Override
  public void execute() {
    Translation2d translation = translationSupplier.get();
    double requestedOmega = omegaSupplier.getAsDouble();
    double headingControlOmega = headingController.calculate();
    double headingControlError = Math.abs(headingController.getError());

    boolean requiresHeadingCorrection = errorActivationController.calculate(headingControlError);

    boolean useFieldRelativeValue = useFieldRelative.getAsBoolean();

    if (requestedOmega != 0) {
      isStillRotating = true;
    }

    if (isStillRotating
        && MathUtil.isNear(
            drive.getRobotSpeeds().omegaRadiansPerSecond,
            0.0,
            Units.rotationsPerMinuteToRadiansPerSecond(1))) {
      isStillRotating = false;
    }

    Logger.recordOutput(
        "Drive/JoystickDriveSmartAngleLock/requestedRPM",
        Units.radiansPerSecondToRotationsPerMinute(requestedOmega));
    Logger.recordOutput(
        "Drive/JoystickDriveSmartAngleLock/headingControllerRequestedRPM",
        Units.radiansPerSecondToRotationsPerMinute(headingControlOmega));
    Logger.recordOutput(
        "Drive/JoystickDriveSmartAngleLock/headingControlError",
        Units.radiansToDegrees(headingControlError));
    Logger.recordOutput(
        "Drive/JoystickDriveSmartAngleLock/measuredRPM",
        Units.radiansPerSecondToRotationsPerMinute(drive.getRobotSpeeds().omegaRadiansPerSecond));
    Logger.recordOutput(
        "Drive/JoystickDriveSmartAngleLock/headingControlPassedThreshold",
        requiresHeadingCorrection);
    Logger.recordOutput("Drive/JoystickDriveSmartAngleLock/isStillRotating", isStillRotating);

    ChassisSpeeds speeds =
        SpeedLevelController.apply(
            new ChassisSpeeds(translation.getX(), translation.getY(), requestedOmega),
            speedLevelSupplier.get());

    if (!isStillRotating && requiresHeadingCorrection && useFieldRelativeValue) {
      speeds.omegaRadiansPerSecond = headingControlOmega;
    } else {
      headingController.setGoalToCurrentHeading();
      headingController.reset();
    }

    drive.setRobotSpeeds(speeds, useFieldRelativeValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
