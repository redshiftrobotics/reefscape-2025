package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

  private final Drive drive;

  private final ProfiledPIDController headingControllerRadians;

  /**
   * Creates a new HeadingController object
   *
   * @param drive drivetrain of robot
   */
  public HeadingController(Drive drive) {
    this(drive, HEADING_CONTROLLER_CONFIG.toleranceDegrees());
  }

  /**
   * Creates a new HeadingController object
   *
   * @param drive drivetrain of robot
   */
  public HeadingController(Drive drive, double toleranceDegrees) {
    this.drive = drive;

    headingControllerRadians =
        new ProfiledPIDController(
            HEADING_CONTROLLER_CONFIG.pid().kP(),
            HEADING_CONTROLLER_CONFIG.pid().kI(),
            HEADING_CONTROLLER_CONFIG.pid().kD(),
            new TrapezoidProfile.Constraints(
                DRIVE_CONFIG.maxAngularVelocity(), DRIVE_CONFIG.maxAngularAcceleration()),
            Constants.LOOP_PERIOD_SECONDS);

    headingControllerRadians.enableContinuousInput(-Math.PI, Math.PI);

    headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees));
  }

  /** Reset last position and rotation to prepare for new use */
  public void reset() {
    headingControllerRadians.reset(
        drive.getRobotPose().getRotation().getRadians(),
        drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Set goal heading. Calculate will now give values to get to this heading.
   *
   * @param heading desired heading of chassis
   */
  public void setGoal(Rotation2d heading) {
    headingControllerRadians.setGoal(heading.getRadians());
  }

  /** Set goal heading to current heading of chassis */
  public void setGoalToCurrentHeading() {
    setGoal(drive.getRobotPose().getRotation());
  }

  /**
   * Get goal heading.
   *
   * @return desired heading of chassis
   */
  public Rotation2d getGoal() {
    return new Rotation2d(headingControllerRadians.getGoal().position);
  }

  /**
   * Get speed chassis needs to rotation at to reach heading goal
   *
   * @param goalHeadingRadians desired heading of chassis
   * @return rotation speed to reach heading goal, omega radians per second
   */
  public double calculate(Rotation2d goalHeadingRadians) {
    setGoal(goalHeadingRadians);
    return calculate();
  }

  /**
   * Get speed chassis needs to rotation at to reach heading goal
   *
   * @return rotation speed to reach heading goal, omega radians per second
   */
  public double calculate() {
    // Calculate output
    double measurement = drive.getRobotPose().getRotation().getRadians();
    double output = headingControllerRadians.calculate(measurement);

    Logger.recordOutput(
        "Drive/HeadingController/Goal", headingControllerRadians.getGoal().position);
    Logger.recordOutput("Drive/HeadingController/Output", output);
    Logger.recordOutput(
        "Drive/HeadingController/HeadingError", headingControllerRadians.getPositionError());
    Logger.recordOutput("Drive/HeadingController/AtGoal", headingControllerRadians.atGoal());

    return output;
  }

  /**
   * Get if the chassis heading is our goal heading
   *
   * @return true if the absolute value of the position error is less than tolerance
   */
  public boolean atGoal() {
    return headingControllerRadians.atGoal();
  }

  /**
   * Get the error in the heading
   *
   * @return error in the heading in radians
   */
  public double getError() {
    return headingControllerRadians.getPositionError();
  }
}
