package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;

import org.littletonrobotics.junction.Logger;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

  private final Drive drive;

  private static final LoggedTunableNumberFactory factory =
      new LoggedTunableNumberFactory("HeadingController/");

  private static final LoggedTunableNumber kP =
      factory.getNumber("kP", HEADING_CONTROLLER_CONFIG.pid().kP());

  private static final LoggedTunableNumber kI =
      factory.getNumber("kI", HEADING_CONTROLLER_CONFIG.pid().kI());

  private static final LoggedTunableNumber kD =
      factory.getNumber("kD", HEADING_CONTROLLER_CONFIG.pid().kD());

  private static final LoggedTunableNumber kAngularVelocity =
      factory.getNumber("kAngularVelocity", DRIVE_CONFIG.maxAngularVelocity());

  private static final LoggedTunableNumber kAngularAcceleration =
      factory.getNumber("kAngularAcceleration", DRIVE_CONFIG.maxAngularAcceleration());

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
            kP.get(),
            kI.get(),
            kD.get(),
            new TrapezoidProfile.Constraints(
                kAngularVelocity.get(), kAngularAcceleration.get()),
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
    
   // Update PID values
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          headingControllerRadians.setPID(values[0], values[1], values[2]);
        },
        kP,
        kI,
        kD);

    // Update motion profile constraints
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          headingControllerRadians.setConstraints(
              new TrapezoidProfile.Constraints(values[0], values[1]));
        },
        kAngularVelocity,
        kAngularAcceleration);

    // Calculate output
    double measurement = drive.getRobotPose().getRotation().getRadians();
    double output = headingControllerRadians.calculate(measurement);

    // Record data
    Logger.recordOutput("HeadingController/Goal", headingControllerRadians.getGoal().position);
    Logger.recordOutput("HeadingController/Output", output);
    Logger.recordOutput(
        "HeadingController/HeadingError", headingControllerRadians.getPositionError());
    Logger.recordOutput("HeadingController/AtGoal", headingControllerRadians.atGoal());

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
