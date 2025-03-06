package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_TOLERANCE;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_TOLERANCE;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimpleDriveController extends HolonomicDriveController {

  private Pose2d setpoint;

  public SimpleDriveController() {
    super(
        new PIDController(
            TRANSLATION_CONTROLLER_CONSTANTS.kP(),
            TRANSLATION_CONTROLLER_CONSTANTS.kI(),
            TRANSLATION_CONTROLLER_CONSTANTS.kD()),
        new PIDController(
            TRANSLATION_CONTROLLER_CONSTANTS.kP(),
            TRANSLATION_CONTROLLER_CONSTANTS.kI(),
            TRANSLATION_CONTROLLER_CONSTANTS.kD()),
        new ProfiledPIDController(
            ROTATION_CONTROLLER_CONSTANTS.kP(),
            ROTATION_CONTROLLER_CONSTANTS.kI(),
            ROTATION_CONTROLLER_CONSTANTS.kD(),
            DRIVE_CONFIG.getAngularConstraints()));
    setTolerance(new Pose2d(TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, ROTATION_TOLERANCE));
  }

  public void setSetpoint(Pose2d setpoint) {
    this.setpoint = setpoint;
  }

  public void reset(Pose2d pose) {
    getXController().reset();
    getYController().reset();
    getThetaController().reset(pose.getRotation().getRadians());
  }

  public ChassisSpeeds calculate(Pose2d measured) {
    return calculate(measured, setpoint, 0, setpoint.getRotation());
  }

  public ChassisSpeeds calculate(Pose2d measured, Pose2d setpoint) {
    setSetpoint(setpoint);
    return calculate(measured);
  }
}
