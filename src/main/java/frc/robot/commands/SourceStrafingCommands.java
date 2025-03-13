package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.commands.controllers.JoystickInputController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;

public class SourceStrafingCommands {
  /** IF auto alignment is finished and placed us by a source */
  public static Command takeAction(Drive drive, CommandXboxController xbox) {
    Pose2d robotPose = drive.getRobotPose();
    boolean bySourceLeft = false, bySourceRight = false;

    if (bySourceLeft) {
      return strafeOnSourceLeft(drive, xbox).until(xbox.leftBumper().negate());
    } else { // Since autoalign just ran we must be on the right
      // return
    }
  }

  public static Command strafeOnSourceLeft(Drive drivetrain, CommandXboxController controller) {
    final JoystickInputController input = getInput(drivetrain, controller);
    final PIDController xController = new PIDController(0, 0, 0);
    final PIDController angleController = new PIDController(0, 0, 0);

    Pose2d startingPose = // TODO offset this
        AllianceFlipUtil.apply(FieldConstants.CoralStation.leftCenterFace);

    Runnable keepAtRange =
        () -> {
          Translation2d inputTranslation = input.getTranslationMetersPerSecond();
          double targetHeadingRadians =
              AllianceFlipUtil.apply(
                      FieldConstants.CoralStation.leftCenterFace
                          .getRotation()
                          .minus(Rotation2d.k180deg))
                  .getRadians();

          double omega =
              angleController.calculate(
                  AllianceFlipUtil.apply(drivetrain.getRobotPose().getRotation()).getRadians());

          double targetX = 0; // TODO

          boolean notAtDesiredAngle =
              !MathUtil.isNear(
                  drivetrain.getRobotPose().getRotation().getRadians(),
                  targetHeadingRadians,
                  Math.PI / 4);

          drivetrain.setRobotSpeeds(
              new ChassisSpeeds(
                  xController.calculate(drivetrain.getRobotPose().getX(), targetX),
                  inputTranslation.getY(),
                  notAtDesiredAngle ? omega : 0),
              true);
        };

    return Commands.sequence(
        DriveCommands.pathfindToPoseCommand(drivetrain, startingPose, 1, 0),
        drivetrain.run(keepAtRange));
  }

  private static JoystickInputController getInput(Drive drive, CommandXboxController xbox) {
    return new JoystickInputController(
        drive,
        () -> -xbox.getLeftY(),
        () -> -xbox.getLeftX(),
        () -> -xbox.getRightY(),
        () -> -xbox.getRightX());
  }
}
