package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.commands.controllers.JoystickInputController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;

public class SourceStrafingCommands {
  private static final PIDController xController = new PIDController(0, 0, 0); // TODO
  private static final PIDController angleController = new PIDController(0, 0, 0);

  /** IF auto alignment is finished and placed us by a source */
  public static Command takeAction(Drive drive, CommandXboxController xbox) {
    if (bySourceLeft(drive.getRobotPose().getY())) {
      return strafeAlongLine(
              drive,
              xbox,
              AllianceFlipUtil.apply(
                  FieldConstants.CoralStation.leftCenterFace
                      .getRotation()
                      .minus(Rotation2d.k180deg)),
              Translation2d.kZero, // TODO
              Translation2d.kZero)
          .until(xbox.leftBumper().negate());
    } else { // Since autoalign just ran we must be on the right
      return strafeAlongLine(
          drive,
          xbox,
          AllianceFlipUtil.apply(
              FieldConstants.CoralStation.rightCenterFace.getRotation().minus(Rotation2d.k180deg)),
          Translation2d.kZero, // TODO
          Translation2d.kZero);
    }
  }

  private static boolean bySourceLeft(double y) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    boolean isTopHalf = y >= 4;

    return (alliance == Alliance.Blue && isTopHalf) || (alliance == Alliance.Red && !isTopHalf);
  }

  /** Do not flip input translations, they will be handled by the function */
  public static Command strafeAlongLine(
      Drive drivetrain,
      CommandXboxController xbox,
      Rotation2d desiredHeading,
      Translation2d a,
      Translation2d b) {
    final JoystickInputController input = getInput(drivetrain, xbox);

    Pose2d startingPose = // TODO offset this
        AllianceFlipUtil.apply(FieldConstants.CoralStation.leftCenterFace);

    Runnable keepAtRange =
        () -> {
          Translation2d inputTranslation = input.getTranslationMetersPerSecond();
          Pose2d robotPose = drivetrain.getRobotPose();
          double targetHeadingRadians = desiredHeading.getRadians();

          double omega = angleController.calculate(desiredHeading.getRadians());

          double targetX = findTargetX(a, b, robotPose.getY());

          boolean notAtDesiredAngle =
              !MathUtil.isNear(
                  robotPose.getRotation().getRadians(), targetHeadingRadians, Math.PI / 4);

          drivetrain.setRobotSpeeds(
              new ChassisSpeeds(
                  xController.calculate(robotPose.getX(), targetX),
                  inputTranslation.getY(),
                  notAtDesiredAngle ? omega : 0),
              true);
        };

    return Commands.sequence(
        DriveCommands.pathfindToPoseCommand(drivetrain, startingPose, 1, 0),
        drivetrain.run(keepAtRange));
  }

  private static double findTargetX(Translation2d a, Translation2d b, double y) {
    double m = (b.getY() - a.getY()) / (b.getX() - a.getX());

    return ((y - a.getY()) / m) + a.getX();
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
