// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_TOLERANCE;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.controllers.SimpleDriveController;
import frc.robot.commands.controllers.SpeedLevelController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double FF_START_DELAY_SECONDS = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public static Command joystickDrive(
      Drive drive,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier omegaSupplier,
      Supplier<SpeedLevelController.SpeedLevel> speedLevelSupplier,
      BooleanSupplier useFieldRelative) {
    return drive
        .run(
            () -> {
              Translation2d translation = translationSupplier.get();
              double omega = omegaSupplier.getAsDouble();
              ChassisSpeeds speeds =
                  SpeedLevelController.apply(
                      new ChassisSpeeds(translation.getX(), translation.getY(), omega),
                      speedLevelSupplier.get());
              drive.setRobotSpeeds(speeds, useFieldRelative.getAsBoolean());
            })
        .finallyDo(drive::stop);
  }

  public static Command joystickHeadingDrive(
      Drive drive,
      Supplier<Translation2d> translationSupplier,
      Supplier<Optional<Rotation2d>> headingSupplier,
      Supplier<SpeedLevelController.SpeedLevel> speedLevelSupplier,
      BooleanSupplier useFieldRelative) {

    ProfiledPIDController controller =
        new ProfiledPIDController(
            ROTATION_CONTROLLER_CONSTANTS.kP(),
            ROTATION_CONTROLLER_CONSTANTS.kI(),
            ROTATION_CONTROLLER_CONSTANTS.kD(),
            DRIVE_CONFIG.getAngularConstraints());
    controller.setTolerance(ROTATION_TOLERANCE.getRadians());
    controller.enableContinuousInput(0, Units.rotationsToRadians(1));

    return drive
        .run(
            () -> {
              Translation2d translation = translationSupplier.get();
              Optional<Rotation2d> heading = headingSupplier.get();

              if (heading.isPresent()) {
                controller.setGoal(heading.get().getRadians());
              }

              double omega =
                  controller.calculate(
                      AllianceFlipUtil.apply(drive.getRobotPose().getRotation()).getRadians());

              ChassisSpeeds speeds =
                  SpeedLevelController.apply(
                      new ChassisSpeeds(translation.getX(), translation.getY(), 0),
                      speedLevelSupplier.get());

              if (!controller.atGoal()) {
                speeds.omegaRadiansPerSecond = omega;
              }

              drive.setRobotSpeeds(speeds, useFieldRelative.getAsBoolean());
            })
        .beforeStarting(
            () -> {
              controller.setGoal(
                  AllianceFlipUtil.apply(drive.getRobotPose().getRotation()).getRadians());
              controller.reset(
                  drive.getRobotPose().getRotation().getRadians(),
                  drive.getRobotSpeeds().omegaRadiansPerSecond);
            })
        .finallyDo(drive::stop);
  }

  public static Command joystickDriveWithSlowdown(
      Drive drivetrain,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier elevatorHeightSupplier,
      BooleanSupplier useFieldRelativeSupplier) {

    Runnable drive =
        () -> {
          Translation2d translation = translationSupplier.get();
          double elevatorHeight = elevatorHeightSupplier.getAsDouble();
          boolean fieldRelative = useFieldRelativeSupplier.getAsBoolean();

          double multiplier = getMultiplier(elevatorHeight);

          double dx = translation.getX() * multiplier;
          double dy = translation.getY() * multiplier;
          double omega = omegaSupplier.getAsDouble();

          drivetrain.setRobotSpeeds(new ChassisSpeeds(dx, dy, omega), fieldRelative);
        };

    return drivetrain.run(drive).finallyDo(drivetrain::stop);
  }

  public static Command joystickHeadingDriveWithSlowdown(
      Drive drivetrain,
      Supplier<Translation2d> translationSupplier,
      Supplier<Optional<Rotation2d>> headingSupplier,
      DoubleSupplier elevatorHeightSupplier,
      BooleanSupplier useFieldRelativeSupplier) {
    ProfiledPIDController controller =
        new ProfiledPIDController(
            ROTATION_CONTROLLER_CONSTANTS.kP(),
            ROTATION_CONTROLLER_CONSTANTS.kI(),
            ROTATION_CONTROLLER_CONSTANTS.kD(),
            DRIVE_CONFIG.getAngularConstraints());

    controller.setTolerance(ROTATION_TOLERANCE.getRadians());
    controller.enableContinuousInput(0, Units.rotationsToRadians(1));

    Runnable setup =
        () -> {
          Rotation2d robotRotation = drivetrain.getRobotPose().getRotation();
          double goal = AllianceFlipUtil.apply(robotRotation).getRadians();

          controller.setGoal(goal);
          controller.reset(
              robotRotation.getRadians(), drivetrain.getRobotSpeeds().omegaRadiansPerSecond);
        };

    Runnable drive =
        () -> {
          Translation2d translation = translationSupplier.get();
          Optional<Rotation2d> heading = headingSupplier.get();

          if (heading.isPresent()) {
            controller.setGoal(heading.get().getRadians());
          }

          double multiplier = getMultiplier(elevatorHeightSupplier.getAsDouble());

          double controllerOutput =
              controller.calculate(
                  AllianceFlipUtil.apply(drivetrain.getRobotPose().getRotation()).getRadians());

          double dx = translation.getX() * multiplier;
          double dy = translation.getY() * multiplier;
          double omega = controller.atGoal() ? 0 : controllerOutput;

          drivetrain.setRobotSpeeds(new ChassisSpeeds(dx, dy, omega));
        };

    return drivetrain.run(drive).beforeStarting(setup).finallyDo(drivetrain::stop);
  }

  /** Joystick drive */
  public static Command joystickDriveSmartAngleLock(
      Drive drive,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier omegaSupplier,
      Supplier<SpeedLevelController.SpeedLevel> speedLevelSupplier,
      BooleanSupplier useFieldRelative) {
    return new SmartJoystickDriveAngleLock(
        drive, translationSupplier, omegaSupplier, speedLevelSupplier, useFieldRelative);
  }

  /** Drive to a pose, more precise */
  public static Command driveToPoseSimple(Drive drive, Pose2d desiredPose) {
    SimpleDriveController controller = new SimpleDriveController();

    return drive
        .run(
            () ->
                drive.setRobotSpeeds(
                    controller.calculate(
                        drive.getRobotPose(), desiredPose, 0, desiredPose.getRotation())))
        .until(controller::atReference)
        .beforeStarting(
            () -> {
              controller.getXController().reset();
              controller.getYController().reset();
              controller
                  .getThetaController()
                  .reset(
                      drive.getRobotPose().getRotation().getRadians(),
                      drive.getRobotSpeeds().omegaRadiansPerSecond);
            })
        .finallyDo(drive::stop);
  }

  public static Command holdPositionCommand(Drive drive) {
    SimpleDriveController controller = new SimpleDriveController();
    return Commands.either(
            drive.run(drive::stopUsingBrakeArrangement),
            drive.run(() -> drive.setRobotSpeeds(controller.calculate(drive.getRobotPose()))),
            controller::atReference)
        .beforeStarting(
            () -> {
              controller.reset(drive.getRobotPose());
              controller.setSetpoint(drive.getRobotPose());
            })
        .finallyDo(drive::stopUsingForwardArrangement);
  }

  /** Pathfind to a pose with pathplanner, only gets you roughly to target pose. */
  public static Command pathfindToPoseCommand(
      Drive drive, Pose2d desiredPose, double speedMultiplier, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(
        desiredPose, DRIVE_CONFIG.getPathConstraints(speedMultiplier), goalEndVelocity);
  }

  /** Estimated feed forward Ks and Kv by driving robot forward, control motors by voltage */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY_SECONDS),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.setRobotSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRawGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      Rotation2d rotation = drive.getRawGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DRIVE_CONFIG.driveBaseRadius()) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  private static double getMultiplier(double elevatorHeight) {
    return getMultiplier(elevatorHeight, 0.5);
  }

  private static double getMultiplier(double elevatorHeight, double multiplier) {
    return 1 - (elevatorHeight * multiplier);
  }
}
