// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_FEEDBACK;
import static frc.robot.subsystems.drive.DriveConstants.TURN_FEEDBACK;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.commands.controllers.SpeedLevelController;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
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

  /** Joystick drive */
  public static Command joystickDriveSmartAngleLock(
      Drive drive,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier omegaSupplier,
      Supplier<SpeedLevelController.SpeedLevel> speedLevelSupplier,
      BooleanSupplier useFieldRelative) {

    HeadingController headingController = new HeadingController(drive);
    Debouncer noRotationDebouncer = new Debouncer(0.1);

    return drive
        .run(
            () -> {
              Translation2d translation = translationSupplier.get();
              double omega = omegaSupplier.getAsDouble();
              ChassisSpeeds speeds =
                  SpeedLevelController.apply(
                      new ChassisSpeeds(translation.getX(), translation.getY(), omega),
                      speedLevelSupplier.get());
              if (noRotationDebouncer.calculate(omega == 0.0)) {
                speeds.omegaRadiansPerSecond = headingController.calculate();
              } else {
                headingController.setGoalToCurrentHeading();
                headingController.reset();
              }
              drive.setRobotSpeeds(speeds, useFieldRelative.getAsBoolean());
            })
        .beforeStarting(
            () -> {
              headingController.setGoalToCurrentHeading();
              headingController.reset();
            })
        .finallyDo(drive::stop);
  }

  /** Drive to a pose, more precise */
  public static Command driveToPosePrecise(Drive drive, Pose2d desiredPose) {
    HolonomicDriveController controller =
        new HolonomicDriveController(
            new PIDController(DRIVE_FEEDBACK.Kp(), DRIVE_FEEDBACK.Ki(), DRIVE_FEEDBACK.Kd()),
            new PIDController(DRIVE_FEEDBACK.Kp(), DRIVE_FEEDBACK.Ki(), DRIVE_FEEDBACK.Kd()),
            new ProfiledPIDController(
                TURN_FEEDBACK.Kp(),
                TURN_FEEDBACK.Ki(),
                TURN_FEEDBACK.Kd(),
                DRIVE_CONFIG.getAngularConstraints()));
    controller.setTolerance(
        new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(5)));

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

  /** Pathfind to a pose with pathplanner, only gets you roughly to target pose. */
  public static Command pathfindToPoseCommand(
      Drive drive, Pose2d desiredPose, double speedMultiplier, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(
        desiredPose, DRIVE_CONFIG.getPathConstraints(), goalEndVelocity);
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
                  state.lastAngle = drive.getRobotPose().getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      Rotation2d rotation = drive.getRobotPose().getRotation();
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
}
