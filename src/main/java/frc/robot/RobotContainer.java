package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.dashboard.DriverDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.SpeedController;
import frc.robot.subsystems.drive.controllers.SpeedController.SpeedLevel;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;
import frc.robot.utility.OverrideSwitch;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandGenericHID driverController = new CommandXboxController(0);
  private final CommandGenericHID operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final Alert fmsAndNotCompBotAlert =
      new Alert(
          "FMS attached and bot is not comp bot, are you sure this is correct?",
          AlertType.kWarning);
  private final Alert tuningModeActiveAlert =
      new Alert("Tuning mode active, do not use in competition.", AlertType.kWarning);
  private final Alert onBlockMode =
      new Alert("On block mode active, do not use in competition.", AlertType.kWarning);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.getRobot()) {
      case COMP_BOT, T_SHIRT_CANNON_CHASSIS:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
        break;

      case CRESCENDO_CHASSIS_2024:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
        break;

      case SIM_BOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to
    // auto populate
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());

    // Configure autos
    configureAutos();

    // Configure sysids
    if (Constants.TUNING_MODE) {
      configureSysIds();
    }

    // Alerts for constants to avoid using them in competition
    if (Constants.TUNING_MODE) {
      tuningModeActiveAlert.set(true);
    }
    if (Constants.ON_BLOCKS_TEST_MODE) {
      onBlockMode.set(true);
    }
    if (DriverStation.isFMSAttached() && Constants.getRobot() != Constants.RobotType.COMP_BOT) {
      fmsAndNotCompBotAlert.set(true);
    }

    // Hide controller missing warnings for sim
    if (Constants.getMode() != Mode.REAL) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    initDashboard();

    // Configure the button bindings
    configureControllerBindings();
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

    DriverDashboard dashboard = DriverDashboard.getInstance();
    dashboard.addSubsystem(drive);
    dashboard.setPoseSupplier(drive::getPose);
    dashboard.setRobotSupplier(drive::getRobotSpeeds);
    dashboard.setSpeedLevelSupplier(() -> SpeedController.SpeedLevel.NO_LEVEL);
    dashboard.setFieldRelativeSupplier(() -> false);
    dashboard.setAngleDrivenSupplier(() -> false);

    dashboard.addCommand("Reset Pose", () -> drive.resetPose(new Pose2d()), true);
    dashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () -> drive.resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))),
        true);
  }

  /** Define button->command mappings. */
  private void configureControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings();
    configureOperatorControllerBindings();
    configureAlertTriggers();
  }

  private void configureDriverControllerBindings() {
    if (driverController instanceof CommandXboxController) {
      final CommandXboxController driverXbox = (CommandXboxController) driverController;

      final Trigger useFieldRelative =
          new Trigger(new OverrideSwitch(driverXbox.y(), OverrideSwitch.Mode.TOGGLE, true));

      final Trigger useAngleControlMode =
          new Trigger(
              new OverrideSwitch(driverXbox.rightBumper(), OverrideSwitch.Mode.HOLD, false));

      // Controllers
      final TeleopDriveController input =
          new TeleopDriveController(
              drive,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              () -> -driverXbox.getRightY(),
              () -> -driverXbox.getRightX());

      final SpeedController speedController = new SpeedController(SpeedLevel.DEFAULT);

      final HeadingController headingController = new HeadingController(drive);

      DriverDashboard.getInstance().setSpeedLevelSupplier(speedController::getCurrentSpeedLevel);
      DriverDashboard.getInstance().setAngleDrivenSupplier(useAngleControlMode);
      DriverDashboard.getInstance().setFieldRelativeSupplier(useFieldRelative);

      // Default command
      drive.setDefaultCommand(
          drive
              .runEnd(
                  () -> {
                    Translation2d translation = input.getTranslationMetersPerSecond();
                    double rotation = input.getOmegaRadiansPerSecond();
                    drive.setRobotSpeeds(
                        speedController.updateSpeed(
                            new ChassisSpeeds(translation.getX(), translation.getY(), rotation)),
                        useFieldRelative.getAsBoolean());
                  },
                  drive::stop)
              .withName("DefaultDrive"));

      useAngleControlMode
          .debounce(0.1)
          .onTrue(
              Commands.runOnce(
                      () -> {
                        headingController.reset();
                        headingController.setGoal(drive.getPose().getRotation());
                      })
                  .withName("PrepareAngleDrive"))
          .whileTrue(
              drive
                  .runEnd(
                      () -> {
                        Translation2d translation = input.getTranslationMetersPerSecond();
                        Optional<Rotation2d> rotation = input.getHeadingDirection();
                        rotation.ifPresent(headingController::setGoal);
                        double omegaRadiansPerSecond = headingController.calculate();
                        drive.setRobotSpeeds(
                            speedController.updateSpeed(
                                new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    headingController.atGoal() ? 0 : omegaRadiansPerSecond)),
                            useFieldRelative.getAsBoolean());
                      },
                      drive::stop)
                  .withName("RotationAngleDrive"));

      boolean includeDiagonalPOV = true;
      for (int pov = 0; pov < 360; pov += includeDiagonalPOV ? 45 : 90) {

        // POV angles are in Clock Wise degrees, needs to be flipped to get correct rotation2d
        final Rotation2d angle = Rotation2d.fromDegrees(-pov);
        final String name = String.format("%d\u00B0", pov);

        // While the POV is being pressed and we are not in angle control mode, set the
        // chassis speeds to the Cos and Sin of the angle
        driverXbox
            .pov(pov)
            .and(useAngleControlMode)
            .whileTrue(
                drive
                    .runEnd(
                        () ->
                            drive.setRobotSpeeds(
                                speedController.updateSpeed(
                                    new ChassisSpeeds(angle.getCos(), angle.getSin(), 0))),
                        drive::stop)
                    .withName(String.format("DriveRobotRelative %s", name)));

        // While the POV is being pressed and we are angle control mode
        // Start by resetting the controller and setting the goal angle to the pov angle
        driverXbox
            .pov(pov)
            .and(useAngleControlMode.negate())
            .onTrue(
                drive
                    .runOnce(
                        () -> {
                          headingController.reset();
                          headingController.setGoal(angle);
                        })
                    .withName(String.format("PrepareLockedHeading %s", name)));

        // Then if the button is held for more than 0.2 seconds, drive forward at the
        // angle once the chassis reaches it
        driverXbox
            .pov(pov)
            .debounce(0.2)
            .and(useAngleControlMode.negate())
            .whileTrue(
                drive
                    .run(
                        () -> {
                          double rotationRadians = headingController.calculate();
                          drive.setRobotSpeeds(
                              speedController.updateSpeed(
                                  new ChassisSpeeds(
                                      (headingController.atGoal() ? 1 : 0),
                                      0,
                                      headingController.atGoal() ? 0 : rotationRadians)));
                        })
                    .withName(String.format("ForwardLockedHeading %s", name)));

        // Then once the pov is let go, if we are not at the angle continue turn to it,
        // while also accepting x and y input to drive. Cancel once we get turn request
        driverXbox
            .pov(pov)
            .and(useAngleControlMode.negate())
            .onFalse(
                drive
                    .run(
                        () -> {
                          Translation2d translation = input.getTranslationMetersPerSecond();
                          double rotationRadians = headingController.calculate();
                          drive.setRobotSpeeds(
                              speedController.updateSpeed(
                                  new ChassisSpeeds(
                                      translation.getX(),
                                      translation.getY(),
                                      headingController.atGoal() ? 0 : rotationRadians)),
                              useFieldRelative.getAsBoolean());
                        })
                    .until(() -> input.getOmegaRadiansPerSecond() != 0)
                    .withName(String.format("DriveLockedHeading %s", name)));
      }

      // While X is held down go into stop and go into the cross position to resistent movement,
      // then once X button is let go put modules forward
      driverXbox
          .x()
          .whileTrue(
              drive
                  .startEnd(drive::stopUsingBrakeArrangement, drive::stopUsingForwardArrangement)
                  .withName("StopWithX"));

      // When be is pressed stop the drivetrain then idle it, cancelling all incoming commands.
      // Also do this when robot is disabled
      driverXbox
          .b()
          .or(RobotModeTriggers.disabled())
          .whileTrue(
              drive
                  .runOnce(drive::stop)
                  .andThen(Commands.idle(drive))
                  .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                  .withName("StopCancel"));

      // When right (accelerator/fire) trigger is pressed, put in boost (fast) mode
      driverXbox
          .rightTrigger(0.5)
          .whileTrue(
              Commands.startEnd(
                  () -> speedController.pushSpeedLevel(SpeedLevel.BOOST),
                  () -> speedController.removeSpeedLevel(SpeedLevel.BOOST)));

      // When left (brake/ADS) trigger is is pressed put in precise (slow) mode
      driverXbox
          .leftTrigger(0.5)
          .whileTrue(
              Commands.startEnd(
                  () -> speedController.pushSpeedLevel(SpeedLevel.PRECISE),
                  () -> speedController.removeSpeedLevel(SpeedLevel.PRECISE)));

    } else if (driverController instanceof CommandJoystick) {
      final CommandJoystick driverJoystick = (CommandJoystick) driverController;

      TeleopDriveController input =
          new TeleopDriveController(
              drive,
              () -> -driverJoystick.getY(),
              () -> -driverJoystick.getX(),
              () -> -driverJoystick.getTwist(),
              () -> 0);

      drive.setDefaultCommand(
          drive.startEnd(
              () -> {
                Translation2d translation = input.getTranslationMetersPerSecond();
                double rotation = input.getOmegaRadiansPerSecond();
                drive.setRobotSpeeds(
                    new ChassisSpeeds(translation.getX(), translation.getY(), rotation), false);
              },
              drive::stop));
    }
  }

  private void configureOperatorControllerBindings() {
    if (operatorController instanceof CommandXboxController) {
      final CommandXboxController operatorXbox = (CommandXboxController) operatorController;

      operatorXbox.b().onTrue(Commands.idle(drive).withName("Operator Idle Drive"));
    }
  }

  private Command rumbleControllers(double rumbleIntensity) {
    return Commands.startEnd(
            () -> {
              driverController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
              operatorController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
            },
            () -> {
              driverController.setRumble(RumbleType.kBothRumble, 0);
              operatorController.setRumble(RumbleType.kBothRumble, 0);
            })
        .withName("RumbleController");
  }

  private void configureAlertTriggers() {
    // Endgame alert triggers
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 20)
        .onTrue(rumbleControllers(0.5).withTimeout(0.5));

    RobotModeTriggers.teleop()
        .and(RobotBase::isReal)
        .onChange(rumbleControllers(0.2).withTimeout(0.2));
  }

  private void configureAutos() {
    // Set up named commands for path planner auto
    // https://pathplanner.dev/pplib-named-commands.html
    NamedCommands.registerCommand("StopWithX", drive.runOnce(drive::stopUsingBrakeArrangement));
    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
    autoChooser.addOption("Triangle Auto", new PathPlannerAuto("Triangle Auto"));
    autoChooser.addOption("Rotate Auto", new PathPlannerAuto("Rotate Auto"));
    autoChooser.addOption("Circle Auto", new PathPlannerAuto("Circle Auto"));
  }

  private void configureSysIds() {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
