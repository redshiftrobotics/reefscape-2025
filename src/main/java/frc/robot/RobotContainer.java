package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.AdaptiveAutoAlignCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.controllers.JoystickInputController;
import frc.robot.commands.controllers.SpeedLevelController;
import frc.robot.subsystems.dashboard.DriverDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleConstants;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.utility.OverrideSwitch;
import frc.robot.utility.commands.CustomCommands;
import java.io.IOException;
import java.util.Arrays;
import org.json.simple.parser.ParseException;
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
  private final AprilTagVision vision;

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
      case WOOD_BOT_TWO_2025, T_SHIRT_CANNON_CHASSIS:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        break;

      case CRESCENDO_CHASSIS_2024:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        break;

      case SIM_BOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSim(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSim(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSim(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision(new CameraIOSim(VisionConstants.FRONT_CAMERA, drive::getRobotPose));
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
        vision = new AprilTagVision();
        break;
    }

    vision.addVisionEstimateConsumer(
        (estimate) -> {
          if (estimate.status().isSuccess() && Constants.getMode() != Mode.SIM) {
            drive.addVisionMeasurement(
                estimate.robotPose().toPose2d(),
                estimate.timestampSeconds(),
                estimate.standardDeviations());
          }
        });

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to
    // auto populate
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());

    // Configure autos
    configureAutos(autoChooser);

    // Configure sys ids
    if (Constants.TUNING_MODE) {
      configureSysIds(autoChooser);
    }

    // Alerts for constants to avoid using them in competition
    if (Constants.TUNING_MODE) {
      tuningModeActiveAlert.set(true);
    }
    if (Constants.ON_BLOCKS_TEST_MODE) {
      onBlockMode.set(true);
    }
    if (DriverStation.isFMSAttached()
        && Constants.getRobot() != Constants.RobotType.WOOD_BOT_TWO_2025) {
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
    dashboard.setPoseSupplier(drive::getRobotPose);
    dashboard.setRobotSupplier(drive::getRobotSpeeds);
    dashboard.setFieldRelativeSupplier(() -> false);

    dashboard.addCommand("Reset Pose", () -> drive.resetPose(new Pose2d()), true);
    dashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () ->
                drive.resetPose(
                    new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero))),
        true);

    dashboard.addCommand(
        "Reset To Reef",
        () ->
            drive.resetPose(
                FieldConstants.Reef.centerFaces[0].transformBy(
                    new Transform2d(
                        DRIVE_CONFIG.bumperCornerToCorner().getX() / 2, 0, Rotation2d.kPi))),
        true);
  }

  /** Define button->command mappings. */
  private void configureControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings(false);
    configureOperatorControllerBindings();
    configureAlertTriggers();
  }

  private void configureDriverControllerBindings(boolean includeAutoAlign) {
    if (driverController instanceof CommandXboxController) {
      final CommandXboxController driverXbox = (CommandXboxController) driverController;

      final Trigger useFieldRelative =
          new Trigger(new OverrideSwitch(driverXbox.y(), OverrideSwitch.Mode.TOGGLE, true));

      DriverDashboard.getInstance().setFieldRelativeSupplier(useFieldRelative);

      final JoystickInputController input =
          new JoystickInputController(
              drive,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              () -> -driverXbox.getRightY(),
              () -> -driverXbox.getRightX());

      final SpeedLevelController level =
          new SpeedLevelController(SpeedLevelController.SpeedLevel.NO_LEVEL);

      // Default command
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
                  drive,
                  input::getTranslationMetersPerSecond,
                  input::getOmegaRadiansPerSecond,
                  level::getCurrentSpeedLevel,
                  useFieldRelative::getAsBoolean)
              .withName("Default Drive"));

      // Cause the robot to resist movement by forming an X shape with the swerve
      // modules
      // Helps prevent getting pushed around
      driverXbox
          .x()
          .whileTrue(
              drive
                  .startEnd(drive::stopUsingBrakeArrangement, drive::stopUsingForwardArrangement)
                  .withName("Resist Movement With X"));

      // Stop the robot and cancel any running commands
      driverXbox
          .b()
          .or(RobotModeTriggers.disabled())
          .onTrue(drive.runOnce(drive::stop).withName("Stop and Cancel"));

      if (includeAutoAlign) {
        // Align to reef
        final AdaptiveAutoAlignCommands reefAlignmentCommands =
            new AdaptiveAutoAlignCommands(Arrays.asList(FieldConstants.Reef.alignmentFaces));

        driverXbox
            .rightTrigger()
            .onTrue(reefAlignmentCommands.driveToClosest(drive).withName("Drive to reef"));

        driverXbox
            .rightTrigger()
            .and(driverXbox.leftBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    reefAlignmentCommands.driveToNext(drive).withName("Drive to next reef")));

        driverXbox
            .rightTrigger()
            .and(driverXbox.rightBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    reefAlignmentCommands
                        .driveToPrevious(drive)
                        .withName("Drive to previous reef")));

        driverXbox.rightTrigger(0.1).onFalse(drive.runOnce(drive::stop));

        // Align to intake

        final AdaptiveAutoAlignCommands intakeAlignmentCommands =
            new AdaptiveAutoAlignCommands(
                Arrays.asList(FieldConstants.CoralStation.alignmentFaces));

        driverXbox
            .leftTrigger()
            .onTrue(intakeAlignmentCommands.driveToClosest(drive).withName("Drive to intake"));

        driverXbox
            .leftTrigger()
            .and(driverXbox.leftBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    intakeAlignmentCommands.driveToNext(drive).withName("Drive to next intake")));

        driverXbox
            .leftTrigger()
            .and(driverXbox.rightBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    intakeAlignmentCommands
                        .driveToPrevious(drive)
                        .withName("Drive to previous intake")));

        driverXbox.leftTrigger(0.1).onFalse(drive.runOnce(drive::stop));
      }
    } else if (driverController instanceof CommandJoystick) {
      final CommandJoystick driverJoystick = (CommandJoystick) driverController;

      JoystickInputController driverController =
          new JoystickInputController(
              drive,
              () -> -driverJoystick.getY(),
              () -> -driverJoystick.getX(),
              () -> -driverJoystick.getTwist(),
              () -> 0.0);

      drive.setDefaultCommand(
          drive
              .run(
                  () -> {
                    Translation2d translation = driverController.getTranslationMetersPerSecond();
                    double omega = driverController.getOmegaRadiansPerSecond();
                    drive.setRobotSpeeds(
                        new ChassisSpeeds(translation.getX(), translation.getY(), omega));
                  })
              .finallyDo(drive::stop)
              .withName("Default Drive"));
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

  private void configureAutos(LoggedDashboardChooser<Command> dashboardChooser) {
    // Set up named commands for path planner auto
    // https://pathplanner.dev/pplib-named-commands.html
    NamedCommands.registerCommand("StopWithX", drive.runOnce(drive::stopUsingBrakeArrangement));

    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
    dashboardChooser.addOption("Triangle Auto", AutoBuilder.buildAuto("Triangle Auto"));
    dashboardChooser.addOption("Rotate Auto", AutoBuilder.buildAuto("Rotate Auto"));
    dashboardChooser.addOption("Circle Auto", AutoBuilder.buildAuto("Circle Auto"));

    // Choreo Autos
    // https://pathplanner.dev/pplib-choreo-interop.html#load-choreo-trajectory-as-a-pathplannerpath
    try {
      dashboardChooser.addOption(
          "Four Coral Test",
          AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Four Coral Auto")));
    } catch (IOException e) {
      System.out.println("Failed to load Choreo auto " + e.getMessage());
    } catch (ParseException e) {
      System.out.println("Failed to parse Choreo auto " + e.getMessage());
    }
  }

  private void configureSysIds(LoggedDashboardChooser<Command> dashboardChooser) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    dashboardChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    dashboardChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    dashboardChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    dashboardChooser.addOption(
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
