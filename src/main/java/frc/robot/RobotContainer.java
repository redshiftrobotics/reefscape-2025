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
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOHardware;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIOPhotonVision;
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

  private final Elevator elevator;
  private final Superstructure superstructure;

  // Controller
  private final CommandGenericHID driverController = new CommandXboxController(0);
  private final CommandGenericHID operatorController = new CommandXboxController(1);

  private final Alert driverDisconnected =
      new Alert(
          String.format(
              "Driver xbox controller disconnected (port %s).",
              driverController.getHID().getPort()),
          AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert(
          String.format(
              "Operator xbox controller disconnected (port %s).",
              operatorController.getHID().getPort()),
          AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final Alert notPrimaryBotAlert =
      new Alert(
          "Robot type is not the primary robot type. Be careful you are not using the wrong robot.",
          AlertType.kInfo);
  private final Alert tuningModeActiveAlert =
      new Alert("Tuning mode active, do not use in competition.", AlertType.kWarning);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.getRobot()) {
      case COMP_BOT_2025:
        // Real robot (Wood bot test chassis), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        elevator = new Elevator(new ElevatorIOHardware(ElevatorConstants.ELEVATOR_CONFIG));
        break;

      case WOOD_BOT_TWO_2025:
        // Real robot (Wood bot test chassis), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision =
            new AprilTagVision(
                new CameraIOPhotonVision(VisionConstants.WOODV2_LEFT_CAMERA),
                new CameraIOPhotonVision(VisionConstants.WOODV2_RIGHT_CAMERA));
        elevator = new Elevator(new ElevatorIO() {});
        break;

      case T_SHIRT_CANNON_CHASSIS:
        // Real robot (T-Shirt cannon chassis), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        elevator = new Elevator(new ElevatorIO() {});
        break;

      case CRESCENDO_CHASSIS_2024:
        // Real robot (robot from last year chassis), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        elevator = new Elevator(new ElevatorIO() {});
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
        vision =
            new AprilTagVision(new CameraIOSim(VisionConstants.FRONT_CAMERA, drive::getRobotPose));
        elevator = new Elevator(new ElevatorIOSim());
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
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Superstructure
    superstructure = new Superstructure(elevator);

    // Vision setup
    // vision.setLastRobotPoseSupplier(drive::getRobotPose);
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
    if (Constants.getRobot() != Constants.PRIMARY_ROBOT_TYPE) {
      notPrimaryBotAlert.set(true);
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

    dashboard.setHasVisionEstimate(vision::hasVisionEstimate);

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

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  }

  /** Define button->command mappings. */
  private void configureControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings(true);
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

      // Default command, normal joystick drive
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
                  drive,
                  input::getTranslationMetersPerSecond,
                  input::getOmegaRadiansPerSecond,
                  level::getCurrentSpeedLevel,
                  useFieldRelative::getAsBoolean)
              .withName("Default Drive"));

      // Secondary drive command, angle controlled drive
      driverXbox
          .rightBumper()
          .and(driverXbox.leftTrigger().negate())
          .and(driverXbox.rightTrigger().negate())
          .whileTrue(
              DriveCommands.joystickHeadingDrive(
                      drive,
                      input::getTranslationMetersPerSecond,
                      input::getHeadingDirection,
                      level::getCurrentSpeedLevel,
                      useFieldRelative::getAsBoolean)
                  .withName("Heading Drive"));

      // Cause the robot to resist movement by forming an X shape with the swerve modules
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

      // Reset the gyro heading
      driverXbox
          .start()
          .onTrue(
              drive
                  .runOnce(
                      () ->
                          drive.resetPose(
                              new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero)))
                  .ignoringDisable(true)
                  .withName("Reset Gyro Heading"));

      if (includeAutoAlign) {
        // Align to reef

        final AdaptiveAutoAlignCommands reefAlignmentCommands =
            new AdaptiveAutoAlignCommands(
                Arrays.asList(FieldConstants.Reef.centerFaces),
                new Transform2d(
                    DRIVE_CONFIG.bumperCornerToCorner().getX() / 2 + Units.inchesToMeters(6),
                    0,
                    Rotation2d.kPi));

        driverXbox
            .rightTrigger()
            .onTrue(reefAlignmentCommands.driveToClosest(drive).withName("Algin Reef"))
            .onFalse(reefAlignmentCommands.stop(drive));

        driverXbox
            .rightTrigger()
            .and(driverXbox.leftBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    reefAlignmentCommands.driveToNext(drive).withName("Algin Reef -1")));

        driverXbox
            .rightTrigger()
            .and(driverXbox.rightBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    reefAlignmentCommands.driveToPrevious(drive).withName("Algin Reef +1")));

        // Align to intake

        final AdaptiveAutoAlignCommands intakeAlignmentCommands =
            new AdaptiveAutoAlignCommands(
                Arrays.asList(FieldConstants.CoralStation.alignmentFaces),
                new Transform2d(
                    DRIVE_CONFIG.bumperCornerToCorner().getX() / 2 + Units.inchesToMeters(3),
                    0,
                    Rotation2d.kPi));

        driverXbox
            .leftTrigger()
            .onTrue(intakeAlignmentCommands.driveToClosest(drive).withName("Align Intake"))
            .onFalse(intakeAlignmentCommands.stop(drive));

        driverXbox
            .leftTrigger()
            .and(driverXbox.leftBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    intakeAlignmentCommands.driveToNext(drive).withName("Align Intake +1")));

        driverXbox
            .leftTrigger()
            .and(driverXbox.rightBumper())
            .onTrue(
                CustomCommands.reInitCommand(
                    intakeAlignmentCommands.driveToPrevious(drive).withName("Align Intake -1")));
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

      operatorXbox.povDown().onTrue(elevator.runOnce(() -> elevator.setGoalHeightMeters(0.0)));
      operatorXbox.povRight().onTrue(elevator.runOnce(() -> elevator.setGoalHeightMeters(0.2)));
      operatorXbox.povLeft().onTrue(elevator.runOnce(() -> elevator.setGoalHeightMeters(0.4)));
      operatorXbox
          .povUp()
          .onTrue(
              elevator.runOnce(
                  () -> elevator.setGoalHeightMeters(ElevatorConstants.carriageMaxHeight)));
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

    SmartDashboard.putData(
        "Simple Feed Forward Characterization", DriveCommands.feedforwardCharacterization(drive));
    SmartDashboard.putData(
        "Simple Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    SmartDashboard.putData(
        "Elevator Static Characterization", elevator.staticCharacterization(0.2));
    SmartDashboard.putData("Elevator Coast", elevator.coast());

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
