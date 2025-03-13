package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.AdaptiveAutoAlignCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualAlignCommands;
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
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.HangConstants;
import frc.robot.subsystems.hang.HangIO;
import frc.robot.subsystems.hang.HangIOHardware;
import frc.robot.subsystems.hang.HangIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.State;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOHardware;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.subsystems.superstructure.intake.IntakeIOHardware;
import frc.robot.subsystems.superstructure.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristConstants;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOHardware;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIOPhotonVision;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.utility.JoystickUtil;
import frc.robot.utility.OverrideSwitch;
import frc.robot.utility.commands.CustomCommands;
import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
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

  private final Superstructure superstructure;
  private final Elevator elevator;

  private final Wrist coralWrist;
  private final Intake coralIntake;

  private final Hang hang;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
  private static final Alert testPlansAvailable =
      new Alert(
          "Running with test plans enabled, ensure you are using the correct auto.",
          AlertType.kWarning);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMP_BOT_2025:
        // Real robot (Competition bot with mechanisms), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));

        vision =
            new AprilTagVision(
                new CameraIOPhotonVision(VisionConstants.COMP_FRONT_LEFT_CAMERA),
                new CameraIOPhotonVision(VisionConstants.COMP_FRONT_RIGHT_CAMERA),
                new CameraIOPhotonVision(VisionConstants.COMP_BACK_LEFT_CAMERA),
                new CameraIOPhotonVision(VisionConstants.COMP_BACK_RIGHT_CAMERA));

        elevator = new Elevator(new ElevatorIOHardware(ElevatorConstants.ELEVATOR_CONFIG));
        hang = new Hang(new HangIOHardware(HangConstants.HANG_CONFIG));
        coralWrist = new Wrist(new WristIOHardware(WristConstants.WRIST_CONFIG));
        coralIntake = new Intake(new IntakeIOHardware(IntakeConstants.CORAL_INTAKE_CONFIG));
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
        vision = new AprilTagVision(new CameraIOPhotonVision(VisionConstants.WOODV2_LEFT_CAMERA));
        elevator = new Elevator(new ElevatorIO() {});
        hang = new Hang(new HangIO() {});
        coralWrist = new Wrist(new WristIO() {});
        coralIntake = new Intake(new IntakeIO() {});
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
        hang = new Hang(new HangIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        coralWrist = new Wrist(new WristIO() {});
        coralIntake = new Intake(new IntakeIO() {});
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
        hang = new Hang(new HangIO() {});
        elevator = new Elevator(new ElevatorIO() {});

        coralWrist = new Wrist(new WristIO() {});

        coralIntake = new Intake(new IntakeIO() {});

        break;

      case SIM_BOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new AprilTagVision(
                new CameraIOSim(VisionConstants.COMP_FRONT_LEFT_CAMERA, drive::getRobotPose),
                new CameraIOSim(VisionConstants.COMP_FRONT_RIGHT_CAMERA, drive::getRobotPose),
                new CameraIOSim(VisionConstants.COMP_BACK_LEFT_CAMERA, drive::getRobotPose),
                new CameraIOSim(VisionConstants.COMP_BACK_RIGHT_CAMERA, drive::getRobotPose));

        hang = new Hang(new HangIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        coralWrist = new Wrist(new WristIOSim(WristConstants.WRIST_CONFIG));
        coralIntake = new Intake(new IntakeIOSim());
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
        hang = new Hang(new HangIO() {});
        vision = new AprilTagVision();
        elevator = new Elevator(new ElevatorIO() {});
        coralWrist = new Wrist(new WristIO() {});
        coralIntake = new Intake(new IntakeIO() {});

        break;
    }

    // Superstructure
    superstructure = new Superstructure(elevator, coralWrist, coralIntake);

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

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to auto populate
    // autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());
    registerNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("None", Commands.none());

    // Configure autos
    configureAutos(autoChooser);

    // Alerts for constants to avoid using them in competition
    tuningModeActiveAlert.set(Constants.TUNING_MODE);
    testPlansAvailable.set(Constants.RUNNING_TEST_PLANS);
    notPrimaryBotAlert.set(Constants.getRobot() != Constants.PRIMARY_ROBOT_TYPE);

    // Hide controller missing warnings for sim
    DriverStation.silenceJoystickConnectionWarning(Constants.getMode() != Mode.REAL);

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

    dashboard.setAutoAlignPoseSupplier(AdaptiveAutoAlignCommands::getCurrentAutoAlignGoal);

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
    configureDriverControllerBindings(driverController, true);
    configureOperatorControllerBindings(operatorController);
    configureAlertTriggers();
  }

  private void configureDriverControllerBindings(
      CommandXboxController xbox, boolean includeAutoAlign) {
    final Trigger useFieldRelative =
        new Trigger(new OverrideSwitch(xbox.y(), OverrideSwitch.Mode.TOGGLE, true));

    final Trigger useHeadingControlled =
        new Trigger(
            new OverrideSwitch(
                xbox.rightBumper()
                    .and(xbox.leftTrigger().negate())
                    .and(xbox.rightTrigger().negate()),
                OverrideSwitch.Mode.HOLD,
                false));

    DriverDashboard.getInstance().setFieldRelativeSupplier(useFieldRelative);
    DriverDashboard.getInstance().setHeadingControlledSupplier(useHeadingControlled);

    final JoystickInputController input =
        new JoystickInputController(
            drive,
            () -> -xbox.getLeftY(),
            () -> -xbox.getLeftX(),
            () -> -xbox.getRightY(),
            () -> -xbox.getRightX());

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
            .withName("DEFAULT Drive"));

    // Secondary drive command, angle controlled drive
    useHeadingControlled.whileTrue(
        DriveCommands.joystickHeadingDrive(
                drive,
                input::getTranslationMetersPerSecond,
                input::getHeadingDirection,
                level::getCurrentSpeedLevel,
                useFieldRelative::getAsBoolean)
            .withName("HEADING Drive"));

    // Cause the robot to resist movement by forming an X shape with the swerve modules
    // Helps prevent getting pushed around
    xbox.x().whileTrue(DriveCommands.holdPositionCommand(drive).withName("RESIST Movement With X"));

    // Stop the robot and cancel any running commands
    xbox.b()
        .or(RobotModeTriggers.disabled())
        .onTrue(drive.runOnce(drive::stop).withName("CANCEL and stop"));

    // Reset the gyro heading
    xbox.start()
        .debounce(0.3)
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero)))
                .andThen(rumbleController(xbox, 0.3).withTimeout(0.25))
                .ignoringDisable(true)
                .withName("Reset Gyro Heading"));

    final BiConsumer<Trigger, Command> configureAlignmentAuto =
        (trigger, command) -> {
          trigger.onTrue(command.until(() -> input.getOmegaRadiansPerSecond() != 0));
        };

    configureAlignmentAuto.accept(
        xbox.povRight(), ManualAlignCommands.alignToSourceRight(drive, input));
    configureAlignmentAuto.accept(
        xbox.povLeft(), ManualAlignCommands.alignToSourceLeft(drive, input));
    configureAlignmentAuto.accept(xbox.povDown(), ManualAlignCommands.alignToCageAdv(drive, input));
    configureAlignmentAuto.accept(xbox.povUp(), ManualAlignCommands.alignToReef(drive, input));

    if (includeAutoAlign) {
      // Align to reef

      final AdaptiveAutoAlignCommands reefAlignmentCommands =
          new AdaptiveAutoAlignCommands(
              Arrays.asList(FieldConstants.Reef.alignmentFaces),
              new Transform2d(
                  DRIVE_CONFIG.bumperCornerToCorner().getX() / 2.0 + Units.inchesToMeters(10),
                  0,
                  Rotation2d.k180deg),
              new Transform2d(0, 0, Rotation2d.k180deg),
              new Translation2d(Units.inchesToMeters(6), 0));

      reefAlignmentCommands.setEndCommand(() -> rumbleController(xbox, 0.5).withTimeout(0.1));

      xbox.rightTrigger()
          .onTrue(reefAlignmentCommands.driveToClosest(drive).withName("Algin REEF"))
          .onFalse(reefAlignmentCommands.stop(drive));

      xbox.rightTrigger()
          .and(xbox.leftBumper())
          .onTrue(
              CustomCommands.reInitCommand(
                  reefAlignmentCommands.driveToNext(drive).withName("Algin REEF -1")));

      xbox.rightTrigger()
          .and(xbox.rightBumper())
          .onTrue(
              CustomCommands.reInitCommand(
                  reefAlignmentCommands.driveToPrevious(drive).withName("Algin REEF +1")));

      // Align to intake

      final AdaptiveAutoAlignCommands intakeAlignmentCommands =
          new AdaptiveAutoAlignCommands(
              Arrays.asList(FieldConstants.CoralStation.alignmentFaces),
              new Transform2d(
                  DRIVE_CONFIG.bumperCornerToCorner().getX() / 2.0 + Units.inchesToMeters(14),
                  0,
                  Rotation2d.k180deg),
              new Transform2d(0, 0, Rotation2d.k180deg),
              new Translation2d(Units.inchesToMeters(4), 0));

      intakeAlignmentCommands.setEndCommand(() -> rumbleController(xbox, 0.3).withTimeout(0.1));

      xbox.leftTrigger()
          .onTrue(intakeAlignmentCommands.driveToClosest(drive).withName("Align INTAKE"))
          .onFalse(intakeAlignmentCommands.stop(drive));

      xbox.leftTrigger()
          .and(xbox.leftBumper())
          .onTrue(
              CustomCommands.reInitCommand(
                  intakeAlignmentCommands.driveToNext(drive).withName("Align INTAKE +1")));

      xbox.leftTrigger()
          .and(xbox.rightBumper())
          .onTrue(
              CustomCommands.reInitCommand(
                  intakeAlignmentCommands.driveToPrevious(drive).withName("Align INTAKE -1")));
    }
  }

  private void configureOperatorControllerBindings(CommandXboxController xbox) {

    new Trigger(DriverStation::isEnabled)
        .onTrue(superstructure.runAction(Superstructure.State.STOW_HIGH));

    final double heightOffsetAdjustment = Units.inchesToMeters(1);
    final Rotation2d angleOffsetAdjustment = Rotation2d.fromDegrees(5);

    final BiConsumer<Trigger, Superstructure.State> configureOperatorControllerBindingLevel =
        (trigger, level) -> {
          trigger.whileTrue(superstructure.run(level));

          if (level.isLevel()) {
            trigger
                .and(xbox.rightTrigger())
                .and(superstructure::atGoal)
                .whileTrue(superstructure.runWheels(level));
          } else {
            trigger.whileTrue(superstructure.runWheels(level));
          }

          trigger
              .and(xbox.povUp())
              .onTrue(Commands.runOnce(() -> level.adjustHeight(heightOffsetAdjustment)));
          trigger
              .and(xbox.povDown())
              .onTrue(Commands.runOnce(() -> level.adjustHeight(-heightOffsetAdjustment)));
          trigger
              .and(xbox.povRight())
              .onTrue(Commands.runOnce(() -> level.adjustAngle(angleOffsetAdjustment)));
          trigger
              .and(xbox.povLeft())
              .onTrue(
                  Commands.runOnce(() -> level.adjustAngle(angleOffsetAdjustment.unaryMinus())));
          trigger.and(xbox.back()).onTrue(Commands.runOnce(level::adjustReset));
        };

    configureOperatorControllerBindingLevel.accept(xbox.leftTrigger(), Superstructure.State.INTAKE);

    configureOperatorControllerBindingLevel.accept(xbox.y(), Superstructure.State.L4);
    configureOperatorControllerBindingLevel.accept(xbox.x(), Superstructure.State.L3);
    configureOperatorControllerBindingLevel.accept(xbox.a(), Superstructure.State.L2);
    configureOperatorControllerBindingLevel.accept(xbox.b(), Superstructure.State.L1);

    // Intake

    coralIntake.setDefaultCommand(superstructure.passiveIntake());

    DoubleSupplier intakeSpeed =
        () ->
            MathUtil.clamp(
                JoystickUtil.applyDeadband(-xbox.getLeftX())
                    + JoystickUtil.applyDeadband(-xbox.getLeftY()),
                -1,
                +1);

    new Trigger(() -> intakeSpeed.getAsDouble() != 0)
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(coralIntake.run(() -> coralIntake.setMotors(intakeSpeed.getAsDouble())));

    // Hang
    hang.setDefaultCommand(hang.run(() -> hang.set(JoystickUtil.applyDeadband(xbox.getLeftX()))));

    xbox.rightBumper().and(xbox.leftBumper().negate()).onTrue(hang.deploy());
    xbox.leftBumper().and(xbox.rightBumper().negate()).onTrue(hang.retract());
    xbox.rightBumper().and(xbox.leftBumper()).onTrue(hang.stow());
  }

  private Command rumbleController(CommandXboxController controller, double rumbleIntensity) {
    return Commands.startEnd(
            () -> controller.setRumble(RumbleType.kBothRumble, rumbleIntensity),
            () -> controller.setRumble(RumbleType.kBothRumble, 0))
        .withName("RumbleController");
  }

  private Command rumbleControllers(double rumbleIntensity) {
    return Commands.parallel(
        rumbleController(driverController, rumbleIntensity),
        rumbleController(operatorController, rumbleIntensity));
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

  private void registerNamedCommands() {
    // Set up named commands for path planner auto
    // https://pathplanner.dev/pplib-named-commands.html

    NamedCommands.registerCommand(
        "l1",
        Commands.parallel(
                Commands.runOnce(() -> elevator.setGoalHeightMeters(State.L1.getHeight())),
                Commands.runOnce(() -> coralWrist.setGoalRotation(State.L1.getAngle())))
            .andThen(Commands.waitUntil(superstructure::atGoal))
            .andThen(
                Commands.runEnd(() -> coralIntake.setMotors(1), coralIntake::stopMotors)
                    .withTimeout(0.5)));

    NamedCommands.registerCommand(
        "stow",
        Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeightMeters(State.STOW.getHeight())),
            Commands.runOnce(() -> coralWrist.setGoalRotation(State.STOW.getAngle()))));

    NamedCommands.registerCommand(
        "intake",
        Commands.parallel(
                Commands.runOnce(() -> elevator.setGoalHeightMeters(State.INTAKE.getHeight())),
                Commands.runOnce(() -> coralWrist.setGoalRotation(State.INTAKE.getAngle())))
            .andThen(
                Commands.runEnd(
                    () -> coralIntake.setMotors(-0.6), () -> coralIntake.setMotors(0.05)))
            .withTimeout(4));
  }

  private void configureAutos(LoggedDashboardChooser<Command> dashboardChooser) {

    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
    // dashboardChooser.addOption("Center Test", AutoBuilder.buildAuto("Center-Auto"));
    // dashboardChooser.addOption("Right Test", AutoBuilder.buildAuto("Right-Auto"));
    // dashboardChooser.addOption("Left Test", AutoBuilder.buildAuto("Left-Auto"));

    // // Choreo Autos
    // //
    // https://pathplanner.dev/pplib-choreo-interop.html#load-choreo-trajectory-as-a-pathplannerpath
    // try {
    //   dashboardChooser.addOption(
    //       "Four Coral Test",
    //       AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Four Coral Auto")));
    // } catch (IOException e) {
    //   System.out.println("Failed to load Choreo auto " + e.getMessage());
    // } catch (ParseException e) {
    //   System.out.println("Failed to parse Choreo auto " + e.getMessage());
    // }

    if (Constants.RUNNING_TEST_PLANS) {
      dashboardChooser.addOption("[TEST] Stow Hang Arm", hang.stow());
      dashboardChooser.addOption("[TEST] Deploy Hang Arm", hang.deploy());
      dashboardChooser.addOption("[TEST] Retract Hang Arm", hang.retract());

      dashboardChooser.addOption(
          "[Characterization] Elevator Static Forward", elevator.staticCharacterization(0.02));
      dashboardChooser.addOption(
          "[Characterization] Drive Feed Forward",
          DriveCommands.feedforwardCharacterization(drive));
      dashboardChooser.addOption(
          "[Characterization] Drive Wheel Radius",
          DriveCommands.wheelRadiusCharacterization(drive));
    }
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
