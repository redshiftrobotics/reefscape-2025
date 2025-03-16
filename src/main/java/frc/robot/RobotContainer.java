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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.led.LEDSubsystem;
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
import frc.robot.subsystems.superstructure.intake.sensor.Sensor;
import frc.robot.subsystems.superstructure.intake.sensor.SensorIO;
import frc.robot.subsystems.superstructure.intake.sensor.SensorIOBeam;
import frc.robot.subsystems.superstructure.intake.sensor.SensorIOSim;
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
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
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

  private final Sensor sensor;

  private final Hang hang;

  private final LEDSubsystem ledSubsystem = new LEDSubsystem(LEDConstants.PWM_PORTS);

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
        sensor = new Sensor(new SensorIOBeam());
        coralIntake = new Intake(new IntakeIOHardware(IntakeConstants.CORAL_INTAKE_CONFIG), sensor);
        break;

      case WOOD_BOT_TWO_2025:
      case T_SHIRT_CANNON_CHASSIS:
      case CRESCENDO_CHASSIS_2024:
        // Various other chassis we have
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
        sensor = new Sensor(new SensorIO() {});
        coralIntake = new Intake(new IntakeIO() {}, sensor);
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
        sensor = new Sensor(new SensorIOSim());
        coralIntake = new Intake(new IntakeIOSim(), sensor);
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
        sensor = new Sensor(new SensorIO() {});
        coralIntake = new Intake(new IntakeIO() {}, sensor);
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
    registerNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());
    // autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
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

    dashboard.setHasVisionEstimateSupplier(vision::hasVisionEstimateDebounce);

    dashboard.setSensorSuppliers(
        coralIntake::usingSensor, () -> coralIntake.hasCoral().orElse(false));
    dashboard.setHangSuppliers(hang::getRawValue, hang::withinSafeToleranceSoftLimits);
    dashboard.setSuperstructureAtGoal(superstructure::atGoal);

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
    configureOperatorControllerBindings(operatorController, false);
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
                  DRIVE_CONFIG.bumperCornerToCorner().getX() / 2.0, 0, Rotation2d.k180deg),
              new Transform2d(Units.inchesToMeters(-4.5), 0, Rotation2d.k180deg),
              new Translation2d(Units.inchesToMeters(6), 0));

      reefAlignmentCommands.setEndCommand(() -> rumbleController(xbox, 0.5).withTimeout(0.1));

      xbox.rightTrigger()
          .and(vision::hasVisionEstimateDebounce)
          .onTrue(reefAlignmentCommands.driveToClosest(drive).withName("Algin REEF"));

      xbox.rightTrigger().onFalse(reefAlignmentCommands.stop(drive));

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
                  DRIVE_CONFIG.bumperCornerToCorner().getX() / 2.0, 0, Rotation2d.k180deg),
              new Transform2d(Units.inchesToMeters(-1), 0, Rotation2d.k180deg),
              new Translation2d(Units.inchesToMeters(4), 0));

      intakeAlignmentCommands.setEndCommand(() -> rumbleController(xbox, 0.5).withTimeout(0.1));

      xbox.leftTrigger()
          .and(vision::hasVisionEstimateDebounce)
          .onTrue(intakeAlignmentCommands.driveToClosest(drive).withName("Align INTAKE"));

      xbox.leftTrigger().onFalse(intakeAlignmentCommands.stop(drive));

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

  private void configureOperatorControllerBindings(
      CommandXboxController xbox, boolean hangSetpoints) {

    // Enable

    new Trigger(DriverStation::isTeleopEnabled)
        .onTrue(
            Commands.defer(
                () -> superstructure.runAction(superstructure.startState),
                Set.of(elevator, coralWrist)));

    coralWrist.setSlowModeSupplier(() -> coralIntake.hasCoral().orElse(false));

    new Trigger(() -> coralIntake.hasCoral().orElse(false))
        .onChange(rumbleControllers(0.5).withTimeout(0.1));

    // Intake and score

    final double heightOffsetAdjustment = Units.inchesToMeters(1);
    final Rotation2d angleOffsetAdjustment = Rotation2d.fromDegrees(5);

    final BiConsumer<Trigger, Superstructure.State> configureOperatorControllerBindingLevel =
        (trigger, level) -> {
          if (level.isLevel()) {
            if (level.equals(Superstructure.State.L1)) {
              trigger.whileTrue(superstructure.run(level));
            } else {
              trigger.whileTrue(superstructure.run2(level));
            }
            // trigger.whileTrue(superstructure.run(level));
          } else if (level.isIntake()) {
            trigger.whileTrue(superstructure.run(level));
          }

          if (level.isLevel()) {
            trigger
                .and(xbox.rightTrigger())
                .and(superstructure::atGoal)
                .whileTrue(superstructure.runWheels(level))
                .onTrue(Commands.runOnce(sensor::simulateItemEjection));
          } else if (level.isIntake()) {
            trigger.whileTrue(
                superstructure.runWheels(level).until(() -> coralIntake.hasCoral().orElse(false)));
            trigger
                .and(superstructure::atGoal)
                .onTrue(Commands.runOnce(sensor::simulateItemRequest));
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

    final Trigger anyButton = xbox.a().or(xbox.x()).or(xbox.y()).or(xbox.b());

    configureOperatorControllerBindingLevel.accept(xbox.y(), Superstructure.State.L4);
    configureOperatorControllerBindingLevel.accept(xbox.x(), Superstructure.State.L3);
    configureOperatorControllerBindingLevel.accept(xbox.a(), Superstructure.State.L2);
    configureOperatorControllerBindingLevel.accept(xbox.b(), Superstructure.State.L1);

    final double intakeManualSpeed = 0.24;
    anyButton.and(xbox.rightBumper()).whileTrue(coralIntake.runMotors(-intakeManualSpeed));
    anyButton.and(xbox.leftBumper()).whileTrue(coralIntake.runMotors(+intakeManualSpeed));

    xbox.rightStick()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(() -> elevator.setGoalHeightMeters(State.STOW_HIGH.getHeight())),
                Commands.runOnce(() -> coralWrist.setGoalRotation(State.STOW_HIGH.getAngle()))));

    // Intake

    coralIntake.setDefaultCommand(superstructure.passiveIntake());

    xbox.start()
        .debounce(0.3)
        .onTrue(
            drive
                .runOnce(coralIntake::toggleUseSensor)
                .andThen(rumbleController(xbox, 0.3).withTimeout(0.25))
                .ignoringDisable(true)
                .withName("Toggle Sensor Use"));

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

    DoubleSupplier hangSpeed =
        () ->
            MathUtil.clamp(
                JoystickUtil.applyDeadband(-xbox.getRightX())
                    + JoystickUtil.applyDeadband(-xbox.getRightY()),
                -1,
                +1);

    // new Trigger(() -> hangSpeed.getAsDouble() != 0)
    //     .and(DriverStation::isTeleopEnabled)
    //     .whileTrue(hang.run(() -> hang.set(hangSpeed.getAsDouble())).finallyDo(hang::stop));

    Function<RumbleType, Command> rumble =
        (rumbleType) ->
            Commands.runEnd(
                    () -> xbox.setRumble(rumbleType, 1), () -> xbox.setRumble(rumbleType, 0))
                .withTimeout(0.2);

    if (hangSetpoints) {
      // TODO for district comps!
      xbox.rightBumper()
          .debounce(0.1)
          .and(xbox.leftBumper().negate().debounce(0.1))
          .and(anyButton.negate())
          .onTrue(hang.retract().andThen(rumble.apply(RumbleType.kRightRumble)));
      xbox.leftBumper()
          .debounce(0.1)
          .and(xbox.rightBumper().negate().debounce(0.1))
          .and(anyButton.negate())
          .onTrue(hang.deploy().andThen(rumble.apply(RumbleType.kLeftRumble)));
      xbox.rightBumper()
          .and(xbox.leftBumper())
          .and(anyButton.negate())
          .onTrue(hang.stow().andThen(rumble.apply(RumbleType.kBothRumble)));
    } else {
      xbox.rightBumper().whileTrue(hang.runSet(-1));
      xbox.leftBumper().whileTrue(hang.runSet(+1));
    }
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
        "l1_stow",
        Commands.sequence(
            Commands.runOnce(() -> elevator.setGoalHeightMeters(State.L1.getHeight())),
            Commands.runOnce(() -> coralWrist.setGoalRotation(State.L1.getAngle()))));
    NamedCommands.registerCommand(
        "l1",
        Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(() -> elevator.setGoalHeightMeters(State.L1.getHeight())),
                    Commands.runOnce(() -> coralWrist.setGoalRotation(State.L1.getAngle()))),
                Commands.waitUntil(superstructure::atGoal).withTimeout(3),
                Commands.runEnd(
                        () -> {
                          coralIntake.setLeftMotor(0.8);
                          coralIntake.setRightMotor(0.6);
                        },
                        coralIntake::stopMotors)
                    .withTimeout(0.5))
            .finallyDo(() -> superstructure.startState = State.L1));

    NamedCommands.registerCommand(
        "l4",
        Commands.sequence(
                Commands.runOnce(() -> coralWrist.setGoalRotation(State.STOW_HIGHER.getAngle())),
                Commands.waitUntil(coralWrist::atGoal).withTimeout(0.6),
                Commands.parallel(
                    Commands.runOnce(() -> elevator.setGoalHeightMeters(State.L4.getHeight())),
                    Commands.runOnce(
                        () -> coralWrist.setGoalRotation(State.STOW_HIGHER.getAngle()))),
                Commands.waitUntil(elevator::atGoalHeightRough).withTimeout(2),
                Commands.parallel(
                    Commands.runOnce(() -> elevator.setGoalHeightMeters(State.L4.getHeight())),
                    Commands.runOnce(() -> coralWrist.setGoalRotation(State.L4.getAngle()))),
                Commands.waitUntil(superstructure::atGoal).withTimeout(2),
                Commands.waitSeconds(0.1),
                Commands.runEnd(() -> coralIntake.setMotors(-1), coralIntake::stopMotors)
                    .withTimeout(0.5)
                    .alongWith(Commands.runOnce(sensor::simulateItemEjection)),
                Commands.runOnce(() -> coralWrist.setGoalRotation(State.STOW_HIGHER.getAngle())),
                Commands.waitUntil(coralWrist::atGoal).withTimeout(0.6))
            .finallyDo(() -> coralWrist.setGoalRotation(State.STOW_HIGHER.getAngle()))
            .finallyDo(() -> elevator.setGoalHeightMeters(State.L4.getHeight()))
            .finallyDo(() -> superstructure.startState = State.L4_STOW));

    NamedCommands.registerCommand(
        "intake_prep",
        Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeightMeters(State.INTAKE.getHeight())),
            Commands.runOnce(() -> coralWrist.setGoalRotation(State.INTAKE.getAngle()))));

    NamedCommands.registerCommand(
        "stow",
        Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeightMeters(State.STOW_HIGHER.getHeight())),
            Commands.runOnce(() -> coralWrist.setGoalRotation(State.STOW_HIGHER.getAngle()))));

    NamedCommands.registerCommand(
        "intake",
        Commands.deadline(
            Commands.parallel(
                    Commands.runOnce(() -> elevator.setGoalHeightMeters(State.INTAKE.getHeight())),
                    Commands.runOnce(() -> coralWrist.setGoalRotation(State.INTAKE.getAngle())),
                    Commands.runEnd(
                            () -> coralIntake.setMotors(-0.6), () -> coralIntake.setMotors(0.0))
                        .until(() -> coralIntake.hasCoral().orElse(false)))
                .withTimeout(3),
            Commands.waitUntil(superstructure::atGoal)
                .withTimeout(3)
                .andThen(sensor::simulateItemRequest)));
  }

  private void configureAutos(LoggedDashboardChooser<Command> dashboardChooser) {

    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
    dashboardChooser.addOption("Center Test", AutoBuilder.buildAuto("Center-Auto"));
    dashboardChooser.addOption("Right Test", AutoBuilder.buildAuto("Right-Auto"));
    dashboardChooser.addOption("Left Test", AutoBuilder.buildAuto("Left-Auto"));
    // dashboardChooser.addOption("SAFE-SAFE-SAFE", AutoBuilder.buildAuto("TEST SAFE AUTO"));

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
