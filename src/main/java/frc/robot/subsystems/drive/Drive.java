package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.utility.AllianceFlipUtil;
import frc.robot.utility.LocalADStarAK;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve drivetrain (chassis) of robot. This contains four swerve modules and a gyro */
public class Drive extends SubsystemBase {

  // https://www.geeksforgeeks.org/reentrant-lock-java/
  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Alert gyroConnectionAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

  private final Module[] modules; // FL, FR, BL, BR

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private final SysIdRoutine sysId;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private SwerveModulePosition[] lastModulePositions;

  private Rotation2d rawGyroRotation = new Rotation2d();
  private Pose2d pose = new Pose2d();

  /**
   * Creates a new drivetrain for robot
   *
   * @param gyroIO gyroscope for yaw
   * @param flModuleIO front left swerve module
   * @param frModuleIO front right swerve module
   * @param blModuleIO back left swerve module
   * @param brModuleIO back right swerve module
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    // --- Save components ---

    // save gyro
    this.gyroIO = gyroIO;

    // Create and save modules and give them position
    modules =
        new Module[] {
          new Module(flModuleIO, DriveConstants.FRONT_LEFT_MODULE_DISTANCE_FROM_CENTER),
          new Module(frModuleIO, DriveConstants.FRONT_RIGHT_MODULE_DISTANCE_FROM_CENTER),
          new Module(blModuleIO, DriveConstants.BACK_LEFT_MODULE_DISTANCE_FROM_CENTER),
          new Module(brModuleIO, DriveConstants.BACK_RIGHT_MODULE_DISTANCE_FROM_CENTER)
        };

    // --- Set up kinematics ---

    Translation2d[] moduleTranslations =
        modules().map(Module::getDistanceFromCenter).toArray(Translation2d[]::new);

    kinematics = new SwerveDriveKinematics(moduleTranslations);

    // --- Set up odometry ---

    lastModulePositions = modules().map(Module::getPosition).toArray(SwerveModulePosition[]::new);
    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, pose);

    // --- Start odometry threads ---

    // Start threads (does nothing if no signals have been created)
    SparkOdometryThread.getInstance().start();

    // --- PathPlanner ---

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotSpeeds,
        (speeds, feedForward) -> setRobotSpeeds(speeds),
        new PPHolonomicDriveController(
            DriveConstants.driveControllerConstants.toPathPlannerPIDConstants(),
            DriveConstants.rotationControllerConstants.toPathPlannerPIDConstants(),
            Constants.LOOP_PERIOD_SECONDS),
        DriveConstants.pathPlannerRobotConfig,
        AllianceFlipUtil::shouldFlip,
        this);

    Pathfinding.setPathfinder(
        new LocalADStarAK()); // https://pathplanner.dev/pplib-pathfinding.html#advantagekit-compatibility

    PathfindingCommand.warmupCommand();

    PathPlannerLogging.setLogActivePathCallback(
        activePath ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(Pose2d[]::new)));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    // --- Configure SysId ---

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    // Open the SysId tool

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Units.Volts)), null, this));

    // --- Break mode ---
    setMotorBrakeMode(true);
  }

  // --- Robot Pose ---

  /**
   * Periodic of drivetrain, is called every command scheduler loop (20ms). Updates pose with
   * odometry.
   */
  @Override
  public void periodic() {

    // Prevents odometry updates while reading data, this is needed as odometry is handed on a
    // different thread
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    modules().forEach(Module::updateInputs);
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      stop();
    }

    // Log current wheel speeds
    Logger.recordOutput("SwerveStates/MeasuredWheelSpeeds", getWheelSpeeds());
    Logger.recordOutput("SwerveStates/ModuleDesiredWheelSpeeds", getDesiredWheelSpeeds());

    // Log current chassis speeds
    Logger.recordOutput("ChassisStates/MeasuredRobotSpeeds", getRobotSpeeds());
    Logger.recordOutput(
        "ChassisStates/ModuleDesiredSpeeds", kinematics.toChassisSpeeds(getDesiredWheelSpeeds()));

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together, use first
    int sampleCount = sampleTimestamps.length;

    // for each new odometry sample
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];

      // Read wheel positions from each module, and calculate delta using
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {

        SwerveModulePosition modulePosition = modules[moduleIndex].getOdometryPositions()[i];

        modulePositions[moduleIndex] = modulePosition;

        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                modulePosition.angle);

        lastModulePositions[moduleIndex] = modulePosition;
      }

      // Update gyro angle
      if (gyroInputs.connected && !Constants.ON_BLOCKS_TEST_MODE) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Fallback option: use the delta of swerve module to create estimated amount twisted
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update to pose estimator
      pose = poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroConnectionAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /**
   * Get estimated position of robot from swerve drive position estimator
   *
   * @return the estimated position of robot
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /**
   * Set the current estimated position of robot.
   *
   * @param pose new position robot believes it is located at
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getWheelPositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose the pose of the robot as measured by the vision camera.
   * @param timestamp the timestamp of the vision measurement in seconds. You must use a timestamp
   *     with an epoch since FPGA time startup.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /**
   * Adds a vision measurement to the pose estimator with standard deviations.
   *
   * @param visionPose the pose of the robot as measured by the vision camera.
   * @param timestamp the timestamp of the vision measurement in seconds. You must use a timestamp
   *     with an epoch since FPGA time startup.
   * @param standardDeviations standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta], with units in meters and radians.
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestamp, Matrix<N3, N1> standardDeviations) {
    poseEstimator.setVisionMeasurementStdDevs(standardDeviations);
    addVisionMeasurement(visionPose, timestamp);
  }

  // --- Robot Speeds ---

  /**
   * Get robot relative velocity of robot chassis
   *
   * @return translational speed in meters/sec and rotation speed in radians/sec
   */
  public ChassisSpeeds getRobotSpeeds() {
    return getRobotSpeeds(false);
  }

  /**
   * Get velocity of robot chassis, either robot or field relative.
   *
   * @param fieldRelative true if velocity is relative to field, false if relative to chassis
   * @return translational speed in meters/sec and rotation speed in radians/sec
   */
  public ChassisSpeeds getRobotSpeeds(boolean fieldRelative) {
    SwerveModuleState[] wheelSpeeds = getWheelSpeeds();

    ChassisSpeeds speeds = kinematics.toChassisSpeeds(wheelSpeeds);

    if (fieldRelative) {
      speeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              speeds, AllianceFlipUtil.apply(getPose().getRotation()));
    }

    return speeds;
  }

  /**
   * Set desired robot relative velocity of robot chassis.
   *
   * @param speeds translational speed in meters/sec and rotation speed in radians/sec
   */
  public void setRobotSpeeds(ChassisSpeeds speeds) {
    setRobotSpeeds(speeds, false);
  }

  /**
   * Runs the drive at the desired velocity, either robot or field relative.
   *
   * @param speeds translational speed in meters/sec and rotation speed in radians/sec
   * @param fieldRelative true if velocity is relative to field, false if relative to chassis
   */
  public void setRobotSpeeds(ChassisSpeeds speeds, boolean fieldRelative) {

    if (fieldRelative) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds, AllianceFlipUtil.apply(getPose().getRotation()));
    }

    Logger.recordOutput("ChassisStates/DesiredRobotSpeeds", speeds);

    speeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SECONDS);

    SwerveModuleState[] wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    setWheelSpeeds(wheelSpeeds);
  }

  // --- Wheel States ---

  /**
   * Set desired swerve modules for each swerve module. Each wheel state is a turn angle and drive
   * velocity in meters/second.
   *
   * @param speeds array of {@link SwerveDriveWheelStates} which contains of all desired swerve
   *     module states
   */
  public void setWheelSpeeds(SwerveModuleState[] speeds) {

    SwerveDriveKinematics.desaturateWheelSpeeds(speeds, getMaxLinearSpeedMetersPerSec());

    Logger.recordOutput("SwerveStates/DesiredWheelSpeeds", speeds);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setSpeeds(speeds[i]);
    }
  }

  /**
   * Get measured swerve module speeds for each swerve module. Each wheel state is a turn angle and
   * drive velocity in meters/second.
   *
   * @return array of {@link SwerveModuleState} which contains an array of all swerve module states
   */
  public SwerveModuleState[] getWheelSpeeds() {
    return modules().map(Module::getSpeeds).toArray(SwerveModuleState[]::new);
  }

  /**
   * Get desired swerve module desired speeds for each swerve module. Each wheel state is a turn
   * angle and drive velocity in meters/second.
   *
   * @return array of {@link SwerveModuleState} which contains all desired swerve module states.
   */
  public SwerveModuleState[] getDesiredWheelSpeeds() {
    return modules().map(Module::getDesiredState).toArray(SwerveModuleState[]::new);
  }

  // --- Wheel Positions ---

  /**
   * Get measured swerve module position from each swerve module. Each wheel position is a turn
   * angle and drive position in meters
   *
   * @return array of {@link SwerveModulePosition} which contains all swerve module positions
   */
  public SwerveModulePosition[] getWheelPositions() {
    return modules().map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  // --- Stops ---

  /**
   * Stops the drive. The modules will return to their normal driving the next time a nonzero
   * velocity is requested.
   */
  public void stop() {
    setRobotSpeeds(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopUsingBrakeArrangement() {
    Rotation2d[] headings =
        modules()
            .map(Module::getDistanceFromCenter)
            .map(Translation2d::getAngle)
            .toArray(Rotation2d[]::new);
    kinematics.resetHeadings(headings);
    setRobotSpeeds(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to their forward position. The modules will return to
   * their normal driving the next time a nonzero velocity is requested.
   */
  public void stopUsingForwardArrangement() {
    Rotation2d[] headings = modules().map(module -> new Rotation2d()).toArray(Rotation2d[]::new);
    kinematics.resetHeadings(headings);
    setRobotSpeeds(new ChassisSpeeds());
  }

  // --- Break Mode ---

  /**
   * Sets whether swerve motors will brake to prevent coasting. IMPORTANT: Only do this after robot
   * has been stopped for a bit
   *
   * @param enabled true if motors should brake, false if they should coast
   */
  public void setMotorBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      modules().forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Get whether swerve motors will brake to prevent coasting, and robot is safe to drive
   *
   * @return true if motors should brake, false if they should coast
   */
  public boolean getMotorBrakeMode() {
    return brakeModeEnabled;
  }

  // --- Chassis Max Speeds---

  /** Returns the maximum linear speed in meters per second. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DRIVE_CONFIG.maxLinearVelocity();
  }

  /** Returns the maximum angular speed in radians per second. */
  public double getMaxAngularSpeedRadPerSec() {
    return DRIVE_CONFIG.maxAngularVelocity();
  }

  // --- SysId ---

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterization(double volts) {
    for (Module module : modules) {
      module.runCharacterization(0, volts);
    }
  }

  // --- Characterization ---

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  // --- Module Util ---

  /** Utility method. Get stream of modules */
  private Stream<Module> modules() {
    return Arrays.stream(modules);
  }
}
