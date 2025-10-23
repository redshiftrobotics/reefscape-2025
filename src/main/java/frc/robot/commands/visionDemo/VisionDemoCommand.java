package frc.robot.commands.visionDemo;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.controllers.SimpleDriveController;
import frc.robot.commands.visionDemo.SuperstructureUtil.SuperstructureState;
import frc.robot.commands.visionDemo.filters.ComboAngleFilter;
import frc.robot.commands.visionDemo.filters.ComboFilter;
import frc.robot.commands.visionDemo.filters.MeanPoseFilterTimeBased;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.BlinkenLEDPattern;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionDemoCommand extends Command {

  private final boolean DO_SLEW = false;

  public interface VisionDemoMode {

    public default void reset() {}

    public ResultSaftyMode getExspectedSaftyMode();

    public Pose2d getRawPose(Pose2d robotPose, Pose3d tagPose);

    public VisionDemoResult calculate(Pose2d robotPose, Pose3d tagPose, double dt);

    public default VisionDemoResult calculate(
        Pose2d robotPose, Pose3d tagPose, double dt, ContainmentBox box) {
      return calculate(robotPose, tagPose, dt);
    }

    public Optional<SuperstructureState> getSuperstructureState(Pose2d robotPose, Pose3d tagPose3d);
  }

  public record VisionDemoResult(Optional<Pose2d> setpointPose, ResultSaftyMode saftyMode) {}

  public enum ResultSaftyMode {
    FULL_CONTROL,
    ROTATIONAL_CONTROL,
    STOP,
    UNKNOWN;
  }

  private final Drive drive;
  private final TagFollowingVision vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final LEDSubsystem leds;

  private final VisionDemoMode mode;

  private final ContainmentBox box;

  private final SimpleDriveController controller = new SimpleDriveController();
  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  private final ComboFilter elevatorHeightFilter = new ComboFilter(3, 5);
  private final ComboAngleFilter wristAngleFilter = new ComboAngleFilter(3, 5);

  private ResultSaftyMode saftyMode = ResultSaftyMode.STOP;

  private final Timer deltaTimeTimer = new Timer();

  private final MeanPoseFilterTimeBased setpointStablityCheckingFilter =
      new MeanPoseFilterTimeBased(0.6);

  private final SlewRateLimiter vxLimiter = new SlewRateLimiter(3); // m/s^2
  private final SlewRateLimiter vyLimiter = new SlewRateLimiter(3); // m/s^2
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(3); // rad/s^2

  public VisionDemoCommand(
      Drive drive,
      TagFollowingVision vision,
      Elevator elevator,
      Wrist wrist,
      LEDSubsystem leds,
      VisionDemoMode mode,
      ContainmentBox box) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.wrist = wrist;
    this.leds = leds;
    this.mode = mode;
    this.box = box;

    controller.setTolerance(
        new Pose2d(Units.inchesToMeters(4), Units.inchesToMeters(4), Rotation2d.fromDegrees(5)));

    addRequirements(drive, elevator, wrist, leds);
  }

  @Override
  public void initialize() {
    controller.reset(drive.getRobotPose());
    controller.setSetpoint(drive.getRobotPose());
    mode.reset();
    deltaTimeTimer.restart();
    saftyMode = ResultSaftyMode.UNKNOWN;
    Logger.recordOutput("TagFollowing/Box/Coners", box.getCorners());
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getRobotPose();

    // --- Target Logging ---

    Logger.recordOutput("TagFollowing/Result/SaftyMode", saftyMode.toString());
    updateLEDS(vision.isTracking());

    vision
        .getLastTagPose()
        .ifPresent(
            tagPose -> {
              Pose2d rawSetpointPose = mode.getRawPose(robotPose, tagPose);
              Logger.recordOutput("TagFollowing/RobotRawSetpointPose", rawSetpointPose);
              SmartDashboard.putNumber(
                  "Target Heading", -rawSetpointPose.getRotation().getDegrees());

              double dt = deltaTimeTimer.get();
              Logger.recordOutput("TagFollowing/DeltaTime", dt);
              deltaTimeTimer.restart();

              VisionDemoResult setpointPose = mode.calculate(robotPose, tagPose, dt, box);
              saftyMode = setpointPose.saftyMode();

              mode.getSuperstructureState(robotPose, tagPose).ifPresent(this::updateSuperstructure);

              setpointPose
                  .setpointPose()
                  .ifPresent(
                      pose -> {
                        Pose2d clampedPose = box.clamp(pose);
                        controller.setSetpoint(clampedPose);

                        setpointStablityCheckingFilter.calculate(
                            clampedPose, Timer.getFPGATimestamp());

                        Logger.recordOutput("TagFollowing/Box/RobotUnclampedSetpointPose", pose);
                        Logger.recordOutput("TagFollowing/RobotSetpointPose", clampedPose);
                        Logger.recordOutput(
                            "TagFollowing/Box/SetpointPoseInBox", box.contains(pose));
                      });
            });

    Logger.recordOutput("TagFollowing/Box/CurrentPoseInBox", box.contains(robotPose));

    ChassisSpeeds speeds = getChassisSpeeds();
    drive.setRobotSpeeds(speeds);
  }

  private void updateLEDS(boolean isUpdating) {

    ResultSaftyMode saftyMode = this.saftyMode;

    if (saftyMode == ResultSaftyMode.UNKNOWN) {
      saftyMode = mode.getExspectedSaftyMode();
    }

    BlinkenLEDPattern pattern =
        switch (saftyMode) {
          case FULL_CONTROL -> isUpdating ? BlinkenLEDPattern.CHASE_RED : BlinkenLEDPattern.ORANGE;
          case ROTATIONAL_CONTROL -> isUpdating
              ? BlinkenLEDPattern.CHASE_BLUE
              : BlinkenLEDPattern.GREEN;
          case STOP -> BlinkenLEDPattern.WHITE;
          case UNKNOWN -> BlinkenLEDPattern.RED;
        };

    Logger.recordOutput("TagFollowing/LEDPattern", pattern.toString());

    leds.set(pattern);
  }

  private void updateSuperstructure(SuperstructureState state) {
    double filteredElevatorHeight = elevatorHeightFilter.calculate(state.elevatorHeight());
    Rotation2d filteredWristAngle = wristAngleFilter.calculate(state.wristAngle());

    elevator.setGoalHeightMeters(filteredElevatorHeight);
    wrist.setGoalRotation(filteredWristAngle);

    Logger.recordOutput(
        "TagFollowing/Superstructure/DesiredElevatorHeight", filteredElevatorHeight);
    Logger.recordOutput(
        "TagFollowing/Superstructure/DesiredWristAngle", filteredWristAngle.getDegrees());
  }

  private ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds speeds = controller.calculate(drive.getRobotPose());

    boolean atReference = controller.atReference();
    boolean atReferenceStable = atGoalDebouncer.calculate(atReference);

    Logger.recordOutput("TagFollowing/AtReference", atReference);
    Logger.recordOutput("TagFollowing/AtReferenceStable", atReferenceStable);

    boolean stopped = false;

    if (atReferenceStable) {
      speeds = new ChassisSpeeds(0, 0, 0);
    }

    switch (saftyMode) {
      case FULL_CONTROL:
        // Do nothing, full control
        break;
      case ROTATIONAL_CONTROL:
        speeds.vxMetersPerSecond = 0;
        speeds.vyMetersPerSecond = 0;
        break;
      case STOP, UNKNOWN:
        speeds = new ChassisSpeeds(0, 0, 0);
        stopped = true;
        break;
    }

    ChassisSpeeds slewSpeeds =
        new ChassisSpeeds(
            vxLimiter.calculate(speeds.vxMetersPerSecond),
            vyLimiter.calculate(speeds.vyMetersPerSecond),
            omegaLimiter.calculate(speeds.omegaRadiansPerSecond));

    Logger.recordOutput("TagFollowing/ChassisSpeeds/Unslewed", speeds);
    Logger.recordOutput("TagFollowing/ChassisSpeeds/Slewed", slewSpeeds);

    boolean isSafeToSlew = !stopped && box.contains(drive.getRobotPose());

    boolean stableUnslewed =
        setpointStablityCheckingFilter.lastValue() == null
            || setpointStablityCheckingFilter
                    .calculateMean()
                    .getTranslation()
                    .getDistance(setpointStablityCheckingFilter.lastValue().getTranslation())
                < Units.inchesToMeters(6);

    Logger.recordOutput("TagFollowing/ChassisSpeeds/IsSafeToSlew", isSafeToSlew);
    Logger.recordOutput("TagFollowing/ChassisSpeeds/StableUnslewed", stableUnslewed);

    if (!isSafeToSlew || stableUnslewed || !DO_SLEW) {
      return speeds;
    }

    return slewSpeeds;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Has Track Target", false);
    SmartDashboard.putNumber("Target Heading", Double.NaN);
    drive.stop();
  }
}
