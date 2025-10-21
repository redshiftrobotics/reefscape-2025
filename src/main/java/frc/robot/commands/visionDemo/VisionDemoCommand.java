package frc.robot.commands.visionDemo;

import edu.wpi.first.math.filter.Debouncer;
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
import frc.robot.commands.visionDemo.filters.ComboFilter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.BlinkenLEDPattern;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.TrackedTarget;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class VisionDemoCommand extends Command {

  public interface VisionDemoMode {

    public default void reset() {}

    public ResultSaftyMode getExspectedSaftyMode();

    public Pose2d getRawPose(Pose2d robotPose, Pose3d tagPose);

    public VisionDemoResult calculate(Pose2d robotPose, Pose3d tagPose, double dt);

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
  private final AprilTagVision vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final LEDSubsystem leds;

  private final VisionDemoMode mode;
  private final int tagId;

  private final SimpleDriveController controller = new SimpleDriveController();
  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  private final ComboFilter elevatorHeightFilter = new ComboFilter(3, 5);
  private final ComboFilter wristAngleFilter = new ComboFilter(3, 5);

  private ResultSaftyMode saftyMode = ResultSaftyMode.STOP;

  private final Debouncer hasTargetDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kFalling);

  private final Timer deltaTimeTimer = new Timer();

  public VisionDemoCommand(
      AprilTagVision vision,
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      LEDSubsystem leds,
      VisionDemoMode mode,
      int tagId) {
    this.vision = vision;
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.leds = leds;
    this.mode = mode;
    this.tagId = tagId;

    controller.setTolerance(
        new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(3)));

    addRequirements(drive, elevator, wrist, leds);
  }

  @Override
  public void initialize() {
    controller.reset(drive.getRobotPose());
    controller.setSetpoint(drive.getRobotPose());
    mode.reset();
    deltaTimeTimer.restart();
    saftyMode = ResultSaftyMode.UNKNOWN;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getRobotPose();

    // --- Tag Collection and Filtering ---

    List<TrackedTarget> tags = getTag(tagId);

    boolean hasTags = !tags.isEmpty();
    boolean hasTagsDebounced = hasTargetDebouncer.calculate(hasTags);

    Logger.recordOutput("TagFollowing/hasTags", hasTags);
    SmartDashboard.putBoolean("Sees Tags?", hasTagsDebounced);

    if (!tags.isEmpty() || !hasTagsDebounced) {
      Logger.recordOutput(
          "TagFollowing/Tags/Pose3d",
          tags.stream().map(t -> t.getTargetPose(robotPose)).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "TagFollowing/Tags/CameraPoses3d",
          tags.stream().map(t -> t.getCamearaPose(robotPose)).toArray(Pose3d[]::new));
    }

    // --- Target Logging ---

    Logger.recordOutput("TagFollowing/Result/SaftyMode", saftyMode.toString());
    updateLEDS(hasTagsDebounced);

    if (hasTags) {
      Pose3d averageTagPose =
          TagFollowUtil.averagePoses(tags.stream().map(t -> t.getTargetPose(robotPose)).toList());

      Logger.recordOutput("TagFollowing/Target/Pose3d", averageTagPose);
      Logger.recordOutput(
          "TagFollowing/Target/CameraPoses3d",
          tags.stream().map(t -> t.getCamearaPose(robotPose)).toArray(Pose3d[]::new));

      Pose2d rawSetpointPose = mode.getRawPose(robotPose, averageTagPose);
      Logger.recordOutput("TagFollowing/RobotRawSetpointPose", rawSetpointPose);
      SmartDashboard.putNumber("Target Heading", -rawSetpointPose.getRotation().getDegrees());

      double dt = deltaTimeTimer.get();
      Logger.recordOutput("TagFollowing/DeltaTime", dt);
      deltaTimeTimer.restart();

      VisionDemoResult setpointPose = mode.calculate(robotPose, averageTagPose, dt);
      saftyMode = setpointPose.saftyMode();

      mode.getSuperstructureState(robotPose, averageTagPose).ifPresent(this::updateSuperstructure);

      setpointPose
          .setpointPose()
          .ifPresent(
              pose -> {
                controller.setSetpoint(pose);
                Logger.recordOutput("TagFollowing/RobotSetpointPose", pose);
              });
    }

    ChassisSpeeds speeds = getChassisSpeeds();
    drive.setRobotSpeeds(speeds);
  }

  private List<TrackedTarget> getTag(int tagId) {
    return vision.getLatestTargets().stream()
        .filter(TrackedTarget::isGoodPoseAmbiguity)
        .filter(
            tag -> Math.abs(tag.cameraToTarget().getRotation().getY()) < Units.degreesToRadians(80))
        .filter(tag -> tag.id() == tagId)
        .toList();
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
    Rotation2d filteredWristAngle =
        new Rotation2d(wristAngleFilter.calculate(state.wristAngle().getRadians()));

    elevator.setGoalHeightMeters(filteredElevatorHeight);
    wrist.setGoalRotation(filteredWristAngle);

    Logger.recordOutput(
        "TagFollowing/Superstructure/DesiredElevatorHeight", filteredElevatorHeight);
    Logger.recordOutput(
        "TagFollowing/Superstructure/DesiredWristAngle", filteredWristAngle.getDegrees());
  }

  private ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds speeds = controller.calculate(drive.getRobotPose());

    boolean atGoal = atGoalDebouncer.calculate(controller.atReference());
    Logger.recordOutput("TagFollowing/AtGoal", atGoal);
    if (atGoal) {
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
        break;
    }

    return speeds;
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
