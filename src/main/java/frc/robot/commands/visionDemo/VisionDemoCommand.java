package frc.robot.commands.visionDemo;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.controllers.SimpleDriveController;
import frc.robot.commands.visionDemo.TagFollowUtil.TagFollowingSuperstructureState;
import frc.robot.commands.visionDemo.filters.ComboFilter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.TrackedTarget;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class VisionDemoCommand extends Command {

  public interface VisionDemoState extends Comparable<VisionDemoState> {

    public Pose2d updateSetpoint(Pose2d robotPose, Pose3d tagPose);

    public default ChassisSpeeds updateSpeeds(ChassisSpeeds currentSpeeds) {
      return currentSpeeds;
    }

    public String name();

    public int tagId();

    public default int priority() {
      return 0;
    }

    public default boolean usesSuperstructure() {
      return false;
    }

    @Override
    default int compareTo(VisionDemoState other) {
      return Integer.compare(this.priority(), other.priority());
    }
  }

  private final Drive drive;
  private final AprilTagVision vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final LEDSubsystem leds;

  private final List<VisionDemoState> tagToFollow;
  private final VisionDemoState passiveMode;

  private final SimpleDriveController controller = new SimpleDriveController();
  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  private VisionDemoState currentMode;

  private final ComboFilter elevatorHeightFilter = new ComboFilter(3, 5);
  private final ComboFilter wristAngleFilter = new ComboFilter(3, 5);

  private final Debouncer hasTargetDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

  private final Debouncer modeSwitchDebouncer = new Debouncer(1);

  public VisionDemoCommand(
      AprilTagVision vision,
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      LEDSubsystem leds,
      List<VisionDemoState> tags,
      VisionDemoState passiveMode) {
    this.vision = vision;
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.leds = leds;
    this.tagToFollow = tags;
    this.passiveMode = passiveMode;

    addRequirements(drive, elevator, wrist, leds);
  }

  @Override
  public void initialize() {
    this.currentMode = passiveMode;
    controller.reset(drive.getRobotPose());
    controller.setSetpoint(drive.getRobotPose());
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getRobotPose();

    // --- Tag Collection and Filtering ---

    List<TrackedTarget> tags = vision.getLatestTargets(); // 36h11

    Map<Integer, VisionDemoState> tagMap =
        tagToFollow.stream().collect(Collectors.toMap(VisionDemoState::tagId, t -> t));

    List<TrackedTarget> filtedTags =
        tags.stream()
            .filter(TrackedTarget::isGoodPoseAmbiguity)
            .filter(
                t -> Math.abs(t.cameraToTarget().getRotation().getY()) < Units.degreesToRadians(80))
            .filter(t -> tagMap.containsKey(t.id()))
            .toList();

    boolean hasTags = !filtedTags.isEmpty();
    boolean hasTagsDebounced = hasTargetDebouncer.calculate(hasTags);

    Logger.recordOutput("TagFollowing/hasTags", hasTags);
    SmartDashboard.putBoolean("Sees Tags?", hasTagsDebounced);

    if (!filtedTags.isEmpty() || !hasTagsDebounced) {
      Logger.recordOutput(
          "TagFollowing/Tags/IDs", filtedTags.stream().mapToInt(TrackedTarget::id).toArray());
      Logger.recordOutput(
          "TagFollowing/Tags/Pose3ds",
          filtedTags.stream().map(t -> t.getTargetPose(robotPose)).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "TagFollowing/Tags/CameraPose3ds",
          filtedTags.stream().map(t -> t.getCamearaPose(robotPose)).toArray(Pose3d[]::new));
    }

    // --- Mode Selection ---

    VisionDemoState desiredMode =
        filtedTags.stream()
            .map(t -> tagMap.get(t.id()))
            .max(VisionDemoState::compareTo)
            .orElse(passiveMode);

    if (desiredMode.priority() > currentMode.priority()
        || modeSwitchDebouncer.calculate(desiredMode != currentMode)) {
      currentMode = desiredMode;
    }

    List<TrackedTarget> usedTags =
        filtedTags.stream().filter(t -> t.id() == currentMode.tagId()).toList();

    Logger.recordOutput("TagFollowing/DesiredMode", desiredMode.name());
    Logger.recordOutput("TagFollowing/CurrentMode", currentMode.name());

    SmartDashboard.putString("Tag Following Mode", currentMode.name());

    // --- Target Logging ---

    if (!usedTags.isEmpty()) {
      Pose3d averageTagPose =
          TagFollowUtil.averagePoses(
              usedTags.stream().map(t -> t.getTargetPose(robotPose)).toList());

      Logger.recordOutput("TagFollowing/Target/ID", currentMode.tagId());
      Logger.recordOutput("TagFollowing/Target/Pose3d", new Pose3d[] {averageTagPose});
      Logger.recordOutput(
          "TagFollowing/Target/CameraPoses3d",
          filtedTags.stream().map(t -> t.getCamearaPose(robotPose)).toArray(Pose3d[]::new));

      Pose2d setpointPose = currentMode.updateSetpoint(robotPose, averageTagPose);

      if (setpointPose != null) {
        controller.setSetpoint(setpointPose);
        Logger.recordOutput(
            "TagFollowing/RobotSetpointPose", new Pose3d[] {new Pose3d(setpointPose)});
        SmartDashboard.putNumber("Target Heading", -setpointPose.getRotation().getDegrees());
      }

      if (currentMode.usesSuperstructure()) {
        TagFollowingSuperstructureState desiredState = getSuperstructureState(averageTagPose);

        double filteredElevatorHeight =
            elevatorHeightFilter.calculate(desiredState.elevatorHeight());
        Rotation2d filteredWristAngle =
            new Rotation2d(wristAngleFilter.calculate(desiredState.wristAngle().getRadians()));

        elevator.setGoalHeightMeters(filteredElevatorHeight);
        wrist.setGoalRotation(filteredWristAngle);

        Logger.recordOutput(
            "TagFollowing/Superstructure/DesiredElevatorHeight", filteredElevatorHeight);
        Logger.recordOutput(
            "TagFollowing/Superstructure/DesiredWristAngle", filteredWristAngle.getDegrees());
      }
    }

    // --- Chassis Speed Calculation ---

    ChassisSpeeds speeds = controller.calculate(drive.getRobotPose());

    if (atGoalDebouncer.calculate(controller.atReference())) {
      speeds = new ChassisSpeeds(0, 0, 0);
    }

    speeds = currentMode.updateSpeeds(speeds);

    drive.setRobotSpeeds(speeds);
  }

  private static TagFollowingSuperstructureState getSuperstructureState(Pose3d tagPose) {
    final Rotation2d wristAngle = new Rotation2d(tagPose.getRotation().getY());
    final double elevatorHeight =
        tagPose.getTranslation().getZ()
            - TagFollowUtil.WRIST_LENGTH * wristAngle.getSin()
            - TagFollowUtil.ELEVATOR_HEIGHT_OFF_GROUND;
    return new TagFollowingSuperstructureState(elevatorHeight, wristAngle);
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
