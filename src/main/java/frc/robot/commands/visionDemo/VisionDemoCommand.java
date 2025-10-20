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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class VisionDemoCommand extends Command {

  public interface VisionDemoState {

    public default void reset() {}

    public Pose2d getRawPose(Pose2d robotPose, Pose3d tagPose);
    public Pose2d getSafePose(Pose2d robotPose, Pose3d tagPose);

    public default boolean blocksDriving() {
      return false;
    }
  }

  private final Drive drive;
  private final AprilTagVision vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final LEDSubsystem leds;

  private final VisionDemoState mode;
  private final BooleanSupplier useSuperstructure;
  private final int tagId;

  private final SimpleDriveController controller = new SimpleDriveController();
  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  private final ComboFilter elevatorHeightFilter = new ComboFilter(3, 5);
  private final ComboFilter wristAngleFilter = new ComboFilter(3, 5);

  private final Debouncer hasTargetDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

  public VisionDemoCommand(
      AprilTagVision vision,
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      LEDSubsystem leds,
      VisionDemoState mode,
      BooleanSupplier useSuperstructure,
      int tagId) {
    this.vision = vision;
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.leds = leds;
    this.mode = mode;
    this.useSuperstructure = useSuperstructure;
    this.tagId = tagId;

    controller.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(3)));

    addRequirements(drive, elevator, wrist, leds);
  }

  @Override
  public void initialize() {
    controller.reset(drive.getRobotPose());
    controller.setSetpoint(drive.getRobotPose());
    mode.reset();
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getRobotPose();

    // --- Tag Collection and Filtering ---

    List<TrackedTarget> tags = vision.getLatestTargets();

    List<TrackedTarget> filtedTags =
        tags.stream()
            .filter(TrackedTarget::isGoodPoseAmbiguity)
            .filter(
                tag ->
                    Math.abs(tag.cameraToTarget().getRotation().getY())
                        < Units.degreesToRadians(80))
            .filter(tag -> tag.id() == tagId)
            .toList();

    boolean hasTags = !filtedTags.isEmpty();
    boolean hasTagsDebounced = hasTargetDebouncer.calculate(hasTags);

    Logger.recordOutput("TagFollowing/hasTags", hasTags);
    SmartDashboard.putBoolean("Sees Tags?", hasTagsDebounced);

    if (!filtedTags.isEmpty() || !hasTagsDebounced) {
      Logger.recordOutput(
          "TagFollowing/Tags/Number", filtedTags.stream().mapToInt(TrackedTarget::id).toArray());
      Logger.recordOutput(
          "TagFollowing/Tags/Pose3ds",
          filtedTags.stream().map(t -> t.getTargetPose(robotPose)).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "TagFollowing/Tags/CameraPose3ds",
          filtedTags.stream().map(t -> t.getCamearaPose(robotPose)).toArray(Pose3d[]::new));
    }

    // --- Target Logging ---

    if (!filtedTags.isEmpty()) {
      Pose3d averageTagPose =
          TagFollowUtil.averagePoses(
              filtedTags.stream().map(t -> t.getTargetPose(robotPose)).toList());

      Logger.recordOutput("TagFollowing/Target/Pose3d", averageTagPose);
      Logger.recordOutput(
          "TagFollowing/Target/CameraPoses3d",
          filtedTags.stream().map(t -> t.getCamearaPose(robotPose)).toArray(Pose3d[]::new));


      Pose2d rawSetpointPose = mode.getRawPose(robotPose, averageTagPose);
      Logger.recordOutput("TagFollowing/RobotRawSetpointPose", rawSetpointPose);
      SmartDashboard.putNumber("Target Heading", -rawSetpointPose.getRotation().getDegrees());

      Pose2d setpointPose = mode.getSafePose(robotPose, averageTagPose);

      if (setpointPose != null) {
        controller.setSetpoint(setpointPose);
        Logger.recordOutput(
            "TagFollowing/RobotSetpointPose", setpointPose);
      }

      if (useSuperstructure.getAsBoolean()) {
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

    if (mode.blocksDriving()) {
      speeds.vxMetersPerSecond = 0;
      speeds.vyMetersPerSecond = 0;
    }

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
