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

    public Pose2d getRawPose(Pose2d robotPose, Pose3d tagPose);

    public VisionDemoResult calculate(Pose2d robotPose, Pose3d tagPose, double dt);

    public Optional<SuperstructureState> getSuperstructureState(Pose2d robotPose, Pose3d tagPose3d);

    public Optional<BlinkenLEDPattern> getLEDPattern(Pose2d robotPose, Pose3d tagPose3d);
  }

  public record VisionDemoResult(
    Optional<Pose2d> setpointPose, ResultSaftyMode saftyMode
  ) {}

  public enum ResultSaftyMode {
    SAFE, BLOCK_DRIVING, STOP;
  }

  private final Drive drive;
  private final VisionSystem vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final LEDSubsystem leds;

  private final VisionDemoMode mode;
  private final int tagId;

  private final SimpleDriveController controller = new SimpleDriveController();
  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  private final ComboFilter elevatorHeightFilter = new ComboFilter(3, 5);
  private final ComboFilter wristAngleFilter = new ComboFilter(3, 5);

  private final Debouncer hasTargetDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

  private final Timer deltaTimeTimer = new Timer();

  public VisionDemoCommand(
      AprilTagVision vision,
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      LEDSubsystem leds,
      VisionDemoMode mode,
      BooleanSupplier useSuperstructure,
      int tagId) {
    this.vision = new VisionSystem(vision);
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
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getRobotPose();

    // --- Tag Collection and Filtering ---

    List<TrackedTarget> tags = vision.getTag(tagId);

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

    if (!tags.isEmpty()) {
      Pose3d averageTagPose =
          TagFollowUtil.averagePoses(
              tags.stream().map(t -> t.getTargetPose(robotPose)).toList());

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

      mode.getSuperstructureState(robotPose, averageTagPose).ifPresent(this::updateSuperstructure);
      mode.getLEDPattern(robotPose, averageTagPose).ifPresent(leds::set);

      setpointPose.setpointPose().ifPresent(pose -> {
        controller.setSetpoint(pose);
        Logger.recordOutput("TagFollowing/RobotSetpointPose", pose);
      });
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

  private void updateSuperstructure(SuperstructureState state) {
      double filteredElevatorHeight =
      elevatorHeightFilter.calculate(state.elevatorHeight());
  Rotation2d filteredWristAngle =
      new Rotation2d(wristAngleFilter.calculate(state.wristAngle().getRadians()));

  elevator.setGoalHeightMeters(filteredElevatorHeight);
  wrist.setGoalRotation(filteredWristAngle);

  Logger.recordOutput(
      "TagFollowing/Superstructure/DesiredElevatorHeight", filteredElevatorHeight);
  Logger.recordOutput(
      "TagFollowing/Superstructure/DesiredWristAngle", filteredWristAngle.getDegrees());
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
