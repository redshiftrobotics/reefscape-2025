package frc.robot.commands.visionDemo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.visionDemo.SuperstructureUtil.SuperstructureState;
import frc.robot.commands.visionDemo.VisionDemoCommand.ResultSaftyMode;
import frc.robot.commands.visionDemo.VisionDemoCommand.VisionDemoMode;
import frc.robot.commands.visionDemo.VisionDemoCommand.VisionDemoResult;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class FollowTagMode implements VisionDemoMode {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private static final double FILTER_SLEW_RATE = Units.feetToMeters(6);

  private static final double MAX_TAG_JUMP = Units.feetToMeters(1);
  private static final double MAX_TAG_JUMP_NEW_POSITION = Units.feetToMeters(3);

  private static final Debouncer timeTillActive = new Debouncer(0.5);

  private final Translation2d targetOffset;

  private Translation3d tagFilteredPosition;

  private BooleanSupplier followWithSuperstructure;

  public FollowTagMode(Translation2d targetOffset, BooleanSupplier followWithSuperstructure) {
    this.followWithSuperstructure = followWithSuperstructure;
    this.targetOffset = targetOffset;
  }

  @Override
  public void reset() {
    tagFilteredPosition = null;
    timeTillActive.calculate(false);
  }

  @Override
  public ResultSaftyMode getExspectedSaftyMode() {
    return ResultSaftyMode.FULL_CONTROL;
  }

  @Override
  public Pose2d getRawPose(Pose2d robotPose, Pose3d tagPose) {
    Rotation2d targetRotation =
        tagPose.getRotation().toRotation2d().plus(Rotation2d.k180deg).plus(TARGET_HEADING_OFFSET);

    Translation2d targetTranslation =
        tagPose.toPose2d().plus(new Transform2d(targetOffset, Rotation2d.kZero)).getTranslation();

    return new Pose2d(targetTranslation, targetRotation);
  }

  @Override
  public VisionDemoResult calculate(Pose2d robotPose, Pose3d tagPose, double dt) {

    boolean isNewPosition =
        tagFilteredPosition == null
            || tagFilteredPosition.getDistance(tagPose.getTranslation())
                > MAX_TAG_JUMP_NEW_POSITION;
    boolean isActive = timeTillActive.calculate(!isNewPosition);

    Logger.recordOutput("TagFollowing/Follow/IsNewPosition", isNewPosition);
    Logger.recordOutput("TagFollowing/Follow/IsActive", isActive);

    if (tagFilteredPosition == null) {
      tagFilteredPosition = tagPose.getTranslation();
    }

    tagFilteredPosition =
        MathUtil.slewRateLimit(tagFilteredPosition, tagPose.getTranslation(), dt, FILTER_SLEW_RATE);

    if (!isActive) {
      return new VisionDemoResult(Optional.empty(), ResultSaftyMode.UNKNOWN);
    }

    Pose2d target = getRawPose(robotPose, tagPose);

    double distance = tagFilteredPosition.getDistance(tagPose.getTranslation());
    boolean withinMaxJump = distance < MAX_TAG_JUMP;

    Logger.recordOutput("TagFollowing/Follow/TagDistanceToSafe", distance);
    Logger.recordOutput("TagFollowing/Follow/WithinMaxJump", withinMaxJump);
    Logger.recordOutput(
        "TagFollowing/Follow/TagFilteredPosition",
        new Pose3d(this.tagFilteredPosition, tagPose.getRotation()));

    if (!withinMaxJump) {
      return new VisionDemoResult(Optional.empty(), ResultSaftyMode.FULL_CONTROL);
    }

    return new VisionDemoResult(Optional.of(target), ResultSaftyMode.FULL_CONTROL);
  }

  @Override
  public Optional<SuperstructureState> getSuperstructureState(Pose2d robotPose, Pose3d tagPose3d) {
    if (!followWithSuperstructure.getAsBoolean()) {
      return Optional.empty();
    }
    return Optional.of(SuperstructureUtil.pointAtTag(tagPose3d));
  }
}
