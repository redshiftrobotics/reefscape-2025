package frc.robot.commands.visionDemo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.commands.visionDemo.VisionDemoCommand.VisionDemoState;
import org.littletonrobotics.junction.Logger;

public class FollowTagMode implements VisionDemoState {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private static final double FILTER_SLEW_RATE = Units.feetToMeters(5);
  private static final double MAX_TAG_JUMP = Units.feetToMeters(1);

  private final Translation2d targetOffset;

  private final int tagId;
  private Translation3d tagFilteredPosition = Translation3d.kZero;

  private boolean hasValidTag = false;

  public FollowTagMode(int tag, Translation2d targetOffset) {
    this.tagId = tag;
    this.targetOffset = targetOffset;
  }

  @Override
  public Pose2d updateSetpoint(Pose2d robotPose, Pose3d tagPose) {

    tagFilteredPosition =
        MathUtil.slewRateLimit(
            tagFilteredPosition,
            tagPose.getTranslation(),
            Constants.LOOP_PERIOD_SECONDS,
            FILTER_SLEW_RATE);

    Rotation2d targetRotation =
        tagPose.getRotation().toRotation2d().plus(Rotation2d.k180deg).plus(TARGET_HEADING_OFFSET);

    Translation2d targetTranslation =
        tagPose.toPose2d().plus(new Transform2d(targetOffset, Rotation2d.kZero)).getTranslation();

    Pose2d target = new Pose2d(targetTranslation, targetRotation);

    double distance = tagFilteredPosition.getDistance(tagPose.getTranslation());
    boolean withinMaxJump = distance < MAX_TAG_JUMP;

    String keyPrefix = "TagFollowing/Follow" + tagId + "/";

    Logger.recordOutput(keyPrefix + "TagDistanceToSafe", distance);
    Logger.recordOutput(keyPrefix + "WithinMaxJump", withinMaxJump);
    Logger.recordOutput(
        keyPrefix + "TagFilteredPosition",
        new Pose3d(this.tagFilteredPosition, tagPose.getRotation()));
    Logger.recordOutput(keyPrefix + "RawTargetPosition", target);

    if (!withinMaxJump) {
      return null;
    }

    return target;
  }

  @Override
  public String name() {
    return "FOLLOW TAG " + tagId;
  }

  @Override
  public int tagId() {
    return tagId;
  }

  @Override
  public int priority() {
    return 2;
  }
}
