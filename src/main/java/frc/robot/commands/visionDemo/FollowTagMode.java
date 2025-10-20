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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.visionDemo.VisionDemoCommand.VisionDemoState;
import org.littletonrobotics.junction.Logger;

public class FollowTagMode implements VisionDemoState {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private static final double FILTER_SLEW_RATE = Units.feetToMeters(4);
  private static final double MAX_TAG_JUMP = Units.feetToMeters(1);

  private final Translation2d targetOffset;

  private Translation3d tagFilteredPosition;

  private final Timer resetTimer = new Timer();

  private boolean shouldDrive = false;
  private final Debouncer shouldDriveDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);

  public FollowTagMode(Translation2d targetOffset) {
    this.targetOffset = targetOffset;
  }

  @Override
  public void reset() {
    tagFilteredPosition = null;
    resetTimer.reset();

    shouldDrive = false;
    shouldDriveDebouncer.calculate(false);
  }

  @Override
  public Pose2d updateSetpoint(Pose2d robotPose, Pose3d tagPose) {

    if (tagFilteredPosition == null) {
      tagFilteredPosition = tagPose.getTranslation();
    }

    Logger.recordOutput("TagFollowing/Follow/DeltaTime", resetTimer.get());

    tagFilteredPosition =
        MathUtil.slewRateLimit(
            tagFilteredPosition, tagPose.getTranslation(), resetTimer.get(), FILTER_SLEW_RATE);

    resetTimer.restart();

    Rotation2d targetRotation =
        tagPose.getRotation().toRotation2d().plus(Rotation2d.k180deg).plus(TARGET_HEADING_OFFSET);

    Translation2d targetTranslation =
        tagPose.toPose2d().plus(new Transform2d(targetOffset, Rotation2d.kZero)).getTranslation();

    Pose2d target = new Pose2d(targetTranslation, targetRotation);

    double distance = tagFilteredPosition.getDistance(tagPose.getTranslation());
    boolean withinMaxJump = distance < MAX_TAG_JUMP;

    Logger.recordOutput("TagFollowing/Follow/TagDistanceToSafe", distance);
    Logger.recordOutput("TagFollowing/Follow/WithinMaxJump", withinMaxJump);
    Logger.recordOutput(
        "TagFollowing/Follow/TagFilteredPosition",
        new Pose3d(this.tagFilteredPosition, tagPose.getRotation()));
    Logger.recordOutput("TagFollowing/Follow/RawTargetPosition", target);

    shouldDrive = shouldDriveDebouncer.calculate(withinMaxJump);
    Logger.recordOutput("TagFollowing/Follow/ShouldDrive", shouldDrive);

    if (!withinMaxJump) {
      return null;
    }

    return target;
  }

  @Override
  public boolean blocksDriving() {
    return !shouldDrive;
  }
}
