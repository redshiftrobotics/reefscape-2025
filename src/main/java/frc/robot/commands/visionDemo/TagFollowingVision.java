package frc.robot.commands.visionDemo;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.AbsoluteTrackedTarget;
import frc.robot.utility.VirtualSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class TagFollowingVision extends VirtualSubsystem {

  public static BooleanSupplier debugModeSupplier = () -> false;

  private static final double MAX_POSE_AMBIGUITY = 0.2;
  private static final double MAX_PITCH_ANGLE = Units.degreesToRadians(70);

  private final AprilTagVision vision;
  private final int tagId;

  private final Debouncer hasTargetDebouncer = new Debouncer(0.3);
  private boolean isTracking = false;

  private Optional<Pose3d> lastTagPose = Optional.empty();

  public TagFollowingVision(AprilTagVision vision, int tagId) {
    this.vision = vision;
    this.tagId = tagId;
  }

  @Override
  public void periodic() {
    List<AbsoluteTrackedTarget> tags =
        vision.getAbsoluteTrackedTargets().stream().filter(tag -> tag.id() == tagId).toList();

    if (!tags.isEmpty()) {
      Logger.recordOutput(
          "TagTracking/Tags/MaxPoseAmbiguity",
          tags.stream().mapToDouble(AbsoluteTrackedTarget::poseAmbiguity).max().orElse(-1));
    }

    List<AbsoluteTrackedTarget> filteredTags =
        tags.stream()
            .filter(tag -> tag.poseAmbiguity() < MAX_POSE_AMBIGUITY)
            .filter(tag -> tag.targetPose().getTranslation().getZ() > 0) // Below floor
            .filter(
                tag -> isPointingAtPose(tag.robotPose(), tag.targetPose().toPose2d(), Math.PI / 2))
            .filter(tag -> Math.abs(tag.targetPose().getRotation().getY()) < MAX_PITCH_ANGLE)
            .toList();

    Logger.recordOutput("TagTracking/TotalDetected", tags.size());
    Logger.recordOutput("TagTracking/TotalFiltered", filteredTags.size());

    boolean noTargets = filteredTags.isEmpty();
    boolean noTargetsStable = hasTargetDebouncer.calculate(noTargets);

    this.isTracking = !noTargetsStable;
    SmartDashboard.putBoolean("Is Tracking Tag?", isTracking);

    Logger.recordOutput("TagTracking/noTargets", noTargets);
    Logger.recordOutput("TagTracking/noTargetsStable", noTargetsStable);

    if (!noTargets || noTargetsStable) {
      Logger.recordOutput(
          "TagTracking/Tags/Pose3d",
          filteredTags.stream().map(AbsoluteTrackedTarget::targetPose).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "TagTracking/Tags/CameraPoses3d",
          filteredTags.stream().map(AbsoluteTrackedTarget::cameraPose).toArray(Pose3d[]::new));
    }

    if (!noTargets) {
      Pose3d averageTagPose =
          meanPose(filteredTags.stream().map(AbsoluteTrackedTarget::targetPose).toList());
      Logger.recordOutput("TagTracking/Target/Pose3d", averageTagPose);
      lastTagPose = Optional.of(averageTagPose);
    } else {
      lastTagPose = Optional.empty();
    }
  }

  public boolean isTracking() {
    return isTracking;
  }

  public Optional<Pose3d> getLastTagPose() {
    return lastTagPose;
  }

  private static boolean isPointingAtPose(Pose2d robotPose, Pose2d targetPose, double maxAngle) {
    Rotation2d angleToFaceTag =
        robotPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Rotation2d angleOfTag = targetPose.getRotation();
    double angleDiff = angleToFaceTag.minus(angleOfTag).getRadians();
    return Math.abs(angleDiff) < maxAngle;
  }

  private static Pose3d meanPose(List<Pose3d> poses) {
    if (poses.isEmpty()) {
      throw new IllegalArgumentException("Cannot average an empty list of poses");
    }

    if (poses.size() == 1) {
      return poses.get(0);
    }

    Translation3d summedTranslations = new Translation3d(0, 0, 0);
    Quaternion summmedQuaternion = new Quaternion(0, 0, 0, 0);

    Quaternion reference = poses.get(0).getRotation().getQuaternion();

    for (Pose3d pose : poses) {
      summedTranslations = summedTranslations.plus(pose.getTranslation());

      Quaternion q = pose.getRotation().getQuaternion();

      if (q.dot(reference) < 0) {
        q = q.times(-1);
      }

      summmedQuaternion = summmedQuaternion.plus(q);
    }

    Translation3d avgTranslations = summedTranslations.div(poses.size());
    Quaternion avgQuaternion = summmedQuaternion.divide(poses.size()).normalize();

    return new Pose3d(avgTranslations, new Rotation3d(avgQuaternion));
  }
}
