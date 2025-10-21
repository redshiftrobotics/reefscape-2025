package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ContainmentBox {
  private final Translation2d bottomLeft;
  private final Translation2d topRight;

  public ContainmentBox(Translation2d bottomLeft, Translation2d topRight) {
    this.bottomLeft = bottomLeft;
    this.topRight = topRight;
  }

  public Translation2d clamp(Translation2d point) {
    double clampedX = Math.max(bottomLeft.getX(), Math.min(topRight.getX(), point.getX()));
    double clampedY = Math.max(bottomLeft.getY(), Math.min(topRight.getY(), point.getY()));
    return new Translation2d(clampedX, clampedY);
  }

  public Pose2d clamp(Pose2d pose) {
    Translation2d clampedTranslation = clamp(pose.getTranslation());
    return new Pose2d(clampedTranslation, pose.getRotation());
  }

  public boolean contains(Translation2d point) {
    return point.getX() >= bottomLeft.getX()
        && point.getX() <= topRight.getX()
        && point.getY() >= bottomLeft.getY()
        && point.getY() <= topRight.getY();
  }

  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  public Pose2d[] getCorners() {
    return new Pose2d[] {
      new Pose2d(bottomLeft, Rotation2d.kZero),
      new Pose2d(new Translation2d(topRight.getX(), bottomLeft.getY()), Rotation2d.kZero),
      new Pose2d(topRight, Rotation2d.kZero),
      new Pose2d(new Translation2d(bottomLeft.getX(), topRight.getY()), Rotation2d.kZero),
      new Pose2d(bottomLeft, Rotation2d.kZero),
    };
  }
}
