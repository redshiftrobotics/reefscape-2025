package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.visionDemo.filters.ComboFilter;

public class AimAtTagMode implements VisionDemoCommand.VisionDemoState {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private final ComboFilter angleFilter = new ComboFilter(3, 5);

  @Override
  public Pose2d getRawPose(Pose2d robotPose, Pose3d tagPose) {
    Rotation2d angle =
        tagPose
            .toPose2d()
            .getTranslation()
            .minus(robotPose.getTranslation())
            .getAngle()
            .plus(TARGET_HEADING_OFFSET);

    return new Pose2d(robotPose.getTranslation(), angle);
  }

  @Override
  public Pose2d getSafePose(Pose2d robotPose, Pose3d tagPose) {
    Pose2d rawPose = getRawPose(robotPose, tagPose);
    Rotation2d filteredAngle =
        new Rotation2d(angleFilter.calculate(rawPose.getRotation().getRadians()));

    return new Pose2d(rawPose.getTranslation(), filteredAngle);
  }

  @Override
  public boolean blocksDriving() {
    return true;
  }
}
