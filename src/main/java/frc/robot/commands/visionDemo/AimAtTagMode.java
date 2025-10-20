package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.visionDemo.filters.ComboFilter;
import org.littletonrobotics.junction.Logger;

public class AimAtTagMode implements VisionDemoCommand.VisionDemoState {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private final ComboFilter angleFilter = new ComboFilter(15, 5);

  @Override
  public Pose2d updateSetpoint(Pose2d robotPose, Pose3d tagPose) {

    Rotation2d angle =
        tagPose
            .toPose2d()
            .getTranslation()
            .minus(robotPose.getTranslation())
            .getAngle()
            .plus(TARGET_HEADING_OFFSET);

    Rotation2d filteredAngle = new Rotation2d(angleFilter.calculate(angle.getRadians()));

    Logger.recordOutput("TagFollowing/AimAtTag/RawAngle", angle.getDegrees());
    Logger.recordOutput("TagFollowing/AimAtTag/FilteredAngle", filteredAngle.getDegrees());

    return new Pose2d(robotPose.getTranslation(), filteredAngle);
  }

  @Override
  public boolean blocksDriving() {
    return true;
  }
}
