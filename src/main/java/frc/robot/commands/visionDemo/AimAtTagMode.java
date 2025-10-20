package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.visionDemo.filters.ComboFilter;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AimAtTagMode implements VisionDemoCommand.VisionDemoState {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private final int tagId;
  private final BooleanSupplier useSuperstructure;

  private final ComboFilter angleFilter = new ComboFilter(15, 5);

  public AimAtTagMode(int tagId, BooleanSupplier useSuperstructure) {
    this.tagId = tagId;
    this.useSuperstructure = useSuperstructure;
  }

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

    String keyPrefix = "TagFollowing/AimAtTag" + tagId + "/";
    Logger.recordOutput(keyPrefix + "RawAngle", angle.getDegrees());
    Logger.recordOutput(keyPrefix + "FilteredAngle", filteredAngle.getDegrees());

    return new Pose2d(robotPose.getTranslation(), filteredAngle);
  }

  @Override
  public String name() {
    return "AIM AT TAG " + tagId;
  }

  @Override
  public int tagId() {
    return tagId;
  }

  @Override
  public int priority() {
    return 1;
  }

  @Override
  public boolean usesSuperstructure() {
    return useSuperstructure.getAsBoolean();
  }
}
