package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.visionDemo.SuperstructureUtil.SuperstructureState;
import frc.robot.commands.visionDemo.VisionDemoCommand.ResultSaftyMode;
import frc.robot.commands.visionDemo.VisionDemoCommand.VisionDemoResult;
import frc.robot.commands.visionDemo.filters.ComboFilter;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class AimAtTagMode implements VisionDemoCommand.VisionDemoMode {

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;

  private final ComboFilter angleFilter = new ComboFilter(3, 5);

  private final BooleanSupplier followWithSuperstructure;

  public AimAtTagMode(BooleanSupplier followWithSuperstructure) {
    this.followWithSuperstructure = followWithSuperstructure;
  }

  @Override
  public ResultSaftyMode getExspectedSaftyMode() {
    return ResultSaftyMode.ROTATIONAL_CONTROL;
  }

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
  public VisionDemoResult calculate(Pose2d robotPose, Pose3d tagPose, double dt) {
    Pose2d rawPose = getRawPose(robotPose, tagPose);
    Rotation2d filteredAngle =
        new Rotation2d(angleFilter.calculate(rawPose.getRotation().getRadians()));

    return new VisionDemoResult(
        Optional.of(new Pose2d(rawPose.getTranslation(), filteredAngle)),
        ResultSaftyMode.ROTATIONAL_CONTROL);
  }

  @Override
  public Optional<SuperstructureState> getSuperstructureState(Pose2d robotPose, Pose3d tagPose3d) {
    if (!followWithSuperstructure.getAsBoolean()) {
      return Optional.empty();
    }
    return Optional.of(SuperstructureUtil.pointAtTag(tagPose3d));
  }
}
