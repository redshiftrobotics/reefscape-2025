package frc.robot.commands.visionDemo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utility.VirtualSubsystem;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/** Virtual subsystem for logging/displaying the state of the Safety box. */
public class ContainmentBox extends VirtualSubsystem {

  private final Translation2d center;
  private final Translation2d objectSize;

  private Translation2d bottomLeft;
  private Translation2d topRight;

  private Translation2d innerBottomLeft;
  private Translation2d innerTopRight;

  private final String name;

  private final LoggedTunableNumberFactory tunableFactory;
  private final LoggedTunableNumber lengthTunable;
  private final LoggedTunableNumber widthTunable;

  public ContainmentBox(
      String name,
      Translation2d center,
      double length,
      double width,
      Translation2d containedObjectSize) {
    this.name = name;
    this.center = center;
    this.objectSize = containedObjectSize;

    this.tunableFactory = new LoggedTunableNumberFactory("ContainmentBox/" + name);

    updateSize(length, width);

    this.lengthTunable = tunableFactory.getNumber("Length", length);
    this.widthTunable = tunableFactory.getNumber("Width", width);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          double length = values[0];
          double width = values[1];
          updateSize(length, width);
        },
        lengthTunable,
        widthTunable);

    Logger.recordOutput("ContainmentBox/" + name + "/Outline", getOutline());
    Logger.recordOutput("ContainmentBox/" + name + "/Cones", getCones());
    Logger.recordOutput("ContainmentBox/" + name + "/Center", center);
  }

  public void updateSize(double length, double width) {
    Translation2d size = new Translation2d(length, width);
    bottomLeft = center.minus(size.div(2.0));
    topRight = center.plus(size.div(2.0));
    innerBottomLeft = bottomLeft.plus(objectSize.div(2.0));
    innerTopRight = topRight.minus(objectSize.div(2.0));
  }

  public Translation2d clamp(Translation2d point) {
    return new Translation2d(
        MathUtil.clamp(point.getX(), innerBottomLeft.getX(), innerTopRight.getX()),
        MathUtil.clamp(point.getY(), innerBottomLeft.getY(), innerTopRight.getY()));
  }

  public Pose2d clamp(Pose2d pose) {
    return new Pose2d(clamp(pose.getTranslation()), pose.getRotation());
  }

  public boolean contains(Translation2d point) {
    return point.getX() >= innerBottomLeft.getX()
        && point.getX() <= innerTopRight.getX()
        && point.getY() >= innerBottomLeft.getY()
        && point.getY() <= innerTopRight.getY();
  }

  public boolean contains(Pose2d pose) {
    return contains(pose.getTranslation());
  }

  private final Pose3d[] getCones() {
    Rotation3d rotation = new Rotation3d(0, -Math.PI / 2, 0);
    return Arrays.stream(getCorners())
        .map(corner -> new Pose3d(new Translation3d(corner.getTranslation()), rotation))
        .toArray(Pose3d[]::new);
  }

  private Pose2d[] getOutline() {
    return new Pose2d[] {
      new Pose2d(bottomLeft, Rotation2d.kZero),
      new Pose2d(new Translation2d(topRight.getX(), bottomLeft.getY()), Rotation2d.kZero),
      new Pose2d(topRight, Rotation2d.kZero),
      new Pose2d(new Translation2d(bottomLeft.getX(), topRight.getY()), Rotation2d.kZero),
      new Pose2d(bottomLeft, Rotation2d.kZero),
    };
  }

  public Pose2d[] getCorners() {
    return new Pose2d[] {
      new Pose2d(bottomLeft, Rotation2d.kZero),
      new Pose2d(new Translation2d(topRight.getX(), bottomLeft.getY()), Rotation2d.kZero),
      new Pose2d(topRight, Rotation2d.kZero),
      new Pose2d(new Translation2d(bottomLeft.getX(), topRight.getY()), Rotation2d.kZero),
    };
  }
}
