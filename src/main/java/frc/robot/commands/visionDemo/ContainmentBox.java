package frc.robot.commands.visionDemo;

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

public class ContainmentBox extends VirtualSubsystem {

  private final Translation2d center;
  private final Translation2d containedObjectSzie;

  private Translation2d bottomLeft;
  private Translation2d topRight;

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
    this.containedObjectSzie = containedObjectSize;

    this.tunableFactory = new LoggedTunableNumberFactory("ContainmentBox/" + name);

    bottomLeft = new Translation2d(center.getX() - length / 2.0, center.getY() - width / 2.0);

    topRight = new Translation2d(center.getX() + length / 2.0, center.getY() + width / 2.0);

    this.lengthTunable = tunableFactory.getNumber("Length", length);
    this.widthTunable = tunableFactory.getNumber("Width", width);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          bottomLeft =
              new Translation2d(center.getX() - values[0] / 2.0, center.getY() - values[1] / 2.0);
          topRight =
              new Translation2d(center.getX() + values[0] / 2.0, center.getY() + values[1] / 2.0);
        },
        lengthTunable,
        widthTunable);

    Logger.recordOutput("ContainmentBox/" + name + "/Outline", getOutline());
    Logger.recordOutput("ContainmentBox/" + name + "/Cones", getCones());
    Logger.recordOutput("ContainmentBox/" + name + "/Center", center);
  }

  public Translation2d clamp(Translation2d point) {
    double clampedX =
        Math.min(
            Math.max(point.getX(), bottomLeft.getX() + containedObjectSzie.getX() / 2.0),
            topRight.getX() - containedObjectSzie.getX() / 2.0);
    double clampedY =
        Math.min(
            Math.max(point.getY(), bottomLeft.getY() + containedObjectSzie.getY() / 2.0),
            topRight.getY() - containedObjectSzie.getY() / 2.0);
    return new Translation2d(clampedX, clampedY);
  }

  public Pose2d clamp(Pose2d pose) {
    return new Pose2d(clamp(pose.getTranslation()), pose.getRotation());
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
