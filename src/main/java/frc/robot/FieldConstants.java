package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.
 *
 * <p>Length refers to the <i>x</i> direction (as described by wpilib) Width refers to the <i>y</i>
 * direction (as described by wpilib)
 */
public class FieldConstants {

  public static final double FIELD_SIZE_Y = Units.inchesToMeters(690.875);
  public static final double FIELD_SIZE_X = Units.inchesToMeters(317);

  public static final double APRIL_TAG_WIDTH = Units.inchesToMeters(6.50);
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  static {
    APRIL_TAG_FIELD_LAYOUT.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }
}
