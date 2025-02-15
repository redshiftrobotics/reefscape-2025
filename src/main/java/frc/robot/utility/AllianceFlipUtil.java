package frc.robot.utility;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {

  public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

  private AllianceFlipUtil() {}

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? FlippingUtil.flipFieldPosition(translation) : translation;
  }

  /** Flips a rotation 180 degrees based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  /** Get whether to flip. If alliance is blue or unknown don't flip, if it is red then flip. */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE).equals(Alliance.Red);
  }
}
