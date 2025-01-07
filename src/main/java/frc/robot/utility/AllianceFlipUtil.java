package frc.robot.utility;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {

  private AllianceFlipUtil() {}

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) translation = FlippingUtil.flipFieldPosition(translation);
    return translation;
  }

  /** Flips a rotation 180 degrees based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) rotation = FlippingUtil.flipFieldRotation(rotation);
    return rotation;
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) FlippingUtil.flipFieldPose(pose);
    return pose;
  }
  /** Get whether to flip. If alliance is blue or unknown don't flip, if it is red then flip. */
  public static boolean shouldFlip() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }
}
