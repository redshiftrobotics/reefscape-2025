package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Camera.AbsoluteTrackedTarget;
import frc.robot.subsystems.vision.Camera.RelativeTrackedTarget;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    int updatesReceived;

    boolean[] hasNewData;
    Pose3d[] estimatedRobotPose;
    double[] timestampSecondFPGA;
    int[][] tagsUsed;

    boolean connected = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(CameraIOInputs inputs) {}

  default void setLastPoseSupplier(Supplier<Pose2d> lastPoseSupplier) {}

  default List<RelativeTrackedTarget> getRelativeTargets() {
    return List.of();
  }

  default List<AbsoluteTrackedTarget> getAbsoluteTargets() {
    return List.of();
  }

  /** Get name of io camera */
  default String getCameraName() {
    return "Camera";
  }

  default CameraConfig getCameraConfig() {
    return null;
  }

  /** Set april tag field layout to use */
  default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {}
}
