package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {

    VisionResult[] results = new VisionResult[0];

    boolean connected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CameraIOInputs inputs) {}

  /** Get name of io camera */
  public default String getCameraName() {
    return "Camera";
  }

  /** Set april tag field layout to use */
  public default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {}

  public record VisionResult(
      Pose3d estimatedRobotPose,
      double timestampSecondFPGA,
      List<Integer> tagsUsed,
      boolean hasNewData) {}
}
