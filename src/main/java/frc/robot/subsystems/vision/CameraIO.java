package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {

    boolean hasNewData = false;

    Pose3d estimatedRobotPose = new Pose3d();
    double timestampSecondsFPGA = -1;

    int[] tagsUsed = new int[] {};

    boolean connected = false;
  }

  /** Get name of io camera */
  public default String getCameraName() {
    return "Camera";
  }

  /** Set april tag field layout to use */
  public default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CameraIOInputs inputs) {}
}
