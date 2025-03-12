package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
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

  /** Get name of io camera */
  default String getCameraName() {
    return "Camera";
  }

  default String getCameraPosition() {
    return getCameraName();
  }

  /** Set april tag field layout to use */
  default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {}
}
