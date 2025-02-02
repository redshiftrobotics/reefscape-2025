package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    int updatesReceived;

    boolean[] hasNewData;
    Pose3d[] estimatedRobotPose;
    double[] timestampSecondFPGA;
    PhotonTrackedTarget[][] targets;
    int[][] tagsUsed;

    boolean connected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CameraIOInputs inputs) {}

  /** Get name of io camera */
  public default String getCameraName() {
    return "Camera";
  }

  public default Transform3d getRobotToCamera() {
    return new Transform3d();
  }
}
