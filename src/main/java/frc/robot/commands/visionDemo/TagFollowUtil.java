package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

public class TagFollowUtil {

  public static Pose3d averagePoses(List<Pose3d> poses) {
    double x = 0;
    double y = 0;
    double z = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    for (Pose3d pose : poses) {
      x += pose.getX();
      y += pose.getY();
      z += pose.getZ();
      roll += pose.getRotation().getX();
      pitch += pose.getRotation().getY();
      yaw += pose.getRotation().getZ();
    }

    int count = poses.size();
    return new Pose3d(
        x / count, y / count, z / count, new Rotation3d(roll / count, pitch / count, yaw / count));
  }
}
