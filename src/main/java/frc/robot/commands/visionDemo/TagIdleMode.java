package frc.robot.commands.visionDemo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.visionDemo.VisionDemoCommand.VisionDemoState;

public class TagIdleMode implements VisionDemoState {

  @Override
  public Pose2d updateSetpoint(Pose2d robotPose, Pose3d tagPose) {
    return robotPose;
  }

  @Override
  public ChassisSpeeds updateSpeeds(ChassisSpeeds currentSpeeds) {
    return new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public String name() {
    return "IDLE MODE";
  }

  @Override
  public int tagId() {
    return -1;
  }

  @Override
  public int priority() {
    return 0;
  }
}
