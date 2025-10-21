package frc.robot.commands.visionDemo;

import java.util.List;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.TrackedTarget;

public class VisionSystem {
  
  private final AprilTagVision vision;

  public VisionSystem(AprilTagVision vision) {
    this.vision = vision;
  }
  
  public List<TrackedTarget> getTag(int tagId) {
    return vision.getLatestTargets().stream()
            .filter(TrackedTarget::isGoodPoseAmbiguity)
            .filter(
                tag ->
                    Math.abs(tag.cameraToTarget().getRotation().getY())
                        < Units.degreesToRadians(80))
            .filter(tag -> tag.id() == tagId)
            .toList();
  }

}
