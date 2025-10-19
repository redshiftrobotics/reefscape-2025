package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utility.JoystickUtil;
import frc.robot.utility.VirtualSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SimControlledTarget extends VirtualSubsystem {

  private static final double TRANSLATION_SPEED = 0.03;
  private static final double TRIGGER_SPEED = 0.03;
  private static final double ROTATION_SPEED = 0.1;
  private static final double BUTTON_ROTATION_SPEED = 0.1;

  private final XboxController controller;

  private final int tagId;

  private Pose3d tagPose;
  private boolean tagHidden = true;

  public SimControlledTarget(int tagID, Pose3d startingPose, XboxController controller) {
    this.tagId = tagID;
    this.controller = controller;

    this.tagPose = startingPose;
  }

  @Override
  public void periodic() {

    tagHidden = controller.getAButton();

    tagPose =
        new Pose3d(
            new Translation3d(
                tagPose.getX()
                    - JoystickUtil.applyDeadband(controller.getLeftY()) * TRANSLATION_SPEED,
                tagPose.getY()
                    - JoystickUtil.applyDeadband(controller.getLeftX()) * TRANSLATION_SPEED,
                tagPose.getZ()
                    - controller.getLeftTriggerAxis() * TRIGGER_SPEED
                    + controller.getRightTriggerAxis() * TRIGGER_SPEED),
            new Rotation3d(
                MathUtil.angleModulus(
                    tagPose.getRotation().getX() * (controller.getPOV() == 0 ? 0 : 1)
                        - (controller.getPOV() == 270 ? BUTTON_ROTATION_SPEED : 0)
                        + (controller.getPOV() == 90 ? BUTTON_ROTATION_SPEED : 0)),
                MathUtil.angleModulus(
                    tagPose.getRotation().getY()
                        + JoystickUtil.applyDeadband(controller.getRightY()) * ROTATION_SPEED),
                MathUtil.angleModulus(
                    tagPose.getRotation().getZ()
                        - JoystickUtil.applyDeadband(controller.getRightX()) * ROTATION_SPEED)));

    Logger.recordOutput("SimControlledTarget/TagPose", tagPose);
    Logger.recordOutput(
        "SimControlledTarget/VisibleTags", tagHidden ? new Pose3d[] {} : new Pose3d[] {tagPose});

    Logger.recordOutput("SimControlledTarget/TagId", tagId);
  }

  public AprilTagFieldLayout createFieldWithTarget(AprilTagFieldLayout initialField) {

    List<AprilTag> tags = initialField.getTags();

    tags.removeIf(tag -> tag.ID == tagId);
    if (!tagHidden) {
      tags.add(new AprilTag(tagId, tagPose));
    }

    return new AprilTagFieldLayout(
        tags, initialField.getFieldLength(), initialField.getFieldWidth());
  }

  public int getTagId() {
    return tagId;
  }

  public Pose3d getTagPose() {
    return tagPose;
  }
}
