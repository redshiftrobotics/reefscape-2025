package frc.robot.commands.visionDemo;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.TrackedTarget;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimAtTag extends Command {

  public static Supplier<Translation2d> drivingTranslationSupplier = () -> Translation2d.kZero;
  public static BooleanSupplier fieldRelativeDrivingSupplier = () -> false;

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;
  private static final int MEDIAN_FILTER_SIZE = 10;

  private static final int SUPERSTRUCTURE_MEDIAN_FILTER_SIZE = 15;

  private final Drive drive;
  private final AprilTagVision vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final IntSupplier tagToFollow;

  private final MedianFilter targetHeadingFilter = new MedianFilter(MEDIAN_FILTER_SIZE);
  private final HeadingController headingController;

  private final MedianFilter elevatorHeightFilter =
      new MedianFilter(SUPERSTRUCTURE_MEDIAN_FILTER_SIZE);
  private final MedianFilter wristAngleFilter = new MedianFilter(SUPERSTRUCTURE_MEDIAN_FILTER_SIZE);

  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  public AimAtTag(AprilTagVision vision, Drive drive, IntSupplier tagToFollow) {
    this(vision, drive, tagToFollow, null, null);
  }

  public AimAtTag(
      AprilTagVision vision, Drive drive, IntSupplier tagToFollow, Elevator elevator, Wrist wrist) {
    this.vision = vision;
    this.drive = drive;
    this.tagToFollow = tagToFollow;
    this.elevator = elevator;
    this.wrist = wrist;

    this.headingController = new HeadingController(drive);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    headingController.reset();
    targetHeadingFilter.reset();
    headingController.setGoalToCurrentHeading();
  }

  @Override
  public void execute() {
    List<TrackedTarget> targets = vision.getLatestTargets();

    List<TrackedTarget> validTargets =
        targets.stream()
            .filter(
                t -> Math.abs(t.cameraToTarget().getRotation().getY()) < MAX_ALLOWED_TAG_PITCH)
            .toList();

    targets.stream()
        .filter(t -> t.id() == tagToFollow.getAsInt())
        .forEach(
            t -> {
              Pose3d robotPose = SuperstrcutreMovementUtil.toPose3d(drive.getRobotPose());

              Pose3d cameraPose = robotPose.plus(t.robotToCamera());

              Pose3d tagPose = cameraPose.plus(t.cameraToTarget());

              Rotation2d targetHeading =
                  tagPose
                      .getTranslation()
                      .minus(robotPose.getTranslation())
                      .toTranslation2d()
                      .getAngle()
                      .plus(TARGET_HEADING_OFFSET);

              Logger.recordOutput("TagFollowing/Aim/TargetHeading", targetHeading.getDegrees());

              Rotation2d meanTargetHeading =
                  new Rotation2d(targetHeadingFilter.calculate(targetHeading.getRadians()));

              SmartDashboard.putNumber(
                  "Target Heading", ((-targetHeading.getDegrees() + 360) % 360));

              if (elevator != null && wrist != null) {
                SuperstrcutreMovementUtil.TagFollowingSuperstructureState state =
                    SuperstrcutreMovementUtil.getSuperstructureState(tagPose);

                elevator.setGoalHeightMeters(
                    elevatorHeightFilter.calculate(state.elevatorHeight()));
                    
                wrist.setGoalRotation(
                    new Rotation2d(wristAngleFilter.calculate(state.wristAngle().getRadians())));
              }

              headingController.setGoal(meanTargetHeading);
            });

    Translation2d drivingTranslation = drivingTranslationSupplier.get().times(0.25);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            drivingTranslation.getX(), drivingTranslation.getY(), headingController.calculate());

    if (atGoalDebouncer.calculate(headingController.atGoal())) {
      speeds.omegaRadiansPerSecond = 0;
    }

    drive.setRobotSpeeds(speeds, fieldRelativeDrivingSupplier.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}