package frc.robot.commands.visionDemo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.controllers.SimpleDriveController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.Camera.TrackedTarget;

import java.util.List;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FollowTag extends Command {

  public static Supplier<Translation2d> drivingTranslationSupplier = () -> Translation2d.kZero;

  private static final Rotation2d TARGET_HEADING_OFFSET = Rotation2d.k180deg;
  private static final Translation2d TARGET_OFFSET = new Translation2d(2, 0);
  private static final Pose2d TOLERANCE =
      new Pose2d(Units.inchesToMeters(6), Units.inchesToMeters(3), Rotation2d.fromDegrees(6));

  private static final int SUPERSTRUCTURE_MEDIAN_FILTER_SIZE = 5;

  private static final double FILTER_SLEW_RATE = Units.feetToMeters(5);
  private static final double MAX_TAG_JUMP = Units.feetToMeters(1);

  private static final double MAX_ALLOWED_TAG_PITCH = Units.degreesToRadians(80);

  private final Drive drive;
  private final AprilTagVision vision;

  private final Elevator elevator;
  private final Wrist wrist;

  private final IntSupplier tagToFollow;

  private final SimpleDriveController controller;

  private Translation2d targetOffset;

  private Translation3d tagFilteredPosition = Translation3d.kZero;

  private final MedianFilter elevatorHeightFilter =
      new MedianFilter(SUPERSTRUCTURE_MEDIAN_FILTER_SIZE);
  private final MedianFilter wristAngleFilter = new MedianFilter(SUPERSTRUCTURE_MEDIAN_FILTER_SIZE);

  private final Debouncer atGoalDebouncer = new Debouncer(0.2);

  public FollowTag(AprilTagVision vision, Drive drive, IntSupplier tagToFollow) {
    this(vision, drive, tagToFollow, null, null);
  }

  public FollowTag(
      AprilTagVision vision, Drive drive, IntSupplier tagToFollow, Elevator elevator, Wrist wrist) {
    this.vision = vision;
    this.drive = drive;
    this.tagToFollow = tagToFollow;
    this.elevator = elevator;
    this.wrist = wrist;

    this.controller = new SimpleDriveController();

    controller.setTolerance(TOLERANCE);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = drive.getRobotPose();

    targetOffset = TARGET_OFFSET;

    controller.reset(robotPose);
    controller.setSetpoint(robotPose);

    this.tagFilteredPosition = SuperstrcutreMovementUtil.toPose3d(robotPose).getTranslation();
  }

  @Override
  public void execute() {
    List<TrackedTarget> targets = vision.getLatestTargets();

    targets.stream()
        .filter(TrackedTarget::isGoodPoseAmbiguity)
        .filter(t -> t.id() == tagToFollow.getAsInt())
        .forEach(
            t -> {
              Pose3d robotPose = SuperstrcutreMovementUtil.toPose3d(drive.getRobotPose());

              Pose3d cameraPose = robotPose.plus(t.robotToCamera());

              Pose3d tagPose = cameraPose.plus(t.cameraToTarget());

              tagFilteredPosition =
                  MathUtil.slewRateLimit(
                      tagFilteredPosition,
                      tagPose.getTranslation(),
                      Constants.LOOP_PERIOD_SECONDS,
                      FILTER_SLEW_RATE);

              Rotation2d targetRotation =
                  tagPose
                      .getRotation()
                      .toRotation2d()
                      .plus(Rotation2d.k180deg)
                      .plus(TARGET_HEADING_OFFSET);

              Translation2d targetTranslation =
                  tagPose
                      .toPose2d()
                      .plus(new Transform2d(targetOffset, Rotation2d.kZero))
                      .getTranslation();

              SmartDashboard.putNumber(
                  "Target Heading", ((-targetRotation.getDegrees() + 360) % 360));

              Pose2d target = new Pose2d(targetTranslation, targetRotation);

              double distance = tagFilteredPosition.getDistance(tagPose.getTranslation());
              boolean withinMaxJump = distance < MAX_TAG_JUMP;

              boolean tagRotationGood =
                  Math.abs(tagPose.getRotation().getY()) < MAX_ALLOWED_TAG_PITCH;

              Logger.recordOutput("TagFollowing/Follow/TagDistanceToSafe", distance);
              Logger.recordOutput("TagFollowing/Follow/WithinMaxJump", withinMaxJump);
              Logger.recordOutput(
                  "TagFollowing/Follow/TagFilteredPosition",
                  new Pose3d(this.tagFilteredPosition, tagPose.getRotation()));
              Logger.recordOutput("TagFollowing/Follow/RawTargetPosition", target);
              Logger.recordOutput("TagFollowing/Follow/TagRotationGood", tagRotationGood);

              if (withinMaxJump && tagRotationGood) {
                controller.setSetpoint(
                    target.plus(
                        new Transform2d(
                            drivingTranslationSupplier
                                .get()
                                .unaryMinus()
                                .div(DriveConstants.DRIVE_CONFIG.maxLinearVelocity()),
                            Rotation2d.kZero)));

                if (elevator != null && wrist != null) {
                  SuperstrcutreMovementUtil.TagFollowingSuperstructureState state =
                      SuperstrcutreMovementUtil.getSuperstructureState(tagPose);
                  elevator.setGoalHeightMeters(
                      elevatorHeightFilter.calculate(state.elevatorHeight()));
                  wrist.setGoalRotation(
                      new Rotation2d(wristAngleFilter.calculate(state.wristAngle().getRadians())));
                }
              }
            });

    ChassisSpeeds speeds = controller.calculate(drive.getRobotPose());
    if (atGoalDebouncer.calculate(controller.atReference())) {
      speeds = new ChassisSpeeds();
    }
    drive.setRobotSpeeds(speeds);
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