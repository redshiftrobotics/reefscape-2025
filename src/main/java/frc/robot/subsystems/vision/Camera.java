package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import java.util.Arrays;
import java.util.DoubleSummaryStatistics;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

/** Wrapper for CameraIO layer */
public class Camera {

  private static LoggedTunableNumberFactory group = new LoggedTunableNumberFactory("Vision", false);

  private static final LoggedTunableNumber xyStdDevCoefficient =
      group.getNumber("xyStdDevCoefficient", 0.075);
  private static final LoggedTunableNumber thetaStdDevCoefficient =
      group.getNumber("thetaStdDevCoefficient", 0.085);

  private static final LoggedTunableNumber zHeightToleranceMeters =
      group.getNumber("zHeightToleranceMeters", 0.6);
  private static final LoggedTunableNumber pitchAndRollToleranceDegrees =
      group.getNumber("pitchToleranceDegrees", 10.0);

  private static final LoggedTunableNumber maxValidDistanceAwayFromCurrentEstimateMeters =
      group.getNumber("MaxValidDistanceFromCurrentEstimateMeters", 30.0);
  private static final LoggedTunableNumber maxValidDistanceAwayFromCurrentHeadingDegrees =
      group.getNumber("GyroFilteringToleranceDegrees", 30.0);

  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

  private double lastTimestampSecondsFPGA = -1;
  private Pose3d[] tagPositionsOnField = new Pose3d[] {};

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Set<Integer> tagsIdsOnField;

  private final Alert missingCameraAlert;

  /**
   * Create a new robot camera with IO layer
   *
   * @param io camera implantation
   */
  public Camera(CameraIO io, AprilTagFieldLayout aprilTagFieldLayout) {
    this.io = io;

    io.setAprilTagFieldLayout(aprilTagFieldLayout);

    this.aprilTagFieldLayout = aprilTagFieldLayout;
    this.tagsIdsOnField =
        aprilTagFieldLayout.getTags().stream().map((tag) -> tag.ID).collect(Collectors.toSet());

    this.missingCameraAlert =
        new Alert("Missing Camera: " + this.getCameraName(), Alert.AlertType.kError);
  }

  /** Get name of camera as specified by IO */
  public String getCameraName() {
    return io.getCameraName();
  }

  /** Run periodic of module. Updates the set of loggable inputs, updating vision result. */
  public void periodic() {
    lastTimestampSecondsFPGA = inputs.timestampSecondsFPGA;
    Logger.processInputs("Vision/" + getCameraName(), inputs);
    io.updateInputs(inputs);
    missingCameraAlert.set(inputs.connected);

    // Logging

    tagPositionsOnField =
        Arrays.stream(inputs.tagsUsed)
            .mapToObj(aprilTagFieldLayout::getTagPose)
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toArray(Pose3d[]::new);
  }

  public boolean hasNewData() {
    return inputs.hasNewData;
  }

  public Pose3d[] getTagPositionsOnField() {
    return tagPositionsOnField;
  }

  /** Get the pose of the robot as measured by the vision camera. */
  public Pose3d getEstimatedRobotPose() {
    return inputs.estimatedRobotPose;
  }

  /**
   * Get the timestamp of the vision measurement in seconds. The timestamp has an epoch since FPGA
   * time startup
   */
  public double getTimestampSeconds() {
    return inputs.timestampSecondsFPGA;
  }

  /**
   * Get standard deviations of the vision measurements. Higher values numbers here means trust
   * global measurements from this camera less. The matrix is in the form [x, y, theta], with units
   * in meters and radians.
   */
  public Matrix<N3, N1> getStandardDeviations() {

    // Get data about distance to each tag that is present on field
    DoubleSummaryStatistics distanceToTagsUsedSummary =
        Arrays.stream(getTagPositionsOnField())
            .mapToDouble(
                (tagPose3d) ->
                    tagPose3d
                        .getTranslation()
                        .getDistance(getEstimatedRobotPose().getTranslation()))
            .summaryStatistics();

    // This equation is heuristic, good enough but can probably be improved
    // Larger distances to tags and fewer observed tags result in higher uncertainty (larger
    // standard deviations). Average distance increases uncertainty exponentially while more
    // tags decreases uncertainty linearly
    double standardDeviation =
        distanceToTagsUsedSummary.getCount() > 0
            ? Math.pow(distanceToTagsUsedSummary.getAverage(), 2)
                * Math.pow(distanceToTagsUsedSummary.getCount(), -1)
            : Double.POSITIVE_INFINITY;

    double xyStandardDeviation = xyStdDevCoefficient.get() * standardDeviation;

    double thetaStandardDeviation = thetaStdDevCoefficient.get() * standardDeviation;

    // x, y, theta
    return VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation);
  }

  /** Get the status of the vision measurement */
  public VisionResultStatus getStatus(Pose2d lastRobotPose) {
    VisionResultStatus status = getStatus();

    if (!status.isSuccess()) {
      return status;
    }

    Pose2d estimatedRobotPose2d = inputs.estimatedRobotPose.toPose2d();

    if (!MathUtil.isNear(
        estimatedRobotPose2d.getRotation().getDegrees(),
        lastRobotPose.getRotation().getDegrees(),
        maxValidDistanceAwayFromCurrentHeadingDegrees.get())) {
      return VisionResultStatus.NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION;
    }

    if (estimatedRobotPose2d.getTranslation().getDistance(lastRobotPose.getTranslation())
        > maxValidDistanceAwayFromCurrentEstimateMeters.get()) {
      return VisionResultStatus.TOO_FAR_FROM_EXISTING_ESTIMATE;
    }

    return status;
  }

  public VisionResultStatus getStatus() {

    if (!inputs.hasNewData) {
      return VisionResultStatus.NO_DATA;
    }

    if (inputs.timestampSecondsFPGA == this.lastTimestampSecondsFPGA) {
      return VisionResultStatus.NOT_A_NEW_RESULT;
    }

    if (inputs.tagsUsed.length == 0) {
      return VisionResultStatus.NO_TARGETS_VISIBLE;
    }

    if (!Arrays.stream(inputs.tagsUsed).allMatch(tagsIdsOnField::contains)) {
      return VisionResultStatus.INVALID_TAG;
    }

    if (inputs.estimatedRobotPose.getX() < 0
        || inputs.estimatedRobotPose.getY() < 0
        || inputs.estimatedRobotPose.getX() > aprilTagFieldLayout.getFieldLength()
        || inputs.estimatedRobotPose.getY() > aprilTagFieldLayout.getFieldWidth()) {
      return VisionResultStatus.INVALID_POSE_OUTSIDE_FIELD;
    }

    if (!MathUtil.isNear(0, inputs.estimatedRobotPose.getZ(), zHeightToleranceMeters.get())) {
      return VisionResultStatus.Z_HEIGHT_BAD;
    }

    double pitchAndRollToleranceValueRadians =
        Units.degreesToRadians(pitchAndRollToleranceDegrees.get());
    if (!MathUtil.isNear(
            0, inputs.estimatedRobotPose.getRotation().getX(), pitchAndRollToleranceValueRadians)
        && !MathUtil.isNear(
            0, inputs.estimatedRobotPose.getRotation().getY(), pitchAndRollToleranceValueRadians)) {
      return VisionResultStatus.PITCH_OR_ROLL_BAD;
    }

    return VisionResultStatus.SUCCESSFUL;
  }

  public enum VisionResultStatus {
    NO_DATA(false),

    NOT_A_NEW_RESULT(false),
    NO_TARGETS_VISIBLE(false),
    INVALID_TAG(false),

    INVALID_POSE_OUTSIDE_FIELD(false),
    Z_HEIGHT_BAD(false),
    PITCH_OR_ROLL_BAD(false),

    NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION(false),
    TOO_FAR_FROM_EXISTING_ESTIMATE(false),

    SUCCESSFUL(true);

    public final boolean success;

    private VisionResultStatus(boolean success) {
      this.success = success;
    }

    public boolean isSuccess() {
      return success;
    }
  }
}
