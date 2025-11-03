package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.utility.tunable.LoggedTunableNumber;
import frc.robot.utility.tunable.LoggedTunableNumberFactory;
import java.util.Arrays;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

/** Wrapper for CameraIO layer */
public class Camera {

  private static final LoggedTunableNumberFactory group =
      new LoggedTunableNumberFactory("VisionResultsStatus");

  private static final LoggedTunableNumber xyStdDevCoefficient =
      group.getNumber("xyStdDevCoefficient", 0.075);
  private static final LoggedTunableNumber thetaStdDevCoefficient =
      group.getNumber("thetaStdDevCoefficient", 0.085);

  private static final LoggedTunableNumber zHeightToleranceMeters =
      group.getNumber("zHeightToleranceMeters", 0.6);
  private static final LoggedTunableNumber pitchAndRollToleranceDegrees =
      group.getNumber("pitchToleranceDegrees", 10.0);

  private static final LoggedTunableNumber maxValidDistanceAwayFromCurrentEstimateMeters =
      group.getNumber("maxValidDistanceFromCurrentEstimateMeters", 10.0);
  private static final LoggedTunableNumber maxValidDistanceAwayFromCurrentHeadingDegrees =
      group.getNumber("gyroFilteringToleranceDegrees", 30.0);

  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Set<Integer> tagsIdsOnField;

  private VisionResult[] results = new VisionResult[0];

  private Supplier<Pose2d> lastRobotPoseSupplier;

  private final Alert missingCameraAlert;

  public static record VisionResult(
      boolean hasNewData,
      Pose3d estimatedRobotPose,
      double timestampSecondFPGA,
      int[] tagsUsed,
      Pose3d[] tagPositionsOnField,
      Matrix<N3, N1> standardDeviation,
      VisionResultStatus status) {}

  public record RelativeTrackedTarget(
      int id, Transform3d cameraToTarget, CameraConfig camera, double poseAmbiguity) {}

  public record AbsoluteTrackedTarget(
      int id,
      Pose3d targetPose,
      Pose3d cameraPose,
      Transform3d cameraToTarget,
      CameraConfig camera,
      double poseAmbiguity,
      Pose2d robotPose) {
    public AbsoluteTrackedTarget(RelativeTrackedTarget relativeTarget, Pose2d robotPose) {
      this(
          relativeTarget.id,
          new Pose3d(robotPose)
              .plus(relativeTarget.camera.robotToCamera())
              .plus(relativeTarget.cameraToTarget),
          new Pose3d(robotPose).plus(relativeTarget.camera.robotToCamera()),
          relativeTarget.cameraToTarget,
          relativeTarget.camera,
          relativeTarget.poseAmbiguity,
          robotPose);
    }
  }

  /**
   * Create a new robot camera with IO layer
   *
   * @param io camera implantation
   */
  public Camera(CameraIO io) {
    this.io = io;

    this.missingCameraAlert =
        new Alert(String.format("Missing cameras %s", getCameraName()), Alert.AlertType.kWarning);
  }

  public void setFieldTags(AprilTagFieldLayout field) {
    if (field != this.aprilTagFieldLayout) {
      io.setAprilTagFieldLayout(field);
      tagsIdsOnField = field.getTags().stream().map((tag) -> tag.ID).collect(Collectors.toSet());
      this.aprilTagFieldLayout = field;
    }
  }

  /** Get name of camera as specified by IO */
  public String getCameraName() {
    return io.getCameraConfig() + " (" + io.getCameraName() + ")";
  }

  /** Run periodic of module. Updates the set of loggable inputs, updating vision result. */
  public void periodic() {
    Logger.processInputs("Vision/" + getCameraName(), inputs);
    io.updateInputs(inputs);
    missingCameraAlert.set(!inputs.connected);

    results = new VisionResult[inputs.updatesReceived];
    for (int i = 0; i < inputs.updatesReceived; i++) {
      Pose3d[] tagPositionsOnField = getTagPositionsOnField(inputs.tagsUsed[i]);

      if (inputs.hasNewData[i]) {
        Matrix<N3, N1> standardDeviations =
            getStandardDeviations(tagPositionsOnField, inputs.estimatedRobotPose[i]);
        VisionResultStatus status =
            lastRobotPoseSupplier == null
                ? getStatus(inputs.estimatedRobotPose[i], inputs.tagsUsed[i])
                : getStatus(
                    inputs.estimatedRobotPose[i], inputs.tagsUsed[i], lastRobotPoseSupplier.get());
        results[i] =
            new VisionResult(
                true,
                inputs.estimatedRobotPose[i],
                inputs.timestampSecondFPGA[i],
                inputs.tagsUsed[i],
                tagPositionsOnField,
                standardDeviations,
                status);
      } else {
        results[i] =
            new VisionResult(
                false,
                null,
                0,
                new int[0],
                new Pose3d[0],
                VecBuilder.fill(0, 0, 0),
                VisionResultStatus.NO_DATA);
      }
    }
  }

  public List<AbsoluteTrackedTarget> getAbsoluteTargets() {
    return io.getAbsoluteTargets();
  }

  public List<RelativeTrackedTarget> getRelativeTargets() {
    return io.getRelativeTargets();
  }

  public void filterBasedOnLastPose(boolean filter, Supplier<Pose2d> lastRobotPose) {
    this.io.setLastPoseSupplier(lastRobotPose);
    this.lastRobotPoseSupplier = filter ? lastRobotPose : null;
  }

  public VisionResult[] getResults() {
    return results;
  }

  private Pose3d[] getTagPositionsOnField(int[] tagsUsed) {
    return Arrays.stream(tagsUsed)
        .mapToObj(aprilTagFieldLayout::getTagPose)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  /**
   * Get standard deviations of the vision measurements. Higher values numbers here means trust
   * global measurements from this camera less. The matrix is in the form [x, y, theta], with units
   * in meters and radians.
   */
  private Matrix<N3, N1> getStandardDeviations(Pose3d[] tagPositionsOnField, Pose3d lastRobotPose) {

    // Get data about distance to each tag that is present on field
    DoubleSummaryStatistics distancesToTags =
        Arrays.stream(tagPositionsOnField)
            .mapToDouble(
                (tagPose3d) ->
                    tagPose3d.getTranslation().getDistance(lastRobotPose.getTranslation()))
            .summaryStatistics();

    // This equation is heuristic, good enough but can probably be improved
    // Larger distances to tags and fewer observed tags result in higher uncertainty (larger
    // standard deviations). Average distance increases uncertainty exponentially while more
    // tags decreases uncertainty linearly
    double standardDeviation =
        distancesToTags.getCount() > 0
            ? Math.pow(distancesToTags.getAverage(), 2) * Math.pow(distancesToTags.getCount(), -1)
            : Double.POSITIVE_INFINITY;

    double xyStandardDeviation = xyStdDevCoefficient.get() * standardDeviation;

    double thetaStandardDeviation = thetaStdDevCoefficient.get() * standardDeviation;

    // x, y, theta
    return VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation);
  }

  /** Get the status of the vision measurement */
  private VisionResultStatus getStatus(
      Pose3d estimatedRobotPose, int[] tagsUsed, Pose2d lastRobotPose) {
    VisionResultStatus status = getStatus(estimatedRobotPose, tagsUsed);

    if (!status.isSuccess()) {
      return status;
    }

    Pose2d estimatedRobotPose2d = estimatedRobotPose.toPose2d();

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

  private VisionResultStatus getStatus(Pose3d estimatedRobotPose, int[] tagsUsed) {

    if (tagsUsed.length == 0) {
      return VisionResultStatus.NO_TARGETS_VISIBLE;
    }

    if (!Arrays.stream(tagsUsed).allMatch(tagsIdsOnField::contains)) {
      return VisionResultStatus.INVALID_TAG;
    }

    if (estimatedRobotPose.getX() < 0
        || estimatedRobotPose.getY() < 0
        || estimatedRobotPose.getX() > aprilTagFieldLayout.getFieldLength()
        || estimatedRobotPose.getY() > aprilTagFieldLayout.getFieldWidth()) {
      return VisionResultStatus.INVALID_POSE_OUTSIDE_FIELD;
    }

    if (!MathUtil.isNear(0, estimatedRobotPose.getZ(), zHeightToleranceMeters.get())) {
      return VisionResultStatus.Z_HEIGHT_BAD;
    }

    double pitchAndRollToleranceValueRadians =
        Units.degreesToRadians(pitchAndRollToleranceDegrees.get());
    if (!MathUtil.isNear(
            0, estimatedRobotPose.getRotation().getX(), pitchAndRollToleranceValueRadians)
        && !MathUtil.isNear(
            0, estimatedRobotPose.getRotation().getY(), pitchAndRollToleranceValueRadians)) {
      return VisionResultStatus.PITCH_OR_ROLL_BAD;
    }

    return VisionResultStatus.SUCCESSFUL;
  }

  public enum VisionResultStatus {
    NO_DATA(false),

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
