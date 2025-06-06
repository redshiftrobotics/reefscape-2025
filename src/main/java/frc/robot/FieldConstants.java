// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.
 *
 * <p>Length refers to the <i>x</i> direction. Width refers to the <i>y</i> direction. (as described
 * by WPILib)
 *
 * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);

  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {

    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));

    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));

    private static final double[] offset = {Units.inchesToMeters(-14), 0, Units.inchesToMeters(14)};

    public static final Pose2d[] alignmentFaces = new Pose2d[2 * offset.length];

    static {
      for (int i = 0; i < offset.length; i++) {
        alignmentFaces[i] =
            leftCenterFace.transformBy(new Transform2d(0, offset[i], Rotation2d.kZero));
      }
      for (int i = 0; i < offset.length; i++) {
        alignmentFaces[i + offset.length] =
            rightCenterFace.transformBy(new Transform2d(0, offset[i], Rotation2d.kZero));
      }
    }
  }

  public static class Reef {

    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    public static final Pose2d[] alignmentFaces =
        new Pose2d
            [12]; // Starting facing the driver station in clockwise order, positions robot is in
    // front of branch

    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    public static final List<Pose3d> coralPlacementPositions = new ArrayList<>();

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));

      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));

      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));

      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));

      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));

      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions

      double adjustX = Units.inchesToMeters(30.738);
      double adjustY = Units.inchesToMeters(6.469);

      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();

        alignmentFaces[(face * 2) + 0] =
            centerFaces[face].transformBy(new Transform2d(0, adjustY, new Rotation2d()));
        alignmentFaces[(face * 2) + 1] =
            centerFaces[face].transformBy(new Transform2d(0, -adjustY, new Rotation2d()));

        for (ReefHeight level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }

        branchPositions.add((face * 2) + 0, fillRight);
        branchPositions.add((face * 2) + 1, fillLeft);

        coralPlacementPositions.addAll(fillRight.values());
        coralPlacementPositions.addAll(fillLeft.values());
        coralPlacementPositions.addAll(
            fillLeft.values().stream()
                .map(
                    p -> {
                      Pose2d p2d = p.toPose2d();
                      return new Pose3d(FlippingUtil.flipFieldPose(p2d))
                          .plus(p.minus(new Pose3d(p2d)));
                    })
                .toList());
        coralPlacementPositions.addAll(
            fillRight.values().stream()
                .map(
                    p -> {
                      Pose2d p2d = p.toPose2d();
                      return new Pose3d(FlippingUtil.flipFieldPose(p2d))
                          .plus(p.minus(new Pose3d(p2d)));
                    })
                .toList());
      }
    }

    static {
      // idk
      var flipped =
          coralPlacementPositions.stream()
              .map(
                  p ->
                      new Pose3d(
                          p.getTranslation(),
                          new Rotation3d(
                              p.getRotation().getX(),
                              p.getRotation().getY() + Math.PI,
                              p.getRotation().getZ())))
              .toList();
      coralPlacementPositions.clear();
      coralPlacementPositions.addAll(flipped);
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());

    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());

    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitchDegrees) {
      this.height = height;
      this.pitch = pitchDegrees; // in degrees
    }

    public final double height;
    public final double pitch;
  }
}
