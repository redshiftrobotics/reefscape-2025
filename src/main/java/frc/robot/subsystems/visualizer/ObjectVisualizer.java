package frc.robot.subsystems.visualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem for visualizing objects in a 3D space, such as corals or other items.
 *
 * <p>This subsystem allows for the placement and visualization of items in a 3D environment,
 * including the ability to hold an item and place it at a specified position.
 *
 * <p>It can hold one item at a time and can place it at the current position of the robot. The
 * items held position is calculated based on the robot's base position and a mechanism offset.
 */
public class ObjectVisualizer extends SubsystemBase {

  private final String name;

  private final Supplier<Pose2d> baseSupplier;
  private final Supplier<Transform3d> mechanismOffsetSupplier;

  private boolean isHolding = false;
  private Transform3d interpolationTransform = Transform3d.kZero;

  private Pose3d[] placedItems;

  public ObjectVisualizer(
      String name, Supplier<Pose2d> baseSupplier, Supplier<Transform3d> mechanismOffsetSupplier) {
    super();
    this.name = name;
    this.baseSupplier = baseSupplier;
    this.mechanismOffsetSupplier = mechanismOffsetSupplier;

    placedItems = new Pose3d[] {};
  }

  /**
   * Sets whether the subsystem is currently holding an item, and whether it should be displayed as
   * such.
   *
   * @param isHolding true if the subsystem is holding an item, false otherwise.
   */
  public void setHolding(boolean isHolding) {
    this.isHolding = isHolding;
  }

  /** Set holding to true */
  public void hold() {
    setHolding(true);
  }

  /**
   * Set holding to false
   *
   * @return whether an item was removed
   */
  public boolean ejectHeldItem() {
    boolean wasHolding = isHolding;
    setHolding(false);
    return wasHolding;
  }

  /** Get whether an item is being held */
  public boolean isHolding() {
    return isHolding;
  }

  /**
   * Add a placed item at the held items current position and remove the held item
   *
   * @param target place to add the held item
   * @return whether there was an item to move
   */
  public boolean placeHeldItem(Pose3d target) {
    if (isHolding) {
      addPlacedItem(target);
      setHolding(false);
      return true;
    }
    return false;
  }

  public Command placeHeldItemWithInterpolation(Pose3d target, double seconds) {
    if (seconds == 0) {
      return runOnce(() -> placeHeldItem(target));
    }

    Timer timer = new Timer();
    Pose3d startPosition = getHeldItemPosition();

    return runEnd(
            () -> {
              interpolationTransform =
                  startPosition.interpolate(target, timer.get() / seconds).minus(startPosition);
            },
            () -> placeHeldItem(target))
        .until(() -> timer.hasElapsed(seconds))
        .beforeStarting(timer::restart)
        .finallyDo(() -> interpolationTransform = Transform3d.kZero)
        .onlyIf(this::isHolding);
  }

  /**
   * Place held item at the nearest position from a list of positions.
   *
   * @param targets a list of positions where the item can be placed.
   * @param maxDistance max distance a position can be to place in meters.
   * @return true if the item was successfully placed, false if not holding an item.
   */
  public boolean placeHeldItemOnNearest(List<Pose3d> targets, double maxDistance) {
    Optional<Pose3d> nearestTarget = findHeldItemPlacementTarget(targets, maxDistance);

    if (nearestTarget.isPresent()) {
      return placeHeldItem(nearestTarget.get());
    }
    return ejectHeldItem();
  }

  public Command placeHeldItemOnNearestWithInterpolation(
      List<Pose3d> targets, double maxDistance, double seconds) {
    return defer(
        () -> {
          Optional<Pose3d> target = findHeldItemPlacementTarget(targets, maxDistance);
          return placeHeldItemWithInterpolation(target.get(), seconds).onlyIf(target::isPresent);
        });
  }

  private Optional<Pose3d> findHeldItemPlacementTarget(
      Collection<Pose3d> targets, double maxDistance) {
    Pose3d heldItemPose = getHeldItemPosition();
    Pose3d pose = nearest(heldItemPose, targets);
    if (pose.getTranslation().getDistance(heldItemPose.getTranslation()) > maxDistance) {
      return Optional.empty();
    }
    return Optional.of(pose);
  }

  public Pose3d getHeldItemPosition() {
    Pose2d basePosition = baseSupplier.get();
    Transform3d mechanismOffset = mechanismOffsetSupplier.get();

    return new Pose3d(
            new Translation3d(basePosition.getTranslation()),
            new Rotation3d(basePosition.getRotation()))
        .plus(mechanismOffset);
  }

  public void clearPlacedItems() {
    placedItems = new Pose3d[] {};
  }

  @Override
  public void periodic() {
    if (isHolding) {
      Logger.recordOutput("ObjectVisualizer/" + name + "/HasCoral", true);
      Logger.recordOutput(
          "ObjectVisualizer/" + name + "/HeldCoral",
          new Pose3d[] {getHeldItemPosition().plus(interpolationTransform)});
    } else {
      Logger.recordOutput("ObjectVisualizer/" + name + "/HasCoral", false);
      Logger.recordOutput("ObjectVisualizer/" + name + "/HeldCoral", Pose3d.kZero);
    }

    Logger.recordOutput("ObjectVisualizer/" + name + "/PlacedCoral", placedItems);
  }

  private void addPlacedItem(Pose3d target) {
    placedItems = Arrays.copyOf(placedItems, placedItems.length + 1);
    placedItems[placedItems.length - 1] = target;
  }

  private static Pose3d nearest(Pose3d target, Collection<Pose3d> candidates) {
    return Collections.min(
        candidates,
        Comparator.comparing(
                (Pose3d other) -> target.getTranslation().getDistance(other.getTranslation()))
            .thenComparing(
                (Pose3d other) -> target.getRotation().minus(other.getRotation()).getAngle()));
  }
}
