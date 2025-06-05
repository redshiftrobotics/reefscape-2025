package frc.robot.subsystems.visualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utility.VirtualSubsystem;
import java.util.Arrays;
import java.util.List;
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
public class ObjectVisualizer extends VirtualSubsystem {

  private final String name;

  private final Supplier<Pose2d> baseSupplier;
  private final Supplier<Transform3d> mechanismOffsetSupplier;

  private boolean isHolding = false;

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

  public void hold() {
    setHolding(true);
  }

  public void clearHeldItem() {
    setHolding(false);
  }

  public boolean isHolding() {
    return isHolding;
  }

  /**
   * Place the item currently held by the subsystem at its current position.
   *
   * @return true if the item was successfully placed, false if not holding an item.
   */
  public boolean placeHeldItem() {
    if (isHolding) {
      addPlacedItem(getHeldItemPosition());
      clearHeldItem();
      return true;
    }
    return false;
  }

  /**
   * Place an item at the nearest position from a list of positions.
   *
   * @param targets a list of positions where the item can be placed.
   * @return true if the item was successfully placed, false if not holding an item.
   */
  public boolean placeItemOnNearest(List<Pose3d> targets) {
    return placeItemOnNearest(targets, Double.POSITIVE_INFINITY);
  }

  /**
   * Place an item at the nearest position from a list of positions.
   *
   * @param targets a list of positions where the item can be placed.
   * @param maxDistance max distance a position can be to place in meters.
   * @return true if the item was successfully placed, false if not holding an item.
   */
  public boolean placeItemOnNearest(List<Pose3d> targets, double maxDistance) {
    if (targets == null || targets.isEmpty()) {
      return false;
    }

    if (isHolding) {
      Pose3d position = getHeldItemPosition();
      Pose3d nearestPosition =
          targets.stream()
              .min(
                  (p1, p2) ->
                      Double.compare(
                          p1.getTranslation().getDistance(position.getTranslation()),
                          p2.getTranslation().getDistance(position.getTranslation())))
              .filter(p -> p.getTranslation().getDistance(position.getTranslation()) < maxDistance)
              .orElse(null);

      if (nearestPosition != null) {
        addPlacedItem(nearestPosition);
      }
      
      setHolding(false);
      return true;
    }
    return false;
  }

  private void addPlacedItem(Pose3d target) {
    placedItems = Arrays.copyOf(placedItems, placedItems.length + 1);
    placedItems[placedItems.length - 1] = target;
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
          "ObjectVisualizer/" + name + "/HeldCoral", new Pose3d[] {getHeldItemPosition()});
    } else {
      Logger.recordOutput("ObjectVisualizer/" + name + "/HasCoral", false);
      Logger.recordOutput("ObjectVisualizer/" + name + "/HeldCoral", Pose3d.kZero);
    }

    Logger.recordOutput("ObjectVisualizer/" + name + "/PlacedCoral", placedItems);
  }
}
