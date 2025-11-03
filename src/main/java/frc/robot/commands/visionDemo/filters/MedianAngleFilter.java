package frc.robot.commands.visionDemo.filters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.CircularBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * A class that implements a moving-window median filter for angles. Useful for rejecting outliers
 * in angular measurements while properly handling angle wrapping.
 */
public class MedianAngleFilter {
  private final CircularBuffer<Rotation2d> m_valueBuffer;
  private final int m_size;

  /**
   * Creates a new MedianAngleFilter.
   *
   * @param size The number of samples in the moving window.
   */
  public MedianAngleFilter(int size) {
    m_valueBuffer = new CircularBuffer<>(size);
    m_size = size;
  }

  /**
   * Calculates the moving-window median for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The median of the moving window, updated to include the next value.
   */
  public Rotation2d calculate(Rotation2d next) {
    // If buffer is at max size, remove oldest value
    if (m_valueBuffer.size() >= m_size) {
      m_valueBuffer.removeLast();
    }

    // Add next value to circular buffer
    m_valueBuffer.addFirst(next);

    // Return median
    return calculateMedian();
  }

  /**
   * Calculates the median angle from the current buffer using the "smallest arc" method. This
   * method finds the reference angle that minimizes the sum of angular distances, then computes the
   * median relative to that reference.
   */
  private Rotation2d calculateMedian() {
    int bufferSize = m_valueBuffer.size();
    if (bufferSize == 0) {
      return Rotation2d.kZero;
    }
    if (bufferSize == 1) {
      return m_valueBuffer.getFirst();
    }

    // Use the first angle as reference point
    Rotation2d reference = m_valueBuffer.getFirst();

    // Convert all angles to offsets from the reference
    // The offsets will be in range [-pi, pi]
    List<Double> offsets = new ArrayList<>(bufferSize);
    for (int i = 0; i < bufferSize; i++) {
      Rotation2d angle = m_valueBuffer.get(i);
      double offset = angle.minus(reference).getRadians();
      offsets.add(offset);
    }

    // Sort the offsets
    offsets.sort(Comparator.naturalOrder());

    // Find median offset
    double medianOffset;
    if (bufferSize % 2 == 1) {
      medianOffset = offsets.get(bufferSize / 2);
    } else {
      // For even number of elements, average the two middle elements
      // Need to be careful here if the two middle values are on opposite sides
      double lower = offsets.get(bufferSize / 2 - 1);
      double upper = offsets.get(bufferSize / 2);

      // If the gap between them is > pi, we're wrapping around
      if (upper - lower > Math.PI) {
        // Average by wrapping around the other way
        medianOffset = lower + (upper - lower - 2 * Math.PI) / 2.0;
      } else {
        medianOffset = (lower + upper) / 2.0;
      }
    }

    // Convert back to absolute angle
    return reference.plus(new Rotation2d(medianOffset));
  }

  /**
   * Returns the last value added to the MedianAngleFilter.
   *
   * @return The last value.
   */
  public Rotation2d lastValue() {
    return m_valueBuffer.getFirst();
  }

  /** Resets the filter, clearing the window of all elements. */
  public void reset() {
    m_valueBuffer.clear();
  }
}
