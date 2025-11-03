package frc.robot.commands.visionDemo.filters;

import edu.wpi.first.util.DoubleCircularBuffer;

/**
 * A class that implements a moving-window mean filter. Useful for reducing measurement noise by
 * averaging values over a sliding window.
 */
public class MeanFilter {
  private final DoubleCircularBuffer m_valueBuffer;
  private final int m_size;
  private double m_sum;

  /**
   * Creates a new MeanFilter.
   *
   * @param size The number of samples in the moving window.
   */
  public MeanFilter(int size) {
    // Circular buffer of values currently in the window
    m_valueBuffer = new DoubleCircularBuffer(size);
    // Size of rolling window
    m_size = size;
    // Running sum of values in the window
    m_sum = 0.0;
  }

  /**
   * Calculates the moving-window mean for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The mean of the moving window, updated to include the next value.
   */
  public double calculate(double next) {
    // If buffer is at max size, remove oldest value from sum
    if (m_valueBuffer.size() >= m_size) {
      m_sum -= m_valueBuffer.removeLast();
    }

    // Add next value to circular buffer and sum
    m_valueBuffer.addFirst(next);
    m_sum += next;

    // Return average
    return m_sum / m_valueBuffer.size();
  }

  /**
   * Returns the last value calculated by the MeanFilter.
   *
   * @return The last value.
   */
  public double lastValue() {
    return m_valueBuffer.getFirst();
  }

  /** Resets the filter, clearing the window of all elements. */
  public void reset() {
    m_valueBuffer.clear();
    m_sum = 0.0;
  }
}
