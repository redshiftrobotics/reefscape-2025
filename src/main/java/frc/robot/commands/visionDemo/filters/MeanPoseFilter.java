package frc.robot.commands.visionDemo.filters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.CircularBuffer;

/** A class that implements a moving-window mean filter for poses. */
public class MeanPoseFilter {
  private final CircularBuffer<Pose2d> m_valueBuffer;
  private final int m_size;

  /**
   * Creates a new MeanAngleFilter.
   *
   * @param size The number of samples in the moving window.
   */
  public MeanPoseFilter(int size) {
    // Circular buffer of values currently in the window (stored as radians)
    m_valueBuffer = new CircularBuffer<>(size);
    // Size of rolling window
    m_size = size;
  }

  /**
   * Calculates the moving-window mean for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The mean of the moving window, updated to include the next value.
   */
  public Pose2d calculate(Pose2d next) {
    // If buffer is at max size, remove oldest value
    if (m_valueBuffer.size() >= m_size) {
      m_valueBuffer.removeLast();
    }

    // Add next value to circular buffer
    m_valueBuffer.addFirst(next);

    // Calculate mean using vector averaging
    return calculateMean();
  }

  /**
   * Calculates the mean angle using vector averaging. This properly handles wrap-around by
   * converting angles to unit vectors, averaging them, and converting back.
   */
  public Pose2d calculateMean() {
    int bufferSize = m_valueBuffer.size();

    if (bufferSize == 0) {
      return null;
    }

    // Sum of x and y components of unit vectors
    double sumX = 0.0;
    double sumY = 0.0;
    Translation2d sumTranslation = new Translation2d(0, 0);

    for (int i = 0; i < bufferSize; i++) {
      Rotation2d angle = m_valueBuffer.get(i).getRotation();
      sumX += angle.getCos();
      sumY += angle.getSin();
      sumTranslation = sumTranslation.plus(m_valueBuffer.get(i).getTranslation());
    }

    // Average the components
    double avgX = sumX / bufferSize;
    double avgY = sumY / bufferSize;
    Translation2d avgTranslation = sumTranslation.div(bufferSize);

    // Convert back to angle using atan2
    return new Pose2d(avgTranslation, new Rotation2d(avgX, avgY));
  }

  /**
   * Returns the last value added to the MeanAngleFilter.
   *
   * @return The last value.
   */
  public Pose2d lastValue() {
    return m_valueBuffer.getFirst();
  }

  /** Resets the filter, clearing the window of all elements. */
  public void reset() {
    m_valueBuffer.clear();
  }
}
