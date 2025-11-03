package frc.robot.commands.visionDemo.filters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;

/** A class that implements a time-based moving-window mean filter for poses. */
public class MeanPoseFilterTimeBased {
  private static class TimestampedPose {
    public final Pose2d pose;
    public final double timestamp;

    public TimestampedPose(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }
  }

  private final LinkedList<TimestampedPose> m_valueBuffer;
  private final double m_windowDuration;

  /**
   * Creates a new MeanPoseFilterTimeBased.
   *
   * @param windowDuration The duration of the time window in seconds.
   */
  public MeanPoseFilterTimeBased(double windowDuration) {
    m_valueBuffer = new LinkedList<>();
    m_windowDuration = windowDuration;
  }

  /**
   * Calculates the moving-window mean for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The mean of the moving window, updated to include the next value.
   */
  public Pose2d calculate(Pose2d next) {
    return calculate(next, Timer.getFPGATimestamp());
  }

  /**
   * Calculates the moving-window mean for the next value of the input stream.
   *
   * @param next The next input value.
   * @param timestamp The timestamp of this measurement in seconds.
   * @return The mean of the moving window, updated to include the next value.
   */
  public Pose2d calculate(Pose2d next, double timestamp) {
    // Add new value to the front of the buffer
    m_valueBuffer.addFirst(new TimestampedPose(next, timestamp));

    // Remove old values outside the time window
    double cutoffTime = timestamp - m_windowDuration;
    while (!m_valueBuffer.isEmpty() && m_valueBuffer.getLast().timestamp < cutoffTime) {
      m_valueBuffer.removeLast();
    }

    // Calculate mean using vector averaging
    return calculateMean();
  }

  /**
   * Calculates the mean pose using vector averaging for rotation. This properly handles wrap-around
   * by converting angles to unit vectors, averaging them, and converting back.
   */
  public Pose2d calculateMean() {
    int bufferSize = m_valueBuffer.size();

    if (bufferSize == 0) {
      return null;
    }

    // Sum of x and y components of unit vectors (for rotation)
    double sumX = 0.0;
    double sumY = 0.0;
    Translation2d sumTranslation = new Translation2d(0, 0);

    for (TimestampedPose tp : m_valueBuffer) {
      Rotation2d angle = tp.pose.getRotation();
      sumX += angle.getCos();
      sumY += angle.getSin();
      sumTranslation = sumTranslation.plus(tp.pose.getTranslation());
    }

    // Average the components
    double avgX = sumX / bufferSize;
    double avgY = sumY / bufferSize;
    Translation2d avgTranslation = sumTranslation.div(bufferSize);

    // Convert back to angle using atan2
    return new Pose2d(avgTranslation, new Rotation2d(avgX, avgY));
  }

  /**
   * Returns the last value added to the MeanPoseFilter.
   *
   * @return The last value.
   */
  public Pose2d lastValue() {
    return m_valueBuffer.isEmpty() ? null : m_valueBuffer.getFirst().pose;
  }

  /** Resets the filter, clearing the window of all elements. */
  public void reset() {
    m_valueBuffer.clear();
  }

  /**
   * Returns the number of poses currently in the filter window.
   *
   * @return The number of poses in the buffer.
   */
  public int size() {
    return m_valueBuffer.size();
  }
}
