package frc.robot.commands.visionDemo.filters;

import edu.wpi.first.math.geometry.Rotation2d;

public class ComboAngleFilter {
  private final MedianAngleFilter medianFilter;
  private final MeanAngleFilter meanFilter;

  public ComboAngleFilter(int medianSize, int meanSize) {
    medianFilter = new MedianAngleFilter(medianSize);
    meanFilter = new MeanAngleFilter(meanSize);
  }

  public Rotation2d calculate(Rotation2d rawValue) {
    Rotation2d withoutOutliers = medianFilter.calculate(rawValue);
    Rotation2d mean = meanFilter.calculate(withoutOutliers);
    // System.out.printf(
    //     "ComboAngleFilter: raw=%+.2f median=%+.2f mean=%+.2f%n",
    //     rawValue.getDegrees(), withoutOutliers.getDegrees(), mean.getDegrees());
    return mean;
  }

  public void reset() {
    medianFilter.reset();
    meanFilter.reset();
  }
}
