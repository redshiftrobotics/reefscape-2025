package frc.robot.commands.visionDemo.filters;

import edu.wpi.first.math.filter.MedianFilter;

public class ComboFilter {
  private final MedianFilter medianFilter;
  private final MeanFilter meanFilter;

  public ComboFilter(int medianSize, int meanSize) {
    medianFilter = new MedianFilter(medianSize);
    meanFilter = new MeanFilter(meanSize);
  }

  public double calculate(double rawValue) {
    double withoutOutliers = medianFilter.calculate(rawValue);
    return meanFilter.calculate(withoutOutliers);
  }

  public void reset() {
    medianFilter.reset();
    meanFilter.reset();
  }
}
