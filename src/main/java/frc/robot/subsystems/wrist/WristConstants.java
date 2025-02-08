package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

// EVERYTHING IN THIS CLASS IS DUMMY VALUES!!!!!!! DO NOT RELY ON THEM YET!!!!!!!!!
public class WristConstants {
  public record WristConfig(int motorId, boolean motorInverted) {}

  public static final WristConfig WRIST_CONFIG =
      switch (Constants.getRobot()) {
        default -> new WristConfig(0, false);
      };

  public static class WristPositions {
    public static final Rotation2d ROBOT_START = Rotation2d.fromRotations(0);
    public static final Rotation2d CORAL_INTAKE = Rotation2d.fromRotations(0);
    public static final Rotation2d CORAL_DEPOSIT = Rotation2d.fromRotations(0);
  }

  public record PID(double p, double i, double d) {}

  public record FeedForward(double s, double g, double v, double a) {}

  public static final PID PID_CONFIG =
      switch (Constants.getRobot()) {
        default -> new PID(0, 0, 0);
      };

  public static final FeedForward FEED_FORWARD_CONFIG =
      switch (Constants.getRobot()) {
        default -> new FeedForward(0, 0, 0, 0);
      };

  public static final float GEAR_RATIO = 1.0f;
}
