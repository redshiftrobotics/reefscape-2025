package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

// TODO Replace dummy values!
public class WristConstants {
  public record WristConfig(int motorId, boolean motorInverted) {}

  public static final WristConfig WRIST_CONFIG =
      switch (Constants.getRobot()) {
        default -> new WristConfig(0, false);
      };

  public static class WristPositions {
    public static final Rotation2d ROBOT_START = Rotation2d.fromDegrees(0);
    public static final Rotation2d CORAL_INTAKE = Rotation2d.fromDegrees(0);
    public static final Rotation2d CORAL_DEPOSIT = Rotation2d.fromDegrees(0);
  }

  public record PID(double p, double i, double d) {}

  public record FeedForward(double s, double g, double v, double a) {}

  public static final PID PID_CONFIG =
      switch (Constants.getRobot()) {
        default -> new PID(0, 0, 0);
      };

  public static final FeedForward FEED_FORWARD_CONFIG =
      switch (Constants.getRobot()) {
        default -> new FeedForward(0, 0.58, 0.1, 0);
      };

  public static final double GEAR_RATIO = 1.0f;
  public static final double ARM_LENGTH = Units.inchesToMeters(14);

  // This is how many radians off from the target the arm is allowed to be for the move command to
  // consider it done
  public static final double WRIST_MOVE_DONE_THRESHOLD = 0.01f;
}
