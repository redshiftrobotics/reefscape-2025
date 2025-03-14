package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.ElevatorFeedForwardConstants;
import frc.robot.utility.records.PIDConstants;

/** Constants for the Template subsystem. Units in meters. */
public class ElevatorConstants {

  public static record ElevatorConfig(int leaderCanId, int followerCanId, boolean inverted) {}

  public static final ElevatorConfig ELEVATOR_CONFIG =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new ElevatorConfig(3, 17, true); // left is leader, right is follower
        default -> new ElevatorConfig(0, 1, false);
      };

  public static final int currentLimit = 40;
  public static final double gearReduction = 9.0;

  public static final double elevatorHeight = 1.445 + Units.inchesToMeters(8.5);
  public static final double carriageHeight = Units.inchesToMeters(8.5);

  public static final double carriageMaxHeight = elevatorHeight - carriageHeight;

  public static final double carriageMassKg = 2.26796; // 5 lbs

  public static final double drumRadius = Units.inchesToMeters(2.0);

  public static final double maxCarriageVelocity = 3.0;
  public static final double maxCarriageAcceleration = 3.5;

  public static final double carriagePositionTolerance = Units.inchesToMeters(0.2);

  public static final PIDConstants pid =
      switch (Constants.getRobot()) {
        default -> new PIDConstants(1.0, 0.0, 0.0);
      };

  public static final ElevatorFeedForwardConstants feedForward =
      switch (Constants.getRobot()) {
        case SIM_BOT -> new ElevatorFeedForwardConstants(0.001, 0.26056, 0.76, 10);
        case COMP_BOT_2025 -> new ElevatorFeedForwardConstants(0.031600659999, 0, 0, 0);
        default -> new ElevatorFeedForwardConstants(0.0, 0.0, 0.0, 0.0);
      };
}
