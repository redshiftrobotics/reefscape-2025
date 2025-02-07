package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.records.ElevatorFeedForwardConstants;
import frc.robot.utility.records.PIDConstants;

/** Constants for the Template subsystem. Units in meters. */
public class ElevatorConstants {

  public static final int drumMotorCanID =
      switch (Constants.getRobot()) {
        default -> 0;
      };

  public static final double carriageHeight = Units.inchesToMeters(4);
  public static final double elevatorHeight = Units.inchesToMeters(40.8);

  public static final double carriageMaxHeight = elevatorHeight - carriageHeight;

  public static final double carriageMassKg = 2.26796; // 5 lbs

  public static final double drumRadius = Units.inchesToMeters(1.0);
  public static final double drumGearReduction = 1.0;

  public static final double maxCarriageVelocity = 3.0;
  public static final double maxCarriageAcceleration = 3.5;

  public static final double carriagePositionTolerance = Units.inchesToMeters(0.3);

  public static final PIDConstants pid =
      switch (Constants.getRobot()) {
        case SIM_BOT -> new PIDConstants(0.004, 0.0, 0.0);
        default -> new PIDConstants(0.0, 0.0, 0.0);
      };

  public static final ElevatorFeedForwardConstants feedForward =
      switch (Constants.getRobot()) {
        case SIM_BOT -> new ElevatorFeedForwardConstants(0.001, 2.6056, 0.76, 10);
        default -> new ElevatorFeedForwardConstants(0.0, 0.0, 0.0, 0.0);
      };
}
