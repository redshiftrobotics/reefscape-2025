package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

  public static final boolean TUNING_MODE = false;
  public static final boolean ON_BLOCKS_TEST_MODE = false;

  private static RobotType robotType;

  public static final Alert wrongRobotTypeAlertReal =
      new Alert(
          "Invalid robot selected, using competition robot as default.", Alert.AlertType.kWarning);

  private static final Alert wrongRobotTypeAlertSim =
      new Alert(
          "Invalid robot selected for simulation robot, using simulation robot as default.",
          AlertType.kError);

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
      wrongRobotTypeAlertReal.set(true);
      robotType = RobotType.WOOD_BOT_TWO_2025;
    }
    if (RobotBase.isSimulation() && robotType != RobotType.SIM_BOT) {
      wrongRobotTypeAlertSim.set(true);
      robotType = RobotType.SIM_BOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case WOOD_BOT_TWO_2025, T_SHIRT_CANNON_CHASSIS, CRESCENDO_CHASSIS_2024 -> RobotBase.isReal()
          ? Mode.REAL
          : Mode.REPLAY;
      case SIM_BOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIM_BOT,
    T_SHIRT_CANNON_CHASSIS,
    CRESCENDO_CHASSIS_2024,
    WOOD_BOT_TWO_2025
  }

  static {
    if (RobotBase.isReal()) {
      switch (RobotController.getSerialNumber()) {
        default:
          robotType = RobotType.WOOD_BOT_TWO_2025;
          break;
      }
    } else if (RobotBase.isSimulation()) {
      robotType = RobotType.SIM_BOT;
    }
  }
}
