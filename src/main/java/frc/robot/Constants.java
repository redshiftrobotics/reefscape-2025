package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

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

  private static RobotType robotType = RobotType.CRESCENDO_CHASSIS_2024;

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
      robotType = RobotType.COMP_BOT;
    }
    if (RobotBase.isSimulation() && robotType != RobotType.SIM_BOT) {
      wrongRobotTypeAlertSim.set(true);
      robotType = RobotType.SIM_BOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case COMP_BOT, T_SHIRT_CANNON_CHASSIS, CRESCENDO_CHASSIS_2024 -> RobotBase.isReal()
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
    COMP_BOT
  }

  
  /**
	 * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
	 */
  public static class LEDPatterns {
    public static final double HOT_PINK = 0.57;
    public static final double DARK_RED = 0.59;
    public static final double RED = 0.61;
    public static final double RED_ORANGE = 0.63;
    public static final double ORANGE = 0.65;
    public static final double GOLD = 0.67;
    public static final double YELLOW = 0.69;
    public static final double LAWN_GREEN = 0.71;
    public static final double LIME_GREEN = 0.73;
    public static final double DARK_GREEN = 0.75;
    public static final double GREEN = 0.77;
    public static final double BLUE_GREEN = 0.79;
    public static final double AQUA = 0.81;
    public static final double SKY_BLUE = 0.83;
    public static final double DARK_BLUE = 0.85;
    public static final double BLUE = 0.87;
    public static final double BLUE_VIOLET = 0.89;
    public static final double VIOLET = 0.91;
    public static final double WHITE = 0.93;
    public static final double GRAY = 0.95;
    public static final double DARK_GRAY = 0.97;
    public static final double BLACK = 0.99;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIM_BOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }
}
