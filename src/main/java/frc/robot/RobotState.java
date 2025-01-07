package frc.robot;

/** Singleton to store global state of robot, try to use sparingly */
public class RobotState {

  // --- Singleton Setup ---

  private static RobotState instance;

  private RobotState() {}

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }
}
