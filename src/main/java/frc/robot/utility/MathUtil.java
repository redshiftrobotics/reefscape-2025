package frc.robot.utility;

public class MathUtil {
  
  /**
   * @author bforcum
   */
  public static double clamp(double a, double min, double max) {
    return Math.min(Math.max(a, min), max);
  }
  
  /**
   * @author bforcum
   */
  public static float clamp(float a, float min, float max) {
    return Math.min(Math.max(a, min), max);
  }
  
  /**
   * @author bforcum
   */
  public static double deadzone(double a, double range) {
    return (Math.abs(a) < range) ? 0 : a;
  }
  
  /**
   * @author bforcum
   */
  public static float deadzone(float a, float range) {
    return (Math.abs(a) < range) ? 0 : a;
  }
  

}
