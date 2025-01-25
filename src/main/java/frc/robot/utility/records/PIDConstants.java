package frc.robot.utility.records;

public record PIDConstants(double Kp, double Ki, double Kd) {
  public com.pathplanner.lib.config.PIDConstants toPathPlannerPIDConstants() {
    return new com.pathplanner.lib.config.PIDConstants(Kp, Ki, Kd);
  }
}
