package frc.robot.subsystems.elevator;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double currentHeight = 0;

  }

  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  public default void goToHeight(double targetHeight) {
  }

  public default void configurePID(double Kp, double Ki, double Kd) {
  }
  
  public default void stop() {
  }
}
