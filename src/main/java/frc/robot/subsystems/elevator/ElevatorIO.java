package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double currentHeight = 0;
  }

  public default void setGoalHeight(double targetHeight) {}

  public default void setVoltage(double volts) {}

  // public default double getHeight(){}

  public default void updateMotors() {}

  public default void configurePID(
      double maxVelocity,
      double maxAcceleration,
      double Kp,
      double Ki,
      double Kd,
      double Ks,
      double Kg,
      double Kv) {}

  public default void stop() {}
}
