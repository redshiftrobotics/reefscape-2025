package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {

    boolean motorConnected = false;

    double positionRotations = 0.0;
    double velocityRPM = 0.0;

    double[] appliedVolts = new double[] {0.0};
    double[] supplyCurrentAmps = new double[] {0.0};
  }

  public default void updateInputs(HangIOInputs inputs) {}

  public default void runPosition(double positionRotations) {}

  public default void runOpenLoop(double output) {}

  public default void runVolts(double volts) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setBrakeMode(boolean enable) {}

  public default void stop() {}
}
