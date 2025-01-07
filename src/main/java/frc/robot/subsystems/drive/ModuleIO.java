package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for all swerve module hardware */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    boolean driveMotorConnected = false;
    double drivePositionRad = 0.0;
    double driveVelocityRadPerSec = 0.0;
    double driveAppliedVolts = 0.0;
    double driveSupplyCurrentAmps = 0.0;
    double driveTorqueCurrentAmps = 0.0;

    boolean turnMotorConnected = false;
    Rotation2d turnAbsolutePosition = new Rotation2d();
    Rotation2d turnPosition = new Rotation2d();
    double turnVelocityRadPerSec = 0.0;
    double turnAppliedVolts = 0.0;
    double turnSupplyCurrentAmps = 0.0;

    boolean turnAbsoluteEncoderConnected = false;

    double[] odometryTimestamps = new double[] {};
    double[] odometryDrivePositionsRad = new double[] {};
    Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Run to drive velocity setpoint with feedforward */
  public default void setDriveVelocity(double velocityRadsPerSec, double feedForward) {}

  /** Run to turn position setpoint */
  public default void setTurnPosition(double angleRads) {}

  /** Configure drive PID */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /** Configure turn PID */
  public default void setTurnPID(double kP, double kI, double kD) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Disable output to brake and turn motor */
  default void stop() {}
}
