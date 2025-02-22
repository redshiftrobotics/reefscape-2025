package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeSubsystem extends Subsystem {
  public void setLeftMotor(double speed);
  public void setRightMotor(double speed);
  public void setMotors(double speed);
  public void stopMotors();
}
