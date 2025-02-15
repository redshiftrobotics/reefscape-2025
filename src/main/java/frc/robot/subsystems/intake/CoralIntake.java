package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  private CoralIntakeIO io;
  private boolean intakeSensor;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

  public void moveLeftMotor(double Speed) {
    io.moveLeftMotor(Speed);
  }

  public void moveRightMotor(double Speed) {
    io.moveRightMotor(Speed);
  }

  @Override
  public void periodic() {
    intakeSensor = io.checkSensor();
    SmartDashboard.putBoolean("boolean intake", intakeSensor);
  }

  public void stop() {
    io.stop();
  }
}
