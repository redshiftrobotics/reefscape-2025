package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  // no bindings
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
  }

  public void stop() {
    io.stop();
  }
}