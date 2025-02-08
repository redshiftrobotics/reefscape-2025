package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  // !! SENSORS NOT IMPLMENTED !! \\
  // no bindings
  private CoralIntakeIO io;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

  public void moveLeftMotor(double Speed) {
    io.moveLeftMotor(Speed);
  }

  public void moveRightMotor(double Speed) {
    io.moveRightMotor(Speed);
  }

  public void stop() {
    io.stop();
  }
}
