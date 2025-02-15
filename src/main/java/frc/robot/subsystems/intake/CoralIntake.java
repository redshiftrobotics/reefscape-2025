package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Deprecated
public class CoralIntake extends SubsystemBase {

  private CoralIntakeIO io;
  private boolean intakeSensor;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

  public void moveLeftMotor(double speed) {
    io.moveLeftMotor(speed);
  }

  public void moveRightMotor(double speed) {
    io.moveRightMotor(speed);
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
