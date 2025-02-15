package frc.robot.subsystems.Intake;

public interface CoralIntakeIO {

  public void moveLeftMotor(double Speed);

  public void moveRightMotor(double Speed);

  //sensor
  public boolean checkSensor();
  
  //stops motor
  public void stop();
}