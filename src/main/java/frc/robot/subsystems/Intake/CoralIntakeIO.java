package frc.robot.subsystems.Intake;

public interface CoralIntakeIO {

  public void moveLeftMotor(double Speed);

  public void moveRightMotor(double Speed);

  public boolean checkSensor();

  public void stop();
}