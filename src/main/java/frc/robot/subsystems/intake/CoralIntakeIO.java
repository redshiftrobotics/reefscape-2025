package frc.robot.subsystems.intake;

public interface CoralIntakeIO {

  public void moveLeftMotor(double speed);

  public void moveRightMotor(double speed);

  // sensor
  public boolean checkSensor();

  // stops motor
  public void stop();
}
