package frc.robot.subsystems.intake;

public interface IntakeIo {
  double speed;


  public class IntakeSystem {
    private int motorSpeed;

    public IntakeSystem() {
      this.motorSpeed = 0;
    }


    public void setMotorSpeed(int speed) {
      if (speed >= -100 && speed <= 100) {
        this.motorSpeed = speed;
      }
    }

public void setInputs(double speed)

public void updateInputs(inputs)

  }
}