package frc.robot.subsystems.superstructure.intake;

/** Simulation implementation of the TemplateIO. */
public class IntakeIOSim implements IntakeIO {

  private double leftSpeed = 0.0;
  private double rightSpeed = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speedLeft = leftSpeed;
    inputs.speedRight = rightSpeed;

    inputs.appliedVolts = new double[] {leftSpeed * 12.0, rightSpeed * 12.0};
    inputs.supplyCurrentAmps = new double[] {0.0, 0.0};

    inputs.motorConnected = true;
  }

  @Override
  public void setRightMotor(double speed) {
    rightSpeed = speed;
  }

  @Override
  public void setLeftMotor(double speed) {
    leftSpeed = speed;
  }

  @Override
  public void stopMotors() {
    setLeftMotor(0);
    setRightMotor(0);
  }
}
