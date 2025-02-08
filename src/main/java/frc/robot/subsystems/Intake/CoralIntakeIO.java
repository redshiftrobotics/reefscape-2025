package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeIO extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMax rightMotor;

  public CoralIntakeIO(SparkMax LeftMotor, SparkMax rightMax) {

    this.leftMotor = LeftMotor;
    this.rightMotor = rightMotor;
  }

  //motor stuff

  public void moveLeftMotor(double Speed) {
    leftMotor.set(MathUtil.clamp(Speed, -0.5, 0.5));
  }

  public void moveRightMotor(double Speed) {
    rightMotor.set(MathUtil.clamp(Speed, -0.5, 0.5));
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  //TODO add the sensor stuff
}