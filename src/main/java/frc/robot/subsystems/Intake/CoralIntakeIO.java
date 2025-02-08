package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeIO extends SubsystemBase {

  private SparkMax LeftMotor;
  private SparkMax RightMotor;

  public CoralIntakeIO(SparkMax LeftMotor, SparkMax rightMax) {

    this.LeftMotor = LeftMotor;
    this.RightMotor = RightMotor;
  }

  public void MoveLeftMotor(double Speed) {
    LeftMotor.set(MathUtil.clamp(Speed, -0.5, 0.5));
  }

  public void MoveRightMotor(double Speed) {
    RightMotor.set(MathUtil.clamp(Speed, -0.5, 0.5));
  }
}