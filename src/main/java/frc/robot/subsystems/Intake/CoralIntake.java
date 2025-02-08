package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    
  private SparkMax LeftMotor;
  private SparkMax RightMotor;

  public CoralIntake(SparkMax LeftMotor, SparkMax rightMax) {
    
    this.LeftMotor = LeftMotor;
    this.RightMotor = RightMotor;
  
    
  }
}