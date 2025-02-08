package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController pid;

  public Wrist(int motorID, double p, double i, double d) {
    motor = new SparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(encoder.getPosition());
  }

  public void setRotation(Rotation2d rot) {
    pid.setReference(rot.getRotations(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Wrist Current Rotation", getRotation().getDegrees());
  }
}
