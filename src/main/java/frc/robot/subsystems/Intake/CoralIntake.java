package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  // !! SENSORS NOT IMPLMENTED !! \\

  private CoralIntakeIO io;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

    public void MoveLeftMotor(double Speed) {
    io.MoveLeftMotor(Speed);
  }

  public void MoveRightMotor(double Speed) {
    io.MoveRightMotor(Speed);
  }
}