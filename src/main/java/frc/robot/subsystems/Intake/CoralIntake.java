package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  private CoralIntakeIO io;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }
}