package frc.robot.subsystems.hang;

import edu.wpi.first.math.geometry.Rotation2d;

public class HangIODummy implements HangIO {
  @Override
  public void updateInputs(HangIOInputs inputs) {}

  @Override
  public void setSetpoint(Rotation2d setpoint) {}

  @Override
  public double getSetpoint() {
    return 0;
  }

  @Override
  public double getPosition() {
    return 0;
  }
}
